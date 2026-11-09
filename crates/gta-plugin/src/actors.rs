//! The actor list and the standing line-of-sight queries (D25), built to the
//! S6 design rules: positions from `GET_ENTITY_COORDS`, no per-vehicle natives
//! that aren't needed every frame, one ray per actor (from the nearest sensor
//! that covers it), at most 8 async rays in flight (GTA allows about 10), and a
//! synchronous top-up for actors that are overdue.

use std::collections::HashMap;

use adapter_api::{Actor, ActorKind, ActorList, MAX_ACTORS, SensorLayout};
use geom::{EgoPose, V2, wrap_pi};
use shv_sys::{Vector3, invoke};

use crate::n;

const MAX_IN_FLIGHT: usize = 8;
const SYNC_MAX: usize = 10;
const LIST_RANGE_M: f32 = 170.0;
/// Map + vehicles + objects: what blocks a radar's view.
const LOS_FLAGS: i32 = 1 | 2 | 16;
const MAX_VEHICLES: usize = 1024;

#[derive(Clone, Copy, Default)]
struct Los {
    mask: u8,
    checked: u32,
}

#[derive(Default, Clone, Copy, Debug)]
pub struct LosStats {
    pub started: u32,
    pub sync: u32,
    pub failed: u32,
    pub done: u32,
    /// Worst line-of-sight age among actors in view (frames).
    pub age_max: u32,
    pub in_view: u32,
}

pub struct Actors {
    frame: u32,
    vbuf: Vec<i32>,
    los: HashMap<i32, Los>,
    /// (handle, actor, sensor bits it answers for, start frame)
    pending: Vec<(i32, i32, u8, u32)>,
    dims: HashMap<u32, (Vector3, Vector3)>,
}

/// Polls a shape test: (status: 0 none / 1 pending / 2 ready, hit, entity).
unsafe fn ray_result(handle: i32) -> (i32, bool, i32) {
    let mut hit = 0i32;
    let mut end = Vector3::default();
    let mut normal = Vector3::default();
    let mut entity = 0i32;
    let status: i32 = unsafe {
        invoke!(
            n::GET_SHAPE_TEST_RESULT,
            handle,
            &mut hit as *mut i32,
            &mut end as *mut Vector3,
            &mut normal as *mut Vector3,
            &mut entity as *mut i32,
        )
    };
    (status, hit != 0, entity)
}

/// Where a sensor at `mount` sees an actor's box (ego frame): its nearest
/// point, moved 0.3 m toward the centre so the ray ends inside the actor.
fn aim_point(c: V2, yaw: f32, length: f32, width: f32, mount: V2) -> V2 {
    let (s, co) = yaw.sin_cos();
    let d = mount - c;
    let (lx, ly) = (co * d.x + s * d.y, -s * d.x + co * d.y);
    let (cx, cy) = (lx.clamp(-0.5 * length, 0.5 * length), ly.clamp(-0.5 * width, 0.5 * width));
    let p = V2::new(c.x + co * cx - s * cy, c.y + s * cx + co * cy);
    let inward = c - p;
    let l = inward.len();
    if l > 0.3 { V2::new(p.x + 0.3 * inward.x / l, p.y + 0.3 * inward.y / l) } else { c }
}

pub fn model_dims(cache: &mut HashMap<u32, (Vector3, Vector3)>, model: u32) -> (Vector3, Vector3) {
    *cache.entry(model).or_insert_with(|| {
        let (mut lo, mut hi) = (Vector3::default(), Vector3::default());
        let () = unsafe { invoke!(n::GET_MODEL_DIMENSIONS, model, &mut lo as *mut Vector3, &mut hi as *mut Vector3) };
        (lo, hi)
    })
}

impl Actors {
    pub fn new() -> Self {
        Self { frame: 0, vbuf: vec![0; MAX_VEHICLES], los: HashMap::new(), pending: Vec::new(), dims: HashMap::new() }
    }

    pub fn reset(&mut self) {
        self.los.clear();
        self.pending.clear();
    }

    /// One frame: the nearest actors in the ego frame, with line-of-sight bits.
    pub fn step(&mut self, veh: i32, layout: &SensorLayout, out: &mut ActorList) -> LosStats {
        self.frame = self.frame.wrapping_add(1);
        let frame = self.frame;
        let mut st = LosStats::default();

        // 1. Results of earlier async rays.
        let mut still = Vec::with_capacity(self.pending.len());
        for &(h, id, bits, started) in &self.pending {
            let (status, hit, entity) = unsafe { ray_result(h) };
            match status {
                2 => {
                    st.done += 1;
                    let clear = !hit || entity == id;
                    self.los.insert(id, Los { mask: if clear { bits } else { 0 }, checked: frame });
                }
                1 if frame.wrapping_sub(started) < 30 => still.push((h, id, bits, started)),
                _ => {}
            }
        }
        self.pending = still;

        // 2. Ego pose (world: x east, y north, z up; GTA heading 0 = north, CCW).
        let ep: Vector3 = unsafe { invoke!(n::GET_ENTITY_COORDS, veh, true) };
        let ef: Vector3 = unsafe { invoke!(n::GET_ENTITY_FORWARD_VECTOR, veh) };
        let eh: f32 = unsafe { invoke!(n::GET_ENTITY_HEADING, veh) };
        let ego = EgoPose::new(V2::new(ep.x, ep.y), V2::new(ef.x, ef.y));

        // 3. Every vehicle, nearest first, within range.
        let n = unsafe { shv_sys::worldGetAllVehicles(self.vbuf.as_mut_ptr(), self.vbuf.len() as i32) }.max(0) as usize;
        let mut near: Vec<(f32, i32, Vector3)> = Vec::with_capacity(n.min(256));
        for &v in &self.vbuf[..n.min(MAX_VEHICLES)] {
            if v == veh {
                continue;
            }
            let p: Vector3 = unsafe { invoke!(n::GET_ENTITY_COORDS, v, true) };
            let d2 = (p.x - ep.x).powi(2) + (p.y - ep.y).powi(2);
            if d2 <= LIST_RANGE_M * LIST_RANGE_M {
                near.push((d2, v, p));
            }
        }
        near.sort_by(|a, b| a.0.total_cmp(&b.0));
        out.dropped = near.len().saturating_sub(MAX_ACTORS) as u32;
        near.truncate(MAX_ACTORS);

        // 4. Ego-frame state, sensor coverage and line-of-sight bits.
        let mounts = layout.as_slice();
        let mut due: Vec<(f32, usize, u8, usize, V2)> = Vec::new(); // (score, actor index, bits, mount, ray target)
        let ego_z = ep.z;
        out.count = near.len() as u32;
        for (i, &(_, v, p)) in near.iter().enumerate() {
            let model: u32 = unsafe { invoke!(n::GET_ENTITY_MODEL, v) };
            let (lo, hi) = model_dims(&mut self.dims, model);
            let f: Vector3 = unsafe { invoke!(n::GET_ENTITY_FORWARD_VECTOR, v) };
            let vel: Vector3 = unsafe { invoke!(n::GET_ENTITY_VELOCITY, v) };
            let rot: Vector3 = unsafe { invoke!(n::GET_ENTITY_ROTATION_VELOCITY, v) };
            let h: f32 = unsafe { invoke!(n::GET_ENTITY_HEADING, v) };
            // Box centre in the world: origin + the model's centre offset.
            let fl = (f.x * f.x + f.y * f.y).sqrt().max(1e-6);
            let (fx, fy) = (f.x / fl, f.y / fl);
            let (cx, cy) = ((lo.x + hi.x) * 0.5, (lo.y + hi.y) * 0.5);
            let centre = V2::new(p.x + fy * cx + fx * cy, p.y - fx * cx + fy * cy);
            let rel = ego.to_vehicle(centre);
            let vv = ego.rotate_to_vehicle(V2::new(vel.x, vel.y));
            let (yaw, length, width) = (wrap_pi((h - eh).to_radians()), hi.y - lo.y, hi.x - lo.x);

            // Which sensors cover it, and the nearest one. Wide-beam (corner)
            // sensors see it where its box is nearest them (a car alongside
            // is in a corner radar's beam even when its centre isn't); the
            // front radar uses the centre, as in Phase 1.
            let mut bits = 0u8;
            let mut best: Option<(f32, usize, V2)> = None;
            for (k, m) in mounts.iter().enumerate() {
                let target = if m.half_fov > 0.5 { aim_point(rel, yaw, length, width, V2::new(m.x, m.y)) } else { rel };
                let (dx, dy) = (target.x - m.x, target.y - m.y);
                let range = dx.hypot(dy);
                if range <= m.range && wrap_pi(dy.atan2(dx) - m.facing).abs() <= m.half_fov {
                    bits |= 1 << k;
                    if best.is_none_or(|b| range < b.0) {
                        best = Some((range, k, target));
                    }
                }
            }
            let los = self.los.get(&v).copied();
            let age = los.map(|l| frame.wrapping_sub(l.checked)).unwrap_or(u32::MAX);
            if bits != 0 {
                st.in_view += 1;
                st.age_max = st.age_max.max(age.min(u16::MAX as u32));
                if let Some((_, k, target)) = best
                    && !self.pending.iter().any(|q| q.1 == v)
                {
                    let refresh = mounts[k].refresh_frames.max(1.0);
                    let score = (age.min(10_000) as f32 + 1.0) / refresh;
                    if score >= 1.0 {
                        due.push((score, i, bits, k, target));
                    }
                }
            }
            out.actors[i] = Actor {
                id: v as u32,
                kind: ActorKind::Vehicle,
                los_mask: los.map(|l| l.mask & bits).unwrap_or(0),
                los_age: age.min(u16::MAX as u32) as u16,
                x: rel.x,
                y: rel.y,
                z: p.z + (lo.z + hi.z) * 0.5 - ego_z,
                vx: vv.x,
                vy: vv.y,
                yaw,
                yaw_rate: rot.z,
                length,
                width,
                height: hi.z - lo.z,
            };
        }

        // 5. Rays: the stalest first; async within the in-flight limit, then a
        //    synchronous top-up for actors already past their refresh period.
        due.sort_by(|a, b| b.0.total_cmp(&a.0));
        // GTA's own AI shares the async pool: when a start fails, the rest go
        // synchronous (within SYNC_MAX).
        let mut async_left = MAX_IN_FLIGHT.saturating_sub(self.pending.len());
        for &(score, i, bits, k, target) in &due {
            let m = mounts[k];
            let from = ego.to_world(V2::new(m.x, m.y));
            let a = out.actors[i];
            let to = ego.to_world(target);
            let (fz, tz) = (ego_z + m.z, ego_z + a.z);
            let id = near[i].1;
            if async_left > 0 {
                let h: i32 = unsafe { invoke!(n::START_SHAPE_TEST_LOS_PROBE, from.x, from.y, fz, to.x, to.y, tz, LOS_FLAGS, veh, 7i32) };
                if h != 0 {
                    st.started += 1;
                    async_left -= 1;
                    self.pending.push((h, id, bits, frame));
                    continue;
                }
                st.failed += 1;
                async_left = 0;
            }
            // Synchronous: overdue items only (or anything after an async failure).
            if st.sync as usize >= SYNC_MAX || (score < 1.5 && st.failed == 0) {
                break;
            }
            {
                let h: i32 = unsafe {
                    invoke!(n::START_EXPENSIVE_SYNCHRONOUS_SHAPE_TEST_LOS_PROBE, from.x, from.y, fz, to.x, to.y, tz, LOS_FLAGS, veh, 7i32)
                };
                let (status, hit, entity) = unsafe { ray_result(h) };
                if status == 2 {
                    st.sync += 1;
                    st.done += 1;
                    let clear = !hit || entity == id;
                    self.los.insert(id, Los { mask: if clear { bits } else { 0 }, checked: frame });
                    out.actors[i].los_mask = if clear { bits } else { 0 };
                    out.actors[i].los_age = 0;
                } else {
                    st.failed += 1;
                }
            }
        }

        if frame.is_multiple_of(300) {
            self.los.retain(|_, l| frame.wrapping_sub(l.checked) < 720);
        }
        st
    }
}
