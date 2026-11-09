//! The standing range fans (the ultrasonics' rays), answered with shape
//! tests to the S6 rules: only while the core asks (parking speeds), a few
//! synchronous rays per frame round-robin, so each ray is refreshed about
//! every `refresh_frames`.

use adapter_api::{FanLayout, FanResult, MAX_FAN_RAYS, MAX_FANS, RangeReport, fan_flags};
use geom::{EgoPose, V2};
use shv_sys::{Vector3, invoke};

use crate::n;

/// Map, vehicles, peds and objects: what an ultrasonic echo comes from.
const FAN_FLAGS: i32 = 1 | 2 | 4 | 16;
/// Synchronous rays per frame at most (about 8 µs each, S6).
const MAX_RAYS_PER_FRAME: usize = 10;

pub struct Fans {
    frame: u32,
    hits: [[f32; MAX_FAN_RAYS]; MAX_FANS],
    checked: [[u32; MAX_FAN_RAYS]; MAX_FANS],
    cursor: usize,
    /// Rays cast in the last frame.
    pub cast: u32,
}

impl Fans {
    pub fn new() -> Self {
        Self { frame: 0, hits: [[f32::NAN; MAX_FAN_RAYS]; MAX_FANS], checked: [[0; MAX_FAN_RAYS]; MAX_FANS], cursor: 0, cast: 0 }
    }

    pub fn reset(&mut self) {
        self.hits = [[f32::NAN; MAX_FAN_RAYS]; MAX_FANS];
        self.checked = [[0; MAX_FAN_RAYS]; MAX_FANS];
    }

    /// One frame. Returns the report (count 0 while the fans are off).
    pub fn step(&mut self, veh: i32, layout: &FanLayout) -> RangeReport {
        self.frame = self.frame.wrapping_add(1);
        self.cast = 0;
        let mut r = RangeReport::default();
        if layout.flags & fan_flags::ACTIVE == 0 || layout.count == 0 {
            self.reset();
            return r;
        }
        let fans = layout.as_slice();
        let ep: Vector3 = unsafe { invoke!(n::GET_ENTITY_COORDS, veh, true) };
        let ef: Vector3 = unsafe { invoke!(n::GET_ENTITY_FORWARD_VECTOR, veh) };
        let ego = EgoPose::new(V2::new(ep.x, ep.y), V2::new(ef.x, ef.y));
        // Every ray, in a fixed order; the most overdue first, within budget.
        let rays: Vec<(usize, usize)> = fans.iter().enumerate().flat_map(|(i, f)| (0..(f.rays as usize).clamp(1, MAX_FAN_RAYS)).map(move |k| (i, k))).collect();
        let n_rays = rays.len().max(1);
        let per_frame = fans.iter().map(|f| f.rays.clamp(1, MAX_FAN_RAYS as u32) as f32 / f.refresh_frames.max(1.0)).sum::<f32>().ceil() as usize;
        for _ in 0..per_frame.clamp(1, MAX_RAYS_PER_FRAME).min(n_rays) {
            let (i, k) = rays[self.cursor % n_rays];
            self.cursor = (self.cursor + 1) % n_rays;
            let f = &fans[i];
            let a = FanLayout::ray_angle(f, k);
            let from = ego.to_world(V2::new(f.x, f.y));
            let to = ego.to_world(V2::new(f.x + f.range * a.cos(), f.y + f.range * a.sin()));
            let z = ep.z + f.z;
            let h: i32 = unsafe { invoke!(n::START_EXPENSIVE_SYNCHRONOUS_SHAPE_TEST_LOS_PROBE, from.x, from.y, z, to.x, to.y, z, FAN_FLAGS, veh, 7i32) };
            let mut hit = 0i32;
            let mut end = Vector3::default();
            let mut normal = Vector3::default();
            let mut entity = 0i32;
            let status: i32 = unsafe {
                invoke!(n::GET_SHAPE_TEST_RESULT, h, &mut hit as *mut i32, &mut end as *mut Vector3, &mut normal as *mut Vector3, &mut entity as *mut i32)
            };
            self.cast += 1;
            if status != 2 {
                continue;
            }
            self.hits[i][k] = if hit != 0 { ((end.x - from.x).powi(2) + (end.y - from.y).powi(2) + (end.z - z).powi(2)).sqrt() } else { f32::INFINITY };
            self.checked[i][k] = self.frame;
        }
        r.count = fans.len() as u32;
        for (i, f) in fans.iter().enumerate() {
            let n = (f.rays as usize).clamp(1, MAX_FAN_RAYS);
            let mut res = FanResult { age: 0, ..Default::default() };
            for k in 0..n {
                res.hits[k] = self.hits[i][k];
                let age = if self.checked[i][k] == 0 { u32::MAX } else { self.frame.wrapping_sub(self.checked[i][k]) };
                res.age = res.age.max(age);
            }
            r.fans[i] = res;
        }
        r
    }
}
