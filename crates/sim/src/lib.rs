//! A closed-loop simulator that speaks the adapter contract (D20).
//!
//! It produces the same messages a host adapter does (`VehicleSample`,
//! `ActorList` with line-of-sight bits, `AdapterInfo`) and takes the core's
//! `Command`, so the core is tested end to end without a game. Being a second
//! "host" also keeps the contract honest: nothing in it may be GTA-shaped.
//!
//! Version 1 is longitudinal: the road is straight and then bends at a
//! constant radius; the ego car keeps its lane perfectly (the simulated driver
//! steers), and its speed comes from the core's pedal commands through a
//! simple powertrain and brake model with actuator lag.

use adapter_api::{
    Actor, ActorKind, ActorList, AdapterInfo, CONTRACT_VERSION, Command, Header, MAX_ACTORS, MsgKind, SensorLayout, VehicleSample,
    caps, command_flags, fixed_str,
};
use geom::{EgoPose, V2, wrap_pi};

/// The road: straight for `straight` metres, then a left bend of `radius`
/// (infinite = straight forever). Lane 0 is the ego lane; +1 is to its left.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Road {
    pub straight: f32,
    pub radius: f32,
    pub lane_width: f32,
}

impl Default for Road {
    fn default() -> Self {
        Self { straight: f32::INFINITY, radius: f32::INFINITY, lane_width: 3.6 }
    }
}

impl Road {
    /// World position and heading at arc length `s`, lateral offset `d` (left +).
    pub fn pose(&self, s: f32, d: f32) -> (V2, f32) {
        if s <= self.straight || !self.radius.is_finite() {
            return (V2::new(s, d), 0.0);
        }
        let th = (s - self.straight) / self.radius;
        let r = self.radius - d;
        (V2::new(self.straight + r * th.sin(), self.radius - r * th.cos()), th)
    }

    /// Curvature at `s` (1/m).
    pub fn curvature(&self, s: f32) -> f32 {
        if s > self.straight && self.radius.is_finite() { 1.0 / self.radius } else { 0.0 }
    }
}

/// The simulated ego car.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EgoParams {
    /// Peak traction acceleration at full throttle and low speed (m/s²).
    pub accel_max: f32,
    /// Speed at which traction acceleration falls to zero (m/s).
    pub v_max: f32,
    /// Deceleration at full brake (m/s²).
    pub brake_max: f32,
    /// First-order actuator lag (s).
    pub lag: f32,
    pub length: f32,
    pub width: f32,
}

impl Default for EgoParams {
    fn default() -> Self {
        Self { accel_max: 4.5, v_max: 55.0, brake_max: 8.0, lag: 0.15, length: 4.7, width: 1.9 }
    }
}

/// A scripted actor event.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Event {
    /// Change speed to `v` at `accel` (m/s², positive magnitude).
    Speed { v: f32, accel: f32 },
    /// Move to lateral offset `d` (m, left +) over `duration` seconds.
    Lane { d: f32, duration: f32 },
}

#[derive(Clone, Debug, PartialEq)]
pub struct ActorSpec {
    /// Arc length of the actor's centre along the road (m), at t = 0.
    pub s: f32,
    /// Lateral offset (m, left +).
    pub d: f32,
    pub v: f32,
    pub length: f32,
    pub width: f32,
    /// (time, event), in time order.
    pub events: Vec<(f32, Event)>,
}

#[derive(Clone, Debug)]
struct ActorState {
    spec: ActorSpec,
    s: f32,
    d: f32,
    v: f32,
    v_target: f32,
    accel: f32,
    lane: Option<(f32, f32, f32, f32)>, // from, to, start time, duration
    next_event: usize,
}

/// What happened, for scoring.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Collision {
    pub t: f32,
    pub actor: u32,
    pub speed: f32,
}

pub struct World {
    pub road: Road,
    pub ego_params: EgoParams,
    pub dt: f32,
    pub t: f32,
    pub frame: u64,
    /// Ego arc length and speed.
    pub s: f32,
    pub v: f32,
    /// Ego longitudinal acceleration actually produced (m/s²).
    pub a: f32,
    actors: Vec<ActorState>,
    pub collision: Option<Collision>,
    held: bool,
}

/// Actor ids start here, so they never look like zero.
const ID_BASE: u32 = 100;

impl World {
    pub fn new(road: Road, ego_params: EgoParams, ego_speed: f32, actors: Vec<ActorSpec>, fps: f32) -> Self {
        let actors = actors
            .into_iter()
            .map(|spec| ActorState { s: spec.s, d: spec.d, v: spec.v, v_target: spec.v, accel: 0.0, lane: None, next_event: 0, spec })
            .collect();
        Self { road, ego_params, dt: 1.0 / fps, t: 0.0, frame: 1, s: 0.0, v: ego_speed, a: 0.0, actors, collision: None, held: false }
    }

    fn ego_pose(&self) -> EgoPose {
        let (p, h) = self.road.pose(self.s, 0.0);
        EgoPose::new(p, V2::new(h.cos(), h.sin()))
    }

    /// The ego car's box relative to its reference point (its centre).
    fn ego_box(&self) -> ([f32; 3], [f32; 3]) {
        let (l, w) = (self.ego_params.length, self.ego_params.width);
        ([-l / 2.0, -w / 2.0, 0.0], [l / 2.0, w / 2.0, 1.5])
    }

    pub fn info(&self) -> AdapterInfo {
        let (box_min, box_max) = self.ego_box();
        AdapterInfo {
            header: Header { kind: MsgKind::AdapterInfo, version: CONTRACT_VERSION, seq: 1, ..Default::default() },
            host: fixed_str("adas sim v1"),
            vehicle: fixed_str("sim sedan"),
            capabilities: caps::PEDALS | caps::HOLD | caps::ACTORS | caps::LINE_OF_SIGHT,
            _reserved: 0,
            box_min,
            box_max,
            max_steer_angle: f32::NAN,
            wheelbase: 2.7,
        }
    }

    fn header(&self, kind: MsgKind) -> Header {
        Header {
            kind,
            version: CONTRACT_VERSION,
            seq: self.frame as u32,
            frame: self.frame,
            game_time_us: (self.t as f64 * 1e6).round() as u64 + 1,
            wall_time_ns: self.frame * 1_000_000,
        }
    }

    pub fn sample(&self, flags: u32, buttons: u32) -> VehicleSample {
        VehicleSample {
            header: self.header(MsgKind::VehicleSample),
            flags,
            speed: self.v,
            vx: self.v,
            vy: 0.0,
            yaw_rate: self.v * self.road.curvature(self.s),
            steer_angle: f32::NAN,
            wheel_omega: [f32::NAN; 4],
            rev_ratio: f32::NAN,
            gear: 1,
            driver_throttle: 0.0,
            driver_brake: 0.0,
            driver_steer: 0.0,
            buttons,
        }
    }

    /// Actors in the ego frame, nearest first, with line-of-sight bits for
    /// each sensor of `layout` (blocked by a nearer actor across the line).
    pub fn actors(&self, layout: &SensorLayout) -> ActorList {
        let ego = self.ego_pose();
        let (_, ego_h) = self.road.pose(self.s, 0.0);
        let mut v: Vec<Actor> = self
            .actors
            .iter()
            .enumerate()
            .map(|(i, a)| {
                let (p, h) = self.road.pose(a.s, a.d);
                let rel = ego.to_vehicle(p);
                let dv = ego.rotate_to_vehicle(V2::new(a.v * h.cos(), a.v * h.sin()));
                Actor {
                    id: ID_BASE + i as u32,
                    kind: ActorKind::Vehicle,
                    los_mask: 0,
                    los_age: 0,
                    x: rel.x,
                    y: rel.y,
                    z: 0.0,
                    vx: dv.x,
                    vy: dv.y,
                    yaw: wrap_pi(h - ego_h),
                    yaw_rate: a.v * self.road.curvature(a.s),
                    length: a.spec.length,
                    width: a.spec.width,
                    height: 1.5,
                }
            })
            .collect();
        v.sort_by(|a, b| a.x.hypot(a.y).total_cmp(&b.x.hypot(b.y)));
        let mounts = layout.as_slice();
        let snapshot = v.clone();
        for a in &mut v {
            let mut mask = 0u8;
            for (k, m) in mounts.iter().enumerate() {
                let (dx, dy) = (a.x - m.x, a.y - m.y);
                let range = dx.hypot(dy);
                if range > m.range || wrap_pi(dy.atan2(dx) - m.facing).abs() > m.half_fov {
                    continue;
                }
                // Blocked if another actor nearer to the mount sits across the line.
                let blocked = snapshot.iter().any(|b| {
                    if b.id == a.id {
                        return false;
                    }
                    let (bx, by) = (b.x - m.x, b.y - m.y);
                    let along = (bx * dx + by * dy) / range;
                    if along <= 0.0 || along >= range - 0.5 * a.length.max(1.0) {
                        return false;
                    }
                    let across = (bx * dy - by * dx).abs() / range;
                    across < 0.5 * b.width
                });
                if !blocked {
                    mask |= 1 << k;
                }
            }
            a.los_mask = mask;
        }
        let mut list = ActorList { header: self.header(MsgKind::ActorList), count: v.len().min(MAX_ACTORS) as u32, ..Default::default() };
        list.dropped = v.len().saturating_sub(MAX_ACTORS) as u32;
        list.actors[..list.count as usize].copy_from_slice(&v[..list.count as usize]);
        list
    }

    /// Advances one frame under `cmd`.
    pub fn step(&mut self, cmd: &Command) {
        let p = self.ego_params;
        let dt = self.dt;
        let hold = cmd.flags & command_flags::HOLD != 0;
        let engaged = cmd.flags & (command_flags::ENGAGED | command_flags::AEB) != 0;
        let (thr, brk) = if engaged { (cmd.throttle.clamp(0.0, 1.0), cmd.brake.clamp(0.0, 1.0)) } else { (0.0, 0.0) };
        let traction = thr * p.accel_max * (1.0 - self.v / p.v_max).max(0.0);
        let resist = if self.v > 0.0 { 0.15 + 0.0004 * self.v * self.v } else { 0.0 };
        let want = traction - brk * p.brake_max - resist;
        self.a += (want - self.a) * (dt / p.lag).min(1.0);
        if hold && self.v < 0.5 {
            self.held = true;
        }
        if self.held && (!hold || thr > 0.0) {
            self.held = false;
        }
        let v0 = self.v;
        self.v = if self.held { 0.0 } else { (self.v + self.a * dt).max(0.0) };
        if self.v == 0.0 && self.a < 0.0 {
            self.a = 0.0;
        }
        self.s += 0.5 * (v0 + self.v) * dt;

        // Actors.
        let t = self.t + dt;
        for a in &mut self.actors {
            while let Some(&(et, ev)) = a.spec.events.get(a.next_event) {
                if et > t {
                    break;
                }
                match ev {
                    Event::Speed { v, accel } => {
                        a.v_target = v.max(0.0);
                        a.accel = accel.abs();
                    }
                    Event::Lane { d, duration } => a.lane = Some((a.d, d, et, duration.max(0.1))),
                }
                a.next_event += 1;
            }
            let dv = a.v_target - a.v;
            let step = a.accel * dt;
            a.v += if a.accel > 0.0 { dv.clamp(-step, step) } else { dv };
            a.s += a.v * dt;
            if let Some((from, to, start, dur)) = a.lane {
                let f = ((t - start) / dur).clamp(0.0, 1.0);
                // Smooth (cosine) lateral profile, like a real lane change.
                a.d = from + (to - from) * (0.5 - 0.5 * (core::f32::consts::PI * f).cos());
                if f >= 1.0 {
                    a.lane = None;
                }
            }
        }
        self.t = t;
        self.frame += 1;

        // Collision: box overlap in the ego frame.
        if self.collision.is_none() {
            let ego = self.ego_pose();
            for (i, a) in self.actors.iter().enumerate() {
                let (pw, _) = self.road.pose(a.s, a.d);
                let rel = ego.to_vehicle(pw);
                if rel.x.abs() < 0.5 * (p.length + a.spec.length) && rel.y.abs() < 0.5 * (p.width + a.spec.width) {
                    self.collision = Some(Collision { t, actor: ID_BASE + i as u32, speed: self.v });
                    break;
                }
            }
        }
    }

    /// Bumper-to-bumper gap to the nearest actor ahead in the ego lane (m).
    pub fn gap_ahead(&self) -> Option<(f32, f32)> {
        let p = self.ego_params;
        self.actors
            .iter()
            .filter(|a| a.d.abs() < 0.5 * self.road.lane_width && a.s > self.s)
            .map(|a| (a.s - self.s - 0.5 * (p.length + a.spec.length), a.v))
            .min_by(|x, y| x.0.total_cmp(&y.0))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn layout() -> SensorLayout {
        let mut l = SensorLayout { count: 1, ..Default::default() };
        l.mounts[0] = adapter_api::SensorMount { x: 2.35, y: 0.0, z: 0.5, facing: 0.0, half_fov: 10f32.to_radians(), range: 160.0, refresh_frames: 3.6, _reserved: 0 };
        l
    }

    fn car(s: f32, d: f32, v: f32) -> ActorSpec {
        ActorSpec { s, d, v, length: 4.5, width: 1.9, events: vec![] }
    }

    #[test]
    fn full_throttle_and_brake_are_plausible() {
        let mut w = World::new(Road::default(), EgoParams::default(), 0.0, vec![], 72.0);
        let full = Command { flags: command_flags::ENGAGED, throttle: 1.0, ..Default::default() };
        for _ in 0..(72.0 * 5.0) as usize {
            w.step(&full);
        }
        assert!((14.0..24.0).contains(&w.v), "0-5 s at full throttle: {} m/s", w.v);
        let brake = Command { flags: command_flags::ENGAGED, brake: 1.0, ..Default::default() };
        let v0 = w.v;
        let mut t = 0.0;
        while w.v > 0.0 && t < 10.0 {
            w.step(&brake);
            t += w.dt;
        }
        let decel = v0 / t;
        assert!((6.0..8.5).contains(&decel), "mean full-brake decel {decel}");
    }

    #[test]
    fn occlusion_hides_the_car_behind() {
        let w = World::new(Road::default(), EgoParams::default(), 20.0, vec![car(30.0, 0.0, 20.0), car(60.0, 0.0, 20.0), car(45.0, 3.6, 20.0)], 72.0);
        let l = w.actors(&layout());
        let by_id = |id: u32| l.as_slice().iter().find(|a| a.id == id).copied().unwrap();
        assert_eq!(by_id(100).los_mask, 1, "nearest car visible");
        assert_eq!(by_id(101).los_mask, 0, "car behind it hidden");
        // The next-lane car at 45 m is 4.6° off-axis: inside the ±10° beam and not blocked.
        assert_eq!(by_id(102).los_mask, 1, "next-lane car visible");
    }

    #[test]
    fn bend_geometry() {
        let r = Road { straight: 100.0, radius: 200.0, lane_width: 3.6 };
        let (p, h) = r.pose(100.0 + 200.0 * core::f32::consts::FRAC_PI_2, 0.0);
        assert!((p.x - 300.0).abs() < 1e-2 && (p.y - 200.0).abs() < 1e-2);
        assert!((h - core::f32::consts::FRAC_PI_2).abs() < 1e-5);
    }

    #[test]
    fn collision_is_detected() {
        let mut w = World::new(Road::default(), EgoParams::default(), 20.0, vec![car(20.0, 0.0, 0.0)], 72.0);
        for _ in 0..200 {
            w.step(&Command::default());
        }
        let c = w.collision.expect("drove into a stopped car");
        assert_eq!(c.actor, 100);
    }
}
