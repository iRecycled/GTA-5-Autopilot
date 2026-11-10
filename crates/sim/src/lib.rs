//! A closed-loop simulator that speaks the adapter contract (D20).
//!
//! It produces the same messages a host adapter does (`VehicleSample`,
//! `ActorList` with line-of-sight bits, `AdapterInfo`) and takes the core's
//! `Command`, so the core is tested end to end without a game. Being a second
//! "host" also keeps the contract honest: nothing in it may be GTA-shaped.
//!
//! The road is straight and then bends at a constant radius; the ego car
//! keeps its lane perfectly (the simulated driver steers), and its speed comes
//! from the core's pedal commands through a simple powertrain and brake model
//! with actuator lag. It can reverse along the road and signal.
//!
//! Actors either follow the road (a lane offset, lane changes) or drive in a
//! fixed world direction (`heading`: crossing traffic in a car park). Static
//! obstacles (walls) block the range fans and can be hit, but aren't in the
//! actor list: the radars only see actors (D25). A traffic generator keeps
//! cars in chosen lanes around the ego car for long drives.

use adapter_api::{
    Actor, ActorKind, ActorList, AdapterInfo, CONTRACT_VERSION, Command, FanLayout, FanResult, Header, MAX_ACTORS, MAX_FAN_RAYS, MsgKind,
    RangeReport, SensorLayout, VehicleSample, caps, command_flags, fixed_str, sample_flags,
};
use geom::{EgoPose, V2, wrap_pi};

/// An oriented box in some 2-D frame: centre, heading, length, width.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Obb {
    pub c: V2,
    pub yaw: f32,
    pub length: f32,
    pub width: f32,
}

impl Obb {
    fn axes(&self) -> (V2, V2) {
        let (s, c) = self.yaw.sin_cos();
        (V2::new(c, s), V2::new(-s, c))
    }

    /// Distance along a ray (origin `o`, unit direction `d`) to the box; 0 if
    /// `o` is inside; `None` if missed.
    pub fn ray(&self, o: V2, d: V2) -> Option<f32> {
        let (ax, ay) = self.axes();
        let rel = o - self.c;
        let (po, pd) = ([rel.dot(ax), rel.dot(ay)], [d.dot(ax), d.dot(ay)]);
        let half = [0.5 * self.length, 0.5 * self.width];
        let (mut t0, mut t1) = (0.0f32, f32::INFINITY);
        for i in 0..2 {
            if pd[i].abs() < 1e-9 {
                if po[i].abs() > half[i] {
                    return None;
                }
            } else {
                let (a, b) = ((-half[i] - po[i]) / pd[i], (half[i] - po[i]) / pd[i]);
                t0 = t0.max(a.min(b));
                t1 = t1.min(a.max(b));
            }
        }
        (t0 <= t1).then_some(t0)
    }

    /// Whether two boxes overlap (separating axis test).
    pub fn overlaps(&self, o: &Obb) -> bool {
        let (a1, a2) = self.axes();
        let (b1, b2) = o.axes();
        let d = o.c - self.c;
        let r = |axis: V2, x: &Obb, u: V2, v: V2| 0.5 * x.length * u.dot(axis).abs() + 0.5 * x.width * v.dot(axis).abs();
        [a1, a2, b1, b2].iter().all(|&ax| d.dot(ax).abs() <= r(ax, self, a1, a2) + r(ax, o, b1, b2))
    }
}

/// A static obstacle (a wall, a pillar), axis-aligned with the road at `s`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Obstacle {
    pub s: f32,
    pub d: f32,
    pub length: f32,
    pub width: f32,
}

/// Generated traffic in one lane, kept within `range` of the ego car.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TrafficLane {
    /// Lateral offset (m, left +).
    pub d: f32,
    /// Mean speed and its ± spread (m/s).
    pub speed: f32,
    pub spread: f32,
    /// Mean gap between cars (m).
    pub gap: f32,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Traffic {
    pub lanes: Vec<TrafficLane>,
    /// Cars farther than this from the ego car are replaced at the other end (m).
    pub range: f32,
    pub seed: u64,
}

/// The point of an actor's box nearest to `(mx, my)` (ego frame).
fn nearest_box_point(a: &Actor, mx: f32, my: f32) -> (f32, f32) {
    let (s, c) = a.yaw.sin_cos();
    let (dx, dy) = (mx - a.x, my - a.y);
    let (lx, ly) = (c * dx + s * dy, -s * dx + c * dy);
    let (cx, cy) = (lx.clamp(-0.5 * a.length, 0.5 * a.length), ly.clamp(-0.5 * a.width, 0.5 * a.width));
    (a.x + c * cx - s * cy, a.y + s * cx + c * cy)
}

/// Obstacle ids in [`Collision::actor`] start here.
pub const OBSTACLE_ID_BASE: u32 = 1_000_000;

pub mod road;
pub use road::{Marking, MarkingKind, Road};

/// Who steers the ego car when the core doesn't.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum DriverSteer {
    /// Holds the car exactly on its line (Phases 1 and 2's driver).
    Perfect,
    /// Steers toward this offset (m, left +) like a driver would.
    Hands(f32),
    /// Hands off: the wheel returns to centre (self-aligning, about 1 s).
    Off,
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
    pub wheelbase: f32,
    /// The steering's first-order lag (s).
    pub steer_lag: f32,
    /// Understeer: the car turns `1 / (1 + (v / char_speed)²)` of what its
    /// wheel angle says (the characteristic speed, m/s; infinite = none).
    pub char_speed: f32,
    /// The yaw's first-order lag behind the wheel angle (s; 0 = none).
    pub yaw_lag: f32,
}

impl Default for EgoParams {
    fn default() -> Self {
        Self { accel_max: 4.5, v_max: 55.0, brake_max: 8.0, lag: 0.15, length: 4.7, width: 1.9, wheelbase: 2.7, steer_lag: 0.08, char_speed: f32::INFINITY, yaw_lag: 0.0 }
    }
}

impl EgoParams {
    /// Steers like a GTA car, measured on the Sultan (2026-11-09): it turns
    /// 0.75, 0.27 and 0.15 of the bicycle model's curvature at 35, 80 and
    /// 120 km/h, with its yaw about 0.1 s behind the wheels.
    pub fn gta() -> Self {
        Self { char_speed: 15.0, yaw_lag: 0.1, ..Self::default() }
    }

    /// The share of the bicycle model's curvature the car turns at `v`.
    pub fn steer_response(&self, v: f32) -> f32 {
        1.0 / (1.0 + (v / self.char_speed).powi(2))
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
    /// `Some`: drive in this fixed direction instead of along the road (rad,
    /// relative to the road's direction at `s`; π/2 = toward +d). Lane events
    /// don't apply.
    pub heading: Option<f32>,
    pub kind: ActorKind,
}

impl ActorSpec {
    /// A car following the road.
    pub fn car(s: f32, d: f32, v: f32) -> Self {
        Self { s, d, v, length: 4.5, width: 1.9, events: vec![], heading: None, kind: ActorKind::Vehicle }
    }

    /// A pedestrian standing at (s, d), walking at `heading` when told to.
    pub fn pedestrian(s: f32, d: f32, heading: f32) -> Self {
        Self { s, d, v: 0.0, length: 0.5, width: 0.5, events: vec![], heading: Some(heading), kind: ActorKind::Pedestrian }
    }
}

#[derive(Clone, Debug)]
struct ActorState {
    id: u32,
    spec: ActorSpec,
    s: f32,
    d: f32,
    v: f32,
    v_target: f32,
    accel: f32,
    lane: Option<(f32, f32, f32, f32)>, // from, to, start time, duration
    next_event: usize,
    /// Free-moving actors: world position and heading.
    free: Option<(V2, f32)>,
    /// Generated traffic: the lane it belongs to.
    traffic_lane: Option<usize>,
}

/// A small deterministic generator (xorshift64*).
#[derive(Clone, Debug)]
struct Rng(u64);

impl Rng {
    fn unit(&mut self) -> f32 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        (self.0.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 40) as f32 / (1u64 << 24) as f32
    }
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
    /// Ego lateral offset from the road's reference line (m, left +), its
    /// heading relative to the road (rad) and road-wheel angle (rad).
    pub d: f32,
    pub psi: f32,
    pub steer: f32,
    /// The curvature the car turns at (1/m): the wheels' after understeer
    /// and the yaw lag.
    pub kappa: f32,
    /// Who steers when the core doesn't.
    pub driver: DriverSteer,
    /// The core steered in the last step.
    pub core_steering: bool,
    /// Ego longitudinal acceleration actually produced (m/s²).
    pub a: f32,
    actors: Vec<ActorState>,
    pub collision: Option<Collision>,
    held: bool,
    /// Reverse is selected: the car moves backwards along the road.
    pub reverse: bool,
    /// The turn signal: 1 left, −1 right, 0 off.
    pub turn: i8,
    pub obstacles: Vec<Obstacle>,
    traffic: Option<(Traffic, Rng)>,
    next_id: u32,
}

/// Actor ids start here, so they never look like zero.
const ID_BASE: u32 = 100;

impl World {
    pub fn new(road: Road, ego_params: EgoParams, ego_speed: f32, actors: Vec<ActorSpec>, fps: f32) -> Self {
        let mut w = Self {
            road,
            ego_params,
            dt: 1.0 / fps,
            t: 0.0,
            frame: 1,
            s: 0.0,
            v: ego_speed,
            d: 0.0,
            psi: 0.0,
            steer: 0.0,
            kappa: 0.0,
            driver: DriverSteer::Perfect,
            core_steering: false,
            a: 0.0,
            actors: Vec::new(),
            collision: None,
            held: false,
            reverse: false,
            turn: 0,
            obstacles: Vec::new(),
            traffic: None,
            next_id: ID_BASE,
        };
        for spec in actors {
            w.push_actor(spec, None);
        }
        w
    }

    fn push_actor(&mut self, spec: ActorSpec, traffic_lane: Option<usize>) {
        let free = spec.heading.map(|h| {
            let (p, road_h) = self.road.pose(spec.s, spec.d);
            (p, road_h + h)
        });
        let id = self.next_id;
        self.next_id += 1;
        self.actors.push(ActorState { id, s: spec.s, d: spec.d, v: spec.v, v_target: spec.v, accel: 0.0, lane: None, next_event: 0, free, traffic_lane, spec });
    }

    /// Fills the traffic lanes from `range` behind to `range` ahead of the
    /// ego car; from then on, cars that drift out of range are replaced at
    /// the other end.
    pub fn set_traffic(&mut self, traffic: Traffic) {
        let mut rng = Rng(traffic.seed | 1);
        for (li, l) in traffic.lanes.iter().enumerate() {
            let mut s = self.s - traffic.range + l.gap * rng.unit();
            while s < self.s + traffic.range {
                let v = (l.speed + l.spread * (2.0 * rng.unit() - 1.0)).max(0.0);
                // Not on top of the ego car in its own lane.
                if (l.d - self.d).abs() > 0.5 * self.road.lane_width || (s - self.s).abs() > 30.0 {
                    self.push_actor(ActorSpec::car(s, l.d, v), Some(li));
                }
                s += l.gap * (0.5 + rng.unit());
            }
        }
        self.traffic = Some((traffic, rng));
    }

    /// The ego car's world position and heading.
    pub fn ego_world(&self) -> (V2, f32) {
        let (p, h) = self.road.pose(self.s, self.d);
        (p, h + self.psi)
    }

    fn ego_pose(&self) -> EgoPose {
        let (p, h) = self.ego_world();
        EgoPose::new(p, V2::new(h.cos(), h.sin()))
    }

    /// The lane markings around the ego car (`caps::MAP`): every marking
    /// present from 10 m behind to 150 m ahead, sampled every 8 m of road.
    pub fn lane_map(&self) -> adapter_api::LaneMap {
        use adapter_api::{LaneMap, MAX_LANE_LINES, MAX_LINE_POINTS, line_kind};
        let ego = self.ego_pose();
        let mut m = LaneMap { header: self.header(MsgKind::LaneMap), speed_limit: self.road.speed_limit, ..Default::default() };
        for mk in &self.road.markings {
            if m.count as usize >= MAX_LANE_LINES {
                break;
            }
            let line = &mut m.lines[m.count as usize];
            for k in 0..MAX_LINE_POINTS {
                let s = self.s - 10.0 + 160.0 * k as f32 / (MAX_LINE_POINTS - 1) as f32;
                if let Some(d) = mk.d_at(s) {
                    let (p, _) = self.road.pose(s, d);
                    let q = ego.to_vehicle(p);
                    line.points[line.count as usize] = [q.x, q.y];
                    line.count += 1;
                }
            }
            if line.count >= 2 {
                line.kind = match mk.kind {
                    MarkingKind::Dashed => line_kind::DASHED,
                    MarkingKind::Solid => line_kind::SOLID,
                    MarkingKind::Edge => line_kind::EDGE,
                };
                m.count += 1;
            } else {
                *line = Default::default();
            }
        }
        m
    }

    /// Ground truth: the ego car's offset from the centre of the lane it is
    /// in (m, left +), and that lane's width; None off the road.
    pub fn lane_offset(&self) -> Option<(f32, f32)> {
        let (r, l) = self.road.lane_at(self.s, self.d)?;
        Some((self.d - 0.5 * (r + l), l - r))
    }

    /// An actor's world position and heading.
    fn actor_pose(&self, a: &ActorState) -> (V2, f32) {
        match a.free {
            Some(p) => p,
            None => self.road.pose(a.s, a.d),
        }
    }

    /// Every box the world holds, in the ego frame: (id, box, is an actor).
    fn boxes(&self) -> Vec<(u32, Obb, bool)> {
        let ego = self.ego_pose();
        let (_, ego_h) = self.ego_world();
        let mut out = Vec::with_capacity(self.actors.len() + self.obstacles.len());
        for a in &self.actors {
            let (p, h) = self.actor_pose(a);
            out.push((a.id, Obb { c: ego.to_vehicle(p), yaw: wrap_pi(h - ego_h), length: a.spec.length, width: a.spec.width }, true));
        }
        for (i, o) in self.obstacles.iter().enumerate() {
            let (p, h) = self.road.pose(o.s, o.d);
            out.push((OBSTACLE_ID_BASE + i as u32, Obb { c: ego.to_vehicle(p), yaw: wrap_pi(h - ego_h), length: o.length, width: o.width }, false));
        }
        out
    }

    /// The ego car's box in its own frame.
    fn ego_obb(&self) -> Obb {
        Obb { c: V2::new(0.0, 0.0), yaw: 0.0, length: self.ego_params.length, width: self.ego_params.width }
    }

    /// Answers the range fans: per ray, the distance to the first box.
    pub fn ranges(&self, fans: &FanLayout) -> RangeReport {
        let boxes = self.boxes();
        let mut r = RangeReport { header: self.header(MsgKind::RangeReport), count: fans.count.min(adapter_api::MAX_FANS as u32), ..Default::default() };
        for (i, f) in fans.as_slice().iter().enumerate() {
            let mut res = FanResult { age: 0, ..Default::default() };
            for k in 0..(f.rays as usize).clamp(1, MAX_FAN_RAYS) {
                let a = FanLayout::ray_angle(f, k);
                let (o, d) = (V2::new(f.x, f.y), V2::new(a.cos(), a.sin()));
                let hit = boxes.iter().filter_map(|(_, b, _)| b.ray(o, d)).filter(|&t| t <= f.range).fold(f32::INFINITY, f32::min);
                res.hits[k] = hit;
            }
            r.fans[i] = res;
        }
        r
    }

    /// Ground truth for scoring: the nearest gap (m) from the ego car's
    /// bumper to any box behind it (`behind`) or ahead of it, in its path
    /// (within 0.3 m of its sides).
    pub fn clearance(&self, behind: bool) -> f32 {
        let (l, w) = (self.ego_params.length, self.ego_params.width);
        let mut best = f32::INFINITY;
        for (_, b, _) in self.boxes() {
            // Sample the box's outline against the ego's bumper line.
            let (s, c) = b.yaw.sin_cos();
            for i in 0..=8 {
                for j in 0..=8 {
                    let (u, v) = ((i as f32 / 8.0 - 0.5) * b.length, (j as f32 / 8.0 - 0.5) * b.width);
                    let p = V2::new(b.c.x + c * u - s * v, b.c.y + s * u + c * v);
                    if p.y.abs() > 0.5 * w + 0.3 {
                        continue;
                    }
                    let gap = if behind { -0.5 * l - p.x } else { p.x - 0.5 * l };
                    if gap >= -0.05 {
                        best = best.min(gap.max(0.0));
                    }
                }
            }
        }
        best
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
            host: fixed_str("adas sim v2"),
            vehicle: fixed_str("sim sedan"),
            capabilities: caps::PEDALS
                | caps::HOLD
                | caps::ACTORS
                | caps::LINE_OF_SIGHT
                | caps::RANGE_FANS
                | caps::GEAR
                | caps::TURN_SIGNALS
                | caps::STEER
                | caps::MAP,
            _reserved: 0,
            box_min,
            box_max,
            max_steer_angle: 0.6,
            wheelbase: self.ego_params.wheelbase,
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
        let mut flags = flags;
        if self.reverse {
            flags |= sample_flags::REVERSE;
        }
        flags |= match self.turn {
            1 => sample_flags::TURN_LEFT,
            -1 => sample_flags::TURN_RIGHT,
            _ => 0,
        };
        VehicleSample {
            header: self.header(MsgKind::VehicleSample),
            flags,
            speed: self.v,
            vx: if self.reverse { -self.v } else { self.v },
            vy: 0.0,
            yaw_rate: self.yaw_rate(),
            steer_angle: self.steer,
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
        let (_, ego_h) = self.ego_world();
        let mut v: Vec<Actor> = self
            .actors
            .iter()
            .map(|a| {
                let (p, h) = self.actor_pose(a);
                let rel = ego.to_vehicle(p);
                let dv = ego.rotate_to_vehicle(V2::new(a.v * h.cos(), a.v * h.sin()));
                Actor {
                    id: a.id,
                    kind: a.spec.kind,
                    los_mask: 0,
                    los_age: 0,
                    x: rel.x,
                    y: rel.y,
                    z: 0.0,
                    vx: dv.x,
                    vy: dv.y,
                    yaw: wrap_pi(h - ego_h),
                    yaw_rate: if a.free.is_some() { 0.0 } else { a.v * self.road.curvature(a.s) },
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
                // Seen where its box is nearest the sensor (a car alongside a
                // corner radar is in its beam even when its centre isn't).
                // The front radar keeps the centre, as in Phase 1.
                let (tx, ty, short) = if m.half_fov > 0.5 {
                    let (px, py) = nearest_box_point(a, m.x, m.y);
                    (px, py, 0.2)
                } else {
                    (a.x, a.y, 0.5 * a.length.max(1.0))
                };
                let (dx, dy) = (tx - m.x, ty - m.y);
                let range = dx.hypot(dy).max(1e-3);
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
                    if along <= 0.0 || along >= range - short {
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

    /// The ego car's yaw rate (rad/s).
    fn yaw_rate(&self) -> f32 {
        let k = self.road.curvature(self.s);
        if self.core_steering || self.driver != DriverSteer::Perfect {
            let w = self.v * self.kappa;
            if self.reverse { -w } else { w }
        } else {
            self.v * k
        }
    }

    /// Moves the ego car `ds` (m, along its heading) and steers it: by the
    /// core when it asks (`STEER` while engaged, or lane keeping's `LKA`
    /// nudge), else by the simulated driver.
    fn lateral(&mut self, cmd: &Command, ds: f32) {
        let p = self.ego_params;
        let dt = self.dt;
        let core = cmd.flags & command_flags::STEER != 0 && cmd.flags & (command_flags::ENGAGED | command_flags::LKA) != 0;
        self.core_steering = core;
        let k = self.road.curvature(self.s);
        let want = if core {
            Some(cmd.steer_angle.clamp(-0.6, 0.6))
        } else {
            match self.driver {
                DriverSteer::Perfect => None,
                DriverSteer::Off => Some(self.steer * (1.0 - dt / 1.0).max(0.0)),
                DriverSteer::Hands(target) => {
                    // A driver's pursuit of the target line 1.2 s ahead.
                    let look = (1.2 * self.v).max(6.0);
                    let err = target - self.d - self.psi * look;
                    Some((p.wheelbase * (2.0 * err / (look * look) + k)).atan().clamp(-0.5, 0.5))
                }
            }
        };
        let sign = if self.reverse { -1.0 } else { 1.0 };
        match want {
            None => {
                // On its line exactly: the car follows the road.
                self.s += sign * ds;
                self.steer = (p.wheelbase * k).atan();
                self.kappa = k;
                self.psi = 0.0;
            }
            Some(delta) => {
                self.steer += (delta - self.steer) * (dt / p.steer_lag).min(1.0);
                // The curvature the car turns at: the wheels' (bicycle
                // model), less understeer, after the yaw lag.
                let want = self.steer.tan() / p.wheelbase * p.steer_response(self.v);
                self.kappa += (want - self.kappa) * if p.yaw_lag > 0.0 { (dt / p.yaw_lag).min(1.0) } else { 1.0 };
                // Kinematic bicycle in the road frame.
                let along = sign * ds * self.psi.cos() / (1.0 - k * self.d).max(0.2);
                self.d += sign * ds * self.psi.sin();
                self.psi = wrap_pi(self.psi + sign * ds * self.kappa - k * along);
                self.s += along;
            }
        }
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
        let ds = 0.5 * (v0 + self.v) * dt;
        self.lateral(cmd, ds);

        // Generated traffic follows the car ahead in its lane, and cars out
        // of range are replaced at the other end.
        self.traffic_step();

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
            if let Some((p, h)) = a.free.as_mut() {
                *p = *p + V2::new(a.v * dt * h.cos(), a.v * dt * h.sin());
                a.lane = None;
                continue;
            }
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
            let ego = self.ego_obb();
            if let Some((id, _, _)) = self.boxes().into_iter().find(|(_, b, _)| ego.overlaps(b)) {
                self.collision = Some(Collision { t, actor: id, speed: self.v });
            }
        }
    }

    fn traffic_step(&mut self) {
        let Some((tr, mut rng)) = self.traffic.take() else { return };
        let ego_s = self.s;
        // Car following: no faster than the car ahead when close.
        let n = self.actors.len();
        for i in 0..n {
            let Some(li) = self.actors[i].traffic_lane else { continue };
            let (s, want) = (self.actors[i].s, self.actors[i].spec.v);
            // The car ahead in its lane: traffic, or the ego car.
            let ego_ahead = ((self.actors[i].d - self.d).abs() < 0.5 * self.road.lane_width && self.s > s).then_some((self.s - s, self.v));
            let ahead = (0..n)
                .filter(|&j| j != i && self.actors[j].traffic_lane == Some(li) && self.actors[j].s > s)
                .map(|j| (self.actors[j].s - s, self.actors[j].v))
                .chain(ego_ahead)
                .min_by(|a, b| a.0.total_cmp(&b.0));
            let v = match ahead {
                Some((gap, v_lead)) if gap < 30.0 => want.min(v_lead * (gap / 20.0).clamp(0.5, 1.0)),
                _ => want,
            };
            (self.actors[i].v_target, self.actors[i].accel) = (v, 3.0);
        }
        // Out of range: a new car at the other end, if there is room.
        for i in 0..n {
            let Some(li) = self.actors[i].traffic_lane else { continue };
            let rel = self.actors[i].s - ego_s;
            if rel.abs() <= tr.range {
                continue;
            }
            let lane = tr.lanes[li];
            // The far end, or the first clear slot moving in from it.
            let (end, inward) = if rel > 0.0 { (ego_s - tr.range + 1.0, 1.0) } else { (ego_s + tr.range - 1.0, -1.0) };
            let slot = (0..12).map(|k| end + inward * k as f32 * 0.6 * lane.gap).find(|&s| {
                !self.actors.iter().enumerate().any(|(j, a)| j != i && a.traffic_lane == Some(li) && (a.s - s).abs() < 0.6 * lane.gap)
            });
            let Some(new_s) = slot else { continue };
            let v = (lane.speed + lane.spread * (2.0 * rng.unit() - 1.0)).max(0.0);
            let a = &mut self.actors[i];
            a.id = self.next_id;
            self.next_id += 1;
            (a.s, a.v, a.v_target, a.spec.v) = (new_s, v, v, v);
        }
        self.traffic = Some((tr, rng));
    }

    /// Bumper-to-bumper gap to the nearest actor ahead in the ego lane (m).
    pub fn gap_ahead(&self) -> Option<(f32, f32)> {
        let p = self.ego_params;
        self.actors
            .iter()
            .filter(|a| a.free.is_none() && (a.d - self.d).abs() < 0.5 * self.road.lane_width && a.s > self.s)
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
        ActorSpec::car(s, d, v)
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
    fn rays_and_overlaps() {
        let b = Obb { c: V2::new(-6.0, 0.0), yaw: core::f32::consts::FRAC_PI_2, length: 4.5, width: 1.9 };
        // From the rear bumper straight back: hits the near side at x = -5.05.
        let t = b.ray(V2::new(-2.35, 0.0), V2::new(-1.0, 0.0)).unwrap();
        assert!((t - 2.7).abs() < 1e-4, "{t}");
        assert!(b.ray(V2::new(-2.35, 0.0), V2::new(1.0, 0.0)).is_none(), "pointing away");
        assert!(b.ray(V2::new(-2.35, 3.0), V2::new(-1.0, 0.0)).is_none(), "beside it (it's 4.5 m long across y: ±2.25)");
        let ego = Obb { c: V2::new(0.0, 0.0), yaw: 0.0, length: 4.7, width: 1.9 };
        assert!(!ego.overlaps(&b));
        assert!(ego.overlaps(&Obb { c: V2::new(-3.0, 0.0), ..b }));
        // Diagonal boxes clear of each other's corners.
        let d = Obb { c: V2::new(3.9, 2.2), yaw: 0.785, length: 2.0, width: 2.0 };
        assert!(!ego.overlaps(&d));
    }

    #[test]
    fn reversing_toward_a_wall_the_fans_see_it_and_hit_it() {
        let mut w = World::new(Road::default(), EgoParams::default(), 0.0, vec![], 72.0);
        w.reverse = true;
        w.obstacles.push(Obstacle { s: -6.0, d: 0.0, length: 0.4, width: 6.0 });
        let mut fans = FanLayout { count: 1, ..Default::default() };
        fans.fans[0] = adapter_api::RangeFan { x: -2.35, y: 0.0, z: 0.5, facing: core::f32::consts::PI, half_spread: 0.4, range: 4.5, rays: 3, refresh_frames: 5.0 };
        let r = w.ranges(&fans);
        // 6 − 0.2 − 2.35 = 3.45 m to the wall; the slanted rays are longer.
        assert!((r.fans[0].hits[1] - 3.45).abs() < 1e-3 && r.fans[0].hits[0] > 3.45, "{:?}", r.fans[0]);
        assert!((w.clearance(true) - 3.45).abs() < 0.01 && w.clearance(false).is_infinite());
        assert!(w.sample(0, 0).flags & sample_flags::REVERSE != 0);
        let back = Command { flags: command_flags::ENGAGED, throttle: 0.3, ..Default::default() };
        for _ in 0..72 * 10 {
            w.step(&back);
        }
        assert!(w.s < -1.0, "moved backwards: s {}", w.s);
        assert_eq!(w.collision.map(|c| c.actor), Some(OBSTACLE_ID_BASE));
        // Walls aren't actors: the radars don't see them.
        assert_eq!(w.actors(&layout()).count, 0);
    }

    #[test]
    fn a_crossing_car_drives_across_the_road() {
        let mut spec = car(-8.0, -15.0, 8.0);
        spec.heading = Some(core::f32::consts::FRAC_PI_2);
        let mut w = World::new(Road::default(), EgoParams::default(), 0.0, vec![spec], 72.0);
        for _ in 0..72 {
            w.step(&Command::default());
        }
        let a = w.actors(&layout()).actors[0];
        assert!((a.y + 7.0).abs() < 0.2 && (a.x + 8.0).abs() < 1e-3, "({}, {})", a.x, a.y);
        assert!((a.vy - 8.0).abs() < 1e-3 && (a.yaw - core::f32::consts::FRAC_PI_2).abs() < 1e-4);
    }

    #[test]
    fn traffic_stays_around_the_ego_car() {
        let mut w = World::new(Road::default(), EgoParams::default(), 30.0, vec![], 72.0);
        let lane = |d, speed| TrafficLane { d, speed, spread: 3.0, gap: 40.0 };
        w.set_traffic(Traffic { lanes: vec![lane(3.6, 33.0), lane(-3.6, 25.0)], range: 150.0, seed: 7 });
        let n0 = w.actors(&layout()).count;
        let ids0: Vec<u32> = w.actors(&layout()).as_slice().iter().map(|a| a.id).collect();
        let keep = Command { flags: command_flags::ENGAGED, throttle: 0.42, ..Default::default() };
        for _ in 0..72 * 120 {
            w.step(&keep);
        }
        let l = w.actors(&layout());
        assert!(l.as_slice().iter().all(|a| a.x.abs() <= 152.0), "all within range");
        assert!((l.count as i32 - n0 as i32).abs() <= 2, "{} cars, started with {n0}", l.count);
        assert!(l.as_slice().iter().any(|a| !ids0.contains(&a.id)), "replaced cars get new ids");
        assert!(w.collision.is_none());
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
