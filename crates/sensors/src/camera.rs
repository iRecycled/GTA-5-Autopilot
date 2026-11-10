//! The forward camera (D26): lane lines, objects and the speed limit, as a
//! production camera ECU reports them, from ground truth through an error
//! model. Phase 5 replaces its eyes with a detector on real frames; its
//! output stays the same.
//!
//! - **Lanes:** the four markings nearest the car (left, right, and the next
//!   one out on each side), each fitted as a cubic y = c0 + c1·x + c2·x² +
//!   c3·x³ over the stretch the camera sees (up to 80 m), with the visible
//!   range, the marking type and a quality. Noise on the offset, heading and
//!   curvature, and the occasional frame where a line is missed.
//! - **Objects:** vehicles (to 120 m) and pedestrians (to 60 m) in its view
//!   (±50° across, ±25° up and down), standing on the road and in sight, classified (with rare misclassification). Range is
//!   worse than radar's (about 0.2 m + 4 %), as for a monocular camera.
//! - 30 Hz.

use adapter_api::{ActorKind, ActorList, LaneMap, SensorMount, line_kind};

use crate::{normal, unit};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CameraConfig {
    pub period: f32,
    pub lane_range: f32,
    pub c0_noise: f32,
    pub c1_noise: f32,
    pub c2_noise: f32,
    /// Probability of missing a line in one frame.
    pub p_line_miss: f32,
    pub vehicle_range: f32,
    pub pedestrian_range: f32,
    pub range_noise_m: f32,
    pub range_noise_frac: f32,
    pub lateral_noise: f32,
    pub vel_noise: f32,
    pub p_miss: f32,
    pub p_misclass: f32,
    /// Half the vertical field of view (rad).
    pub half_vfov: f32,
    pub seed: u64,
}

impl Default for CameraConfig {
    fn default() -> Self {
        Self {
            period: 1.0 / 30.0,
            lane_range: 80.0,
            c0_noise: 0.03,
            c1_noise: 0.0015,
            c2_noise: 1.0e-5,
            p_line_miss: 0.005,
            vehicle_range: 120.0,
            pedestrian_range: 60.0,
            range_noise_m: 0.2,
            range_noise_frac: 0.04,
            lateral_noise: 0.08,
            vel_noise: 0.4,
            p_miss: 0.02,
            p_misclass: 0.01,
            half_vfov: 25f32.to_radians(),
            seed: 0x00CA_4E7A_0D26,
        }
    }
}

/// Object classes on the bus.
pub mod class {
    pub const UNKNOWN: u8 = 0;
    pub const CAR: u8 = 1;
    pub const TRUCK: u8 = 2;
    pub const PEDESTRIAN: u8 = 3;
}

/// One fitted line, in the ego frame (x from the reference point).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LaneFit {
    pub valid: bool,
    pub c: [f32; 4],
    /// How far ahead it was seen (m).
    pub range: f32,
    /// `adapter_api::line_kind`.
    pub kind: u8,
    /// 0..=3.
    pub quality: u8,
}

impl LaneFit {
    pub fn y(&self, x: f32) -> f32 {
        let c = &self.c;
        c[0] + x * (c[1] + x * (c[2] + x * c[3]))
    }
    /// Curvature at x (1/m, positive left), for small slopes.
    pub fn curvature(&self, x: f32) -> f32 {
        2.0 * self.c[2] + 6.0 * self.c[3] * x
    }
}

/// Line slots in a frame.
pub const LEFT: usize = 0;
pub const RIGHT: usize = 1;
pub const LEFT2: usize = 2;
pub const RIGHT2: usize = 3;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CamObject {
    pub id: u32,
    pub class: u8,
    /// Its near face (vehicles) or centre (pedestrians), ego frame (m).
    pub x: f32,
    pub y: f32,
    /// Over ground, in the ego frame (m/s).
    pub vx: f32,
    pub vy: f32,
    pub width: f32,
    pub confidence: f32,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct CameraFrame {
    pub frame: u32,
    pub lanes: [LaneFit; 4],
    pub objects: Vec<CamObject>,
    /// The speed limit it read (m/s); 0 = none.
    pub speed_limit: f32,
}

pub struct Camera {
    pub cfg: CameraConfig,
    /// The index of the camera in the sensor layout (its line-of-sight bit).
    pub index: usize,
    /// No line-of-sight checks from the host: everything in view counts.
    pub ignore_los: bool,
    since: f32,
    frame: u32,
}

/// A line's lateral offset near the car (x ≈ 0), and whether it is there.
fn offset_near(pts: &[[f32; 2]]) -> Option<f32> {
    let i = pts.iter().position(|p| p[0] >= 0.0)?;
    if pts[i][0] > 20.0 {
        return None;
    }
    if i == 0 {
        return Some(pts[0][1]);
    }
    let (a, b) = (pts[i - 1], pts[i]);
    let f = (0.0 - a[0]) / (b[0] - a[0]).max(1e-3);
    Some(a[1] + f * (b[1] - a[1]))
}

/// A least-squares cubic through points with x in [-5, range] (fewer terms
/// with few points). Returns (coefficients, farthest x used).
pub fn fit_cubic(pts: &[[f32; 2]], range: f32) -> Option<([f32; 4], f32)> {
    let use_: Vec<[f64; 2]> = pts.iter().filter(|p| p[0] >= -5.0 && p[0] <= range).map(|p| [p[0] as f64, p[1] as f64]).collect();
    if use_.len() < 2 {
        return None;
    }
    let far = use_.iter().map(|p| p[0]).fold(f64::MIN, f64::max);
    let terms = (use_.len()).min(4);
    // Scale x to keep the normal equations well conditioned.
    let sx = far.abs().max(10.0);
    let mut a = [[0f64; 5]; 4];
    for p in &use_ {
        let u = p[0] / sx;
        let pw = [1.0, u, u * u, u * u * u];
        for r in 0..terms {
            for c in 0..terms {
                a[r][c] += pw[r] * pw[c];
            }
            a[r][4] += pw[r] * p[1];
        }
    }
    // A light ridge on the higher terms keeps short fits sane.
    for (r, row) in a.iter_mut().enumerate().take(terms).skip(2) {
        row[r] += 1e-6 * r as f64;
    }
    // Gaussian elimination.
    for i in 0..terms {
        let piv = (i..terms).max_by(|&x, &y| a[x][i].abs().total_cmp(&a[y][i].abs()))?;
        a.swap(i, piv);
        if a[i][i].abs() < 1e-12 {
            return None;
        }
        for r in 0..terms {
            if r != i {
                let f = a[r][i] / a[i][i];
                for c in i..5 {
                    a[r][c] -= f * a[i][c];
                }
            }
        }
    }
    let mut c = [0f32; 4];
    for i in 0..terms {
        c[i] = (a[i][4] / a[i][i] / sx.powi(i as i32)) as f32;
    }
    Some((c, far as f32))
}

impl Camera {
    pub fn new(cfg: CameraConfig, index: usize) -> Self {
        Self { cfg, index, ignore_los: false, since: f32::INFINITY, frame: 0 }
    }

    /// Advances by `dt`; returns a frame when one is due.
    pub fn step(&mut self, dt: f32, mount: &SensorMount, actors: &ActorList, map: Option<&LaneMap>, frame: u64) -> Option<CameraFrame> {
        self.since += dt.max(0.0);
        if self.since < self.cfg.period {
            return None;
        }
        self.since = if self.since.is_finite() { (self.since - self.cfg.period).min(self.cfg.period) } else { 0.0 };
        self.frame = self.frame.wrapping_add(1);
        let c = self.cfg;
        let mut out = CameraFrame { frame: self.frame, ..Default::default() };

        // Lanes: the nearest markings each side.
        if let Some(m) = map {
            let mut left: Vec<(f32, usize)> = Vec::new();
            let mut right: Vec<(f32, usize)> = Vec::new();
            for (i, l) in m.as_slice().iter().enumerate() {
                if let Some(y0) = offset_near(l.as_slice()) {
                    if y0 >= 0.0 { left.push((y0, i)) } else { right.push((-y0, i)) }
                }
            }
            left.sort_by(|a, b| a.0.total_cmp(&b.0));
            right.sort_by(|a, b| a.0.total_cmp(&b.0));
            let slots = [(LEFT, left.first()), (RIGHT, right.first()), (LEFT2, left.get(1)), (RIGHT2, right.get(1))];
            for (slot, pick) in slots {
                let Some(&(_, i)) = pick else { continue };
                let line = &m.lines[i];
                let key = 100 + slot as u64;
                if unit(c.seed, frame, key, 1) < c.p_line_miss {
                    continue;
                }
                let Some((mut co, far)) = fit_cubic(line.as_slice(), c.lane_range) else { continue };
                co[0] += normal(c.seed, frame, key, 2) * c.c0_noise;
                co[1] += normal(c.seed, frame, key, 3) * c.c1_noise;
                co[2] += normal(c.seed, frame, key, 4) * c.c2_noise;
                let range = far.clamp(0.0, c.lane_range);
                out.lanes[slot] = LaneFit {
                    valid: range > 5.0,
                    c: co,
                    range,
                    kind: line.kind.min(line_kind::EDGE) as u8,
                    quality: if range > 40.0 { 3 } else if range > 20.0 { 2 } else { 1 },
                };
            }
            if m.speed_limit.is_finite() && m.speed_limit > 0.0 {
                out.speed_limit = m.speed_limit;
            }
        }

        // Objects in view and in sight.
        let bit = 1u8 << self.index;
        for a in actors.as_slice() {
            let ped = a.kind == ActorKind::Pedestrian;
            let (dx, dy) = (a.x - mount.x, a.y - mount.y);
            let range = dx.hypot(dy);
            let az = geom::wrap_pi(dy.atan2(dx) - mount.facing);
            let max = if ped { c.pedestrian_range } else { c.vehicle_range }.min(mount.range);
            if range > max || range < 1.0 || az.abs() > mount.half_fov || !crate::sees(a, mount.z, range, c.half_vfov) {
                continue;
            }
            if !self.ignore_los && (a.los_mask & bit == 0 || a.los_age as f32 > 4.0 * mount.refresh_frames.max(1.0)) {
                continue;
            }
            let id = a.id as u64;
            if unit(c.seed, frame, id, 11) < c.p_miss {
                continue;
            }
            let class = if unit(c.seed, frame, id, 12) < c.p_misclass {
                class::UNKNOWN
            } else if ped {
                class::PEDESTRIAN
            } else if a.length > 7.0 {
                class::TRUCK
            } else {
                class::CAR
            };
            // Vehicles by their near face, pedestrians by their centre.
            let near = if ped { 0.0 } else { 0.5 * (a.length * a.yaw.cos().abs() + a.width * a.yaw.sin().abs()) };
            let (ux, uy) = (dx / range, dy / range);
            let r_err = normal(c.seed, frame, id, 13) * (c.range_noise_m + c.range_noise_frac * range);
            let lat_err = normal(c.seed, frame, id, 14) * (c.lateral_noise + 0.003 * range);
            out.objects.push(CamObject {
                id: 1 + (unit(0xCA_3E1D, id, self.index as u64, 0) * 0x7FFE as f32) as u32,
                class,
                x: a.x - near * ux + r_err * ux - lat_err * uy,
                y: a.y - near * uy + r_err * uy + lat_err * ux,
                vx: a.vx + normal(c.seed, frame, id, 15) * c.vel_noise,
                vy: a.vy + normal(c.seed, frame, id, 16) * c.vel_noise,
                width: a.width.max(0.3),
                confidence: if class == class::UNKNOWN { 0.5 } else { 0.9 },
            });
        }
        Some(out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use adapter_api::{Actor, LaneLine};

    fn mount() -> SensorMount {
        SensorMount { x: 0.5, y: 0.0, z: 1.3, facing: 0.0, half_fov: 50f32.to_radians(), range: 120.0, refresh_frames: 2.4, _reserved: 0 }
    }

    /// Markings of a left bend of radius R, at offsets `ds`, as seen from
    /// the car at the road's centre heading along it.
    fn bend_map(radius: f32, ds: &[f32]) -> LaneMap {
        let mut m = LaneMap { count: ds.len() as u32, ..Default::default() };
        for (i, &d) in ds.iter().enumerate() {
            let mut l = LaneLine { kind: line_kind::DASHED, count: 20, ..Default::default() };
            for k in 0..20 {
                let s = -10.0 + 8.0 * k as f32;
                let th = s / radius;
                let r = radius - d;
                l.points[k] = [r * th.sin(), radius - r * th.cos()];
            }
            m.lines[i] = l;
        }
        m
    }

    #[test]
    fn fits_a_bend_and_picks_the_lines() {
        let cfg = CameraConfig { c0_noise: 0.0, c1_noise: 0.0, c2_noise: 0.0, p_line_miss: 0.0, ..Default::default() };
        let mut cam = Camera::new(cfg, 5);
        let map = bend_map(300.0, &[-5.4, -1.8, 1.8, 5.4]);
        let f = cam.step(0.04, &mount(), &ActorList::default(), Some(&map), 1).expect("a frame");
        let (l, r) = (f.lanes[LEFT], f.lanes[RIGHT]);
        assert!(l.valid && r.valid && f.lanes[LEFT2].valid && f.lanes[RIGHT2].valid);
        assert!((l.c[0] - 1.8).abs() < 0.02 && (r.c[0] + 1.8).abs() < 0.02, "offsets {} {}", l.c[0], r.c[0]);
        // Curvature 1/R at the centre: c2 ≈ 1/(2R), slightly more inside.
        assert!((r.curvature(0.0) - 1.0 / 301.8).abs() < 3e-4, "curvature {}", r.curvature(0.0));
        assert!((f.lanes[LEFT2].c[0] - 5.4).abs() < 0.05);
        assert!((l.range - 80.0).abs() < 8.1, "range {}", l.range);
        // 30 Hz.
        let mut cam = Camera::new(CameraConfig::default(), 5);
        let n = (0..72).filter(|&k| cam.step(1.0 / 72.0, &mount(), &ActorList::default(), Some(&map), k).is_some()).count();
        assert!((29..=31).contains(&n), "{n} frames per second");
    }

    #[test]
    fn sees_vehicles_and_pedestrians_in_view() {
        let cfg = CameraConfig { p_miss: 0.0, p_misclass: 0.0, range_noise_m: 0.0, range_noise_frac: 0.0, lateral_noise: 0.0, vel_noise: 0.0, ..Default::default() };
        let mut cam = Camera::new(cfg, 5);
        let mut l = ActorList { count: 4, ..Default::default() };
        let v = |id, x: f32, y: f32, kind| Actor { id, kind, los_mask: 1 << 5, los_age: 0, x, y, length: 4.5, width: 1.9, ..Actor::ZERO };
        l.actors[0] = v(1, 40.0, 0.0, ActorKind::Vehicle);
        l.actors[1] = Actor { length: 0.5, width: 0.5, ..v(2, 20.0, 3.0, ActorKind::Pedestrian) };
        l.actors[2] = Actor { length: 0.5, width: 0.5, ..v(3, 80.0, 0.0, ActorKind::Pedestrian) };
        l.actors[3] = v(4, -10.0, 0.0, ActorKind::Vehicle);
        let f = cam.step(0.04, &mount(), &l, None, 1).unwrap();
        assert_eq!(f.objects.len(), 2, "the far pedestrian and the car behind are out of sight: {:?}", f.objects);
        let car = f.objects.iter().find(|o| o.class == class::CAR).unwrap();
        assert!((car.x - 37.75).abs() < 0.01, "near face {}", car.x);
        assert!(f.objects.iter().any(|o| o.class == class::PEDESTRIAN && (o.x - 20.0).abs() < 0.1));
        assert!(f.lanes.iter().all(|l| !l.valid), "no map, no lanes");
    }
}
