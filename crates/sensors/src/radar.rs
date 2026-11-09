//! The front radar (D25): actor list → field of view and line of sight →
//! error model → an object list at the radar's update rate.
//!
//! Parameters follow a typical 77 GHz long-range automotive radar: 20 Hz
//! updates, about 160 m range, ±10° (long-range beam), range noise of about
//! 0.1 m + 0.5 %, range-rate noise of about 0.1 m/s, azimuth noise of about
//! 0.3°, around 1 % missed detections per object per scan and occasional
//! clutter. Stationary objects are reported, flagged as such: whether to act
//! on them is the feature's decision, as on a real car.

use adapter_api::{ActorList, SensorMount};

use crate::{normal, unit};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RadarConfig {
    /// Update period (s).
    pub period: f32,
    pub range_noise_m: f32,
    pub range_noise_frac: f32,
    pub rate_noise: f32,
    pub azimuth_noise_rad: f32,
    /// Probability of missing an object in a scan.
    pub p_miss: f32,
    /// Expected clutter (false) objects per scan.
    pub clutter_per_scan: f32,
    /// Objects closer than this along the boresight are not resolved (m).
    pub min_range: f32,
    pub seed: u64,
}

impl Default for RadarConfig {
    fn default() -> Self {
        Self {
            period: 0.05,
            range_noise_m: 0.10,
            range_noise_frac: 0.005,
            rate_noise: 0.10,
            azimuth_noise_rad: 0.3_f32.to_radians(),
            p_miss: 0.01,
            clutter_per_scan: 0.02,
            min_range: 0.5,
            seed: 0x5EED_0F_FAD,
        }
    }
}

/// One reported object, in the ego vehicle frame (ISO 8855, m and m/s),
/// relative to the ego reference point. `x`, `y` is the nearest face (where a
/// radar's reflection comes from), not the object's centre.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RadarObject {
    /// The radar's own track id (stable while it keeps seeing the object).
    pub id: u32,
    pub x: f32,
    pub y: f32,
    /// Velocity over ground, in the ego frame.
    pub vx: f32,
    pub vy: f32,
    /// Object width estimate (m).
    pub width: f32,
    /// Half its length (m): the distance from the reported point to the rear.
    pub half_length: f32,
    /// Over-ground speed below 0.5 m/s.
    pub stationary: bool,
    /// A clutter return, not a real object (for scoring only: a real radar
    /// can't tell; features must not read this).
    pub clutter: bool,
}

/// One scan's output.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct RadarScan {
    /// Increments every scan.
    pub scan: u32,
    pub objects: Vec<RadarObject>,
}

pub struct FrontRadar {
    cfg: RadarConfig,
    /// Index of this radar in the sensor layout (its line-of-sight bit).
    pub index: usize,
    since: f32,
    scan: u32,
}

impl FrontRadar {
    pub fn new(cfg: RadarConfig, index: usize) -> Self {
        Self { cfg, index, since: f32::INFINITY, scan: 0 }
    }

    /// Advances by `dt`. Returns a scan when one is due. `ego_vx` is the ego
    /// car's forward speed (to report over-ground velocities, as radars do from
    /// measured range rate plus the ego speed they receive on the bus).
    pub fn step(&mut self, dt: f32, mount: &SensorMount, actors: &ActorList, frame: u64) -> Option<RadarScan> {
        self.since += dt.max(0.0);
        if self.since < self.cfg.period {
            return None;
        }
        self.since = if self.since.is_finite() { self.since - self.cfg.period } else { 0.0 };
        self.since = self.since.min(self.cfg.period);
        self.scan = self.scan.wrapping_add(1);
        let c = self.cfg;
        let seed = c.seed ^ self.index as u64;
        let bit = 1u8 << self.index;
        let mut objects = Vec::new();

        for a in actors.as_slice() {
            let (dx, dy) = (a.x - mount.x, a.y - mount.y);
            let range = dx.hypot(dy);
            let az = dy.atan2(dx) - mount.facing;
            if range > mount.range || range < c.min_range || az.abs() > mount.half_fov {
                continue;
            }
            // Line of sight: the bit must be set and recent (4 refresh periods).
            if a.los_mask & bit == 0 || a.los_age as f32 > 4.0 * mount.refresh_frames.max(1.0) {
                continue;
            }
            let id = a.id as u64;
            if unit(seed, frame, id, 1) < c.p_miss {
                continue;
            }
            // Range, azimuth and range-rate noise, applied in polar form.
            let r_n = range + normal(seed, frame, id, 2) * (c.range_noise_m + c.range_noise_frac * range);
            let az_n = az + normal(seed, frame, id, 3) * c.azimuth_noise_rad + mount.facing;
            let (ux, uy) = (az_n.cos(), az_n.sin());
            // The radar measures the nearest face, about half a length closer.
            let near = (r_n - (a.length * 0.5).max(0.25)).max(c.min_range);
            let (x, y) = (mount.x + near * ux, mount.y + near * uy);
            let v_noise = normal(seed, frame, id, 4) * c.rate_noise;
            let (vx, vy) = (a.vx + v_noise * ux, a.vy + v_noise * uy);
            let speed = a.vx.hypot(a.vy);
            objects.push(RadarObject {
                id: a.id,
                x,
                y,
                vx,
                vy,
                width: a.width.max(0.5),
                half_length: (a.length * 0.5).max(0.25),
                stationary: speed < 0.5,
                clutter: false,
            });
        }

        // Clutter: a stationary return somewhere in the beam now and then.
        if unit(seed, frame, 0, 5) < c.clutter_per_scan {
            let r = 10.0 + unit(seed, frame, 0, 6) * (mount.range - 10.0);
            let az = (unit(seed, frame, 0, 7) * 2.0 - 1.0) * mount.half_fov + mount.facing;
            objects.push(RadarObject {
                id: 0x8000_0000 | (self.scan & 0xFFFF),
                x: mount.x + r * az.cos(),
                y: mount.y + r * az.sin(),
                vx: 0.0,
                vy: 0.0,
                width: 0.5,
                half_length: 0.25,
                stationary: true,
                clutter: true,
            });
        }
        Some(RadarScan { scan: self.scan, objects })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use adapter_api::{Actor, ActorKind};

    fn mount() -> SensorMount {
        SensorMount { x: 2.0, y: 0.0, z: 0.5, facing: 0.0, half_fov: 10f32.to_radians(), range: 160.0, refresh_frames: 3.6, _reserved: 0 }
    }

    fn list(actors: &[Actor]) -> ActorList {
        let mut l = ActorList { count: actors.len() as u32, ..Default::default() };
        l.actors[..actors.len()].copy_from_slice(actors);
        l
    }

    fn car(id: u32, x: f32, y: f32, vx: f32) -> Actor {
        Actor { id, kind: ActorKind::Vehicle, los_mask: 1, los_age: 0, x, y, vx, length: 4.5, width: 1.9, height: 1.5, ..Actor::ZERO }
    }

    #[test]
    fn rate_fov_and_line_of_sight() {
        let mut r = FrontRadar::new(RadarConfig { p_miss: 0.0, clutter_per_scan: 0.0, ..Default::default() }, 0);
        let mut hidden = car(3, 30.0, 0.0, 10.0);
        hidden.los_mask = 0;
        let l = list(&[car(1, 50.0, 0.0, 10.0), car(2, 20.0, 15.0, 0.0), hidden, car(4, 200.0, 0.0, 0.0)]);
        let first = r.step(1.0 / 72.0, &mount(), &l, 1).expect("first scan at once");
        let ids: Vec<u32> = first.objects.iter().map(|o| o.id).collect();
        assert_eq!(ids, vec![1], "only the visible car in the beam and in range");
        // 20 Hz at 72 fps: the next scan comes 3 or 4 frames later.
        let mut frames = 0;
        for f in 2..10 {
            frames += 1;
            if r.step(1.0 / 72.0, &mount(), &l, f).is_some() {
                break;
            }
        }
        assert!((3..=4).contains(&frames), "scan period {frames} frames");
    }

    #[test]
    fn noise_is_bounded_and_unbiased() {
        let mut r = FrontRadar::new(RadarConfig { p_miss: 0.0, clutter_per_scan: 0.0, ..Default::default() }, 0);
        let l = list(&[car(1, 80.0, 0.0, 20.0)]);
        let mut xs = Vec::new();
        for f in 0..4000u64 {
            if let Some(s) = r.step(0.05, &mount(), &l, f) {
                xs.push(s.objects[0].x);
            }
        }
        let mean = xs.iter().sum::<f32>() / xs.len() as f32;
        let sd = (xs.iter().map(|x| (x - mean).powi(2)).sum::<f32>() / xs.len() as f32).sqrt();
        // The reported point is the near face: centre minus half the 4.5 m length.
        assert!((mean - (80.0 - 2.25)).abs() < 0.05, "mean {mean}");
        assert!((sd - (0.1 + 0.005 * 78.0)).abs() < 0.08, "sd {sd}");
    }

    #[test]
    fn misses_happen_at_about_the_configured_rate() {
        let mut r = FrontRadar::new(RadarConfig { p_miss: 0.05, clutter_per_scan: 0.0, ..Default::default() }, 0);
        let l = list(&[car(1, 40.0, 0.0, 10.0)]);
        let (mut scans, mut seen) = (0, 0);
        for f in 0..20_000u64 {
            if let Some(s) = r.step(0.05, &mount(), &l, f) {
                scans += 1;
                seen += s.objects.len();
            }
        }
        let miss = 1.0 - seen as f32 / scans as f32;
        assert!((miss - 0.05).abs() < 0.01, "miss rate {miss}");
    }
}
