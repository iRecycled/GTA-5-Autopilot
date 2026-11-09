//! The ultrasonic sensors (D25): twelve sensors, six in each bumper, each
//! answered by the host as a short fan of rays.
//!
//! A sensor reports the nearest echo over its fan, every 40 ms (25 Hz),
//! through an error model modelled on parking sensors: about 1 cm + 1 %
//! noise, a blind zone below 0.15 m (closer objects read as 0.15 m), a few
//! percent of cycles with no echo (the ECU holds the last echo for up to two
//! cycles, as parking ECUs do) and rare ghost echoes from nothing. A sensor
//! whose fan hasn't been answered recently reports no data (NaN): the
//! degradation matrix treats that as a sensor fault.

use adapter_api::{FanLayout, RangeReport};

use crate::{normal, unit};

pub const ULTRASONIC_COUNT: usize = 12;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UltrasonicConfig {
    /// Update period (s).
    pub period: f32,
    pub noise_m: f32,
    pub noise_frac: f32,
    /// Closer than this, a sensor reads this (m).
    pub min_range: f32,
    /// Probability of no echo from a real object in one cycle.
    pub p_dropout: f32,
    /// Probability of a ghost echo in one cycle, per sensor.
    pub p_ghost: f32,
    /// Cycles a lost echo is held for.
    pub hold_cycles: u32,
    /// A fan older than this (frames) gives no data.
    pub stale_frames: u32,
    pub seed: u64,
}

impl Default for UltrasonicConfig {
    fn default() -> Self {
        Self {
            period: 0.04,
            noise_m: 0.01,
            noise_frac: 0.01,
            min_range: 0.15,
            p_dropout: 0.03,
            p_ghost: 0.003,
            hold_cycles: 2,
            stale_frames: 20,
            seed: 0x0005_0A1C_0B1E,
        }
    }
}

/// One cycle of all twelve sensors (m): infinite = no echo, NaN = no data.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UltrasonicScan {
    pub cycle: u32,
    pub dist: [f32; ULTRASONIC_COUNT],
}

pub struct Ultrasonics {
    cfg: UltrasonicConfig,
    since: f32,
    cycle: u32,
    /// The last real echo per sensor and the cycles since it.
    last: [(f32, u32); ULTRASONIC_COUNT],
}

impl Ultrasonics {
    pub fn new(cfg: UltrasonicConfig) -> Self {
        Self { cfg, since: f32::INFINITY, cycle: 0, last: [(f32::INFINITY, u32::MAX); ULTRASONIC_COUNT] }
    }

    /// Advances by `dt`; returns a cycle when one is due. `fans` is the
    /// layout the core asked for; `ranges` the host's latest answer (None:
    /// nothing answered).
    pub fn step(&mut self, dt: f32, fans: &FanLayout, ranges: Option<&RangeReport>, frame: u64) -> Option<UltrasonicScan> {
        self.since += dt.max(0.0);
        if self.since < self.cfg.period {
            return None;
        }
        self.since = if self.since.is_finite() { (self.since - self.cfg.period).min(self.cfg.period) } else { 0.0 };
        self.cycle = self.cycle.wrapping_add(1);
        let c = self.cfg;
        let mut dist = [f32::NAN; ULTRASONIC_COUNT];
        for (i, f) in fans.as_slice().iter().enumerate().take(ULTRASONIC_COUNT) {
            let Some(res) = ranges.and_then(|r| r.as_slice().get(i)) else { continue };
            let n = (f.rays as usize).clamp(1, adapter_api::MAX_FAN_RAYS);
            let hits = &res.hits[..n];
            if res.age > c.stale_frames || hits.iter().any(|h| h.is_nan()) {
                continue;
            }
            let truth = hits.iter().copied().filter(|h| *h <= f.range).fold(f32::INFINITY, f32::min);
            let key = i as u64;
            let mut d = if truth.is_finite() {
                if unit(c.seed, frame, key, 1) < c.p_dropout {
                    f32::INFINITY
                } else {
                    (truth + normal(c.seed, frame, key, 2) * (c.noise_m + c.noise_frac * truth)).max(c.min_range)
                }
            } else {
                f32::INFINITY
            };
            if d.is_finite() {
                self.last[i] = (d, 0);
            } else {
                // Hold a lost echo for a couple of cycles.
                let (v, age) = self.last[i];
                let age = age.saturating_add(1);
                self.last[i] = (v, age);
                if truth.is_finite() && age <= c.hold_cycles {
                    d = v;
                }
            }
            if !d.is_finite() && unit(c.seed, frame, key, 3) < c.p_ghost {
                d = c.min_range + unit(c.seed, frame, key, 4) * (f.range - c.min_range);
            }
            dist[i] = d;
        }
        Some(UltrasonicScan { cycle: self.cycle, dist })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use adapter_api::{FanResult, RangeFan};

    fn fans() -> FanLayout {
        let mut l = FanLayout { count: 2, ..Default::default() };
        let f = RangeFan { x: 2.3, rays: 3, half_spread: 0.4, range: 4.5, refresh_frames: 5.0, ..Default::default() };
        l.fans[0] = f;
        l.fans[1] = RangeFan { y: 0.5, ..f };
        l
    }

    fn report(a: [f32; 3], b: [f32; 3]) -> RangeReport {
        let mut r = RangeReport { count: 2, ..Default::default() };
        r.fans[0] = FanResult { hits: [a[0], a[1], a[2], f32::NAN], age: 1, _reserved: 0 };
        r.fans[1] = FanResult { hits: [b[0], b[1], b[2], f32::NAN], age: 1, _reserved: 0 };
        r
    }

    #[test]
    fn nearest_echo_with_noise_blind_zone_and_no_data() {
        let mut u = Ultrasonics::new(UltrasonicConfig { p_dropout: 0.0, p_ghost: 0.0, ..Default::default() });
        let inf = f32::INFINITY;
        let r = report([1.2, 0.8, inf], [0.05, inf, inf]);
        let s = u.step(0.04, &fans(), Some(&r), 1).expect("a cycle");
        assert!((s.dist[0] - 0.8).abs() < 0.06, "{}", s.dist[0]);
        assert_eq!(s.dist[1], 0.15, "the blind zone");
        assert!(s.dist[2].is_nan(), "no fan, no data");
        // Too old: no data.
        let mut old = r;
        old.fans[0].age = 100;
        let s = u.step(0.04, &fans(), Some(&old), 2).unwrap();
        assert!(s.dist[0].is_nan());
        let s = u.step(0.04, &fans(), Some(&report([inf; 3], [inf; 3])), 3).unwrap();
        assert!(s.dist[0].is_infinite() && s.dist[1].is_infinite());
        // 25 Hz at 72 fps: two cycles in 6 frames.
        let mut u = Ultrasonics::new(UltrasonicConfig::default());
        let n = (0..6).filter(|&f| u.step(1.0 / 72.0, &fans(), Some(&r), f).is_some()).count();
        assert!((2..=3).contains(&n), "{n} cycles");
    }

    #[test]
    fn dropouts_are_held_and_ghosts_are_rare() {
        let inf = f32::INFINITY;
        let mut u = Ultrasonics::new(UltrasonicConfig { p_dropout: 0.3, p_ghost: 0.0, ..Default::default() });
        let r = report([1.0, inf, inf], [inf; 3]);
        let mut clear = 0;
        for f in 0..2000 {
            if u.step(0.04, &fans(), Some(&r), f).unwrap().dist[0].is_infinite() {
                clear += 1;
            }
        }
        // Lost only when three cycles in a row drop out: 0.3³ ≈ 2.7 %.
        assert!(clear < 120, "{clear} clear cycles of 2000");
        let mut u = Ultrasonics::new(UltrasonicConfig::default());
        let clear_r = report([inf; 3], [inf; 3]);
        let ghosts = (0..10_000).filter(|&f| u.step(0.04, &fans(), Some(&clear_r), f).unwrap().dist[0].is_finite()).count();
        assert!((10..60).contains(&ghosts), "{ghosts} ghosts in 10,000 cycles");
    }
}
