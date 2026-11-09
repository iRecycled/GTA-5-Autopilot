//! Park assist (F6): distances, tone and low-speed automatic braking from the
//! twelve ultrasonic sensors.
//!
//! Sensor order: front bumper left corner → right corner (0–5), rear bumper
//! left corner → right corner (6–11). The four middle sensors of a bumper
//! cover the car's path; the corner sensors only show and sound.
//!
//! - **Display:** below 10 km/h, whenever reversing, or going forward with
//!   something within 1.2 m of the front.
//! - **Tone:** for the bumper in the direction of travel. It starts at 1.5 m
//!   behind (1.0 m in front), beeps faster as the gap closes and is
//!   continuous from 0.3 m.
//! - **Braking:** moving toward an obstacle in the path below 10 km/h, it
//!   brakes when the gap is about what the car needs to stop (its braking
//!   distance plus a reaction allowance) plus 0.45 m. An echo counts only
//!   after two cycles in a row agree, so a ghost echo doesn't brake. After a
//!   stop it holds for 2 s, or until the driver brakes, and then lets the
//!   driver creep closer: it brakes again only after a gear change or once
//!   the gap has opened by 0.3 m.

use crate::Ego;

pub const PATH_FRONT: [usize; 4] = [1, 2, 3, 4];
pub const PATH_REAR: [usize; 4] = [7, 8, 9, 10];

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ParkConfig {
    pub max_speed: f32,
    pub show_front: f32,
    pub tone_far_rear: f32,
    pub tone_far_front: f32,
    pub tone_continuous: f32,
    pub tone_max_hz: f32,
    /// Deceleration the braking distance assumes (m/s²), and the most it
    /// requests.
    pub decel: f32,
    pub decel_max: f32,
    /// Reaction allowance in the stopping distance (s).
    pub latency: f32,
    /// Gap kept when braking (m).
    pub stop_gap: f32,
    pub hold_time: f32,
    pub rearm: f32,
    /// Two echoes agree when within this (m).
    pub agree: f32,
}

impl Default for ParkConfig {
    fn default() -> Self {
        Self {
            max_speed: 10.0 / 3.6,
            show_front: 1.2,
            tone_far_rear: 1.5,
            tone_far_front: 1.0,
            tone_continuous: 0.3,
            tone_max_hz: 8.0,
            decel: 4.0,
            decel_max: 8.0,
            latency: 0.25,
            stop_gap: 0.45,
            hold_time: 2.0,
            rearm: 0.3,
            agree: 0.3,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ParkOut {
    /// Show the distances.
    pub active: bool,
    /// The latest distances (m; infinite = clear, NaN = no data).
    pub dist: [f32; 12],
    /// Beeps per second (0 = silent, infinite = continuous).
    pub tone: f32,
    pub decel: f32,
    pub hold: bool,
    /// A sensor gives no data.
    pub fault: bool,
}

impl Default for ParkOut {
    fn default() -> Self {
        Self { active: false, dist: [f32::NAN; 12], tone: 0.0, decel: 0.0, hold: false, fault: false }
    }
}

#[derive(Clone, Debug)]
pub struct ParkAssist {
    pub cfg: ParkConfig,
    dist: [f32; 12],
    /// Per sensor: the echo two cycles agreed on (infinite = none).
    confirmed: [f32; 12],
    braking: bool,
    held: Option<f32>,
    /// After a stop: (reverse?, gap at the stop) until re-armed.
    inhibit: Option<(bool, f32)>,
    pub activations: u32,
}

impl Default for ParkAssist {
    fn default() -> Self {
        Self::new(ParkConfig::default())
    }
}

fn tone(c: &ParkConfig, d: f32, far: f32) -> f32 {
    if d.is_nan() || d >= far {
        0.0
    } else if d <= c.tone_continuous {
        f32::INFINITY
    } else {
        1.0 + (c.tone_max_hz - 1.0) * (far - d) / (far - c.tone_continuous)
    }
}

impl ParkAssist {
    pub fn new(cfg: ParkConfig) -> Self {
        Self { cfg, dist: [f32::NAN; 12], confirmed: [f32::INFINITY; 12], braking: false, held: None, inhibit: None, activations: 0 }
    }

    /// `cycle`: a new ultrasonic cycle that arrived this step.
    pub fn step(&mut self, dt: f32, ego: &Ego, cycle: Option<&[f32; 12]>, driver_brake: f32) -> ParkOut {
        let c = self.cfg;
        if let Some(d) = cycle {
            for ((conf, &prev), &now) in self.confirmed.iter_mut().zip(&self.dist).zip(d) {
                let agree = now.is_finite() && prev.is_finite() && (now - prev).abs() <= c.agree;
                *conf = if agree { now } else { f32::INFINITY };
            }
            self.dist = *d;
        }
        let mut out = ParkOut { dist: self.dist, fault: self.dist.iter().any(|d| d.is_nan()), ..Default::default() };
        let slow = ego.speed <= c.max_speed;
        let front_min = self.dist[..6].iter().copied().filter(|d| !d.is_nan()).fold(f32::INFINITY, f32::min);
        let rear_min = self.dist[6..].iter().copied().filter(|d| !d.is_nan()).fold(f32::INFINITY, f32::min);
        out.active = slow && (ego.reverse || front_min < c.show_front);
        if out.active {
            out.tone = if ego.reverse { tone(&c, rear_min, c.tone_far_rear) } else { tone(&c, front_min, c.tone_far_front) };
        }

        // Braking, in the direction of travel.
        let path = if ego.reverse { PATH_REAR } else { PATH_FRONT };
        let gap = path.iter().map(|&i| self.confirmed[i]).fold(f32::INFINITY, f32::min);
        if let Some((rev, at)) = self.inhibit
            && (rev != ego.reverse || gap > at + c.rearm)
        {
            self.inhibit = None;
        }
        if let Some(h) = self.held.as_mut() {
            *h += dt;
            if *h >= c.hold_time || driver_brake > 0.1 {
                self.held = None;
            } else {
                out.hold = true;
                return out;
            }
        }
        let v = ego.speed;
        let need = c.stop_gap + v * v / (2.0 * c.decel) + v * c.latency;
        let threat = slow && v > 0.05 && self.inhibit.is_none() && gap <= need;
        if threat && !self.braking {
            self.activations += 1;
        }
        if threat || (self.braking && v > 0.05) {
            self.braking = true;
            // Enough to stop short of the gap kept, within limits.
            let room = (gap - 0.8 * c.stop_gap).max(0.05);
            out.decel = (v * v / (2.0 * room)).clamp(c.decel, c.decel_max);
        } else if self.braking {
            self.braking = false;
            self.held = Some(0.0);
            self.inhibit = Some((ego.reverse, gap));
            out.hold = true;
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cycle_with(i: usize, d: f32) -> [f32; 12] {
        let mut c = [f32::INFINITY; 12];
        c[i] = d;
        c
    }

    fn reversing(speed: f32) -> Ego {
        Ego { speed, reverse: true, ..Default::default() }
    }

    #[test]
    fn tone_rises_as_the_gap_closes() {
        let c = ParkConfig::default();
        assert_eq!(tone(&c, 2.0, 1.5), 0.0);
        assert!((tone(&c, 1.5 - 1e-3, 1.5) - 1.0).abs() < 0.01);
        assert!(tone(&c, 0.9, 1.5) > tone(&c, 1.2, 1.5));
        assert!(tone(&c, 0.25, 1.5).is_infinite());
        let mut p = ParkAssist::default();
        let o = p.step(0.04, &reversing(0.5), Some(&cycle_with(8, 0.8)), 0.0);
        assert!(o.active && o.tone > 3.0 && o.tone.is_finite(), "{o:?}");
        // Forward with nothing near the front: no display.
        assert!(!p.step(0.04, &Ego { speed: 1.0, ..Default::default() }, Some(&cycle_with(8, 0.8)), 0.0).active);
    }

    #[test]
    fn brakes_short_of_a_wall_and_lets_the_driver_creep() {
        let mut p = ParkAssist::default();
        let e = reversing(1.5);
        // A wall 3 m behind: no braking yet (needs about 1.1 m at 1.5 m/s).
        assert_eq!(p.step(0.04, &e, Some(&cycle_with(8, 3.0)), 0.0).decel, 0.0);
        // A single ghost echo at 0.6 m doesn't brake.
        assert_eq!(p.step(0.04, &e, Some(&cycle_with(9, 0.6)), 0.0).decel, 0.0);
        assert_eq!(p.step(0.04, &e, Some(&cycle_with(8, 1.05)), 0.0).decel, 0.0, "one cycle at 1.05 m");
        let o = p.step(0.04, &e, Some(&cycle_with(8, 1.0)), 0.0);
        assert!(o.decel >= 4.0, "two agreeing cycles: brake ({o:?})");
        assert!(p.step(0.04, &reversing(0.5), None, 0.0).decel > 0.0, "until stopped");
        assert!(p.step(0.04, &reversing(0.0), None, 0.0).hold);
        // The driver brakes: hold ends; then creeping closer doesn't brake.
        assert!(!p.step(0.04, &reversing(0.0), None, 0.5).hold);
        let o = p.step(0.04, &reversing(0.3), Some(&cycle_with(8, 0.7)), 0.0);
        assert_eq!((o.decel, o.hold), (0.0, false));
        assert_eq!(p.activations, 1);
        // Going forward toward the front: armed again (front sensor 2).
        let fwd = Ego { speed: 1.0, ..Default::default() };
        p.step(0.04, &fwd, Some(&cycle_with(2, 0.7)), 0.0);
        assert!(p.step(0.04, &fwd, Some(&cycle_with(2, 0.7)), 0.0).decel > 0.0);
        // Corner sensors never brake.
        let mut p = ParkAssist::default();
        p.step(0.04, &e, Some(&cycle_with(6, 0.5)), 0.0);
        assert_eq!(p.step(0.04, &e, Some(&cycle_with(6, 0.5)), 0.0).decel, 0.0);
    }
}
