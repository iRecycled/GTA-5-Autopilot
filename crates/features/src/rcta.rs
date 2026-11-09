//! Rear cross-traffic alert and braking (F5).
//!
//! Active while reversing below 10 km/h. A vehicle crossing behind the car
//! sweeps a strip across the car's path, as deep as the vehicle is wide. For
//! each one it predicts when the vehicle is in the car's path (the car's width
//! plus a margin), and whether, if the car keeps reversing, the car will be in
//! that strip then (the car's own reversing is part of the relative motion):
//! - **warn** (icon on that side, chime) if the vehicle reaches the path
//!   within 4 s and will pass the car, or pass within 6 m of the bumper;
//! - **brake** if the car would be in the vehicle's strip while it passes,
//!   and the bumper is now as close to the strip as the car needs to stop
//!   (its stopping distance plus 0.8 m): the last moment to stop short of
//!   its lane, not of the vehicle. The brake stops the car, holds it 1.5 s
//!   (or until the driver brakes), then hands back.

use fusion::SurroundTrack;

use crate::Ego;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RctaConfig {
    pub max_speed: f32,
    pub warn_horizon: f32,
    pub warn_depth: f32,
    pub brake_horizon: f32,
    pub brake_margin: f32,
    /// Lateral reach (m) and the slowest crossing speed (m/s).
    pub lateral_range: f32,
    pub min_cross_speed: f32,
    /// Added to the car's half width for its path (m).
    pub path_margin: f32,
    pub decel: f32,
    /// The brake's reaction allowance in the stopping distance (s).
    pub latency: f32,
    pub hold_time: f32,
}

impl Default for RctaConfig {
    fn default() -> Self {
        Self {
            max_speed: 10.0 / 3.6,
            warn_horizon: 4.0,
            warn_depth: 6.0,
            brake_horizon: 5.0,
            brake_margin: 0.8,
            lateral_range: 40.0,
            min_cross_speed: 1.0,
            path_margin: 0.4,
            decel: 5.0,
            latency: 0.2,
            hold_time: 1.5,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RctaOut {
    pub active: bool,
    pub left: bool,
    pub right: bool,
    /// Deceleration request (m/s², 0 = none).
    pub decel: f32,
    pub hold: bool,
}

/// One crossing vehicle's prediction.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Crossing {
    /// 0 from the left, 1 from the right.
    pub side: usize,
    pub warn: bool,
    pub brake: bool,
}

pub fn crossing(c: &RctaConfig, ego: &Ego, t: &SurroundTrack) -> Option<Crossing> {
    if !t.confirmed || t.x() > ego.rear + 1.0 || t.y().abs() > c.lateral_range || t.vy().abs() < c.min_cross_speed {
        return None;
    }
    let hw = ego.half_width() + c.path_margin;
    let (y, vy) = (t.y(), t.vy());
    let (hx, hy) = t.extent();
    // Moving toward the path, or already in it.
    if y.abs() - hy > hw && y * vy > 0.0 {
        return None;
    }
    // In the path from t_in to t_out.
    let t_in = ((y.abs() - hy - hw) / vy.abs()).max(0.0);
    let t_out = (y.abs() + hy + hw) / vy.abs();
    // Its strip, relative to the car: from its near face (toward the car)
    // to its far face; the car reversing brings it closer.
    let rel_vx = t.vx() - ego.vx();
    let near_at = |s: f32| t.x() + hx + rel_vx * s;
    let width = 2.0 * hx;
    // If the car keeps going, is it in the strip while the vehicle passes?
    let conflict = near_at(t_out) >= ego.rear - 0.3 && near_at(t_in) - width <= ego.front;
    let side = if y > 0.0 { 0 } else { 1 };
    let warn = t_in <= c.warn_horizon && (conflict || near_at(t_in) >= ego.rear - c.warn_depth);
    let v = ego.speed;
    let need = c.brake_margin + v * v / (2.0 * c.decel) + v * c.latency;
    let gap = ego.rear - (t.x() + hx);
    let brake = v > 0.2 && t_in <= c.brake_horizon && conflict && gap <= need;
    (warn || brake).then_some(Crossing { side, warn: warn || brake, brake })
}

#[derive(Clone, Debug, Default)]
pub struct Rcta {
    pub cfg: RctaConfig,
    braking: bool,
    /// Seconds held at a standstill after a stop.
    held: Option<f32>,
    /// Brake activations so far (for scoring).
    pub activations: u32,
}

impl Rcta {
    pub fn new(cfg: RctaConfig) -> Self {
        Self { cfg, ..Default::default() }
    }

    pub fn step(&mut self, dt: f32, ego: &Ego, tracks: &[SurroundTrack], driver_brake: f32) -> RctaOut {
        let c = self.cfg;
        let active = ego.reverse && ego.speed <= c.max_speed;
        let mut out = RctaOut { active, ..Default::default() };
        let mut brake = false;
        if active {
            for t in tracks {
                if let Some(x) = crossing(&c, ego, t) {
                    if x.side == 0 {
                        out.left = true;
                    } else {
                        out.right = true;
                    }
                    brake |= x.brake;
                }
            }
        }
        if let Some(h) = self.held.as_mut() {
            *h += dt;
            if *h >= c.hold_time || driver_brake > 0.1 || !ego.reverse {
                self.held = None;
            } else {
                out.hold = true;
                return out;
            }
        }
        if brake && !self.braking {
            self.activations += 1;
        }
        if brake || (self.braking && ego.speed > 0.1 && ego.reverse) {
            self.braking = true;
            out.decel = c.decel;
        } else if self.braking {
            self.braking = false;
            if ego.speed <= 0.1 {
                self.held = Some(0.0);
                out.hold = true;
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fusion::{SurroundConfig, SurroundTracker};
    use sensors::{RadarObject, RadarScan};

    fn track(x: f32, y: f32, vx: f32, vy: f32) -> Vec<SurroundTrack> {
        let mut t = SurroundTracker::new(SurroundConfig::default());
        for i in 0..4 {
            let o = RadarObject { id: 1, x, y, vx, vy, width: 1.9, half_length: 2.25, stationary: false, clutter: false };
            t.update(3, &RadarScan { scan: i, objects: vec![o] });
        }
        t.tracks().to_vec()
    }

    fn reversing(speed: f32) -> Ego {
        Ego { speed, reverse: true, ..Default::default() }
    }

    #[test]
    fn warns_then_brakes_for_crossing_traffic() {
        let c = RctaConfig::default();
        // A car whose near side is 4 m behind the bumper, 15 m to the left, crossing right at 8 m/s.
        let far = &track(-7.3, 15.0, 0.0, -8.0)[0];
        let x = crossing(&c, &reversing(1.5), far).expect("crossing");
        assert!(x.side == 0 && x.warn && !x.brake, "{x:?}");
        // 4.5 m back at 1.5 m/s: warn, there is still room to stop.
        assert!(!crossing(&c, &reversing(1.5), &track(-5.45, 6.0, 0.0, -8.0)[0]).unwrap().brake);
        // 3.5 m back (1.15 m from the bumper to its lane): brake now.
        let near = &track(-4.45, 6.0, 0.0, -8.0)[0];
        assert!(crossing(&c, &reversing(1.5), near).unwrap().brake);
        // Standing still: warn only.
        let x = crossing(&c, &reversing(0.0), near).unwrap();
        assert!(x.warn && !x.brake);
        // Driving away from the path, or along it: nothing.
        assert!(crossing(&c, &reversing(1.5), &track(-5.45, 6.0, 0.0, 8.0)[0]).is_none());
        assert!(crossing(&c, &reversing(1.5), &track(-8.0, 0.0, -3.0, 0.0)[0]).is_none());
        // Passing far behind (12 m): no warning.
        assert!(crossing(&c, &reversing(1.0), &track(-14.95, 10.0, 0.0, -8.0)[0]).is_none());
    }

    #[test]
    fn brakes_to_a_stop_holds_and_hands_back() {
        let mut r = Rcta::new(RctaConfig::default());
        let near = track(-4.45, 6.0, 0.0, -8.0);
        let o = r.step(0.05, &reversing(1.5), &near, 0.0);
        assert!(o.active && o.left && o.decel > 0.0);
        // Keeps braking until stopped, even once the car has passed.
        assert!(r.step(0.05, &reversing(0.8), &[], 0.0).decel > 0.0);
        let o = r.step(0.05, &reversing(0.0), &[], 0.0);
        assert!(o.hold && o.decel == 0.0);
        let held = (0..40).filter(|_| r.step(0.05, &reversing(0.0), &[], 0.0).hold).count();
        assert!((27..=30).contains(&held), "held {held} steps");
        assert_eq!(r.activations, 1);
        // Inactive going forward or too fast.
        assert!(!r.step(0.05, &Ego { speed: 1.0, ..Default::default() }, &near, 0.0).active);
        assert!(!r.step(0.05, &reversing(4.0), &near, 0.0).active);
    }
}
