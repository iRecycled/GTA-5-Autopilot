//! Lane features from the camera's lane lines (F7): the lane path, lane
//! centering, lane keeping's nudge (LKA), and the curve speed ACC slows to.
//!
//! - **Lane path:** the centre between the camera's left and right lines;
//!   with only one, that line offset by half the last width seen.
//! - **Lane centering** steers along it by pure pursuit (`control::lateral`).
//!   A frame without lines is bridged for 0.5 s; then the lanes are lost and
//!   the core asks the driver to take over (D11). When the lane narrows below
//!   2.6 m within 4 s of driving, it ends: the driver is asked to take over
//!   then too.
//! - **LKA**, when lane centering isn't on: above 60 km/h, without the turn
//!   signal toward that side, a car about to touch a line (within 0.5 s, or
//!   closer than 0.2 m) is steered back toward the centre, at up to 1.5 m/s²
//!   of lateral acceleration, until it is near the centre and lined up with
//!   the lane again (at most 2 s). The driver's own steering ends it, and
//!   keeps it off for 2 s after.
//! - **Curve speed:** the speed at which the sharpest curvature the camera
//!   sees ahead gives 2 m/s² of lateral acceleration.
//! - **The steering response is learned on line** (as openpilot's `paramsd`
//!   learns the steer ratio): the curvature the car actually drives (yaw rate
//!   ÷ speed) over the curvature its measured road-wheel angle would give on
//!   the kinematic model. Hosts' cars understeer differently (GTA's a lot);
//!   the requests are divided by that gain, so steady bends hold their line.

use control::lateral::{self, LateralConfig};
use sensors::{LaneFit, camera};

use crate::Ego;

/// The lane the car is in, as a centre path.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LanePath {
    /// The centre as a cubic in the ego frame.
    pub c: [f32; 4],
    pub width: f32,
    /// How far ahead it is known (m).
    pub range: f32,
    pub left: Option<LaneFit>,
    pub right: Option<LaneFit>,
}

impl LanePath {
    /// From a camera frame's lines; `width`: the last width seen (m).
    pub fn from_lines(lanes: &[LaneFit; 4], width: f32) -> Option<Self> {
        let (l, r) = (lanes[camera::LEFT], lanes[camera::RIGHT]);
        let l = l.valid.then_some(l);
        let r = r.valid.then_some(r);
        let half = |f: &LaneFit, sign: f32| {
            let mut c = f.c;
            c[0] += sign * 0.5 * width;
            c
        };
        let (c, w, range) = match (l, r) {
            (Some(l), Some(r)) => {
                let c = [0.5 * (l.c[0] + r.c[0]), 0.5 * (l.c[1] + r.c[1]), 0.5 * (l.c[2] + r.c[2]), 0.5 * (l.c[3] + r.c[3])];
                (c, l.c[0] - r.c[0], l.range.min(r.range))
            }
            (Some(l), None) => (half(&l, -1.0), width, l.range),
            (None, Some(r)) => (half(&r, 1.0), width, r.range),
            (None, None) => return None,
        };
        Some(Self { c, width: w, range, left: l, right: r })
    }

    /// The car's offset from the centre (m, left +).
    pub fn offset(&self) -> f32 {
        -self.c[0]
    }

    /// Where the lane narrows below `min_width` (m ahead), if it does
    /// within `within` m.
    pub fn ends_at(&self, min_width: f32, within: f32) -> Option<f32> {
        let (l, r) = (self.left?, self.right?);
        let to = within.min(l.range.min(r.range));
        (0..=16).map(|i| to * i as f32 / 16.0).find(|&x| l.y(x) - r.y(x) < min_width)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LaneConfig {
    pub lateral: LateralConfig,
    /// Lines missing for longer than this: lanes lost (s).
    pub lost_after: f32,
    pub end_width: f32,
    pub end_time: f32,
    /// Lateral acceleration for the curve speed (m/s²).
    pub curve_lat_accel: f32,
    /// LKA: minimum speed, time to the line, distance to the line, longest nudge.
    pub lka_speed: f32,
    pub lka_ttlc: f32,
    pub lka_margin: f32,
    pub lka_time: f32,
    /// A nudge's lateral acceleration cap (m/s²): firm, not a swerve.
    pub lka_lat_accel: f32,
}

impl Default for LaneConfig {
    fn default() -> Self {
        Self {
            lateral: LateralConfig::default(),
            lost_after: 0.5,
            end_width: 2.6,
            end_time: 4.0,
            curve_lat_accel: 2.0,
            lka_speed: 60.0 / 3.6,
            lka_ttlc: 0.5,
            lka_margin: 0.2,
            lka_time: 2.0,
            lka_lat_accel: 1.5,
        }
    }
}

/// Why lane centering can't steer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LaneIssue {
    NoLanes,
    LaneEnds,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LaneOut {
    /// The road-wheel angle lane centering asks for, when it can steer.
    pub angle: Option<f32>,
    pub issue: Option<LaneIssue>,
    /// LKA's nudge (road-wheel angle), when not centering.
    pub nudge: Option<f32>,
    /// The curve speed (m/s; infinite with no curve or no lanes).
    pub curve_speed: f32,
    pub left_seen: bool,
    pub right_seen: bool,
    pub path: Option<LanePath>,
}

#[derive(Clone, Debug)]
pub struct Lanes {
    pub cfg: LaneConfig,
    path: Option<LanePath>,
    /// Seconds since the camera last gave a line.
    unseen: f32,
    width: f32,
    nudge_for: Option<f32>,
    /// Seconds since the driver last steered.
    since_driver: f32,
    /// Driven curvature ÷ the kinematic curvature of the wheel angle.
    pub steer_gain: f32,
    /// LKA nudges so far.
    pub nudges: u32,
}

impl Default for Lanes {
    fn default() -> Self {
        Self::new(LaneConfig::default())
    }
}

impl Lanes {
    /// Learns the steering response from what the car did: its measured
    /// road-wheel angle (rad), yaw rate (rad/s) and speed (m/s). Only while
    /// turning steadily enough to measure, forwards.
    pub fn learn(&mut self, dt: f32, wheel_angle: f32, yaw_rate: f32, speed: f32, wheelbase: f32) {
        if !(wheel_angle.is_finite() && yaw_rate.is_finite()) || speed < 8.0 {
            return;
        }
        let model = wheel_angle.tan() / wheelbase;
        if model.abs() < 0.002 || model.signum() != yaw_rate.signum() {
            return;
        }
        let g = ((yaw_rate / speed) / model).clamp(0.3, 2.0);
        self.steer_gain += (g - self.steer_gain) * (dt / 3.0).min(1.0);
    }

    pub fn new(cfg: LaneConfig) -> Self {
        Self { cfg, path: None, unseen: f32::INFINITY, width: 3.6, nudge_for: None, since_driver: f32::INFINITY, steer_gain: 1.0, nudges: 0 }
    }

    /// One step. `lines`: a camera frame that arrived this step. `centering`:
    /// lane centering is engaged. `driver_steering`: the driver is steering.
    pub fn step(&mut self, dt: f32, lines: Option<&[LaneFit; 4]>, ego: &Ego, wheelbase: f32, centering: bool, driver_steering: bool) -> LaneOut {
        let c = self.cfg;
        self.since_driver = if driver_steering { 0.0 } else { self.since_driver + dt };
        if let Some(l) = lines {
            match LanePath::from_lines(l, self.width) {
                Some(p) => {
                    if p.left.is_some() && p.right.is_some() && (2.5..6.0).contains(&p.width) {
                        self.width += 0.05 * (p.width - self.width);
                    }
                    self.path = Some(p);
                    self.unseen = 0.0;
                }
                None => self.unseen += dt,
            }
        } else {
            self.unseen += dt;
        }
        if self.unseen > c.lost_after {
            self.path = None;
        }
        let mut out = LaneOut { curve_speed: f32::INFINITY, ..Default::default() };
        let Some(p) = self.path else {
            out.issue = Some(LaneIssue::NoLanes);
            self.nudge_for = None;
            return out;
        };
        out.path = Some(p);
        out.left_seen = p.left.is_some();
        out.right_seen = p.right.is_some();
        let v = ego.speed;
        let k_max = lateral::max_curvature(&p.c, 0.0, p.range.min((4.0 * v).max(30.0)));
        if k_max > 1e-4 {
            out.curve_speed = (c.curve_lat_accel / k_max).sqrt();
        }
        let g = self.steer_gain;
        let steer = || lateral::angle_for(lateral::pursuit_curvature(&c.lateral, &p.c, p.range, v) / g, wheelbase);
        let nudge_cfg = LateralConfig { max_lat_accel: c.lka_lat_accel, ..c.lateral };
        let nudge = || lateral::angle_for(lateral::pursuit_curvature(&nudge_cfg, &p.c, p.range, v) / g, wheelbase);
        if p.ends_at(c.end_width, (c.end_time * v).max(20.0)).is_some() {
            out.issue = Some(LaneIssue::LaneEnds);
        }
        if centering {
            self.nudge_for = None;
            out.angle = Some(steer());
            return out;
        }

        // LKA: about to touch a line?
        let half = ego.half_width();
        let side = |f: Option<LaneFit>, left: bool| -> bool {
            let Some(f) = f else { return false };
            // Room from the car's side to the line, and how fast it closes.
            let room = if left { f.c[0] - half } else { -f.c[0] - half };
            let closing = if left { -f.c[1] * v } else { f.c[1] * v };
            let signalling = if left { ego.turn_left } else { ego.turn_right };
            !signalling && (room < c.lka_margin && closing > 0.0 || closing > 0.05 && room / closing < c.lka_ttlc)
        };
        let drifting = v >= c.lka_speed && self.since_driver > 2.0 && (side(p.left, true) || side(p.right, false));
        match self.nudge_for {
            Some(t) if t < c.lka_time && !driver_steering && (p.offset().abs() > 0.3 || p.c[1].abs() > 0.004) => {
                self.nudge_for = Some(t + dt);
                out.nudge = Some(nudge());
            }
            Some(_) if !drifting => self.nudge_for = None,
            None if drifting => {
                self.nudge_for = Some(0.0);
                self.nudges += 1;
                out.nudge = Some(nudge());
            }
            _ => {}
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn line(c0: f32, c1: f32, c2: f32) -> LaneFit {
        LaneFit { valid: true, c: [c0, c1, c2, 0.0], range: 80.0, kind: 1, quality: 3 }
    }

    fn frame(l: Option<LaneFit>, r: Option<LaneFit>) -> [LaneFit; 4] {
        let mut f = [LaneFit::default(); 4];
        if let Some(l) = l {
            f[camera::LEFT] = l;
        }
        if let Some(r) = r {
            f[camera::RIGHT] = r;
        }
        f
    }

    fn ego(speed: f32) -> Ego {
        Ego { speed, ..Default::default() }
    }

    #[test]
    fn the_path_with_two_lines_one_line_and_a_lane_that_ends() {
        let p = LanePath::from_lines(&frame(Some(line(2.0, 0.0, 0.001)), Some(line(-1.6, 0.0, 0.001))), 3.6).unwrap();
        assert!((p.c[0] - 0.2).abs() < 1e-6 && (p.width - 3.6).abs() < 1e-6 && (p.offset() + 0.2).abs() < 1e-6);
        let p = LanePath::from_lines(&frame(None, Some(line(-1.6, 0.0, 0.0))), 3.6).unwrap();
        assert!((p.c[0] - 0.2).abs() < 1e-6, "one line: half a width in");
        assert!(LanePath::from_lines(&frame(None, None), 3.6).is_none());
        // The right line sweeps left: 3.6 m wide here, 2.0 m at 60 m.
        let p = LanePath::from_lines(&frame(Some(line(1.8, 0.0, 0.0)), Some(line(-1.8, 0.0, 1.6 / 3600.0))), 3.6).unwrap();
        let x = p.ends_at(2.6, 100.0).expect("ends");
        assert!((47.0..=53.0).contains(&x), "narrower than 2.6 m from {x} m (47.4 m, sampled every 6.25 m)");
    }

    #[test]
    fn centering_lost_lanes_and_curve_speed() {
        let mut lanes = Lanes::default();
        // Left of centre (lines at +1.4 and -2.2): steer right; a 250 m bend.
        let f = frame(Some(line(1.4, 0.0, 0.002)), Some(line(-2.2, 0.0, 0.002)));
        let o = lanes.step(0.033, Some(&f), &ego(25.0), 2.7, true, false);
        assert!(o.issue.is_none() && o.left_seen && o.right_seen);
        assert!(o.angle.is_some());
        assert!((o.curve_speed - (2.0f32 * 250.0).sqrt()).abs() < 0.5, "curve speed {}", o.curve_speed);
        // Lines gone: bridged for 0.5 s, then lost.
        let held = (0..14).filter(|_| lanes.step(0.033, None, &ego(25.0), 2.7, true, false).angle.is_some()).count();
        assert!((14..=15).contains(&held), "{held}");
        let gone = (0..20).filter(|_| lanes.step(0.033, None, &ego(25.0), 2.7, true, false).issue == Some(LaneIssue::NoLanes)).count();
        assert!(gone >= 15);
    }

    #[test]
    fn the_steering_response_is_learned() {
        let mut lanes = Lanes::default();
        // A car that drives 60 % of the kinematic curvature.
        for _ in 0..2000 {
            let angle = 0.02f32;
            let yaw = 0.6 * 20.0 * angle.tan() / 2.7;
            lanes.learn(0.014, angle, yaw, 20.0, 2.7);
        }
        assert!((lanes.steer_gain - 0.6).abs() < 0.01, "{}", lanes.steer_gain);
        // Straight ahead, or slow: nothing to learn.
        lanes.learn(0.014, 0.0001, 0.0, 20.0, 2.7);
        lanes.learn(0.014, 0.05, 0.0, 3.0, 2.7);
        assert!((lanes.steer_gain - 0.6).abs() < 0.01);
    }

    #[test]
    fn lka_nudges_back_from_a_line_unless_signalling() {
        let mut lanes = Lanes::default();
        // Drifting left at 0.3 m/s (heading 0.01 rad at 30 m/s), 0.15 m from the line.
        let f = frame(Some(line(1.1, -0.01, 0.0)), Some(line(-2.5, -0.01, 0.0)));
        let o = lanes.step(0.033, Some(&f), &ego(30.0), 2.7, false, false);
        let a = o.nudge.expect("a nudge");
        assert!(a < 0.0, "steers right: {a}");
        assert_eq!(lanes.nudges, 1);
        // Signalling left: no nudge.
        let mut lanes = Lanes::default();
        let e = Ego { turn_left: true, ..ego(30.0) };
        assert!(lanes.step(0.033, Some(&f), &e, 2.7, false, false).nudge.is_none());
        // Too slow, or the driver steering: none.
        let mut lanes = Lanes::default();
        assert!(lanes.step(0.033, Some(&f), &ego(10.0), 2.7, false, false).nudge.is_none());
        assert!(lanes.step(0.033, Some(&f), &ego(30.0), 2.7, false, true).nudge.is_none());
        // ... nor within 2 s after the driver steered.
        let held = (0..90).filter(|_| lanes.step(0.033, Some(&f), &ego(30.0), 2.7, false, false).nudge.is_none()).count();
        assert!((60..=62).contains(&held), "{held}");
        // Centred and straight: none.
        let mut lanes = Lanes::default();
        let c = frame(Some(line(1.8, 0.0, 0.0)), Some(line(-1.8, 0.0, 0.0)));
        assert!(lanes.step(0.033, Some(&c), &ego(30.0), 2.7, false, false).nudge.is_none());
    }
}
