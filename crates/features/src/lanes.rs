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
    /// The curvature loop: how fast (1/s) the steering scales toward
    /// driving the curvature asked for, the most it may scale either way, and
    /// the smallest curvature it measures by (1/m).
    pub curvature_rate: f32,
    pub curvature_scale_max: f32,
    pub curvature_min: f32,
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
            curvature_rate: 1.0,
            curvature_scale_max: 4.0,
            curvature_min: 2.5e-4,
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

/// Speed bands of the learned steering response (m/s): 0, 5, ... 55.
const BAND_MS: f32 = 5.0;
const BANDS: usize = 12;

/// How much the car turns for its wheel angle, by speed: driven curvature ÷
/// the kinematic (bicycle) curvature. A car understeers more the faster it
/// goes (GTA's Sultan: 0.75 at 35 km/h, 0.27 at 80, 0.15 at 120), so one
/// number learned at one speed steers wrongly at another: a manual GTA drive
/// weaved out of its lane at 35 km/h with the gain learned at 80.
///
/// Each band learns from steady turning near its speed. A band not learned
/// yet takes the understeer curve `1 / (1 + (v / v_c)²)` fitted to the
/// bands that are, or 1 with none.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SteerResponse {
    gain: [f32; BANDS],
    /// How much each band has learned (0 = nothing, 1 = settled).
    weight: [f32; BANDS],
    /// The wheel angle, smoothed: learning waits for it to hold steady, as
    /// the yaw follows the wheels about 0.1 s late.
    steady: f32,
}

impl Default for SteerResponse {
    fn default() -> Self {
        Self { gain: [1.0; BANDS], weight: [0.0; BANDS], steady: 0.0 }
    }
}

impl SteerResponse {
    /// Learning time constant (s), and how long a band takes to count as
    /// learned (s of steady turning).
    const TAU: f32 = 3.0;
    const SETTLE: f32 = 5.0;

    /// The response at speed `v` (m/s).
    pub fn at(&self, v: f32) -> f32 {
        let f = (v.max(0.0) / BAND_MS).min((BANDS - 1) as f32);
        let i = (f as usize).min(BANDS - 2);
        let w = f - i as f32;
        let model = self.model(v);
        let band = |b: usize| self.gain[b] * self.weight[b] + model * (1.0 - self.weight[b]);
        band(i) * (1.0 - w) + band(i + 1) * w
    }

    /// The understeer curve fitted to the learned bands (1 with none).
    fn model(&self, v: f32) -> f32 {
        let (mut sum, mut n) = (0.0f32, 0.0f32);
        for b in 1..BANDS {
            let (g, wt) = (self.gain[b], self.weight[b]);
            if wt > 0.2 && g < 0.97 {
                // 1/g - 1 = (v/v_c)², so 1/v_c² = (1/g - 1) / v².
                let vb = b as f32 * BAND_MS;
                sum += wt * (1.0 / g - 1.0) / (vb * vb);
                n += wt;
            }
        }
        if n == 0.0 { 1.0 } else { 1.0 / (1.0 + v * v * sum / n) }
    }

    /// Learns from the measured road-wheel angle (rad), yaw rate (rad/s)
    /// and speed (m/s): only while turning forwards, enough to measure, with
    /// the wheel angle steady.
    pub fn learn(&mut self, dt: f32, wheel_angle: f32, yaw_rate: f32, speed: f32, wheelbase: f32) {
        if !(wheel_angle.is_finite() && yaw_rate.is_finite() && dt > 0.0) {
            return;
        }
        let held = (wheel_angle - self.steady).abs() <= 0.15 * wheel_angle.abs() + 0.002;
        self.steady += (wheel_angle - self.steady) * (dt / 0.5).min(1.0);
        let model = wheel_angle.tan() / wheelbase;
        if !held || speed < 5.0 || model.abs() < 0.002 || model.signum() != yaw_rate.signum() {
            return;
        }
        let g = ((yaw_rate / speed) / model).clamp(0.08, 2.0);
        let f = (speed / BAND_MS).min((BANDS - 1) as f32);
        let i = (f as usize).min(BANDS - 2);
        let w = f - i as f32;
        for (b, share) in [(i, 1.0 - w), (i + 1, w)] {
            if share <= 0.0 {
                continue;
            }
            // A band starts from what the curve says, not from 1.
            if self.weight[b] == 0.0 {
                self.gain[b] = self.model(b as f32 * BAND_MS);
            }
            self.gain[b] += (g - self.gain[b]) * (share * dt / Self::TAU).min(1.0);
            self.weight[b] = (self.weight[b] + share * dt / Self::SETTLE).min(1.0);
        }
    }
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
    /// Driven curvature ÷ the kinematic curvature of the wheel angle, by speed.
    pub response: SteerResponse,
    /// LKA nudges so far.
    pub nudges: u32,
    /// The curvature the car drives (yaw rate ÷ speed, from the bus), and
    /// the curvature loop's scale on the steering (log).
    driven: Option<f32>,
    correction: f32,
}

impl Default for Lanes {
    fn default() -> Self {
        Self::new(LaneConfig::default())
    }
}

impl Lanes {
    /// Learns the steering response from what the car did (see
    /// [`SteerResponse::learn`]).
    pub fn learn(&mut self, dt: f32, wheel_angle: f32, yaw_rate: f32, speed: f32, wheelbase: f32) {
        self.driven = (speed > 2.0 && yaw_rate.is_finite()).then(|| yaw_rate / speed);
        self.response.learn(dt, wheel_angle, yaw_rate, speed, wheelbase);
    }

    /// The curvature loop's scale on the steering now.
    pub fn curvature_correction(&self) -> f32 {
        self.correction.exp()
    }

    pub fn new(cfg: LaneConfig) -> Self {
        Self { cfg, path: None, unseen: f32::INFINITY, width: 3.6, nudge_for: None, since_driver: f32::INFINITY, response: SteerResponse::default(), nudges: 0, driven: None, correction: 0.0 }
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
        let g = self.response.at(v);
        let steer = || lateral::angle_for(lateral::pursuit_curvature(&c.lateral, &p.c, p.range, v) / g, wheelbase);
        let nudge_cfg = LateralConfig { max_lat_accel: c.lka_lat_accel, ..c.lateral };
        let nudge = || lateral::angle_for(lateral::pursuit_curvature(&nudge_cfg, &p.c, p.range, v) / g, wheelbase);
        if p.ends_at(c.end_width, (c.end_time * v).max(20.0)).is_some() {
            out.issue = Some(LaneIssue::LaneEnds);
        }
        if centering && !driver_steering {
            self.nudge_for = None;
            // Pure pursuit asks for a curvature; the learned response turns
            // it into a wheel angle; the curvature loop scales that by what
            // the car actually drives (a car turning a fifth of what it was
            // asked would otherwise run wide until the response is learned).
            // It adapts in log space, so it settles as fast (about 1 s)
            // however wrong the response is; on the straight, with nothing
            // to measure by, it holds.
            let want = lateral::pursuit_curvature(&c.lateral, &p.c, p.range, v);
            if let Some(k) = self.driven
                && want.abs() > c.curvature_min
                && k.abs() > 0.3 * c.curvature_min
                && want.signum() == k.signum()
            {
                let lim = c.curvature_scale_max.ln();
                self.correction = (self.correction + c.curvature_rate * (want.abs().ln() - k.abs().ln()) * dt).clamp(-lim, lim);
            }
            out.angle = Some(lateral::angle_for(want * self.correction.exp() / g, wheelbase));
            return out;
        }
        self.correction = 0.0;
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
        // A car that drives 60 % of the kinematic curvature at 20 m/s.
        for _ in 0..2000 {
            let angle = 0.02f32;
            let yaw = 0.6 * 20.0 * angle.tan() / 2.7;
            lanes.learn(0.014, angle, yaw, 20.0, 2.7);
        }
        assert!((lanes.response.at(20.0) - 0.6).abs() < 0.01, "{}", lanes.response.at(20.0));
        // Straight ahead, or slow: nothing to learn.
        lanes.learn(0.014, 0.0001, 0.0, 20.0, 2.7);
        lanes.learn(0.014, 0.05, 0.0, 3.0, 2.7);
        assert!((lanes.response.at(20.0) - 0.6).abs() < 0.01);
    }

    #[test]
    fn the_response_is_learned_per_speed() {
        // A GTA-like car: 1 / (1 + (v/15)²) of the kinematic curvature.
        let truth = |v: f32| 1.0 / (1.0 + (v / 15.0).powi(2));
        let mut r = SteerResponse::default();
        // Learned at 80 km/h only.
        for _ in 0..3000 {
            let (v, angle) = (22.2f32, 0.03f32);
            r.learn(0.014, angle, truth(v) * v * angle.tan() / 2.7, v, 2.7);
        }
        assert!((r.at(22.2) - truth(22.2)).abs() < 0.03, "at 80 km/h {}", r.at(22.2));
        // Not learned at 35 km/h: from the fitted curve, near the truth and
        // nowhere near the 80 km/h value.
        assert!((r.at(9.7) - truth(9.7)).abs() < 0.08, "at 35 km/h {} vs {}", r.at(9.7), truth(9.7));
        // Nor at 120 km/h.
        assert!((r.at(33.3) - truth(33.3)).abs() < 0.05, "at 120 km/h {} vs {}", r.at(33.3), truth(33.3));
        // While the wheel angle swings, nothing is learned (the yaw lags).
        let before = r;
        for i in 0..200 {
            let angle = 0.03 * (i as f32 * 0.3).sin();
            r.learn(0.014, angle, 0.5 * 22.2 * angle.tan() / 2.7, 22.2, 2.7);
        }
        assert!((r.at(22.2) - before.at(22.2)).abs() < 0.03, "{} vs {}", r.at(22.2), before.at(22.2));
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
