//! The actuator ECUs on chassis CAN (D9, D28): what turns the ADAS ECU's
//! requests into actuation, as on a TSS2 car.
//!
//! The ADAS ECU sends `ACC_CONTROL` (the acceleration request and standstill
//! hold), `PRE_COLLISION_2` (AEB's deceleration request) and `STEERING_LTA`
//! (the steering-wheel angle request). Then:
//! - the **powertrain/brake ECU** turns the acceleration request into throttle
//!   and brake through the car's calibration (the vehicle profile's gains,
//!   D19), tracks AEB's deceleration request in a closed loop on the measured
//!   acceleration, and holds the car at a standstill;
//! - the **EPS ECU** steers the road wheels toward the requested angle within
//!   its rate and range limits.
//!
//! The host then gets pedals, hold and a road-wheel angle, as before: nothing
//! about the adapter contract changes.

use crate::vbus::STEERING_RATIO;

/// What the ADAS ECU puts on the bus this step.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AdasRequest {
    /// A longitudinal feature is engaged (`ACC_CONTROL.ACC_TYPE` = 1).
    pub acc_active: bool,
    /// m/s² (`ACCEL_CMD`, ±20).
    pub accel: f32,
    /// Hold at a standstill (`RELEASE_STANDSTILL` = 0).
    pub hold: bool,
    /// AEB's deceleration request (m/s², positive; infinite = full braking).
    pub aeb_decel: f32,
    /// AEB holds the car after its stop (`PBATRGR`).
    pub aeb_hold: bool,
    /// `STEERING_LTA.STEER_REQUEST`.
    pub steer_request: bool,
    /// Road-wheel angle (rad, left +); sent as steering-wheel degrees.
    pub steer_angle: f32,
}

/// What the powertrain ECU last received.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AccelRequest {
    pub acc_active: bool,
    pub accel: f32,
    pub hold: bool,
    /// m/s², positive; ≥ `FULL_DECEL` = full braking.
    pub aeb_decel: f32,
    pub aeb_hold: bool,
}

/// What the EPS last received.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SteerRequest {
    pub request: bool,
    /// Steering-wheel angle (deg, left +).
    pub sw_deg: f32,
}

/// AEB requests at or above this are full braking (m/s²). Full braking is
/// sent as `FULL_DECEL_SENT`.
pub const FULL_DECEL: f32 = 9.5;
pub const FULL_DECEL_SENT: f32 = 10.0;
/// `ACCEL_CMD`'s range (m/s²).
pub const ACCEL_CMD_MAX: f32 = 20.0;
/// Integral gain of the AEB deceleration loop (demand per m/s²·s).
const AEB_KI: f32 = 0.35;

/// The car's calibration: acceleration per unit of pedal demand (m/s²).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Gains {
    pub throttle: f32,
    pub brake: f32,
}

impl Gains {
    /// The ADAS ECU's side: the acceleration that a pedal demand asks for.
    pub fn to_accel(&self, throttle: f32, brake: f32) -> f32 {
        (throttle * self.throttle - brake * self.brake).clamp(-ACCEL_CMD_MAX, ACCEL_CMD_MAX)
    }
}

/// The powertrain/brake ECU.
#[derive(Default)]
pub struct Powertrain {
    aeb_i: f32,
}

impl Powertrain {
    pub fn new() -> Self {
        Self::default()
    }

    /// One step: (throttle, brake, hold). `a_meas` is the measured
    /// acceleration (m/s²).
    pub fn step(&mut self, r: &AccelRequest, g: Gains, a_meas: f32, dt: f32) -> (f32, f32, bool) {
        // The acceleration request through the calibration.
        let (mut thr, mut brk) = if !r.acc_active {
            (0.0, 0.0)
        } else if r.accel >= 0.0 {
            ((r.accel / g.throttle).min(1.0), 0.0)
        } else {
            (0.0, (-r.accel / g.brake).min(1.0))
        };
        let mut hold = r.acc_active && r.hold;
        // AEB: track its deceleration on the host's brakes; it wins.
        if r.aeb_decel > 0.0 {
            let b = if r.aeb_decel >= FULL_DECEL {
                1.0
            } else {
                self.aeb_i = (self.aeb_i + AEB_KI * (r.aeb_decel + a_meas) * dt).clamp(-0.6, 0.6);
                (r.aeb_decel / g.brake + self.aeb_i).clamp(0.02, 1.0)
            };
            thr = 0.0;
            brk = brk.max(b);
            hold = false;
        } else {
            self.aeb_i = 0.0;
        }
        if r.aeb_hold {
            (thr, brk, hold) = (0.0, 0.0, true);
        }
        (thr, brk, hold)
    }
}

/// The EPS (D28): angle commands with rate and range limits.
#[derive(Default)]
pub struct Eps {
    angle: f32,
    engaged: bool,
    pub limits: EpsLimits,
}

/// The wide road-wheel limits of Phases 0 to 2 (the test driver keeps them):
/// range (rad) and rate (rad/s).
pub const EPS_MAX_ANGLE: f32 = 0.6;
pub const EPS_MAX_RATE: f32 = 0.8;

/// What the EPS accepts from `STEERING_LTA`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum EpsLimits {
    /// Any angle within ±0.6 rad at 0.8 rad/s (development: the test driver).
    Wide,
    /// A TSS2 car's LTA limits (opendbc `toyota/values.py`, `ANGLE_LIMITS`):
    /// the steering wheel within ±94.9°; turning further at 30 °/s at 5 m/s
    /// falling to 15 °/s at 25 m/s, turning back at 36 to 26 °/s.
    Tss2,
}

impl Default for EpsLimits {
    fn default() -> Self {
        EpsLimits::Wide
    }
}

impl EpsLimits {
    /// (range, rate further from centre, rate back toward it), road wheel
    /// (rad, rad/s), at `speed` (m/s).
    pub fn at(self, speed: f32) -> (f32, f32, f32) {
        match self {
            EpsLimits::Wide => (EPS_MAX_ANGLE, EPS_MAX_RATE, EPS_MAX_RATE),
            EpsLimits::Tss2 => {
                let f = ((speed - 5.0) / 20.0).clamp(0.0, 1.0);
                let sw = |a: f32, b: f32| (a + (b - a) * f).to_radians() / STEERING_RATIO;
                (94.9f32.to_radians() / STEERING_RATIO, sw(30.0, 15.0), sw(36.0, 26.0))
            }
        }
    }
}

impl Eps {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_limits(limits: EpsLimits) -> Self {
        Self { limits, ..Self::default() }
    }

    /// The road-wheel angle to apply, or `None` when steering isn't
    /// requested. `measured`: the current road-wheel angle (rad), to start
    /// from when engaging, so engaging doesn't jerk the wheel.
    /// `speed`: the car's speed (m/s), for speed-dependent limits.
    pub fn step(&mut self, r: &SteerRequest, measured: f32, dt: f32, speed: f32) -> Option<f32> {
        if !r.request {
            self.engaged = false;
            return None;
        }
        let (max, rate_out, rate_in) = self.limits.at(speed);
        if !self.engaged {
            self.engaged = true;
            self.angle = if measured.is_finite() { measured.clamp(-max, max) } else { 0.0 };
        }
        let want = (r.sw_deg.to_radians() / STEERING_RATIO).clamp(-max, max);
        let d = want - self.angle;
        // Turning away from centre is slower than turning back.
        let rate = if d * self.angle > 0.0 || self.angle == 0.0 { rate_out } else { rate_in };
        let step = rate * dt.max(0.0);
        self.angle += d.clamp(-step, step);
        Some(self.angle)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const G: Gains = Gains { throttle: 4.0, brake: 8.0 };

    #[test]
    fn acc_requests_become_pedals_through_the_calibration() {
        let mut p = Powertrain::new();
        let acc = |accel, hold| AccelRequest { acc_active: true, accel, hold, ..Default::default() };
        assert_eq!(p.step(&acc(2.0, false), G, 0.0, 0.01), (0.5, 0.0, false));
        assert_eq!(p.step(&acc(-4.0, false), G, 0.0, 0.01), (0.0, 0.5, false));
        assert_eq!(p.step(&acc(-1.0, true), G, 0.0, 0.01), (0.0, 0.125, true));
        assert_eq!(p.step(&AccelRequest::default(), G, 0.0, 0.01), (0.0, 0.0, false));
        // The ADAS ECU's conversion is the inverse.
        assert_eq!(G.to_accel(0.5, 0.0), 2.0);
        assert_eq!(G.to_accel(0.0, 0.5), -4.0);
        assert_eq!(G.to_accel(0.0, 10.0), -ACCEL_CMD_MAX);
    }

    #[test]
    fn aeb_tracks_its_deceleration_and_wins() {
        let mut p = Powertrain::new();
        // The host's brakes are twice as strong as the calibration says.
        let r = AccelRequest { acc_active: true, accel: 1.0, aeb_decel: 5.0, ..Default::default() };
        let mut brk = 0.0;
        for _ in 0..200 {
            let (t, b, h) = p.step(&r, G, -16.0 * brk, 1.0 / 72.0);
            assert!(t == 0.0 && !h);
            brk = b;
        }
        assert!((16.0 * brk - 5.0).abs() < 0.3, "decel {}", 16.0 * brk);
        let full = AccelRequest { aeb_decel: FULL_DECEL_SENT, ..Default::default() };
        assert_eq!(p.step(&full, G, 0.0, 0.01), (0.0, 1.0, false));
        let hold = AccelRequest { aeb_hold: true, ..Default::default() };
        assert_eq!(p.step(&hold, G, 0.0, 0.01), (0.0, 0.0, true));
    }

    #[test]
    fn eps_limits_rate_and_range() {
        let mut e = Eps::new();
        let req = |sw_deg| SteerRequest { request: true, sw_deg };
        assert_eq!(e.step(&SteerRequest { request: false, sw_deg: 100.0 }, 0.0, 0.1, 10.0), None);
        // A big step request: the road wheels move at most 0.8 rad/s.
        let a = e.step(&req(500.0), 0.0, 0.1, 10.0).unwrap();
        assert!((a - 0.08).abs() < 1e-5, "{a}");
        for _ in 0..50 {
            e.step(&req(2000.0), 0.0, 0.1, 10.0);
        }
        assert!((e.step(&req(2000.0), 0.0, 0.1, 10.0).unwrap() - EPS_MAX_ANGLE).abs() < 1e-6);
        // Engaging starts from the measured angle (no jump).
        let mut e = Eps::new();
        assert!((e.step(&req(0.0), 0.2, 0.0, 10.0).unwrap() - 0.2).abs() < 1e-6);
    }

    #[test]
    fn tss2_limits_follow_speed() {
        let mut e = Eps::with_limits(EpsLimits::Tss2);
        let req = |sw_deg| SteerRequest { request: true, sw_deg };
        // At 25 m/s the wheel turns at 15 °/s: 1.5° in 0.1 s.
        let a = e.step(&req(90.0), 0.0, 0.1, 25.0).unwrap();
        assert!(((a * STEERING_RATIO).to_degrees() - 1.5).abs() < 1e-3, "{}", (a * STEERING_RATIO).to_degrees());
        // At 5 m/s, 30 °/s.
        let mut e = Eps::with_limits(EpsLimits::Tss2);
        let a = e.step(&req(90.0), 0.0, 0.1, 5.0).unwrap();
        assert!(((a * STEERING_RATIO).to_degrees() - 3.0).abs() < 1e-3);
        // Never past 94.9° at the wheel.
        for _ in 0..100 {
            e.step(&req(500.0), 0.0, 0.1, 5.0);
        }
        let a = e.step(&req(500.0), 0.0, 0.1, 5.0).unwrap();
        assert!(((a * STEERING_RATIO).to_degrees() - 94.9).abs() < 1e-3);
        // Back toward centre faster: 36 °/s at 5 m/s.
        let b = e.step(&req(0.0), 0.0, 0.1, 5.0).unwrap();
        assert!((((a - b) * STEERING_RATIO).to_degrees() - 3.6).abs() < 1e-3);
    }
}
