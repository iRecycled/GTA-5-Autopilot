//! Longitudinal (speed) control: comfort-limited reference, feed-forward + PI,
//! stop-and-hold.
//!
//! The controller follows a *reference* speed that moves toward the target no
//! faster than the comfort limits allow, so a step in the target (for example
//! 0 → 8 m/s at a launch) becomes a smooth, jerk-limited ramp. Its output is a
//! pedal demand in [-1, 1]: positive is throttle, negative is brake. The adapter
//! maps the demand onto the host's pedals.
//!
//! Tuned in spike S4 on GTA V (see `docs/spikes/s45-results.md`): speed held
//! within 0.02–0.15 m/s, launch peak 2.2 m/s², clean stop.

/// Controller parameters. `Default` holds the values tuned in spike S4.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Config {
    /// Comfort limit on the reference's acceleration (m/s²).
    pub accel_max: f32,
    /// Comfort limit on the reference's deceleration (m/s², positive).
    pub decel_max: f32,
    /// Comfort limit on the reference's jerk (m/s³). Kept below the 2.5 m/s³
    /// gate on the car itself: the controller and the plant add a little.
    pub jerk_max: f32,
    /// How hard the reference closes on the target before the limits apply (1/s).
    pub ref_gain: f32,
    /// The reference never closes on its target slower than this (m/s²), so a
    /// stop doesn't end in a long crawl.
    pub ref_min_accel: f32,
    /// The reference locks onto the target within this distance (m/s).
    pub ref_snap: f32,
    /// Proportional gain on the speed error (demand per m/s).
    pub kp: f32,
    /// Integral gain (demand per m/s·s).
    pub ki: f32,
    /// Feed-forward: demand per m/s² of planned acceleration.
    pub kff: f32,
    /// Clamp on the integrated error (m/s·s).
    pub integ_clamp: f32,
    /// A pedal engages when |demand| exceeds this...
    pub pedal_on: f32,
    /// ...and releases when the demand falls back through this (hysteresis).
    pub pedal_off: f32,
    /// Below this speed (m/s), with a zero target, the car counts as stopped and
    /// the standstill hold engages.
    pub stop_speed: f32,
    /// While finishing a stop, brake demand is at least this.
    pub stop_brake: f32,
    /// Jerk limit on the command itself (m/s³): the pedal command is turned
    /// into the acceleration it implies (through the plant gains) and that
    /// may change only this fast. Below the 2.5 m/s³ gate: the gains are
    /// estimates (the learned brake gain reads ~15 % low while ramping).
    pub jerk_out: f32,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            accel_max: 2.0,
            decel_max: 3.0,
            jerk_max: 2.2,
            ref_gain: 0.8,
            ref_min_accel: 0.5,
            ref_snap: 0.05,
            kp: 0.22,
            ki: 0.14,
            kff: 0.11,
            integ_clamp: 1.0,
            pedal_on: 0.03,
            pedal_off: 0.005,
            stop_speed: 0.1,
            stop_brake: 0.12,
            jerk_out: 2.2,
        }
    }
}

/// One step's result.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Output {
    /// Throttle demand, 0..=1.
    pub throttle: f32,
    /// Brake demand, 0..=1.
    pub brake: f32,
    /// Request the standstill hold (the host's auto-hold or parking brake).
    pub hold: bool,
    /// The comfort-limited reference speed being tracked (m/s).
    pub reference: f32,
    /// The reference's planned acceleration (m/s²).
    pub ref_accel: f32,
    /// The raw demand before the pedal split, -1..=1.
    pub demand: f32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Pedal {
    Coast,
    Throttle,
    Brake,
}

/// Speed controller. Call [`SpeedController::reset`] when it engages, then
/// [`SpeedController::step`] once per host frame.
#[derive(Clone, Debug)]
pub struct SpeedController {
    cfg: Config,
    reference: f32,
    ref_accel: f32,
    integ: f32,
    pedal: Pedal,
    /// Plant gains: acceleration per unit of throttle and of brake (m/s²).
    gain_throttle: f32,
    gain_brake: f32,
    /// The acceleration the last command implied (for the jerk limit).
    a_cmd: f32,
}

/// Default plant gains until the host's are known.
const GAIN_THROTTLE: f32 = 4.0;
const GAIN_BRAKE: f32 = 8.0;

impl SpeedController {
    pub fn new(cfg: Config) -> Self {
        Self { cfg, reference: 0.0, ref_accel: 0.0, integ: 0.0, pedal: Pedal::Coast, gain_throttle: GAIN_THROTTLE, gain_brake: GAIN_BRAKE, a_cmd: 0.0 }
    }

    /// The host's plant gains (m/s² per unit of pedal), when known.
    pub fn set_gains(&mut self, throttle: f32, brake: f32) {
        if throttle.is_finite() && throttle > 0.1 {
            self.gain_throttle = throttle;
        }
        if brake.is_finite() && brake > 0.1 {
            self.gain_brake = brake;
        }
    }

    pub fn config(&self) -> &Config {
        &self.cfg
    }

    /// Starts from the car's current speed, so engaging never jumps.
    pub fn reset(&mut self, speed: f32) {
        self.reference = speed.max(0.0);
        self.ref_accel = 0.0;
        self.integ = 0.0;
        self.pedal = Pedal::Coast;
        self.a_cmd = 0.0;
    }

    /// Advances by `dt` seconds toward `target` (m/s) given the measured `speed`.
    pub fn step(&mut self, target: f32, speed: f32, dt: f32) -> Output {
        let c = self.cfg;
        let target = target.max(0.0);
        if !(dt > 0.0 && dt.is_finite() && speed.is_finite()) {
            return self.output(0.0, 0.0, 0.0, false);
        }

        // 1. Comfort-limited reference.
        let e_ref = target - self.reference;
        let mut want = (c.ref_gain * e_ref).clamp(-c.decel_max, c.accel_max);
        if e_ref.abs() > c.ref_snap && want.abs() < c.ref_min_accel {
            want = c.ref_min_accel * e_ref.signum();
        }
        let step = c.jerk_max * dt;
        self.ref_accel += (want - self.ref_accel).clamp(-step, step);
        let next = (self.reference + self.ref_accel * dt).max(0.0);
        if (target - next).abs() <= c.ref_snap || (target - next).signum() != e_ref.signum() {
            self.reference = target;
            self.ref_accel = 0.0;
        } else {
            self.reference = next;
        }

        // 2. Feed-forward + PI with anti-windup (stop integrating while saturated
        //    in the direction the error is pushing).
        let err = self.reference - speed;
        let integ_next = (self.integ + err * dt).clamp(-c.integ_clamp, c.integ_clamp);
        let raw = c.kff * self.ref_accel + c.kp * err + c.ki * integ_next;
        let demand = raw.clamp(-1.0, 1.0);
        let saturated_same_way = (raw > 1.0 && err > 0.0) || (raw < -1.0 && err < 0.0);
        if !saturated_same_way {
            self.integ = integ_next;
        }

        // 3. Stop-and-hold: once the reference is at zero, finish the stop on
        //    gentle brake (never throttle); hold only once actually stopped.
        let stopping = target < c.ref_snap && self.reference < c.ref_snap;
        if stopping {
            self.pedal = Pedal::Brake;
            if speed < c.stop_speed {
                self.integ = 0.0;
                self.a_cmd = 0.0;
                return self.output(0.0, 0.0, demand, true);
            }
            let brk = (-demand).clamp(c.stop_brake, 1.0);
            self.a_cmd = -brk * self.gain_brake;
            return self.output(0.0, brk, demand, false);
        }

        // 4. Pedal split with hysteresis, so the demand doesn't chatter around zero.
        let band = c.pedal_on - c.pedal_off;
        self.pedal = match self.pedal {
            Pedal::Coast if demand > c.pedal_on => Pedal::Throttle,
            Pedal::Coast if demand < -c.pedal_on => Pedal::Brake,
            Pedal::Throttle if demand < c.pedal_off => {
                if demand < -c.pedal_on { Pedal::Brake } else { Pedal::Coast }
            }
            Pedal::Brake if demand > -c.pedal_off => {
                if demand > c.pedal_on { Pedal::Throttle } else { Pedal::Coast }
            }
            p => p,
        };
        // Each pedal starts from (almost) zero when it engages: subtract the
        // band, so crossing it is not a step (the integral absorbs the offset).
        let (thr, brk) = match self.pedal {
            Pedal::Throttle => ((demand - band).max(0.0), 0.0),
            Pedal::Brake => (0.0, (-demand - band).max(0.0)),
            Pedal::Coast => (0.0, 0.0),
        };
        // 5. Jerk limit on the command, in acceleration terms.
        let want = thr * self.gain_throttle - brk * self.gain_brake;
        let step = c.jerk_out * dt;
        let a = self.a_cmd + (want - self.a_cmd).clamp(-step, step);
        self.a_cmd = a;
        let (thr, brk) = if a >= 0.0 { ((a / self.gain_throttle).min(1.0), 0.0) } else { (0.0, (-a / self.gain_brake).min(1.0)) };
        self.output(thr, brk, demand, false)
    }

    fn output(&self, throttle: f32, brake: f32, demand: f32, hold: bool) -> Output {
        Output { throttle, brake, hold, reference: self.reference, ref_accel: self.ref_accel, demand }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 1.0 / 60.0;

    /// A crude car: throttle gives up to 4 m/s², brake up to 8 m/s², plus
    /// rolling resistance and drag. A standstill hold pins it at zero.
    struct Car {
        v: f32,
    }

    impl Car {
        fn apply(&mut self, o: &Output) -> f32 {
            if o.hold {
                self.v = 0.0;
                return 0.0;
            }
            let resist = if self.v > 0.0 { 0.15 + 0.0004 * self.v * self.v } else { 0.0 };
            let a = 4.0 * o.throttle - 8.0 * o.brake * (self.v > 0.0) as i32 as f32 - resist;
            let v0 = self.v;
            self.v = (self.v + a * DT).max(0.0);
            (self.v - v0) / DT
        }
    }

    fn run(ctl: &mut SpeedController, car: &mut Car, target: f32, seconds: f32) -> Vec<(Output, f32, f32)> {
        let mut out = Vec::new();
        for _ in 0..(seconds / DT) as usize {
            let o = ctl.step(target, car.v, DT);
            let a = car.apply(&o);
            out.push((o, car.v, a));
        }
        out
    }

    #[test]
    fn launch_is_comfort_limited_and_reaches_target() {
        let mut ctl = SpeedController::new(Config::default());
        let mut car = Car { v: 0.0 };
        ctl.reset(0.0);
        let log = run(&mut ctl, &mut car, 8.0, 15.0);
        let peak_ref_acc = log.iter().map(|l| l.0.ref_accel).fold(f32::MIN, f32::max);
        assert!(peak_ref_acc <= 2.0 + 1e-4, "reference accel {peak_ref_acc}");
        let peak_acc = log.iter().map(|l| l.2).fold(f32::MIN, f32::max);
        assert!(peak_acc < 2.8, "car accel {peak_acc}");
        assert!((car.v - 8.0).abs() < 0.1, "final speed {}", car.v);
    }

    #[test]
    fn reference_jerk_is_limited() {
        let mut ctl = SpeedController::new(Config::default());
        let mut car = Car { v: 0.0 };
        ctl.reset(0.0);
        let log = run(&mut ctl, &mut car, 15.0, 12.0);
        for w in log.windows(2) {
            let jerk = (w[1].0.ref_accel - w[0].0.ref_accel) / DT;
            // A snap onto the target may drop the planned accel to zero at once.
            if w[1].0.ref_accel != 0.0 {
                assert!(jerk.abs() <= 2.5 + 1e-3, "jerk {jerk}");
            }
        }
    }

    #[test]
    fn stop_ends_in_hold_without_throttle_or_crawl() {
        let mut ctl = SpeedController::new(Config::default());
        let mut car = Car { v: 12.0 };
        ctl.reset(12.0);
        let log = run(&mut ctl, &mut car, 0.0, 12.0);
        let first_hold = log.iter().position(|l| l.0.hold).expect("hold engaged");
        assert!(first_hold as f32 * DT < 9.0, "stop took {} s", first_hold as f32 * DT);
        // Never throttle while the reference is being brought to zero.
        assert!(log.iter().all(|l| l.0.throttle == 0.0 || l.0.reference > 0.05));
        // Hold engages only when stopped, and stays.
        assert!(log[first_hold..].iter().all(|l| l.0.hold && l.0.throttle == 0.0 && l.0.brake == 0.0));
        let peak_decel = log.iter().map(|l| l.2).fold(f32::MAX, f32::min);
        assert!(peak_decel > -4.0, "peak decel {peak_decel}");
    }

    #[test]
    fn holds_speed_without_pedal_chatter() {
        let mut ctl = SpeedController::new(Config::default());
        let mut car = Car { v: 10.0 };
        ctl.reset(10.0);
        let log = run(&mut ctl, &mut car, 10.0, 20.0);
        let tail = &log[log.len() / 2..];
        let err = tail.iter().map(|l| (l.1 - 10.0).abs()).fold(0.0, f32::max);
        assert!(err < 0.05, "speed error {err}");
        let switches = tail.windows(2).filter(|w| (w[0].0.throttle > 0.0) != (w[1].0.throttle > 0.0)).count();
        assert_eq!(switches, 0, "throttle switched on/off {switches} times");
    }

    #[test]
    fn reset_starts_from_current_speed() {
        let mut ctl = SpeedController::new(Config::default());
        ctl.reset(13.0);
        let o = ctl.step(13.0, 13.0, DT);
        assert!((o.reference - 13.0).abs() < 1e-6);
        assert!(o.brake == 0.0);
    }

    #[test]
    fn bad_dt_does_nothing() {
        let mut ctl = SpeedController::new(Config::default());
        ctl.reset(5.0);
        for dt in [0.0, -1.0, f32::NAN, f32::INFINITY] {
            let o = ctl.step(10.0, 5.0, dt);
            assert_eq!((o.throttle, o.brake, o.hold), (0.0, 0.0, false));
        }
    }
}
