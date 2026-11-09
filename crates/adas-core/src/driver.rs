//! What the driver is doing, from their raw inputs (D11, openpilot's model):
//!
//! - **brake** cancels everything, at once (a brake switch, on any device);
//! - **gas** pauses speed control while pressed (the driver's foot wins);
//! - **steering** pauses lateral control while held; it doesn't cancel ACC.
//!
//! Steering is debounced per input device: a key is all-or-nothing, so one
//! tap mustn't count, while an analogue stick is noisy near its centre.
//! The adapter reports the raw inputs and which kind of device they came
//! from (`sample_flags::INPUT_ANALOG`); the decision is the core's, so every
//! host gets the same rules.

use adapter_api::{VehicleSample, sample_flags};

/// Brake pedal above this cancels.
pub const BRAKE_CANCEL: f32 = 0.10;
/// Accelerator above this overrides speed control.
pub const GAS_OVERRIDE: f32 = 0.05;
/// Analogue steering: above this for `ANALOG_STEER_TIME` (s).
pub const ANALOG_STEER: f32 = 0.15;
pub const ANALOG_STEER_TIME: f32 = 0.10;
/// Digital (key) steering: held for `DIGITAL_STEER_TIME` (s).
pub const DIGITAL_STEER: f32 = 0.5;
pub const DIGITAL_STEER_TIME: f32 = 0.15;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct DriverState {
    /// The driver braked: cancel.
    pub brake: bool,
    /// The driver is on the accelerator: speed control pauses.
    pub gas: bool,
    /// The driver is steering: lateral control pauses.
    pub steering: bool,
}

impl DriverState {
    /// Any input that counts as the driver taking over.
    pub fn any(&self) -> bool {
        self.brake || self.gas || self.steering
    }
}

#[derive(Clone, Debug, Default)]
pub struct DriverMonitor {
    steer_for: f32,
}

impl DriverMonitor {
    pub fn new() -> Self {
        Self::default()
    }

    /// One step of `dt` seconds. Explicit adapter flags (`DRIVER_CANCEL`,
    /// `DRIVER_OVERRIDE`, e.g. a cancel button) count as well.
    pub fn step(&mut self, s: &VehicleSample, dt: f32) -> DriverState {
        let analog = s.flags & sample_flags::INPUT_ANALOG != 0;
        let (threshold, hold) = if analog { (ANALOG_STEER, ANALOG_STEER_TIME) } else { (DIGITAL_STEER, DIGITAL_STEER_TIME) };
        if s.driver_steer.abs() > threshold {
            self.steer_for += dt.max(0.0);
        } else {
            self.steer_for = 0.0;
        }
        DriverState {
            brake: s.driver_brake > BRAKE_CANCEL || s.flags & sample_flags::DRIVER_CANCEL != 0,
            gas: s.driver_throttle > GAS_OVERRIDE || s.flags & sample_flags::DRIVER_OVERRIDE != 0,
            // `>=` with a small margin: frame times don't add up exactly.
            steering: self.steer_for + 1e-4 >= hold,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample(steer: f32, analog: bool) -> VehicleSample {
        VehicleSample { driver_steer: steer, flags: if analog { sample_flags::INPUT_ANALOG } else { 0 }, ..Default::default() }
    }

    fn held_for(steer: f32, analog: bool, secs: f32) -> bool {
        let mut m = DriverMonitor::new();
        let dt = 1.0 / 72.0;
        let mut last = DriverState::default();
        for _ in 0..(secs / dt).round() as usize {
            last = m.step(&sample(steer, analog), dt);
        }
        last.steering
    }

    #[test]
    fn a_key_tap_is_not_steering_but_a_held_key_is() {
        assert!(!held_for(1.0, false, 0.10), "a 100 ms tap");
        assert!(held_for(1.0, false, 0.16), "held 160 ms");
    }

    #[test]
    fn a_stick_counts_above_15_percent_for_100_ms() {
        assert!(!held_for(0.12, true, 1.0), "small stick input");
        assert!(!held_for(0.5, true, 0.07), "a flick");
        assert!(held_for(0.5, true, 0.11));
    }

    #[test]
    fn brake_cancels_and_gas_overrides_at_once() {
        let mut m = DriverMonitor::new();
        let s = VehicleSample { driver_brake: 0.2, ..Default::default() };
        assert!(m.step(&s, 0.0).brake);
        let s = VehicleSample { driver_throttle: 0.3, ..Default::default() };
        let d = m.step(&s, 0.0);
        assert!(d.gas && !d.brake && d.any());
        // Explicit flags still count.
        let s = VehicleSample { flags: sample_flags::DRIVER_CANCEL, ..Default::default() };
        assert!(m.step(&s, 0.0).brake);
    }
}
