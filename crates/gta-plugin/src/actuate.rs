//! Applying commands in GTA (D5, S4): steering by the per-frame write of the
//! smoothed steering input with an integral correction on the measured angle
//! (the adapter's EPS loop), pedals through the 25% deadzone, the standstill
//! hold as the handbrake.

use gta_edition::{Offsets, STEER_INPUT_MAX, pedal_inject};
use shv_sys::invoke;

use crate::{n, win};

/// The adapter's angle-tracking loop: the core commands a road-wheel angle,
/// this turns it into the input value GTA wants and corrects GTA's pull-back.
pub struct Steering {
    /// Angle ÷ input, estimated on line (S4: 0.61–0.87 by vehicle).
    pub ratio: f32,
    integ: f32,
    prev_cmd: Option<f32>,
}

const KI: f32 = 0.3;
const INTEG_MAX: f32 = 0.3;

impl Steering {
    pub fn new() -> Self {
        Self { ratio: 0.611, integ: 0.0, prev_cmd: None }
    }

    /// Call when the vehicle changes or steering (re)engages.
    pub fn reset(&mut self) {
        self.integ = 0.0;
        self.prev_cmd = None;
    }

    /// Updates the ratio estimate from what the game did last frame.
    pub fn observe(&mut self, input: f32, angle: f32) {
        if input.abs() > 0.05 && angle.is_finite() && input.is_finite() {
            let r = angle / input;
            if (0.2..=2.0).contains(&r) {
                self.ratio += 0.1 * (r - self.ratio);
            }
        }
    }

    /// Writes this frame's input for a commanded angle. `measured` is the angle
    /// the game produced from last frame's write.
    pub fn apply(&mut self, base: usize, off: &Offsets, cmd: f32, measured: f32) {
        if let Some(prev) = self.prev_cmd
            && measured.is_finite()
        {
            self.integ = (self.integ + KI * (prev - measured) / self.ratio.max(0.1)).clamp(-INTEG_MAX, INTEG_MAX);
        }
        let w = (cmd / self.ratio.max(0.1) + self.integ).clamp(-STEER_INPUT_MAX, STEER_INPUT_MAX);
        win::write_f32(base + off.steer_input, w);
        self.prev_cmd = Some(cmd);
    }

    /// Full-lock road-wheel angle for the current vehicle (rad).
    pub fn max_angle(&self) -> f32 {
        self.ratio * STEER_INPUT_MAX
    }
}

pub fn pedals(throttle: f32, brake: f32) {
    unsafe {
        if throttle > 0.0 {
            let _: bool = invoke!(n::SET_CONTROL_VALUE_NEXT_FRAME, n::PLAYER_CONTROL, n::INPUT_VEH_ACCELERATE, pedal_inject(throttle));
        }
        if brake > 0.0 {
            let _: bool = invoke!(n::SET_CONTROL_VALUE_NEXT_FRAME, n::PLAYER_CONTROL, n::INPUT_VEH_BRAKE, pedal_inject(brake));
        }
    }
}

pub fn hold(veh: i32, on: bool) {
    let () = unsafe { invoke!(n::SET_VEHICLE_HANDBRAKE, veh, on) };
}

/// The brake ramp after an unplanned release (D24): the last brake demand
/// fades to zero over about a second instead of letting go at once.
pub struct ReleaseRamp {
    brake: f32,
    rate: f32,
    /// Keep braking until the car stops (AEB was braking when the core went away).
    to_stop: bool,
}

impl ReleaseRamp {
    pub fn new() -> Self {
        Self { brake: 0.0, rate: 0.0, to_stop: false }
    }
    pub fn start(&mut self, brake: f32) {
        self.brake = brake.clamp(0.0, 1.0);
        self.rate = self.brake; // per second → about 1 s to zero
        self.to_stop = false;
    }
    /// D24: if AEB was braking, keep braking to a stop.
    pub fn start_to_stop(&mut self, brake: f32) {
        self.brake = brake.clamp(0.3, 1.0);
        self.rate = 0.0;
        self.to_stop = true;
    }
    pub fn active(&self) -> bool {
        self.brake > 0.0
    }
    pub fn cancel(&mut self) {
        self.brake = 0.0;
    }
    /// Applies this frame's ramp brake.
    pub fn step(&mut self, dt: f32, speed: f32) {
        if self.to_stop && speed < 0.3 {
            self.brake = 0.0;
            self.to_stop = false;
        }
        if self.brake > 0.0 {
            pedals(0.0, self.brake);
            self.brake = (self.brake - self.rate * dt).max(0.0);
        }
    }
}
