//! The driver's own inputs, read from the OS rather than through the game (D5):
//! blocking the game's controls would also block our injected pedals (S4).

use crate::win;

#[derive(Clone, Copy, Debug, Default)]
pub struct Driver {
    pub throttle: f32,
    pub brake: f32,
    /// -1..=1, positive left.
    pub steer: f32,
    /// The gamepad (analogue) supplied the inputs, not the keyboard: the
    /// core debounces steering per device (D11).
    pub analog: bool,
}

/// The adapter's own, immediate thresholds: the driver's pedals win over the
/// core's at once, and they stop a safe-state brake fade. Whether the driver
/// cancelled or overrode a feature is the core's decision (D11).
pub const CANCEL_BRAKE: f32 = 0.10;
pub const CANCEL_STEER: f32 = 0.20;
pub const OVERRIDE_THROTTLE: f32 = 0.05;

const VK_W: i32 = b'W' as i32;
const VK_S: i32 = b'S' as i32;
const VK_A: i32 = b'A' as i32;
const VK_D: i32 = b'D' as i32;
const XINPUT_STICK_DEADZONE: f32 = 7849.0;

/// Keyboard (digital) and the first XInput pad (analogue), whichever is larger.
pub fn read() -> Driver {
    let mut d = Driver {
        throttle: if win::key_down(VK_W) { 1.0 } else { 0.0 },
        brake: if win::key_down(VK_S) { 1.0 } else { 0.0 },
        steer: match (win::key_down(VK_A), win::key_down(VK_D)) {
            (true, false) => 1.0,
            (false, true) => -1.0,
            _ => 0.0,
        },
        analog: false,
    };
    if let Some(p) = win::gamepad() {
        let (rt, lt) = (p.right_trigger as f32 / 255.0, p.left_trigger as f32 / 255.0);
        let lx = p.thumb_lx as f32;
        // Stick right is positive in XInput; ours is left-positive.
        let stick = if lx.abs() > XINPUT_STICK_DEADZONE { -(lx - lx.signum() * XINPUT_STICK_DEADZONE) / (32767.0 - XINPUT_STICK_DEADZONE) } else { 0.0 };
        // Analogue when the pad supplies the steering, or the pad is in use
        // and no key is pressed.
        let keys = d.throttle > 0.0 || d.brake > 0.0 || d.steer != 0.0;
        d.analog = stick.abs() > d.steer.abs() || (!keys && (rt > 0.0 || lt > 0.0));
        d.throttle = d.throttle.max(rt);
        d.brake = d.brake.max(lt);
        if stick.abs() > d.steer.abs() {
            d.steer = stick.clamp(-1.0, 1.0);
        }
    }
    d
}

impl Driver {
    /// Any clear driver input (stops a safe-state brake fade).
    pub fn active(&self) -> bool {
        self.brake > CANCEL_BRAKE || self.steer.abs() > CANCEL_STEER || self.throttle > OVERRIDE_THROTTLE
    }
    /// The driver's foot is on a pedal: their pedals win over the core's.
    pub fn on_pedals(&self) -> bool {
        self.brake > CANCEL_BRAKE || self.throttle > OVERRIDE_THROTTLE
    }
}
