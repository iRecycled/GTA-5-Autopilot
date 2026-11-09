//! The driver's own inputs, read from the OS rather than through the game (D5):
//! blocking the game's controls would also block our injected pedals (S4).

use crate::win;

#[derive(Clone, Copy, Debug, Default)]
pub struct Driver {
    pub throttle: f32,
    pub brake: f32,
    /// -1..=1, positive left.
    pub steer: f32,
}

/// Thresholds (D11): a press on the brake or a clear steering input cancels;
/// the accelerator overrides only while pressed.
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
    };
    if let Some(p) = win::gamepad() {
        d.throttle = d.throttle.max(p.right_trigger as f32 / 255.0);
        d.brake = d.brake.max(p.left_trigger as f32 / 255.0);
        let lx = p.thumb_lx as f32;
        if lx.abs() > XINPUT_STICK_DEADZONE {
            // Stick right is positive in XInput; ours is left-positive.
            let s = -(lx - lx.signum() * XINPUT_STICK_DEADZONE) / (32767.0 - XINPUT_STICK_DEADZONE);
            if s.abs() > d.steer.abs() {
                d.steer = s.clamp(-1.0, 1.0);
            }
        }
    }
    d
}

impl Driver {
    pub fn cancels(&self) -> bool {
        self.brake > CANCEL_BRAKE || self.steer.abs() > CANCEL_STEER
    }
    pub fn overrides(&self) -> bool {
        self.throttle > OVERRIDE_THROTTLE
    }
}
