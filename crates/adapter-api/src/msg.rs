//! Message layouts. Every message starts with [`Header`].

/// Message type tag.
#[repr(u16)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum MsgKind {
    #[default]
    None = 0,
    VehicleSample = 1,
    Command = 2,
}

/// Common header (layout from `docs/implementation-plan.md`).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Header {
    pub kind: MsgKind,
    pub version: u16,
    /// Per channel; a gap means a dropped message.
    pub seq: u32,
    /// Host frame counter.
    pub frame: u64,
    /// Accumulated from per-frame delta time (stops while the game is paused).
    pub game_time_us: u64,
    /// [`crate::wall_time_ns`] when the message was written.
    pub wall_time_ns: u64,
}

/// Sample flags.
pub mod sample_flags {
    /// The game is paused, in a menu or loading: the core must not act.
    pub const PAUSED: u32 = 1 << 0;
    /// Set while the driver wants the core engaged. The core engages only on a
    /// rising edge it has seen itself, so a restarted core never re-engages on
    /// its own; it disengages as soon as the flag drops.
    pub const ENGAGE_REQUEST: u32 = 1 << 1;
    /// The driver is overriding (pedal or steering input above threshold).
    pub const DRIVER_OVERRIDE: u32 = 1 << 2;
}

/// adapter → core, every frame. ISO 8855: x forward, y left, z up.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct VehicleSample {
    pub header: Header,
    pub flags: u32,
    /// Speed over ground (m/s).
    pub speed: f32,
    /// Body velocity, x forward and y left (m/s).
    pub vx: f32,
    pub vy: f32,
    /// Yaw rate, positive turning left (rad/s).
    pub yaw_rate: f32,
    /// Road-wheel steering angle, positive left (rad).
    pub steer_angle: f32,
    /// Wheel spin speeds FL, FR, RL, RR, positive rolling forward (rad/s);
    /// NaN when the host doesn't provide them.
    pub wheel_omega: [f32; 4],
    /// Engine speed as a fraction of the rev range, 0..=1; NaN if unknown.
    pub rev_ratio: f32,
    /// Current gear; 0 = reverse or neutral, as the host reports it.
    pub gear: i32,
}

/// Command flags.
pub mod command_flags {
    /// The core is engaged and wants these actuator values applied.
    pub const ENGAGED: u32 = 1 << 0;
    /// Request the standstill hold.
    pub const HOLD: u32 = 1 << 1;
}

/// core → adapter, once per core step.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Command {
    pub header: Header,
    /// The sample `seq` this command answers; the adapter uses it for its
    /// freshness watchdog (D24).
    pub answers_seq: u32,
    pub flags: u32,
    /// Road-wheel steering angle, positive left (rad).
    pub steer_angle: f32,
    /// Pedal demands, 0..=1. The adapter maps them onto the host's pedals.
    pub throttle: f32,
    pub brake: f32,
    /// Free-form diagnostics for logging; adapters must not act on them.
    pub diag: [f32; 4],
    /// Explicit padding to an 8-byte multiple; zero.
    pub _reserved: u32,
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::mem::{align_of, size_of};

    /// The layout is the contract: these sizes change only with CONTRACT_VERSION.
    #[test]
    fn layouts_are_stable() {
        assert_eq!(size_of::<Header>(), 32);
        assert_eq!(align_of::<Header>(), 8);
        // header + flags + 5 scalars + wheel_omega + rev_ratio + gear: no padding.
        assert_eq!(size_of::<VehicleSample>(), 32 + 4 + 5 * 4 + 16 + 4 + 4);
        // header + answers_seq + flags + 3 scalars + diag + _reserved: no padding.
        assert_eq!(size_of::<Command>(), 32 + 4 + 4 + 3 * 4 + 16 + 4);
    }
}
