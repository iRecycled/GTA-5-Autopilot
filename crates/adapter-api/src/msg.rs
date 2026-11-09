//! Message layouts. Every message starts with [`Header`]. All positions are
//! ISO 8855 (x forward, y left, z up) in the ego vehicle frame, SI units.

/// Message type tag.
#[repr(u16)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum MsgKind {
    #[default]
    None = 0,
    VehicleSample = 1,
    Command = 2,
    ActorList = 3,
    SensorLayout = 4,
    AdapterInfo = 5,
    Hud = 6,
    FanLayout = 7,
    RangeReport = 8,
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
    /// An explicit override from the host (the core also judges the raw
    /// driver inputs itself, D11).
    pub const DRIVER_OVERRIDE: u32 = 1 << 2;
    /// An explicit cancel from the host, e.g. a cancel button (the core also
    /// judges the raw driver inputs itself, D11).
    pub const DRIVER_CANCEL: u32 = 1 << 3;
    /// The driver inputs come from an analogue device (gamepad, wheel), not
    /// keys: the core debounces steering per device (D11).
    pub const INPUT_ANALOG: u32 = 1 << 4;
    /// Reverse is selected: the car moves (or will move) backwards; `vx` is
    /// negative while it does (`caps::GEAR`).
    pub const REVERSE: u32 = 1 << 5;
    /// The driver's turn signal is on, left or right (`caps::TURN_SIGNALS`).
    pub const TURN_LEFT: u32 = 1 << 6;
    pub const TURN_RIGHT: u32 = 1 << 7;
}

/// adapter → core, every frame.
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
    /// Wheel spin speeds in host order (rad/s, positive rolling forward);
    /// NaN when the host doesn't provide them.
    pub wheel_omega: [f32; 4],
    /// Engine speed as a fraction of the rev range, 0..=1; NaN if unknown.
    pub rev_ratio: f32,
    /// Current gear; 0 = reverse or neutral, as the host reports it.
    pub gear: i32,
    /// The driver's own inputs, read by the adapter from the OS (D5):
    /// pedals 0..=1, steering -1..=1 (positive left).
    pub driver_throttle: f32,
    pub driver_brake: f32,
    pub driver_steer: f32,
    /// Driver-assistance buttons pressed this frame (edges, see [`buttons`]).
    pub buttons: u32,
}

/// Button edges in [`VehicleSample::buttons`]: set on the one frame the button
/// goes down. The core decides what they mean (set speed, gap).
pub mod buttons {
    pub const SPEED_UP: u32 = 1 << 0;
    pub const SPEED_DOWN: u32 = 1 << 1;
    pub const GAP: u32 = 1 << 2;
}

/// Command flags.
pub mod command_flags {
    /// The core is engaged and wants these actuator values applied.
    pub const ENGAGED: u32 = 1 << 0;
    /// Request the standstill hold.
    pub const HOLD: u32 = 1 << 1;
    /// The core wants to steer (lateral control engaged).
    pub const STEER: u32 = 1 << 2;
    /// Automatic braking (AEB, rear cross-traffic braking, park assist): apply
    /// `brake` (and `HOLD` once stopped) even when not engaged. It overrides
    /// the driver's accelerator whichever way the car is moving, so a host
    /// whose brake control reverses at a standstill must map it. The HUD says
    /// which feature braked.
    pub const AEB: u32 = 1 << 3;
    /// Forward collision warning: alert the driver (chime and flash).
    pub const FCW: u32 = 1 << 4;
    /// Hazard lights on (a controlled stop, D11).
    pub const HAZARDS: u32 = 1 << 5;
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
    /// Road-wheel steering angle, positive left (rad). Applied only with `STEER`.
    pub steer_angle: f32,
    /// Pedal demands, 0..=1. The adapter maps them onto the host's pedals.
    pub throttle: f32,
    pub brake: f32,
    /// Free-form diagnostics for logging; adapters must not act on them.
    pub diag: [f32; 4],
    /// Explicit padding to an 8-byte multiple; zero.
    pub _reserved: u32,
}

/// Actor kinds.
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum ActorKind {
    #[default]
    Unknown = 0,
    Vehicle = 1,
    Pedestrian = 2,
}

/// One actor around the ego car: ground truth from the host. The core's
/// sensor models decide what a sensor would actually report (D18, D25).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Actor {
    /// Host handle, stable while the actor exists.
    pub id: u32,
    pub kind: ActorKind,
    /// Bit i: sensor i of the current [`SensorLayout`] had a clear line of
    /// sight to this actor, as of `los_age` frames ago.
    pub los_mask: u8,
    /// Frames since `los_mask` was refreshed; `u16::MAX` = never checked.
    pub los_age: u16,
    /// Centre of the actor's box relative to the ego reference point (m).
    pub x: f32,
    pub y: f32,
    pub z: f32,
    /// Velocity over ground, rotated into the ego frame (m/s).
    pub vx: f32,
    pub vy: f32,
    /// Heading relative to the ego car (rad, positive left).
    pub yaw: f32,
    /// Yaw rate (rad/s, positive left).
    pub yaw_rate: f32,
    /// Box size (m).
    pub length: f32,
    pub width: f32,
    pub height: f32,
}

impl Actor {
    pub const ZERO: Actor = Actor {
        id: 0,
        kind: ActorKind::Unknown,
        los_mask: 0,
        los_age: u16::MAX,
        x: 0.0,
        y: 0.0,
        z: 0.0,
        vx: 0.0,
        vy: 0.0,
        yaw: 0.0,
        yaw_rate: 0.0,
        length: 0.0,
        width: 0.0,
        height: 0.0,
    };
}

pub const MAX_ACTORS: usize = 64;

/// adapter → core, every frame: the nearest actors, nearest first.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ActorList {
    pub header: Header,
    pub count: u32,
    /// Actors the host knew about but didn't fit (beyond `MAX_ACTORS`).
    pub dropped: u32,
    pub actors: [Actor; MAX_ACTORS],
}

impl Default for ActorList {
    fn default() -> Self {
        Self { header: Header::default(), count: 0, dropped: 0, actors: [Actor::ZERO; MAX_ACTORS] }
    }
}

impl ActorList {
    pub fn as_slice(&self) -> &[Actor] {
        &self.actors[..(self.count as usize).min(MAX_ACTORS)]
    }
}

/// One sensor for which the adapter should run line-of-sight checks.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SensorMount {
    /// Mount position in the ego frame (m).
    pub x: f32,
    pub y: f32,
    pub z: f32,
    /// Boresight, CCW from forward (rad).
    pub facing: f32,
    /// Half field of view (rad).
    pub half_fov: f32,
    /// Range (m).
    pub range: f32,
    /// Desired line-of-sight refresh period per actor (frames).
    pub refresh_frames: f32,
    pub _reserved: u32,
}

pub const MAX_SENSORS: usize = 8;

/// core → adapter (latest wins): the standing line-of-sight queries.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SensorLayout {
    pub header: Header,
    pub count: u32,
    pub _reserved: u32,
    pub mounts: [SensorMount; MAX_SENSORS],
}

impl SensorLayout {
    pub fn as_slice(&self) -> &[SensorMount] {
        &self.mounts[..(self.count as usize).min(MAX_SENSORS)]
    }
}

/// One ultrasonic-style range sensor: a fan of `rays` rays from one mount,
/// spread evenly over ±`half_spread` around `facing` (with an odd count, one
/// ray points along `facing`). The host reports, per ray, the distance to the
/// first solid thing it hits: vehicles, walls, poles, anything.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RangeFan {
    /// Mount position in the ego frame (m); `z` is the height of the rays.
    pub x: f32,
    pub y: f32,
    pub z: f32,
    /// Fan centre, CCW from forward (rad).
    pub facing: f32,
    /// Half the fan's spread (rad).
    pub half_spread: f32,
    /// Ray length (m).
    pub range: f32,
    /// Rays in the fan, 1..=[`MAX_FAN_RAYS`].
    pub rays: u32,
    /// Desired refresh period per ray (frames).
    pub refresh_frames: f32,
}

pub const MAX_FANS: usize = 16;
pub const MAX_FAN_RAYS: usize = 4;

/// Fan layout flags.
pub mod fan_flags {
    /// The core wants the fans answered now. Ultrasonics run only at parking
    /// speeds, so hosts don't spend rays at speed.
    pub const ACTIVE: u32 = 1 << 0;
}

/// core → adapter (latest wins): the standing range fans.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FanLayout {
    pub header: Header,
    pub count: u32,
    pub flags: u32,
    pub fans: [RangeFan; MAX_FANS],
}

impl FanLayout {
    pub fn as_slice(&self) -> &[RangeFan] {
        &self.fans[..(self.count as usize).min(MAX_FANS)]
    }

    /// The direction of ray `i` of fan `f` (rad, CCW from forward).
    pub fn ray_angle(f: &RangeFan, i: usize) -> f32 {
        let n = f.rays.clamp(1, MAX_FAN_RAYS as u32) as usize;
        if n == 1 { f.facing } else { f.facing - f.half_spread + 2.0 * f.half_spread * i as f32 / (n - 1) as f32 }
    }
}

/// One fan's answer.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FanResult {
    /// Distance along each ray to the first hit (m); infinite = nothing within
    /// range; NaN = not measured.
    pub hits: [f32; MAX_FAN_RAYS],
    /// Frames since the oldest ray of this fan was measured.
    pub age: u32,
    pub _reserved: u32,
}

impl Default for FanResult {
    fn default() -> Self {
        Self { hits: [f32::NAN; MAX_FAN_RAYS], age: u32::MAX, _reserved: 0 }
    }
}

/// adapter → core, every frame while the fans are active: one result per fan
/// of the current [`FanLayout`], in the same order.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RangeReport {
    pub header: Header,
    pub count: u32,
    pub _reserved: u32,
    pub fans: [FanResult; MAX_FANS],
}

impl RangeReport {
    pub fn as_slice(&self) -> &[FanResult] {
        &self.fans[..(self.count as usize).min(MAX_FANS)]
    }
}

/// Capability bits in [`AdapterInfo::capabilities`].
pub mod caps {
    pub const STEER: u32 = 1 << 0;
    pub const PEDALS: u32 = 1 << 1;
    pub const HOLD: u32 = 1 << 2;
    pub const WHEEL_SPEEDS: u32 = 1 << 3;
    pub const ACTORS: u32 = 1 << 4;
    pub const LINE_OF_SIGHT: u32 = 1 << 5;
    pub const DRIVER_INPUT: u32 = 1 << 6;
    /// The host answers the standing range fans ([`FanLayout`], [`RangeReport`]).
    pub const RANGE_FANS: u32 = 1 << 7;
    /// The sample carries the turn signal (`sample_flags::TURN_*`).
    pub const TURN_SIGNALS: u32 = 1 << 8;
    /// The sample says when reverse is selected (`sample_flags::REVERSE`).
    pub const GEAR: u32 = 1 << 9;
}

/// adapter → core (latest wins): who the adapter is and what it can do.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AdapterInfo {
    pub header: Header,
    /// UTF-8, zero-padded.
    pub host: [u8; 32],
    pub vehicle: [u8; 32],
    pub capabilities: u32,
    pub _reserved: u32,
    /// The ego vehicle's box relative to its reference point, ISO frame (m).
    pub box_min: [f32; 3],
    pub box_max: [f32; 3],
    /// Road-wheel angle at full lock (rad); NaN if unknown.
    pub max_steer_angle: f32,
    /// NaN if unknown.
    pub wheelbase: f32,
}

/// HUD flags.
pub mod hud_flags {
    /// ACC is engaged.
    pub const ACC: u32 = 1 << 0;
    /// ACC is following a lead (not cruising at the set speed).
    pub const FOLLOWING: u32 = 1 << 1;
    /// A lead is detected.
    pub const LEAD: u32 = 1 << 2;
    pub const FCW: u32 = 1 << 3;
    pub const AEB: u32 = 1 << 4;
    /// Holding the car at a standstill.
    pub const HOLD: u32 = 1 << 5;
    /// The driver's accelerator is overriding ACC.
    pub const OVERRIDE: u32 = 1 << 6;
    /// The selected feature is unavailable (see `Hud::message`).
    pub const UNAVAILABLE: u32 = 1 << 7;
    /// AEB is unavailable.
    pub const AEB_OFF: u32 = 1 << 8;
    /// Take over: the engaged feature can't continue (D11). Show it at once.
    pub const TAKEOVER: u32 = 1 << 9;
    /// The takeover request has escalated: chime.
    pub const TAKEOVER_CHIME: u32 = 1 << 10;
    /// Not answered: the car is making a controlled stop, hazards on.
    pub const CONTROLLED_STOP: u32 = 1 << 11;
}

/// core → adapter (latest wins): what the driver should see. The adapter only
/// draws it (D8).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Hud {
    pub header: Header,
    pub flags: u32,
    /// ACC set speed (m/s).
    pub set_speed: f32,
    /// Selected time gap (s) and its index (0 = shortest).
    pub time_gap: f32,
    pub gap_index: u32,
    /// Gap to the lead (m) and its speed (m/s); NaN without one.
    pub lead_gap: f32,
    pub lead_speed: f32,
    /// Time to collision with the most threatening object (s); infinite if none.
    pub ttc: f32,
    /// Why the selected feature is unavailable (a [`hud_message`] code; 0 = none).
    pub message: u32,
    /// Blind spot, cross traffic and park assist ([`surround_flags`]).
    pub surround: u32,
    /// Park assist tone: beeps per second (0 = silent, infinite = continuous).
    pub park_tone: f32,
    /// Park assist: the distance each ultrasonic sensor reports (m), in the
    /// core's sensor order (front, left corner to right corner, then rear,
    /// left corner to right corner); infinite = clear, NaN = not shown.
    pub park: [f32; 12],
}

/// Surround flags in [`Hud::surround`]: blind spot (BSD), rear cross traffic
/// (RCTA) and park assist.
pub mod surround_flags {
    /// Blind spot monitoring is active (above its minimum speed).
    pub const BSD_ACTIVE: u32 = 1 << 0;
    /// A vehicle in the left / right blind spot: the mirror icon, steady.
    pub const BSD_LEFT: u32 = 1 << 1;
    pub const BSD_RIGHT: u32 = 1 << 2;
    /// Signalling toward an occupied side: flash the icon and chime.
    pub const BSD_WARN_LEFT: u32 = 1 << 3;
    pub const BSD_WARN_RIGHT: u32 = 1 << 4;
    /// Rear cross-traffic alert is active (reversing).
    pub const RCTA_ACTIVE: u32 = 1 << 5;
    /// Crossing traffic approaching from the left / right: warn and chime.
    pub const RCTA_LEFT: u32 = 1 << 6;
    pub const RCTA_RIGHT: u32 = 1 << 7;
    /// Rear cross-traffic braking.
    pub const RCTA_BRAKE: u32 = 1 << 8;
    /// Park assist is showing its distances (`Hud::park`).
    pub const PARK_ACTIVE: u32 = 1 << 9;
    /// Park assist braked for an obstacle.
    pub const PARK_BRAKE: u32 = 1 << 10;
    /// Unavailable (D21): shown greyed out with an "off" mark.
    pub const BSD_OFF: u32 = 1 << 11;
    pub const RCTA_OFF: u32 = 1 << 12;
    pub const PARK_OFF: u32 = 1 << 13;
}

/// HUD message codes (D21): why a feature is unavailable. Adapters show the
/// text; the core decides the code.
pub mod hud_message {
    pub const NONE: u32 = 0;
    pub const STARTING_UP: u32 = 1;
    pub const NO_PEDALS: u32 = 2;
    pub const NO_RADAR_DATA: u32 = 3;
    pub const NO_STANDSTILL_HOLD: u32 = 4;
    pub const NO_STEERING: u32 = 5;
    pub const VEHICLE_DATA_LOST: u32 = 6;
    pub const RADAR_BLOCKED: u32 = 7;
    pub const SIDE_RADAR_BLOCKED: u32 = 8;
    pub const NO_PARK_SENSORS: u32 = 9;
    pub const PARK_SENSOR_FAULT: u32 = 10;
    pub const NO_GEAR: u32 = 11;
    pub const REVERSING: u32 = 12;

    pub fn text(code: u32) -> &'static str {
        match code {
            STARTING_UP => "starting up",
            NO_PEDALS => "no pedal control on this host",
            NO_RADAR_DATA => "no radar data on this host",
            NO_STANDSTILL_HOLD => "not available below 30 km/h (no standstill hold)",
            NO_STEERING => "no steering control on this host",
            VEHICLE_DATA_LOST => "vehicle data lost",
            RADAR_BLOCKED => "radar blocked",
            SIDE_RADAR_BLOCKED => "side radar blocked",
            NO_PARK_SENSORS => "no parking sensors on this host",
            PARK_SENSOR_FAULT => "parking sensor fault",
            NO_GEAR => "no gear signal on this host",
            REVERSING => "not available in reverse",
            _ => "",
        }
    }
}

/// Copies `s` into a zero-padded fixed-size field (truncating on a char boundary).
pub fn fixed_str<const N: usize>(s: &str) -> [u8; N] {
    let mut out = [0u8; N];
    let mut end = s.len().min(N - 1);
    while !s.is_char_boundary(end) {
        end -= 1;
    }
    out[..end].copy_from_slice(&s.as_bytes()[..end]);
    out
}

/// Reads a zero-padded fixed-size string field.
pub fn str_of(b: &[u8]) -> &str {
    let end = b.iter().position(|&c| c == 0).unwrap_or(b.len());
    core::str::from_utf8(&b[..end]).unwrap_or("?")
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
        // header + flags + 5 scalars + wheel_omega + rev_ratio + gear + 3 driver + buttons
        assert_eq!(size_of::<VehicleSample>(), 32 + 4 + 5 * 4 + 16 + 4 + 4 + 3 * 4 + 4);
        assert_eq!(size_of::<Command>(), 32 + 4 + 4 + 3 * 4 + 16 + 4);
        assert_eq!(size_of::<Actor>(), 4 + 1 + 1 + 2 + 10 * 4);
        assert_eq!(size_of::<ActorList>(), 32 + 8 + MAX_ACTORS * 48);
        assert_eq!(size_of::<SensorMount>(), 32);
        assert_eq!(size_of::<SensorLayout>(), 32 + 8 + MAX_SENSORS * 32);
        assert_eq!(size_of::<AdapterInfo>(), 32 + 64 + 8 + 24 + 8);
        assert_eq!(size_of::<Hud>(), 32 + 32 + 8 + 48);
        assert_eq!(size_of::<RangeFan>(), 32);
        assert_eq!(size_of::<FanLayout>(), 32 + 8 + MAX_FANS * 32);
        assert_eq!(size_of::<FanResult>(), 24);
        assert_eq!(size_of::<RangeReport>(), 32 + 8 + MAX_FANS * 24);
    }

    #[test]
    fn fan_rays_spread_evenly() {
        let f = RangeFan { facing: 1.0, half_spread: 0.5, rays: 3, ..Default::default() };
        let a: Vec<f32> = (0..3).map(|i| FanLayout::ray_angle(&f, i)).collect();
        assert_eq!(a, vec![0.5, 1.0, 1.5]);
        assert_eq!(FanLayout::ray_angle(&RangeFan { facing: 0.3, rays: 1, ..f }, 0), 0.3);
    }

    #[test]
    fn fixed_strings() {
        let b: [u8; 8] = fixed_str("GTA V Enhanced");
        assert_eq!(str_of(&b), "GTA V E");
        let b: [u8; 8] = fixed_str("ab");
        assert_eq!(str_of(&b), "ab");
        let b: [u8; 4] = fixed_str("éé");
        assert_eq!(str_of(&b), "é");
    }
}
