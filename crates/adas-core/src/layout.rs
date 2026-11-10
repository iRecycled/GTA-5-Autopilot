//! The sensor layout the core asks the adapter to run line-of-sight checks
//! for, and the range fans for the ultrasonics (the "standing queries").
//! Ranges and fields of view follow the PRD's sensor table (D25).

use adapter_api::{FanLayout, MAX_FANS, MAX_SENSORS, RangeFan, SensorLayout, SensorMount};

/// The corner radars' indices, in the order of `vbus::CORNER_IDS`.
pub const CORNERS: [usize; 4] = [CORNER_FL, CORNER_FR, CORNER_RL, CORNER_RR];

/// Twelve ultrasonic sensors, six per bumper (left corner → right corner,
/// front then rear): 3 rays each over ±25° (±30° at the corners), 4.5 m
/// (2.5 m at the corners), every 5 frames, at bumper height (0.5 m above the
/// bottom of the box).
pub fn default_fans(box_min: [f32; 3], box_max: [f32; 3]) -> FanLayout {
    let (front, rear, left, right) = (box_max[0], box_min[0], box_max[1], box_min[1]);
    let z = box_min[2] + 0.5;
    let deg = |d: f32| d.to_radians();
    let fan = |x: f32, y: f32, facing: f32, corner: bool| RangeFan {
        x,
        y,
        z,
        facing: deg(facing),
        half_spread: deg(if corner { 30.0 } else { 25.0 }),
        range: if corner { 2.5 } else { 4.5 },
        rays: 3,
        refresh_frames: 5.0,
    };
    // Lateral positions across a bumper, left to right, and their facings.
    let ys = [left - 0.1, 0.6 * left, 0.2 * left, 0.2 * right, 0.6 * right, right + 0.1];
    let front_facing = [75.0, 25.0, 5.0, -5.0, -25.0, -75.0];
    let mut l = FanLayout { count: 12, ..Default::default() };
    for i in 0..6 {
        let corner = i == 0 || i == 5;
        l.fans[i] = fan(front, ys[i], front_facing[i], corner);
        // Rear: mirrored about the y axis (facing 180° − front facing).
        let f = front_facing[i];
        l.fans[6 + i] = fan(rear, ys[i], if f > 0.0 { 180.0 - f } else { -180.0 - f }, corner);
    }
    debug_assert!(l.count as usize <= MAX_FANS);
    l
}

/// Sensor indices in the default layout (bit positions in `Actor::los_mask`).
pub const FRONT_RADAR: usize = 0;
pub const CORNER_FL: usize = 1;
pub const CORNER_FR: usize = 2;
pub const CORNER_RL: usize = 3;
pub const CORNER_RR: usize = 4;
pub const FRONT_CAMERA: usize = 5;

/// Front radar (160 m, ±10°), four corner radars (80 m, ±75°) and the
/// windshield camera (120 m, ±50°, 30 Hz), on a box from `box_min` to
/// `box_max` (ISO, m). Radar refresh: 20 Hz at 72 fps ≈ 3.6 frames.
pub fn default_layout(box_min: [f32; 3], box_max: [f32; 3]) -> SensorLayout {
    let z = 0.5;
    let deg = |d: f32| d.to_radians();
    let mount = |x: f32, y: f32, facing: f32, half_fov: f32, range: f32| SensorMount {
        x,
        y,
        z,
        facing: deg(facing),
        half_fov: deg(half_fov),
        range,
        refresh_frames: 3.6,
        _reserved: 0,
    };
    let (front, rear, left, right) = (box_max[0], box_min[0], box_max[1], box_min[1]);
    let mut l = SensorLayout { count: 6, ..Default::default() };
    l.mounts[FRONT_RADAR] = mount(front, 0.0, 0.0, 10.0, 160.0);
    l.mounts[CORNER_FL] = mount(front, left, 45.0, 75.0, 80.0);
    l.mounts[CORNER_FR] = mount(front, right, -45.0, 75.0, 80.0);
    l.mounts[CORNER_RL] = mount(rear, left, 135.0, 75.0, 80.0);
    l.mounts[CORNER_RR] = mount(rear, right, -135.0, 75.0, 80.0);
    // Behind the windshield, 1.8 m back from the front, 1.3 m up.
    l.mounts[FRONT_CAMERA] = SensorMount { x: front - 1.8, z: box_min[2] + 1.3, refresh_frames: 2.4, ..mount(front - 1.8, 0.0, 0.0, 50.0, 120.0) };
    debug_assert!(l.count as usize <= MAX_SENSORS);
    l
}
