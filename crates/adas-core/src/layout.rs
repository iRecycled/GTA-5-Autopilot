//! The sensor layout the core asks the adapter to run line-of-sight checks for
//! (the "standing queries"). Ranges and fields of view follow the PRD's sensor
//! table (D25); the `sensors` crate takes this over in Phase 1.

use adapter_api::{MAX_SENSORS, SensorLayout, SensorMount};

/// Sensor indices in the default layout (bit positions in `Actor::los_mask`).
pub const FRONT_RADAR: usize = 0;
pub const CORNER_FL: usize = 1;
pub const CORNER_FR: usize = 2;
pub const CORNER_RL: usize = 3;
pub const CORNER_RR: usize = 4;

/// Front radar (160 m, ±10°) and four corner radars (80 m, ±75°), on a box
/// from `box_min` to `box_max` (ISO, m). Refresh: 20 Hz at 72 fps ≈ 3.6 frames.
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
    let mut l = SensorLayout { count: 5, ..Default::default() };
    l.mounts[FRONT_RADAR] = mount(front, 0.0, 0.0, 10.0, 160.0);
    l.mounts[CORNER_FL] = mount(front, left, 45.0, 75.0, 80.0);
    l.mounts[CORNER_FR] = mount(front, right, -45.0, 75.0, 80.0);
    l.mounts[CORNER_RL] = mount(rear, left, 135.0, 75.0, 80.0);
    l.mounts[CORNER_RR] = mount(rear, right, -135.0, 75.0, 80.0);
    debug_assert!(l.count as usize <= MAX_SENSORS);
    l
}
