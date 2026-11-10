//! Sensor models (D18, D25).
//!
//! A sensor model turns the host's ground truth (the actor list and its
//! line-of-sight bits) into what a production sensor would report, through an
//! error model with datasheet-style parameters: noise, misses, false targets,
//! update rate and latency. The output is an object list, the same message a
//! real radar ECU puts on the bus.
//!
//! The noise is a pure function of (seed, frame, object, quantity), not a
//! running random generator, so replaying a recording reproduces it exactly.

pub mod camera;
pub mod radar;
pub mod ultrasonic;

pub use radar::{FrontRadar, Radar, RadarConfig, RadarObject, RadarScan, nearest_box_point};
pub use camera::{CamObject, Camera, CameraConfig, CameraFrame, LaneFit};
pub use ultrasonic::{ULTRASONIC_COUNT, UltrasonicConfig, UltrasonicScan, Ultrasonics};

/// The sensor's vertical view reaches part of the actor: its box, from
/// `z ± height/2`, overlaps the cone of half-angle `half_vfov` from the
/// mount's height at that range. A sensor sees only along the road: a bird
/// overhead is not in view (GTA counts birds as pedestrians; a 2026-11-09 run
/// braked for one 47 m up).
pub fn in_vertical_view(a: &adapter_api::Actor, mount_z: f32, range: f32, half_vfov: f32) -> bool {
    let reach = range * half_vfov.tan();
    let h = 0.5 * a.height.max(0.0);
    a.z + h >= mount_z - reach && a.z - h <= mount_z + reach
}

/// A deterministic hash → uniform in [0, 1).
pub fn unit(seed: u64, a: u64, b: u64, c: u64) -> f32 {
    let mut x = seed ^ a.wrapping_mul(0x9E37_79B9_7F4A_7C15) ^ b.wrapping_mul(0xC2B2_AE3D_27D4_EB4F) ^ c.wrapping_mul(0x1656_67B1_9E37_79F9);
    x ^= x >> 33;
    x = x.wrapping_mul(0xFF51_AFD7_ED55_8CCD);
    x ^= x >> 33;
    x = x.wrapping_mul(0xC4CE_B9FE_1A85_EC53);
    x ^= x >> 33;
    (x >> 40) as f32 / (1u64 << 24) as f32
}

/// A deterministic standard normal sample (Box–Muller from two hashes).
pub fn normal(seed: u64, a: u64, b: u64, c: u64) -> f32 {
    let u1 = unit(seed, a, b, c.wrapping_mul(2)).max(1e-7);
    let u2 = unit(seed, a, b, c.wrapping_mul(2) + 1);
    (-2.0 * u1.ln()).sqrt() * (core::f32::consts::TAU * u2).cos()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_bird_overhead_is_out_of_view() {
        let ped = |z: f32| adapter_api::Actor { kind: adapter_api::ActorKind::Pedestrian, x: 25.0, z, height: 1.8, ..adapter_api::Actor::ZERO };
        let v = 25f32.to_radians();
        // On the road, and up a 10 % slope 25 m on: in view.
        assert!(in_vertical_view(&ped(0.0), 1.3, 25.0, v));
        assert!(in_vertical_view(&ped(2.5), 1.3, 25.0, v));
        // The seagull from the GTA run: 47 m up at 26 m.
        assert!(!in_vertical_view(&ped(47.3), 1.3, 26.0, v));
        // A tall truck right in front still is.
        let truck = adapter_api::Actor { z: 1.0, height: 4.0, ..ped(0.0) };
        assert!(in_vertical_view(&truck, 0.5, 2.0, 10f32.to_radians()));
    }

    #[test]
    fn noise_is_deterministic_and_roughly_standard() {
        assert_eq!(normal(1, 2, 3, 4), normal(1, 2, 3, 4));
        let xs: Vec<f32> = (0..20_000).map(|i| normal(7, i, 0, 0)).collect();
        let mean = xs.iter().sum::<f32>() / xs.len() as f32;
        let var = xs.iter().map(|x| (x - mean).powi(2)).sum::<f32>() / xs.len() as f32;
        assert!(mean.abs() < 0.03, "mean {mean}");
        assert!((var - 1.0).abs() < 0.05, "variance {var}");
        let us: Vec<f32> = (0..10_000).map(|i| unit(3, i, 1, 2)).collect();
        assert!(us.iter().all(|u| (0.0..1.0).contains(u)));
    }
}
