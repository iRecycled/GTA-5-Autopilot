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

pub mod radar;
pub mod ultrasonic;

pub use radar::{FrontRadar, Radar, RadarConfig, RadarObject, RadarScan, nearest_box_point};
pub use ultrasonic::{ULTRASONIC_COUNT, UltrasonicConfig, UltrasonicScan, Ultrasonics};

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
