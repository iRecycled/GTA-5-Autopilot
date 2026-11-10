//! Lateral control: pure pursuit on a path given as a cubic in the ego frame
//! (y = c0 + c1·x + c2·x² + c3·x³, the lane centre as a camera reports it).
//!
//! The target is the path point a look-ahead distance ahead (a time headway
//! times speed, within limits); the steering is the arc that reaches it.
//! On a circle the arc is the circle itself, so steady bends need no
//! integrator. The curvature is capped by ISO 11270's 3 m/s² lateral
//! acceleration; the EPS then limits how fast the angle may change.

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LateralConfig {
    /// Look-ahead time (s) and its distance limits (m).
    pub lookahead_time: f32,
    pub lookahead_min: f32,
    pub lookahead_max: f32,
    /// Lateral acceleration cap (m/s²).
    pub max_lat_accel: f32,
}

impl Default for LateralConfig {
    fn default() -> Self {
        Self { lookahead_time: 1.1, lookahead_min: 8.0, lookahead_max: 45.0, max_lat_accel: 3.0 }
    }
}

/// y(x) of a cubic.
pub fn poly(c: &[f32; 4], x: f32) -> f32 {
    c[0] + x * (c[1] + x * (c[2] + x * c[3]))
}

/// The look-ahead distance at `speed` (m).
pub fn lookahead(cfg: &LateralConfig, speed: f32) -> f32 {
    (cfg.lookahead_time * speed).clamp(cfg.lookahead_min, cfg.lookahead_max)
}

/// The curvature (1/m, positive left) that reaches the path's look-ahead
/// point, capped by the lateral acceleration limit. `range`: how far ahead
/// the path is known (the look-ahead is no farther).
pub fn pursuit_curvature(cfg: &LateralConfig, path: &[f32; 4], range: f32, speed: f32) -> f32 {
    let ld = lookahead(cfg, speed).min(range.max(cfg.lookahead_min));
    let y = poly(path, ld);
    let k = 2.0 * y / (ld * ld + y * y);
    let cap = cfg.max_lat_accel / speed.max(1.0).powi(2);
    k.clamp(-cap, cap)
}

/// The road-wheel angle (rad) for a curvature, on a car of this wheelbase.
pub fn angle_for(curvature: f32, wheelbase: f32) -> f32 {
    (wheelbase * curvature).atan()
}

/// The largest |curvature| along the path from `from` to `to` m ahead.
pub fn max_curvature(path: &[f32; 4], from: f32, to: f32) -> f32 {
    let mut k: f32 = 0.0;
    let n = 12;
    for i in 0..=n {
        let x = from + (to - from) * i as f32 / n as f32;
        let (d1, d2) = (path[1] + x * (2.0 * path[2] + 3.0 * x * path[3]), 2.0 * path[2] + 6.0 * path[3] * x);
        k = k.max(d2.abs() / (1.0 + d1 * d1).powf(1.5));
    }
    k
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_circle_needs_its_own_curvature() {
        // Lane centre on a left bend of R = 400 m (y ≈ x²/2R).
        let r = 400.0;
        let path = [0.0, 0.0, 1.0 / (2.0 * r), 0.0];
        let k = pursuit_curvature(&LateralConfig::default(), &path, 80.0, 25.0);
        assert!((k - 1.0 / r).abs() < 0.03 / r, "curvature {k} vs {}", 1.0 / r);
        // Offset 0.5 m to the right of the centre on a straight: steer left, gently.
        let k = pursuit_curvature(&LateralConfig::default(), &[0.5, 0.0, 0.0, 0.0], 80.0, 25.0);
        assert!(k > 0.0 && k < 0.002, "{k}");
        // The lateral acceleration cap: a tight bend at speed.
        let k = pursuit_curvature(&LateralConfig::default(), &[0.0, 0.0, 1.0 / 60.0, 0.0], 80.0, 30.0);
        assert!((k * 900.0 - 3.0).abs() < 1e-3, "capped at 3 m/s²: {}", k * 900.0);
        assert!((max_curvature(&path, 0.0, 60.0) - 1.0 / r).abs() < 1e-5);
    }
}
