//! Tracking and lead selection.
//!
//! Each radar object id gets a track: two constant-velocity Kalman filters (x
//! and y, each estimating position and over-ground velocity), predicted every
//! frame with ego-motion compensation and corrected at each radar scan. A track
//! is confirmed after a few hits and deleted after a few misses.
//!
//! The lead is the nearest confirmed track inside the lane along the ego car's
//! predicted path (curvature from yaw rate ÷ speed).

use sensors::RadarScan;

pub mod surround;
pub use surround::{SurroundConfig, SurroundTrack, SurroundTracker};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TrackerConfig {
    /// Process noise: unmodelled acceleration (m/s², 1σ).
    pub accel_noise: f32,
    pub pos_noise_m: f32,
    pub pos_noise_frac: f32,
    pub vel_noise: f32,
    /// Hits to confirm a track.
    pub confirm_hits: u32,
    /// Consecutive missed scans to delete a confirmed track.
    pub max_misses: u32,
}

impl Default for TrackerConfig {
    fn default() -> Self {
        Self { accel_noise: 3.0, pos_noise_m: 0.10, pos_noise_frac: 0.005, vel_noise: 0.10, confirm_hits: 3, max_misses: 5 }
    }
}

/// A 1-D constant-velocity Kalman filter: state (p, v), covariance P.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Kf {
    p: f32,
    v: f32,
    pp: f32,
    pv: f32,
    vv: f32,
}

impl Kf {
    pub(crate) fn new(p: f32, v: f32, rp: f32, rv: f32) -> Self {
        Self { p, v, pp: rp, pv: 0.0, vv: rv }
    }
    pub(crate) fn predict(&mut self, dt: f32, q: f32) {
        self.p += self.v * dt;
        let (pp, pv, vv) = (self.pp, self.pv, self.vv);
        // P = F P Fᵀ + Q, with Q for white acceleration noise of variance q.
        self.pp = pp + 2.0 * dt * pv + dt * dt * vv + q * dt.powi(4) / 4.0;
        self.pv = pv + dt * vv + q * dt.powi(3) / 2.0;
        self.vv = vv + q * dt * dt;
    }
    /// Measures position and velocity directly (H = I).
    pub(crate) fn update(&mut self, zp: f32, zv: f32, rp: f32, rv: f32) {
        let (pp, pv, vv) = (self.pp, self.pv, self.vv);
        // S = P + R; K = P S⁻¹ (2×2).
        let (s11, s12, s22) = (pp + rp, pv, vv + rv);
        let det = s11 * s22 - s12 * s12;
        if det.abs() < 1e-12 {
            return;
        }
        let (i11, i12, i22) = (s22 / det, -s12 / det, s11 / det);
        let (k11, k12) = (pp * i11 + pv * i12, pp * i12 + pv * i22);
        let (k21, k22) = (pv * i11 + vv * i12, pv * i12 + vv * i22);
        let (ep, ev) = (zp - self.p, zv - self.v);
        self.p += k11 * ep + k12 * ev;
        self.v += k21 * ep + k22 * ev;
        // P = (I − K) P
        self.pp = (1.0 - k11) * pp - k12 * pv;
        self.pv = (1.0 - k11) * pv - k12 * vv;
        self.vv = -k21 * pv + (1.0 - k22) * vv;
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Track {
    pub id: u32,
    x: Kf,
    y: Kf,
    pub width: f32,
    pub half_length: f32,
    pub hits: u32,
    pub misses: u32,
    pub confirmed: bool,
    /// Seen moving (> 2 m/s over ground) at some point while tracked.
    pub ever_moving: bool,
    /// Over-ground acceleration estimate along x (m/s², low-pass).
    pub ax: f32,
    prev_vx: f32,
}

impl Track {
    pub fn x(&self) -> f32 {
        self.x.p
    }
    pub fn y(&self) -> f32 {
        self.y.p
    }
    pub fn vx(&self) -> f32 {
        self.x.v
    }
    pub fn vy(&self) -> f32 {
        self.y.v
    }
    pub fn stationary(&self) -> bool {
        self.x.v.hypot(self.y.v) < 0.5
    }
}

#[derive(Clone, Debug, Default)]
pub struct Tracker {
    pub cfg: TrackerConfig,
    tracks: Vec<Track>,
}

impl Tracker {
    pub fn new(cfg: TrackerConfig) -> Self {
        Self { cfg, tracks: Vec::new() }
    }

    pub fn tracks(&self) -> &[Track] {
        &self.tracks
    }

    pub fn clear(&mut self) {
        self.tracks.clear();
    }

    /// Moves every track by `dt`, compensating the ego car's own motion: the
    /// frame translates by `ego_vx·dt` and turns by `yaw_rate·dt`.
    pub fn predict(&mut self, dt: f32, ego_vx: f32, yaw_rate: f32) {
        if dt <= 0.0 {
            return;
        }
        let q = self.cfg.accel_noise.powi(2);
        let (s, c) = (-yaw_rate * dt).sin_cos();
        for t in &mut self.tracks {
            // Over-ground motion, then the ego frame's own translation.
            t.x.predict(dt, q);
            t.y.predict(dt, q);
            t.x.p -= ego_vx * dt;
            // Rotate position and velocity into the turned ego frame.
            let (px, py) = (t.x.p, t.y.p);
            t.x.p = c * px - s * py;
            t.y.p = s * px + c * py;
            let (vx, vy) = (t.x.v, t.y.v);
            t.x.v = c * vx - s * vy;
            t.y.v = s * vx + c * vy;
        }
    }

    /// Corrects with a radar scan (after `predict` for the same frame).
    pub fn update(&mut self, scan: &RadarScan, dt_since_last_scan: f32) {
        let c = self.cfg;
        for o in &scan.objects {
            let r = o.x.hypot(o.y);
            let rp = (c.pos_noise_m + c.pos_noise_frac * r).powi(2);
            let rv = c.vel_noise.powi(2);
            match self.tracks.iter_mut().find(|t| t.id == o.id) {
                Some(t) => {
                    t.x.update(o.x, o.vx, rp, rv);
                    t.y.update(o.y, o.vy, rp, rv);
                    t.width = o.width;
                    t.half_length = o.half_length;
                    t.hits += 1;
                    t.misses = 0;
                    if t.hits >= c.confirm_hits {
                        t.confirmed = true;
                    }
                    if t.vx().hypot(t.vy()) > 2.0 {
                        t.ever_moving = true;
                    }
                    if dt_since_last_scan > 0.0 {
                        let a = (t.vx() - t.prev_vx) / dt_since_last_scan;
                        t.ax += 0.3 * (a - t.ax);
                    }
                    t.prev_vx = t.vx();
                }
                None => self.tracks.push(Track {
                    id: o.id,
                    x: Kf::new(o.x, o.vx, rp, rv),
                    y: Kf::new(o.y, o.vy, rp, rv),
                    width: o.width,
                    half_length: o.half_length,
                    hits: 1,
                    misses: 0,
                    confirmed: c.confirm_hits <= 1,
                    ever_moving: o.vx.hypot(o.vy) > 2.0,
                    ax: 0.0,
                    prev_vx: o.vx,
                }),
            }
        }
        // Misses: confirmed tracks survive a few; tentative ones don't.
        for t in &mut self.tracks {
            if !scan.objects.iter().any(|o| o.id == t.id) {
                t.misses += 1;
            }
        }
        self.tracks.retain(|t| if t.confirmed { t.misses < c.max_misses } else { t.misses < 2 });
    }
}

/// The vehicle the ego car follows or might hit.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Lead {
    pub id: u32,
    /// Bumper-to-bumper gap (m): the track's near face minus the ego front.
    pub gap: f32,
    /// Lead speed over ground along the ego x axis (m/s).
    pub v: f32,
    /// Lead acceleration estimate (m/s²).
    pub a: f32,
    /// Lateral offset from the predicted path (m).
    pub offset: f32,
    pub stationary: bool,
}

/// What may be selected.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LeadKind {
    /// ACC: moving vehicles, and stopped ones that were seen moving.
    Follow,
    /// AEB: anything confirmed in the path, including stationary objects.
    Collision,
}

/// Nearest confirmed track in the lane along the predicted path.
pub fn select_lead(tracks: &[Track], ego_speed: f32, yaw_rate: f32, ego_front_x: f32, half_lane: f32, kind: LeadKind) -> Option<Lead> {
    let k = (yaw_rate / ego_speed.max(2.0)).clamp(-0.1, 0.1);
    tracks
        .iter()
        .filter(|t| t.confirmed && t.x() > ego_front_x - 0.5)
        .filter(|t| match kind {
            LeadKind::Follow => !t.stationary() || t.ever_moving,
            LeadKind::Collision => true,
        })
        .filter_map(|t| {
            let path_y = 0.5 * k * t.x() * t.x();
            let offset = t.y() - path_y;
            (offset.abs() <= half_lane + 0.5 * t.width.min(2.5) - 0.3).then_some(Lead {
                id: t.id,
                gap: t.x() - ego_front_x,
                v: t.vx(),
                a: t.ax,
                offset,
                stationary: t.stationary(),
            })
        })
        .min_by(|a, b| a.gap.total_cmp(&b.gap))
}

#[cfg(test)]
mod tests {
    use super::*;
    use sensors::RadarObject;

    fn obj(id: u32, x: f32, y: f32, vx: f32) -> RadarObject {
        RadarObject { id, x, y, vx, vy: 0.0, width: 1.9, half_length: 2.25, stationary: vx.abs() < 0.5, clutter: false }
    }

    #[test]
    fn kalman_converges_on_a_moving_target() {
        let mut t = Tracker::new(TrackerConfig::default());
        // Ego at 20 m/s, lead 40 m ahead at 15 m/s: the gap closes at 5 m/s.
        let mut x = 40.0f32;
        for i in 0..100 {
            t.predict(0.05, 20.0, 0.0);
            x -= 5.0 * 0.05;
            let n = if i % 2 == 0 { 0.2 } else { -0.2 };
            t.update(&RadarScan { scan: i, objects: vec![obj(1, x + n, 0.0, 15.0)] }, 0.05);
        }
        let tr = &t.tracks()[0];
        assert!(tr.confirmed);
        assert!((tr.x() - x).abs() < 0.2, "x {} vs {x}", tr.x());
        assert!((tr.vx() - 15.0).abs() < 0.2, "vx {}", tr.vx());
    }

    #[test]
    fn tracks_confirm_and_expire() {
        let mut t = Tracker::new(TrackerConfig::default());
        for i in 0..2 {
            t.predict(0.05, 0.0, 0.0);
            t.update(&RadarScan { scan: i, objects: vec![obj(5, 30.0, 0.0, 0.0)] }, 0.05);
        }
        assert!(!t.tracks()[0].confirmed, "confirmed after only two hits");
        t.update(&RadarScan { scan: 2, objects: vec![obj(5, 30.0, 0.0, 0.0)] }, 0.05);
        assert!(t.tracks()[0].confirmed);
        for i in 3..8 {
            t.update(&RadarScan { scan: i, objects: vec![] }, 0.05);
        }
        assert!(t.tracks().is_empty(), "a track survived five misses");
    }

    #[test]
    fn lead_selection_follows_the_path() {
        let mut t = Tracker::new(TrackerConfig::default());
        for i in 0..4 {
            t.update(
                &RadarScan {
                    scan: i,
                    objects: vec![obj(1, 60.0, 0.2, 20.0), obj(2, 30.0, -3.6, 20.0), obj(3, 45.0, 0.0, 0.0), obj(4, 50.0, 6.0, 20.0)],
                },
                0.05,
            );
        }
        // Straight ahead: the car in the next lane (2) is ignored; the stopped
        // object (3) is ignored by ACC but not by AEB.
        let f = select_lead(t.tracks(), 20.0, 0.0, 2.0, 1.8, LeadKind::Follow).unwrap();
        assert_eq!(f.id, 1);
        assert!((f.gap - 58.0).abs() < 0.01);
        let c = select_lead(t.tracks(), 20.0, 0.0, 2.0, 1.8, LeadKind::Collision).unwrap();
        assert_eq!(c.id, 3);
        // Turning left (path bends toward +y): at x = 50 the path is at
        // 0.5·(0.0048)·2500 = 6 m, so object 4 becomes the lead.
        let k_yaw = 0.0048 * 20.0;
        let l = select_lead(t.tracks(), 20.0, k_yaw, 2.0, 1.8, LeadKind::Follow).unwrap();
        assert_eq!(l.id, 4);
    }

    #[test]
    fn ego_motion_compensation() {
        let mut t = Tracker::new(TrackerConfig::default());
        t.update(&RadarScan { scan: 0, objects: vec![obj(1, 20.0, 0.0, 0.0)] }, 0.05);
        // Ego drives 10 m/s for 1 s toward a stopped object.
        for _ in 0..20 {
            t.predict(0.05, 10.0, 0.0);
        }
        assert!((t.tracks()[0].x() - 10.0).abs() < 1e-3);
    }
}
