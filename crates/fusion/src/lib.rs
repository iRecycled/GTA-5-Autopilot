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
    /// Camera frames in a row that saw a vehicle here, moving as this track
    /// moves (radar–camera fusion), and the time since the last (s).
    camera_hits: u32,
    since_camera: f32,
}

/// Camera frames that must agree before a track counts as a vehicle.
const CAMERA_CONFIRM: u32 = 2;
/// A camera confirmation counts this long (s) without being renewed.
const CAMERA_HOLD: f32 = 1.0;
/// The most a camera vehicle's velocity may differ from the track's (m/s,
/// each axis): the camera's velocity noise (0.4 m/s) with room to spare.
const CAMERA_VEL_GATE: f32 = 2.0;

impl Track {
    /// The camera sees a vehicle here (radar–camera fusion): a stopped one
    /// is then a car to stop behind, not a sign or a bridge. It must have
    /// agreed for a couple of frames, and lapses a second after the camera
    /// stops confirming it.
    pub fn camera_vehicle(&self) -> bool {
        self.camera_hits >= CAMERA_CONFIRM && self.since_camera <= CAMERA_HOLD
    }
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

    /// Radar–camera fusion: one camera frame's vehicle at (`x`, `y`), moving
    /// at (`vx`, `vy`) over ground, confirms the nearest confirmed track
    /// within the camera's range error (1.5 m plus 8 % of the range along,
    /// 1.5 m across) that moves as it does. A car passing a stationary
    /// return doesn't make that return a car. See [`Track::camera_vehicle`].
    pub fn confirm_vehicle(&mut self, x: f32, y: f32, vx: f32, vy: f32) {
        let gate_x = 1.5 + 0.08 * x.abs();
        if let Some(t) = self
            .tracks
            .iter_mut()
            .filter(|t| t.confirmed && (t.x() - x).abs() <= gate_x + t.half_length && (t.y() - y).abs() <= 1.5)
            .filter(|t| (t.vx() - vx).abs() <= CAMERA_VEL_GATE && (t.vy() - vy).abs() <= CAMERA_VEL_GATE)
            .min_by(|a, b| (a.x() - x).hypot(a.y() - y).total_cmp(&(b.x() - x).hypot(b.y() - y)))
        {
            t.camera_hits = if t.since_camera <= CAMERA_HOLD { t.camera_hits.saturating_add(1) } else { 1 };
            t.since_camera = 0.0;
        }
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
            t.since_camera += dt;
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
                    camera_hits: 0,
                    since_camera: f32::INFINITY,
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
    /// ACC: moving vehicles, and stopped ones that were seen moving or that
    /// the camera confirms are vehicles.
    Follow,
    /// AEB: anything confirmed in the path, including stationary objects.
    Collision,
}

/// Nearest confirmed track in the lane along the predicted path.
pub fn select_lead(tracks: &[Track], ego_speed: f32, yaw_rate: f32, ego_front_x: f32, half_lane: f32, kind: LeadKind) -> Option<Lead> {
    let k = (yaw_rate / ego_speed.max(2.0)).clamp(-0.1, 0.1);
    select_on(tracks, ego_speed, ego_front_x, half_lane, kind, |_| true, |x| 0.5 * k * x * x)
}

/// As [`select_lead`], with the path given (Phase 3: the lane centre the
/// camera sees, a cubic in the ego frame) instead of predicted from the yaw
/// rate, and `half_lane` that lane's half width. Oncoming traffic (closing
/// faster than 2 m/s over ground) is left out: across a whole lane, in a
/// bend, the oncoming lane's cars pass close to it. AEB pairs this with
/// [`select_in_body_path`], which keeps them.
pub fn select_lead_in_lane(tracks: &[Track], ego_speed: f32, lane: &[f32; 4], ego_front_x: f32, half_lane: f32, kind: LeadKind) -> Option<Lead> {
    let not_oncoming = |t: &Track| t.vx() >= -2.0;
    select_on(tracks, ego_speed, ego_front_x, half_lane, kind, not_oncoming, |x| lane[0] + x * (lane[1] + x * (lane[2] + x * lane[3])))
}

/// For AEB: the nearest confirmed track whose footprint overlaps the car's
/// own along the path the yaw rate predicts, oncoming or not, whatever the
/// lanes say. With lanes, AEB brakes for the nearer of this and the lane's
/// collision lead, so neither a lane the car has left nor a lane it is
/// halfway out of can hide what is straight ahead.
pub fn select_in_body_path(tracks: &[Track], ego_speed: f32, yaw_rate: f32, ego_front_x: f32, ego_half_width: f32) -> Option<Lead> {
    // `select_on` reaches half_lane + the track's half width − 0.3 m; the
    // 0.3 m back makes it the footprints touching.
    select_lead(tracks, ego_speed, yaw_rate, ego_front_x, ego_half_width + 0.3, LeadKind::Collision)
}

/// The nearer of two leads.
pub fn nearer(a: Option<Lead>, b: Option<Lead>) -> Option<Lead> {
    match (a, b) {
        (Some(a), Some(b)) => Some(if b.gap < a.gap { b } else { a }),
        (a, b) => a.or(b),
    }
}

/// A track crossing the path (≥ 2 m/s across it, at ≥ 30° to it: not a
/// cut-in) that will have left the lane by the time the car gets there. AEB
/// leaves it to junction AEB, which times both: braking for a car that
/// sweeps through the lane far ahead is a false activation.
fn clears_before_arrival(t: &Track, ego_speed: f32, gap: f32, offset: f32, reach: f32) -> bool {
    let (vx, vy) = (t.vx(), t.vy());
    if vy.abs() < 2.0 || vy.abs() < vx.abs() * 30f32.to_radians().tan() {
        return false;
    }
    let closing = (ego_speed - vx).max(0.1);
    let arrive = (gap.max(0.0) / closing).min(4.0);
    let later = offset + vy * arrive;
    later.abs() > reach && later * vy > 0.0
}

fn select_on(tracks: &[Track], ego_speed: f32, ego_front_x: f32, half_lane: f32, kind: LeadKind, keep: impl Fn(&Track) -> bool, path: impl Fn(f32) -> f32) -> Option<Lead> {
    tracks
        .iter()
        .filter(|t| t.confirmed && t.x() > ego_front_x - 0.5 && keep(t))
        .filter(|t| match kind {
            LeadKind::Follow => !t.stationary() || t.ever_moving || t.camera_vehicle(),
            LeadKind::Collision => true,
        })
        .filter_map(|t| {
            let path_y = path(t.x());
            let offset = t.y() - path_y;
            let reach = half_lane + 0.5 * t.width.min(2.5) - 0.3;
            let gap = t.x() - ego_front_x;
            if kind == LeadKind::Collision && clears_before_arrival(t, ego_speed, gap, offset, reach) {
                return None;
            }
            (offset.abs() <= reach).then_some(Lead {
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

/// The pedestrian the car would hit: the nearest confirmed track ahead that
/// is in the car's path now (its half width plus 0.3 m), or will be by the
/// time the car reaches it (its half width plus 0.6 m), from its walking
/// velocity. As a `Lead` for AEB: gap, its speed along x, stationary.
pub fn select_pedestrian(tracks: &[Track], ego_speed: f32, ego_front_x: f32, half_width: f32) -> Option<Lead> {
    tracks
        .iter()
        .filter(|t| t.confirmed && t.x() > ego_front_x - 0.5)
        .filter_map(|t| {
            let gap = (t.x() - ego_front_x).max(0.0);
            let closing = ego_speed - t.vx();
            let in_path = t.y().abs() < half_width + 0.3;
            let will_be = closing > 0.5 && {
                let reach = (gap / closing).min(4.0);
                (t.y() + t.vy() * reach).abs() < half_width + 0.6
            };
            (in_path || will_be).then_some(Lead { id: t.id, gap, v: t.vx(), a: 0.0, offset: t.y(), stationary: true })
        })
        .min_by(|a, b| a.gap.total_cmp(&b.gap))
}

#[cfg(test)]
mod tests {
    use super::*;
    use sensors::RadarObject;

    #[test]
    fn a_pedestrian_stepping_into_the_path() {
        let ped = |y: f32, vy: f32| {
            let mut t = Tracker::new(TrackerConfig { confirm_hits: 1, ..Default::default() });
            t.update(&RadarScan { scan: 0, objects: vec![RadarObject { id: 3, x: 25.0, y, vx: 0.0, vy, width: 0.5, half_length: 0.25, stationary: false, clutter: false }] }, 0.03);
            select_pedestrian(t.tracks(), 8.3, 2.3, 0.95)
        };
        // 4 m to the right, walking left at 1.5 m/s: in the path in 2.7 s, the car arrives in 2.8 s.
        assert!(ped(-4.0, 1.5).is_some());
        // Standing on the kerb: no.
        assert!(ped(-4.0, 0.0).is_none());
        // Walking away: no.
        assert!(ped(-4.0, -1.5).is_none());
        // In the path: yes.
        let l = ped(0.5, 0.0).unwrap();
        assert!((l.gap - 22.7).abs() < 0.01);
    }

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

    fn tracked(objects: &[RadarObject]) -> Tracker {
        let mut t = Tracker::new(TrackerConfig::default());
        for i in 0..4 {
            t.update(&RadarScan { scan: i, objects: objects.to_vec() }, 0.05);
        }
        t
    }

    /// Issue #20: a car passing a stationary return doesn't make it a car.
    #[test]
    fn camera_confirms_only_a_track_moving_as_the_vehicle_does() {
        // A stationary return at 30 m, 1 m left; a car passing it at 15 m/s.
        let mut t = tracked(&[obj(1, 30.0, 1.0, 0.0)]);
        for _ in 0..5 {
            t.confirm_vehicle(31.0, 0.0, 15.0, 0.0);
        }
        assert!(!t.tracks()[0].camera_vehicle(), "a moving camera car confirmed a stationary return");
        // A stopped car the camera sees as stopped: confirmed after two frames, not one.
        t.confirm_vehicle(30.5, 1.2, 0.2, 0.0);
        assert!(!t.tracks()[0].camera_vehicle(), "confirmed by a single frame");
        t.predict(1.0 / 30.0, 0.0, 0.0);
        t.confirm_vehicle(30.5, 1.2, -0.1, 0.1);
        assert!(t.tracks()[0].camera_vehicle());
        let f = select_lead(t.tracks(), 0.0, 0.0, 2.0, 1.8, LeadKind::Follow).unwrap();
        assert_eq!(f.id, 1);
    }

    /// Issue #20: the confirmation lapses once the camera stops seeing it.
    #[test]
    fn camera_confirmation_lapses() {
        let mut t = tracked(&[obj(1, 30.0, 0.0, 0.0)]);
        t.confirm_vehicle(30.0, 0.0, 0.0, 0.0);
        t.predict(0.03, 0.0, 0.0);
        t.confirm_vehicle(30.0, 0.0, 0.0, 0.0);
        assert!(t.tracks()[0].camera_vehicle());
        for _ in 0..40 {
            t.predict(0.03, 0.0, 0.0);
        }
        assert!(!t.tracks()[0].camera_vehicle(), "still a vehicle 1.2 s after the camera last saw it");
        assert!(select_lead(t.tracks(), 0.0, 0.0, 2.0, 1.8, LeadKind::Follow).is_none());
        // One frame after the lapse doesn't restore it on its own.
        t.confirm_vehicle(30.0, 0.0, 0.0, 0.0);
        assert!(!t.tracks()[0].camera_vehicle());
    }

    /// Issues #18 and #19: with the lane off to one side (the car half out of
    /// it), or a car coming the wrong way, AEB still sees what is straight
    /// ahead of the car's body.
    #[test]
    fn body_path_catches_what_the_lane_misses() {
        // Stopped object 1 m left of the car at 25 m; the lane's centre 2.9 m right.
        let t = tracked(&[obj(1, 25.0, 1.0, 0.0)]);
        let lane = [-2.9, 0.0, 0.0, 0.0];
        assert!(select_lead_in_lane(t.tracks(), 15.0, &lane, 2.0, 2.5, LeadKind::Collision).is_none());
        let b = select_in_body_path(t.tracks(), 15.0, 0.0, 2.0, 0.95).unwrap();
        assert_eq!(b.id, 1);
        assert_eq!(nearer(select_lead_in_lane(t.tracks(), 15.0, &lane, 2.0, 2.5, LeadKind::Collision), Some(b)).unwrap().id, 1);

        // A car reversing at the ego car, dead ahead, in its own lane: not
        // followed, but braked for.
        let t = tracked(&[obj(2, 30.0, 0.0, -4.0)]);
        let lane = [0.0; 4];
        assert!(select_lead_in_lane(t.tracks(), 10.0, &lane, 2.0, 1.8, LeadKind::Follow).is_none());
        assert_eq!(select_in_body_path(t.tracks(), 10.0, 0.0, 2.0, 0.95).unwrap().id, 2);

        // Oncoming traffic in the next lane (3.5 m left) is in neither.
        let t = tracked(&[obj(3, 30.0, 3.5, -15.0)]);
        assert!(select_in_body_path(t.tracks(), 15.0, 0.0, 2.0, 0.95).is_none());
        assert!(select_lead_in_lane(t.tracks(), 15.0, &lane, 2.0, 1.8, LeadKind::Collision).is_none());
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
