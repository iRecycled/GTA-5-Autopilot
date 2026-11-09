//! Surround tracking: one track list from the four corner radars.
//!
//! Each corner radar numbers its objects itself, so the same car has a
//! different id in each radar, and a car passing alongside is handed from one
//! radar to the next. The tracker associates each radar's (radar, id) pair
//! with a track: once known, by the pair; a new pair by gating against the
//! existing tracks (nearest within the gate, so a hand-over or an overlap of
//! two beams joins one track); otherwise it starts a new track. The corner
//! radars report object centres, so every radar puts one car at about the
//! same point; a track's box comes from its size and its direction of travel
//! ([`SurroundTrack::extent`]).
//!
//! The radars scan out of phase, so tracks are confirmed by hits and deleted
//! by time since their last update, not by missed scans of one radar.
//! Motion is the Phase 1 constant-velocity Kalman filter per axis, with
//! ego-motion compensation (signed ego speed: the car may be reversing).

use sensors::RadarScan;

use crate::Kf;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SurroundConfig {
    pub accel_noise: f32,
    pub pos_noise_m: f32,
    pub pos_noise_frac: f32,
    /// Cross-range noise from azimuth error (rad).
    pub azimuth_noise: f32,
    pub vel_noise: f32,
    pub confirm_hits: u32,
    /// A confirmed track not updated for this long is deleted (s).
    pub max_age: f32,
    /// A tentative track not updated for this long is deleted (s).
    pub tentative_age: f32,
    /// Gate for joining a new (radar, id) pair to a track: lateral (m) and
    /// longitudinal (m, plus the track's length: radars see different points
    /// of the same car), and speed (m/s).
    pub gate_y: f32,
    pub gate_x: f32,
    pub gate_v: f32,
}

impl Default for SurroundConfig {
    fn default() -> Self {
        Self {
            accel_noise: 3.0,
            pos_noise_m: 0.15,
            pos_noise_frac: 0.01,
            azimuth_noise: 1.0_f32.to_radians(),
            vel_noise: 0.15,
            confirm_hits: 3,
            max_age: 0.35,
            tentative_age: 0.15,
            gate_y: 1.5,
            gate_x: 1.5,
            gate_v: 3.0,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct SurroundTrack {
    pub id: u32,
    x: Kf,
    y: Kf,
    pub width: f32,
    pub half_length: f32,
    pub hits: u32,
    pub confirmed: bool,
    /// Seconds since the last update.
    pub age: f32,
    /// Seconds since the track started.
    pub life: f32,
    /// Radars that have reported it (bit per radar).
    pub sources: u8,
    /// Where it was first seen (ego frame at that time, m): BSD uses it to
    /// tell a car coming from behind from one the ego car is overtaking.
    pub first_x: f32,
}

impl SurroundTrack {
    pub fn x(&self) -> f32 {
        self.x.p
    }
    pub fn y(&self) -> f32 {
        self.y.p
    }
    /// Over-ground velocity in the ego frame (m/s).
    pub fn vx(&self) -> f32 {
        self.x.v
    }
    pub fn vy(&self) -> f32 {
        self.y.v
    }
    pub fn speed(&self) -> f32 {
        self.x.v.hypot(self.y.v)
    }

    /// Half the object's extent along the ego x and y axes (m). Radars give
    /// size, not heading: a moving object points where it goes; a still one
    /// is taken as parallel to the ego car.
    pub fn extent(&self) -> (f32, f32) {
        let (hl, hw) = (self.half_length.max(0.25), 0.5 * self.width.max(0.5));
        if self.speed() < 1.0 {
            return (hl, hw);
        }
        let (c, s) = ((self.x.v / self.speed()).abs(), (self.y.v / self.speed()).abs());
        (hl * c + hw * s, hl * s + hw * c)
    }
}

#[derive(Clone, Debug, Default)]
pub struct SurroundTracker {
    pub cfg: SurroundConfig,
    tracks: Vec<SurroundTrack>,
    /// (radar, radar's object id) → track id, and the time since it was seen.
    pairs: Vec<((u8, u32), u32, f32)>,
    next_id: u32,
}

impl SurroundTracker {
    pub fn new(cfg: SurroundConfig) -> Self {
        Self { cfg, tracks: Vec::new(), pairs: Vec::new(), next_id: 1 }
    }

    pub fn tracks(&self) -> &[SurroundTrack] {
        &self.tracks
    }

    pub fn clear(&mut self) {
        self.tracks.clear();
        self.pairs.clear();
    }

    /// Moves every track by `dt`, compensating the ego car's motion
    /// (`ego_vx` signed: negative when reversing). Deletes stale tracks.
    pub fn predict(&mut self, dt: f32, ego_vx: f32, yaw_rate: f32) {
        if dt <= 0.0 {
            return;
        }
        let q = self.cfg.accel_noise.powi(2);
        let (s, c) = (-yaw_rate * dt).sin_cos();
        for t in &mut self.tracks {
            t.x.predict(dt, q);
            t.y.predict(dt, q);
            t.x.p -= ego_vx * dt;
            let (px, py) = (t.x.p, t.y.p);
            t.x.p = c * px - s * py;
            t.y.p = s * px + c * py;
            let (vx, vy) = (t.x.v, t.y.v);
            t.x.v = c * vx - s * vy;
            t.y.v = s * vx + c * vy;
            t.age += dt;
            t.life += dt;
        }
        let cfg = self.cfg;
        self.tracks.retain(|t| t.age <= if t.confirmed { cfg.max_age } else { cfg.tentative_age });
        let alive: Vec<u32> = self.tracks.iter().map(|t| t.id).collect();
        for p in &mut self.pairs {
            p.2 += dt;
        }
        self.pairs.retain(|p| p.2 <= 0.5 && alive.contains(&p.1));
    }

    /// Corrects with one radar's scan (after `predict` for the same frame).
    pub fn update(&mut self, radar: u8, scan: &RadarScan) {
        let c = self.cfg;
        let mut used: Vec<u32> = Vec::new();
        for o in &scan.objects {
            let r = o.x.hypot(o.y);
            let rp = (c.pos_noise_m + c.pos_noise_frac * r).powi(2) + (r * c.azimuth_noise).powi(2) * 0.5;
            let rv = c.vel_noise.powi(2);
            let key = (radar, o.id);
            let known = self.pairs.iter().find(|p| p.0 == key).map(|p| p.1).filter(|id| !used.contains(id));
            let track = known.or_else(|| {
                // Gate: nearest track, not yet updated by this scan, that
                // could be the same object.
                self.tracks
                    .iter()
                    .filter(|t| !used.contains(&t.id))
                    .filter(|t| {
                        let (dx, dy) = ((o.x - t.x()).abs(), (o.y - t.y()).abs());
                        dy <= c.gate_y && dx <= c.gate_x + 2.0 * t.half_length.min(3.0) && (o.vx - t.vx()).hypot(o.vy - t.vy()) <= c.gate_v
                    })
                    .min_by(|a, b| {
                        let da = (o.x - a.x()).hypot(o.y - a.y());
                        let db = (o.x - b.x()).hypot(o.y - b.y());
                        da.total_cmp(&db)
                    })
                    .map(|t| t.id)
            });
            let id = match track.and_then(|id| self.tracks.iter_mut().find(|t| t.id == id)) {
                Some(t) => {
                    t.x.update(o.x, o.vx, rp, rv);
                    t.y.update(o.y, o.vy, rp, rv);
                    t.width = o.width;
                    t.half_length = o.half_length;
                    t.hits += 1;
                    t.age = 0.0;
                    t.sources |= 1 << radar;
                    if t.hits >= c.confirm_hits {
                        t.confirmed = true;
                    }
                    t.id
                }
                None => {
                    let id = self.next_id;
                    self.next_id = self.next_id.wrapping_add(1).max(1);
                    self.tracks.push(SurroundTrack {
                        id,
                        x: Kf::new(o.x, o.vx, rp, rv),
                        y: Kf::new(o.y, o.vy, rp, rv),
                        width: o.width,
                        half_length: o.half_length,
                        hits: 1,
                        confirmed: c.confirm_hits <= 1,
                        age: 0.0,
                        life: 0.0,
                        sources: 1 << radar,
                        first_x: o.x,
                    });
                    id
                }
            };
            used.push(id);
            match self.pairs.iter_mut().find(|p| p.0 == key) {
                Some(p) => (p.1, p.2) = (id, 0.0),
                None => self.pairs.push((key, id, 0.0)),
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sensors::RadarObject;

    fn obj(id: u32, x: f32, y: f32, vx: f32) -> RadarObject {
        RadarObject { id, x, y, vx, vy: 0.0, width: 1.9, half_length: 2.25, stationary: vx.abs() < 0.5, clutter: false }
    }

    #[test]
    fn a_car_passing_alongside_is_one_track_across_radars() {
        let mut t = SurroundTracker::new(SurroundConfig::default());
        // Ego at 20 m/s; a car in the left lane at 25 m/s overtakes from 20 m
        // behind to 20 m ahead. The rear-left radar (2) sees it, then the
        // front-left (0), with their own ids and their own errors.
        let mut x = -20.0f32;
        let mut ids = std::collections::BTreeSet::new();
        for i in 0..160u32 {
            t.predict(0.05, 20.0, 0.0);
            x += 5.0 * 0.05;
            if x < 2.0 {
                t.update(2, &RadarScan { scan: i, objects: vec![obj(900, x + 0.3, 3.5, 25.0)] });
            }
            if x > -3.0 {
                t.update(0, &RadarScan { scan: i, objects: vec![obj(31, x - 0.3, 3.7, 25.0)] });
            }
            for tr in t.tracks() {
                ids.insert(tr.id);
            }
        }
        assert_eq!(ids.len(), 1, "one track for one car: {ids:?}");
        let tr = &t.tracks()[0];
        assert!(tr.confirmed && tr.sources == 0b101 && (tr.vx() - 25.0).abs() < 0.3, "{tr:?}");
        assert!(tr.first_x < -15.0, "first seen behind");
        let (hx, hy) = tr.extent();
        assert!((hx - 2.25).abs() < 0.05 && (hy - 0.95).abs() < 0.05, "along the road: ({hx}, {hy})");
    }

    #[test]
    fn two_cars_side_by_side_stay_two_tracks_and_stale_ones_go() {
        let mut t = SurroundTracker::new(SurroundConfig::default());
        for i in 0..10 {
            t.predict(0.05, 0.0, 0.0);
            t.update(2, &RadarScan { scan: i, objects: vec![obj(1, -8.0, 3.0, 0.0), obj(2, -8.0, -3.0, 0.0)] });
        }
        assert_eq!(t.tracks().iter().filter(|x| x.confirmed).count(), 2);
        // Reversing at 2 m/s: the stationary objects come closer.
        for _ in 0..5 {
            t.predict(0.05, -2.0, 0.0);
        }
        assert!((t.tracks()[0].x() + 7.5).abs() < 0.05, "x {}", t.tracks()[0].x());
        for _ in 0..3 {
            t.predict(0.05, 0.0, 0.0);
        }
        assert!(t.tracks().is_empty(), "not updated for 0.4 s: deleted");
    }
}
