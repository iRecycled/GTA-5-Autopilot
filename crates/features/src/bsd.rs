//! Blind spot detection (F4), with lane change assist's closing-vehicle
//! warning, after ISO 17387.
//!
//! Each side has a zone beside and behind the car: from 0.3 m to 3.8 m out
//! from the car's side, and from 3 m behind the rear bumper to about the
//! B-pillar (1.5 m behind the front). A vehicle is reported when any part of
//! its box is in the zone, or when it closes on the zone from behind fast
//! enough to reach it within 3 s (up to 40 m back). Not reported:
//! - stationary objects (parked cars, barriers): under 2 m/s over ground;
//! - vehicles the ego car passes clearly faster (by more than 10 km/h):
//!   they drop behind on their own, and real systems suppress them.
//!
//! The mirror icon lights while a side is occupied and stays lit 0.4 s after,
//! so it doesn't flicker. Signalling toward an occupied side is the warning:
//! flash and chime. Active above 15 km/h (off again below 10), not in reverse.

use fusion::SurroundTrack;

use crate::Ego;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BsdConfig {
    pub on_speed: f32,
    pub off_speed: f32,
    /// Zone, lateral: from the car's side (m).
    pub lat_near: f32,
    pub lat_far: f32,
    /// Zone, longitudinal: behind the rear bumper (m), and the front edge
    /// behind the front bumper (m).
    pub back: f32,
    pub front_inset: f32,
    /// Targets slower than this over ground are ignored (m/s).
    pub min_target_speed: f32,
    /// Targets the ego car passes faster than this are ignored (m/s).
    pub overtaken: f32,
    /// Closing vehicles: time to the zone (s) and how far back (m).
    pub closing_ttc: f32,
    pub closing_range: f32,
    /// The icon stays lit this long after the side clears (s).
    pub clear_time: f32,
}

impl Default for BsdConfig {
    fn default() -> Self {
        Self {
            on_speed: 15.0 / 3.6,
            off_speed: 10.0 / 3.6,
            lat_near: 0.3,
            lat_far: 3.8,
            back: 3.0,
            front_inset: 1.5,
            min_target_speed: 2.0,
            overtaken: 10.0 / 3.6,
            closing_ttc: 3.0,
            closing_range: 40.0,
            clear_time: 0.4,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BsdOut {
    pub active: bool,
    pub left: bool,
    pub right: bool,
    /// Signalling toward an occupied side.
    pub warn_left: bool,
    pub warn_right: bool,
}

#[derive(Clone, Copy, Debug, Default)]
struct Side {
    lit: bool,
    since_clear: f32,
}

#[derive(Clone, Debug, Default)]
pub struct Bsd {
    pub cfg: BsdConfig,
    active: bool,
    sides: [Side; 2],
    /// Episodes of a lit icon so far (left, right), for scoring.
    pub episodes: [u32; 2],
}

/// Which side a target is on, if it is reported: 0 left, 1 right.
pub fn bsd_side(c: &BsdConfig, ego: &Ego, t: &SurroundTrack) -> Option<usize> {
    if !t.confirmed || t.speed() < c.min_target_speed {
        return None;
    }
    // The object's box against the zone.
    let (hx, hy) = t.extent();
    let (side, near, far) = if t.y() > 0.0 { (0, t.y() - hy - ego.left, t.y() + hy - ego.left) } else { (1, ego.right - t.y() - hy, ego.right - t.y() + hy) };
    if far < c.lat_near || near > c.lat_far {
        return None;
    }
    let rel = t.vx() - ego.vx();
    let zone_back = ego.rear - c.back;
    let (back, front) = (t.x() - hx, t.x() + hx);
    let in_zone = front >= zone_back && back <= ego.front - c.front_inset && rel >= -c.overtaken;
    let closing = front < zone_back && front >= ego.rear - c.closing_range && rel > 0.5 && (zone_back - front) / rel <= c.closing_ttc;
    (in_zone || closing).then_some(side)
}

impl Bsd {
    pub fn new(cfg: BsdConfig) -> Self {
        Self { cfg, ..Default::default() }
    }

    pub fn step(&mut self, dt: f32, ego: &Ego, tracks: &[SurroundTrack]) -> BsdOut {
        let c = self.cfg;
        self.active = !ego.reverse && if self.active { ego.speed >= c.off_speed } else { ego.speed >= c.on_speed };
        let mut raw = [false; 2];
        if self.active {
            for t in tracks {
                if let Some(s) = bsd_side(&c, ego, t) {
                    raw[s] = true;
                }
            }
        }
        for (i, side) in self.sides.iter_mut().enumerate() {
            if raw[i] {
                if !side.lit {
                    self.episodes[i] += 1;
                }
                side.lit = true;
                side.since_clear = 0.0;
            } else if side.lit {
                side.since_clear += dt;
                if side.since_clear > c.clear_time || !self.active {
                    side.lit = false;
                }
            }
        }
        let (l, r) = (self.sides[0].lit, self.sides[1].lit);
        BsdOut { active: self.active, left: l, right: r, warn_left: l && ego.turn_left, warn_right: r && ego.turn_right }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fusion::{SurroundConfig, SurroundTracker};
    use sensors::{RadarObject, RadarScan};

    /// A tracker holding one confirmed track at (x, y) with over-ground vx.
    fn track(x: f32, y: f32, vx: f32) -> Vec<SurroundTrack> {
        let mut t = SurroundTracker::new(SurroundConfig::default());
        for i in 0..4 {
            let o = RadarObject { id: 1, x, y, vx, vy: 0.0, width: 1.9, half_length: 2.25, stationary: false, clutter: false };
            t.update(2, &RadarScan { scan: i, objects: vec![o] });
        }
        t.tracks().to_vec()
    }

    fn ego(speed: f32) -> Ego {
        Ego { speed, ..Default::default() }
    }

    #[test]
    fn zone_closing_and_suppression() {
        let c = BsdConfig::default();
        let e = ego(25.0);
        // Alongside on the left, same speed: in the zone.
        assert_eq!(bsd_side(&c, &e, &track(-3.0, 2.7, 25.0)[0]), Some(0));
        // On the right.
        assert_eq!(bsd_side(&c, &e, &track(-1.0, -2.7, 24.0)[0]), Some(1));
        // Two lanes over: not the blind spot.
        assert_eq!(bsd_side(&c, &e, &track(-3.0, 6.3, 25.0)[0]), None);
        // Its tail still beside the driver: reported. Wholly ahead of the
        // B-pillar: the driver sees it.
        assert_eq!(bsd_side(&c, &e, &track(2.5, 3.6, 25.0)[0]), Some(0));
        assert_eq!(bsd_side(&c, &e, &track(3.5, 3.6, 25.0)[0]), None);
        // Parked on the shoulder: stationary.
        assert_eq!(bsd_side(&c, &e, &track(-3.0, -2.7, 0.0)[0]), None);
        // Being passed at 30 km/h more: suppressed.
        assert_eq!(bsd_side(&c, &e, &track(-3.0, 2.7, 25.0 - 30.0 / 3.6)[0]), None);
        // Closing from 20 m behind at 10 m/s more: 1.6 s to the zone.
        assert_eq!(bsd_side(&c, &e, &track(-21.0, 2.7, 35.0)[0]), Some(0));
        // Far behind and barely closing: not yet.
        assert_eq!(bsd_side(&c, &e, &track(-30.0, 2.7, 26.0)[0]), None);
    }

    #[test]
    fn icons_warnings_and_activation() {
        let mut b = Bsd::new(BsdConfig::default());
        let car = track(-3.0, 2.7, 20.0);
        // Too slow: inactive.
        assert!(!b.step(0.05, &ego(3.0), &car).active);
        let mut e = ego(20.0);
        let o = b.step(0.05, &e, &car);
        assert!(o.active && o.left && !o.right && !o.warn_left);
        e.turn_left = true;
        assert!(b.step(0.05, &e, &car).warn_left);
        // Clears 0.4 s after the car is gone.
        let lit: Vec<bool> = (0..12).map(|_| b.step(0.05, &e, &[]).left).collect();
        assert!(lit[0] && lit[6] && !lit[11], "{lit:?}");
        assert_eq!(b.episodes, [1, 0]);
        // Never in reverse.
        e.reverse = true;
        assert!(!b.step(0.05, &e, &car).left);
    }
}
