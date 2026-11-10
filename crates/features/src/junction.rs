//! Junction AEB: braking for traffic that crosses the car's path ahead (F3,
//! as Euro NCAP's junction and crossing tests ask of AEB).
//!
//! The forward AEB sees what is in the lane ahead; a car pulling out of a
//! side road comes from the side, where the front corner radars see it. For
//! each vehicle they track crossing the car's predicted path (the arc its
//! speed and yaw rate trace), this predicts, at constant velocities:
//! - when the vehicle is in the car's path: its box across the path's width
//!   (the car's, plus a margin), from `t_in` to `t_out`;
//! - when the car is in the vehicle's strip, the stretch of the path the
//!   vehicle sweeps across: from the front bumper reaching it to the rear
//!   bumper leaving it.
//!
//! If the two overlap (with a margin), the near edge of the strip is a
//! collision threat: it goes to AEB as a stationary obstacle, so AEB's
//! stages, hysteresis, hold and driver override apply unchanged. When the
//! vehicle will have cleared the path by the time the car gets there
//! (braking makes that so too), the threat goes away.
//!
//! Only crossing traffic counts: moving across the path at ≥ 2 m/s, at an
//! angle of ≥ 30° to it. Oncoming traffic in a bend and cars waiting at the
//! line don't, nor does a car braking to give way: one decelerating across
//! the path hard enough to stop before it (the track's acceleration
//! estimate). One slowing less reaches the path later, and is timed so.
//! Forward only, 7–70 km/h; vehicles reaching the path within 3 s.

use fusion::{Lead, SurroundTrack};

use crate::Ego;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JunctionConfig {
    pub min_speed: f32,
    pub max_speed: f32,
    /// Slowest crossing speed across the path (m/s), and the smallest angle
    /// to it (rad).
    pub min_cross_speed: f32,
    pub min_angle: f32,
    /// How far to either side of the path a vehicle is considered (m), and
    /// how soon it must reach the path (s).
    pub lateral_range: f32,
    pub horizon: f32,
    /// Added to the car's half width for its path (m).
    pub path_margin: f32,
    /// Added to both time windows (s).
    pub time_margin: f32,
    /// A crossing vehicle decelerating at least this much (m/s²) is taken
    /// to be braking.
    pub min_decel: f32,
}

impl Default for JunctionConfig {
    fn default() -> Self {
        Self {
            min_speed: 7.0 / 3.6,
            max_speed: 70.0 / 3.6,
            min_cross_speed: 2.0,
            min_angle: 30f32.to_radians(),
            lateral_range: 40.0,
            horizon: 3.0,
            path_margin: 0.5,
            time_margin: 0.3,
            min_decel: 1.0,
        }
    }
}

/// One crossing vehicle's conflict with the car, if any.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Conflict {
    pub id: u32,
    /// From the front bumper to the near edge of the vehicle's strip, along
    /// the path (m; 0 once the bumper is in it).
    pub gap: f32,
    /// When the vehicle is in the path, and the car in its strip (s).
    pub t_in: f32,
    pub t_out: f32,
    pub t_car_in: f32,
    pub t_car_out: f32,
    /// 0 from the left, 1 from the right.
    pub side: usize,
}

/// The conflict with one track. `k`: the path's curvature (1/m, left
/// positive), from the car's yaw rate over its speed.
pub fn conflict(c: &JunctionConfig, ego: &Ego, k: f32, t: &SurroundTrack) -> Option<Conflict> {
    let v = ego.speed;
    if !t.confirmed || ego.reverse || v < c.min_speed || v > c.max_speed {
        return None;
    }
    let (hx, hy) = t.extent();
    let (x, y) = (t.x(), t.y());
    if x + hx < ego.front {
        return None;
    }
    // Into the path's frame at the vehicle: along it (s ≈ x, since the path
    // bends little over the distances that matter), across it (d, left
    // positive), and its velocity in those directions.
    let d = y - 0.5 * k * x * x;
    let th = k * x;
    let (sn, cs) = th.sin_cos();
    let (u, w) = (t.vx() * cs + t.vy() * sn, -t.vx() * sn + t.vy() * cs);
    if d.abs() > c.lateral_range || w.abs() < c.min_cross_speed || w.abs() < u.abs() * c.min_angle.tan() {
        return None;
    }
    let hw = ego.half_width() + c.path_margin;
    // Moving toward the path, or in it.
    if d.abs() - hy > hw && d * w > 0.0 {
        return None;
    }
    // Braking across the path (to give way): stopping short is no conflict;
    // slowing, it gets there later.
    let a_w = -t.ax * sn + t.ay * cs;
    let edge = (d.abs() - hy - hw).max(0.0);
    let decel = if a_w * w < 0.0 { a_w.abs() } else { 0.0 };
    if decel >= c.min_decel && edge > 0.0 && w * w / (2.0 * decel) < edge {
        return None;
    }
    let t_in = if decel >= c.min_decel {
        let disc = (w * w - 2.0 * decel * edge).max(0.0);
        (w.abs() - disc.sqrt()) / decel
    } else {
        edge / w.abs()
    };
    let t_out = t_in + (2.0 * (hy + hw)) / w.abs().max(0.5);
    if t_in > c.horizon {
        return None;
    }
    // Its strip along the path when it gets there (it also moves along it).
    let near = x - hx + u * t_in;
    let far = x + hx + u * t_out;
    let t_car_in = ((near - ego.front) / v).max(0.0);
    let t_car_out = (far - ego.rear) / v;
    let m = c.time_margin;
    let overlap = t_car_in <= t_out + m && t_car_out >= t_in - m;
    overlap.then_some(Conflict { id: t.id, gap: (near - ego.front).max(0.0), t_in, t_out, t_car_in, t_car_out, side: if d > 0.0 { 0 } else { 1 } })
}

/// The nearest crossing conflict, as a stationary threat for AEB.
pub fn select_crossing(c: &JunctionConfig, ego: &Ego, k: f32, tracks: &[SurroundTrack]) -> Option<(Lead, Conflict)> {
    tracks
        .iter()
        .filter_map(|t| conflict(c, ego, k, t))
        .min_by(|a, b| a.gap.total_cmp(&b.gap))
        .map(|x| (Lead { id: x.id, gap: x.gap, v: 0.0, a: 0.0, offset: 0.0, stationary: true }, x))
}

#[cfg(test)]
mod tests {
    use super::*;
    use fusion::{SurroundConfig, SurroundTracker};
    use sensors::{RadarObject, RadarScan};

    fn track(x: f32, y: f32, vx: f32, vy: f32) -> SurroundTrack {
        let mut t = SurroundTracker::new(SurroundConfig::default());
        for i in 0..4 {
            let o = RadarObject { id: 1, x, y, vx, vy, width: 1.9, half_length: 2.25, stationary: false, clutter: false };
            t.update(1, &RadarScan { scan: i, objects: vec![o] });
        }
        t.tracks()[0].clone()
    }

    /// A car on a side road on the right, driving toward the path at `v0`
    /// and changing speed at `a` (m/s²), tracked for 1.5 s: where it is then.
    fn approaching(v0: f32, a: f32, x: f32, y0: f32) -> SurroundTrack {
        let mut t = SurroundTracker::new(SurroundConfig::default());
        let (mut y, mut v) = (y0, v0);
        for i in 0..30 {
            t.predict(0.05, 0.0, 0.0);
            y += v * 0.05;
            v = (v + a * 0.05).max(0.0);
            let o = RadarObject { id: 1, x, y, vx: 0.0, vy: v, width: 1.9, half_length: 2.25, stationary: false, clutter: false };
            t.update(2, &RadarScan { scan: i, objects: vec![o] });
        }
        t.tracks()[0].clone()
    }

    #[test]
    fn a_car_braking_to_give_way_is_not_a_threat() {
        let c = JunctionConfig::default();
        let ego = driving(4.0);
        // From 30 m out on the right at 11 m/s, braking at 3.5 m/s² for the
        // line: 1.5 s on, 18 m out at 6 m/s, it stops 5 m from the path.
        let braking = approaching(11.0, -3.5, 8.0, -30.0);
        assert!(braking.ay < -2.0, "acceleration estimate {}", braking.ay);
        assert!(conflict(&c, &ego, 0.0, &braking).is_none());
        // The same car not braking (at 6 m/s, 21 m out after 1.5 s): a threat.
        let steady = approaching(6.0, 0.0, 8.0, -30.0);
        assert!(steady.ay.abs() < 0.5, "acceleration estimate {}", steady.ay);
        assert!(conflict(&c, &ego, 0.0, &steady).is_some());
    }

    fn driving(speed: f32) -> Ego {
        Ego { speed, ..Default::default() }
    }

    #[test]
    fn a_car_pulling_out_ahead_is_a_threat() {
        let c = JunctionConfig::default();
        // At 33 km/h; a car 15 m ahead, 10 m to the left, crossing right at
        // 40 km/h (the GTA quad): it reaches the path in 0.6 s, the car its
        // strip in 1.2 s, while the car is still in the path.
        let x = conflict(&c, &driving(9.2), 0.0, &track(15.0, 10.0, 0.0, -11.0)).expect("conflict");
        assert_eq!(x.side, 0);
        assert!((x.gap - (15.0 - 0.95 - 2.3)).abs() < 0.1, "{x:?}");
        let (lead, _) = select_crossing(&c, &driving(9.2), 0.0, &[track(15.0, 10.0, 0.0, -11.0)]).unwrap();
        assert!(lead.stationary && lead.v == 0.0);
    }

    #[test]
    fn traffic_that_clears_or_waits_or_comes_towards_is_not() {
        let c = JunctionConfig::default();
        let ego = driving(14.0);
        // Crossing 60 m ahead at 50 km/h: through the path in 0.6 s, long
        // before the car gets there (4 s).
        assert!(conflict(&c, &ego, 0.0, &track(60.0, 4.0, 0.0, -14.0)).is_none());
        // Waiting at the line of a side road.
        assert!(conflict(&c, &ego, 0.0, &track(20.0, 8.0, 0.0, 0.0)).is_none());
        // Driving away from the path.
        assert!(conflict(&c, &ego, 0.0, &track(20.0, 8.0, 0.0, 6.0)).is_none());
        // Oncoming in the next lane on a straight road.
        assert!(conflict(&c, &ego, 0.0, &track(40.0, 3.5, -14.0, 0.0)).is_none());
        // Oncoming in a left bend (radius 150 m): in the ego frame it moves
        // sideways at 40 m, but along the bend's path it doesn't cross.
        let k = 1.0 / 150.0;
        let (x, th) = (40.0, k * 40.0);
        let y = 0.5 * k * x * x + 3.5;
        assert!(conflict(&c, &ego, k, &track(x, y, -14.0 * th.cos(), -14.0 * th.sin())).is_none());
        // Behind the bumper already.
        assert!(conflict(&c, &ego, 0.0, &track(-5.0, 6.0, 0.0, -8.0)).is_none());
        // Too fast for the feature, or reversing.
        assert!(conflict(&c, &driving(25.0), 0.0, &track(15.0, 10.0, 0.0, -11.0)).is_none());
        assert!(conflict(&c, &Ego { reverse: true, ..driving(3.0) }, 0.0, &track(15.0, 10.0, 0.0, -11.0)).is_none());
    }
}
