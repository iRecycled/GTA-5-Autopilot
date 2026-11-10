//! The sim's road: pieces of constant curvature, and its lane markings.
//!
//! Arc length `s` runs along the road's reference line and `d` is the offset
//! to the left of it. Lane 0 is centred on `d = 0`; lane +1 is to its left.
//! Before the first piece the road continues straight backwards; the last
//! piece continues forever.
//!
//! Markings are lines at an offset `d`, present from `from` to `to`, that may
//! move sideways over a stretch (`shift`): that is how a lane ends, its edge
//! line sweeping across it into the next lane's.

use geom::V2;

/// What a lane marking is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MarkingKind {
    Dashed,
    Solid,
    /// The edge of the carriageway.
    Edge,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Marking {
    pub d: f32,
    pub kind: MarkingKind,
    pub from: f32,
    pub to: f32,
    /// (start s, end s, final d): the line moves from `d` to the final d
    /// along this stretch, smoothly (a cosine profile).
    pub shift: Option<(f32, f32, f32)>,
}

impl Marking {
    pub fn line(d: f32, kind: MarkingKind) -> Self {
        Self { d, kind, from: f32::NEG_INFINITY, to: f32::INFINITY, shift: None }
    }

    /// Its offset at `s`, if it is there.
    pub fn d_at(&self, s: f32) -> Option<f32> {
        if s < self.from || s > self.to {
            return None;
        }
        Some(match self.shift {
            Some((a, b, d1)) if s > a => {
                let f = ((s - a) / (b - a).max(1e-3)).clamp(0.0, 1.0);
                self.d + (d1 - self.d) * (0.5 - 0.5 * (core::f32::consts::PI * f).cos())
            }
            _ => self.d,
        })
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Road {
    /// (length m, curvature 1/m, positive left), in order.
    segments: Vec<(f32, f32)>,
    /// Each piece's start: (s, position, heading).
    starts: Vec<(f32, V2, f32)>,
    pub lane_width: f32,
    pub markings: Vec<Marking>,
    /// The speed limit (m/s); infinite = none.
    pub speed_limit: f32,
}

impl Default for Road {
    fn default() -> Self {
        Self::new(vec![(f32::INFINITY, 0.0)], 3.6)
    }
}

impl Road {
    /// A road of these pieces with three lanes (−1, 0, +1): edges outside,
    /// dashed lines between.
    pub fn new(segments: Vec<(f32, f32)>, lane_width: f32) -> Self {
        let segments = if segments.is_empty() { vec![(f32::INFINITY, 0.0)] } else { segments };
        let mut starts = Vec::with_capacity(segments.len());
        let (mut s, mut p, mut h) = (0.0f32, V2::new(0.0, 0.0), 0.0f32);
        for &(len, k) in &segments {
            starts.push((s, p, h));
            if len.is_finite() {
                p = arc(p, h, k, len);
                h += k * len;
                s += len;
            }
        }
        let w = lane_width;
        let markings = vec![
            Marking::line(-1.5 * w, MarkingKind::Edge),
            Marking::line(-0.5 * w, MarkingKind::Dashed),
            Marking::line(0.5 * w, MarkingKind::Dashed),
            Marking::line(1.5 * w, MarkingKind::Edge),
        ];
        Self { segments, starts, lane_width, markings, speed_limit: f32::INFINITY }
    }

    /// Straight for `straight` m, then a left bend of `radius` forever
    /// (Phase 1's road).
    pub fn bend(straight: f32, radius: f32) -> Self {
        if !straight.is_finite() || !radius.is_finite() {
            return Self::default();
        }
        Self::new(vec![(straight, 0.0), (f32::INFINITY, 1.0 / radius)], 3.6)
    }

    /// The piece holding `s`: (its start s, position, heading, curvature).
    fn piece(&self, s: f32) -> (f32, V2, f32, f32) {
        let i = self.starts.iter().rposition(|st| st.0 <= s).unwrap_or(0);
        let (s0, p0, h0) = self.starts[i];
        // Before the start: straight backwards.
        let k = if s < 0.0 && i == 0 { 0.0 } else { self.segments[i].1 };
        (s0, p0, h0, k)
    }

    /// World position and heading at arc length `s`, offset `d` (left +).
    pub fn pose(&self, s: f32, d: f32) -> (V2, f32) {
        let (s0, p0, h0, k) = self.piece(s);
        let u = s - s0;
        let c = arc(p0, h0, k, u);
        let h = h0 + k * u;
        (V2::new(c.x - d * h.sin(), c.y + d * h.cos()), h)
    }

    /// Curvature at `s` (1/m, positive left).
    pub fn curvature(&self, s: f32) -> f32 {
        if s < 0.0 { 0.0 } else { self.piece(s).3 }
    }

    /// The total length of the pieces (infinite if the last goes on).
    pub fn length(&self) -> f32 {
        self.segments.iter().map(|s| s.0).sum()
    }

    /// The lane whose markings bracket offset `d` at `s`: (right line d,
    /// left line d).
    pub fn lane_at(&self, s: f32, d: f32) -> Option<(f32, f32)> {
        let right = self.markings.iter().filter_map(|m| m.d_at(s)).filter(|&x| x <= d).fold(f32::NEG_INFINITY, f32::max);
        let left = self.markings.iter().filter_map(|m| m.d_at(s)).filter(|&x| x > d).fold(f32::INFINITY, f32::min);
        (right.is_finite() && left.is_finite()).then_some((right, left))
    }
}

/// A point `u` along an arc of curvature `k` from `p`, heading `h`.
fn arc(p: V2, h: f32, k: f32, u: f32) -> V2 {
    if k.abs() < 1e-9 {
        V2::new(p.x + u * h.cos(), p.y + u * h.sin())
    } else {
        V2::new(p.x + ((h + k * u).sin() - h.sin()) / k, p.y + (h.cos() - (h + k * u).cos()) / k)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pieces_join_up() {
        let r = Road::bend(100.0, 200.0);
        let (p, h) = r.pose(100.0 + 200.0 * core::f32::consts::FRAC_PI_2, 0.0);
        assert!((p.x - 300.0).abs() < 1e-2 && (p.y - 200.0).abs() < 1e-2);
        assert!((h - core::f32::consts::FRAC_PI_2).abs() < 1e-5);
        // A left offset on the bend is nearer its centre.
        let (q, _) = r.pose(100.0 + 200.0 * core::f32::consts::FRAC_PI_2, 3.6);
        assert!((q.x - 296.4).abs() < 1e-2);
        // An S: left then right of equal length ends parallel to the start.
        let s = Road::new(vec![(50.0, 0.0), (100.0, 0.01), (100.0, -0.01), (f32::INFINITY, 0.0)], 3.6);
        let (_, h) = s.pose(400.0, 0.0);
        assert!(h.abs() < 1e-5 && s.curvature(120.0) == 0.01 && s.curvature(200.0) == -0.01 && s.curvature(-5.0) == 0.0);
        let (b, _) = s.pose(-10.0, 1.0);
        assert!((b.x + 10.0).abs() < 1e-5 && (b.y - 1.0).abs() < 1e-5, "straight backwards");
    }

    #[test]
    fn a_lane_that_ends() {
        let mut r = Road::default();
        // The right edge sweeps across lane −1 into lane 0's right line from
        // 200 to 300 m; that line (dashed) ends at 300 m.
        r.markings[0].shift = Some((200.0, 300.0, -1.8));
        r.markings[1].to = 300.0;
        let close = |a: Option<(f32, f32)>, b: (f32, f32)| a.is_some_and(|(x, y)| (x - b.0).abs() < 1e-4 && (y - b.1).abs() < 1e-4);
        assert!(close(r.lane_at(100.0, -3.6), (-5.4, -1.8)));
        let (right, left) = r.lane_at(250.0, -2.5).unwrap();
        assert!((right + 3.6).abs() < 1e-3 && (left + 1.8).abs() < 1e-4, "half way: ({right}, {left})");
        assert!(close(r.lane_at(350.0, 0.0), (-1.8, 1.8)));
        assert!(r.lane_at(350.0, -3.0).is_none(), "off the road");
    }
}
