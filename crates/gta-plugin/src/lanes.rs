//! The map capability in GTA (`caps::MAP`): lane markings fitted from GTA's
//! road graph, which has no lanes, only path nodes with lane counts.
//!
//! The road's reference line is traced with `GET_CLOSEST_ROAD`: for a point,
//! the nearest road segment (its two nodes, lanes each way, the median's
//! width). The point projects onto that segment; the next is 10 m on along it
//! in the car's direction of travel. The trace is kept in world coordinates
//! and extended as the car moves (at most a couple of calls a frame, about
//! 85 µs each); it starts again if the car leaves it (a fork, a teleport).
//! Each frame it is reprojected into the ego frame.
//!
//! Lane lines are laid out from the lane counts and the lane width:
//! - a two-way road: the node line is the middle; the car's lanes are to its
//!   right (GTA drives on the right), beyond half the median;
//! - a one-way road (no lanes back): the node line is the carriageway's centre.
//!
//! GTA's lane width isn't published: `LANE_WIDTH` is measured from where AI
//! traffic drives (see `docs/status/phase3.md`); `%LOCALAPPDATA%\adas\lane_width.txt`
//! overrides it for experiments. With `survey`, the adapter logs AI traffic's
//! offsets from the node line, for that measurement.

use std::collections::VecDeque;
use std::io::Write;

use adapter_api::{ActorList, LaneMap, MAX_LANE_LINES, MAX_LINE_POINTS, line_kind};
use geom::{EgoPose, V2};
use shv_sys::{Vector3, invoke};

use crate::n;

/// GTA's lane width (m), measured in-game: AI traffic in the two lanes of
/// the Del Perro freeway's other carriageway passes about 5.0 m apart.
pub const LANE_WIDTH: f32 = 5.0;
const STEP_M: f32 = 10.0;
const BEHIND_M: f32 = 20.0;
const AHEAD_M: f32 = 150.0;
/// Calls to `GET_CLOSEST_ROAD` a frame, at most.
const CALLS_PER_FRAME: usize = 2;
/// The car this far off the trace's line: start again (m).
const LOST_M: f32 = 12.0;
/// Steps the road may be taken to go straight on past a junction.
const MAX_STRAIGHT: u32 = 3;

/// One traced point of the road's reference line (world frame).
#[derive(Clone, Copy, Debug)]
struct RoadPoint {
    p: V2,
    /// Unit direction of travel.
    dir: V2,
    lanes_ours: i32,
    lanes_back: i32,
    median: f32,
}

pub struct Lanes {
    points: VecDeque<RoadPoint>,
    /// Steps in a row taken as straight on (no matching road found).
    straight: u32,
    pub lane_width: f32,
    /// Log AI traffic's offsets from the node line (the lane-width survey).
    pub survey: bool,
    /// Diagnostics: calls this frame and their time (µs).
    pub calls: u32,
    pub call_us: f32,
    frame: u32,
}

/// The nearest road segment to `p`: (node a, node b, lanes a→b, lanes b→a,
/// median width). The native gives the lane counts the other way round
/// (b→a first): measured on a one-way freeway carriageway, where the car on
/// node a heading to b got "0, 2".
pub(crate) fn closest_road(p: V2, z: f32) -> Option<(V2, V2, i32, i32, f32)> {
    let (mut a, mut b) = (Vector3::default(), Vector3::default());
    let (mut lanes_ab, mut lanes_ba) = (0i32, 0i32);
    let mut median = 0.0f32;
    let ok: bool = unsafe {
        invoke!(
            n::GET_CLOSEST_ROAD,
            p.x,
            p.y,
            z,
            1.0f32,
            1i32,
            &mut a as *mut Vector3,
            &mut b as *mut Vector3,
            &mut lanes_ab as *mut i32,
            &mut lanes_ba as *mut i32,
            &mut median as *mut f32,
            false
        )
    };
    ok.then_some((V2::new(a.x, a.y), V2::new(b.x, b.y), lanes_ba, lanes_ab, median))
}

impl Lanes {
    pub fn new() -> Self {
        let dir = std::env::var_os("LOCALAPPDATA").map(|d| std::path::PathBuf::from(d).join("adas"));
        let lane_width = dir
            .as_ref()
            .and_then(|d| std::fs::read_to_string(d.join("lane_width.txt")).ok())
            .and_then(|s| s.trim().parse::<f32>().ok())
            .filter(|w| (2.5..8.0).contains(w))
            .unwrap_or(LANE_WIDTH);
        let survey = dir.is_some_and(|d| d.join("lane_survey").exists());
        Self { points: VecDeque::new(), straight: 0, lane_width, survey, calls: 0, call_us: 0.0, frame: 0 }
    }

    pub fn reset(&mut self) {
        self.points.clear();
    }

    /// One traced point near `guess`, heading roughly along `dir`.
    fn point(&mut self, guess: V2, z: f32, dir: V2) -> Option<RoadPoint> {
        let t0 = adapter_api::wall_time_ns();
        let r = closest_road(guess, z);
        self.calls += 1;
        self.call_us += (adapter_api::wall_time_ns() - t0) as f32 / 1e3;
        let (pa, pb, ab, ba, median) = r?;
        let seg = pb - pa;
        let len = seg.len();
        if len < 0.5 {
            return None;
        }
        let mut u = V2::new(seg.x / len, seg.y / len);
        let (lanes_ours, lanes_back) = if u.dot(dir) >= 0.0 { (ab, ba) } else { (ba, ab) };
        if u.dot(dir) < 0.0 {
            u = V2::new(-u.x, -u.y);
        }
        let t = (guess - pa).dot(u);
        Some(RoadPoint { p: V2::new(pa.x + u.x * t, pa.y + u.y * t), dir: u, lanes_ours: lanes_ours.max(1), lanes_back, median })
    }

    /// One frame: keep the trace up to date, then lay out the lines in the
    /// ego frame.
    pub fn step(&mut self, veh: i32, actors: &ActorList, log: &mut impl Write) -> LaneMap {
        self.frame = self.frame.wrapping_add(1);
        self.calls = 0;
        self.call_us = 0.0;
        let ep: Vector3 = unsafe { invoke!(n::GET_ENTITY_COORDS, veh, true) };
        let ef: Vector3 = unsafe { invoke!(n::GET_ENTITY_FORWARD_VECTOR, veh) };
        let ego = EgoPose::new(V2::new(ep.x, ep.y), V2::new(ef.x, ef.y));

        // Lost the line (a fork, a teleport): start again behind the car.
        // Only once the trace reaches past the car can it tell.
        let covers = self.points.back().is_some_and(|rp| ego.to_vehicle(rp.p).x > STEP_M);
        let near = self.points.iter().map(|rp| ego.to_vehicle(rp.p)).filter(|v| v.x.abs() <= STEP_M).map(|v| v.y.abs()).fold(f32::INFINITY, f32::min);
        if self.points.is_empty() || (covers && near > LOST_M) {
            self.points.clear();
            self.straight = 0;
            let start = V2::new(ep.x - ego.fwd.x * BEHIND_M, ep.y - ego.fwd.y * BEHIND_M);
            if let Some(p) = self.point(start, ep.z, ego.fwd) {
                self.points.push_back(p);
            }
        }
        // Drop what's behind; extend ahead within the call budget.
        while self.points.len() > 2 && ego.to_vehicle(self.points[1].p).x < -BEHIND_M {
            self.points.pop_front();
        }
        // Past a straight-on stretch: try the real road again now and then.
        if self.straight >= MAX_STRAIGHT && self.frame.is_multiple_of(36) {
            self.straight = 0;
        }
        // A new trace gets more calls, to reach past the car at once.
        let budget = if covers { CALLS_PER_FRAME } else { 6 };
        while (self.calls as usize) < budget && self.points.len() < MAX_LINE_POINTS && self.straight < MAX_STRAIGHT {
            let Some(&last) = self.points.back() else { break };
            if ego.to_vehicle(last.p).x > AHEAD_M {
                break;
            }
            let guess = V2::new(last.p.x + last.dir.x * STEP_M, last.p.y + last.dir.y * STEP_M);
            match self.point(guess, ep.z, last.dir) {
                // GTA's nearest road can be a crossing road at a junction (a
                // sharp turn) or a parallel carriageway (a sideways jump). Then
                // the road is taken to go straight on, for a few steps.
                Some(p) if p.dir.dot(last.dir) > 25f32.to_radians().cos() && (p.p - guess).len() < 3.5 => {
                    self.points.push_back(p);
                    self.straight = 0;
                }
                Some(_) => {
                    self.points.push_back(RoadPoint { p: guess, ..last });
                    self.straight += 1;
                }
                None => break,
            }
        }

        let mut m = LaneMap::default();
        if self.points.len() < 2 {
            return m;
        }
        let pts: Vec<(V2, RoadPoint)> = self.points.iter().map(|rp| (ego.to_vehicle(rp.p), *rp)).collect();
        // The lane counts at the car (the lowest nearby, so a slip road's
        // extra lane ahead doesn't move the lines here).
        let here = pts.iter().filter(|(v, _)| v.x > -BEHIND_M && v.x < 30.0).map(|(_, rp)| *rp).min_by_key(|rp| rp.lanes_ours).unwrap_or(pts[0].1);
        let w = self.lane_width;
        let n = here.lanes_ours.clamp(1, (MAX_LANE_LINES - 1) as i32);
        let (first, kind_first) = if here.lanes_back > 0 { (-0.5 * here.median.max(0.0), line_kind::SOLID) } else { (0.5 * n as f32 * w, line_kind::EDGE) };
        for i in 0..=n {
            let d = first - i as f32 * w;
            let kind = if i == 0 { kind_first } else if i == n { line_kind::EDGE } else { line_kind::DASHED };
            let line = &mut m.lines[m.count as usize];
            for (_, rp) in &pts {
                // Offset to the left of the direction of travel.
                let v = ego.to_vehicle(V2::new(rp.p.x - rp.dir.y * d, rp.p.y + rp.dir.x * d));
                line.points[line.count as usize] = [v.x, v.y];
                line.count += 1;
            }
            line.kind = kind;
            m.count += 1;
        }

        // The survey: where AI traffic drives relative to the node line.
        if self.survey && self.frame.is_multiple_of(36) {
            let node_y = |x: f32| -> Option<f32> {
                let i = pts.iter().position(|(v, _)| v.x >= x)?;
                if i == 0 {
                    return None;
                }
                let (a, b) = (pts[i - 1].0, pts[i].0);
                Some(a.y + (b.y - a.y) * (x - a.x) / (b.x - a.x).max(0.01))
            };
            let (mut same, mut other) = (Vec::new(), Vec::new());
            for a in actors.as_slice() {
                if a.kind != adapter_api::ActorKind::Vehicle || a.x.abs() > 60.0 || a.vx.hypot(a.vy) < 3.0 {
                    continue;
                }
                if let Some(y0) = node_y(a.x) {
                    let off = a.y - y0;
                    if a.vx > 0.0 { same.push(off) } else { other.push(off) }
                }
            }
            same.sort_by(f32::total_cmp);
            other.sort_by(f32::total_cmp);
            let f = |v: &[f32]| v.iter().map(|x| format!("{x:+.1}")).collect::<Vec<_>>().join(" ");
            let ego_off = node_y(0.0).map_or(f32::NAN, |y0| -y0);
            // Independent of the trace: parallel traffic passing alongside,
            // in the car's own frame; and the raw node natives at the car.
            let mut beside: Vec<String> = actors
                .as_slice()
                .iter()
                .filter(|a| a.kind == adapter_api::ActorKind::Vehicle && a.x.abs() < 15.0 && a.vx.hypot(a.vy) > 3.0 && (a.yaw.sin().abs() < 0.26))
                .map(|a| format!("{:+.1}{}", a.y, if a.vx > 0.0 { "f" } else { "b" }))
                .collect();
            beside.sort();
            let raw_road = closest_road(ego.pos, ep.z).map(|(a, b, ab, ba, med)| {
                let (va, vb) = (ego.to_vehicle(a), ego.to_vehicle(b));
                format!("road a ({:+.1},{:+.1}) b ({:+.1},{:+.1}) lanes {ab}/{ba} median {med:.1}", va.x, va.y, vb.x, vb.y)
            });
            let mut node = Vector3::default();
            let mut node_h = 0.0f32;
            let ok: bool = unsafe { invoke!(n::GET_CLOSEST_VEHICLE_NODE_WITH_HEADING, ep.x, ep.y, ep.z, &mut node as *mut Vector3, &mut node_h as *mut f32, 1i32, 3.0f32, 0i32) };
            let eh: f32 = unsafe { invoke!(n::GET_ENTITY_HEADING, veh) };
            let raw_node = ok.then(|| {
                let v = ego.to_vehicle(V2::new(node.x, node.y));
                format!("node ({:+.1},{:+.1}) heading {:+.0} vs car", v.x, v.y, geom::wrap_pi((node_h - eh).to_radians()).to_degrees())
            });
            let _ = writeln!(log, "lane survey raw: beside [{}] | {} | {}", beside.join(" "), raw_road.unwrap_or_default(), raw_node.unwrap_or_default());
            if self.frame.is_multiple_of(36 * 5) {
                let tr: Vec<String> = pts.iter().map(|(v, rp)| format!("({:.0},{:+.1} {}/{})", v.x, v.y, rp.lanes_ours, rp.lanes_back)).collect();
                let _ = writeln!(log, "lane survey trace: {}", tr.join(" "));
            }
            let _ = writeln!(
                log,
                "lane survey: lanes {} ours / {} back, median {:.1} m | ego {ego_off:+.2} | same way [{}] | other way [{}] | {} calls {:.0} us",
                here.lanes_ours,
                here.lanes_back,
                here.median,
                f(&same),
                f(&other),
                self.calls,
                self.call_us
            );
        }
        m
    }
}

/// Where a car on a road's node line should stand to be in a lane: the
/// offset to the left (m) of the first lane in its direction (`heading`,
/// GTA degrees). Two-way roads: the first lane right of the median; one-way
/// roads: the lane just right of the carriageway's centre.
pub(crate) fn lane_offset_at(p: V2, z: f32, heading: f32, lane_width: f32) -> f32 {
    let Some((a, b, ab, ba, median)) = closest_road(p, z) else { return 0.0 };
    let h = heading.to_radians();
    // GTA heading 0 is north (+y), counter-clockwise.
    let fwd = V2::new(-h.sin(), h.cos());
    let (ours, back) = if (b - a).dot(fwd) >= 0.0 { (ab, ba) } else { (ba, ab) };
    if back > 0 {
        -(0.5 * median.max(0.0) + 0.5 * lane_width)
    } else if ours.max(1) % 2 == 0 {
        -0.5 * lane_width
    } else {
        0.0
    }
}
