//! Scenarios (D27): `scenarios/*.toml` run through the real core against the
//! closed-loop simulator, scored on pass metrics: collisions, jerk, time-gap
//! error, AEB activations, stopping gap and time to collision.
//!
//! A scenario may repeat (`[repeat] count = 10`) with the first actor's start
//! position spread over `vary_s` metres, so a gate like "AEB stops in 10 of
//! 10 runs" is checked over different geometries and sensor noise.
//!
//! `[[fault]]` tables inject bus faults (drop, corrupt or delay frames) on
//! the virtual vehicle's buses, by message, with a probability and a window.
//!
//! Phase 2 adds the surround: the ego car may reverse (`gear = "reverse"`)
//! and signal (`[[ego.signal]]`); actors may cross the road (`heading`);
//! `[[obstacle]]` places walls; `[traffic]` generates cars in chosen lanes
//! for long drives. BSD is scored against ground truth (detections and false
//! alerts), RCTA and park assist by their activations and the clearance left.

use std::path::Path;

use adapter_api::{ActorList, Command, buttons, command_flags, hud_flags, sample_flags, surround_flags};
use adas_core::{Core, Event as CoreEvent, Feature, Inputs};
use serde::Deserialize;
use sim::{ActorSpec, EgoParams, Event, Obstacle, Road, Traffic, TrafficLane, World};

#[derive(Debug, Deserialize)]
pub struct Scenario {
    pub name: String,
    /// "acc", or "none" (no feature engaged: a driver cruises; AEB is armed).
    pub feature: String,
    pub duration: f32,
    #[serde(default = "default_fps")]
    pub fps: f32,
    #[serde(default)]
    pub road: RoadSpec,
    pub ego: EgoSpec,
    #[serde(default)]
    pub actor: Vec<ActorToml>,
    #[serde(default)]
    pub repeat: Repeat,
    #[serde(default)]
    pub fault: Vec<FaultToml>,
    #[serde(default)]
    pub obstacle: Vec<ObstacleToml>,
    pub traffic: Option<TrafficToml>,
    /// Surround features switched off ("bsd", "rcta", "park"): control runs
    /// that show a scenario needs the feature.
    #[serde(default)]
    pub disabled: Vec<String>,
    pub pass: Pass,
}

/// A wall or other static obstacle, aligned with the road.
#[derive(Debug, Deserialize)]
pub struct ObstacleToml {
    /// Centre, along the road from the ego car's centre (m; negative = behind).
    pub s: f32,
    #[serde(default)]
    pub d: f32,
    pub length: f32,
    pub width: f32,
}

#[derive(Debug, Deserialize)]
pub struct TrafficToml {
    #[serde(default = "default_traffic_range")]
    pub range: f32,
    #[serde(default)]
    pub seed: u64,
    pub lane: Vec<TrafficLaneToml>,
}

fn default_traffic_range() -> f32 {
    150.0
}

#[derive(Debug, Deserialize)]
pub struct TrafficLaneToml {
    /// Lane (1 = left of the ego lane, −1 = right).
    pub lane: f32,
    pub speed_kph: f32,
    #[serde(default)]
    pub spread_kph: f32,
    /// Mean gap between cars (m).
    pub gap: f32,
}

#[derive(Debug, Deserialize)]
pub struct SignalToml {
    pub t: f32,
    /// "left", "right" or "off".
    pub side: String,
}

/// A bus fault for the whole run or a window of it.
#[derive(Debug, Deserialize)]
pub struct FaultToml {
    /// "chassis" or "adas".
    pub bus: String,
    /// A message name from the bus's DBC; omit for every frame on the bus.
    pub message: Option<String>,
    /// "drop", "corrupt" or "delay".
    pub kind: String,
    /// Delay for "delay" (s).
    pub delay: Option<f64>,
    /// Probability per frame (default 1).
    pub p: Option<f64>,
    /// Window (s); default the whole run.
    pub from: Option<f64>,
    pub until: Option<f64>,
}

fn default_fps() -> f32 {
    72.0
}

#[derive(Debug, Default, Deserialize)]
pub struct RoadSpec {
    /// Straight length before a left bend (m); omit for a straight road.
    pub straight: Option<f32>,
    pub radius: Option<f32>,
}

#[derive(Debug, Deserialize)]
pub struct EgoSpec {
    pub speed_kph: f32,
    /// When the driver engages ACC (s).
    #[serde(default)]
    pub engage_at: f32,
    /// Set speed after engaging (km/h); the runner presses the buttons.
    pub set_kph: Option<f32>,
    /// Time gap index (0 = 1.0 s … 3 = 2.5 s); default 2 (2.0 s).
    pub gap_index: Option<usize>,
    /// "reverse": the driver reverses at `speed_kph`.
    pub gear: Option<String>,
    /// Turn signal changes.
    #[serde(default)]
    pub signal: Vec<SignalToml>,
}

#[derive(Debug, Deserialize)]
pub struct ActorToml {
    /// Distance ahead of the ego car's centre at t = 0 (m).
    pub s: f32,
    /// Lane (0 = ego lane, 1 = left, −1 = right).
    #[serde(default)]
    pub lane: f32,
    /// Lateral offset (m, left +), instead of `lane`.
    pub d: Option<f32>,
    /// Drive in a fixed direction relative to the road (deg; 90 = across it,
    /// toward the left) instead of along it.
    pub heading: Option<f32>,
    pub speed_kph: f32,
    #[serde(default = "default_len")]
    pub length: f32,
    #[serde(default = "default_width")]
    pub width: f32,
    #[serde(default)]
    pub event: Vec<EventToml>,
}

fn default_len() -> f32 {
    4.5
}
fn default_width() -> f32 {
    1.9
}

#[derive(Debug, Deserialize)]
pub struct EventToml {
    pub t: f32,
    pub speed_kph: Option<f32>,
    /// Acceleration or deceleration magnitude for the speed change (m/s²).
    pub accel: Option<f32>,
    /// Move to this lane over `duration` seconds.
    pub lane: Option<f32>,
    pub duration: Option<f32>,
}

#[derive(Debug, Default, Deserialize)]
pub struct Repeat {
    #[serde(default = "one")]
    pub count: u32,
    /// Spread of the first actor's start position over the runs (m).
    #[serde(default)]
    pub vary_s: f32,
    /// Spread of the first actor's lateral start position (m).
    #[serde(default)]
    pub vary_d: f32,
    /// Spread of the first obstacle's position (m).
    #[serde(default)]
    pub vary_obstacle: f32,
}

fn one() -> u32 {
    1
}

#[derive(Debug, Deserialize)]
pub struct Pass {
    #[serde(default = "yes")]
    pub no_collision: bool,
    /// The run must end in a collision (a control run with a feature off).
    #[serde(default)]
    pub collision: bool,
    /// Max |jerk| while ACC is in control and AEB is not braking (m/s³).
    pub max_jerk: Option<f32>,
    /// Max |time gap − set time gap| inside `steady` (s).
    pub gap_error_s: Option<f32>,
    pub steady: Option<[f32; 2]>,
    pub max_aeb: Option<u32>,
    pub min_aeb: Option<u32>,
    /// Bumper gap once stopped, at the end (m).
    pub min_stop_gap: Option<f32>,
    pub max_stop_gap: Option<f32>,
    /// The ego car must end below this speed (m/s).
    pub end_speed_below: Option<f32>,
    /// The ego car must end above this speed (m/s): it resumed.
    pub end_speed_above: Option<f32>,
    /// A feature must have been reported unavailable for this reason (the
    /// HUD text, e.g. "radar blocked").
    pub unavailable: Option<String>,
    /// No feature may be reported unavailable.
    #[serde(default)]
    pub stays_available: bool,
    /// BSD must report every vehicle that is truly in this side's zone for a
    /// second or more ("left" / "right"): lit within 0.5 s, and lit for at
    /// least 80 % of the rest of the time it is there.
    pub bsd_detect: Option<String>,
    /// BSD must warn (signalling into an occupied lane) on this side.
    pub bsd_warn: Option<String>,
    /// At most this many BSD icon episodes with nothing truly there.
    pub max_bsd_false: Option<u32>,
    /// Rear cross-traffic braking activations.
    pub min_rcta: Option<u32>,
    pub max_rcta: Option<u32>,
    /// RCTA must warn before it brakes.
    #[serde(default)]
    pub rcta_warns_first: bool,
    /// Park assist braking activations.
    pub min_park: Option<u32>,
    pub max_park: Option<u32>,
    /// At the end, at least this much room (m) to the nearest obstacle in
    /// the direction of travel.
    pub min_clearance: Option<f32>,
}

fn yes() -> bool {
    true
}

/// Metrics of one run.
#[derive(Debug, Clone, Default)]
pub struct Report {
    pub run: u32,
    pub collision: Option<(f32, f32)>,
    pub max_jerk: f32,
    pub gap_error: f32,
    pub aeb: u32,
    pub min_ttc: f32,
    pub end_speed: f32,
    pub end_gap: f32,
    /// Unavailability reasons reported during the run (HUD text).
    pub unavailable: Vec<String>,
    /// Bus frames lost to checksum failures.
    pub bus_errors: u64,
    /// BSD: vehicles truly in a zone (≥ 1 s) and those reported, per side;
    /// the worst time to light up (s); icon episodes with nothing there.
    pub bsd_truth: [u32; 2],
    pub bsd_detected: [u32; 2],
    pub bsd_latency: f32,
    pub bsd_false: u32,
    pub bsd_warned: [bool; 2],
    pub rcta: u32,
    pub rcta_warned_first: bool,
    pub park: u32,
    /// Room left to the nearest obstacle in the direction of travel (m).
    pub clearance: f32,
    pub failures: Vec<String>,
}

impl Report {
    pub fn passed(&self) -> bool {
        self.failures.is_empty()
    }
}

pub fn load(path: &Path) -> Result<Scenario, String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
    toml::from_str(&text).map_err(|e| format!("{}: {e}", path.display()))
}

fn kph(v: f32) -> f32 {
    v / 3.6
}

/// Runs one repetition.
pub fn run(sc: &Scenario, run: u32) -> Report {
    let road = Road {
        straight: sc.road.straight.unwrap_or(f32::INFINITY),
        radius: sc.road.radius.unwrap_or(f32::INFINITY),
        lane_width: 3.6,
    };
    let frac = if sc.repeat.count > 1 { run as f32 / (sc.repeat.count - 1) as f32 - 0.5 } else { 0.0 };
    let spread = sc.repeat.vary_s * frac;
    let spread_d = sc.repeat.vary_d * frac;
    let actors: Vec<ActorSpec> = sc
        .actor
        .iter()
        .enumerate()
        .map(|(i, a)| {
            let mut events = Vec::new();
            for e in &a.event {
                if let Some(v) = e.speed_kph {
                    events.push((e.t, Event::Speed { v: kph(v), accel: e.accel.unwrap_or(0.0) }));
                }
                if let Some(l) = e.lane {
                    events.push((e.t, Event::Lane { d: l * road.lane_width, duration: e.duration.unwrap_or(3.0) }));
                }
            }
            events.sort_by(|x, y| x.0.total_cmp(&y.0));
            ActorSpec {
                s: a.s + if i == 0 { spread } else { 0.0 },
                d: a.d.unwrap_or(a.lane * road.lane_width) + if i == 0 { spread_d } else { 0.0 },
                v: kph(a.speed_kph),
                length: a.length,
                width: a.width,
                events,
                heading: a.heading.map(f32::to_radians),
            }
        })
        .collect();
    let mut world = World::new(road, EgoParams::default(), kph(sc.ego.speed_kph), actors, sc.fps);
    let reverse = sc.ego.gear.as_deref() == Some("reverse");
    world.reverse = reverse;
    for (i, o) in sc.obstacle.iter().enumerate() {
        let shift = if i == 0 { sc.repeat.vary_obstacle * frac } else { 0.0 };
        world.obstacles.push(Obstacle { s: o.s + shift, d: o.d, length: o.length, width: o.width });
    }
    if let Some(t) = &sc.traffic {
        let lanes = t
            .lane
            .iter()
            .map(|l| TrafficLane { d: l.lane * road.lane_width, speed: kph(l.speed_kph), spread: kph(l.spread_kph), gap: l.gap })
            .collect();
        world.set_traffic(Traffic { lanes, range: t.range, seed: t.seed ^ ((run as u64 + 1) * 0x9E37) });
    }
    let mut signals: Vec<(f32, i8)> = sc
        .ego
        .signal
        .iter()
        .map(|g| {
            let side = match g.side.as_str() {
                "left" => 1,
                "right" => -1,
                "off" => 0,
                other => panic!("{}: signal side {other:?} (left, right, off)", sc.name),
            };
            (g.t, side)
        })
        .collect();
    signals.sort_by(|a, b| a.0.total_cmp(&b.0));
    let feature = match sc.feature.as_str() {
        "acc" => Feature::Acc(Default::default()),
        _ => Feature::speed_profile(), // never engaged in "none" scenarios
    };
    let acc = sc.feature == "acc";
    let mut core = Core::new(feature, false);
    core.set_info(&world.info());
    let mut toggles = adas_core::Toggles::default();
    for d in &sc.disabled {
        match d.as_str() {
            "bsd" => toggles.bsd = false,
            "rcta" => toggles.rcta = false,
            "park" => toggles.park = false,
            other => panic!("{}: unknown feature {other:?} in `disabled` (bsd, rcta, park)", sc.name),
        }
    }
    core.set_toggles(toggles);
    for f in &sc.fault {
        let kind = match (f.kind.as_str(), f.delay) {
            ("drop", _) => can::FaultKind::Drop,
            ("corrupt", _) => can::FaultKind::Corrupt,
            ("delay", Some(d)) => can::FaultKind::Delay(d),
            (k, _) => panic!("{}: fault kind {k:?} (drop, corrupt, or delay with `delay`)", sc.name),
        };
        core.vehicle_mut()
            .inject(&f.bus, f.message.as_deref(), kind, f.p.unwrap_or(1.0), f.from.unwrap_or(f64::NEG_INFINITY), f.until.unwrap_or(f64::INFINITY))
            .unwrap_or_else(|e| panic!("{}: {e}", sc.name));
    }
    let cruise = kph(sc.ego.speed_kph);

    let mut rep = Report { run, min_ttc: f32::INFINITY, ..Default::default() };
    let (mut a_hist, mut prev_a): (Vec<f32>, f32) = (Vec::new(), 0.0);
    let set_target = sc.ego.set_kph.map(kph);
    let mut gap_presses = 0usize;
    let mut aeb_seen = false;
    let want_gap = sc.ego.gap_index.unwrap_or(2);
    let frames = (sc.duration * sc.fps) as usize;
    let trace = std::env::var_os("SCENARIO_TRACE").is_some() && run == 0;
    let mut bsd = BsdScore::default();
    let (mut rcta_warned, mut rcta_braked) = (false, false);
    let info = world.info();
    for _ in 0..frames {
        world.turn = signals.iter().rev().find(|(t, _)| *t <= world.t).map_or(0, |s| s.1);
        let engage = acc && world.t >= sc.ego.engage_at;
        let mut btn = 0;
        if engage && core.hud().flags & hud_flags::ACC != 0 {
            if let Some(t) = set_target {
                let s = core.hud().set_speed;
                if s < t - 0.1 {
                    btn |= buttons::SPEED_UP;
                } else if s > t + 0.1 {
                    btn |= buttons::SPEED_DOWN;
                }
            }
            if core.hud().gap_index as usize != want_gap && gap_presses < 8 {
                btn |= buttons::GAP;
                gap_presses += 1;
            }
        }
        let flags = if engage { sample_flags::ENGAGE_REQUEST } else { 0 };
        let mut s = world.sample(flags, btn);
        // The simulated driver holds the cruise speed while nothing is engaged
        // and doesn't react to the hazard; as in Euro NCAP's AEB tests, it
        // releases the accelerator once AEB intervenes.
        let driver_throttle = if acc || aeb_seen { 0.0 } else { (0.3 * (cruise - world.v) + 0.05).clamp(0.0, 0.6) };
        s.driver_throttle = driver_throttle;
        let a = world.actors(core.layout());
        let fans = *core.fans();
        let ranges = (fans.flags & adapter_api::fan_flags::ACTIVE != 0).then(|| world.ranges(&fans));
        let (cmd, ev) = core.step(Inputs { sample: &s, actors: &a, ranges: ranges.as_ref(), stick: None });
        let sf = core.hud().surround;
        bsd.frame(&sc.pass, &a, world.v, &info, sf, 1.0 / sc.fps);
        if std::env::var_os("BSD_TRACE").is_some() && world.frame.is_multiple_of(9) {
            let truth = bsd_truth(&a, world.v, &info, false);
            let lit = [sf & surround_flags::BSD_LEFT != 0, sf & surround_flags::BSD_RIGHT != 0];
            for i in 0..2 {
                if truth[i] && !lit[i] {
                    let near: Vec<String> = a
                        .as_slice()
                        .iter()
                        .filter(|x| (x.y > 0.0) == (i == 0) && x.y.abs() < 6.0 && x.x > -25.0 && x.x < 5.0)
                        .map(|x| format!("id {} ({:.1},{:.1}) vx {:.1} los {:#b}", x.id, x.x, x.y, x.vx - world.v, x.los_mask))
                        .collect();
                    let tr: Vec<String> = core
                        .surround_tracks()
                        .iter()
                        .filter(|t| (t.y() > 0.0) == (i == 0) && t.x() > -25.0 && t.x() < 5.0)
                        .map(|t| format!("T{} ({:.1},{:.1}) rel {:.1} c{} age {:.2}", t.id, t.x(), t.y(), t.vx() - world.v, t.confirmed as u8, t.age))
                        .collect();
                    println!("MISS t {:.2} side {i}: truth {near:?} | tracks {tr:?}", world.t);
                }
            }
        }
        if sf & (surround_flags::RCTA_LEFT | surround_flags::RCTA_RIGHT) != 0 && sf & surround_flags::RCTA_BRAKE == 0 {
            rcta_warned = true;
        }
        if sf & surround_flags::RCTA_BRAKE != 0 && !rcta_braked {
            rcta_braked = true;
            rep.rcta_warned_first = rcta_warned;
        }
        if let CoreEvent::Unavailable(r) = ev
            && !rep.unavailable.iter().any(|u| u == r.text())
        {
            rep.unavailable.push(r.text().to_string());
        }
        let aeb_on = cmd.flags & command_flags::AEB != 0;
        aeb_seen |= aeb_on;
        let applied = if cmd.flags & (command_flags::ENGAGED | command_flags::AEB) != 0 {
            cmd
        } else {
            Command { flags: command_flags::ENGAGED, throttle: driver_throttle, ..Default::default() }
        };
        world.step(&applied);

        if trace && (world.frame.is_multiple_of(7) || aeb_on) && world.frame.is_multiple_of(2) {
            let (g, lv) = world.gap_ahead().unwrap_or((f32::NAN, f32::NAN));
            println!(
                "t {:6.2} v {:5.2} a {:+5.2} | gap {:6.2} lead_v {:5.2} | hud gap {:6.2} ttc {:5.2} flags {:#04x} | cmd flags {:#04x} thr {:.2} brk {:.2} | tracks {}",
                world.t, world.v, world.a, g, lv, core.hud().lead_gap, core.hud().ttc, core.hud().flags, cmd.flags, cmd.throttle, cmd.brake,
                core.tracker().tracks().len()
            );
        }
        // Metrics.
        let ttc = core.hud().ttc;
        if ttc.is_finite() {
            rep.min_ttc = rep.min_ttc.min(ttc);
        }
        a_hist.push(world.a);
        let jerk = (world.a - prev_a) * sc.fps;
        prev_a = world.a;
        let acc_in_control = cmd.flags & command_flags::ENGAGED != 0 && !aeb_on;
        if trace && jerk.abs() > 2.0 && acc_in_control {
            println!("JERK {jerk:+.2} at t {:.2}: v {:.2} a {:+.3} | thr {:.3} brk {:.3} hold {} | ref {:.2}", world.t, world.v, world.a, cmd.throttle, cmd.brake, cmd.flags & command_flags::HOLD != 0, cmd.diag[1]);
        }
        if acc_in_control && world.t > sc.ego.engage_at + 0.5 {
            rep.max_jerk = rep.max_jerk.max(jerk.abs());
        }
        if let (Some([t0, t1]), Some((gap, _))) = (sc.pass.steady, world.gap_ahead())
            && world.t >= t0
            && world.t <= t1
            && world.v > 1.0
        {
            let tg = core.hud().time_gap;
            // The ACC keeps standstill gap + time gap × speed; compare like for like.
            let err = ((gap - 4.0) / world.v - tg).abs();
            rep.gap_error = rep.gap_error.max(err);
        }
        if let Some(c) = world.collision {
            rep.collision = Some((c.t, c.speed));
            break;
        }
    }
    rep.aeb = core.aeb_activations();
    rep.bus_errors = core.vehicle().stats.errors;
    rep.end_speed = world.v;
    rep.end_gap = world.gap_ahead().map(|g| g.0).unwrap_or(f32::INFINITY);
    bsd.finish();
    (rep.bsd_truth, rep.bsd_detected, rep.bsd_latency, rep.bsd_false, rep.bsd_warned) = (bsd.truth, bsd.detected, bsd.latency, bsd.false_alerts, bsd.warned);
    (rep.rcta, rep.park) = core.surround_activations();
    rep.clearance = world.clearance(reverse);

    let p = &sc.pass;
    let mut fail = |m: String| rep.failures.push(m);
    if p.no_collision
        && let Some((t, v)) = rep.collision
    {
        fail(format!("collision at {t:.2} s, {:.1} km/h", v * 3.6));
    }
    if p.collision && rep.collision.is_none() {
        fail("expected a collision (control run), got none".into());
    }
    if let Some(j) = p.max_jerk
        && rep.max_jerk > j
    {
        fail(format!("jerk {:.2} > {j} m/s³", rep.max_jerk));
    }
    if let Some(g) = p.gap_error_s
        && rep.gap_error > g
    {
        fail(format!("time-gap error {:.2} > {g} s", rep.gap_error));
    }
    if let Some(m) = p.max_aeb
        && rep.aeb > m
    {
        fail(format!("{} AEB activations > {m}", rep.aeb));
    }
    if let Some(m) = p.min_aeb
        && rep.aeb < m
    {
        fail(format!("{} AEB activations < {m}", rep.aeb));
    }
    if let Some(g) = p.min_stop_gap
        && rep.end_gap < g
    {
        fail(format!("stopped {:.2} m from the car ahead (< {g})", rep.end_gap));
    }
    if let Some(g) = p.max_stop_gap
        && rep.end_gap > g
    {
        fail(format!("stopped {:.2} m from the car ahead (> {g})", rep.end_gap));
    }
    if let Some(v) = p.end_speed_below
        && rep.end_speed >= v
    {
        fail(format!("end speed {:.2} ≥ {v} m/s", rep.end_speed));
    }
    if let Some(v) = p.end_speed_above
        && rep.end_speed <= v
    {
        fail(format!("end speed {:.2} ≤ {v} m/s", rep.end_speed));
    }
    if let Some(u) = &p.unavailable
        && !rep.unavailable.iter().any(|x| x == u)
    {
        fail(format!("never reported unavailable for {u:?} (got {:?})", rep.unavailable));
    }
    if p.stays_available && !rep.unavailable.is_empty() {
        fail(format!("reported unavailable: {:?}", rep.unavailable));
    }
    let side = |s: &str| match s {
        "left" => 0,
        "right" => 1,
        other => panic!("{}: side {other:?} (left, right)", sc.name),
    };
    if let Some(s) = &p.bsd_detect {
        let i = side(s);
        if rep.bsd_truth[i] == 0 {
            fail(format!("no vehicle was ever in the {s} blind spot"));
        } else if rep.bsd_detected[i] < rep.bsd_truth[i] {
            fail(format!("BSD {s}: reported {} of {} vehicles in the zone (latency {:.2} s)", rep.bsd_detected[i], rep.bsd_truth[i], rep.bsd_latency));
        }
    }
    if let Some(s) = &p.bsd_warn
        && !rep.bsd_warned[side(s)]
    {
        fail(format!("BSD never warned on the {s}"));
    }
    if let Some(m) = p.max_bsd_false
        && rep.bsd_false > m
    {
        fail(format!("{} false BSD alerts > {m}", rep.bsd_false));
    }
    if let Some(m) = p.min_rcta
        && rep.rcta < m
    {
        fail(format!("{} RCTA brake activations < {m}", rep.rcta));
    }
    if let Some(m) = p.max_rcta
        && rep.rcta > m
    {
        fail(format!("{} RCTA brake activations > {m}", rep.rcta));
    }
    if p.rcta_warns_first && rep.rcta > 0 && !rep.rcta_warned_first {
        fail("RCTA braked without warning first".into());
    }
    if let Some(m) = p.min_park
        && rep.park < m
    {
        fail(format!("{} park assist brake activations < {m}", rep.park));
    }
    if let Some(m) = p.max_park
        && rep.park > m
    {
        fail(format!("{} park assist brake activations > {m}", rep.park));
    }
    if let Some(c) = p.min_clearance
        && rep.clearance < c
    {
        fail(format!("ended {:.2} m from the obstacle (< {c})", rep.clearance));
    }
    let _ = a_hist;
    rep
}

/// BSD scored against ground truth. A side is truly occupied when a moving
/// vehicle's box overlaps its zone (the feature's own geometry and
/// thresholds), or closes on it within the closing time. Each spell of truth
/// of 1 s or more must be reported (lit within 0.5 s, then for 80 % of the
/// rest). An icon episode is a false alert if nothing was there by a looser
/// truth (the zone 1 m bigger all round, slower targets, 4 s closing) at any
/// time during it.
#[derive(Default)]
struct BsdScore {
    /// Per side: the current spell of strict truth.
    spell: [Option<Spell>; 2],
    /// Per side: the current icon episode, and whether loose truth was seen.
    episode: [Option<bool>; 2],
    t: f32,
    truth: [u32; 2],
    detected: [u32; 2],
    latency: f32,
    false_alerts: u32,
    warned: [bool; 2],
}

/// A spell of truth: start time, frames, lit frames after the first 0.5 s,
/// first lit time.
type Spell = (f32, u32, u32, Option<f32>);

/// Is a vehicle in (or closing on) each side's zone? `loose` widens it.
fn bsd_truth(actors: &ActorList, ego_v: f32, info: &adapter_api::AdapterInfo, loose: bool) -> [bool; 2] {
    let c = features::BsdConfig::default();
    let (grow, min_v, closing_ttc, rel_min) = if loose { (1.0, 1.0, 4.0, -c.overtaken - 1.5) } else { (0.0, c.min_target_speed, c.closing_ttc, -c.overtaken) };
    let (front, rear, left, right) = (info.box_max[0], info.box_min[0], info.box_max[1], info.box_min[1]);
    let mut out = [false; 2];
    for a in actors.as_slice() {
        if a.vx.hypot(a.vy) < min_v {
            continue;
        }
        let (hx, hy) = (0.5 * (a.length * a.yaw.cos().abs() + a.width * a.yaw.sin().abs()), 0.5 * (a.length * a.yaw.sin().abs() + a.width * a.yaw.cos().abs()));
        let (side, near, far) = if a.y > 0.0 { (0, a.y - hy - left, a.y + hy - left) } else { (1, right - (a.y + hy), right - (a.y - hy)) };
        if far < c.lat_near - grow || near > c.lat_far + grow {
            continue;
        }
        let rel = a.vx - ego_v;
        let zone_back = rear - c.back - grow;
        let in_zone = a.x + hx >= zone_back && a.x - hx <= front - c.front_inset + grow && rel >= rel_min;
        let closing = a.x + hx < zone_back && a.x + hx >= rear - c.closing_range - grow && rel > 0.3 && (zone_back - (a.x + hx)) / rel <= closing_ttc;
        out[side] |= in_zone || closing;
    }
    out
}

impl BsdScore {
    fn frame(&mut self, _p: &Pass, actors: &ActorList, ego_v: f32, info: &adapter_api::AdapterInfo, sf: u32, dt: f32) {
        self.t += dt;
        let active = sf & surround_flags::BSD_ACTIVE != 0;
        let strict = bsd_truth(actors, ego_v, info, false);
        let loose = bsd_truth(actors, ego_v, info, true);
        let lit = [sf & surround_flags::BSD_LEFT != 0, sf & surround_flags::BSD_RIGHT != 0];
        self.warned[0] |= sf & surround_flags::BSD_WARN_LEFT != 0;
        self.warned[1] |= sf & surround_flags::BSD_WARN_RIGHT != 0;
        for i in 0..2 {
            // Truth spells count only while BSD is active (above its speed).
            match (&mut self.spell[i], strict[i] && active) {
                (Some((t0, n, on, first)), true) => {
                    *n += 1;
                    if lit[i] && first.is_none() {
                        *first = Some(self.t);
                    }
                    if self.t - *t0 > 0.5 && lit[i] {
                        *on += 1;
                    }
                }
                (None, true) => self.spell[i] = Some((self.t, 1, 0, lit[i].then_some(self.t))),
                (Some(_), false) => self.end_spell(i),
                (None, false) => {}
            }
            match (&mut self.episode[i], lit[i]) {
                (Some(seen), true) => *seen |= loose[i],
                (None, true) => self.episode[i] = Some(loose[i]),
                (Some(seen), false) => {
                    if !*seen {
                        self.false_alerts += 1;
                    }
                    self.episode[i] = None;
                }
                (None, false) => {}
            }
        }
    }

    fn end_spell(&mut self, i: usize) {
        let Some((t0, n, on, first)) = self.spell[i].take() else { return };
        let dur = self.t - t0;
        if dur < 1.0 {
            return;
        }
        self.truth[i] += 1;
        let after = (n as f32 - 0.5 / (dur / n as f32)).max(1.0);
        let lat = first.map_or(f32::INFINITY, |f| f - t0);
        self.latency = self.latency.max(lat);
        if lat <= 0.5 && on as f32 >= 0.8 * after {
            self.detected[i] += 1;
        }
    }

    fn finish(&mut self) {
        for i in 0..2 {
            self.end_spell(i);
            if self.episode[i].take() == Some(false) {
                self.false_alerts += 1;
            }
        }
    }
}

/// All repetitions of a scenario.
pub fn run_all(sc: &Scenario) -> Vec<Report> {
    (0..sc.repeat.count.max(1)).map(|i| run(sc, i)).collect()
}
