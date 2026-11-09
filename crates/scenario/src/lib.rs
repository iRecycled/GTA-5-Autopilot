//! Scenarios (D27): `scenarios/*.toml` run through the real core against the
//! closed-loop simulator, scored on pass metrics: collisions, jerk, time-gap
//! error, AEB activations, stopping gap and time to collision.
//!
//! A scenario may repeat (`[repeat] count = 10`) with the first actor's start
//! position spread over `vary_s` metres, so a gate like "AEB stops in 10 of
//! 10 runs" is checked over different geometries and sensor noise.

use std::path::Path;

use adapter_api::{Command, buttons, command_flags, hud_flags, sample_flags};
use adas_core::{Core, Feature, Inputs};
use serde::Deserialize;
use sim::{ActorSpec, EgoParams, Event, Road, World};

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
    pub pass: Pass,
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
}

#[derive(Debug, Deserialize)]
pub struct ActorToml {
    /// Distance ahead of the ego car's centre at t = 0 (m).
    pub s: f32,
    /// Lane (0 = ego lane, 1 = left, −1 = right).
    #[serde(default)]
    pub lane: f32,
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
}

fn one() -> u32 {
    1
}

#[derive(Debug, Deserialize)]
pub struct Pass {
    #[serde(default = "yes")]
    pub no_collision: bool,
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
    let spread = if sc.repeat.count > 1 { sc.repeat.vary_s * run as f32 / (sc.repeat.count - 1) as f32 - 0.5 * sc.repeat.vary_s } else { 0.0 };
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
                d: a.lane * road.lane_width,
                v: kph(a.speed_kph),
                length: a.length,
                width: a.width,
                events,
            }
        })
        .collect();
    let mut world = World::new(road, EgoParams::default(), kph(sc.ego.speed_kph), actors, sc.fps);
    let feature = match sc.feature.as_str() {
        "acc" => Feature::Acc(Default::default()),
        _ => Feature::speed_profile(), // never engaged in "none" scenarios
    };
    let acc = sc.feature == "acc";
    let mut core = Core::new(feature, false);
    core.set_info(&world.info());
    let cruise = kph(sc.ego.speed_kph);

    let mut rep = Report { run, min_ttc: f32::INFINITY, ..Default::default() };
    let (mut a_hist, mut prev_a): (Vec<f32>, f32) = (Vec::new(), 0.0);
    let set_target = sc.ego.set_kph.map(kph);
    let mut gap_presses = 0usize;
    let mut aeb_seen = false;
    let want_gap = sc.ego.gap_index.unwrap_or(2);
    let frames = (sc.duration * sc.fps) as usize;
    let trace = std::env::var_os("SCENARIO_TRACE").is_some() && run == 0;
    for _ in 0..frames {
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
        let (cmd, _) = core.step(Inputs { sample: &s, actors: &a, stick: None });
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
    rep.end_speed = world.v;
    rep.end_gap = world.gap_ahead().map(|g| g.0).unwrap_or(f32::INFINITY);

    let p = &sc.pass;
    let mut fail = |m: String| rep.failures.push(m);
    if p.no_collision
        && let Some((t, v)) = rep.collision
    {
        fail(format!("collision at {t:.2} s, {:.1} km/h", v * 3.6));
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
    let _ = a_hist;
    rep
}

/// All repetitions of a scenario.
pub fn run_all(sc: &Scenario) -> Vec<Report> {
    (0..sc.repeat.count.max(1)).map(|i| run(sc, i)).collect()
}
