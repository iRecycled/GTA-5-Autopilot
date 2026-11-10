//! In-game tests, one command for all of them:
//!
//! ```text
//! cargo xtask gta [TEST.toml ...] [--car sultan,baller] [--list]
//! ```
//!
//! Each test (`gta-tests/*.toml`) runs on each car:
//! 1. GTA is started if it isn't running (the Straight To Story Mode mod
//!    skips the menus), and the runner waits for the adapter;
//! 2. the car is spawned and teleported to the test spot, by the adapter's
//!    test commands (UDP 47310: no key presses, no window focus);
//! 3. a fresh `adas-core` starts, recording to `recordings/gta-tests/`;
//! 4. the steps run; waits end on what the core's telemetry says (speed,
//!    events), not on fixed sleeps;
//! 5. the run is scored from the telemetry (AEB activations, impacts, the
//!    ground-truth gap to the car ahead, events) against the test's `[pass]`.
//!
//! Steps, one per string:
//!
//! | Step | Does |
//! | --- | --- |
//! | `engage`, `cancel` | ACC engage request on / off |
//! | `set KPH` | press the speed buttons until the set speed is KPH |
//! | `gap SECONDS` | press the gap button until the time gap is SECONDS |
//! | `lead DIST KPH`, `stopped DIST`, `lead_brake`, `clear` | test cars |
//! | `drive THR BRK STEER SECS` | the driver's inputs (steer +left) |
//! | `fault adas\|chassis drop\|crc\|delay SECS` | a bus fault, as the panel injects it |
//! | `sleep SECS` | wait |
//! | `wait COND [TIMEOUT]` | until `speed>=KPH`, `speed<=KPH`, `stopped`, `engaged`, `off` or `event:TEXT` (default 30 s) |
//! | `check_gap SECS` | measure the time-gap error to the lead for SECS |

use std::net::UdpSocket;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use serde::Deserialize;
use telemetry::{Control, Packet};

const ADAPTER_PORT: u16 = 47310;
const RUNNER_PORT: u16 = 47303;
/// The ACC's standstill gap (m): the time gap is measured beyond it.
const STANDSTILL_GAP: f32 = 4.0;

fn root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..").canonicalize().expect("workspace root")
}

#[derive(Debug, Deserialize)]
struct Test {
    name: String,
    #[serde(default = "default_spot")]
    spot: String,
    steps: Vec<String>,
    /// The core's feature: "acc" (the default) or "autopilot".
    #[serde(default = "default_feature")]
    feature: String,
    #[serde(default)]
    pass: Pass,
}

fn default_feature() -> String {
    "acc".into()
}

fn default_spot() -> String {
    "airport taxiway".into()
}

fn yes() -> bool {
    true
}

#[derive(Debug, Deserialize)]
struct Pass {
    min_aeb: Option<u32>,
    max_aeb: Option<u32>,
    #[serde(default = "yes")]
    no_impact: bool,
    /// Ground-truth bumper gap to the car ahead at the end (m).
    stop_gap_min: Option<f32>,
    stop_gap_max: Option<f32>,
    /// The car must end below this speed (km/h).
    end_kph_below: Option<f32>,
    /// Events (substrings) that must / must not appear.
    #[serde(default)]
    events: Vec<String>,
    #[serde(default)]
    no_events: Vec<String>,
    /// Max time-gap error measured by `check_gap` (s).
    max_gap_error_s: Option<f32>,
    /// Rear cross-traffic and park assist brake activations.
    min_rcta: Option<u32>,
    max_rcta: Option<u32>,
    min_park: Option<u32>,
    max_park: Option<u32>,
    /// Ground truth at the end: room (m) from the bumper in the direction of
    /// travel to the nearest actor in the car's path.
    min_room: Option<f32>,
    /// Lane centering: RMS of the offset from the lane centre the camera
    /// sees while it steers (m), the share of the run it steered after
    /// engaging, and takeover requests.
    max_lane_rms: Option<f32>,
    min_centering: Option<f32>,
    max_takeovers: Option<u32>,
    /// The share of the run from the first engagement to the end that the
    /// system stayed engaged: a drop-out (the adapter releasing a stalled
    /// core, an unasked disengagement) fails the run.
    min_engaged: Option<f32>,
}

impl Default for Pass {
    fn default() -> Self {
        Self {
            min_aeb: None,
            max_aeb: None,
            no_impact: true,
            stop_gap_min: None,
            stop_gap_max: None,
            end_kph_below: None,
            events: vec![],
            no_events: vec![],
            max_gap_error_s: None,
            min_rcta: None,
            max_rcta: None,
            min_park: None,
            max_park: None,
            min_room: None,
            max_lane_rms: None,
            min_centering: None,
            max_takeovers: None,
            min_engaged: None,
        }
    }
}

/// What the runner keeps of each core step.
#[derive(Clone, Copy, Debug)]
struct Snap {
    speed: f32,
    state: u8,
    set_speed: f32,
    time_gap: f32,
    gt_gap: f32,
    aeb: u32,
    rcta: u32,
    park: u32,
    reverse: bool,
    /// Ground truth: room behind (reversing) or ahead to the nearest actor in the path.
    room: f32,
    hud_flags: u32,
    lane_offset: f32,
    distance: f32,
}

#[derive(Default)]
struct Live {
    snaps: Vec<Snap>,
    events: Vec<String>,
}

impl Live {
    fn last(&self) -> Option<Snap> {
        self.snaps.last().copied()
    }
}

/// Ground truth: bumper gap to the nearest actor ahead in the ego lane.
fn gt_gap(s: &telemetry::Step) -> f32 {
    s.actors
        .iter()
        .filter(|a| a.x > 0.0 && a.y.abs() < 1.5)
        .map(|a| a.x - 0.5 * a.length - s.ego_front)
        .fold(f32::NAN, |m, g| if m.is_nan() || g < m { g } else { m })
}

/// Ground truth: room from the bumper (rear when `behind`) to the nearest
/// actor's box within 0.3 m of the car's sides.
fn gt_room(s: &telemetry::Step, behind: bool) -> f32 {
    let half_w = 0.5 * (s.ego_left - s.ego_right);
    s.actors
        .iter()
        .filter_map(|a| {
            let (c, sn) = (a.yaw.cos().abs(), a.yaw.sin().abs());
            let (hx, hy) = (0.5 * (a.length * c + a.width * sn), 0.5 * (a.length * sn + a.width * c));
            if a.y.abs() - hy > half_w + 0.3 {
                return None;
            }
            let room = if behind { s.ego_rear - (a.x + hx) } else { a.x - hx - s.ego_front };
            (room >= -0.5).then_some(room.max(0.0))
        })
        .fold(f32::INFINITY, f32::min)
}

fn listen(live: Arc<Mutex<Live>>) {
    let mut rx = telemetry::Receiver::bind(RUNNER_PORT).unwrap_or_else(|e| panic!("cannot listen for telemetry on UDP {RUNNER_PORT}: {e}"));
    std::thread::spawn(move || {
        loop {
            match rx.try_recv() {
                Some(Packet::Step(s)) => {
                    let snap = Snap {
                        speed: s.speed,
                        state: s.state,
                        set_speed: s.set_speed,
                        time_gap: s.time_gap,
                        gt_gap: gt_gap(&s),
                        aeb: s.aeb_activations,
                        rcta: s.rcta_activations,
                        park: s.park_activations,
                        reverse: s.sample_flags & adapter_api::sample_flags::REVERSE != 0,
                        room: gt_room(&s, s.sample_flags & adapter_api::sample_flags::REVERSE != 0),
                        hud_flags: s.hud_flags,
                        lane_offset: s.lane_offset,
                        distance: s.speed / 72.0,
                    };
                    live.lock().expect("live").snaps.push(snap);
                }
                Some(Packet::Event { text, .. }) => live.lock().expect("live").events.push(text),
                Some(_) => {}
                None => std::thread::sleep(Duration::from_millis(2)),
            }
        }
    });
}

/// The adapter's test commands.
struct Game {
    sock: UdpSocket,
}

impl Game {
    fn new() -> Self {
        let sock = UdpSocket::bind("127.0.0.1:0").expect("bind");
        Self { sock }
    }

    fn cmd_timeout(&self, text: &str, timeout: Duration) -> Result<String, String> {
        self.sock.set_read_timeout(Some(timeout)).map_err(|e| e.to_string())?;
        self.sock.send_to(text.as_bytes(), ("127.0.0.1", ADAPTER_PORT)).map_err(|e| e.to_string())?;
        let mut buf = [0u8; 512];
        loop {
            match self.sock.recv_from(&mut buf) {
                Ok((n, _)) => {
                    let r = String::from_utf8_lossy(&buf[..n]).to_string();
                    return if r.starts_with("ok") { Ok(r) } else { Err(format!("{text}: {r}")) };
                }
                // Windows reports an earlier send's ICMP here: skip it.
                Err(e) if e.kind() == std::io::ErrorKind::ConnectionReset => continue,
                Err(_) => return Err(format!("{text}: no reply from the adapter")),
            }
        }
    }

    fn cmd(&self, text: &str) -> Result<String, String> {
        self.cmd_timeout(text, Duration::from_secs(20))
    }

    fn running() -> bool {
        Command::new("tasklist")
            .args(["/FI", "IMAGENAME eq GTA5_Enhanced.exe", "/NH"])
            .output()
            .map(|o| String::from_utf8_lossy(&o.stdout).contains("GTA5_Enhanced"))
            .unwrap_or(false)
    }

    /// Starts GTA if needed and waits until the adapter answers.
    fn ensure_ready(&self) -> Result<(), String> {
        if self.cmd_timeout("status", Duration::from_secs(1)).is_ok() {
            return Ok(());
        }
        if !Self::running() {
            println!("starting GTA V (Steam)...");
            Command::new("cmd").args(["/C", "start", "", "steam://rungameid/3240220"]).status().map_err(|e| e.to_string())?;
        } else {
            println!("waiting for the adapter (is story mode loading?)...");
        }
        let t0 = Instant::now();
        while t0.elapsed() < Duration::from_secs(300) {
            if let Ok(r) = self.cmd_timeout("status", Duration::from_secs(2)) {
                println!("adapter ready after {:.0} s: {r}", t0.elapsed().as_secs_f32());
                // Let the world settle after loading.
                std::thread::sleep(Duration::from_secs(5));
                return Ok(());
            }
        }
        Err("the adapter never answered (is the game in story mode?)".into())
    }
}

struct Run<'a> {
    game: &'a Game,
    live: Arc<Mutex<Live>>,
    gap_error: f32,
}

impl Run<'_> {
    fn last(&self) -> Option<Snap> {
        self.live.lock().expect("live").last()
    }

    fn wait_for(&self, what: &str, timeout: Duration, mut ok: impl FnMut(&Live) -> bool) -> Result<(), String> {
        let t0 = Instant::now();
        while t0.elapsed() < timeout {
            if ok(&self.live.lock().expect("live")) {
                return Ok(());
            }
            std::thread::sleep(Duration::from_millis(20));
        }
        Err(format!("timed out after {:.0} s waiting for {what}", timeout.as_secs_f32()))
    }

    fn step(&mut self, line: &str) -> Result<(), String> {
        let w: Vec<&str> = line.split_whitespace().collect();
        let num = |i: usize| -> Result<f32, String> { w.get(i).and_then(|v| v.parse().ok()).ok_or(format!("{line}: expected a number at {i}")) };
        match w.first().copied().unwrap_or("") {
            "engage" | "cancel" | "lead_brake" | "clear" | "stopped" | "drive" | "car" | "tp" | "signal" | "obstacle" | "creep" | "ai" | "walker" | "walk" => self.game.cmd(line).map(|_| ()),
            "lead" => self.game.cmd(&format!("lead {} {}", num(1)?, num(2)? / 3.6)).map(|_| ()),
            // `overtake|cross|pullout SIDE DIST KPH`: the adapter takes m/s.
            "overtake" | "cross" | "pullout" => {
                let side = w.get(1).copied().unwrap_or("");
                self.game.cmd(&format!("{} {side} {} {}", w[0], num(2)?, num(3)? / 3.6)).map(|_| ())
            }
            "set" => {
                let want = num(1)?;
                for _ in 0..60 {
                    let cur = self.last().map(|s| s.set_speed * 3.6).unwrap_or(f32::NAN);
                    if (cur - want).abs() < 1.0 {
                        return Ok(());
                    }
                    self.game.cmd(if cur < want { "button up" } else { "button down" })?;
                    std::thread::sleep(Duration::from_millis(120));
                }
                Err(format!("set speed never reached {want} km/h (is ACC engaged?)"))
            }
            "gap" => {
                let want = num(1)?;
                for _ in 0..8 {
                    if self.last().is_some_and(|s| (s.time_gap - want).abs() < 0.05) {
                        return Ok(());
                    }
                    self.game.cmd("button gap")?;
                    std::thread::sleep(Duration::from_millis(150));
                }
                Err(format!("time gap never reached {want} s"))
            }
            "sleep" => {
                std::thread::sleep(Duration::from_secs_f32(num(1)?));
                Ok(())
            }
            "fault" => {
                let bus = match w.get(1).copied() {
                    Some("chassis") => telemetry::CHASSIS,
                    Some("adas") => telemetry::ADAS,
                    _ => return Err(format!("{line}: fault chassis|adas ...")),
                };
                let kind = match w.get(2).copied() {
                    Some("drop") => 0,
                    Some("crc") => 1,
                    Some("delay") => 2,
                    _ => return Err(format!("{line}: drop|crc|delay")),
                };
                let c = Control::Inject { bus, id: None, kind, delay: 0.05, p: 1.0, duration: num(3)? as f64 };
                let s = UdpSocket::bind("127.0.0.1:0").map_err(|e| e.to_string())?;
                s.send_to(&Packet::Control(c).encode(), ("127.0.0.1", telemetry::CONTROL_PORT)).map_err(|e| e.to_string())?;
                Ok(())
            }
            "wait" => {
                // `wait COND [TIMEOUT]`; COND may contain spaces (event text).
                let (cond, timeout) = match w.last().and_then(|v| v.parse::<f32>().ok()) {
                    Some(t) if w.len() > 2 => (w[1..w.len() - 1].join(" "), t),
                    _ => (w[1..].join(" "), 30.0),
                };
                let timeout = Duration::from_secs_f32(timeout);
                if let Some(text) = cond.strip_prefix("event:") {
                    let t = text.to_ascii_lowercase();
                    return self.wait_for(&cond, timeout, |l| l.events.iter().any(|e| e.to_ascii_lowercase().contains(&t)));
                }
                if let Some(v) = cond.strip_prefix("speed>=") {
                    let v: f32 = v.parse().map_err(|_| format!("{line}: bad speed"))?;
                    return self.wait_for(&cond, timeout, |l| l.last().is_some_and(|s| s.speed * 3.6 >= v));
                }
                if let Some(v) = cond.strip_prefix("speed<=") {
                    let v: f32 = v.parse().map_err(|_| format!("{line}: bad speed"))?;
                    return self.wait_for(&cond, timeout, |l| l.last().is_some_and(|s| s.speed * 3.6 <= v));
                }
                match cond.as_str() {
                    "stopped" => self.wait_for(&cond, timeout, |l| l.last().is_some_and(|s| s.speed < 0.3)),
                    "engaged" => self.wait_for(&cond, timeout, |l| l.last().is_some_and(|s| s.state != 0)),
                    "off" => self.wait_for(&cond, timeout, |l| l.last().is_some_and(|s| s.state == 0)),
                    "reverse" => self.wait_for(&cond, timeout, |l| l.last().is_some_and(|s| s.reverse)),
                    _ => Err(format!("{line}: unknown condition {cond:?}")),
                }
            }
            "check_gap" => {
                let from = self.live.lock().expect("live").snaps.len();
                std::thread::sleep(Duration::from_secs_f32(num(1)?));
                let l = self.live.lock().expect("live");
                let mut worst = 0.0f32;
                let mut n = 0;
                for s in &l.snaps[from..] {
                    if s.speed > 1.0 && s.gt_gap.is_finite() {
                        worst = worst.max(((s.gt_gap - STANDSTILL_GAP) / s.speed - s.time_gap).abs());
                        n += 1;
                    }
                }
                if n == 0 {
                    return Err("check_gap: no lead to measure".into());
                }
                self.gap_error = self.gap_error.max(worst);
                Ok(())
            }
            _ => Err(format!("unknown step {line:?}")),
        }
    }
}

struct Row {
    test: String,
    car: String,
    failures: Vec<String>,
    summary: String,
}

fn start_core(stem: &str, feature: &str) -> Result<Child, String> {
    let root = root();
    let exe = std::env::current_exe().map_err(|e| e.to_string())?.with_file_name("adas-core.exe");
    let dir = root.join("recordings").join("gta-tests");
    std::fs::create_dir_all(&dir).map_err(|e| e.to_string())?;
    let log = std::fs::File::create(dir.join(format!("{stem}.log"))).map_err(|e| e.to_string())?;
    Command::new(&exe)
        .current_dir(&root)
        .args(["--feature", feature, "--no-profile-save", "--record", &dir.display().to_string(), "--telemetry", &format!("47300,47302,{RUNNER_PORT}")])
        .stdout(Stdio::from(log.try_clone().map_err(|e| e.to_string())?))
        .stderr(Stdio::from(log))
        .spawn()
        .map_err(|e| format!("{}: {e}", exe.display()))
}

fn run_test(game: &Game, live: &Arc<Mutex<Live>>, t: &Test, car: &str) -> Row {
    let mut row = Row { test: t.name.clone(), car: car.into(), failures: vec![], summary: String::new() };
    let setup = (|| -> Result<(), String> {
        game.cmd("cancel")?;
        game.cmd("clear")?;
        game.cmd(&format!("car {car}"))?;
        game.cmd(&format!("tp {}", t.spot))?;
        Ok(())
    })();
    if let Err(e) = setup {
        row.failures.push(format!("setup: {e}"));
        return row;
    }
    {
        let mut l = live.lock().expect("live");
        l.snaps.clear();
        l.events.clear();
    }
    let secs = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0);
    let slug: String = t.name.chars().map(|c| if c.is_ascii_alphanumeric() { c.to_ascii_lowercase() } else { '-' }).collect();
    let stem = format!("{secs}-{car}-{}", slug.trim_matches('-'));
    let mut core = match start_core(&stem, &t.feature) {
        Ok(c) => c,
        Err(e) => {
            row.failures.push(format!("core: {e}"));
            return row;
        }
    };
    let mut run = Run { game, live: live.clone(), gap_error: 0.0 };
    // The core is up once its telemetry arrives.
    let mut result = run.wait_for("the core's telemetry", Duration::from_secs(15), |l| l.snaps.len() > 72);
    if result.is_ok() {
        for s in &t.steps {
            if let Err(e) = run.step(s) {
                result = Err(format!("step `{s}`: {e}"));
                break;
            }
        }
    }
    std::thread::sleep(Duration::from_millis(300));
    let _ = core.kill();
    let _ = core.wait();
    let _ = game.cmd("cancel");
    if let Err(e) = result {
        row.failures.push(e);
    }

    // Score.
    let l = live.lock().expect("live");
    let last = l.last();
    let aeb = last.map(|s| s.aeb).unwrap_or(0);
    let impact = l.events.iter().any(|e| e.contains("IMPACT"));
    let end_gap = last.map(|s| s.gt_gap).unwrap_or(f32::NAN);
    let end_kph = last.map(|s| s.speed * 3.6).unwrap_or(f32::NAN);
    let p = &t.pass;
    let mut fail = |m: String| row.failures.push(m);
    if p.min_aeb.is_some_and(|m| aeb < m) {
        fail(format!("{aeb} AEB activations (want ≥ {})", p.min_aeb.unwrap_or(0)));
    }
    if p.max_aeb.is_some_and(|m| aeb > m) {
        fail(format!("{aeb} AEB activations (want ≤ {})", p.max_aeb.unwrap_or(0)));
    }
    if p.no_impact && impact {
        fail("impact".into());
    }
    if let Some(g) = p.stop_gap_min
        && !(end_gap >= g)
    {
        fail(format!("stopped {end_gap:.1} m short (want ≥ {g})"));
    }
    if let Some(g) = p.stop_gap_max
        && !(end_gap <= g)
    {
        fail(format!("stopped {end_gap:.1} m short (want ≤ {g})"));
    }
    if let Some(v) = p.end_kph_below
        && !(end_kph < v)
    {
        fail(format!("ended at {end_kph:.1} km/h (want < {v})"));
    }
    for e in &p.events {
        if !l.events.iter().any(|x| x.to_ascii_lowercase().contains(&e.to_ascii_lowercase())) {
            fail(format!("no event {e:?}"));
        }
    }
    for e in &p.no_events {
        if l.events.iter().any(|x| x.to_ascii_lowercase().contains(&e.to_ascii_lowercase())) {
            fail(format!("unexpected event {e:?}"));
        }
    }
    if let Some(g) = p.max_gap_error_s
        && run.gap_error > g
    {
        fail(format!("time-gap error {:.2} s (want ≤ {g})", run.gap_error));
    }
    let (rcta, park) = last.map(|s| (s.rcta, s.park)).unwrap_or((0, 0));
    let room = last.map(|s| s.room).unwrap_or(f32::NAN);
    if p.min_rcta.is_some_and(|m| rcta < m) || p.max_rcta.is_some_and(|m| rcta > m) {
        fail(format!("{rcta} RCTA brake activations (want {:?}..={:?})", p.min_rcta, p.max_rcta));
    }
    if p.min_park.is_some_and(|m| park < m) || p.max_park.is_some_and(|m| park > m) {
        fail(format!("{park} park assist brake activations (want {:?}..={:?})", p.min_park, p.max_park));
    }
    if let Some(r) = p.min_room
        && room.partial_cmp(&r) == Some(std::cmp::Ordering::Less)
    {
        fail(format!("ended {room:.2} m from the obstacle (want ≥ {r})"));
    }
    // Lane centering, from the steps it steered.
    use adapter_api::hud_flags as hf;
    let engaged: Vec<&Snap> = l.snaps.iter().filter(|s| s.state != 0).collect();
    let centering: Vec<&Snap> = engaged.iter().copied().filter(|s| s.hud_flags & hf::LANE_CENTERING != 0 && s.lane_offset.is_finite()).collect();
    let lane_rms = if centering.is_empty() { f32::NAN } else { (centering.iter().map(|s| s.lane_offset * s.lane_offset).sum::<f32>() / centering.len() as f32).sqrt() };
    let share = if engaged.is_empty() { 0.0 } else { centering.len() as f32 / engaged.len() as f32 };
    let takeovers = l.snaps.windows(2).filter(|w| w[1].hud_flags & hf::TAKEOVER != 0 && w[0].hud_flags & hf::TAKEOVER == 0).count() as u32;
    let km = l.snaps.iter().map(|s| s.distance).sum::<f32>() / 1000.0;
    let since_engaged = l.snaps.iter().position(|s| s.state != 0).map_or(&l.snaps[..0], |i| &l.snaps[i..]);
    let engaged_share = if since_engaged.is_empty() { 0.0 } else { since_engaged.iter().filter(|s| s.state != 0).count() as f32 / since_engaged.len() as f32 };
    let dropouts = l.snaps.windows(2).filter(|w| w[0].state != 0 && w[1].state == 0).count();
    if let Some(m) = p.min_engaged
        && engaged_share < m
    {
        fail(format!("engaged {:.0} % of the run after engaging, {dropouts} drop-outs (want ≥ {:.0} %)", 100.0 * engaged_share, 100.0 * m));
    }
    if let Some(m) = p.max_lane_rms
        && !(lane_rms <= m)
    {
        fail(format!("lane-centre offset {lane_rms:.2} m RMS (want ≤ {m})"));
    }
    if let Some(m) = p.min_centering
        && share < m
    {
        fail(format!("lane centering steered {:.0} % of the engaged time (want ≥ {:.0} %)", 100.0 * share, 100.0 * m));
    }
    if let Some(m) = p.max_takeovers
        && takeovers > m
    {
        fail(format!("{takeovers} takeover requests (want ≤ {m})"));
    }
    let mut summary = format!("AEB {aeb}");
    if p.max_lane_rms.is_some() || p.min_centering.is_some() {
        summary += &format!(", lane RMS {lane_rms:.2} m, centering {:.0} %, takeovers {takeovers}, engaged {:.0} %, {km:.2} km", 100.0 * share, 100.0 * engaged_share);
    }
    if rcta + park > 0 || p.min_room.is_some() {
        summary += &format!(", RCTA {rcta}, park {park}, room {room:.2} m");
    }
    if end_gap.is_finite() {
        summary += &format!(", gap at end {end_gap:.1} m");
    }
    if run.gap_error > 0.0 {
        summary += &format!(", time-gap error {:.2} s", run.gap_error);
    }
    summary += &format!(", end {end_kph:.0} km/h; recordings/gta-tests/{stem}.log");
    row.summary = summary;
    row
}

fn load(path: &Path) -> Result<Test, String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
    toml::from_str(&text).map_err(|e| format!("{}: {e}", path.display()))
}

fn main() {
    let mut files: Vec<PathBuf> = Vec::new();
    let mut cars = vec!["sultan".to_string()];
    let mut list = false;
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--car" | "--cars" => cars = args.next().expect("--car needs names").split(',').map(|s| s.trim().to_string()).collect(),
            "--list" => list = true,
            _ => files.push(PathBuf::from(a)),
        }
    }
    if files.is_empty() {
        let dir = root().join("gta-tests");
        files = std::fs::read_dir(&dir)
            .map(|rd| rd.filter_map(|e| e.ok().map(|e| e.path())).filter(|p| p.extension().is_some_and(|x| x == "toml")).collect())
            .unwrap_or_default();
        files.sort();
    }
    let tests: Vec<(PathBuf, Test)> = files
        .iter()
        .map(|f| match load(f) {
            Ok(t) => (f.clone(), t),
            Err(e) => {
                eprintln!("{e}");
                std::process::exit(2);
            }
        })
        .collect();
    if list {
        for (f, t) in &tests {
            println!("{}  {}", f.file_name().unwrap_or_default().to_string_lossy(), t.name);
        }
        return;
    }

    let live = Arc::new(Mutex::new(Live::default()));
    listen(live.clone());
    let game = Game::new();
    if let Err(e) = game.ensure_ready() {
        eprintln!("{e}");
        std::process::exit(1);
    }
    let mut rows = Vec::new();
    for car in &cars {
        for (_, t) in &tests {
            println!("--- {} on {car}", t.name);
            let row = run_test(&game, &live, t, car);
            println!("    {}: {}", if row.failures.is_empty() { "pass" } else { "FAIL" }, if row.failures.is_empty() { row.summary.clone() } else { format!("{}; {}", row.failures.join("; "), row.summary) });
            rows.push(row);
        }
    }
    let passed = rows.iter().filter(|r| r.failures.is_empty()).count();
    println!("\n{passed}/{} passed", rows.len());
    let w = rows.iter().map(|r| r.test.len()).max().unwrap_or(10);
    for r in &rows {
        println!("  {:<w$}  {:<8} {}", r.test, r.car, if r.failures.is_empty() { "pass".to_string() } else { format!("FAIL: {}", r.failures.join("; ")) });
    }
    if passed < rows.len() {
        std::process::exit(1);
    }
}
