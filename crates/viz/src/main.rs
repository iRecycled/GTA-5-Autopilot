//! The Rerun views (D8): what the engineer sees of a drive.
//!
//! - **Bird's-eye view:** ego; tracks (what the ADAS ECU believes), with the
//!   lead and the collision threat marked and velocity arrows; host actors
//!   (ground truth) as outlines.
//! - **Plots:** speed (host, chassis CAN, set, lead), acceleration, pedals
//!   (command and driver), lead gap and TTC, bus load per network, latency.
//! - **CAN trace** per bus, decoded with the generated message types, and the
//!   core's events.
//!
//! Usage:
//! - `adas-viz` — live: listen for the core's telemetry on UDP 47302;
//! - `adas-viz --replay X.mcap` — run the core over a recorded drive here;
//! - `adas-viz --file X.telem` — a telemetry file (`replay --telemetry`);
//! - `--save X.rrd` writes a file instead of opening the viewer; `--no-can`
//!   leaves the CAN trace out.
//!
//! The viewer (`rerun`, 0.42.0) is found on PATH, in `RERUN_VIEWER`, or where
//! `pip install --user rerun-sdk==0.42.0` puts it.

use std::path::{Path, PathBuf};

use adas_core::msgs::{adas, chassis};
use adas_core::{Core, Feature, Inputs, Stick, telem};
use rerun::blueprint::{Blueprint, Horizontal, Spatial2DView, Tabs, TextLogView, TimePanel, TimeSeriesView, Vertical};
use rerun::{Arrows2D, Boxes2D, Color, LineStrips2D, RecordingStream, Scalars, SeriesLines, TextLog};
use telemetry::{BUS_NAMES, CanFrame, Packet, Step, track_flags};

const BUS_COLORS: [[u8; 3]; 2] = [[90, 160, 255], [255, 150, 60]];
const LOAD_WINDOW_S: f64 = 0.25;
const HUD_BITS: [&str; 9] = ["ACC", "FOLLOW", "LEAD", "FCW", "AEB", "HOLD", "OVERRIDE", "UNAVAIL", "AEB-OFF"];

fn rgb(c: [u8; 3]) -> Color {
    Color::from_rgb(c[0], c[1], c[2])
}

/// ISO 8855 (x forward, y left) to the view: forward is up.
fn bev(x: f32, y: f32) -> [f32; 2] {
    [-y, -x]
}

fn outline(x: f32, y: f32, yaw: f32, length: f32, width: f32) -> Vec<[f32; 2]> {
    let (s, c) = yaw.sin_cos();
    [(0.5, 0.5), (0.5, -0.5), (-0.5, -0.5), (-0.5, 0.5), (0.5, 0.5)]
        .iter()
        .map(|(dx, dy)| {
            let (px, py) = (dx * length, dy * width);
            bev(x + c * px - s * py, y + s * px + c * py)
        })
        .collect()
}

fn decode(bus: u8, id: u32, data: &[u8]) -> (Option<&'static str>, Vec<(&'static str, f64)>) {
    match bus {
        telemetry::CHASSIS => (chassis::name(id), chassis::decode_signals(id, data).unwrap_or_default()),
        _ => (adas::name(id), adas::decode_signals(id, data).unwrap_or_default()),
    }
}

fn blueprint() -> Blueprint {
    let plots = Vertical::new([
        TimeSeriesView::new("Speed (km/h)").with_origin("/vehicle/speed").into(),
        TimeSeriesView::new("Acceleration (m/s²)").with_origin("/vehicle/accel").into(),
        TimeSeriesView::new("Pedals: command and driver").with_origin("/command").into(),
        TimeSeriesView::new("Lead gap (m) and TTC (s)").with_origin("/lead").into(),
        TimeSeriesView::new("Bus load per network (%)").with_origin("/bus/load").into(),
        TimeSeriesView::new("Latency (µs)").with_origin("/latency").into(),
    ]);
    let left = Vertical::new([
        Spatial2DView::new("Bird's-eye view").with_origin("/bev").into(),
        TextLogView::new("Events").with_origin("/events").into(),
    ])
    .with_row_shares([3.0, 1.0]);
    let can = Tabs::new([
        TextLogView::new("CAN trace: chassis").with_origin("/can/chassis").into(),
        TextLogView::new("CAN trace: ADAS CAN FD").with_origin("/can/adas").into(),
    ]);
    Blueprint::new(Horizontal::new([left.into(), plots.into(), can.into()]).with_column_shares([2.0, 2.0, 3.0]))
        .with_time_panel(TimePanel::new().with_state(rerun::blueprint::components::PanelState::Collapsed))
}

struct Logger {
    rec: RecordingStream,
    can_text: bool,
    last_busy: [Option<(f64, f64)>; 2],
    prev_t: Option<f64>,
    last_hello: String,
}

impl Logger {
    fn new(rec: RecordingStream, can_text: bool) -> rerun::RecordingStreamResult<Self> {
        let l = Self { rec, can_text, last_busy: [None, None], prev_t: None, last_hello: String::new() };
        l.styles()?;
        Ok(l)
    }

    fn styles(&self) -> rerun::RecordingStreamResult<()> {
        let line = |path: &str, name: &str, c: [u8; 3], w: f32| self.rec.log_static(path, &SeriesLines::new().with_names([name]).with_colors([rgb(c)]).with_widths([w]));
        line("vehicle/speed/host", "host", [220, 220, 220], 1.5)?;
        line("vehicle/speed/bus", "chassis CAN", BUS_COLORS[0], 1.5)?;
        line("vehicle/speed/set", "set speed", [120, 200, 120], 1.0)?;
        line("vehicle/speed/lead", "lead", [255, 200, 80], 1.5)?;
        line("vehicle/accel/estimate", "estimate", [220, 220, 220], 1.5)?;
        line("command/throttle", "throttle", [120, 200, 120], 1.5)?;
        line("command/brake", "brake", [240, 90, 90], 1.5)?;
        line("command/driver_throttle", "driver throttle", [120, 200, 120], 1.0)?;
        line("command/driver_brake", "driver brake", [240, 90, 90], 1.0)?;
        line("lead/gap", "gap", [255, 200, 80], 1.5)?;
        line("lead/ttc", "TTC (capped at 10 s)", [240, 90, 90], 1.5)?;
        for (b, n) in BUS_NAMES.iter().enumerate() {
            line(&format!("bus/load/{n}"), n, BUS_COLORS[b], 1.5)?;
        }
        line("latency/sample_to_command", "sample to command", [200, 160, 255], 1.5)?;
        line("latency/core_step", "core step", [160, 160, 160], 1.5)?;
        self.rec.log_static(
            "bev/ego",
            &Boxes2D::from_centers_and_half_sizes([bev(0.0, 0.0)], [[0.95, 2.3]]).with_colors([rgb(BUS_COLORS[0])]).with_labels(["ego"]),
        )
    }

    fn time(&self, frame: u64, t: f64) {
        self.rec.set_time_sequence("frame", frame as i64);
        self.rec.set_duration_secs("game_time", t);
    }

    fn scalar(&self, path: &str, v: f64) -> rerun::RecordingStreamResult<()> {
        self.rec.log(path, &Scalars::single(v))
    }

    fn step(&mut self, s: &Step) -> rerun::RecordingStreamResult<()> {
        if self.prev_t.is_some_and(|p| s.t + 1.0 < p) {
            self.last_busy = [None, None]; // a new run
        }
        self.prev_t = Some(s.t);
        self.time(s.frame, s.t);
        let kph = 3.6f64;
        self.scalar("vehicle/speed/host", s.speed as f64 * kph)?;
        self.scalar("vehicle/speed/bus", s.bus_speed as f64 * kph)?;
        if s.hud_flags & adapter_api::hud_flags::ACC != 0 {
            self.scalar("vehicle/speed/set", s.set_speed as f64 * kph)?;
        }
        if s.lead_speed.is_finite() {
            self.scalar("vehicle/speed/lead", s.lead_speed as f64 * kph)?;
        }
        self.scalar("vehicle/accel/estimate", s.accel as f64)?;
        self.scalar("command/throttle", s.throttle as f64)?;
        self.scalar("command/brake", s.brake as f64)?;
        self.scalar("command/driver_throttle", s.driver_throttle as f64)?;
        self.scalar("command/driver_brake", s.driver_brake as f64)?;
        if s.lead_gap.is_finite() {
            self.scalar("lead/gap", s.lead_gap as f64)?;
        }
        self.scalar("lead/ttc", if s.ttc.is_finite() { (s.ttc as f64).min(10.0) } else { 10.0 })?;
        if s.latency_ns > 0 {
            self.scalar("latency/sample_to_command", s.latency_ns as f64 / 1e3)?;
            self.scalar("latency/core_step", s.step_ns as f64 / 1e3)?;
        }
        for (b, snap) in s.buses.iter().enumerate() {
            match self.last_busy[b] {
                None => self.last_busy[b] = Some((snap.now, snap.busy_total)),
                Some((t0, busy0)) if snap.now - t0 >= LOAD_WINDOW_S => {
                    let load = (snap.busy_total - busy0) / (snap.now - t0);
                    self.scalar(&format!("bus/load/{}", BUS_NAMES[b]), load * 100.0)?;
                    self.last_busy[b] = Some((snap.now, snap.busy_total));
                }
                Some(_) => {}
            }
        }

        // Bird's-eye view: tracks (what the ADAS ECU believes) over actors (truth).
        let strips: Vec<Vec<[f32; 2]>> = s.actors.iter().map(|a| outline(a.x, a.y, a.yaw, a.length, a.width)).collect();
        self.rec.log("bev/actors", &LineStrips2D::new(strips).with_colors([Color::from_rgb(130, 130, 130)]).with_radii([0.08]))?;
        let color = |f: u8| {
            if f & track_flags::THREAT != 0 {
                Color::from_rgb(240, 90, 90)
            } else if f & track_flags::LEAD != 0 {
                Color::from_rgb(120, 220, 120)
            } else if f & track_flags::CONFIRMED != 0 {
                Color::from_rgb(255, 200, 80)
            } else {
                Color::from_rgb(120, 120, 120)
            }
        };
        let centers: Vec<[f32; 2]> = s.tracks.iter().map(|t| bev(t.x, t.y)).collect();
        let halves: Vec<[f32; 2]> = s.tracks.iter().map(|t| [t.width.max(0.5) / 2.0, t.half_length.max(0.5)]).collect();
        let colors: Vec<Color> = s.tracks.iter().map(|t| color(t.flags)).collect();
        let labels: Vec<String> = s
            .tracks
            .iter()
            .map(|t| {
                let mut l = String::new();
                if t.flags & track_flags::LEAD != 0 {
                    l.push_str("LEAD ");
                }
                if t.flags & track_flags::THREAT != 0 {
                    l.push_str("THREAT ");
                }
                l + &t.id.to_string()
            })
            .collect();
        self.rec.log("bev/tracks", &Boxes2D::from_centers_and_half_sizes(centers.clone(), halves).with_colors(colors.clone()).with_labels(labels))?;
        let vectors: Vec<[f32; 2]> = s.tracks.iter().map(|t| bev(t.vx, t.vy)).collect();
        self.rec.log("bev/velocity", &Arrows2D::from_vectors(vectors).with_origins(centers).with_colors(colors))?;
        let hud: Vec<&str> = HUD_BITS.iter().enumerate().filter(|(i, _)| s.hud_flags >> i & 1 == 1).map(|(_, n)| *n).collect();
        self.rec.log("bev/hud", &TextLog::new(if hud.is_empty() { "-".to_string() } else { hud.join(" ") }))
    }

    fn frames(&self, frame: u64, frames: &[CanFrame]) -> rerun::RecordingStreamResult<()> {
        if !self.can_text {
            return Ok(());
        }
        self.rec.set_time_sequence("frame", frame as i64);
        for f in frames {
            let t = if f.t.is_finite() { f.t } else { self.prev_t.unwrap_or(0.0) };
            self.rec.set_duration_secs("game_time", t);
            let (name, sigs) = decode(f.bus, f.id, &f.data);
            let hex: Vec<String> = f.data.iter().map(|b| format!("{b:02X}")).collect();
            let sigs: Vec<String> = sigs.iter().filter(|(n, v)| *v != 0.0 && *n != "CHECKSUM" && *n != "COUNTER").map(|(n, v)| format!("{n}={v}")).collect();
            let text = format!("{:03X} {:<22} {:<48} {}", f.id, name.unwrap_or("?"), hex.join(" "), sigs.join("  "));
            self.rec.log(format!("can/{}", BUS_NAMES[f.bus as usize & 1]), &TextLog::new(text))?;
        }
        Ok(())
    }

    fn event(&self, frame: u64, text: &str) -> rerun::RecordingStreamResult<()> {
        self.rec.set_time_sequence("frame", frame as i64);
        let warn = ["AEB", "IMPACT", "unavailable", "fault", "Fault"].iter().any(|w| text.contains(w));
        self.rec.log("events", &TextLog::new(text).with_level(if warn { rerun::TextLogLevel::WARN } else { rerun::TextLogLevel::INFO }))
    }

    fn packet(&mut self, p: &Packet) -> rerun::RecordingStreamResult<()> {
        match p {
            Packet::Hello { feature, host, vehicle, recording, .. } => {
                // Sent every second so late viewers learn the run; log it once per run.
                let text = format!("{feature} on {host} / {vehicle}  {recording}");
                if text != self.last_hello {
                    self.last_hello = text;
                    self.rec.log("events", &TextLog::new(self.last_hello.as_str()).with_level(rerun::TextLogLevel::INFO))?;
                }
                Ok(())
            }
            Packet::Step(s) => self.step(s),
            Packet::Frames { frame, frames } => self.frames(*frame, frames),
            Packet::Event { frame, text } => self.event(*frame, text),
            Packet::Control(_) => Ok(()),
        }
    }
}

/// The core over a recorded drive, as `replay` runs it, logging each step.
fn replay(log: &mut Logger, path: &Path) -> Result<usize, String> {
    let rec = record::Recording::open(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let feature = Feature::from_meta(&rec.meta)?;
    let initial = rec.meta.get("initial_request").map(|v| v == "true").unwrap_or(false);
    let name = feature.name().to_string();
    let mut core = Core::new(feature, initial);
    let err = |e: rerun::RecordingStreamError| e.to_string();
    for (i, f) in rec.frames.iter().enumerate() {
        if let Some(info) = &f.info {
            core.set_info(info);
            let hello = Packet::Hello {
                contract: adapter_api::CONTRACT_VERSION,
                feature: name.clone(),
                host: adapter_api::str_of(&info.host).to_string(),
                vehicle: adapter_api::str_of(&info.vehicle).to_string(),
                recording: path.display().to_string(),
            };
            log.packet(&hello).map_err(err)?;
        }
        if let Some(p) = &f.profile {
            core.set_brake_gain(p.brake_gain);
        }
        for fault in &f.faults {
            telem::apply_fault(&mut core, fault).map_err(|e| format!("step {i}: {e}"))?;
            log.event(f.sample.header.frame, &format!("fault: {fault:?}")).map_err(err)?;
        }
        let stick = f.stick.map(|s| Stick { throttle: s.throttle, brake: s.brake, steer: s.steer });
        let (c, event) = core.step(Inputs { sample: &f.sample, actors: &f.actors, stick });
        log.step(&telem::step(&core, &f.sample, &f.actors, &c, 0, 0)).map_err(err)?;
        log.frames(f.sample.header.frame, &telem::frames(&core)).map_err(err)?;
        if event != adas_core::Event::None {
            log.event(f.sample.header.frame, &format!("{event:?}")).map_err(err)?;
        }
    }
    Ok(rec.frames.len())
}

/// The viewer executable: `RERUN_VIEWER`, `rerun` on PATH, or the pip user install.
fn viewer() -> Option<String> {
    if let Ok(p) = std::env::var("RERUN_VIEWER") {
        return Some(p);
    }
    let on_path = std::env::var_os("PATH").is_some_and(|p| std::env::split_paths(&p).any(|d| d.join("rerun.exe").exists() || d.join("rerun").exists()));
    if on_path {
        return None; // the SDK's default lookup finds it
    }
    let appdata = std::env::var_os("APPDATA")?;
    let py = PathBuf::from(appdata).join("Python");
    std::fs::read_dir(&py).ok()?.filter_map(|e| e.ok()).map(|e| e.path().join("Scripts").join("rerun.exe")).find(|p| p.exists()).map(|p| p.display().to_string())
}

fn main() {
    let (mut file, mut mcap, mut save, mut can_text) = (None::<PathBuf>, None::<PathBuf>, None::<PathBuf>, true);
    let mut port = telemetry::RERUN_PORT;
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--file" => file = Some(args.next().expect("--file needs a path").into()),
            "--replay" => mcap = Some(args.next().expect("--replay needs a recording").into()),
            "--save" => save = Some(args.next().expect("--save needs a path").into()),
            "--port" => port = args.next().and_then(|v| v.parse().ok()).expect("--port needs a number"),
            "--no-can" => can_text = false,
            other => {
                eprintln!("unknown argument {other}; usage: adas-viz [--replay X.mcap | --file X.telem] [--save X.rrd] [--no-can] [--port N]");
                std::process::exit(2);
            }
        }
    }
    let builder = rerun::RecordingStreamBuilder::new("adas").with_blueprint(blueprint());
    let rec = match &save {
        Some(p) => builder.save(p),
        None => {
            let mut opts = rerun::SpawnOptions::default();
            opts.executable_path = viewer();
            builder.spawn_opts(&opts)
        }
    };
    let rec = rec.unwrap_or_else(|e| {
        eprintln!("cannot start Rerun: {e}\n(the viewer is `pip install --user rerun-sdk==0.42.0`; or set RERUN_VIEWER)");
        std::process::exit(1)
    });
    let mut log = Logger::new(rec.clone(), can_text).expect("log styles");

    let result = if let Some(p) = &mcap {
        replay(&mut log, p).map(|n| println!("logged {n} steps of {}", p.display()))
    } else if let Some(p) = &file {
        telemetry::read_file(p).map_err(|e| format!("{}: {e}", p.display())).and_then(|packets| {
            for pk in &packets {
                log.packet(pk).map_err(|e| e.to_string())?;
            }
            println!("logged {} packets from {}", packets.len(), p.display());
            Ok(())
        })
    } else {
        let mut rx = telemetry::Receiver::bind(port).unwrap_or_else(|e| {
            eprintln!("cannot listen on UDP {port}: {e}");
            std::process::exit(1)
        });
        println!("listening for adas-core telemetry on UDP {port}; Ctrl+C to stop");
        loop {
            match rx.try_recv() {
                Some(p) => {
                    if let Err(e) = log.packet(&p) {
                        eprintln!("rerun: {e}");
                    }
                }
                None => std::thread::sleep(std::time::Duration::from_millis(2)),
            }
        }
    };
    if let Err(e) = result {
        eprintln!("{e}");
        std::process::exit(1);
    }
    let _ = rec.flush_blocking();
}
