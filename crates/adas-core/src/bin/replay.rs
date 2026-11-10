//! Replays a recorded drive through the core, without any game (D20).
//!
//! Usage: `replay <file.mcap>... [--check] [--csv OUT] [--candump OUT]
//!                [--telemetry OUT] [--udp [--speed X]]`
//!
//! `--candump` writes the bus traffic the core regenerates, in `candump -L`
//! format (chassis CAN as `can0`, ADAS CAN FD as `can1`).
//!
//! `--telemetry` writes the telemetry stream the live core would have sent
//! (for `adas-viz --file`). `--udp` sends it to the panel and `adas-viz` as
//! the live core does, paced at the recording's game time
//! (`--speed 2` = twice as fast), so the views can be tried without a game.
//!
//! Faults injected live from the panel are in the recording and apply before
//! the same step here.
//!
//! For each recorded step it feeds the core the same sample, actor list and
//! stick it saw live, and compares the command it computes now with the one it
//! sent then. `--check` exits non-zero on any difference: the core must be
//! deterministic, so a difference means its behaviour changed.

use std::io::Write;
use std::path::{Path, PathBuf};

use adapter_api::Command;
use adas_core::{Core, Feature, Inputs, Stick, telem};
use record::Recording;
use telemetry::Packet;

/// Where the regenerated telemetry goes, and how fast.
struct Out {
    tx: Option<telemetry::Sender>,
    /// `Some(speed)`: pace by game time.
    pace: Option<f64>,
}

fn same(a: &Command, b: &Command) -> bool {
    a.flags == b.flags
        && a.steer_angle.to_bits() == b.steer_angle.to_bits()
        && a.throttle.to_bits() == b.throttle.to_bits()
        && a.brake.to_bits() == b.brake.to_bits()
        && a.diag.iter().zip(&b.diag).all(|(x, y)| x.to_bits() == y.to_bits())
}

/// Ground truth, for scoring only: bumper gap to the nearest actor ahead in
/// the ego lane (|y| < 1.5 m), or NaN.
fn gt_gap(actors: &adapter_api::ActorList, front: f32) -> f32 {
    actors
        .as_slice()
        .iter()
        .filter(|a| a.y.abs() < 1.5 && a.x > 0.0)
        .map(|a| a.x - 0.5 * a.length - front)
        .fold(f32::NAN, |m, g| if m.is_nan() || g < m { g } else { m })
}

fn replay(path: &Path, csv: Option<&PathBuf>, candump: Option<&PathBuf>, out_t: &mut Out) -> Result<(usize, usize), String> {
    let rec = Recording::open(path).map_err(|e| format!("{}: {e}", path.display()))?;
    if rec.truncated {
        eprintln!("{}: the recording ends early (recorder killed); replaying the {} complete steps", path.display(), rec.frames.len());
    }
    let feature = Feature::from_meta(&rec.meta)?;
    let initial = rec.meta.get("initial_request").map(|v| v == "true").unwrap_or(false);
    let feature_name = feature.name().to_string();
    let mut core = Core::new(feature, initial);
    let mut out = match csv {
        Some(p) => Some(std::io::BufWriter::new(std::fs::File::create(p).map_err(|e| e.to_string())?)),
        None => None,
    };
    if let Some(w) = out.as_mut() {
        let _ = writeln!(w, "frame,t_s,speed,actors,flags,throttle,brake,steer,diag0,diag1,diag2,diag3,hud_flags,lead_gap,lead_speed,ttc,gt_gap,match");
    }
    let mut dump = match candump {
        Some(p) => Some(std::io::BufWriter::new(std::fs::File::create(p).map_err(|e| e.to_string())?)),
        None => None,
    };
    let (mut compared, mut diffs) = (0usize, 0usize);
    // How big the differences are: quantisation, or changed behaviour?
    let (mut d_thr, mut d_brk, mut d_steer, mut d_flags) = (0f32, 0f32, 0f32, 0usize);
    let mut front = rec.info.map(|i| i.box_max[0]).unwrap_or(2.3);
    let (mut host, mut vehicle) = (String::new(), String::new());
    let wall0 = std::time::Instant::now();
    let game0 = rec.frames.first().map(|f| f.sample.header.game_time_us).unwrap_or(0);
    for (i, f) in rec.frames.iter().enumerate() {
        let mut events = Vec::new();
        if let Some(info) = &f.info {
            core.set_info(info);
            front = info.box_max[0];
            (host, vehicle) = (adapter_api::str_of(&info.host).to_string(), adapter_api::str_of(&info.vehicle).to_string());
        }
        if let Some(p) = &f.profile {
            core.set_brake_gain(p.brake_gain);
            if p.brake_reference > 0.0 {
                core.set_brake_reference(p.brake_reference);
            }
        }
        for fault in &f.faults {
            telem::apply_fault(&mut core, fault).map_err(|e| format!("{}: step {i}: {e}", path.display()))?;
            events.push(format!("fault at frame {}: {fault:?}", f.sample.header.frame));
        }
        let stick = f.stick.map(|s| Stick { throttle: s.throttle, brake: s.brake, steer: s.steer });
        let (c, event) = core.step(Inputs { sample: &f.sample, actors: &f.actors, ranges: f.ranges.as_ref(), lanes: f.lanes.as_deref(), stick });
        if std::env::var_os("REPLAY_LANES").is_some() && i % 72 == 0 {
            // The lane map the host gave, and what the camera and lane centering made of it.
            let map: Vec<String> = f
                .lanes
                .as_deref()
                .map(|m| m.as_slice().iter().map(|l| format!("k{} n{} y0 {:+.1}", l.kind, l.count, l.as_slice().iter().find(|p| p[0] >= 0.0).map_or(f32::NAN, |p| p[1]))).collect())
                .unwrap_or_default();
            let lo = core.lane_out();
            let path = lo.path.map(|p| format!("c0 {:+.2} w {:.1} L {} R {}", p.c[0], p.width, p.left.is_some(), p.right.is_some())).unwrap_or("none".into());
            println!("step {i}: map [{}] | lanes {path} issue {:?} angle {:?} | avail {:?}", map.join(", "), lo.issue, lo.angle, core.availability().lanes);
        }
        if let Some(tx) = out_t.tx.as_mut() {
            if let Some(speed) = out_t.pace {
                let due = (f.sample.header.game_time_us.saturating_sub(game0)) as f64 / 1e6 / speed.max(0.01);
                let ahead = due - wall0.elapsed().as_secs_f64();
                if ahead > 0.0 {
                    std::thread::sleep(std::time::Duration::from_secs_f64(ahead.min(1.0)));
                }
            }
            if i % 72 == 0 {
                let recording = path.display().to_string();
                tx.send(&Packet::Hello { contract: adapter_api::CONTRACT_VERSION, feature: feature_name.clone(), host: host.clone(), vehicle: vehicle.clone(), recording });
            }
            if event != adas_core::Event::None {
                events.push(format!("{event:?} at frame {}", f.sample.header.frame));
            }
            tx.send(&Packet::Step(Box::new(telem::step(&core, &f.sample, &f.actors, &c, 0, 0))));
            for p in telemetry::frame_packets(f.sample.header.frame, telem::frames(&core)) {
                tx.send(&p);
            }
            for text in events {
                tx.send(&Packet::Event { frame: f.sample.header.frame, text });
            }
        }
        if let Some(w) = dump.as_mut() {
            for (bus, fr) in &core.vehicle().last_frames {
                let hex: String = fr.data.iter().map(|b| format!("{b:02X}")).collect();
                let t = fr.delivered;
                // candump -L: classic `ID#DATA`, CAN FD `ID##<flags>DATA` (flags 1 = BRS).
                let _ = match *bus {
                    "adas" => writeln!(w, "({t:.6}) can1 {:03X}##1{hex}", fr.id),
                    _ => writeln!(w, "({t:.6}) can0 {:03X}#{hex}", fr.id),
                };
            }
        }
        let ok = match &f.command {
            Some(live) => {
                compared += 1;
                let ok = same(&c, live);
                if !ok {
                    diffs += 1;
                    let d = |a: f32, b: f32| if a.is_finite() && b.is_finite() { (a - b).abs() } else { 0.0 };
                    d_thr = d_thr.max(d(c.throttle, live.throttle));
                    d_brk = d_brk.max(d(c.brake, live.brake));
                    d_steer = d_steer.max(d(c.steer_angle, live.steer_angle));
                    d_flags += (c.flags != live.flags) as usize;
                    if diffs <= 3 {
                        eprintln!(
                            "  step {i} (frame {}): live flags {} thr {} brk {} steer {} | replay flags {} thr {} brk {} steer {}",
                            f.sample.header.frame, live.flags, live.throttle, live.brake, live.steer_angle, c.flags, c.throttle, c.brake, c.steer_angle
                        );
                    }
                }
                ok
            }
            None => true,
        };
        if let Some(w) = out.as_mut() {
            let _ = writeln!(
                w,
                "{},{:.3},{:.3},{},{},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{},{:.2},{:.2},{:.2},{:.2},{}",
                f.sample.header.frame,
                f.sample.header.game_time_us as f64 / 1e6,
                f.sample.speed,
                f.actors.count,
                c.flags,
                c.throttle,
                c.brake,
                c.steer_angle,
                c.diag[0],
                c.diag[1],
                c.diag[2],
                c.diag[3],
                core.hud().flags,
                core.hud().lead_gap,
                core.hud().lead_speed,
                core.hud().ttc,
                gt_gap(&f.actors, front),
                ok as u8
            );
        }
    }
    if diffs > 0 {
        eprintln!("  largest differences: throttle {d_thr:.5}, brake {d_brk:.5}, steer {d_steer:.5} rad; flags differ in {d_flags} steps");
    }
    Ok((compared, diffs))
}

fn main() {
    let mut files = Vec::new();
    let mut check = false;
    let mut csv: Option<PathBuf> = None;
    let mut candump: Option<PathBuf> = None;
    let mut telemetry_file: Option<PathBuf> = None;
    let (mut udp, mut speed) = (false, 1.0f64);
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--check" => check = true,
            "--csv" => csv = Some(PathBuf::from(args.next().expect("--csv needs a path"))),
            "--candump" => candump = Some(PathBuf::from(args.next().expect("--candump needs a path"))),
            "--telemetry" => telemetry_file = Some(PathBuf::from(args.next().expect("--telemetry needs a path"))),
            "--udp" => udp = true,
            "--speed" => speed = args.next().and_then(|v| v.parse().ok()).expect("--speed needs a number"),
            _ => files.push(PathBuf::from(a)),
        }
    }
    if files.is_empty() {
        eprintln!("usage: replay <file.mcap>... [--check] [--csv OUT] [--candump OUT] [--telemetry OUT] [--udp [--speed X]]");
        std::process::exit(2);
    }
    let mut out = match (&telemetry_file, udp) {
        (Some(p), _) => Out {
            tx: Some(telemetry::Sender::file(p).unwrap_or_else(|e| {
                eprintln!("{}: {e}", p.display());
                std::process::exit(2)
            })),
            pace: None,
        },
        (None, true) => Out { tx: Some(telemetry::Sender::udp(&[telemetry::PANEL_PORT, telemetry::RERUN_PORT])), pace: Some(speed) },
        (None, false) => Out { tx: None, pace: None },
    };
    let mut failed = false;
    for f in &files {
        let r = replay(f, csv.as_ref(), candump.as_ref(), &mut out);
        if let Some(tx) = out.tx.as_mut() {
            tx.flush();
        }
        match r {
            Ok((n, 0)) => println!("{}: {n} steps, identical", f.display()),
            Ok((n, d)) => {
                println!("{}: {n} steps, {d} differ", f.display());
                failed = true;
            }
            Err(e) => {
                println!("{e}");
                failed = true;
            }
        }
    }
    if check && failed {
        std::process::exit(1);
    }
}
