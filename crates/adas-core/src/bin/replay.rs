//! Replays a recorded drive through the core, without any game (D20).
//!
//! Usage: `replay <file.mcap>... [--check] [--csv OUT] [--candump OUT]`
//!
//! `--candump` writes the bus traffic the core regenerates, in `candump -L`
//! format (chassis CAN as `can0`, ADAS CAN FD as `can1`).
//!
//! For each recorded step it feeds the core the same sample, actor list and
//! stick it saw live, and compares the command it computes now with the one it
//! sent then. `--check` exits non-zero on any difference: the core must be
//! deterministic, so a difference means its behaviour changed.

use std::io::Write;
use std::path::{Path, PathBuf};

use adapter_api::Command;
use adas_core::{Core, Feature, Inputs, Stick};
use record::Recording;

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

fn replay(path: &Path, csv: Option<&PathBuf>, candump: Option<&PathBuf>) -> Result<(usize, usize), String> {
    let rec = Recording::open(path).map_err(|e| format!("{}: {e}", path.display()))?;
    if rec.truncated {
        eprintln!("{}: the recording ends early (recorder killed); replaying the {} complete steps", path.display(), rec.frames.len());
    }
    let feature = Feature::from_meta(&rec.meta)?;
    let initial = rec.meta.get("initial_request").map(|v| v == "true").unwrap_or(false);
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
    for (i, f) in rec.frames.iter().enumerate() {
        if let Some(info) = &f.info {
            core.set_info(info);
            front = info.box_max[0];
        }
        if let Some(p) = &f.profile {
            core.set_brake_gain(p.brake_gain);
        }
        let stick = f.stick.map(|s| Stick { throttle: s.throttle, brake: s.brake, steer: s.steer });
        let (c, _) = core.step(Inputs { sample: &f.sample, actors: &f.actors, stick });
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
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--check" => check = true,
            "--csv" => csv = Some(PathBuf::from(args.next().expect("--csv needs a path"))),
            "--candump" => candump = Some(PathBuf::from(args.next().expect("--candump needs a path"))),
            _ => files.push(PathBuf::from(a)),
        }
    }
    if files.is_empty() {
        eprintln!("usage: replay <file.mcap>... [--check] [--csv OUT] [--candump OUT]");
        std::process::exit(2);
    }
    let mut failed = false;
    for f in &files {
        match replay(f, csv.as_ref(), candump.as_ref()) {
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
