//! Replays a recorded drive through the core, without any game (D20).
//!
//! Usage: `replay <file.mcap>... [--check] [--csv OUT]`
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

fn replay(path: &Path, csv: Option<&PathBuf>) -> Result<(usize, usize), String> {
    let rec = Recording::open(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let feature = Feature::from_meta(&rec.meta)?;
    let initial = rec.meta.get("initial_request").map(|v| v == "true").unwrap_or(false);
    let mut core = Core::new(feature, initial);
    let mut out = match csv {
        Some(p) => Some(std::io::BufWriter::new(std::fs::File::create(p).map_err(|e| e.to_string())?)),
        None => None,
    };
    if let Some(w) = out.as_mut() {
        let _ = writeln!(w, "frame,speed,actors,flags,throttle,brake,steer,diag0,diag1,diag2,diag3,match");
    }
    let (mut compared, mut diffs) = (0usize, 0usize);
    for (i, f) in rec.frames.iter().enumerate() {
        let stick = f.stick.map(|s| Stick { throttle: s.throttle, brake: s.brake, steer: s.steer });
        let (c, _) = core.step(Inputs { sample: &f.sample, actors: &f.actors, stick });
        let ok = match &f.command {
            Some(live) => {
                compared += 1;
                let ok = same(&c, live);
                if !ok {
                    diffs += 1;
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
                "{},{:.3},{},{},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{}",
                f.sample.header.frame, f.sample.speed, f.actors.count, c.flags, c.throttle, c.brake, c.steer_angle, c.diag[0], c.diag[1], c.diag[2], c.diag[3], ok as u8
            );
        }
    }
    Ok((compared, diffs))
}

fn main() {
    let mut files = Vec::new();
    let mut check = false;
    let mut csv: Option<PathBuf> = None;
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--check" => check = true,
            "--csv" => csv = Some(PathBuf::from(args.next().expect("--csv needs a path"))),
            _ => files.push(PathBuf::from(a)),
        }
    }
    if files.is_empty() {
        eprintln!("usage: replay <file.mcap>... [--check] [--csv OUT]");
        std::process::exit(2);
    }
    let mut failed = false;
    for f in &files {
        match replay(f, csv.as_ref()) {
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
