//! The ADAS core, first version: connects to the adapter link and runs the
//! speed controller once per host frame.
//!
//! For now the only feature is a scripted speed profile (8, 15, 5, 12 m/s, then
//! stop and hold), which drives the end-to-end test of spike S3. It knows
//! nothing about the host game.
//!
//! Usage: `adas-core [--spin] [--link NAME]`
//! - default: sleep on the adapter's sample event between frames;
//! - `--spin`: busy-poll for samples (lowest latency, uses a CPU core).

use std::io::Write;
use std::sync::atomic::Ordering;

use adapter_api::shm::{DEFAULT_NAME, SharedLink};
use adapter_api::{
    CONTRACT_VERSION, Command, Header, MsgKind, VehicleSample, command_flags, sample_flags, wall_time_ns,
};
use control::{Config, SpeedController};

/// (seconds, target m/s). After the last segment, the target stays at 0 (hold).
const PROFILE: [(f32, f32); 5] = [(10.0, 8.0), (10.0, 15.0), (10.0, 5.0), (9.0, 12.0), (9.0, 0.0)];

fn target_at(t: f32) -> f32 {
    let mut acc = 0.0;
    for (len, tgt) in PROFILE {
        if t < acc + len {
            return tgt;
        }
        acc += len;
    }
    0.0
}

fn pct(sorted: &[u64], p: f64) -> u64 {
    if sorted.is_empty() {
        return 0;
    }
    sorted[((sorted.len() - 1) as f64 * p).round() as usize]
}

fn main() {
    let mut spin = false;
    let mut name = DEFAULT_NAME.to_string();
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--spin" => spin = true,
            "--link" => name = args.next().expect("--link needs a name"),
            other => {
                eprintln!("unknown argument {other}");
                std::process::exit(2);
            }
        }
    }

    let link = match SharedLink::open(&name) {
        Ok(l) => l,
        Err(e) => {
            eprintln!("cannot open link {name}: {e}");
            std::process::exit(1);
        }
    };
    let epoch = link.epoch.fetch_add(1, Ordering::AcqRel) + 1;
    println!("adas-core: link {name}, contract v{CONTRACT_VERSION}, epoch {epoch}, {} mode", if spin { "spin" } else { "event" });
    let _ = std::io::stdout().flush();

    let mut ctl = SpeedController::new(Config::default());
    let mut engaged = false;
    // Treat a request that is already set at start-up as old: a fresh core
    // must see its own rising edge (it never re-engages on its own).
    let mut prev_request = link.sample.read().1.flags & sample_flags::ENGAGE_REQUEST != 0;
    let mut profile_t = 0.0f32;
    let mut last_seq: Option<u32> = None;
    let mut last_game_us = 0u64;
    let mut cmd_seq = 0u32;
    let mut seen_writes = link.sample.writes();

    let mut one_way: Vec<u64> = Vec::with_capacity(1024);
    let mut step_ns: Vec<u64> = Vec::with_capacity(1024);
    let mut gaps = 0u32;
    let mut window_start = wall_time_ns();

    loop {
        // Wait for a new sample.
        if spin {
            while link.sample.writes() == seen_writes {
                core::hint::spin_loop();
            }
        } else if !link.wait_sample(100) && link.sample.writes() == seen_writes {
            continue;
        }
        seen_writes = link.sample.writes();
        let (_, s): (u32, VehicleSample) = link.sample.read();
        if last_seq == Some(s.header.seq) {
            continue;
        }
        let t0 = wall_time_ns();
        if let Some(prev) = last_seq
            && s.header.seq != prev.wrapping_add(1)
        {
            gaps += 1;
        }
        last_seq = Some(s.header.seq);

        let request = s.flags & sample_flags::ENGAGE_REQUEST != 0;
        if request && !prev_request {
            engaged = true;
            ctl.reset(s.speed);
            profile_t = 0.0;
            println!("engaged at frame {} ({:.2} m/s)", s.header.frame, s.speed);
        } else if !request && engaged {
            engaged = false;
            println!("disengaged at frame {}", s.header.frame);
        }
        prev_request = request;

        let dt = if last_game_us == 0 { 0.0 } else { s.header.game_time_us.saturating_sub(last_game_us) as f32 / 1e6 };
        last_game_us = s.header.game_time_us;
        let paused = s.flags & sample_flags::PAUSED != 0;

        let mut cmd = Command::default();
        if engaged && !paused {
            let target = target_at(profile_t);
            profile_t += dt;
            let o = ctl.step(target, s.speed, dt);
            cmd.flags = command_flags::ENGAGED | if o.hold { command_flags::HOLD } else { 0 };
            cmd.throttle = o.throttle;
            cmd.brake = o.brake;
            cmd.steer_angle = 0.0;
            cmd.diag = [target, o.reference, o.ref_accel, o.demand];
        } else if engaged {
            cmd.flags = command_flags::ENGAGED;
        }
        cmd_seq = cmd_seq.wrapping_add(1);
        cmd.answers_seq = s.header.seq;
        let t1 = wall_time_ns();
        cmd.header = Header {
            kind: MsgKind::Command,
            version: CONTRACT_VERSION,
            seq: cmd_seq,
            frame: s.header.frame,
            game_time_us: s.header.game_time_us,
            wall_time_ns: t1,
        };
        link.command.write(&cmd);

        one_way.push(t1.saturating_sub(s.header.wall_time_ns));
        step_ns.push(t1.saturating_sub(t0));
        if t1.saturating_sub(window_start) >= 5_000_000_000 {
            one_way.sort_unstable();
            step_ns.sort_unstable();
            println!(
                "{} samples | sample->command p50 {:.1} us p99 {:.1} us max {:.1} us | core step p50 {:.1} us p99 {:.1} us | gaps {gaps} | {}",
                one_way.len(),
                pct(&one_way, 0.5) as f64 / 1e3,
                pct(&one_way, 0.99) as f64 / 1e3,
                one_way.last().copied().unwrap_or(0) as f64 / 1e3,
                pct(&step_ns, 0.5) as f64 / 1e3,
                pct(&step_ns, 0.99) as f64 / 1e3,
                if engaged { "engaged" } else { "idle" },
            );
            let _ = std::io::stdout().flush();
            one_way.clear();
            step_ns.clear();
            gaps = 0;
            window_start = t1;
        }
    }
}
