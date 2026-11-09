//! The ADAS core process. Connects to the adapter link and runs one
//! `Core::step` per host frame; records every engagement to MCAP.
//!
//! Usage: `adas-core [--feature speed-profile|test-driver] [--spin]
//!                   [--record DIR | --no-record] [--link NAME]`
//!
//! - `--spin`: busy-poll for samples (lowest latency, uses a CPU core);
//!   default: sleep on the adapter's sample event.
//! - test driver: I = throttle, K = brake, J / L = steer (read from Windows,
//!   so they work while GTA has focus).

use std::io::Write;
use std::path::PathBuf;
use std::sync::atomic::Ordering;

use adapter_api::shm::{DEFAULT_NAME, SharedLink};
use adapter_api::{CONTRACT_VERSION, Command, Header, MsgKind, VehicleSample, sample_flags, str_of, wall_time_ns};
use adas_core::{Core, Event, Feature, Inputs, Stick, layout};
use record::{Recorder, Stick3};

#[link(name = "user32", kind = "raw-dylib")]
unsafe extern "system" {
    fn GetAsyncKeyState(vkey: i32) -> i16;
}

fn key(vk: u8) -> bool {
    unsafe { GetAsyncKeyState(vk as i32) as u16 & 0x8000 != 0 }
}

/// I/J/K/L, ramped so a key press feels like a pedal, not a switch.
struct KeyStick {
    s: Stick,
}

impl KeyStick {
    fn read(&mut self, dt: f32) -> Stick {
        let ramp = |cur: f32, on: bool, rate: f32| if on { (cur + rate * dt).min(1.0) } else { (cur - 3.0 * rate * dt).max(0.0) };
        self.s.throttle = ramp(self.s.throttle, key(b'I'), 1.0);
        self.s.brake = ramp(self.s.brake, key(b'K'), 2.0);
        let want = match (key(b'J'), key(b'L')) {
            (true, false) => 1.0,
            (false, true) => -1.0,
            _ => 0.0,
        };
        let step = 2.5 * dt;
        self.s.steer += (want - self.s.steer).clamp(-step, step);
        self.s
    }
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
    let mut feature = Feature::speed_profile();
    let mut rec_dir: Option<PathBuf> = Some(PathBuf::from("recordings"));
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--spin" => spin = true,
            "--link" => name = args.next().expect("--link needs a name"),
            "--record" => rec_dir = Some(PathBuf::from(args.next().expect("--record needs a directory"))),
            "--no-record" => rec_dir = None,
            "--feature" => {
                feature = match args.next().as_deref() {
                    Some("speed-profile") => Feature::speed_profile(),
                    Some("test-driver") => Feature::TestDriver(Default::default()),
                    other => {
                        eprintln!("unknown feature {other:?}");
                        std::process::exit(2);
                    }
                }
            }
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
    link.core_pid.store(std::process::id(), Ordering::Release);
    let epoch = link.epoch.fetch_add(1, Ordering::AcqRel) + 1;
    println!(
        "adas-core: link {name}, contract v{CONTRACT_VERSION}, epoch {epoch}, pid {}, feature {}, {} mode, recording {}",
        std::process::id(),
        feature.name(),
        if spin { "spin" } else { "event" },
        rec_dir.as_ref().map(|d| d.display().to_string()).unwrap_or("off".into())
    );
    let _ = std::io::stdout().flush();

    let first = link.sample.read().1;
    let mut core = Core::new(feature.clone(), first.flags & sample_flags::ENGAGE_REQUEST != 0);
    let mut stick_src = KeyStick { s: Stick::default() };
    let test_driver = matches!(feature, Feature::TestDriver(_));

    let mut last_seq: Option<u32> = None;
    let mut last_game_us = 0u64;
    let mut cmd_seq = 0u32;
    let mut seen_writes = link.sample.writes();
    let mut info_writes = 0u32;
    let mut layout_seq = 0u32;
    let mut info = adapter_api::AdapterInfo::default();

    // Recording: one file per engagement, with one frame of pre-roll.
    let mut recorder: Option<Recorder> = None;
    let mut preroll: Option<(VehicleSample, adapter_api::ActorList, Option<Stick3>, Command)> = None;

    let mut one_way: Vec<u64> = Vec::with_capacity(1024);
    let mut step_ns: Vec<u64> = Vec::with_capacity(1024);
    let mut gaps = 0u32;
    let mut window_start = wall_time_ns();

    loop {
        if spin {
            while link.sample.writes() == seen_writes {
                core::hint::spin_loop();
            }
        } else if !link.wait_sample(100) && link.sample.writes() == seen_writes {
            continue;
        }
        seen_writes = link.sample.writes();
        let (_, s) = link.sample.read();
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

        // Adapter info changed (new vehicle): publish the sensor layout for it.
        if link.info.writes() != info_writes {
            info_writes = link.info.writes();
            info = link.info.read().1;
            layout_seq += 1;
            let mut l = layout::default_layout(info.box_min, info.box_max);
            l.header = Header { kind: MsgKind::SensorLayout, version: CONTRACT_VERSION, seq: layout_seq, wall_time_ns: wall_time_ns(), ..Default::default() };
            link.layout.write(&l);
            println!("adapter: {} / {}; sensor layout published ({} sensors)", str_of(&info.host), str_of(&info.vehicle), l.count);
        }

        let (_, actors) = link.actors.read();
        let dt = if last_game_us == 0 { 0.0 } else { s.header.game_time_us.saturating_sub(last_game_us) as f32 / 1e6 };
        last_game_us = s.header.game_time_us;
        let stick = test_driver.then(|| stick_src.read(dt.clamp(0.0, 0.1)));

        let (mut cmd, event) = core.step(Inputs { sample: &s, actors: &actors, stick });
        cmd_seq = cmd_seq.wrapping_add(1);
        cmd.answers_seq = s.header.seq;
        let t1 = wall_time_ns();
        cmd.header = Header { kind: MsgKind::Command, version: CONTRACT_VERSION, seq: cmd_seq, frame: s.header.frame, game_time_us: s.header.game_time_us, wall_time_ns: t1 };
        link.command.write(&cmd);

        // Everything below is off the critical path (the command is out).
        let stick3 = stick.map(|k| Stick3 { throttle: k.throttle, brake: k.brake, steer: k.steer });
        match event {
            Event::Engaged => {
                println!("engaged at frame {} ({:.2} m/s)", s.header.frame, s.speed);
                if let Some(dir) = &rec_dir {
                    let _ = std::fs::create_dir_all(dir);
                    let secs = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0);
                    let path = dir.join(format!("{secs}-{}.mcap", feature.name()));
                    let mut meta = feature.to_meta();
                    let pre_req = preroll.as_ref().map(|p| p.0.flags & sample_flags::ENGAGE_REQUEST != 0).unwrap_or(false);
                    meta.insert("initial_request".into(), pre_req.to_string());
                    meta.insert("host".into(), str_of(&info.host).into());
                    meta.insert("vehicle".into(), str_of(&info.vehicle).into());
                    match Recorder::create(&path, &meta) {
                        Ok(mut r) => {
                            let _ = r.info(&info);
                            if let Some((ps, pa, pst, pc)) = &preroll {
                                let _ = r.step(ps, pa, *pst, pc);
                            }
                            println!("recording {}", path.display());
                            recorder = Some(r);
                        }
                        Err(e) => eprintln!("cannot record to {}: {e}", path.display()),
                    }
                }
            }
            Event::CancelledByDriver | Event::Disengaged => {
                println!("{} at frame {}", if event == Event::Disengaged { "disengaged" } else { "cancelled by driver" }, s.header.frame);
            }
            Event::None => {}
        }
        if let Some(r) = recorder.as_mut()
            && let Err(e) = r.step(&s, &actors, stick3, &cmd)
        {
            eprintln!("recording failed: {e}");
            recorder = None;
        }
        if matches!(event, Event::CancelledByDriver | Event::Disengaged)
            && let Some(r) = recorder.take()
        {
            let _ = r.finish();
            println!("recording closed");
        }
        preroll = Some((s, actors, stick3, cmd));

        one_way.push(t1.saturating_sub(s.header.wall_time_ns));
        step_ns.push(t1.saturating_sub(t0));
        if t1.saturating_sub(window_start) >= 5_000_000_000 {
            one_way.sort_unstable();
            step_ns.sort_unstable();
            println!(
                "{} samples | sample->command p50 {:.1} us p99 {:.1} us max {:.1} us | core step p50 {:.1} us p99 {:.1} us | gaps {gaps} | actors {} | {:?}",
                one_way.len(),
                pct(&one_way, 0.5) as f64 / 1e3,
                pct(&one_way, 0.99) as f64 / 1e3,
                one_way.last().copied().unwrap_or(0) as f64 / 1e3,
                pct(&step_ns, 0.5) as f64 / 1e3,
                pct(&step_ns, 0.99) as f64 / 1e3,
                actors.count,
                core.state(),
            );
            let _ = std::io::stdout().flush();
            one_way.clear();
            step_ns.clear();
            gaps = 0;
            window_start = t1;
        }
    }
}
