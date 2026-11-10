//! The ADAS core process. Connects to the adapter link and runs one
//! `Core::step` per host frame; records the whole run to MCAP.
//!
//! Usage: `adas-core [--feature acc|autopilot|speed-profile|test-driver] [--spin]
//!                   [--record DIR | --no-record] [--link NAME]
//!                   [--telemetry PORT,PORT... | --no-telemetry] [--no-profile-save]`
//!
//! - `--no-profile-save`: load vehicle profiles but never write them (tests).
//!
//! - `--spin`: busy-poll for samples (lowest latency, uses a CPU core);
//!   default: sleep on the adapter's sample event.
//! - telemetry (on by default): each step goes to the engineering panel
//!   (`adas-panel`, port 47300) and the Rerun views (`adas-viz`, port 47302) over UDP on
//!   localhost, after the command is out. The panel's fault injections arrive
//!   on port 47301; they apply before the next step and are recorded.
//! - test driver: I = throttle, K = brake, J / L = steer (read from Windows,
//!   so they work while GTA has focus).

use std::io::Write;
use std::path::PathBuf;
use std::sync::atomic::Ordering;

use adapter_api::shm::{DEFAULT_NAME, SharedLink};
use adapter_api::{CONTRACT_VERSION, Header, MsgKind, sample_flags, str_of, wall_time_ns};
use adas_core::{Core, Event, Feature, Inputs, Stick, telem};
use record::{FaultRec, Profile, Recorder, Stick3};
use telemetry::Packet;

/// Work for the writer thread: the recording and the telemetry sends. The
/// control loop never does file or socket I/O itself, so a slow disk or a
/// busy machine delays the recording, not the next command.
enum Job {
    Info(adapter_api::AdapterInfo),
    Profile(Profile, u64),
    Fault(FaultRec, u64),
    Step(Box<StepRec>),
    Packets(Vec<Packet>),
}

struct StepRec {
    sample: adapter_api::VehicleSample,
    actors: adapter_api::ActorList,
    ranges: Option<adapter_api::RangeReport>,
    lanes: Option<adapter_api::LaneMap>,
    stick: Option<Stick3>,
    cmd: adapter_api::Command,
}

/// About a minute of steps at 60 fps: the writer can fall that far behind
/// before steps are dropped (and counted) rather than block the loop.
const JOB_QUEUE: usize = 4096;

fn writer(mut recorder: Option<Recorder>, mut tx: telemetry::Sender, jobs: std::sync::mpsc::Receiver<Job>) {
    for job in jobs {
        let r = match job {
            Job::Packets(ps) => {
                for p in &ps {
                    tx.send(p);
                }
                continue;
            }
            job => match recorder.as_mut() {
                None => continue,
                Some(rec) => match job {
                    Job::Info(i) => rec.info(&i),
                    Job::Profile(p, t) => rec.profile(&p, t),
                    Job::Fault(f, t) => rec.fault(&f, t),
                    Job::Step(st) => rec.step(&st.sample, &st.actors, st.ranges.as_ref(), st.lanes.as_ref(), st.stick, &st.cmd),
                    Job::Packets(_) => unreachable!(),
                },
            },
        };
        if let Err(e) = r {
            eprintln!("recording failed: {e}");
            recorder = None;
        }
    }
}

/// The core answers the game every frame, so it must not wait behind
/// whatever else the machine is doing (a 2026-11-09 GTA run lost 6 frames,
/// 97 ms, to that while the core's own step took 0.1 ms at most). It sleeps
/// on the sample event between frames, so a high priority costs nothing.
fn raise_priority() {
    #[cfg(windows)]
    unsafe {
        unsafe extern "system" {
            fn GetCurrentProcess() -> isize;
            fn GetCurrentThread() -> isize;
            fn SetPriorityClass(process: isize, class: u32) -> i32;
            fn SetThreadPriority(thread: isize, priority: i32) -> i32;
        }
        const HIGH_PRIORITY_CLASS: u32 = 0x80;
        const THREAD_PRIORITY_HIGHEST: i32 = 2;
        let p = SetPriorityClass(GetCurrentProcess(), HIGH_PRIORITY_CLASS) != 0;
        let t = SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST) != 0;
        if !(p && t) {
            eprintln!("could not raise the core's priority (process {p}, thread {t})");
        }
    }
}

/// `config/vehicles/<host>/<model>.toml` (D19): a file-system-safe slug.
fn slug(s: &str) -> String {
    let mut out = String::new();
    for c in s.chars() {
        if c.is_ascii_alphanumeric() {
            out.push(c.to_ascii_lowercase());
        } else if !out.ends_with('-') {
            out.push('-');
        }
    }
    out.trim_matches('-').to_string()
}

fn profile_path(host: &str, vehicle: &str) -> PathBuf {
    PathBuf::from("config").join("vehicles").join(slug(host)).join(format!("{}.toml", slug(vehicle)))
}

/// Reads `brake_gain = <x>` from a profile file.
fn load_brake_gain(path: &std::path::Path) -> Option<f32> {
    let text = std::fs::read_to_string(path).ok()?;
    text.lines().find_map(|l| {
        let (k, v) = l.split_once('=')?;
        (k.trim() == "brake_gain").then(|| v.trim().parse().ok()).flatten()
    })
}

fn save_profile(path: &std::path::Path, host: &str, vehicle: &str, brake_gain: f32) -> std::io::Result<()> {
    if let Some(dir) = path.parent() {
        std::fs::create_dir_all(dir)?;
    }
    std::fs::write(
        path,
        format!(
            "# Vehicle profile (D19) for {vehicle} on {host}.\n\
             # Learned on line by adas-core; edit by hand only with the core stopped.\n\n\
             # Deceleration per unit of brake demand (m/s^2).\n\
             brake_gain = {brake_gain:.3}\n"
        ),
    )
}

/// Keyboard state from a low-level keyboard hook. `GetAsyncKeyState` doesn't
/// see keys while GTA has focus (its raw input stops other processes' key
/// state updating), but a low-level hook sees every key event system-wide.
mod keyboard {
    use core::ffi::c_void;
    use std::sync::atomic::{AtomicBool, Ordering};

    #[repr(C)]
    struct KbdLl {
        vk: u32,
        scan: u32,
        flags: u32,
        time: u32,
        extra: usize,
    }

    #[repr(C)]
    struct Msg {
        hwnd: *mut c_void,
        message: u32,
        wparam: usize,
        lparam: isize,
        time: u32,
        pt: [i32; 2],
        private: u32,
    }

    type HookProc = unsafe extern "system" fn(i32, usize, isize) -> isize;

    #[link(name = "user32", kind = "raw-dylib")]
    unsafe extern "system" {
        fn SetWindowsHookExW(id: i32, proc_: HookProc, hmod: *mut c_void, thread: u32) -> *mut c_void;
        fn CallNextHookEx(h: *mut c_void, code: i32, w: usize, l: isize) -> isize;
        fn GetMessageW(msg: *mut Msg, hwnd: *mut c_void, min: u32, max: u32) -> i32;
    }

    #[link(name = "kernel32", kind = "raw-dylib")]
    unsafe extern "system" {
        fn GetModuleHandleW(name: *const u16) -> *mut c_void;
    }

    const WH_KEYBOARD_LL: i32 = 13;
    static VK: [AtomicBool; 256] = [const { AtomicBool::new(false) }; 256];
    static SCAN: [AtomicBool; 256] = [const { AtomicBool::new(false) }; 256];

    unsafe extern "system" fn hook(code: i32, w: usize, l: isize) -> isize {
        if code >= 0 && l != 0 {
            let k = unsafe { &*(l as *const KbdLl) };
            let down = matches!(w, 0x100 | 0x104); // WM_KEYDOWN, WM_SYSKEYDOWN
            VK[(k.vk & 0xFF) as usize].store(down, Ordering::Relaxed);
            SCAN[(k.scan & 0xFF) as usize].store(down, Ordering::Relaxed);
        }
        unsafe { CallNextHookEx(core::ptr::null_mut(), code, w, l) }
    }

    /// Installs the hook on its own thread (a low-level hook needs a message loop).
    pub fn start() {
        std::thread::spawn(|| unsafe {
            let h = SetWindowsHookExW(WH_KEYBOARD_LL, hook, GetModuleHandleW(core::ptr::null()), 0);
            if h.is_null() {
                eprintln!("keyboard hook failed; the test driver won't see keys");
                return;
            }
            let mut msg: Msg = core::mem::zeroed();
            while GetMessageW(&mut msg, core::ptr::null_mut(), 0, 0) > 0 {}
        });
    }

    /// A key by virtual-key code or by scan code (injected input may carry only one).
    pub fn down(vk: u8, scan: u8) -> bool {
        VK[vk as usize].load(Ordering::Relaxed) || SCAN[scan as usize].load(Ordering::Relaxed)
    }
}

/// Test-driver keys: (virtual key, scan code).
fn key(vk: u8) -> bool {
    let scan = match vk {
        b'I' => 0x17,
        b'J' => 0x24,
        b'K' => 0x25,
        b'L' => 0x26,
        _ => 0,
    };
    keyboard::down(vk, scan)
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
    let mut driver_monitor = false;
    let mut rec_dir: Option<PathBuf> = Some(PathBuf::from("recordings"));
    let mut telemetry_ports = vec![telemetry::PANEL_PORT, telemetry::RERUN_PORT];
    // Tests load vehicle profiles but never rewrite them.
    let mut save_profiles = true;
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--spin" => spin = true,
            "--no-profile-save" => save_profiles = false,
            "--no-telemetry" => telemetry_ports.clear(),
            "--telemetry" => {
                let list = args.next().expect("--telemetry needs ports, e.g. 47300,47302");
                telemetry_ports = list.split(',').map(|p| p.trim().parse().expect("bad port")).collect();
            }
            "--link" => name = args.next().expect("--link needs a name"),
            "--record" => rec_dir = Some(PathBuf::from(args.next().expect("--record needs a directory"))),
            "--no-record" => rec_dir = None,
            "--driver-monitor" => driver_monitor = true,
            "--feature" => {
                feature = match args.next().as_deref() {
                    Some("speed-profile") => Feature::speed_profile(),
                    Some("test-driver") => Feature::TestDriver(Default::default()),
                    Some("acc") => Feature::Acc(Default::default()),
                    Some("autopilot") => Feature::Autopilot(Default::default()),
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
    core.set_toggles(adas_core::Toggles { driver_monitor, ..Default::default() });
    let mut stick_src = KeyStick { s: Stick::default() };
    let test_driver = matches!(feature, Feature::TestDriver(_));
    if test_driver {
        keyboard::start();
    }

    let mut last_seq: Option<u32> = None;
    let mut last_game_us = 0u64;
    let mut cmd_seq = 0u32;
    let mut seen_writes = link.sample.writes();
    let mut info_writes = 0u32;
    let mut layout_seq = 0u32;
    let mut fans_sent = adapter_api::FanLayout::default();
    // A fresh core: no fans until it asks (an old layout may be in the link).
    link.fans.write(&fans_sent);
    // The vehicle profile in use: (path, host, vehicle, saved brake gain).
    let mut profile: Option<(PathBuf, String, String, f32)> = None;
    let mut last_profile_check = 0u64;

    // Recording: the whole core run, from its first sample, so a replay can
    // rebuild the perception state as well (the tracker's history starts
    // before any engagement).
    let mut recorder: Option<Recorder> = None;
    let mut recording_name = String::new();
    if let Some(dir) = &rec_dir {
        let _ = std::fs::create_dir_all(dir);
        let secs = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0);
        let path = dir.join(format!("{secs}-{}.mcap", feature.name()));
        let mut meta = feature.to_meta();
        meta.insert("initial_request".into(), (first.flags & sample_flags::ENGAGE_REQUEST != 0).to_string());
        match Recorder::create(&path, &meta) {
            Ok(r) => {
                println!("recording {}", path.display());
                recording_name = path.display().to_string();
                recorder = Some(r);
            }
            Err(e) => eprintln!("cannot record to {}: {e}", path.display()),
        }
    }

    // Telemetry to the viewers, and fault injection from the panel.
    let telemetry_on = !telemetry_ports.is_empty();
    let tx = telemetry::Sender::udp(&telemetry_ports);
    let recording = recorder.is_some();
    let (jobs, job_rx) = std::sync::mpsc::sync_channel::<Job>(JOB_QUEUE);
    std::thread::Builder::new().name("writer".into()).spawn(move || writer(recorder, tx, job_rx)).expect("writer thread");
    let mut dropped_jobs = 0u64;
    let mut post = move |j: Job| {
        if jobs.try_send(j).is_err() {
            dropped_jobs += 1;
            if dropped_jobs.is_power_of_two() {
                eprintln!("writer behind: {dropped_jobs} recording/telemetry jobs dropped");
            }
        }
    };
    raise_priority();
    let mut control = if telemetry_on {
        match telemetry::Receiver::bind(telemetry::CONTROL_PORT) {
            Ok(r) => Some(r),
            Err(e) => {
                eprintln!("panel control port {} unavailable ({e}); fault injection off", telemetry::CONTROL_PORT);
                None
            }
        }
    } else {
        None
    };
    if telemetry_on {
        println!("telemetry to ports {telemetry_ports:?}; panel control on {}", telemetry::CONTROL_PORT);
    }
    let mut last_hello = 0u64;
    let mut host_names = (String::new(), String::new());
    let mut events: Vec<String> = Vec::new();

    let mut one_way: Vec<u64> = Vec::with_capacity(1024);
    let mut step_ns: Vec<u64> = Vec::with_capacity(1024);
    let mut gaps = 0u32;
    let mut window_start = wall_time_ns();
    let mut aeb_logged = false;
    let mut prev_surround = 0u32;

    loop {
        if spin {
            while link.sample.writes() == seen_writes {
                core::hint::spin_loop();
            }
        } else if !link.wait_sample(100) && link.sample.writes() == seen_writes {
            continue;
        }
        seen_writes = link.sample.writes();
        // One core per link: a newer core took over (the adapter follows the
        // newest epoch), so this one stops rather than answer alongside it.
        if link.epoch.load(Ordering::Acquire) != epoch {
            println!("a newer core took the link (epoch {} after {epoch}): exiting", link.epoch.load(Ordering::Acquire));
            let _ = std::io::stdout().flush();
            std::process::exit(0);
        }
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

        // Adapter info changed (new vehicle): the core rebuilds its sensor
        // layout; publish it, and record the change in order.
        if link.info.writes() != info_writes {
            info_writes = link.info.writes();
            let info = link.info.read().1;
            core.set_info(&info);
            layout_seq += 1;
            let mut l = *core.layout();
            l.header = Header { kind: MsgKind::SensorLayout, version: CONTRACT_VERSION, seq: layout_seq, wall_time_ns: wall_time_ns(), ..Default::default() };
            link.layout.write(&l);
            post(Job::Info(info));
            println!("adapter: {} / {}; sensor layout published ({} sensors)", str_of(&info.host), str_of(&info.vehicle), l.count);
            // A new vehicle: start from its profile, if there is one.
            let (host, vehicle) = (str_of(&info.host).to_string(), str_of(&info.vehicle).to_string());
            host_names = (host.clone(), vehicle.clone());
            last_hello = 0;
            if profile.as_ref().is_none_or(|p| p.1 != host || p.2 != vehicle) {
                let path = profile_path(&host, &vehicle);
                match load_brake_gain(&path) {
                    Some(g) => {
                        core.set_brake_gain(g);
                        post(Job::Profile(Profile { brake_gain: g, ..Default::default() }, wall_time_ns()));
                        println!("profile {}: brake gain {g:.2}", path.display());
                    }
                    None => println!("no profile at {} yet; learning", path.display()),
                }
                let g = core.brake_gain();
                profile = Some((path, host, vehicle, g));
            }
        }

        let (_, actors) = link.actors.read();
        // The range fans' answers, while the adapter gives them.
        let (_, ranges) = link.ranges.read();
        let ranges = (ranges.count > 0).then_some(ranges);
        // The lane markings, while the adapter gives them.
        let (_, lanes) = link.lanes.read();
        let lanes = (lanes.count > 0).then_some(lanes);
        let dt = if last_game_us == 0 { 0.0 } else { s.header.game_time_us.saturating_sub(last_game_us) as f32 / 1e6 };
        last_game_us = s.header.game_time_us;
        let stick = test_driver.then(|| stick_src.read(dt.clamp(0.0, 0.1)));

        // Fault injection from the panel: applied before this step, starting
        // at this sample's time, and recorded so a replay applies it too.
        if let Some(rx) = control.as_mut() {
            while let Some(p) = rx.try_recv() {
                let Packet::Control(c) = p else { continue };
                let f = telem::fault_from_control(&c, s.header.game_time_us as f64 / 1e6);
                match telem::apply_fault(&mut core, &f) {
                    Ok(()) => {
                        let text = format!("fault at frame {}: {c:?}", s.header.frame);
                        println!("{text}");
                        events.push(text);
                        post(Job::Fault(f, s.header.wall_time_ns));
                    }
                    Err(e) => eprintln!("panel fault rejected: {e}"),
                }
            }
        }

        let (mut cmd, event) = core.step(Inputs { sample: &s, actors: &actors, ranges: ranges.as_ref(), lanes: lanes.as_ref(), stick });
        cmd_seq = cmd_seq.wrapping_add(1);
        cmd.answers_seq = s.header.seq;
        let t1 = wall_time_ns();
        cmd.header = Header { kind: MsgKind::Command, version: CONTRACT_VERSION, seq: cmd_seq, frame: s.header.frame, game_time_us: s.header.game_time_us, wall_time_ns: t1 };
        link.command.write(&cmd);

        // Everything below is off the critical path (the command is out).
        // The range fans: republished when the layout or its ACTIVE flag changes.
        if *core.fans() != fans_sent {
            fans_sent = *core.fans();
            layout_seq += 1;
            let mut f = fans_sent;
            f.header = Header { kind: MsgKind::FanLayout, version: CONTRACT_VERSION, seq: layout_seq, wall_time_ns: wall_time_ns(), ..Default::default() };
            link.fans.write(&f);
        }
        let mut hud = *core.hud();
        hud.header = Header { kind: MsgKind::Hud, version: CONTRACT_VERSION, seq: cmd_seq, frame: s.header.frame, game_time_us: s.header.game_time_us, wall_time_ns: wall_time_ns() };
        link.hud.write(&hud);
        let stick3 = stick.map(|k| Stick3 { throttle: k.throttle, brake: k.brake, steer: k.steer });
        let text = match event {
            Event::Engaged => Some(format!("engaged at frame {} ({:.2} m/s)", s.header.frame, s.speed)),
            Event::CancelledByDriver => Some(format!("cancelled by driver at frame {}", s.header.frame)),
            Event::Disengaged => Some(format!("disengaged at frame {}", s.header.frame)),
            Event::CancelledByAeb => Some(format!("AEB at frame {}: ACC cancelled", s.header.frame)),
            Event::Impact => Some(format!("IMPACT at frame {} ({:.1} m/s): everything cancelled", s.header.frame, s.speed)),
            Event::Unavailable(r) if core.takeover().is_some() => {
                Some(format!("TAKE OVER at frame {}: {} unavailable ({})", s.header.frame, feature.name(), r.text()))
            }
            Event::Unavailable(r) => Some(format!("{} unavailable at frame {}: {}", feature.name(), s.header.frame, r.text())),
            Event::TakenOver => Some(format!("driver took over at frame {}", s.header.frame)),
            Event::ControlledStop => Some(format!("takeover not answered at frame {} ({:.1} m/s): controlled stop, hazards on", s.header.frame, s.speed)),
            Event::None => None,
        };
        if let Some(t) = text {
            println!("{t}");
            events.push(t);
        }
        // The surround features, as they start to act.
        {
            use adapter_api::surround_flags as sf;
            for (bit, what) in [
                (sf::BSD_LEFT, "BSD: vehicle in the left blind spot"),
                (sf::BSD_RIGHT, "BSD: vehicle in the right blind spot"),
                (sf::BSD_WARN_LEFT, "BSD warning: signalling left into an occupied lane"),
                (sf::BSD_WARN_RIGHT, "BSD warning: signalling right into an occupied lane"),
                (sf::RCTA_LEFT, "RCTA: cross traffic from the left"),
                (sf::RCTA_RIGHT, "RCTA: cross traffic from the right"),
                (sf::RCTA_BRAKE, "RCTA braking"),
                (sf::PARK_BRAKE, "park assist braking"),
            ] {
                if hud.surround & bit != 0 && prev_surround & bit == 0 {
                    let t = format!("{what} at frame {} ({:.1} km/h)", s.header.frame, s.speed * 3.6);
                    println!("{t}");
                    events.push(t);
                }
            }
            prev_surround = hud.surround;
        }
        let surround_braking = hud.surround & (adapter_api::surround_flags::RCTA_BRAKE | adapter_api::surround_flags::PARK_BRAKE) != 0;
        if cmd.flags & adapter_api::command_flags::AEB != 0 && cmd.brake > 0.0 && !aeb_logged && !surround_braking {
            let t = format!("AEB braking at frame {} ({:.1} m/s, ttc {:.2} s, brake {:.2})", s.header.frame, s.speed, hud.ttc, cmd.brake);
            println!("{t}");
            events.push(t);
            aeb_logged = true;
        } else if cmd.flags & adapter_api::command_flags::AEB == 0 {
            aeb_logged = false;
        }
        if recording {
            post(Job::Step(Box::new(StepRec { sample: s, actors, ranges, lanes, stick: stick3, cmd })));
        }

        // Telemetry for the panel and the Rerun views.
        if telemetry_on {
            let mut ps = Vec::with_capacity(8);
            if t1.saturating_sub(last_hello) > 1_000_000_000 {
                last_hello = t1;
                ps.push(Packet::Hello {
                    contract: CONTRACT_VERSION,
                    feature: feature.name().to_string(),
                    host: host_names.0.clone(),
                    vehicle: host_names.1.clone(),
                    recording: recording_name.clone(),
                });
            }
            let latency = t1.saturating_sub(s.header.wall_time_ns).min(u32::MAX as u64) as u32;
            let step_time = t1.saturating_sub(t0).min(u32::MAX as u64) as u32;
            ps.push(Packet::Step(Box::new(telem::step(&core, &s, &actors, &cmd, latency, step_time))));
            ps.extend(telemetry::frame_packets(s.header.frame, telem::frames(&core)));
            for text in events.drain(..) {
                ps.push(Packet::Event { frame: s.header.frame, text });
            }
            post(Job::Packets(ps));
        } else {
            events.clear();
        }

        // Save the profile when the learned brake gain has drifted (every 10 s at most).
        if save_profiles && t1.saturating_sub(last_profile_check) > 10_000_000_000 {
            last_profile_check = t1;
            if let Some((path, host, vehicle, saved)) = profile.as_mut() {
                let g = core.brake_gain();
                if (g - *saved).abs() > 0.03 * saved.abs().max(1.0) {
                    match save_profile(path, host, vehicle, g) {
                        Ok(()) => {
                            println!("profile {}: brake gain {g:.2} saved", path.display());
                            *saved = g;
                        }
                        Err(e) => eprintln!("cannot save {}: {e}", path.display()),
                    }
                }
            }
        }

        one_way.push(t1.saturating_sub(s.header.wall_time_ns));
        step_ns.push(t1.saturating_sub(t0));
        if t1.saturating_sub(window_start) >= 5_000_000_000 {
            one_way.sort_unstable();
            step_ns.sort_unstable();
            println!(
                "{} samples | sample->command p50 {:.1} us p99 {:.1} us max {:.1} us | core step p50 {:.1} us p99 {:.1} us | gaps {gaps} | actors {} | {:?}{}",
                one_way.len(),
                pct(&one_way, 0.5) as f64 / 1e3,
                pct(&one_way, 0.99) as f64 / 1e3,
                one_way.last().copied().unwrap_or(0) as f64 / 1e3,
                pct(&step_ns, 0.5) as f64 / 1e3,
                pct(&step_ns, 0.99) as f64 / 1e3,
                actors.count,
                core.state(),
                stick.map(|k| format!(" | stick thr {:.2} brk {:.2} steer {:+.2}", k.throttle, k.brake, k.steer)).unwrap_or_default(),
            );
            let _ = std::io::stdout().flush();
            one_way.clear();
            step_ns.clear();
            gaps = 0;
            window_start = t1;
        }
    }
}
