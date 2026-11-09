//! The GTA V adapter (D22). Loaded by Script Hook V as `adas.asi`.
//!
//! Every frame it writes the actor list and a vehicle sample to the shared
//! link, wakes the core, waits briefly for the command that answers this very
//! sample (reactive mode, D23), and applies it. It owns the safe state (D24):
//! - the core process exits → released the same frame;
//! - no fresh command for 6 frames (a stalled core) → released;
//! - the driver brakes or steers → cancelled; the accelerator overrides;
//! - on release, steering stops at once, throttle is dropped and any brake
//!   fades out over about a second; TAKE OVER shows on the HUD.
//!
//! Driver keys: F10 engage / cancel ACC · F3 cancel · Page Up / Page Down set speed ±5 km/h
//! · End cycle the time gap (Home opens the Rockstar overlay). AEB is always armed.
//! Test keys: F6 60 fps cap · F7 teleport to the next test spot · F8 spawn a
//! Sultan · F9 GTA AI drives · F11 spawn a lead car ahead (12 m/s) · F5 the
//! lead brakes hard · F4 spawn a stopped car ahead.
//!
//! Logs: `<game folder>/adas-logs/adas-<unix time>.log` and `-frames.csv`.

mod actors;
mod actuate;
mod input;
mod n;
mod tools;
mod win;

use core::ffi::{CStr, c_void};
use std::ffi::CString;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::panic::{AssertUnwindSafe, catch_unwind};
use std::path::PathBuf;
use std::sync::atomic::{AtomicPtr, Ordering};

use adapter_api::shm::{DEFAULT_NAME, SharedLink};
use adapter_api::{
    ActorList, AdapterInfo, CONTRACT_VERSION, Header, MsgKind, VehicleSample, buttons, caps, command_flags, fixed_str, hud_flags, sample_flags,
    wall_time_ns,
};
use shv_sys::{Vector3, invoke};

const WATCHDOG_FRAMES: u32 = 6;
const REPLY_WAIT_NS: u64 = 1_000_000;
const CAP_60_NS: u64 = 16_666_667;
const VK_F6: i32 = 0x75;
const VK_F7: i32 = 0x76;
const VK_F8: i32 = 0x77;
const VK_F9: i32 = 0x78;
const VK_F10: i32 = 0x79;
const VK_F3: i32 = 0x72;
const VK_F4: i32 = 0x73;
const VK_F5: i32 = 0x74;
const VK_F11: i32 = 0x7A;
const VK_PRIOR: i32 = 0x21;
const VK_NEXT: i32 = 0x22;
const VK_END: i32 = 0x23;

static MODULE: AtomicPtr<c_void> = AtomicPtr::new(core::ptr::null_mut());

/// # Safety
/// Called only by the Windows loader.
#[unsafe(no_mangle)]
pub unsafe extern "system" fn DllMain(module: *mut c_void, reason: u32, _reserved: *mut c_void) -> i32 {
    match reason {
        1 => {
            MODULE.store(module, Ordering::Relaxed);
            unsafe {
                win::DisableThreadLibraryCalls(module);
                shv_sys::scriptRegister(module, script_main);
            }
        }
        0 => unsafe { shv_sys::scriptUnregister(module) },
        _ => {}
    }
    1
}

extern "C" fn script_main() {
    if let Err(payload) = catch_unwind(AssertUnwindSafe(run)) {
        let msg = payload
            .downcast_ref::<&str>()
            .map(|s| s.to_string())
            .or_else(|| payload.downcast_ref::<String>().cloned())
            .unwrap_or_else(|| "unknown panic".into());
        if let Ok(mut f) = File::create(log_dir().join(format!("{}-panic.txt", run_stem()))) {
            let _ = writeln!(f, "PANIC: {msg}");
        }
    }
    loop {
        unsafe { shv_sys::scriptWait(0) };
    }
}

fn log_dir() -> PathBuf {
    let mut buf = [0u16; 1024];
    let len = unsafe { win::GetModuleFileNameW(MODULE.load(Ordering::Relaxed), buf.as_mut_ptr(), buf.len() as u32) };
    let path = PathBuf::from(String::from_utf16_lossy(&buf[..len as usize]));
    let dir = path.parent().map(|p| p.join("adas-logs")).unwrap_or_else(|| PathBuf::from("adas-logs"));
    let _ = std::fs::create_dir_all(&dir);
    dir
}

fn run_stem() -> String {
    let secs = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0);
    format!("adas-{secs}")
}

struct Keys {
    prev: [bool; 12],
}

impl Keys {
    fn pressed(&mut self, slot: usize, vk: i32) -> bool {
        let down = win::key_down(vk);
        let edge = down && !self.prev[slot];
        self.prev[slot] = down;
        edge
    }
}

unsafe fn draw_text(x: f32, y: f32, text: &str, rgb: (i32, i32, i32)) {
    let s = CString::new(text).unwrap_or_default();
    unsafe {
        let () = invoke!(n::SET_TEXT_FONT, 0i32);
        let () = invoke!(n::SET_TEXT_SCALE, 0.0f32, 0.32f32);
        let () = invoke!(n::SET_TEXT_COLOUR, rgb.0, rgb.1, rgb.2, 255i32);
        let () = invoke!(n::SET_TEXT_OUTLINE);
        let () = invoke!(n::BEGIN_TEXT_COMMAND_DISPLAY_TEXT, c"STRING");
        let () = invoke!(n::ADD_TEXT_COMPONENT_SUBSTRING_PLAYER_NAME, s.as_c_str());
        let () = invoke!(n::END_TEXT_COMMAND_DISPLAY_TEXT, x, y, 0i32);
    }
}

fn pct(sorted: &[u64], p: f64) -> u64 {
    if sorted.is_empty() {
        return 0;
    }
    sorted[((sorted.len() - 1) as f64 * p).round() as usize]
}

/// Wheel spin speeds (rad/s, positive forward) in host order.
fn wheel_omegas(base: usize, off: &gta_edition::Offsets) -> [f32; 4] {
    let mut out = [f32::NAN; 4];
    let (Some(arr), Some(count)) = (win::read_u64(base + off.wheel_array), win::read_u32(base + off.wheel_count)) else { return out };
    for (i, o) in out.iter_mut().enumerate().take(count.min(4) as usize) {
        if let Some(w) = win::read_u64(arr as usize + i * 8)
            && let Some(v) = win::read_f32(w as usize + off.wheel_omega)
        {
            *o = -v;
        }
    }
    out
}

/// Why control was handed back.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Release {
    CoreExited,
    Stalled,
}

fn run() {
    let dir = log_dir();
    let stem = run_stem();
    let mut log = BufWriter::new(File::create(dir.join(format!("{stem}.log"))).expect("create log"));
    let mut csv = BufWriter::new(File::create(dir.join(format!("{stem}-frames.csv"))).expect("create csv"));
    let _ = writeln!(
        csv,
        "frame,wall_ms,frame_ms,work_us,seq,rtt_us,cmd_age,state,speed,cmd_flags,thr,brk,steer_cmd,steer_angle,\
         drv_thr,drv_brk,drv_steer,actors,in_view,los_age_max,rays,sync_rays,ray_fails,diag0,diag1,diag2,diag3"
    );

    let version = unsafe { shv_sys::getGameVersion() };
    let edition = gta_edition::for_game_version(version);
    let off = edition.offsets;
    let _ = writeln!(log, "ADAS adapter started: {} (eGameVersion {version}, verified {}), contract v{CONTRACT_VERSION}", edition.name, edition.verified);

    let link = match SharedLink::open(DEFAULT_NAME) {
        Ok(l) => l,
        Err(e) => {
            let _ = writeln!(log, "LINK open failed: {e}. The adapter is idle.");
            let _ = log.flush();
            return;
        }
    };
    let _ = log.flush();

    let mut keys = Keys { prev: [false; 12] };
    let mut lead: Option<tools::Lead> = None;
    let mut aeb_last = false;
    let mut fcw_last = false;
    // D11: the core was engaged last fresh frame; hazards; the last chime.
    let mut core_was_engaged = false;
    let mut hazards_on = false;
    let mut last_chime = 0u64;
    let mut cap60 = false;
    let mut spot = 0usize;
    // Set once a test tool has been used (F7 or F8): police stay away from then on.
    let mut test_mode = false;
    let mut test_car = 0i32;
    let mut ai = false;
    let mut engage_req = false;
    let mut was_active = false;
    let mut hold_on = false;
    let mut takeover_until = 0u64;
    let mut takeover_why = "";
    let mut seen_epoch = link.epoch.load(Ordering::Acquire);
    let mut watch = win::ProcessWatch::new();
    let mut steering = actuate::Steering::new();
    let mut ramp = actuate::ReleaseRamp::new();
    let mut actors_sys = actors::Actors::new();
    let mut actor_list = ActorList::default();
    let mut info_for = 0i32;
    let mut info_seq = 0u32;
    let mut info_frames = 0u32;
    let mut last_brake = 0.0f32;

    let mut seq = 0u32;
    let mut game_time_us = 0u64;
    let mut last_tick = wall_time_ns();
    let mut win_start = last_tick;
    let mut win_frames = 0u32;
    let mut win_rtt: Vec<u64> = Vec::new();
    let mut win_work: Vec<u64> = Vec::new();
    let mut win_miss = 0u32;
    let mut win_steer_ns = 0u64;
    let mut win_steer_n = 0u64;
    let mut hud2 = String::new();

    loop {
        if cap60 {
            while wall_time_ns() < last_tick + CAP_60_NS {
                core::hint::spin_loop();
            }
        }
        let t_frame = wall_time_ns();
        let frame_ns = t_frame - last_tick;
        last_tick = t_frame;

        let ped: i32 = unsafe { invoke!(n::PLAYER_PED_ID) };
        let in_vehicle: bool = unsafe { invoke!(n::IS_PED_IN_ANY_VEHICLE, ped, false) };
        let veh: i32 = if in_vehicle { unsafe { invoke!(n::GET_VEHICLE_PED_IS_IN, ped, false) } } else { 0 };
        let base = if in_vehicle { unsafe { shv_sys::getScriptHandleBaseAddress(veh) as usize } } else { 0 };
        let dt: f32 = unsafe { invoke!(n::GET_FRAME_TIME) };
        let frame: i32 = unsafe { invoke!(n::GET_FRAME_COUNT) };
        let pause_menu: bool = unsafe { invoke!(n::IS_PAUSE_MENU_ACTIVE) };
        let loading: bool = unsafe { invoke!(n::GET_IS_LOADING_SCREEN_ACTIVE) };
        let cutscene: bool = unsafe { invoke!(n::IS_CUTSCENE_PLAYING) };
        let paused = pause_menu || loading || cutscene;
        game_time_us += (dt.max(0.0) as f64 * 1e6) as u64;

        // --- Dev keys ---
        if keys.pressed(0, VK_F6) {
            cap60 = !cap60;
            let _ = writeln!(log, "frame {frame}: 60 fps cap {}", if cap60 { "ON" } else { "OFF" });
        }
        if keys.pressed(11, VK_F3) && engage_req {
            engage_req = false;
            let _ = writeln!(log, "frame {frame}: cancel button");
        }
        if test_mode {
            tools::keep_unwanted();
        }
        if keys.pressed(1, VK_F7) {
            test_mode = true;
            // Test cars from an earlier test stay behind otherwise.
            tools::clear_lead(&mut lead);
            tools::teleport(ped, veh, spot, &mut log);
            spot = (spot + 1) % tools::spot_count().max(1);
            last_tick = wall_time_ns();
        }
        if keys.pressed(2, VK_F8) {
            test_mode = true;
            let model = tools::test_car_model();
            let _ = writeln!(log, "frame {frame}: test car {}", model.to_string_lossy());
            tools::spawn(ped, &model, &mut test_car, &mut log);
            last_tick = wall_time_ns();
        }
        if keys.pressed(3, VK_F9) && in_vehicle {
            ai = !ai;
            if ai {
                engage_req = false;
            }
            tools::ai_drive(ped, veh, ai);
            let _ = writeln!(log, "frame {frame}: GTA AI drive {}", if ai { "ON" } else { "OFF" });
        }
        if keys.pressed(4, VK_F10) && in_vehicle {
            engage_req = !engage_req;
            if engage_req && ai {
                ai = false;
                tools::ai_drive(ped, veh, false);
            }
            ramp.cancel();
            let _ = writeln!(log, "frame {frame}: engage request {}", if engage_req { "ON" } else { "OFF" });
        }
        let mut btn = 0u32;
        if keys.pressed(5, VK_PRIOR) {
            btn |= buttons::SPEED_UP;
        }
        if keys.pressed(6, VK_NEXT) {
            btn |= buttons::SPEED_DOWN;
        }
        if keys.pressed(7, VK_END) {
            btn |= buttons::GAP;
        }
        if keys.pressed(8, VK_F11) && in_vehicle {
            tools::spawn_lead(veh, 50.0, 12.0, &mut lead, &mut log);
            last_tick = wall_time_ns();
        }
        if keys.pressed(9, VK_F4) && in_vehicle {
            tools::spawn_lead(veh, 70.0, 0.0, &mut lead, &mut log);
            last_tick = wall_time_ns();
        }
        if keys.pressed(10, VK_F5) {
            tools::lead_brake(&lead, &mut log);
        }
        if !in_vehicle {
            engage_req = false;
        }

        // --- Driver input (from Windows, D5) ---
        // The core decides cancel and override from the raw inputs (D11); the
        // adapter only lets the driver's pedals win at once.
        let drv = if ai { input::Driver::default() } else { input::read() };
        let driver_override = drv.on_pedals();
        if drv.active() {
            ramp.cancel();
        }

        // --- New vehicle: publish who we are and what this car is ---
        if in_vehicle && (veh != info_for || info_frames == 720) {
            if veh != info_for {
                steering.reset();
                actors_sys.reset();
                info_frames = 0;
            }
            info_for = veh;
            let model: u32 = unsafe { invoke!(n::GET_ENTITY_MODEL, veh) };
            let (lo, hi) = {
                let (mut lo, mut hi) = (Vector3::default(), Vector3::default());
                let () = unsafe { invoke!(n::GET_MODEL_DIMENSIONS, model, &mut lo as *mut Vector3, &mut hi as *mut Vector3) };
                (lo, hi)
            };
            let name_ptr: *const core::ffi::c_char = unsafe { invoke!(n::GET_DISPLAY_NAME_FROM_VEHICLE_MODEL, model) };
            let name = if name_ptr.is_null() { "?".into() } else { unsafe { CStr::from_ptr(name_ptr) }.to_string_lossy().into_owned() };
            // GTA's entity frame is x right, y forward: ISO x = GTA y, ISO y = -GTA x.
            let (min_iso, max_iso) = ([lo.y, -hi.x, lo.z], [hi.y, -lo.x, hi.z]);
            info_seq += 1;
            let info = AdapterInfo {
                header: Header { kind: MsgKind::AdapterInfo, version: CONTRACT_VERSION, seq: info_seq, frame: frame as u64, game_time_us, wall_time_ns: wall_time_ns() },
                host: fixed_str(&format!("GTA V {}", edition.name)),
                vehicle: fixed_str(&name),
                capabilities: caps::STEER | caps::PEDALS | caps::HOLD | caps::WHEEL_SPEEDS | caps::ACTORS | caps::LINE_OF_SIGHT | caps::DRIVER_INPUT,
                _reserved: 0,
                box_min: min_iso,
                box_max: max_iso,
                max_steer_angle: steering.max_angle(),
                wheelbase: f32::NAN,
            };
            link.info.write(&info);
            let _ = writeln!(log, "frame {frame}: vehicle {name}: box x {:.2}..{:.2} y {:.2}..{:.2}, full lock {:.3} rad", min_iso[0], max_iso[0], min_iso[1], max_iso[1], info.max_steer_angle);
        }
        info_frames = info_frames.saturating_add(1);

        // --- Actors, then the sample (the core reads both after the wake-up) ---
        let (_, layout) = link.layout.read();
        let los = if in_vehicle && !paused {
            let st = actors_sys.step(veh, &layout, &mut actor_list);
            actor_list.header = Header { kind: MsgKind::ActorList, version: CONTRACT_VERSION, seq: seq.wrapping_add(1), frame: frame as u64, game_time_us, wall_time_ns: wall_time_ns() };
            link.actors.write(&actor_list);
            st
        } else {
            actors::LosStats::default()
        };

        let mut s = VehicleSample::default();
        let mut measured_angle = f32::NAN;
        if in_vehicle {
            let speed: f32 = unsafe { invoke!(n::GET_ENTITY_SPEED, veh) };
            let vel: Vector3 = unsafe { invoke!(n::GET_ENTITY_SPEED_VECTOR, veh, true) };
            let rot: Vector3 = unsafe { invoke!(n::GET_ENTITY_ROTATION_VELOCITY, veh) };
            let (vx, vy, _) = geom::iso_from_right_forward_up(vel.x, vel.y, vel.z);
            s.speed = speed;
            s.vx = vx;
            s.vy = vy;
            s.yaw_rate = rot.z;
            let r_in = win::read_f32(base + off.steer_input).unwrap_or(f32::NAN);
            measured_angle = win::read_f32(base + off.steer_angle).unwrap_or(f32::NAN);
            steering.observe(r_in, measured_angle);
            s.steer_angle = measured_angle;
            s.wheel_omega = wheel_omegas(base, &off);
            s.rev_ratio = unsafe { invoke!(n::GET_VEHICLE_CURRENT_REV_RATIO, veh) };
            s.gear = unsafe { invoke!(n::GET_VEHICLE_CURRENT_DRIVE_GEAR, veh) };
            s.driver_throttle = drv.throttle;
            s.driver_brake = drv.brake;
            s.driver_steer = drv.steer;
        }
        s.flags = if paused { sample_flags::PAUSED } else { 0 }
            | if engage_req { sample_flags::ENGAGE_REQUEST } else { 0 }
            | if drv.analog { sample_flags::INPUT_ANALOG } else { 0 };
        s.buttons = btn;
        seq = seq.wrapping_add(1);
        s.header = Header { kind: MsgKind::VehicleSample, version: CONTRACT_VERSION, seq, frame: frame as u64, game_time_us, wall_time_ns: wall_time_ns() };
        link.sample.write(&s);
        link.adapter_frame.store(frame as u64, Ordering::Release);
        link.signal_sample();
        let t_written = s.header.wall_time_ns;

        // --- Core health ---
        let core_pid = link.core_pid.load(Ordering::Acquire);
        let core_exited = watch.exited(core_pid);
        let epoch = link.epoch.load(Ordering::Acquire);
        if epoch != seen_epoch {
            if engage_req || was_active {
                let _ = writeln!(log, "frame {frame}: core epoch {seen_epoch} -> {epoch}: disengaged (a new core starts disengaged)");
            }
            seen_epoch = epoch;
            engage_req = false;
        }

        // Reactive mode: wait briefly for the answer to this very sample.
        let (_, mut cmd) = link.command.read();
        let core_present = core_pid != 0 && !core_exited && cmd.answers_seq != 0 && seq.wrapping_sub(cmd.answers_seq) < 120;
        let mut rtt_ns: Option<u64> = None;
        let t_wait = wall_time_ns();
        if core_present {
            loop {
                let (_, c) = link.command.read();
                if c.answers_seq == seq {
                    rtt_ns = Some(wall_time_ns() - t_written);
                    cmd = c;
                    break;
                }
                if wall_time_ns() - t_written > REPLY_WAIT_NS {
                    win_miss += 1;
                    cmd = c;
                    break;
                }
                core::hint::spin_loop();
            }
        }
        let t_waited = wall_time_ns();
        if let Some(r) = rtt_ns {
            win_rtt.push(r);
        }

        // --- Safe state and actuation (D24) ---
        let age = seq.wrapping_sub(cmd.answers_seq);
        let fresh = cmd.answers_seq != 0 && age <= WATCHDOG_FRAMES && !core_exited;
        let core_engaged = cmd.flags & command_flags::ENGAGED != 0;
        // The core disengaged (driver cancel, takeover answered, ...): the
        // engage request resets, as a stalk does.
        if engage_req && fresh && core_was_engaged && !core_engaged && !paused {
            engage_req = false;
            let _ = writeln!(log, "frame {frame}: core disengaged (brake {:.2}, gas {:.2}, steer {:.2})", drv.brake, drv.throttle, drv.steer);
        }
        if fresh {
            core_was_engaged = core_engaged && engage_req;
        }
        let active = in_vehicle && engage_req && core_engaged && fresh && !paused;
        // Hazard lights for a controlled stop (D11).
        let want_hazards = active && cmd.flags & command_flags::HAZARDS != 0;
        if want_hazards != hazards_on && in_vehicle {
            unsafe {
                let () = invoke!(n::SET_VEHICLE_INDICATOR_LIGHTS, veh, 0i32, want_hazards);
                let () = invoke!(n::SET_VEHICLE_INDICATOR_LIGHTS, veh, 1i32, want_hazards);
            }
            hazards_on = want_hazards;
            let _ = writeln!(log, "frame {frame}: hazards {}", if want_hazards { "ON (controlled stop)" } else { "off" });
        }
        // AEB acts whether or not anything is engaged, and over the accelerator.
        let aeb = in_vehicle && fresh && !paused && cmd.flags & command_flags::AEB != 0;
        if aeb && engage_req {
            engage_req = false;
            let _ = writeln!(log, "frame {frame}: AEB: ACC cancelled");
        }
        if aeb && !aeb_last {
            let _ = writeln!(log, "frame {frame}: AEB braking ({:.1} m/s, brake {:.2})", s.speed, cmd.brake);
        }
        if aeb_last && !fresh && in_vehicle && s.speed > 0.3 {
            // D24: the core went away while AEB was braking: brake to a stop.
            ramp.start_to_stop(last_brake);
            takeover_until = wall_time_ns() + 3_000_000_000;
            takeover_why = "core stopped during AEB";
            let _ = writeln!(log, "frame {frame}: core lost during AEB: braking to a stop");
        }
        aeb_last = aeb;
        let release = if engage_req && was_active && !fresh {
            Some(if core_exited { Release::CoreExited } else { Release::Stalled })
        } else {
            None
        };
        if let Some(why) = release {
            let _ = writeln!(
                log,
                "frame {frame}: RELEASED ({why:?}): last fresh command {age} frames ago; brake {last_brake:.2} fades out; TAKE OVER"
            );
            engage_req = false;
            takeover_until = wall_time_ns() + 3_000_000_000;
            takeover_why = if why == Release::CoreExited { "core stopped" } else { "core not responding" };
            if last_brake > 0.02 && !driver_override {
                ramp.start(last_brake);
            }
        }
        if active || aeb {
            if active && cmd.flags & command_flags::STEER != 0 {
                let t = wall_time_ns();
                steering.apply(base, &off, cmd.steer_angle, measured_angle);
                win_steer_ns += wall_time_ns() - t;
                win_steer_n += 1;
            } else {
                steering.reset();
            }
            let want_hold = cmd.flags & command_flags::HOLD != 0 && (!driver_override || aeb);
            if !driver_override || aeb {
                actuate::pedals(cmd.throttle, cmd.brake);
                last_brake = cmd.brake;
            }
            if want_hold != hold_on {
                actuate::hold(veh, want_hold);
                hold_on = want_hold;
            }
        } else {
            if was_active {
                steering.reset();
            }
            if hold_on && in_vehicle {
                actuate::hold(veh, false);
                hold_on = false;
            }
            if in_vehicle && !paused {
                ramp.step(dt, s.speed);
            }
            if !was_active || !ramp.active() {
                last_brake = 0.0;
            }
        }
        was_active = active || aeb;
        let t_end = wall_time_ns();
        let work_ns = (t_end - t_frame) - (t_waited - t_wait);
        win_work.push(work_ns);

        // --- Logs ---
        let state = if active {
            if driver_override { "override" } else { "active" }
        } else if engage_req {
            "requested"
        } else if ai {
            "ai"
        } else {
            "off"
        };
        let _ = writeln!(
            csv,
            "{frame},{:.3},{:.3},{:.1},{seq},{},{age},{state},{:.3},{},{:.3},{:.3},{:.4},{:.4},{:.2},{:.2},{:.2},{},{},{},{},{},{},{:.3},{:.3},{:.3},{:.3}",
            (t_frame - win_start) as f64 / 1e6,
            frame_ns as f64 / 1e6,
            work_ns as f64 / 1e3,
            rtt_ns.map(|r| format!("{:.1}", r as f64 / 1e3)).unwrap_or("-1".into()),
            s.speed,
            cmd.flags,
            if active { cmd.throttle } else { 0.0 },
            if active { cmd.brake } else { 0.0 },
            cmd.steer_angle,
            measured_angle,
            drv.throttle,
            drv.brake,
            drv.steer,
            actor_list.count,
            los.in_view,
            los.age_max,
            los.started,
            los.sync,
            los.failed,
            cmd.diag[0],
            cmd.diag[1],
            cmd.diag[2],
            cmd.diag[3]
        );
        win_frames += 1;
        if t_frame - win_start >= 1_000_000_000 {
            let secs = (t_frame - win_start) as f64 / 1e9;
            win_rtt.sort_unstable();
            win_work.sort_unstable();
            let rtt = if win_rtt.is_empty() {
                "no core".to_string()
            } else {
                format!("rtt p50 {:.0} p99 {:.0} us, {win_miss} misses", pct(&win_rtt, 0.5) as f64 / 1e3, pct(&win_rtt, 0.99) as f64 / 1e3)
            };
            let work = format!(
                "adapter p50 {:.0} p99 {:.0} max {:.0} us",
                pct(&win_work, 0.5) as f64 / 1e3,
                pct(&win_work, 0.99) as f64 / 1e3,
                win_work.last().copied().unwrap_or(0) as f64 / 1e3
            );
            let _ = writeln!(
                log,
                "[frame {frame}] {:.0} fps{} | {rtt} | {work} | actors {} (in view {}, LOS age max {}) | {state}",
                win_frames as f64 / secs,
                if cap60 { " (cap 60)" } else { "" },
                actor_list.count,
                los.in_view,
                los.age_max
            );
            if win_steer_n > 0 {
                let w = win::take_write_stats();
                let _ = writeln!(
                    log,
                    "[frame {frame}] steering: {win_steer_n} applies, {:.1} us each | write_f32: {} writes, {} region checks, check {:.1} us, write {:.1} us each",
                    win_steer_ns as f64 / 1e3 / win_steer_n as f64,
                    w[3],
                    w[2],
                    w[0] as f64 / 1e3 / w[3].max(1) as f64,
                    w[1] as f64 / 1e3 / w[3].max(1) as f64
                );
                win_steer_ns = 0;
                win_steer_n = 0;
            }
            hud2 = format!("{rtt} | {work} | actors {} in view {}", actor_list.count, los.in_view);
            win_start = t_frame;
            win_frames = 0;
            win_rtt.clear();
            win_work.clear();
            win_miss = 0;
            let _ = log.flush();
            let _ = csv.flush();
        }

        // --- HUD (what the core says the driver should see, D8) ---
        let (_, hud) = link.hud.read();
        let hud_live = fresh && hud.header.frame + 6 >= frame as u64;
        let fcw = hud_live && hud.flags & (hud_flags::FCW | hud_flags::AEB) != 0;
        if fcw && !fcw_last {
            let () = unsafe { invoke!(n::PLAY_SOUND_FRONTEND, -1i32, c"TIMER_STOP", c"HUD_MINI_GAME_SOUNDSET", true) };
        }
        fcw_last = fcw;
        // D11: a takeover request shows at once, chimes once escalated.
        let takeover = hud_live && hud.flags & hud_flags::TAKEOVER != 0;
        if takeover && hud.flags & hud_flags::TAKEOVER_CHIME != 0 && wall_time_ns().saturating_sub(last_chime) > 1_000_000_000 {
            let () = unsafe { invoke!(n::PLAY_SOUND_FRONTEND, -1i32, c"5_SEC_WARNING", c"HUD_MINI_GAME_SOUNDSET", true) };
            last_chime = wall_time_ns();
        }
        unsafe {
            let () = invoke!(n::DRAW_RECT, 0.25f32, 0.065f32, 0.48f32, 0.11f32, 0i32, 0i32, 0i32, 160i32, false);
            let (label, colour) = if wall_time_ns() < takeover_until {
                (format!("TAKE OVER ({takeover_why})"), (255, 80, 60))
            } else if active && driver_override {
                ("OVERRIDE".to_string(), (255, 200, 60))
            } else if active {
                ("ACTIVE".to_string(), (80, 220, 120))
            } else if engage_req {
                ("waiting for core".to_string(), (200, 200, 200))
            } else if ai {
                ("GTA AI driving".to_string(), (160, 160, 255))
            } else {
                ("off".to_string(), (200, 200, 200))
            };
            draw_text(0.015, 0.012, &format!("ADAS | {:.0} km/h | {label}", s.speed * 3.6), colour);
            draw_text(0.015, 0.037, &hud2, (255, 255, 255));
            let warn = if edition.verified { "" } else { " | UNVERIFIED GAME BUILD" };
            draw_text(
                0.015,
                0.062,
                &format!(
                    "F10 ACC | PgUp/PgDn speed | End gap | F11 lead | F5 lead brakes | F4 stopped car | F7 tp | F8 car | F9 AI | F6 cap60 {}{warn}",
                    if cap60 { "ON" } else { "off" }
                ),
                (255, 255, 255),
            );
            // The driver display: ACC state, set speed, gap bars, lead.
            if hud_live {
                let acc = hud.flags & hud_flags::ACC != 0;
                let bars = "|".repeat(hud.gap_index as usize + 1);
                let lead_txt = if hud.flags & hud_flags::LEAD != 0 {
                    format!("lead {:.0} m @ {:.0} km/h", hud.lead_gap, hud.lead_speed * 3.6)
                } else {
                    "no lead".to_string()
                };
                let hold = if hud.flags & hud_flags::HOLD != 0 { " | HOLD" } else { "" };
                let aeb_off = if hud.flags & hud_flags::AEB_OFF != 0 { " | AEB OFF" } else { "" };
                let colour = if acc { (80, 220, 120) } else if hud.flags & hud_flags::UNAVAILABLE != 0 { (255, 170, 60) } else { (180, 180, 180) };
                let () = invoke!(n::DRAW_RECT, 0.5f32, 0.925f32, 0.34f32, 0.045f32, 0i32, 0i32, 0i32, 170i32, false);
                let line = if hud.flags & hud_flags::UNAVAILABLE != 0 {
                    format!("ACC unavailable: {}{aeb_off}", adapter_api::hud_message::text(hud.message))
                } else {
                    format!("{} {:.0} km/h | gap {bars} {:.1} s | {lead_txt}{hold}{aeb_off}", if acc { "ACC" } else { "ACC off" }, hud.set_speed * 3.6, hud.time_gap)
                };
                draw_text(0.34, 0.91, &line, colour);
            }
            if takeover && !fcw {
                let stop = hud.flags & hud_flags::CONTROLLED_STOP != 0;
                // Steady at first, flashing once it chimes.
                if hud.flags & hud_flags::TAKEOVER_CHIME == 0 || (frame / 9) % 2 == 0 {
                    let () = invoke!(n::DRAW_RECT, 0.5f32, 0.3f32, 0.42f32, 0.09f32, 200i32, 60i32, 20i32, 210i32, false);
                }
                let why = adapter_api::hud_message::text(hud.message);
                let txt = if stop { format!("CONTROLLED STOP - TAKE OVER ({why})") } else { format!("TAKE OVER: {why}") };
                draw_text(0.31, 0.283, &txt, (255, 255, 255));
            }
            if fcw {
                if (frame / 6) % 2 == 0 {
                    let () = invoke!(n::DRAW_RECT, 0.5f32, 0.4f32, 0.3f32, 0.08f32, 200i32, 20i32, 20i32, 200i32, false);
                }
                let txt = if hud.flags & hud_flags::AEB != 0 { "EMERGENCY BRAKING" } else { "BRAKE!" };
                draw_text(0.42, 0.385, txt, (255, 255, 255));
            }
        }

        unsafe { shv_sys::scriptWait(0) };
    }
}
