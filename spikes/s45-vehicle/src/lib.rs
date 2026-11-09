//! Spikes S4/S5: find the steering angle in vehicle memory, test pedal
//! injection through blocked controls, and test whether a written steering
//! value holds. Throwaway code.
//!
//! Keys (sit in a car, stationary for F9):
//! - F8  spawn a test car (Sultan) and put the player in it; player invincible.
//! - F9  steering sweep: blocks the player's steering, injects a sine steering
//!   input for 6 s, snapshots 12 KB of the vehicle struct every frame, then
//!   ranks every float field by correlation with the input (background thread).
//! - F10 pedal test: 3 s injected throttle, 3 s injected brake, 3 s blocked
//!   with nothing injected. Logs what the car and the blocked controls report.
//! - F11 write-hold test: writes 0.5 into the best steering candidate every
//!   frame for 3 s and logs what is read back on the next frame.
//!
//! Output: `<game folder>/adas-logs/s45-<unix time>-*.{log,csv,txt}`.

use core::ffi::{CStr, c_void};
use std::ffi::CString;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::panic::{AssertUnwindSafe, catch_unwind};
use std::path::PathBuf;
use std::sync::atomic::{AtomicPtr, Ordering};
use std::sync::{Arc, Mutex};

use shv_sys::invoke;

// ---- Win32 ----

#[link(name = "kernel32", kind = "raw-dylib")]
unsafe extern "system" {
    fn DisableThreadLibraryCalls(module: *mut c_void) -> i32;
    fn GetModuleFileNameW(module: *mut c_void, buf: *mut u16, len: u32) -> u32;
    fn GetCurrentProcess() -> *mut c_void;
    fn ReadProcessMemory(process: *mut c_void, addr: *const c_void, buf: *mut c_void, size: usize, read: *mut usize) -> i32;
    fn WriteProcessMemory(process: *mut c_void, addr: *mut c_void, buf: *const c_void, size: usize, written: *mut usize) -> i32;
}

#[link(name = "user32", kind = "raw-dylib")]
unsafe extern "system" {
    fn GetAsyncKeyState(vkey: i32) -> i16;
}

const VK_F8: i32 = 0x77;
const VK_F9: i32 = 0x78;
const VK_F10: i32 = 0x79;
const VK_F11: i32 = 0x7A;

// ---- Natives (hashes checked against alloc8or/gta5-nativedb-data) ----

mod n {
    pub const PLAYER_PED_ID: u64 = 0xD80958FC74E988A6;
    pub const IS_PED_IN_ANY_VEHICLE: u64 = 0x997ABD671D25CA0B;
    pub const GET_VEHICLE_PED_IS_IN: u64 = 0x9A9112A0FE9A4713;
    pub const GET_ENTITY_SPEED: u64 = 0xD5037BA82E12416F;
    pub const GET_ENTITY_MODEL: u64 = 0x9F47B058362C84B5;
    pub const GET_DISPLAY_NAME_FROM_VEHICLE_MODEL: u64 = 0xB215AAC32D25D019;
    pub const GET_FRAME_TIME: u64 = 0x15C40837039FFAF7;
    pub const GET_FRAME_COUNT: u64 = 0xFC8202EFC642E6F2;
    pub const VEHICLE_THROTTLE: u64 = 0x92D96892FC06AF22; // _GET_VEHICLE_THROTTLE
    pub const DISABLE_CONTROL_ACTION: u64 = 0xFE99B66D079CF6BC;
    pub const SET_CONTROL_VALUE_NEXT_FRAME: u64 = 0xE8A25867FBA3B05E;
    pub const GET_DISABLED_CONTROL_NORMAL: u64 = 0x11E65974A982637C;
    pub const GET_CONTROL_NORMAL: u64 = 0xEC3C9B8D5327B563;
    pub const IS_DISABLED_CONTROL_PRESSED: u64 = 0xE2587F8CBBD87B1D;
    pub const BEGIN_TEXT_COMMAND_DISPLAY_TEXT: u64 = 0x25FBB336DF1804CB;
    pub const ADD_TEXT_COMPONENT_SUBSTRING_PLAYER_NAME: u64 = 0x6C188BE134E074AA;
    pub const END_TEXT_COMMAND_DISPLAY_TEXT: u64 = 0xCD015E5BB0D96A57;
    pub const SET_TEXT_SCALE: u64 = 0x07C837F9A01C34C9;
    pub const SET_TEXT_FONT: u64 = 0x66E0276CC5F6B9DA;
    pub const SET_TEXT_COLOUR: u64 = 0xBE6B23FFA53FB442;
    pub const SET_TEXT_OUTLINE: u64 = 0x2513DFB0FB8400FE;
    pub const DRAW_RECT: u64 = 0x3A618A217E5154F0;
    pub const GET_ENTITY_COORDS: u64 = 0x3FEF770D40960D5A;
    pub const GET_ENTITY_FORWARD_VECTOR: u64 = 0x0A794A5A57F8DF91;
    pub const GET_ENTITY_HEADING: u64 = 0xE83D4F9BA2A38914;
    pub const GET_HASH_KEY: u64 = 0xD24D37CC275948CC;
    pub const REQUEST_MODEL: u64 = 0x963D27A58DF860AC;
    pub const HAS_MODEL_LOADED: u64 = 0x98A4EB5D89A0C952;
    pub const SET_MODEL_AS_NO_LONGER_NEEDED: u64 = 0xE532F5D78798DAAB;
    pub const CREATE_VEHICLE: u64 = 0xAF35D0D2583051B0;
    pub const SET_VEHICLE_ON_GROUND_PROPERLY: u64 = 0x49733E92263139D1;
    pub const SET_PED_INTO_VEHICLE: u64 = 0xF75B0D629E1C063D;
    pub const SET_ENTITY_INVINCIBLE: u64 = 0x3882114BDE571AD4;
    pub const GET_ENTITY_ROTATION_VELOCITY: u64 = 0x213B91045D09B983;
}

/// Steering fields found by the F9 sweep on build 1158.13 (Sultan):
/// 0x9D4 = smoothed steering input (±0.95), 0x9DC = steering angle in rad (±0.58).
const STEER_INPUT_OFFSET: usize = 0x9D4;
const STEER_ANGLE_OFFSET: usize = 0x9DC;
/// Injected control values have a 25% deadzone: throttle = (v - 0.25) / 0.75.
/// 0.45 gives about 0.27 throttle, enough to creep.
const HOLD_CREEP_THROTTLE: f32 = 0.45;
/// Phase 3 steers through the control instead, for comparison.
const HOLD_STEER_CONTROL: f32 = 0.5;

/// Test tooling: spawns a car 3 m ahead and puts the player in the driver's
/// seat; also makes the player invincible so fights or crashes don't end a test.
/// Yields the fiber while the model streams in.
fn spawn_test_car(ped: i32, log: &mut impl Write) {
    unsafe {
        let model: u32 = invoke!(n::GET_HASH_KEY, c"sultan");
        let () = invoke!(n::REQUEST_MODEL, model);
        let mut waited = 0;
        loop {
            let loaded: bool = invoke!(n::HAS_MODEL_LOADED, model);
            if loaded {
                break;
            }
            if waited > 300 {
                let _ = writeln!(log, "SPAWN failed: model did not load");
                return;
            }
            shv_sys::scriptWait(0);
            waited += 1;
        }
        let pos: shv_sys::Vector3 = invoke!(n::GET_ENTITY_COORDS, ped, true);
        let fwd: shv_sys::Vector3 = invoke!(n::GET_ENTITY_FORWARD_VECTOR, ped);
        let heading: f32 = invoke!(n::GET_ENTITY_HEADING, ped);
        let veh: i32 = invoke!(n::CREATE_VEHICLE, model, pos.x + fwd.x * 3.0, pos.y + fwd.y * 3.0, pos.z, heading, false, false, false);
        let _: bool = invoke!(n::SET_VEHICLE_ON_GROUND_PROPERLY, veh, 5.0f32);
        let () = invoke!(n::SET_PED_INTO_VEHICLE, ped, veh, -1i32);
        let () = invoke!(n::SET_ENTITY_INVINCIBLE, ped, true, false);
        let () = invoke!(n::SET_MODEL_AS_NO_LONGER_NEEDED, model);
        let _ = writeln!(log, "SPAWN sultan handle {veh} after {waited} frames; player invincible");
    }
}

/// Control group and actions.
const PLAYER_CONTROL: i32 = 0;
const INPUT_VEH_MOVE_LR: i32 = 59;
const INPUT_VEH_ACCELERATE: i32 = 71;
const INPUT_VEH_BRAKE: i32 = 72;

const SNAP_BYTES: usize = 0x3000;
/// Sweep phases, 3 s each: A = steering blocked + injected, B = injected only,
/// C = nothing injected (steer with the real keys).
const SWEEP_PHASE_S: f64 = 3.0;
const SWEEP_SECONDS: f64 = 3.0 * SWEEP_PHASE_S;
const SWEEP_PERIOD_S: f64 = 2.0;
const MAX_LAG_FRAMES: usize = 20;
const PEDAL_PHASE_S: f64 = 3.0;
/// Write-hold phases, 3 s each: 0 = write the angle field, 1 = write the input
/// field, 2 = no write (baseline), 3 = steer through the injected control.
/// The car creeps on injected throttle throughout.
const HOLD_PHASE_S: f64 = 3.0;
const HOLD_VALUE: f32 = 0.3;

// ---- DLL entry ----

static MODULE: AtomicPtr<c_void> = AtomicPtr::new(core::ptr::null_mut());

/// # Safety
/// Called only by the Windows loader.
#[unsafe(no_mangle)]
pub unsafe extern "system" fn DllMain(module: *mut c_void, reason: u32, _reserved: *mut c_void) -> i32 {
    match reason {
        1 => {
            MODULE.store(module, Ordering::Relaxed);
            unsafe {
                DisableThreadLibraryCalls(module);
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

// ---- Helpers ----

/// `<game folder>/adas-logs`. GTA locks files that already exist in its folder
/// at startup, so every run writes new, uniquely named files here.
fn log_dir() -> PathBuf {
    let mut buf = [0u16; 1024];
    let len = unsafe { GetModuleFileNameW(MODULE.load(Ordering::Relaxed), buf.as_mut_ptr(), buf.len() as u32) };
    let path = PathBuf::from(String::from_utf16_lossy(&buf[..len as usize]));
    let dir = path.parent().map(|p| p.join("adas-logs")).unwrap_or_else(|| PathBuf::from("adas-logs"));
    let _ = std::fs::create_dir_all(&dir);
    dir
}

fn run_stem() -> String {
    let secs = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);
    format!("s45-{secs}")
}

struct Keys {
    prev: [bool; 4],
}

impl Keys {
    fn pressed(&mut self, slot: usize, vk: i32) -> bool {
        let down = unsafe { GetAsyncKeyState(vk) } as u16 & 0x8000 != 0;
        let edge = down && !self.prev[slot];
        self.prev[slot] = down;
        edge
    }
}

unsafe fn draw_text(x: f32, y: f32, text: &str) {
    let s = CString::new(text).unwrap_or_default();
    unsafe {
        let () = invoke!(n::SET_TEXT_FONT, 0i32);
        let () = invoke!(n::SET_TEXT_SCALE, 0.0f32, 0.32f32);
        let () = invoke!(n::SET_TEXT_COLOUR, 255i32, 255i32, 255i32, 255i32);
        let () = invoke!(n::SET_TEXT_OUTLINE);
        let () = invoke!(n::BEGIN_TEXT_COMMAND_DISPLAY_TEXT, c"STRING");
        let () = invoke!(n::ADD_TEXT_COMPONENT_SUBSTRING_PLAYER_NAME, s.as_c_str());
        let () = invoke!(n::END_TEXT_COMMAND_DISPLAY_TEXT, x, y, 0i32);
    }
}

/// Reads `out.len()` bytes at `addr` without faulting (fails cleanly instead).
fn read_mem(addr: usize, out: &mut [u8]) -> bool {
    let mut read = 0usize;
    let ok = unsafe { ReadProcessMemory(GetCurrentProcess(), addr as *const c_void, out.as_mut_ptr() as *mut c_void, out.len(), &mut read) };
    ok != 0 && read == out.len()
}

fn read_f32(addr: usize) -> Option<f32> {
    let mut b = [0u8; 4];
    read_mem(addr, &mut b).then(|| f32::from_le_bytes(b))
}

fn write_f32(addr: usize, v: f32) -> bool {
    let b = v.to_le_bytes();
    let mut written = 0usize;
    let ok = unsafe { WriteProcessMemory(GetCurrentProcess(), addr as *mut c_void, b.as_ptr() as *const c_void, 4, &mut written) };
    ok != 0 && written == 4
}

fn pearson(x: &[f32], y: &[f32]) -> f32 {
    let n = x.len().min(y.len());
    if n < 3 {
        return 0.0;
    }
    let (mut sx, mut sy) = (0f64, 0f64);
    for i in 0..n {
        sx += x[i] as f64;
        sy += y[i] as f64;
    }
    let (mx, my) = (sx / n as f64, sy / n as f64);
    let (mut sxy, mut sxx, mut syy) = (0f64, 0f64, 0f64);
    for i in 0..n {
        let (dx, dy) = (x[i] as f64 - mx, y[i] as f64 - my);
        sxy += dx * dy;
        sxx += dx * dx;
        syy += dy * dy;
    }
    if sxx < 1e-12 || syy < 1e-12 {
        return 0.0;
    }
    (sxy / (sxx * syy).sqrt()) as f32
}

struct Candidate {
    offset: usize,
    r: f32,
    lag: usize,
    min: f32,
    max: f32,
}

/// Ranks every 4-byte float field of the snapshots by correlation with the
/// input. Lag > 0 means the field follows the input that many frames later.
fn analyse_sweep(input: &[f32], snaps: &[&[u8]]) -> Vec<Candidate> {
    let frames = snaps.len().min(input.len());
    let mut out = Vec::new();
    for offset in (0..SNAP_BYTES).step_by(4) {
        let series: Vec<f32> = snaps[..frames]
            .iter()
            .map(|s| f32::from_le_bytes([s[offset], s[offset + 1], s[offset + 2], s[offset + 3]]))
            .collect();
        if series.iter().any(|v| !v.is_finite() || v.abs() > 1.0e4) {
            continue;
        }
        let (min, max) = series.iter().fold((f32::MAX, f32::MIN), |(a, b), &v| (a.min(v), b.max(v)));
        if max - min < 1e-3 {
            continue;
        }
        let mut best = (0f32, 0usize);
        for lag in 0..=MAX_LAG_FRAMES.min(frames / 4) {
            let r = pearson(&series[lag..], &input[..frames - lag]);
            if r.abs() > best.0.abs() {
                best = (r, lag);
            }
        }
        if best.0.abs() >= 0.8 {
            out.push(Candidate { offset, r: best.0, lag: best.1, min, max });
        }
    }
    out.sort_by(|a, b| b.r.abs().total_cmp(&a.r.abs()));
    out
}

/// Writes the per-frame inputs, then, for each phase and each reference signal
/// that actually varies, the fields that track it. Returns the first plausible
/// steering-angle offset found.
fn analyse_and_report(frames: &[SweepFrame], report: &std::path::Path, csv: &std::path::Path) -> Option<usize> {
    if let Ok(f) = File::create(csv) {
        let mut w = BufWriter::new(f);
        let _ = writeln!(w, "phase,injected,control_normal,disabled_normal");
        for fr in frames {
            let _ = writeln!(w, "{},{:.4},{:.4},{:.4}", fr.phase, fr.injected, fr.control_normal, fr.disabled_normal);
        }
    }
    let Ok(f) = File::create(report) else { return None };
    let mut w = BufWriter::new(f);
    let mut pick = None;
    let names = ["A blocked+injected", "B injected only", "C real keys"];
    for phase in 0..3u8 {
        let sel: Vec<&SweepFrame> = frames.iter().filter(|f| f.phase == phase).collect();
        let snaps: Vec<&[u8]> = sel.iter().map(|f| f.snap.as_slice()).collect();
        let refs: [(&str, Vec<f32>); 3] = [
            ("injected", sel.iter().map(|f| f.injected).collect()),
            ("control_normal", sel.iter().map(|f| f.control_normal).collect()),
            ("disabled_normal", sel.iter().map(|f| f.disabled_normal).collect()),
        ];
        let _ = writeln!(w, "== phase {} ({} frames)", names[phase as usize], sel.len());
        for (name, r) in &refs {
            let (lo, hi) = r.iter().fold((f32::MAX, f32::MIN), |(a, b), &v| (a.min(v), b.max(v)));
            if sel.len() < 10 || hi - lo < 0.05 {
                let _ = writeln!(w, "-- vs {name}: reference flat ({lo:+.3}..{hi:+.3}), skipped");
                continue;
            }
            let cands = analyse_sweep(r, &snaps);
            let _ = writeln!(w, "-- vs {name} (range {lo:+.3}..{hi:+.3}): {} fields with |r| >= 0.8", cands.len());
            let _ = writeln!(w, "   offset   r       lag  min       max");
            for c in cands.iter().take(15) {
                let _ = writeln!(w, "   0x{:04X}  {:+.4}  {:>3}  {:+.4}  {:+.4}", c.offset, c.r, c.lag, c.min, c.max);
            }
            if pick.is_none() {
                pick = pick_angle(&cands);
            }
        }
    }
    let _ = writeln!(w, "picked steering-angle candidate: {}", pick.map(|o| format!("0x{o:X}")).unwrap_or("none".into()));
    pick
}

/// Best steering-angle candidate: strongly correlated, and an angle-like range
/// (not the raw ±1 input copy).
fn pick_angle(c: &[Candidate]) -> Option<usize> {
    c.iter()
        .filter(|c| c.r.abs() > 0.95)
        .find(|c| {
            let amp = c.max.abs().max(c.min.abs());
            (0.15..=1.2).contains(&amp) && (amp - 1.0).abs() > 0.02
        })
        .map(|c| c.offset)
}

/// One sweep frame: the snapshot and the inputs that produced it.
struct SweepFrame {
    phase: u8,
    injected: f32,
    control_normal: f32,
    disabled_normal: f32,
    snap: Vec<u8>,
}

enum Mode {
    Idle,
    Sweep { t: f64, frames: Vec<SweepFrame>, prev: Option<(u8, f32, f32, f32)>, base: usize },
    Pedals { t: f64, csv: BufWriter<File> },
    Hold { t: f64, base: usize, csv: BufWriter<File> },
}

// ---- The spike ----

fn run() {
    let dir = log_dir();
    let stem = run_stem();
    let mut log = BufWriter::new(File::create(dir.join(format!("{stem}.log"))).expect("create log"));
    let _ = writeln!(log, "S4/S5 spike started. F9 sweep, F10 pedals, F11 write-hold");
    let _ = log.flush();

    let mut keys = Keys { prev: [false; 4] };
    let mut mode = Mode::Idle;
    let status: Arc<Mutex<String>> = Arc::new(Mutex::new("idle".into()));
    let angle_offset: Arc<Mutex<Option<usize>>> = Arc::new(Mutex::new(None));
    let mut run_no = 0u32;

    loop {
        let ped: i32 = unsafe { invoke!(n::PLAYER_PED_ID) };
        let in_vehicle: bool = unsafe { invoke!(n::IS_PED_IN_ANY_VEHICLE, ped, false) };
        let veh: i32 = if in_vehicle { unsafe { invoke!(n::GET_VEHICLE_PED_IS_IN, ped, false) } } else { 0 };
        let dt: f32 = unsafe { invoke!(n::GET_FRAME_TIME) };
        let frame: i32 = unsafe { invoke!(n::GET_FRAME_COUNT) };
        let speed: f32 = if in_vehicle { unsafe { invoke!(n::GET_ENTITY_SPEED, veh) } } else { 0.0 };
        let base = if in_vehicle { unsafe { shv_sys::getScriptHandleBaseAddress(veh) as usize } } else { 0 };

        if keys.pressed(3, VK_F8) && matches!(mode, Mode::Idle) {
            spawn_test_car(ped, &mut log);
        }
        let f9 = keys.pressed(0, VK_F9);
        let f10 = keys.pressed(1, VK_F10);
        let f11 = keys.pressed(2, VK_F11);

        if matches!(mode, Mode::Idle) && in_vehicle && base != 0 {
            if f9 {
                let model: u32 = unsafe { invoke!(n::GET_ENTITY_MODEL, veh) };
                let name_ptr: *const core::ffi::c_char = unsafe { invoke!(n::GET_DISPLAY_NAME_FROM_VEHICLE_MODEL, model) };
                let name = if name_ptr.is_null() { "?".into() } else { unsafe { CStr::from_ptr(name_ptr) }.to_string_lossy().into_owned() };
                let _ = writeln!(log, "frame {frame}: SWEEP start, vehicle {name} (model 0x{model:08X}), base 0x{base:X}, speed {speed:.2} m/s");
                *status.lock().unwrap() = format!("sweeping ({name})");
                mode = Mode::Sweep { t: 0.0, frames: Vec::new(), prev: None, base };
            } else if f10 {
                run_no += 1;
                let f = File::create(dir.join(format!("{stem}-pedals-{run_no}.csv"))).expect("create pedals csv");
                let mut csv = BufWriter::new(f);
                let _ = writeln!(
                    csv,
                    "frame,t_s,phase,inj_throttle,inj_brake,speed_mps,veh_throttle,disabled_norm_acc,control_norm_acc,disabled_pressed_acc,disabled_norm_brake"
                );
                let _ = writeln!(log, "frame {frame}: PEDALS start (file pedals-{run_no})");
                *status.lock().unwrap() = "pedal test".into();
                mode = Mode::Pedals { t: 0.0, csv };
            } else if f11 {
                run_no += 1;
                let f = File::create(dir.join(format!("{stem}-hold-{run_no}.csv"))).expect("create hold csv");
                let mut csv = BufWriter::new(f);
                let _ = writeln!(csv, "frame,t_s,phase,read_input_9D4,read_angle_9DC,wrote,yaw_rate,speed_mps");
                let _ = writeln!(log, "frame {frame}: HOLD start (file hold-{run_no})");
                *status.lock().unwrap() = "write-hold test".into();
                mode = Mode::Hold { t: 0.0, base, csv };
            }
        }

        mode = match mode {
            Mode::Idle => Mode::Idle,
            Mode::Sweep { mut t, mut frames, prev, base: sweep_base } => {
                if !in_vehicle || base != sweep_base {
                    let _ = writeln!(log, "frame {frame}: SWEEP aborted (left vehicle)");
                    *status.lock().unwrap() = "sweep aborted".into();
                    Mode::Idle
                } else {
                    // Snapshot first: memory now reflects last tick's inputs.
                    let mut snap = vec![0u8; SNAP_BYTES];
                    if let Some((phase, injected, control_normal, disabled_normal)) = prev
                        && read_mem(base, &mut snap)
                    {
                        frames.push(SweepFrame { phase, injected, control_normal, disabled_normal, snap });
                    }
                    let phase = (t / SWEEP_PHASE_S) as u8;
                    let u = if phase < 2 { input_at(t) } else { 0.0 };
                    unsafe {
                        if phase == 0 {
                            let () = invoke!(n::DISABLE_CONTROL_ACTION, PLAYER_CONTROL, INPUT_VEH_MOVE_LR, true);
                        }
                        if phase < 2 {
                            let _: bool = invoke!(n::SET_CONTROL_VALUE_NEXT_FRAME, PLAYER_CONTROL, INPUT_VEH_MOVE_LR, u);
                        }
                    }
                    let ctl: f32 = unsafe { invoke!(n::GET_CONTROL_NORMAL, PLAYER_CONTROL, INPUT_VEH_MOVE_LR) };
                    let dis: f32 = unsafe { invoke!(n::GET_DISABLED_CONTROL_NORMAL, PLAYER_CONTROL, INPUT_VEH_MOVE_LR) };
                    let prev = Some((phase, u, ctl, dis));
                    t += dt as f64;
                    if t >= SWEEP_SECONDS {
                        let _ = writeln!(log, "frame {frame}: SWEEP done, {} snapshots; analysing in background", frames.len());
                        *status.lock().unwrap() = "analysing...".into();
                        let (st, ao) = (status.clone(), angle_offset.clone());
                        let (report, csv_path) = (dir.join(format!("{stem}-steer.txt")), dir.join(format!("{stem}-steer-inputs.csv")));
                        std::thread::spawn(move || {
                            let pick = analyse_and_report(&frames, &report, &csv_path);
                            *ao.lock().unwrap() = pick;
                            *st.lock().unwrap() = format!("sweep done; angle {}", pick.map(|o| format!("0x{o:X}")).unwrap_or("none".into()));
                        });
                        Mode::Idle
                    } else {
                        Mode::Sweep { t, frames, prev, base: sweep_base }
                    }
                }
            }
            Mode::Pedals { mut t, mut csv } => {
                // 0: unblocked throttle ramp 0→1 (injection scale)   1: unblocked brake 1.0
                // 2: blocked + throttle 0.6 (repeat)                   3: blocked, nothing (press W: driver read)
                // 4: unblocked, nothing (press W: how a real key ramps the throttle)
                let phase = (t / PEDAL_PHASE_S) as u32;
                let within = (t - phase as f64 * PEDAL_PHASE_S) / PEDAL_PHASE_S;
                let (thr, brk, blocked) = match phase {
                    0 => (within as f32, 0.0f32, false),
                    1 => (0.0, 1.0, false),
                    2 => (0.6, 0.0, true),
                    3 => (0.0, 0.0, true),
                    _ => (0.0, 0.0, false),
                };
                unsafe {
                    if blocked {
                        let () = invoke!(n::DISABLE_CONTROL_ACTION, PLAYER_CONTROL, INPUT_VEH_ACCELERATE, true);
                        let () = invoke!(n::DISABLE_CONTROL_ACTION, PLAYER_CONTROL, INPUT_VEH_BRAKE, true);
                    }
                    if thr > 0.0 {
                        let _: bool = invoke!(n::SET_CONTROL_VALUE_NEXT_FRAME, PLAYER_CONTROL, INPUT_VEH_ACCELERATE, thr);
                    }
                    if brk > 0.0 {
                        let _: bool = invoke!(n::SET_CONTROL_VALUE_NEXT_FRAME, PLAYER_CONTROL, INPUT_VEH_BRAKE, brk);
                    }
                }
                let veh_thr: f32 = if in_vehicle { unsafe { invoke!(n::VEHICLE_THROTTLE, veh) } } else { 0.0 };
                let dis_acc: f32 = unsafe { invoke!(n::GET_DISABLED_CONTROL_NORMAL, PLAYER_CONTROL, INPUT_VEH_ACCELERATE) };
                let ctl_acc: f32 = unsafe { invoke!(n::GET_CONTROL_NORMAL, PLAYER_CONTROL, INPUT_VEH_ACCELERATE) };
                let dis_pr: bool = unsafe { invoke!(n::IS_DISABLED_CONTROL_PRESSED, PLAYER_CONTROL, INPUT_VEH_ACCELERATE) };
                let dis_brk: f32 = unsafe { invoke!(n::GET_DISABLED_CONTROL_NORMAL, PLAYER_CONTROL, INPUT_VEH_BRAKE) };
                let _ = writeln!(
                    csv,
                    "{frame},{t:.3},{phase},{thr:.3},{brk},{speed:.3},{veh_thr:.3},{dis_acc:.3},{ctl_acc:.3},{},{dis_brk:.3}",
                    dis_pr as u8
                );
                t += dt as f64;
                if t >= 5.0 * PEDAL_PHASE_S {
                    let _ = csv.flush();
                    let _ = writeln!(log, "frame {frame}: PEDALS done");
                    *status.lock().unwrap() = "pedal test done".into();
                    Mode::Idle
                } else {
                    Mode::Pedals { t, csv }
                }
            }
            Mode::Hold { mut t, base: hold_base, mut csv } => {
                if !in_vehicle || base != hold_base {
                    let _ = writeln!(log, "frame {frame}: HOLD aborted (left vehicle)");
                    Mode::Idle
                } else {
                    let phase = (t / HOLD_PHASE_S) as u32;
                    // Values the game left after last frame, before we write.
                    let r_in = read_f32(base + STEER_INPUT_OFFSET).unwrap_or(f32::NAN);
                    let r_ang = read_f32(base + STEER_ANGLE_OFFSET).unwrap_or(f32::NAN);
                    let wrote = match phase {
                        0 => write_f32(base + STEER_ANGLE_OFFSET, HOLD_VALUE).then_some(STEER_ANGLE_OFFSET),
                        1 => write_f32(base + STEER_INPUT_OFFSET, HOLD_VALUE).then_some(STEER_INPUT_OFFSET),
                        _ => None,
                    };
                    let _: bool = unsafe { invoke!(n::SET_CONTROL_VALUE_NEXT_FRAME, PLAYER_CONTROL, INPUT_VEH_ACCELERATE, HOLD_CREEP_THROTTLE) };
                    if phase == 3 {
                        let _: bool = unsafe { invoke!(n::SET_CONTROL_VALUE_NEXT_FRAME, PLAYER_CONTROL, INPUT_VEH_MOVE_LR, HOLD_STEER_CONTROL) };
                    }
                    let rot: shv_sys::Vector3 = unsafe { invoke!(n::GET_ENTITY_ROTATION_VELOCITY, veh) };
                    let _ = writeln!(
                        csv,
                        "{frame},{t:.3},{phase},{r_in:.4},{r_ang:.4},{},{:.4},{speed:.3}",
                        wrote.map(|o| format!("0x{o:X}")).unwrap_or("-".into()),
                        rot.z
                    );
                    t += dt as f64;
                    if t >= 4.0 * HOLD_PHASE_S {
                        let _ = csv.flush();
                        let _ = writeln!(log, "frame {frame}: HOLD done");
                        *status.lock().unwrap() = "write-hold done".into();
                        Mode::Idle
                    } else {
                        Mode::Hold { t, base: hold_base, csv }
                    }
                }
            }
        };
        let _ = log.flush();

        // --- HUD ---
        let st = status.lock().unwrap().clone();
        unsafe {
            let () = invoke!(n::DRAW_RECT, 0.22f32, 0.05f32, 0.42f32, 0.08f32, 0i32, 0i32, 0i32, 160i32, false);
            draw_text(0.015, 0.015, &format!("ADAS S4/S5  |  {:.1} km/h  |  {st}", speed * 3.6));
            draw_text(0.015, 0.040, "F8 spawn car  |  F9 steering sweep (stationary)  |  F10 pedals  |  F11 write-hold");
        }

        unsafe { shv_sys::scriptWait(0) };
    }
}

/// Steering input for the sweep: a sine of amplitude 1.
fn input_at(t: f64) -> f32 {
    if t < 0.0 {
        return 0.0;
    }
    (2.0 * std::f64::consts::PI * t / SWEEP_PERIOD_S).sin() as f32
}
