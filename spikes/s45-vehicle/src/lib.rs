//! Spikes S4/S5 (see docs/spikes/s45-results.md): find the steering angle in vehicle memory, test pedal
//! injection through blocked controls, and test whether a written steering
//! value holds. Throwaway code.
//!
//! Keys:
//! - F7  teleport to the test area in `adas-logs/teleport.txt` ("x y z heading",
//!   default: airport tarmac); turns the wanted level off for testing.
//! - F8  spawn the next test car (Sultan, Baller, Futo, Mule) and put the player
//!   in it; player invincible; the previous test car is deleted.
//! - F9  steering sweep (stationary): injects a sine steering input and ranks
//!   every float field of the vehicle struct by correlation (background thread).
//! - F10 pedal test (see `Mode::Pedals`).
//! - F11 drive test: speed up to 10 m/s, track a steering-angle command three
//!   ways, brake to a stop, try three standstill holds; snapshots the wheel
//!   structs to find wheel speeds; logs RPM and gear from natives.
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

const VK_F7: i32 = 0x76;
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
    pub const GET_ENTITY_SPEED_VECTOR: u64 = 0x9A8D700A51CB7B0D;
    pub const SET_ENTITY_COORDS: u64 = 0x06843DA7060A026B;
    pub const SET_ENTITY_HEADING: u64 = 0x8E2530AA8ADA980E;
    pub const GET_GROUND_Z_FOR_3D_COORD: u64 = 0xC906A7DAB05C8D2B;
    pub const REQUEST_COLLISION_AT_COORD: u64 = 0x07503F7948F491A7;
    pub const SET_MAX_WANTED_LEVEL: u64 = 0xAA5F02DB48D704B9;
    pub const CLEAR_PLAYER_WANTED_LEVEL: u64 = 0xB302540597885499;
    pub const PLAYER_ID: u64 = 0x4F8644AF03D0E0D6;
    pub const SET_ENTITY_AS_MISSION_ENTITY: u64 = 0xAD738C3085FE7E11;
    pub const DELETE_VEHICLE: u64 = 0xEA386986E786A54F;
    pub const SET_VEHICLE_HANDBRAKE: u64 = 0x684785568EF26A22;
    pub const SET_VEHICLE_BRAKE: u64 = 0xE4E2FD323574965C;
    pub const GET_VEHICLE_CURRENT_REV_RATIO: u64 = 0xF9DDA40BC293A61E;
    pub const GET_VEHICLE_CURRENT_DRIVE_GEAR: u64 = 0x56185A25D45A0DCD;
}

/// Steering fields found by the F9 sweep on build 1158.13 (Sultan):
/// 0x9D4 = smoothed steering input (±0.95), 0x9DC = steering angle in rad (±0.58).
const STEER_INPUT_OFFSET: usize = 0x9D4;
const STEER_ANGLE_OFFSET: usize = 0x9DC;
/// Injected control values have a 25% deadzone: throttle = (v - 0.25) / 0.75.
/// 0.45 gives about 0.27 throttle, enough to creep.
const CREEP_INJECT: f32 = 0.45;
const TEST_MODELS: [&CStr; 4] = [c"sultan", c"baller", c"futo", c"mule"];
const WHEEL_BYTES: usize = 0x400;

/// Test tooling: spawns a car 3 m ahead and puts the player in the driver's
/// seat; also makes the player invincible so fights or crashes don't end a test.
/// Yields the fiber while the model streams in.
fn spawn_test_car(ped: i32, name: &CStr, prev: &mut i32, log: &mut impl Write) {
    unsafe {
        let model: u32 = invoke!(n::GET_HASH_KEY, name);
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
        if *prev != 0 {
            let () = invoke!(n::SET_ENTITY_AS_MISSION_ENTITY, *prev, true, true);
            let () = invoke!(n::DELETE_VEHICLE, prev as *mut i32);
        }
        *prev = veh;
        let _ = writeln!(log, "SPAWN {} handle {veh} after {waited} frames; player invincible", name.to_string_lossy());
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
    prev: [bool; 5],
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
    Drive(Box<Drive>),
}

// ---- Drive test ----

/// Drive-test phases and their maximum length in seconds.
const DRIVE_PHASES: [(&str, f64); 10] = [
    ("ramp", 4.0),              // speed up to 10 m/s, steering written to 0
    ("track-write", 5.0),       // V0: write the steering input only (feed-forward)
    ("settle", 1.5),
    ("track-write+ctl", 5.0),   // V1: write + inject the matching steering control
    ("settle", 1.5),
    ("track-closed", 5.0),      // V2: V1 + integral correction on the measured angle
    ("stop", 6.0),              // injected brake until < 0.3 m/s
    ("hold-handbrake", 2.0),    // SET_VEHICLE_HANDBRAKE, creep throttle against it
    ("hold-brake-native", 2.0), // SET_VEHICLE_BRAKE, creep throttle against it
    ("no-hold", 2.0),           // creep throttle, nothing holding (baseline)
];
const DRIVE_TARGET_MPS: f32 = 10.0;

struct Drive {
    phase: usize,
    tp: f64,
    t: f64,
    base: usize,
    csv: BufWriter<File>,
    ratio: f32,
    integ: f32,
    prev_cmd: f32,
    /// Candidate wheel arrays: (offset in the vehicle struct, wheel pointers).
    wheels: Vec<(usize, Vec<usize>)>,
    /// Per frame while moving: speed and one concatenated snapshot of all wheels.
    wheel_frames: Vec<(f32, Vec<u8>)>,
}

/// Steering-angle command (rad, ISO 8855: left positive) at `tp` seconds into
/// a tracking phase: a left step, a right step, then a 0.8 Hz sine.
fn angle_profile(tp: f64) -> f32 {
    if tp < 1.5 {
        0.04
    } else if tp < 3.0 {
        -0.04
    } else {
        (0.06 * (2.0 * std::f64::consts::PI * 0.8 * (tp - 3.0)).sin()) as f32
    }
}

fn read_u64(addr: usize) -> Option<u64> {
    let mut b = [0u8; 8];
    read_mem(addr, &mut b).then(|| u64::from_le_bytes(b))
}

fn read_u32(addr: usize) -> Option<u32> {
    let mut b = [0u8; 4];
    read_mem(addr, &mut b).then(|| u32::from_le_bytes(b))
}

fn plausible_ptr(p: u64) -> bool {
    (0x1_0000..0x7FFF_FFFF_FFFF).contains(&p) && p.is_multiple_of(8)
}

/// Looks for (pointer, count) pairs in the vehicle struct whose pointer leads to
/// `count` distinct, readable blocks: the shape of a wheel array.
fn find_wheel_arrays(base: usize) -> Vec<(usize, Vec<usize>)> {
    let mut out = Vec::new();
    let mut probe = vec![0u8; WHEEL_BYTES];
    for off in (0..SNAP_BYTES - 16).step_by(8) {
        let (Some(p), Some(count)) = (read_u64(base + off), read_u32(base + off + 8)) else { continue };
        if !(2..=8).contains(&count) || !plausible_ptr(p) {
            continue;
        }
        let mut ptrs = Vec::new();
        for i in 0..count as usize {
            match read_u64(p as usize + i * 8) {
                Some(w) if plausible_ptr(w) && !ptrs.contains(&(w as usize)) && read_mem(w as usize, &mut probe) => ptrs.push(w as usize),
                _ => break,
            }
        }
        if ptrs.len() == count as usize {
            out.push((off, ptrs));
        }
        if out.len() >= 6 {
            break;
        }
    }
    out
}

/// For each wheel of each candidate array, the float fields that track the
/// vehicle speed; `k` is the least-squares slope field/speed (about ±1/r for an
/// angular speed in rad/s, ±1 for a linear speed).
fn analyse_wheels(wheels: &[(usize, Vec<usize>)], frames: &[(f32, Vec<u8>)], report: &std::path::Path) {
    let Ok(f) = File::create(report) else { return };
    let mut w = BufWriter::new(f);
    let speed: Vec<f32> = frames.iter().map(|f| f.0).collect();
    let (lo, hi) = speed.iter().fold((f32::MAX, f32::MIN), |(a, b), &v| (a.min(v), b.max(v)));
    let _ = writeln!(w, "{} frames, speed {lo:.2}..{hi:.2} m/s", frames.len());
    let mut slot = 0usize;
    for (arr_off, ptrs) in wheels {
        let _ = writeln!(w, "== array at vehicle+0x{arr_off:X}: {} wheels", ptrs.len());
        for (wi, ptr) in ptrs.iter().enumerate() {
            let mut hits = Vec::new();
            for o in (0..WHEEL_BYTES).step_by(4) {
                let at = slot * WHEEL_BYTES + o;
                let series: Vec<f32> = frames.iter().map(|f| f32::from_le_bytes([f.1[at], f.1[at + 1], f.1[at + 2], f.1[at + 3]])).collect();
                if series.iter().any(|v| !v.is_finite() || v.abs() > 1.0e4) {
                    continue;
                }
                let r = pearson(&series, &speed);
                if r.abs() >= 0.9 {
                    let (sxy, sxx) = series.iter().zip(&speed).fold((0f64, 0f64), |(a, b), (&y, &x)| (a + (x * y) as f64, b + (x * x) as f64));
                    hits.push((o, r, (sxy / sxx.max(1e-9)) as f32));
                }
            }
            hits.sort_by(|a, b| b.1.abs().total_cmp(&a.1.abs()));
            let _ = writeln!(w, "-- wheel {wi} (0x{ptr:X}): {} fields with |r| >= 0.9", hits.len());
            for (o, r, k) in hits.iter().take(12) {
                let _ = writeln!(w, "   +0x{o:03X}  r {r:+.4}  k {k:+.4}");
            }
            slot += 1;
        }
    }
}

/// Test tooling: moves the player (or their car) to the spot in
/// `adas-logs/teleport.txt`, or to the airport tarmac.
fn teleport(ped: i32, veh: i32, log: &mut impl Write) {
    let cfg = std::fs::read_to_string(log_dir().join("teleport.txt")).unwrap_or_default();
    let v: Vec<f32> = cfg.split_whitespace().filter_map(|x| x.parse().ok()).collect();
    let (x, y, z, h) = if v.len() >= 4 { (v[0], v[1], v[2], v[3]) } else { (-1336.0, -3044.0, 13.9, 330.0) };
    let e = if veh != 0 { veh } else { ped };
    unsafe {
        let () = invoke!(n::SET_MAX_WANTED_LEVEL, 0i32);
        let player: i32 = invoke!(n::PLAYER_ID);
        let () = invoke!(n::CLEAR_PLAYER_WANTED_LEVEL, player);
        let () = invoke!(n::SET_ENTITY_COORDS, e, x, y, z + 1.0, false, false, false, true);
        for _ in 0..60 {
            let () = invoke!(n::REQUEST_COLLISION_AT_COORD, x, y, z);
            shv_sys::scriptWait(0);
        }
        let mut gz = z;
        let found: bool = invoke!(n::GET_GROUND_Z_FOR_3D_COORD, x, y, z + 50.0, &mut gz as *mut f32, false, false);
        let () = invoke!(n::SET_ENTITY_COORDS, e, x, y, if found { gz + 0.5 } else { z + 0.5 }, false, false, false, true);
        let () = invoke!(n::SET_ENTITY_HEADING, e, h);
        if veh != 0 {
            let _: bool = invoke!(n::SET_VEHICLE_ON_GROUND_PROPERLY, veh, 5.0f32);
        }
        let _ = writeln!(log, "TELEPORT to {x} {y} (ground found {found}, z {gz:.2}) heading {h}; wanted level off");
    }
}

fn vehicle_name(veh: i32) -> (u32, String) {
    let model: u32 = unsafe { invoke!(n::GET_ENTITY_MODEL, veh) };
    let name_ptr: *const core::ffi::c_char = unsafe { invoke!(n::GET_DISPLAY_NAME_FROM_VEHICLE_MODEL, model) };
    let name = if name_ptr.is_null() { "?".into() } else { unsafe { CStr::from_ptr(name_ptr) }.to_string_lossy().into_owned() };
    (model, name)
}

// ---- The spike ----

fn run() {
    let dir = log_dir();
    let stem = run_stem();
    let mut log = BufWriter::new(File::create(dir.join(format!("{stem}.log"))).expect("create log"));
    let _ = writeln!(log, "S4/S5 spike started. F7 teleport, F8 next car, F9 sweep, F10 pedals, F11 drive test");
    let _ = log.flush();

    let mut keys = Keys { prev: [false; 5] };
    let mut mode = Mode::Idle;
    let status: Arc<Mutex<String>> = Arc::new(Mutex::new("idle".into()));
    let angle_offset: Arc<Mutex<Option<usize>>> = Arc::new(Mutex::new(None));
    let mut run_no = 0u32;
    let mut next_model = 0usize;
    let mut test_car = 0i32;

    loop {
        let ped: i32 = unsafe { invoke!(n::PLAYER_PED_ID) };
        let in_vehicle: bool = unsafe { invoke!(n::IS_PED_IN_ANY_VEHICLE, ped, false) };
        let veh: i32 = if in_vehicle { unsafe { invoke!(n::GET_VEHICLE_PED_IS_IN, ped, false) } } else { 0 };
        let dt: f32 = unsafe { invoke!(n::GET_FRAME_TIME) };
        let frame: i32 = unsafe { invoke!(n::GET_FRAME_COUNT) };
        let speed: f32 = if in_vehicle { unsafe { invoke!(n::GET_ENTITY_SPEED, veh) } } else { 0.0 };
        let base = if in_vehicle { unsafe { shv_sys::getScriptHandleBaseAddress(veh) as usize } } else { 0 };

        let idle = matches!(mode, Mode::Idle);
        if keys.pressed(4, VK_F7) && idle {
            teleport(ped, veh, &mut log);
        }
        if keys.pressed(3, VK_F8) && idle {
            let name = TEST_MODELS[next_model % TEST_MODELS.len()];
            next_model += 1;
            spawn_test_car(ped, name, &mut test_car, &mut log);
        }
        let f9 = keys.pressed(0, VK_F9);
        let f10 = keys.pressed(1, VK_F10);
        let f11 = keys.pressed(2, VK_F11);

        if idle && in_vehicle && base != 0 {
            if f9 {
                let (model, name) = vehicle_name(veh);
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
                let (model, name) = vehicle_name(veh);
                let f = File::create(dir.join(format!("{stem}-drive-{run_no}.csv"))).expect("create drive csv");
                let mut csv = BufWriter::new(f);
                let _ = writeln!(
                    csv,
                    "frame,t_s,phase,name,cmd,wrote,ctl,read_in,read_ang,ratio,yaw_rate,speed_mps,vx_rel,vy_rel,thr_inj,brk_inj,veh_throttle,rev_ratio,gear,w0_b0,w1_b0,w2_b0,w3_b0,w0_170,w1_170,w2_170,w3_170"
                );
                let wheels = find_wheel_arrays(base);
                let _ = writeln!(log, "frame {frame}: DRIVE start, vehicle {name} (model 0x{model:08X}), file drive-{run_no}; wheel-array candidates:");
                for (off, ptrs) in &wheels {
                    let list: Vec<String> = ptrs.iter().map(|p| format!("0x{p:X}")).collect();
                    let _ = writeln!(log, "  vehicle+0x{off:X}: {} -> {}", ptrs.len(), list.join(" "));
                }
                *status.lock().unwrap() = format!("drive test ({name})");
                mode = Mode::Drive(Box::new(Drive {
                    phase: 0,
                    tp: 0.0,
                    t: 0.0,
                    base,
                    csv,
                    ratio: 0.6,
                    integ: 0.0,
                    prev_cmd: 0.0,
                    wheels,
                    wheel_frames: Vec::new(),
                }));
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
            Mode::Drive(mut d) => {
                if !in_vehicle || base != d.base {
                    let _ = writeln!(log, "frame {frame}: DRIVE aborted (left vehicle)");
                    *status.lock().unwrap() = "drive aborted".into();
                    Mode::Idle
                } else {
                    let (pname, plen) = DRIVE_PHASES[d.phase];
                    // What the game made of last frame's commands.
                    let r_in = read_f32(base + STEER_INPUT_OFFSET).unwrap_or(f32::NAN);
                    let r_ang = read_f32(base + STEER_ANGLE_OFFSET).unwrap_or(f32::NAN);
                    if r_in.abs() > 0.05 && r_ang.is_finite() {
                        d.ratio += 0.1 * (r_ang / r_in - d.ratio);
                    }
                    let tracking = matches!(d.phase, 1 | 3 | 5);
                    let cmd = if tracking { angle_profile(d.tp) } else { 0.0 };
                    if d.phase == 5 {
                        d.integ = (d.integ + 0.3 * (d.prev_cmd - r_ang) / d.ratio).clamp(-0.3, 0.3);
                    } else if !tracking {
                        d.integ = 0.0;
                    }
                    let mut wrote = f32::NAN;
                    let mut ctl = 0.0f32;
                    if d.phase <= 6 {
                        let w = (cmd / d.ratio + d.integ).clamp(-1.0, 1.0);
                        write_f32(base + STEER_INPUT_OFFSET, w);
                        wrote = w;
                        if matches!(d.phase, 3..=5) {
                            // MOVE_LR is right-positive; the input field is left-positive.
                            ctl = -w;
                            let _: bool = unsafe { invoke!(n::SET_CONTROL_VALUE_NEXT_FRAME, PLAYER_CONTROL, INPUT_VEH_MOVE_LR, ctl) };
                        }
                    }
                    // Pedals.
                    let (mut thr, mut brk) = (0.0f32, 0.0f32);
                    match d.phase {
                        0..=5 => {
                            let u = (0.3 + 0.15 * (DRIVE_TARGET_MPS - speed)).clamp(0.0, 1.0);
                            if u > 0.01 {
                                thr = 0.25 + 0.75 * u;
                            }
                        }
                        6 => brk = 1.0,
                        _ => thr = CREEP_INJECT,
                    }
                    unsafe {
                        if thr > 0.0 {
                            let _: bool = invoke!(n::SET_CONTROL_VALUE_NEXT_FRAME, PLAYER_CONTROL, INPUT_VEH_ACCELERATE, thr);
                        }
                        if brk > 0.0 {
                            let _: bool = invoke!(n::SET_CONTROL_VALUE_NEXT_FRAME, PLAYER_CONTROL, INPUT_VEH_BRAKE, brk);
                        }
                        let () = invoke!(n::SET_VEHICLE_HANDBRAKE, veh, d.phase == 7);
                        let () = invoke!(n::SET_VEHICLE_BRAKE, veh, d.phase == 8);
                    }
                    let rot: shv_sys::Vector3 = unsafe { invoke!(n::GET_ENTITY_ROTATION_VELOCITY, veh) };
                    let vel: shv_sys::Vector3 = unsafe { invoke!(n::GET_ENTITY_SPEED_VECTOR, veh, true) };
                    let veh_thr: f32 = unsafe { invoke!(n::VEHICLE_THROTTLE, veh) };
                    let rev: f32 = unsafe { invoke!(n::GET_VEHICLE_CURRENT_REV_RATIO, veh) };
                    let gear: i32 = unsafe { invoke!(n::GET_VEHICLE_CURRENT_DRIVE_GEAR, veh) };
                    let mut wf = [f32::NAN; 8];
                    if let Some((_, ptrs)) = d.wheels.first() {
                        for (i, &p) in ptrs.iter().take(4).enumerate() {
                            wf[i] = read_f32(p + 0xB0).unwrap_or(f32::NAN);
                            wf[4 + i] = read_f32(p + 0x170).unwrap_or(f32::NAN);
                        }
                    }
                    let wcols: Vec<String> = wf.iter().map(|v| format!("{v:.3}")).collect();
                    let _ = writeln!(
                        d.csv,
                        "{frame},{:.3},{},{pname},{cmd:.4},{wrote:.4},{ctl:.4},{r_in:.4},{r_ang:.4},{:.4},{:.4},{speed:.3},{:.3},{:.3},{thr:.3},{brk:.2},{veh_thr:.3},{rev:.3},{gear},{}",
                        d.t, d.phase, d.ratio, rot.z, vel.x, vel.y, wcols.join(",")
                    );
                    if d.phase <= 6 && !d.wheels.is_empty() {
                        let mut snap = Vec::with_capacity(WHEEL_BYTES * 16);
                        let mut chunk = vec![0u8; WHEEL_BYTES];
                        for (_, ptrs) in &d.wheels {
                            for &p in ptrs {
                                if !read_mem(p, &mut chunk) {
                                    chunk.fill(0);
                                }
                                snap.extend_from_slice(&chunk);
                            }
                        }
                        d.wheel_frames.push((speed, snap));
                    }
                    d.prev_cmd = cmd;
                    d.t += dt as f64;
                    d.tp += dt as f64;
                    let stop_done = d.phase == 6 && speed < 0.3 && d.tp > 0.3;
                    if d.tp >= plen || stop_done {
                        d.phase += 1;
                        d.tp = 0.0;
                    }
                    *status.lock().unwrap() = format!("drive: {pname}");
                    if d.phase >= DRIVE_PHASES.len() {
                        unsafe {
                            let () = invoke!(n::SET_VEHICLE_HANDBRAKE, veh, false);
                            let () = invoke!(n::SET_VEHICLE_BRAKE, veh, false);
                        }
                        let _ = d.csv.flush();
                        let _ = writeln!(log, "frame {frame}: DRIVE done; {} wheel frames, analysing in background", d.wheel_frames.len());
                        let st = status.clone();
                        let report = dir.join(format!("{stem}-wheels-{run_no}.txt"));
                        let Drive { wheels, wheel_frames, .. } = *d;
                        std::thread::spawn(move || {
                            analyse_wheels(&wheels, &wheel_frames, &report);
                            *st.lock().unwrap() = "drive test done".into();
                        });
                        Mode::Idle
                    } else {
                        Mode::Drive(d)
                    }
                }
            }
        };
        let _ = log.flush();

        // --- HUD ---
        let st = status.lock().unwrap().clone();
        unsafe {
            let () = invoke!(n::DRAW_RECT, 0.24f32, 0.05f32, 0.46f32, 0.08f32, 0i32, 0i32, 0i32, 160i32, false);
            draw_text(0.015, 0.015, &format!("ADAS S4/S5  |  {:.1} km/h  |  {st}", speed * 3.6));
            draw_text(0.015, 0.040, "F7 teleport  |  F8 next car  |  F9 sweep (stationary)  |  F10 pedals  |  F11 drive test");
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
