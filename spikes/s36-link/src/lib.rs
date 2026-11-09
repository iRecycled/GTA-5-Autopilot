//! Spikes S3 + S6, run together in one GTA session. Throwaway code: the parts
//! that survive move into `gta-plugin`.
//!
//! **S3, the adapter ↔ core link.** Every frame this plugin writes a
//! `VehicleSample` to the shared-memory link, wakes the core (`adas-core`, a
//! separate process), and waits up to 1 ms for the `Command` that answers this
//! very sample (the reactive timing mode, D23). It applies the command: pedal
//! demands through GTA's 25% deadzone, the standstill hold as the handbrake,
//! steering by the per-frame input write (D5). A watchdog releases everything
//! and shows TAKE OVER if no fresh command arrives for 6 frames (D24).
//!
//! **S6, the ray budget.** With the ray pass on, every frame it lists all
//! vehicles, finds those inside the 5 radars' fields of view (front long-range,
//! four corners), starts one line-of-sight probe per (radar, target) pair, and
//! fires 12 ultrasonic sensors × 3 rays. Results are read the next frame (S1:
//! async probes are always 1 frame late). The whole pass is timed.
//!
//! Keys:
//! - F6  toggle a 60 fps cap (busy-wait in the script tick), to test 60 vs 144 fps
//! - F7  teleport to the next spot in `adas-logs/teleport.txt` (one "x y z heading" per line)
//! - F8  spawn a Sultan and put the player in it (player invincible)
//! - F9  toggle GTA's AI driving the car (traffic for the ray test)
//! - F10 toggle the engage request (the core drives its speed profile)
//! - F11 cycle the ray pass: off → native-async → mem-async → mem-sync → off
//!
//! Output: `<game folder>/adas-logs/s36-<unix time>.log` and `-frames.csv`.

use core::ffi::c_void;
use std::ffi::CString;
use std::collections::HashMap;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::panic::{AssertUnwindSafe, catch_unwind};
use std::path::PathBuf;
use std::sync::atomic::{AtomicPtr, Ordering};

use adapter_api::shm::{DEFAULT_NAME, SharedLink};
use adapter_api::{CONTRACT_VERSION, Header, MsgKind, VehicleSample, command_flags, sample_flags, wall_time_ns};
use shv_sys::{Vector3, invoke};

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

const VK_F6: i32 = 0x75;
const VK_F7: i32 = 0x76;
const VK_F8: i32 = 0x77;
const VK_F9: i32 = 0x78;
const VK_F10: i32 = 0x79;
const VK_F11: i32 = 0x7A;

// ---- Natives (hashes checked against alloc8or/gta5-nativedb-data) ----

mod n {
    pub const PLAYER_PED_ID: u64 = 0xD80958FC74E988A6;
    pub const PLAYER_ID: u64 = 0x4F8644AF03D0E0D6;
    pub const IS_PED_IN_ANY_VEHICLE: u64 = 0x997ABD671D25CA0B;
    pub const GET_VEHICLE_PED_IS_IN: u64 = 0x9A9112A0FE9A4713;
    pub const GET_ENTITY_SPEED: u64 = 0xD5037BA82E12416F;
    pub const GET_ENTITY_SPEED_VECTOR: u64 = 0x9A8D700A51CB7B0D;
    pub const GET_ENTITY_ROTATION_VELOCITY: u64 = 0x213B91045D09B983;
    pub const GET_ENTITY_COORDS: u64 = 0x3FEF770D40960D5A;
    pub const GET_ENTITY_FORWARD_VECTOR: u64 = 0x0A794A5A57F8DF91;
    pub const GET_ENTITY_HEADING: u64 = 0xE83D4F9BA2A38914;
    pub const GET_ENTITY_MODEL: u64 = 0x9F47B058362C84B5;
    pub const GET_MODEL_DIMENSIONS: u64 = 0x03E8D3D5F549087A;
    pub const GET_FRAME_TIME: u64 = 0x15C40837039FFAF7;
    pub const GET_FRAME_COUNT: u64 = 0xFC8202EFC642E6F2;
    pub const IS_PAUSE_MENU_ACTIVE: u64 = 0xB0034A223497FFCB;
    pub const GET_IS_LOADING_SCREEN_ACTIVE: u64 = 0x10D0A8F259E93EC9;
    pub const GET_VEHICLE_CURRENT_REV_RATIO: u64 = 0xF9DDA40BC293A61E;
    pub const GET_VEHICLE_CURRENT_DRIVE_GEAR: u64 = 0x56185A25D45A0DCD;
    pub const SET_CONTROL_VALUE_NEXT_FRAME: u64 = 0xE8A25867FBA3B05E;
    pub const SET_VEHICLE_HANDBRAKE: u64 = 0x684785568EF26A22;
    pub const GET_PED_IN_VEHICLE_SEAT: u64 = 0xBB40DD2270B65366;
    pub const START_SHAPE_TEST_LOS_PROBE: u64 = 0x7EE9F5D83DD4F90E;
    pub const START_EXPENSIVE_SYNCHRONOUS_SHAPE_TEST_LOS_PROBE: u64 = 0x377906D8A31E5586;
    pub const GET_SHAPE_TEST_RESULT: u64 = 0x3D87450E15D98694;
    pub const TASK_VEHICLE_DRIVE_WANDER: u64 = 0x480142959D337D00;
    pub const CLEAR_PED_TASKS: u64 = 0xE1EF3C1216AFF2CD;
    pub const GET_HASH_KEY: u64 = 0xD24D37CC275948CC;
    pub const REQUEST_MODEL: u64 = 0x963D27A58DF860AC;
    pub const HAS_MODEL_LOADED: u64 = 0x98A4EB5D89A0C952;
    pub const SET_MODEL_AS_NO_LONGER_NEEDED: u64 = 0xE532F5D78798DAAB;
    pub const CREATE_VEHICLE: u64 = 0xAF35D0D2583051B0;
    pub const SET_VEHICLE_ON_GROUND_PROPERLY: u64 = 0x49733E92263139D1;
    pub const SET_PED_INTO_VEHICLE: u64 = 0xF75B0D629E1C063D;
    pub const SET_ENTITY_INVINCIBLE: u64 = 0x3882114BDE571AD4;
    pub const SET_ENTITY_AS_MISSION_ENTITY: u64 = 0xAD738C3085FE7E11;
    pub const DELETE_VEHICLE: u64 = 0xEA386986E786A54F;
    pub const SET_ENTITY_COORDS: u64 = 0x06843DA7060A026B;
    pub const SET_ENTITY_HEADING: u64 = 0x8E2530AA8ADA980E;
    pub const GET_GROUND_Z_FOR_3D_COORD: u64 = 0xC906A7DAB05C8D2B;
    pub const REQUEST_COLLISION_AT_COORD: u64 = 0x07503F7948F491A7;
    pub const SET_MAX_WANTED_LEVEL: u64 = 0xAA5F02DB48D704B9;
    pub const CLEAR_PLAYER_WANTED_LEVEL: u64 = 0xB302540597885499;
    pub const BEGIN_TEXT_COMMAND_DISPLAY_TEXT: u64 = 0x25FBB336DF1804CB;
    pub const ADD_TEXT_COMPONENT_SUBSTRING_PLAYER_NAME: u64 = 0x6C188BE134E074AA;
    pub const END_TEXT_COMMAND_DISPLAY_TEXT: u64 = 0xCD015E5BB0D96A57;
    pub const SET_TEXT_SCALE: u64 = 0x07C837F9A01C34C9;
    pub const SET_TEXT_FONT: u64 = 0x66E0276CC5F6B9DA;
    pub const SET_TEXT_COLOUR: u64 = 0xBE6B23FFA53FB442;
    pub const SET_TEXT_OUTLINE: u64 = 0x2513DFB0FB8400FE;
    pub const DRAW_RECT: u64 = 0x3A618A217E5154F0;
}

// ---- GTA specifics (S4/S5 findings, build 1158.13) ----

const STEER_INPUT_OFFSET: usize = 0x9D4;
const STEER_ANGLE_OFFSET: usize = 0x9DC;
const WHEEL_ARRAY_OFFSET: usize = 0xC30;
const WHEEL_COUNT_OFFSET: usize = 0xC38;
const WHEEL_OMEGA_OFFSET: usize = 0x170;
const PEDAL_DEADZONE: f32 = 0.25;
const PLAYER_CONTROL: i32 = 0;
const INPUT_VEH_ACCELERATE: i32 = 71;
const INPUT_VEH_BRAKE: i32 = 72;
const AI_SPEED_MPS: f32 = 20.0;
const AI_DRIVING_STYLE: i32 = 786_603;

// ---- Link and timing ----

/// How long the adapter waits in a frame for the core's answer (reactive mode).
const REPLY_WAIT_NS: u64 = 1_000_000;
/// Watchdog (D24): no fresh command for this many frames → release, TAKE OVER.
const WATCHDOG_FRAMES: u32 = 6;
const CAP_60_NS: u64 = 16_666_667;

// ---- Sensor layout for the ray test (D25) ----

/// A radar: mount corner, facing (deg, left positive, 0 = forward), half field
/// of view (deg), range (m).
struct Radar {
    corner: (i8, i8), // (x: -1 left/0 centre/+1 right, y: -1 rear/+1 front) of the bounding box
    facing_deg: f32,
    half_fov_deg: f32,
    range_m: f32,
}

const RADARS: [Radar; 5] = [
    Radar { corner: (0, 1), facing_deg: 0.0, half_fov_deg: 30.0, range_m: 160.0 }, // front long-range
    Radar { corner: (-1, 1), facing_deg: 45.0, half_fov_deg: 75.0, range_m: 80.0 }, // front-left corner
    Radar { corner: (1, 1), facing_deg: -45.0, half_fov_deg: 75.0, range_m: 80.0 }, // front-right corner
    Radar { corner: (-1, -1), facing_deg: 135.0, half_fov_deg: 75.0, range_m: 80.0 }, // rear-left corner
    Radar { corner: (1, -1), facing_deg: -135.0, half_fov_deg: 75.0, range_m: 80.0 }, // rear-right corner
];
const US_PER_BUMPER: usize = 6;
const US_RAYS: [f32; 3] = [-15.0, 0.0, 15.0];
const US_RANGE_M: f32 = 5.5;
/// Map + vehicles + objects (what blocks a radar's view).
const LOS_FLAGS: i32 = 1 | 2 | 16;
/// Everything (ultrasonic sees peds too).
const US_FLAGS: i32 = 511;
const MAX_VEHICLES: usize = 1024;

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

fn log_dir() -> PathBuf {
    let mut buf = [0u16; 1024];
    let len = unsafe { GetModuleFileNameW(MODULE.load(Ordering::Relaxed), buf.as_mut_ptr(), buf.len() as u32) };
    let path = PathBuf::from(String::from_utf16_lossy(&buf[..len as usize]));
    let dir = path.parent().map(|p| p.join("adas-logs")).unwrap_or_else(|| PathBuf::from("adas-logs"));
    let _ = std::fs::create_dir_all(&dir);
    dir
}

fn run_stem() -> String {
    let secs = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0);
    format!("s36-{secs}")
}

struct Keys {
    prev: [bool; 6],
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

fn read_bytes<const N: usize>(addr: usize) -> Option<[u8; N]> {
    let mut b = [0u8; N];
    let mut read = 0usize;
    let ok = unsafe { ReadProcessMemory(GetCurrentProcess(), addr as *const c_void, b.as_mut_ptr() as *mut c_void, N, &mut read) };
    (ok != 0 && read == N).then_some(b)
}

fn read_f32(addr: usize) -> Option<f32> {
    read_bytes::<4>(addr).map(f32::from_le_bytes)
}

fn read_u64(addr: usize) -> Option<u64> {
    read_bytes::<8>(addr).map(u64::from_le_bytes)
}

fn read_u32(addr: usize) -> Option<u32> {
    read_bytes::<4>(addr).map(u32::from_le_bytes)
}

fn write_f32(addr: usize, v: f32) -> bool {
    let b = v.to_le_bytes();
    let mut written = 0usize;
    let ok = unsafe { WriteProcessMemory(GetCurrentProcess(), addr as *mut c_void, b.as_ptr() as *const c_void, 4, &mut written) };
    ok != 0 && written == 4
}

fn pct(sorted: &[u64], p: f64) -> u64 {
    if sorted.is_empty() {
        return 0;
    }
    sorted[((sorted.len() - 1) as f64 * p).round() as usize]
}

/// Wraps an angle in degrees to (-180, 180].
fn wrap_deg(a: f32) -> f32 {
    let mut a = a % 360.0;
    if a > 180.0 {
        a -= 360.0;
    } else if a <= -180.0 {
        a += 360.0;
    }
    a
}

fn spawn_sultan(ped: i32, prev: &mut i32, log: &mut impl Write) {
    unsafe {
        let model: u32 = invoke!(n::GET_HASH_KEY, c"sultan");
        let () = invoke!(n::REQUEST_MODEL, model);
        for waited in 0.. {
            let loaded: bool = invoke!(n::HAS_MODEL_LOADED, model);
            if loaded {
                break;
            }
            if waited > 300 {
                let _ = writeln!(log, "SPAWN failed: model did not load");
                return;
            }
            shv_sys::scriptWait(0);
        }
        let pos: Vector3 = invoke!(n::GET_ENTITY_COORDS, ped, true);
        let fwd: Vector3 = invoke!(n::GET_ENTITY_FORWARD_VECTOR, ped);
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
        let _ = writeln!(log, "SPAWN sultan handle {veh}; player invincible");
    }
}

/// Teleports to line `idx` of `adas-logs/teleport.txt`; returns the number of spots.
fn teleport(ped: i32, veh: i32, idx: usize, log: &mut impl Write) -> usize {
    let cfg = std::fs::read_to_string(log_dir().join("teleport.txt")).unwrap_or_default();
    let spots: Vec<Vec<f32>> = cfg
        .lines()
        .map(|l| l.split_whitespace().filter_map(|x| x.parse().ok()).collect::<Vec<f32>>())
        .filter(|v| v.len() >= 4)
        .collect();
    if spots.is_empty() {
        let _ = writeln!(log, "TELEPORT: no spots in teleport.txt");
        return 0;
    }
    let s = &spots[idx % spots.len()];
    let (x, y, z, h) = (s[0], s[1], s[2], s[3]);
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
        let () = invoke!(n::CLEAR_PLAYER_WANTED_LEVEL, player);
    }
    let _ = writeln!(log, "TELEPORT spot {} of {}: {x} {y} heading {h}", idx % spots.len() + 1, spots.len());
    spots.len()
}

/// Reads the wheel spin speeds (rad/s, positive forward) in host order.
fn wheel_omegas(base: usize) -> [f32; 4] {
    let mut out = [f32::NAN; 4];
    let (Some(arr), Some(count)) = (read_u64(base + WHEEL_ARRAY_OFFSET), read_u32(base + WHEEL_COUNT_OFFSET)) else {
        return out;
    };
    for (i, o) in out.iter_mut().enumerate().take(count.min(4) as usize) {
        if let Some(w) = read_u64(arr as usize + i * 8)
            && let Some(v) = read_f32(w as usize + WHEEL_OMEGA_OFFSET)
        {
            *o = -v;
        }
    }
    out
}

/// Ray-pass modes, cycled by F11.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum RayMode {
    Off,
    /// Positions from `GET_ENTITY_COORDS`; scheduled async probes.
    NativeAsync,
    /// Positions read from entity memory; scheduled async probes.
    MemAsync,
    /// Positions from memory; synchronous probes, up to `SYNC_PER_FRAME`.
    MemSync,
}

impl RayMode {
    fn next(self) -> Self {
        match self {
            RayMode::Off => RayMode::NativeAsync,
            RayMode::NativeAsync => RayMode::MemAsync,
            RayMode::MemAsync => RayMode::MemSync,
            RayMode::MemSync => RayMode::Off,
        }
    }
    fn name(self) -> &'static str {
        match self {
            RayMode::Off => "off",
            RayMode::NativeAsync => "native-async",
            RayMode::MemAsync => "mem-async",
            RayMode::MemSync => "mem-sync",
        }
    }
}

/// GTA allows about 10 async shape tests in flight (S6 run 1); keep 2 spare.
const MAX_IN_FLIGHT: usize = 8;
const SYNC_PER_FRAME: usize = 12;
/// Desired refresh period per radar target and per ultrasonic ray, in frames
/// (at 72 fps: about 18 Hz and 14 Hz).
const RADAR_PERIOD_FRAMES: f32 = 4.0;
const US_PERIOD_FRAMES: f32 = 5.0;
/// Park sensors work only at parking speeds, as on real cars.
const US_MAX_SPEED: f32 = 3.0;
/// The parked-car count is a statistic, sampled every this many frames.
const SEAT_EVERY: u32 = 30;
/// Entity position (world, f32 × 3) in the entity struct (CEntity matrix row 3).
const ENTITY_POS_OFFSET: usize = 0x90;

/// Per-frame ray statistics.
#[derive(Default, Clone, Copy)]
struct RayStats {
    vehicles: u32,
    parked: u32,
    /// (radar, target) pairs in a field of view.
    pairs: u32,
    /// Distinct targets inside any radar's field of view.
    targets: u32,
    started: u32,
    failed: u32,
    done: u32,
    clear: u32,
    us_on: bool,
    us_hits: u32,
    /// Frames since each in-view target's last line-of-sight result.
    age_mean: f32,
    age_max: u32,
    t_results: u64,
    t_list: u64,
    t_pos: u64,
    t_seat: u64,
    t_sched: u64,
    t_probe: u64,
    ns: u64,
    /// Memory vs native position, max over a few vehicles (m); NaN if not checked.
    pos_err: f32,
}

/// A target in view: (vehicle, world position, radar mount in the ego frame).
type Target = (i32, (f32, f32, f32), (f32, f32));

struct RaySystem {
    mode: RayMode,
    frame: u32,
    /// (handle, key, start frame). Key: a vehicle handle, or -(1 + ultrasonic ray index).
    pending: Vec<(i32, i32, u32)>,
    checked: HashMap<i32, u32>,
    first_seen: HashMap<i32, u32>,
    parked: u32,
    vbuf: Vec<i32>,
    dims: (Vector3, Vector3),
    dims_for: i32,
}

/// Polls a shape test: (0 = no such test, 1 = pending, 2 = ready; hit; entity).
unsafe fn ray_result(handle: i32) -> (i32, bool, i32) {
    let mut hit = 0i32;
    let mut end = Vector3::default();
    let mut normal = Vector3::default();
    let mut entity = 0i32;
    let status: i32 = unsafe {
        invoke!(
            n::GET_SHAPE_TEST_RESULT,
            handle,
            &mut hit as *mut i32,
            &mut end as *mut Vector3,
            &mut normal as *mut Vector3,
            &mut entity as *mut i32,
        )
    };
    (status, hit != 0, entity)
}

fn mem_pos(v: i32) -> Option<(f32, f32, f32)> {
    let b = unsafe { shv_sys::getScriptHandleBaseAddress(v) } as usize;
    if b == 0 {
        return None;
    }
    let raw = read_bytes::<12>(b + ENTITY_POS_OFFSET)?;
    let f = |i: usize| f32::from_le_bytes([raw[i], raw[i + 1], raw[i + 2], raw[i + 3]]);
    Some((f(0), f(4), f(8)))
}

impl RaySystem {
    fn new() -> Self {
        Self {
            mode: RayMode::Off,
            frame: 0,
            pending: Vec::new(),
            checked: HashMap::new(),
            first_seen: HashMap::new(),
            parked: 0,
            vbuf: vec![0i32; MAX_VEHICLES],
            dims: (Vector3::default(), Vector3::default()),
            dims_for: 0,
        }
    }

    fn set_mode(&mut self, m: RayMode) {
        self.mode = m;
        self.pending.clear();
        self.checked.clear();
        self.first_seen.clear();
    }

    /// One frame of the ray pass.
    fn step(&mut self, veh: i32, speed: f32) -> RayStats {
        let t0 = wall_time_ns();
        let mut st = RayStats { pos_err: f32::NAN, ..Default::default() };
        self.frame += 1;
        let frame = self.frame;
        if self.dims_for != veh {
            let model: u32 = unsafe { invoke!(n::GET_ENTITY_MODEL, veh) };
            let (mut lo, mut hi) = (Vector3::default(), Vector3::default());
            let () = unsafe { invoke!(n::GET_MODEL_DIMENSIONS, model, &mut lo as *mut Vector3, &mut hi as *mut Vector3) };
            self.dims = (lo, hi);
            self.dims_for = veh;
        }

        // 1. Results of earlier frames' async probes.
        let mut still = Vec::new();
        for &(h, key, started) in &self.pending {
            let (status, hit, entity) = unsafe { ray_result(h) };
            match status {
                2 => {
                    st.done += 1;
                    self.checked.insert(key, frame);
                    if key > 0 {
                        if !hit || entity == key {
                            st.clear += 1;
                        }
                    } else if hit {
                        st.us_hits += 1;
                    }
                }
                1 if frame - started < 30 => still.push((h, key, started)),
                _ => {}
            }
        }
        self.pending = still;
        let t1 = wall_time_ns();
        st.t_results = t1 - t0;

        // 2. List every vehicle.
        let n = unsafe { shv_sys::worldGetAllVehicles(self.vbuf.as_mut_ptr(), self.vbuf.len() as i32) }.max(0) as usize;
        let list: Vec<i32> = self.vbuf[..n.min(self.vbuf.len())].iter().copied().filter(|&v| v != veh).collect();
        st.vehicles = list.len() as u32;
        let t2 = wall_time_ns();
        st.t_list = t2 - t1;

        // 3. Parked count (sampled).
        if frame % SEAT_EVERY == 1 {
            self.parked = list
                .iter()
                .filter(|&&v| {
                    let driver: i32 = unsafe { invoke!(n::GET_PED_IN_VEHICLE_SEAT, v, -1i32, false) };
                    driver == 0
                })
                .count() as u32;
        }
        st.parked = self.parked;
        let t3 = wall_time_ns();
        st.t_seat = t3 - t2;

        // 4. Positions and fields of view.
        let pos: Vector3 = unsafe { invoke!(n::GET_ENTITY_COORDS, veh, true) };
        let fwd: Vector3 = unsafe { invoke!(n::GET_ENTITY_FORWARD_VECTOR, veh) };
        let flen = (fwd.x * fwd.x + fwd.y * fwd.y).sqrt().max(1e-6);
        let (fx, fy) = (fwd.x / flen, fwd.y / flen);
        let (rx, ry) = (fy, -fx);
        let (lo, hi) = self.dims;
        let corner = |c: (i8, i8)| {
            let x = match c.0 {
                -1 => lo.x,
                1 => hi.x,
                _ => 0.0,
            };
            (x, if c.1 > 0 { hi.y } else { lo.y })
        };
        let use_mem = matches!(self.mode, RayMode::MemAsync | RayMode::MemSync);
        if use_mem && frame % 60 == 1 {
            let mut err = 0.0f32;
            for &v in list.iter().take(3) {
                if let Some((mx, my, mz)) = mem_pos(v) {
                    let p: Vector3 = unsafe { invoke!(n::GET_ENTITY_COORDS, v, true) };
                    err = err.max(((p.x - mx).powi(2) + (p.y - my).powi(2) + (p.z - mz).powi(2)).sqrt());
                }
            }
            st.pos_err = err;
        }
        // (key, target world pos, mount in ego frame) for each target in view.
        let mut targets: Vec<Target> = Vec::new();
        for &v in &list {
            let p = if use_mem {
                match mem_pos(v) {
                    Some(p) => p,
                    None => continue,
                }
            } else {
                let p: Vector3 = unsafe { invoke!(n::GET_ENTITY_COORDS, v, true) };
                (p.x, p.y, p.z)
            };
            let (dx, dy) = (p.0 - pos.x, p.1 - pos.y);
            if dx * dx + dy * dy > 170.0 * 170.0 {
                continue;
            }
            let (x_right, y_fwd) = (dx * rx + dy * ry, dx * fx + dy * fy);
            let mut best: Option<(f32, (f32, f32))> = None;
            for r in &RADARS {
                let (mx, my) = corner(r.corner);
                let (ex, ey) = (x_right - mx, y_fwd - my);
                let range = (ex * ex + ey * ey).sqrt();
                if range > r.range_m || wrap_deg((-ex).atan2(ey).to_degrees() - r.facing_deg).abs() > r.half_fov_deg {
                    continue;
                }
                st.pairs += 1;
                if best.is_none_or(|b| range < b.0) {
                    best = Some((range, (mx, my)));
                }
            }
            if let Some((_, mount)) = best {
                targets.push((v, p, mount));
                self.first_seen.entry(v).or_insert(frame);
            }
        }
        st.targets = targets.len() as u32;
        let t4 = wall_time_ns();
        st.t_pos = t4 - t3;

        // 5. Schedule: the stalest due items first, within the probe budget.
        let age_of = |key: i32, checked: &HashMap<i32, u32>, seen: &HashMap<i32, u32>| {
            frame - checked.get(&key).copied().or_else(|| seen.get(&key).copied()).unwrap_or(frame)
        };
        let in_flight: Vec<i32> = self.pending.iter().map(|p| p.1).collect();
        let mut due: Vec<(f32, i32)> = Vec::new();
        let mut age_sum = 0u64;
        for t in &targets {
            let age = age_of(t.0, &self.checked, &self.first_seen);
            age_sum += age as u64;
            st.age_max = st.age_max.max(age);
            let score = (age as f32 + 1.0) / RADAR_PERIOD_FRAMES;
            if score >= 1.0 && !in_flight.contains(&t.0) {
                due.push((score, t.0));
            }
        }
        st.age_mean = if targets.is_empty() { 0.0 } else { age_sum as f32 / targets.len() as f32 };
        st.us_on = speed < US_MAX_SPEED;
        let us_rays = ultrasonic_rays(lo, hi);
        if st.us_on {
            for i in 0..us_rays.len() {
                let key = -(1 + i as i32);
                let age = frame - self.checked.get(&key).copied().unwrap_or(0);
                let score = (age as f32 + 1.0) / US_PERIOD_FRAMES;
                if score >= 1.0 && !in_flight.contains(&key) {
                    due.push((score, key));
                }
            }
        }
        due.sort_by(|a, b| b.0.total_cmp(&a.0));
        let budget = match self.mode {
            RayMode::MemSync => SYNC_PER_FRAME,
            _ => MAX_IN_FLIGHT.saturating_sub(self.pending.len()),
        };
        due.truncate(budget);
        let t5 = wall_time_ns();
        st.t_sched = t5 - t4;

        // 6. Start the probes.
        let world = |x_right: f32, y_fwd: f32| (pos.x + rx * x_right + fx * y_fwd, pos.y + ry * x_right + fy * y_fwd, pos.z + 0.3);
        for &(_, key) in &due {
            let (from, to, flags) = if key > 0 {
                let t = targets.iter().find(|t| t.0 == key).expect("scheduled target");
                (world(t.2.0, t.2.1), (t.1.0, t.1.1, t.1.2 + 0.3), LOS_FLAGS)
            } else {
                let (sx, sy, ex, ey) = us_rays[(-key - 1) as usize];
                (world(sx, sy), world(ex, ey), US_FLAGS)
            };
            if self.mode == RayMode::MemSync {
                let h: i32 = unsafe {
                    invoke!(n::START_EXPENSIVE_SYNCHRONOUS_SHAPE_TEST_LOS_PROBE, from.0, from.1, from.2, to.0, to.1, to.2, flags, veh, 7i32)
                };
                let (status, hit, entity) = unsafe { ray_result(h) };
                if status == 2 {
                    st.started += 1;
                    st.done += 1;
                    self.checked.insert(key, frame);
                    if key > 0 && (!hit || entity == key) {
                        st.clear += 1;
                    } else if key < 0 && hit {
                        st.us_hits += 1;
                    }
                } else {
                    st.failed += 1;
                }
            } else {
                let h: i32 = unsafe { invoke!(n::START_SHAPE_TEST_LOS_PROBE, from.0, from.1, from.2, to.0, to.1, to.2, flags, veh, 7i32) };
                if h == 0 {
                    st.failed += 1;
                } else {
                    st.started += 1;
                    self.pending.push((h, key, frame));
                }
            }
        }
        let t6 = wall_time_ns();
        st.t_probe = t6 - t5;

        // Forget targets not seen for 10 s.
        if frame.is_multiple_of(300) {
            self.first_seen.retain(|_, f| frame - *f < 720);
            self.checked.retain(|k, f| *k < 0 || frame - *f < 720);
        }
        st.ns = t6 - t0;
        st
    }
}

/// Ultrasonic rays in the ego frame: (start x, start y, end x, end y).
fn ultrasonic_rays(lo: Vector3, hi: Vector3) -> Vec<(f32, f32, f32, f32)> {
    let mut out = Vec::with_capacity(2 * US_PER_BUMPER * US_RAYS.len());
    for bumper in [1i8, -1] {
        let y = if bumper > 0 { hi.y } else { lo.y };
        let base_deg = if bumper > 0 { 0.0f32 } else { 180.0 };
        for i in 0..US_PER_BUMPER {
            let f = i as f32 / (US_PER_BUMPER - 1) as f32;
            let x = lo.x + (hi.x - lo.x) * f;
            // Outer sensors angle outwards, like real park sensors.
            let splay = (f - 0.5) * -90.0 * bumper as f32;
            for &ray in &US_RAYS {
                let a = (base_deg + splay + ray).to_radians();
                out.push((x, y, x - a.sin() * US_RANGE_M, y + a.cos() * US_RANGE_M));
            }
        }
    }
    out
}

// ---- The spike ----

fn run() {
    let dir = log_dir();
    let stem = run_stem();
    let mut log = BufWriter::new(File::create(dir.join(format!("{stem}.log"))).expect("create log"));
    let mut csv = BufWriter::new(File::create(dir.join(format!("{stem}-frames.csv"))).expect("create csv"));
    let _ = writeln!(
        csv,
        "frame,wall_ms,frame_ms,cap60,seq,rtt_us,reply_in_frame,cmd_age,engaged,speed,thr,brk,hold,target,reference,\
         ai,ray_mode,vehicles,parked,pairs,targets,started,failed,done,clear,us_on,us_hits,age_mean,age_max,t_results_us,t_list_us,t_seat_us,t_pos_us,t_sched_us,t_probe_us,ray_us,pos_err,pending"
    );

    let link = match SharedLink::open(DEFAULT_NAME) {
        Ok(l) => l,
        Err(e) => {
            let _ = writeln!(log, "LINK open failed: {e}");
            let _ = log.flush();
            return;
        }
    };
    let _ = writeln!(log, "S3/S6 spike started. Link {DEFAULT_NAME}, contract v{CONTRACT_VERSION}. F6 cap60, F7 teleport, F8 sultan, F9 AI drive, F10 engage, F11 rays");
    let _ = log.flush();

    let mut keys = Keys { prev: [false; 6] };
    let mut cap60 = false;
    let mut tp_idx = 0usize;
    let mut test_car = 0i32;
    let mut ai = false;
    let mut engage_req = false;
    let mut rays = RaySystem::new();
    let mut was_active = false;
    let mut takeover_until = 0u64;
    let mut seen_epoch = link.epoch.load(Ordering::Acquire);
    let mut ratio = 0.611f32;

    let mut seq = 0u32;
    let mut game_time_us = 0u64;
    let mut last_tick = wall_time_ns();
    let mut win_start = last_tick;
    let mut win_frames = 0u32;
    let mut win_rtt: Vec<u64> = Vec::new();
    let mut win_miss = 0u32;
    let mut win_ray: Vec<u64> = Vec::new();
    let mut win_rays_started = 0u64;
    let mut win_age: Vec<u64> = Vec::new();
    let mut win_parts = [0u64; 6];
    let mut hud_rtt = String::from("no core yet");
    let mut hud_rays = String::from("rays off");

    loop {
        // --- 60 fps cap (S3 at 60 vs 144 fps) ---
        if cap60 {
            while wall_time_ns() < last_tick + CAP_60_NS {
                core::hint::spin_loop();
            }
        }
        let now = wall_time_ns();
        let frame_ns = now - last_tick;
        last_tick = now;

        let ped: i32 = unsafe { invoke!(n::PLAYER_PED_ID) };
        let in_vehicle: bool = unsafe { invoke!(n::IS_PED_IN_ANY_VEHICLE, ped, false) };
        let veh: i32 = if in_vehicle { unsafe { invoke!(n::GET_VEHICLE_PED_IS_IN, ped, false) } } else { 0 };
        let base = if in_vehicle { unsafe { shv_sys::getScriptHandleBaseAddress(veh) as usize } } else { 0 };
        let dt: f32 = unsafe { invoke!(n::GET_FRAME_TIME) };
        let frame: i32 = unsafe { invoke!(n::GET_FRAME_COUNT) };
        let pause_menu: bool = unsafe { invoke!(n::IS_PAUSE_MENU_ACTIVE) };
        let loading: bool = unsafe { invoke!(n::GET_IS_LOADING_SCREEN_ACTIVE) };
        let paused = pause_menu || loading;
        game_time_us += (dt.max(0.0) as f64 * 1e6) as u64;

        // --- Keys ---
        if keys.pressed(0, VK_F6) {
            cap60 = !cap60;
            let _ = writeln!(log, "frame {frame}: 60 fps cap {}", if cap60 { "ON" } else { "OFF" });
        }
        if keys.pressed(1, VK_F7) {
            teleport(ped, veh, tp_idx, &mut log);
            tp_idx += 1;
            last_tick = wall_time_ns();
        }
        if keys.pressed(2, VK_F8) {
            spawn_sultan(ped, &mut test_car, &mut log);
            last_tick = wall_time_ns();
        }
        if keys.pressed(3, VK_F9) && in_vehicle {
            ai = !ai;
            if ai {
                engage_req = false;
                let () = unsafe { invoke!(n::TASK_VEHICLE_DRIVE_WANDER, ped, veh, AI_SPEED_MPS, AI_DRIVING_STYLE) };
            } else {
                let () = unsafe { invoke!(n::CLEAR_PED_TASKS, ped) };
            }
            let _ = writeln!(log, "frame {frame}: AI drive {}", if ai { "ON" } else { "OFF" });
        }
        if keys.pressed(4, VK_F10) && in_vehicle {
            engage_req = !engage_req;
            if engage_req && ai {
                ai = false;
                let () = unsafe { invoke!(n::CLEAR_PED_TASKS, ped) };
            }
            let _ = writeln!(log, "frame {frame}: engage request {}", if engage_req { "ON" } else { "OFF" });
        }
        if keys.pressed(5, VK_F11) {
            let m = rays.mode.next();
            rays.set_mode(m);
            let _ = writeln!(log, "frame {frame}: ray pass {}", m.name());
        }
        if !in_vehicle {
            engage_req = false;
        }

        // --- S3: sample → core ---
        let mut s = VehicleSample::default();
        if in_vehicle {
            let speed: f32 = unsafe { invoke!(n::GET_ENTITY_SPEED, veh) };
            let vel: Vector3 = unsafe { invoke!(n::GET_ENTITY_SPEED_VECTOR, veh, true) };
            let rot: Vector3 = unsafe { invoke!(n::GET_ENTITY_ROTATION_VELOCITY, veh) };
            s.speed = speed;
            s.vx = vel.y; // GTA: y forward, x right → ISO: x forward, y left
            s.vy = -vel.x;
            s.yaw_rate = rot.z;
            let r_in = read_f32(base + STEER_INPUT_OFFSET).unwrap_or(f32::NAN);
            s.steer_angle = read_f32(base + STEER_ANGLE_OFFSET).unwrap_or(f32::NAN);
            if r_in.abs() > 0.05 && s.steer_angle.is_finite() {
                ratio += 0.1 * (s.steer_angle / r_in - ratio);
            }
            s.wheel_omega = wheel_omegas(base);
            s.rev_ratio = unsafe { invoke!(n::GET_VEHICLE_CURRENT_REV_RATIO, veh) };
            s.gear = unsafe { invoke!(n::GET_VEHICLE_CURRENT_DRIVE_GEAR, veh) };
        }
        s.flags = if paused { sample_flags::PAUSED } else { 0 } | if engage_req { sample_flags::ENGAGE_REQUEST } else { 0 };
        seq = seq.wrapping_add(1);
        s.header = Header {
            kind: MsgKind::VehicleSample,
            version: CONTRACT_VERSION,
            seq,
            frame: frame as u64,
            game_time_us,
            wall_time_ns: wall_time_ns(),
        };
        link.sample.write(&s);
        link.adapter_frame.store(frame as u64, Ordering::Release);
        link.signal_sample();
        let t_written = s.header.wall_time_ns;

        // Reactive mode: wait (briefly) for the answer to this very sample, but
        // only while a core is around, so a missing core costs nothing.
        let (_, mut cmd) = link.command.read();
        let core_present = link.epoch.load(Ordering::Acquire) > 0 && cmd.answers_seq != 0 && seq.wrapping_sub(cmd.answers_seq) < 120;
        let mut rtt_ns: Option<u64> = None;
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
        if let Some(r) = rtt_ns {
            win_rtt.push(r);
        }

        // --- Apply, with the watchdog (D24) ---
        let epoch = link.epoch.load(Ordering::Acquire);
        if epoch != seen_epoch {
            if engage_req || was_active {
                let _ = writeln!(log, "frame {frame}: core epoch {seen_epoch} -> {epoch}: disengaged (a new core starts disengaged)");
            }
            seen_epoch = epoch;
            engage_req = false;
        }
        let age = seq.wrapping_sub(cmd.answers_seq);
        let fresh = cmd.answers_seq != 0 && age <= WATCHDOG_FRAMES;
        let core_engaged = cmd.flags & command_flags::ENGAGED != 0;
        let active = in_vehicle && engage_req && core_engaged && fresh && !paused;
        if engage_req && was_active && !fresh {
            let _ = writeln!(log, "frame {frame}: WATCHDOG: no fresh command for {age} frames -> released, TAKE OVER");
            engage_req = false;
            takeover_until = wall_time_ns() + 3_000_000_000;
        }
        if active {
            write_f32(base + STEER_INPUT_OFFSET, cmd.steer_angle / ratio.max(0.1));
            unsafe {
                if cmd.throttle > 0.0 {
                    let v = PEDAL_DEADZONE + (1.0 - PEDAL_DEADZONE) * cmd.throttle.min(1.0);
                    let _: bool = invoke!(n::SET_CONTROL_VALUE_NEXT_FRAME, PLAYER_CONTROL, INPUT_VEH_ACCELERATE, v);
                }
                if cmd.brake > 0.0 {
                    let v = PEDAL_DEADZONE + (1.0 - PEDAL_DEADZONE) * cmd.brake.min(1.0);
                    let _: bool = invoke!(n::SET_CONTROL_VALUE_NEXT_FRAME, PLAYER_CONTROL, INPUT_VEH_BRAKE, v);
                }
                let () = invoke!(n::SET_VEHICLE_HANDBRAKE, veh, cmd.flags & command_flags::HOLD != 0);
            }
        } else if was_active && in_vehicle {
            // Release everything at once: the player has the car.
            let () = unsafe { invoke!(n::SET_VEHICLE_HANDBRAKE, veh, false) };
        }
        was_active = active;

        // --- S6: ray pass ---
        let rs = if rays.mode != RayMode::Off && in_vehicle && !paused {
            let st = rays.step(veh, s.speed);
            win_ray.push(st.ns);
            win_rays_started += st.started as u64;
            win_age.push(st.age_max as u64);
            for (acc, v) in win_parts.iter_mut().zip([st.t_results, st.t_list, st.t_seat, st.t_pos, st.t_sched, st.t_probe]) {
                *acc += v;
            }
            if !st.pos_err.is_nan() && st.pos_err > 0.05 {
                let _ = writeln!(log, "frame {frame}: memory position differs from native by {:.3} m", st.pos_err);
            }
            st
        } else {
            RayStats { pos_err: f32::NAN, ..Default::default() }
        };

        // --- Log ---
        let wall_ms = (now - win_start) as f64 / 1e6;
        let _ = writeln!(
            csv,
            "{frame},{wall_ms:.3},{:.3},{},{seq},{},{},{age},{},{:.3},{:.3},{:.3},{},{:.2},{:.3},{},{},{},{},{},{},{},{},{},{},{},{},{:.2},{},{:.1},{:.1},{:.1},{:.1},{:.1},{:.1},{:.1},{:.3},{}",
            frame_ns as f64 / 1e6,
            cap60 as u8,
            rtt_ns.map(|r| format!("{:.1}", r as f64 / 1e3)).unwrap_or("-1".into()),
            rtt_ns.is_some() as u8,
            active as u8,
            s.speed,
            if active { cmd.throttle } else { 0.0 },
            if active { cmd.brake } else { 0.0 },
            (active && cmd.flags & command_flags::HOLD != 0) as u8,
            cmd.diag[0],
            cmd.diag[1],
            ai as u8,
            rays.mode.name(),
            rs.vehicles,
            rs.parked,
            rs.pairs,
            rs.targets,
            rs.started,
            rs.failed,
            rs.done,
            rs.clear,
            rs.us_on as u8,
            rs.us_hits,
            rs.age_mean,
            rs.age_max,
            rs.t_results as f64 / 1e3,
            rs.t_list as f64 / 1e3,
            rs.t_seat as f64 / 1e3,
            rs.t_pos as f64 / 1e3,
            rs.t_sched as f64 / 1e3,
            rs.t_probe as f64 / 1e3,
            rs.ns as f64 / 1e3,
            rs.pos_err,
            rays.pending.len()
        );
        win_frames += 1;
        if now - win_start >= 1_000_000_000 {
            let secs = (now - win_start) as f64 / 1e9;
            win_rtt.sort_unstable();
            win_ray.sort_unstable();
            let rtt = if win_rtt.is_empty() {
                "no core".to_string()
            } else {
                format!(
                    "rtt p50 {:.0} us p99 {:.0} us max {:.0} us, {} misses",
                    pct(&win_rtt, 0.5) as f64 / 1e3,
                    pct(&win_rtt, 0.99) as f64 / 1e3,
                    *win_rtt.last().unwrap() as f64 / 1e3,
                    win_miss
                )
            };
            let rays = if win_ray.is_empty() {
                "rays off".to_string()
            } else {
                win_age.sort_unstable();
                let k = win_ray.len() as f64 * 1e3;
                format!(
                    "rays {} {:.1}/frame, pass p50 {:.0} us p99 {:.0} us max {:.0} us \
                     [mean us: results {:.0} list {:.0} seat {:.0} pos {:.0} sched {:.0} probe {:.0}], \
                     LOS age max p50 {} p99 {} frames, targets {}",
                    rays.mode.name(),
                    win_rays_started as f64 / win_ray.len() as f64,
                    pct(&win_ray, 0.5) as f64 / 1e3,
                    pct(&win_ray, 0.99) as f64 / 1e3,
                    *win_ray.last().unwrap() as f64 / 1e3,
                    win_parts[0] as f64 / k,
                    win_parts[1] as f64 / k,
                    win_parts[2] as f64 / k,
                    win_parts[3] as f64 / k,
                    win_parts[4] as f64 / k,
                    win_parts[5] as f64 / k,
                    pct(&win_age, 0.5),
                    pct(&win_age, 0.99),
                    rs.targets
                )
            };
            let _ = writeln!(
                log,
                "[frame {frame}] {:.0} fps{} | {rtt} | {rays} | vehicles {} parked {} | {}",
                win_frames as f64 / secs,
                if cap60 { " (cap 60)" } else { "" },
                rs.vehicles,
                rs.parked,
                if active { "ENGAGED" } else if ai { "AI driving" } else { "manual" }
            );
            hud_rtt = rtt;
            hud_rays = format!("{rays} | veh {} parked {}", rs.vehicles, rs.parked);
            win_start = now;
            win_frames = 0;
            win_rtt.clear();
            win_miss = 0;
            win_ray.clear();
            win_rays_started = 0;
            win_age.clear();
            win_parts = [0; 6];
            let _ = log.flush();
            let _ = csv.flush();
        }

        // --- HUD ---
        unsafe {
            let () = invoke!(n::DRAW_RECT, 0.25f32, 0.065f32, 0.48f32, 0.11f32, 0i32, 0i32, 0i32, 160i32, false);
            let state = if wall_time_ns() < takeover_until {
                "TAKE OVER"
            } else if active {
                "ENGAGED"
            } else if engage_req {
                "engage requested"
            } else if ai {
                "AI driving"
            } else {
                "manual"
            };
            draw_text(0.015, 0.012, &format!("ADAS S3/S6 | {:.1} km/h | {state} | epoch {seen_epoch} | {hud_rtt}", s.speed * 3.6));
            draw_text(0.015, 0.037, &hud_rays);
            draw_text(0.015, 0.062, &format!("F6 cap60 {} | F7 teleport | F8 sultan | F9 AI | F10 engage | F11 rays", if cap60 { "ON" } else { "off" }));
        }

        unsafe { shv_sys::scriptWait(0) };
    }
}
