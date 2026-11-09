//! Spike S1: a Rust `.asi` on Script Hook V for GTA V Enhanced.
//!
//! Throwaway code. It answers:
//! - Does a Rust plugin load, call natives and draw text without crashing (10 min)?
//! - How do GTA's game timer and frame time behave in pause and slow motion?
//! - Can a shape test (raycast) return in the same script tick, or always later?
//! - What does our per-frame work cost on the game thread?
//!
//! Keys: F7 cycles the time scale (1.0 → 0.5 → 0.2), F8 writes a MARK to the log.
//! Soak-test tooling: F9 toggles soak mode (GTA's AI drives the player's car
//! around the city; weather and clock change every minute), F10 cycles the
//! weather, F11 adds 6 hours to the clock.
//! Output, in `<game folder>/adas-logs/`: `s1-<unix time>.log` (summary per
//! second, events) and `s1-<unix time>-frames.csv` (one row per frame).

use core::ffi::c_void;
use std::ffi::CString;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::panic::{AssertUnwindSafe, catch_unwind};
use std::path::PathBuf;
use std::sync::atomic::{AtomicPtr, Ordering};

use shv_sys::{Vector3, invoke};

// ---- Win32 (declared directly; only four functions are needed) ----

#[link(name = "kernel32", kind = "raw-dylib")]
unsafe extern "system" {
    fn DisableThreadLibraryCalls(module: *mut c_void) -> i32;
    fn QueryPerformanceCounter(count: *mut i64) -> i32;
    fn QueryPerformanceFrequency(freq: *mut i64) -> i32;
    fn GetModuleFileNameW(module: *mut c_void, buf: *mut u16, len: u32) -> u32;
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

/// Soak mode: GTA's AI drives the player's car around the city (test tooling,
/// not the autopilot), and the weather and clock change once a minute.
const SOAK_SPEED_MPS: f32 = 20.0;
/// Normal driving style: stops for traffic and lights, avoids vehicles and peds.
const SOAK_DRIVING_STYLE: i32 = 786_603;
const SOAK_CHANGE_EVERY_NS: u64 = 60_000_000_000;
const WEATHERS: [&core::ffi::CStr; 6] = [c"CLEAR", c"RAIN", c"THUNDER", c"FOGGY", c"OVERCAST", c"EXTRASUNNY"];

// ---- Natives (hashes checked against alloc8or/gta5-nativedb-data) ----

mod n {
    pub const PLAYER_PED_ID: u64 = 0xD80958FC74E988A6;
    pub const IS_PED_IN_ANY_VEHICLE: u64 = 0x997ABD671D25CA0B;
    pub const GET_VEHICLE_PED_IS_IN: u64 = 0x9A9112A0FE9A4713;
    pub const GET_ENTITY_SPEED: u64 = 0xD5037BA82E12416F;
    pub const GET_ENTITY_COORDS: u64 = 0x3FEF770D40960D5A;
    pub const GET_ENTITY_FORWARD_VECTOR: u64 = 0x0A794A5A57F8DF91;
    pub const GET_GAME_TIMER: u64 = 0x9CD27B0045628463;
    pub const GET_FRAME_TIME: u64 = 0x15C40837039FFAF7;
    pub const GET_FRAME_COUNT: u64 = 0xFC8202EFC642E6F2;
    pub const SET_TIME_SCALE: u64 = 0x1D408577D440E81E;
    pub const IS_PAUSE_MENU_ACTIVE: u64 = 0xB0034A223497FFCB;
    pub const GET_IS_LOADING_SCREEN_ACTIVE: u64 = 0x10D0A8F259E93EC9;
    pub const IS_CUTSCENE_PLAYING: u64 = 0xD3C2E180A40F031E;
    pub const BEGIN_TEXT_COMMAND_DISPLAY_TEXT: u64 = 0x25FBB336DF1804CB;
    pub const ADD_TEXT_COMPONENT_SUBSTRING_PLAYER_NAME: u64 = 0x6C188BE134E074AA;
    pub const END_TEXT_COMMAND_DISPLAY_TEXT: u64 = 0xCD015E5BB0D96A57;
    pub const SET_TEXT_SCALE: u64 = 0x07C837F9A01C34C9;
    pub const SET_TEXT_FONT: u64 = 0x66E0276CC5F6B9DA;
    pub const SET_TEXT_COLOUR: u64 = 0xBE6B23FFA53FB442;
    pub const SET_TEXT_OUTLINE: u64 = 0x2513DFB0FB8400FE;
    pub const DRAW_RECT: u64 = 0x3A618A217E5154F0;
    pub const START_SHAPE_TEST_LOS_PROBE: u64 = 0x7EE9F5D83DD4F90E;
    pub const START_EXPENSIVE_SYNCHRONOUS_SHAPE_TEST_LOS_PROBE: u64 = 0x377906D8A31E5586;
    pub const GET_SHAPE_TEST_RESULT: u64 = 0x3D87450E15D98694;
    pub const TASK_VEHICLE_DRIVE_WANDER: u64 = 0x480142959D337D00;
    pub const CLEAR_PED_TASKS: u64 = 0xE1EF3C1216AFF2CD;
    pub const SET_VEHICLE_FIXED: u64 = 0x115722B1B9C14C1C;
    pub const SET_WEATHER_TYPE_NOW_PERSIST: u64 = 0xED712CA327900C8A;
    pub const ADD_TO_CLOCK_TIME: u64 = 0xD716F30D8C8980E2;
    pub const GET_CLOCK_HOURS: u64 = 0x25223CA6B4D20B7F;
}

/// Shape-test flags: everything (world, vehicles, peds, objects, …).
const RAY_FLAGS: i32 = 511;
const RAY_LENGTH_M: f32 = 50.0;

// ---- DLL entry ----

static MODULE: AtomicPtr<c_void> = AtomicPtr::new(core::ptr::null_mut());

/// # Safety
/// Called only by the Windows loader.
#[unsafe(no_mangle)]
pub unsafe extern "system" fn DllMain(module: *mut c_void, reason: u32, _reserved: *mut c_void) -> i32 {
    const DLL_PROCESS_DETACH: u32 = 0;
    const DLL_PROCESS_ATTACH: u32 = 1;
    match reason {
        DLL_PROCESS_ATTACH => {
            MODULE.store(module, Ordering::Relaxed);
            unsafe {
                DisableThreadLibraryCalls(module);
                shv_sys::scriptRegister(module, script_main);
            }
        }
        DLL_PROCESS_DETACH => unsafe { shv_sys::scriptUnregister(module) },
        _ => {}
    }
    1
}

extern "C" fn script_main() {
    // No panic may cross into the game. If the spike panics, log it and idle.
    let result = catch_unwind(AssertUnwindSafe(run));
    if let Err(payload) = result {
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

/// `<game folder>/adas-logs`. GTA opens and locks files that already exist in
/// its folder at startup, so every run writes new, uniquely named files here
/// instead of overwriting old ones.
fn log_dir() -> PathBuf {
    let mut buf = [0u16; 1024];
    let len = unsafe { GetModuleFileNameW(MODULE.load(Ordering::Relaxed), buf.as_mut_ptr(), buf.len() as u32) };
    let path = PathBuf::from(String::from_utf16_lossy(&buf[..len as usize]));
    let dir = path.parent().map(|p| p.join("adas-logs")).unwrap_or_else(|| PathBuf::from("adas-logs"));
    let _ = std::fs::create_dir_all(&dir);
    dir
}

/// Unique per-run file stem, e.g. `s1-1762651234`.
fn run_stem() -> String {
    let secs = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);
    format!("s1-{secs}")
}

struct Clock {
    freq: i64,
}

impl Clock {
    fn new() -> Self {
        let mut freq = 0;
        unsafe { QueryPerformanceFrequency(&mut freq) };
        Self { freq }
    }
    fn now_ns(&self) -> u64 {
        let mut c = 0;
        unsafe { QueryPerformanceCounter(&mut c) };
        ((c as i128) * 1_000_000_000 / self.freq as i128) as u64
    }
}

struct Keys {
    prev: [bool; 5],
}

impl Keys {
    /// Returns true on the frame a key goes down.
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

#[derive(Clone, Copy)]
enum Ray {
    Idle,
    Pending { handle: i32, started_frame: i32 },
}

/// Polls a shape test. Returns 0 = no such test, 1 = pending, 2 = ready.
unsafe fn ray_status(handle: i32) -> i32 {
    let mut hit = 0i32;
    let mut end = Vector3::default();
    let mut normal = Vector3::default();
    let mut entity = 0i32;
    unsafe {
        invoke!(
            n::GET_SHAPE_TEST_RESULT,
            handle,
            &mut hit as *mut i32,
            &mut end as *mut Vector3,
            &mut normal as *mut Vector3,
            &mut entity as *mut i32,
        )
    }
}

fn percentile(sorted: &[u64], p: f64) -> u64 {
    if sorted.is_empty() {
        return 0;
    }
    let i = ((sorted.len() - 1) as f64 * p).round() as usize;
    sorted[i]
}

// ---- The spike ----

fn run() {
    let clock = Clock::new();
    let dir = log_dir();
    let stem = run_stem();
    let mut log = BufWriter::new(File::create(dir.join(format!("{stem}.log"))).expect("create log"));
    let mut csv = BufWriter::new(File::create(dir.join(format!("{stem}-frames.csv"))).expect("create csv"));

    let version = unsafe { shv_sys::getGameVersion() };
    let _ = writeln!(log, "S1 spike started. Script Hook V eGameVersion = {version}");
    let _ = writeln!(log, "Keys: F7 = cycle time scale, F8 = MARK");
    let _ = writeln!(
        csv,
        "frame,qpc_ns,game_timer_ms,frame_time_s,acc_game_us,paused,loading,cutscene,time_scale,in_vehicle,speed_mps,tick_us"
    );
    let _ = log.flush();

    let start_ns = clock.now_ns();
    let mut keys = Keys { prev: [false; 5] };
    let mut soak = false;
    let mut soak_last_change_ns = 0u64;
    let mut weather_index = 0usize;
    let time_scales = [1.0f32, 0.5, 0.2];
    let mut ts_index = 0usize;

    let mut acc_game_us: u64 = 0;
    let mut prev_paused = false;
    let mut prev_loading = false;
    let mut prev_cutscene = false;

    // Per-second window.
    let mut window_start_ns = start_ns;
    let mut window_frames = 0u32;
    let mut window_timer_start: i32 = unsafe { invoke!(n::GET_GAME_TIMER) };
    let mut window_acc_start = acc_game_us;
    let mut window_ticks: Vec<u64> = Vec::with_capacity(512);

    // Raycast experiment.
    let mut ray = Ray::Idle;
    let mut ray_same_tick = 0u32;
    let mut ray_latency_hist = [0u32; 6]; // frames until ready: 1, 2, 3, 4, 5, 6+
    let mut ray_lost = 0u32;
    let mut last_hud_ray = String::new();

    loop {
        let tick_start = clock.now_ns();

        // --- Sample ---
        let frame: i32 = unsafe { invoke!(n::GET_FRAME_COUNT) };
        let game_timer: i32 = unsafe { invoke!(n::GET_GAME_TIMER) };
        let frame_time: f32 = unsafe { invoke!(n::GET_FRAME_TIME) };
        let paused: bool = unsafe { invoke!(n::IS_PAUSE_MENU_ACTIVE) };
        let loading: bool = unsafe { invoke!(n::GET_IS_LOADING_SCREEN_ACTIVE) };
        let cutscene: bool = unsafe { invoke!(n::IS_CUTSCENE_PLAYING) };
        let ped: i32 = unsafe { invoke!(n::PLAYER_PED_ID) };
        let in_vehicle: bool = unsafe { invoke!(n::IS_PED_IN_ANY_VEHICLE, ped, false) };
        let entity: i32 = if in_vehicle { unsafe { invoke!(n::GET_VEHICLE_PED_IS_IN, ped, false) } } else { ped };
        let speed: f32 = unsafe { invoke!(n::GET_ENTITY_SPEED, entity) };
        acc_game_us += (frame_time as f64 * 1e6) as u64;

        let uptime_s = (tick_start - start_ns) as f64 / 1e9;

        // --- Events ---
        for (now, prev, name) in [
            (paused, &mut prev_paused, "pause menu"),
            (loading, &mut prev_loading, "loading screen"),
            (cutscene, &mut prev_cutscene, "cutscene"),
        ] {
            if now != *prev {
                let _ = writeln!(
                    log,
                    "[{uptime_s:9.3}s] EVENT {name} {} at frame {frame}, game_timer {game_timer} ms, acc_game {} ms",
                    if now { "ON" } else { "OFF" },
                    acc_game_us / 1000
                );
                *prev = now;
            }
        }
        if keys.pressed(0, VK_F7) {
            ts_index = (ts_index + 1) % time_scales.len();
            let _ = writeln!(
                log,
                "[{uptime_s:9.3}s] EVENT time scale -> {} at frame {frame}, game_timer {game_timer} ms",
                time_scales[ts_index]
            );
            if ts_index == 0 {
                let () = unsafe { invoke!(n::SET_TIME_SCALE, 1.0f32) };
            }
        }
        if keys.pressed(1, VK_F8) {
            let _ = writeln!(log, "[{uptime_s:9.3}s] MARK at frame {frame}, game_timer {game_timer} ms");
        }
        // --- Soak-test tooling: F9 AI drive + auto weather/clock, F10 weather, F11 +6 h ---
        let mut next_weather = keys.pressed(3, VK_F10);
        let mut add_hours = if keys.pressed(4, VK_F11) { 6 } else { 0 };
        if keys.pressed(2, VK_F9) {
            soak = !soak;
            if soak && in_vehicle {
                let () = unsafe { invoke!(n::SET_VEHICLE_FIXED, entity) };
                let () = unsafe { invoke!(n::TASK_VEHICLE_DRIVE_WANDER, ped, entity, SOAK_SPEED_MPS, SOAK_DRIVING_STYLE) };
                soak_last_change_ns = tick_start;
            } else {
                soak = false;
                let () = unsafe { invoke!(n::CLEAR_PED_TASKS, ped) };
            }
            let _ = writeln!(
                log,
                "[{uptime_s:9.3}s] EVENT soak {} at frame {frame} (in_vehicle {in_vehicle})",
                if soak { "ON" } else { "OFF" }
            );
        }
        if soak && tick_start - soak_last_change_ns >= SOAK_CHANGE_EVERY_NS {
            soak_last_change_ns = tick_start;
            next_weather = true;
            add_hours = 3;
        }
        if next_weather {
            weather_index = (weather_index + 1) % WEATHERS.len();
            let () = unsafe { invoke!(n::SET_WEATHER_TYPE_NOW_PERSIST, WEATHERS[weather_index]) };
            let _ = writeln!(log, "[{uptime_s:9.3}s] EVENT weather -> {:?} at frame {frame}", WEATHERS[weather_index]);
        }
        if add_hours > 0 {
            let () = unsafe { invoke!(n::ADD_TO_CLOCK_TIME, add_hours, 0i32, 0i32) };
            let hours: i32 = unsafe { invoke!(n::GET_CLOCK_HOURS) };
            let _ = writeln!(log, "[{uptime_s:9.3}s] EVENT clock +{add_hours} h -> {hours}:00 at frame {frame}");
        }

        let time_scale = time_scales[ts_index];
        if ts_index != 0 {
            // Re-applied every frame in case the game resets it.
            let () = unsafe { invoke!(n::SET_TIME_SCALE, time_scale) };
        }

        // --- Raycast experiment: does an async probe ever return in the same tick? ---
        let pos: Vector3 = unsafe { invoke!(n::GET_ENTITY_COORDS, entity, true) };
        let fwd: Vector3 = unsafe { invoke!(n::GET_ENTITY_FORWARD_VECTOR, entity) };
        let (x1, y1, z1) = (pos.x, pos.y, pos.z + 0.5);
        let (x2, y2, z2) = (x1 + fwd.x * RAY_LENGTH_M, y1 + fwd.y * RAY_LENGTH_M, z1 + fwd.z * RAY_LENGTH_M);

        ray = match ray {
            Ray::Idle => {
                let handle: i32 = unsafe {
                    invoke!(n::START_SHAPE_TEST_LOS_PROBE, x1, y1, z1, x2, y2, z2, RAY_FLAGS, entity, 7i32)
                };
                match unsafe { ray_status(handle) } {
                    2 => {
                        ray_same_tick += 1;
                        Ray::Idle
                    }
                    0 => {
                        ray_lost += 1;
                        Ray::Idle
                    }
                    _ => Ray::Pending { handle, started_frame: frame },
                }
            }
            Ray::Pending { handle, started_frame } => match unsafe { ray_status(handle) } {
                2 => {
                    let frames = (frame - started_frame).clamp(1, 6) as usize;
                    ray_latency_hist[frames - 1] += 1;
                    Ray::Idle
                }
                0 => {
                    ray_lost += 1;
                    Ray::Idle
                }
                _ => ray,
            },
        };

        // --- Per-frame CSV ---
        let tick_ns = clock.now_ns() - tick_start;
        window_ticks.push(tick_ns / 1000);
        window_frames += 1;
        let _ = writeln!(
            csv,
            "{frame},{tick_start},{game_timer},{frame_time},{acc_game_us},{},{},{},{time_scale},{},{speed:.3},{}",
            paused as u8,
            loading as u8,
            cutscene as u8,
            in_vehicle as u8,
            tick_ns / 1000
        );

        // --- Once per second: summary, plus one synchronous ray for its cost ---
        if tick_start - window_start_ns >= 1_000_000_000 {
            let t0 = clock.now_ns();
            let handle: i32 = unsafe {
                invoke!(n::START_EXPENSIVE_SYNCHRONOUS_SHAPE_TEST_LOS_PROBE, x1, y1, z1, x2, y2, z2, RAY_FLAGS, entity, 7i32)
            };
            let sync_status = unsafe { ray_status(handle) };
            let sync_cost_us = (clock.now_ns() - t0) / 1000;

            window_ticks.sort_unstable();
            let d_qpc_ms = (tick_start - window_start_ns) as f64 / 1e6;
            let d_timer_ms = game_timer - window_timer_start;
            let d_acc_ms = (acc_game_us - window_acc_start) as f64 / 1000.0;
            let _ = writeln!(
                log,
                "[{uptime_s:9.3}s] frames={window_frames} d_wall={d_qpc_ms:.1}ms d_game_timer={d_timer_ms}ms \
                 d_frame_time_sum={d_acc_ms:.1}ms paused={paused} loading={loading} ts={time_scale} \
                 speed={:.1}km/h tick_p50={}us tick_p99={}us tick_max={}us \
                 ray_same_tick={ray_same_tick} ray_frames={ray_latency_hist:?} ray_lost={ray_lost} \
                 sync_ray={sync_cost_us}us(status {sync_status})",
                speed * 3.6,
                percentile(&window_ticks, 0.5),
                percentile(&window_ticks, 0.99),
                window_ticks.last().copied().unwrap_or(0),
            );
            let _ = log.flush();
            let _ = csv.flush();
            last_hud_ray = format!(
                "ray same-tick {ray_same_tick} | after 1/2/3+ frames {}/{}/{} | sync {sync_cost_us} us",
                ray_latency_hist[0],
                ray_latency_hist[1],
                ray_latency_hist[2..].iter().sum::<u32>()
            );
            window_start_ns = tick_start;
            window_frames = 0;
            window_timer_start = game_timer;
            window_acc_start = acc_game_us;
            window_ticks.clear();
        }

        // --- HUD ---
        unsafe {
            let () = invoke!(n::DRAW_RECT, 0.22f32, 0.075f32, 0.42f32, 0.13f32, 0i32, 0i32, 0i32, 160i32, false);
            draw_text(0.015, 0.015, &format!("ADAS S1  |  up {uptime_s:.0} s  |  {:.1} km/h", speed * 3.6));
            draw_text(
                0.015,
                0.040,
                &format!("frame {frame}  |  timer {game_timer} ms  |  dt {:.2} ms  |  ts {time_scale}", frame_time * 1000.0),
            );
            draw_text(0.015, 0.065, &format!("paused {paused}  |  tick {} us", tick_ns / 1000));
            draw_text(0.015, 0.090, &last_hud_ray);
            draw_text(
                0.015,
                0.115,
                if soak { "SOAK ON  |  F9 stop  |  F10 weather  |  F11 +6 h" } else { "F7 time scale  |  F8 mark  |  F9 soak  |  F10 weather  |  F11 +6 h" },
            );
        }

        unsafe { shv_sys::scriptWait(0) };
    }
}
