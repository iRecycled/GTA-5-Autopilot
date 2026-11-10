//! Test tooling: spawn a test car, teleport to test spots (snapped to a road
//! when the spot is on the road network), hand the car to GTA's AI.

use std::io::Write;
use std::path::PathBuf;

use shv_sys::{Vector3, invoke};

use crate::n;

/// (name, x, y, z, heading, snap to the nearest main-road node)
pub const SPOTS: &[(&str, f32, f32, f32, f32, bool)] = &[
    ("airport taxiway", -1336.0, -3044.0, 13.9, 60.0, false),
    ("downtown road", 215.0, -880.0, 30.5, 0.0, true),
    ("Del Perro freeway", -1150.0, -750.0, 20.0, 0.0, true),
    // A two-lane country road east of Sandy Shores (one lane each way): the
    // Phase 3 lane-centering road.
    ("Sandy Shores road", 2470.0, 4000.0, 40.0, 0.0, true),
    // Spots tried for the lane map (the Phase 3 survey): b's road stops short,
    // c and d gave no road, e jumps to the parallel carriageway after 40 m.
    ("survey b", -2200.0, 4300.0, 50.0, 0.0, true),
    ("survey c", 1200.0, -1800.0, 35.0, 0.0, true),
    ("survey d", 2400.0, -800.0, 60.0, 0.0, true),
    ("survey e", 1600.0, 1200.0, 80.0, 0.0, true),
];

/// Spots from `%LOCALAPPDATA%\adas\spots.txt` if present (outside the game
/// folder, which GTA locks): `name x y z heading snap(0/1)` per line.
fn user_spots() -> Vec<(String, f32, f32, f32, f32, bool)> {
    let Some(dir) = std::env::var_os("LOCALAPPDATA") else { return Vec::new() };
    let path = PathBuf::from(dir).join("adas").join("spots.txt");
    let Ok(text) = std::fs::read_to_string(path) else { return Vec::new() };
    text.lines()
        .filter_map(|l| {
            let p: Vec<&str> = l.split_whitespace().collect();
            if p.len() < 6 {
                return None;
            }
            let f = |i: usize| p[i].parse::<f32>().ok();
            Some((p[0].to_string(), f(1)?, f(2)?, f(3)?, f(4)?, p[5] == "1"))
        })
        .collect()
}

/// The test car F8 spawns: the model name in `%LOCALAPPDATA%\adas\car.txt`
/// (e.g. `baller`), or the Sultan.
pub fn test_car_model() -> std::ffi::CString {
    let name = std::env::var_os("LOCALAPPDATA")
        .and_then(|d| std::fs::read_to_string(PathBuf::from(d).join("adas").join("car.txt")).ok())
        .map(|s| s.trim().to_string())
        .filter(|s| !s.is_empty() && s.chars().all(|c| c.is_ascii_alphanumeric() || c == '_'))
        .unwrap_or_else(|| "sultan".to_string());
    std::ffi::CString::new(name).unwrap_or_else(|_| c"sultan".to_owned())
}

/// The index of the test spot with this name (case-insensitive), or a number.
pub fn spot_index(name: &str) -> Option<usize> {
    if let Ok(i) = name.parse::<usize>() {
        return (i < spot_count()).then_some(i);
    }
    let user = user_spots();
    if user.is_empty() {
        SPOTS.iter().position(|s| s.0.eq_ignore_ascii_case(name))
    } else {
        user.iter().position(|s| s.0.eq_ignore_ascii_case(name))
    }
}

pub fn spot_count() -> usize {
    let u = user_spots();
    if u.is_empty() { SPOTS.len() } else { u.len() }
}

/// While testing: no police. The airport is a restricted area, and a one-off
/// clear at teleport doesn't stop the wanted level coming back.
pub fn keep_unwanted() {
    unsafe {
        let player: i32 = invoke!(n::PLAYER_ID);
        let level: i32 = invoke!(n::GET_PLAYER_WANTED_LEVEL, player);
        if level > 0 {
            let () = invoke!(n::CLEAR_PLAYER_WANTED_LEVEL, player);
        }
        let () = invoke!(n::SET_MAX_WANTED_LEVEL, 0i32);
        let () = invoke!(n::SET_POLICE_IGNORE_PLAYER, player, true);
    }
}

pub fn teleport(ped: i32, veh: i32, idx: usize, log: &mut impl Write) {
    let user = user_spots();
    let (name, x, y, z, h, snap) = if user.is_empty() {
        let s = SPOTS[idx % SPOTS.len()];
        (s.0.to_string(), s.1, s.2, s.3, s.4, s.5)
    } else {
        user[idx % user.len()].clone()
    };
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
        let (mut tx, mut ty, mut tz, mut th) = (x, y, z, h);
        if snap {
            let mut node = Vector3::default();
            let mut node_h = 0.0f32;
            let ok: bool = invoke!(n::GET_CLOSEST_VEHICLE_NODE_WITH_HEADING, x, y, z, &mut node as *mut Vector3, &mut node_h as *mut f32, 0i32, 3.0f32, 0.0f32);
            if ok {
                (tx, ty, tz, th) = (node.x, node.y, node.z, node_h);
                for _ in 0..30 {
                    let () = invoke!(n::REQUEST_COLLISION_AT_COORD, tx, ty, tz);
                    shv_sys::scriptWait(0);
                }
            }
        }
        let mut gz = tz;
        let found: bool = invoke!(n::GET_GROUND_Z_FOR_3D_COORD, tx, ty, tz + 50.0, &mut gz as *mut f32, false, false);
        let () = invoke!(n::SET_ENTITY_COORDS, e, tx, ty, if found { gz + 0.5 } else { tz + 0.5 }, false, false, false, true);
        let () = invoke!(n::SET_ENTITY_HEADING, e, th);
        // Snapped to a node, the car stands on the road's node line (on a
        // two-way road, its centre line): move it into the first lane in its
        // direction.
        if snap {
            let off = crate::lanes::lane_offset_at(geom::V2::new(tx, ty), tz, th, crate::lanes::LANE_WIDTH);
            if off != 0.0 {
                let r = th.to_radians();
                // Left of heading h (GTA: 0 = north, CCW) is (-cos h, -sin h).
                let (lx, ly) = (-r.cos(), -r.sin());
                let p: Vector3 = invoke!(n::GET_ENTITY_COORDS, e, true);
                let () = invoke!(n::SET_ENTITY_COORDS, e, p.x + lx * off, p.y + ly * off, p.z, false, false, false, true);
                let () = invoke!(n::SET_ENTITY_HEADING, e, th);
                let _ = writeln!(log, "TELEPORT: moved {:.1} m {} into a lane", off.abs(), if off < 0.0 { "right" } else { "left" });
            }
        }
        if veh != 0 {
            let _: bool = invoke!(n::SET_VEHICLE_ON_GROUND_PROPERLY, veh, 5.0f32);
        }
        let () = invoke!(n::CLEAR_PLAYER_WANTED_LEVEL, player);
        let _ = writeln!(log, "TELEPORT {name}: {tx:.1} {ty:.1} {tz:.1} heading {th:.0}{}", if snap { " (road-snapped)" } else { "" });
    }
}

pub fn spawn(ped: i32, model_name: &core::ffi::CStr, prev: &mut i32, log: &mut impl Write) {
    unsafe {
        let model: u32 = invoke!(n::GET_HASH_KEY, model_name);
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
        let _ = writeln!(log, "SPAWN {} handle {veh}; player invincible", model_name.to_string_lossy());
    }
}

pub fn ai_drive(ped: i32, veh: i32, on: bool) {
    unsafe {
        if on {
            let () = invoke!(n::TASK_VEHICLE_DRIVE_WANDER, ped, veh, 20.0f32, 786_603i32);
        } else {
            let () = invoke!(n::CLEAR_PED_TASKS, ped);
        }
    }
}

/// A scripted lead car for ACC/AEB tests: spawned `distance` m ahead on the
/// ego car's heading, driven by an AI driver straight ahead at `speed` m/s
/// (or parked with no driver when `speed` is 0).
pub struct Lead {
    pub veh: i32,
    pub ped: i32,
}

fn load_model(name: &core::ffi::CStr) -> Option<u32> {
    unsafe {
        let model: u32 = invoke!(n::GET_HASH_KEY, name);
        let () = invoke!(n::REQUEST_MODEL, model);
        for _ in 0..300 {
            let loaded: bool = invoke!(n::HAS_MODEL_LOADED, model);
            if loaded {
                return Some(model);
            }
            shv_sys::scriptWait(0);
        }
        None
    }
}

pub fn clear_lead(lead: &mut Option<Lead>) {
    if let Some(mut l) = lead.take() {
        unsafe {
            if l.ped != 0 {
                let () = invoke!(n::SET_ENTITY_AS_MISSION_ENTITY, l.ped, true, true);
                let () = invoke!(n::DELETE_PED, &mut l.ped as *mut i32);
            }
            let () = invoke!(n::SET_ENTITY_AS_MISSION_ENTITY, l.veh, true, true);
            let () = invoke!(n::DELETE_VEHICLE, &mut l.veh as *mut i32);
        }
    }
}

pub fn spawn_lead(ego: i32, distance: f32, speed: f32, lead: &mut Option<Lead>, log: &mut impl Write) {
    clear_lead(lead);
    let (Some(car), Some(driver)) = (load_model(c"blista"), load_model(c"a_m_y_business_01")) else {
        let _ = writeln!(log, "LEAD: models did not load");
        return;
    };
    unsafe {
        let pos: Vector3 = invoke!(n::GET_ENTITY_COORDS, ego, true);
        let fwd: Vector3 = invoke!(n::GET_ENTITY_FORWARD_VECTOR, ego);
        let heading: f32 = invoke!(n::GET_ENTITY_HEADING, ego);
        let (x, y) = (pos.x + fwd.x * distance, pos.y + fwd.y * distance);
        let veh: i32 = invoke!(n::CREATE_VEHICLE, car, x, y, pos.z + 0.5, heading, false, false, false);
        let _: bool = invoke!(n::SET_VEHICLE_ON_GROUND_PROPERLY, veh, 5.0f32);
        let mut ped = 0i32;
        if speed > 0.0 {
            ped = invoke!(n::CREATE_PED_INSIDE_VEHICLE, veh, 26i32, driver, -1i32, false, false);
            let () = invoke!(n::SET_BLOCKING_OF_NON_TEMPORARY_EVENTS, ped, true);
            let () = invoke!(n::SET_PED_KEEP_TASK, ped, true);
            let () = invoke!(n::SET_VEHICLE_ENGINE_ON, veh, true, true, false);
            let (tx, ty) = (pos.x + fwd.x * 3000.0, pos.y + fwd.y * 3000.0);
            // Straight-line driving for the whole distance (no road network on the apron).
            let () = invoke!(n::TASK_VEHICLE_DRIVE_TO_COORD, ped, veh, tx, ty, pos.z, speed, 0i32, car, 786_603i32, 5.0f32, 3000.0f32);
            let () = invoke!(n::SET_VEHICLE_FORWARD_SPEED, veh, speed);
        } else {
            let () = invoke!(n::SET_VEHICLE_ENGINE_ON, veh, false, true, true);
        }
        let () = invoke!(n::SET_MODEL_AS_NO_LONGER_NEEDED, car);
        let () = invoke!(n::SET_MODEL_AS_NO_LONGER_NEEDED, driver);
        let _ = writeln!(log, "LEAD spawned {distance:.0} m ahead, {speed:.1} m/s (veh {veh}, driver {ped})");
        *lead = Some(Lead { veh, ped });
    }
}

/// A car on a fixed profile (a scenario primitive): no driver, its forward
/// speed set every frame, on the heading it was spawned with.
pub struct ProfileCar {
    pub veh: i32,
    pub speed: f32,
}

/// Cars and obstacles spawned for surround tests (Phase 2).
#[derive(Default)]
pub struct Extras {
    pub cars: Vec<ProfileCar>,
    /// Frozen obstacles.
    pub parked: Vec<i32>,
    /// Pedestrians waiting to cross: (ped, where they walk to).
    pub walkers: Vec<(i32, Vector3)>,
}

pub fn clear_extras(x: &mut Extras) {
    for mut v in x.cars.drain(..).map(|c| c.veh).chain(x.parked.drain(..)) {
        unsafe {
            let () = invoke!(n::SET_ENTITY_AS_MISSION_ENTITY, v, true, true);
            let () = invoke!(n::DELETE_VEHICLE, &mut v as *mut i32);
        }
    }
    for (mut p, _) in x.walkers.drain(..) {
        unsafe {
            let () = invoke!(n::SET_ENTITY_AS_MISSION_ENTITY, p, true, true);
            let () = invoke!(n::DELETE_PED, &mut p as *mut i32);
        }
    }
}

/// A pedestrian waiting `forward` m ahead, `left` m to the side (negative
/// right), facing across the road; `walk` sends them to the other side.
pub fn spawn_walker(ego: i32, forward: f32, left: f32, x: &mut Extras, log: &mut impl Write) {
    let Some(model) = load_model(c"a_m_y_business_01") else {
        let _ = writeln!(log, "WALKER: model did not load");
        return;
    };
    let (px, py, pz, h) = ego_offset(ego, forward, left);
    let (tx, ty, tz, _) = ego_offset(ego, forward, -left);
    unsafe {
        let across = if left < 0.0 { h + 90.0 } else { h - 90.0 };
        let ped: i32 = invoke!(n::CREATE_PED, 4i32, model, px, py, pz, across, false, false);
        let () = invoke!(n::SET_BLOCKING_OF_NON_TEMPORARY_EVENTS, ped, true);
        let () = invoke!(n::SET_MODEL_AS_NO_LONGER_NEEDED, model);
        let _ = writeln!(log, "WALKER {forward:.0} m ahead, {left:.1} m to the left (ped {ped})");
        x.walkers.push((ped, Vector3::new(tx, ty, tz)));
    }
}

/// Every waiting pedestrian walks across (about 1.4 m/s).
pub fn walk(x: &Extras, log: &mut impl Write) {
    for &(ped, to) in &x.walkers {
        let () = unsafe { invoke!(n::TASK_GO_STRAIGHT_TO_COORD, ped, to.x, to.y, to.z, 1.0f32, -1i32, 0.0f32, 0.0f32) };
    }
    let _ = writeln!(log, "WALK: {} pedestrians cross", x.walkers.len());
}

/// Keeps the profile cars at their speeds (call every frame).
pub fn drive_profiles(x: &Extras) {
    for c in &x.cars {
        let () = unsafe { invoke!(n::SET_VEHICLE_FORWARD_SPEED, c.veh, c.speed) };
    }
}

/// A point `forward` m ahead of and `left` m to the left of the ego car, and
/// the ego heading (deg).
fn ego_offset(ego: i32, forward: f32, left: f32) -> (f32, f32, f32, f32) {
    unsafe {
        let pos: Vector3 = invoke!(n::GET_ENTITY_COORDS, ego, true);
        let f: Vector3 = invoke!(n::GET_ENTITY_FORWARD_VECTOR, ego);
        let heading: f32 = invoke!(n::GET_ENTITY_HEADING, ego);
        let l = (f.x * f.x + f.y * f.y).sqrt().max(1e-6);
        let (fx, fy) = (f.x / l, f.y / l);
        // Left of forward, in GTA's world (x east, y north): rotate +90°.
        (pos.x + fx * forward - fy * left, pos.y + fy * forward + fx * left, pos.z, heading)
    }
}

/// Spawns a profile car at (`forward`, `left`) from the ego car, turned
/// `turn` degrees from the ego heading (CCW), at `speed` m/s.
pub fn spawn_profile_car(ego: i32, forward: f32, left: f32, turn: f32, speed: f32, x: &mut Extras, log: &mut impl Write) {
    let Some(car) = load_model(c"blista") else {
        let _ = writeln!(log, "PROFILE CAR: model did not load");
        return;
    };
    let (px, py, pz, h) = ego_offset(ego, forward, left);
    unsafe {
        let veh: i32 = invoke!(n::CREATE_VEHICLE, car, px, py, pz + 0.5, h + turn, false, false, false);
        let _: bool = invoke!(n::SET_VEHICLE_ON_GROUND_PROPERLY, veh, 5.0f32);
        let () = invoke!(n::SET_VEHICLE_ENGINE_ON, veh, true, true, false);
        let () = invoke!(n::SET_VEHICLE_FORWARD_SPEED, veh, speed);
        let () = invoke!(n::SET_MODEL_AS_NO_LONGER_NEEDED, car);
        let _ = writeln!(log, "PROFILE CAR {forward:.1} m ahead, {left:.1} m left, turned {turn:.0} deg, {speed:.1} m/s (veh {veh})");
        x.cars.push(ProfileCar { veh, speed });
    }
}

/// A van parked broadside with its near side `gap` m behind (`behind`) or
/// ahead of the ego car's bumper (`bumper`: the bumper's x from the ego
/// reference point, ISO), frozen in place: an obstacle for park assist.
pub fn spawn_obstacle(ego: i32, bumper: f32, gap: f32, behind: bool, x: &mut Extras, log: &mut impl Write) {
    let Some(model) = load_model(c"speedo") else {
        let _ = writeln!(log, "OBSTACLE: model did not load");
        return;
    };
    let (lo, hi) = {
        let (mut lo, mut hi) = (Vector3::default(), Vector3::default());
        let () = unsafe { invoke!(n::GET_MODEL_DIMENSIONS, model, &mut lo as *mut Vector3, &mut hi as *mut Vector3) };
        (lo, hi)
    };
    // Broadside: its width runs along the ego car's x.
    let half = 0.5 * (hi.x - lo.x);
    let forward = if behind { bumper - gap - half } else { bumper + gap + half };
    let (px, py, pz, h) = ego_offset(ego, forward, 0.0);
    unsafe {
        let veh: i32 = invoke!(n::CREATE_VEHICLE, model, px, py, pz + 0.5, h + 90.0, false, false, false);
        let _: bool = invoke!(n::SET_VEHICLE_ON_GROUND_PROPERLY, veh, 5.0f32);
        let () = invoke!(n::FREEZE_ENTITY_POSITION, veh, true);
        let () = invoke!(n::SET_MODEL_AS_NO_LONGER_NEEDED, model);
        let _ = writeln!(log, "OBSTACLE van broadside, near side {gap:.1} m {} the bumper (veh {veh})", if behind { "behind" } else { "ahead of" });
        x.parked.push(veh);
    }
}

/// The lead brakes until it stops (TASK_VEHICLE_TEMP_ACTION 27).
pub fn lead_brake(lead: &Option<Lead>, log: &mut impl Write) {
    if let Some(l) = lead
        && l.ped != 0
    {
        let () = unsafe { invoke!(n::TASK_VEHICLE_TEMP_ACTION, l.ped, l.veh, 27i32, 15_000i32) };
        let _ = writeln!(log, "LEAD brakes hard");
    }
}
