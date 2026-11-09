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

pub fn spot_count() -> usize {
    let u = user_spots();
    if u.is_empty() { SPOTS.len() } else { u.len() }
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
