//! `cargo xtask <command>`
//!
//! - `gta-dir`   print the GTA V Enhanced folder that will be used
//! - `deploy [spike]`  build a spike (`s1` default, `s45`) and make it the only
//!   `adas-*.asi` in the game folder
//! - `undeploy`  remove every deployed spike `.asi` from the game folder
//! - `launch`    start GTA V Enhanced through Steam
//! - `log`       print the newest spike log from `<game folder>/adas-logs`
//!
//! The game folder comes from `GTA5_DIR` if set, otherwise from Steam's library list.

use std::env;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::{Command, ExitCode};

const STEAM_APP_ID: &str = "3240220";
const GAME_FOLDER: &str = "Grand Theft Auto V Enhanced";
const GAME_EXE: &str = "GTA5_Enhanced.exe";
/// Deployable spikes: (name, cargo package, built DLL, deployed .asi).
const SPIKES: &[(&str, &str, &str, &str)] = &[
    ("s1", "s1-asi", "s1_asi.dll", "adas-s1.asi"),
    ("s45", "s45-vehicle", "s45_vehicle.dll", "adas-s45.asi"),
];

fn main() -> ExitCode {
    let cmd = env::args().nth(1).unwrap_or_default();
    let result = match cmd.as_str() {
        "gta-dir" => gta_dir().map(|d| println!("{}", d.display())),
        "deploy" => deploy(&env::args().nth(2).unwrap_or_else(|| "s1".into())),
        "undeploy" => undeploy(),
        "launch" => launch(),
        "log" => show_log(),
        _ => Err(format!(
            "usage: cargo xtask <gta-dir|deploy|undeploy|launch|log>{}",
            if cmd.is_empty() { String::new() } else { format!(" (unknown command '{cmd}')") }
        )),
    };
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("xtask: {e}");
            ExitCode::FAILURE
        }
    }
}

fn workspace_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).parent().unwrap().to_path_buf()
}

fn gta_dir() -> Result<PathBuf, String> {
    if let Ok(dir) = env::var("GTA5_DIR") {
        let dir = PathBuf::from(dir);
        return if dir.join(GAME_EXE).exists() {
            Ok(dir)
        } else {
            Err(format!("GTA5_DIR={} has no {GAME_EXE}", dir.display()))
        };
    }
    for lib in steam_libraries()? {
        let dir = lib.join("steamapps").join("common").join(GAME_FOLDER);
        if dir.join(GAME_EXE).exists() {
            return Ok(dir);
        }
    }
    Err(format!("{GAME_FOLDER} not found in any Steam library; set GTA5_DIR"))
}

/// Steam's install path from the registry, then every library in libraryfolders.vdf.
fn steam_libraries() -> Result<Vec<PathBuf>, String> {
    let out = Command::new("reg")
        .args(["query", r"HKCU\Software\Valve\Steam", "/v", "SteamPath"])
        .output()
        .map_err(|e| format!("reg query failed: {e}"))?;
    let text = String::from_utf8_lossy(&out.stdout);
    let steam = text
        .lines()
        .find_map(|l| l.split_once("REG_SZ").map(|(_, v)| v.trim().to_string()))
        .ok_or("Steam not found in the registry; set GTA5_DIR")?;
    let steam = PathBuf::from(steam);

    let mut libs = vec![steam.clone()];
    let vdf = steam.join("steamapps").join("libraryfolders.vdf");
    if let Ok(text) = fs::read_to_string(&vdf) {
        for line in text.lines() {
            let parts: Vec<&str> = line.split('"').collect();
            // Line shape: <tab>"path"<tabs>"E:\\SteamLibrary"
            if parts.len() >= 4 && parts[1] == "path" {
                libs.push(PathBuf::from(parts[3].replace(r"\\", r"\")));
            }
        }
    }
    Ok(libs)
}

fn game_running() -> bool {
    Command::new("tasklist")
        .args(["/FI", &format!("IMAGENAME eq {GAME_EXE}"), "/NH"])
        .output()
        .map(|o| String::from_utf8_lossy(&o.stdout).contains(GAME_EXE))
        .unwrap_or(false)
}

/// Builds one spike and deploys it as the only `adas-*.asi` in the game folder.
fn deploy(name: &str) -> Result<(), String> {
    let &(_, package, dll, asi) = SPIKES
        .iter()
        .find(|s| s.0 == name)
        .ok_or_else(|| format!("unknown spike '{name}'; known: {}", SPIKES.iter().map(|s| s.0).collect::<Vec<_>>().join(", ")))?;
    let dir = gta_dir()?;
    if game_running() {
        return Err(format!("{GAME_EXE} is running; close it first (the .asi is locked while loaded)"));
    }
    let cargo = env::var("CARGO").unwrap_or_else(|_| "cargo".into());
    let status = Command::new(cargo)
        .current_dir(workspace_root())
        .args(["build", "--release", "--package", package])
        .status()
        .map_err(|e| format!("cargo build failed to start: {e}"))?;
    if !status.success() {
        return Err("cargo build failed".into());
    }
    for &(_, _, _, other) in SPIKES.iter().filter(|s| s.3 != asi) {
        let p = dir.join(other);
        if p.exists() {
            fs::remove_file(&p).map_err(|e| format!("remove {}: {e}", p.display()))?;
            println!("removed {}", p.display());
        }
    }
    let src = workspace_root().join("target").join("release").join(dll);
    let dst = dir.join(asi);
    fs::copy(&src, &dst).map_err(|e| format!("copy {} -> {}: {e}", src.display(), dst.display()))?;
    println!("deployed {}", dst.display());
    if !dir.join("ScriptHookV.dll").exists() {
        println!("warning: ScriptHookV.dll is missing from the game folder");
    }
    Ok(())
}

/// Removes every deployed spike `.asi` from the game folder.
fn undeploy() -> Result<(), String> {
    let dir = gta_dir()?;
    if game_running() {
        return Err(format!("{GAME_EXE} is running; close it first"));
    }
    let mut removed = 0;
    for &(_, _, _, asi) in SPIKES {
        let p = dir.join(asi);
        if p.exists() {
            fs::remove_file(&p).map_err(|e| format!("remove {}: {e}", p.display()))?;
            println!("removed {}", p.display());
            removed += 1;
        }
    }
    if removed == 0 {
        println!("nothing to remove");
    }
    Ok(())
}

fn launch() -> Result<(), String> {
    if game_running() {
        return Err(format!("{GAME_EXE} is already running"));
    }
    Command::new("cmd")
        .args(["/C", "start", "", &format!("steam://rungameid/{STEAM_APP_ID}")])
        .status()
        .map_err(|e| format!("could not start Steam: {e}"))?;
    println!("launch requested through Steam (BattlEye stays off via args.txt)");
    Ok(())
}

/// Prints the newest `*.log` in `<game folder>/adas-logs` (one file per run).
fn show_log() -> Result<(), String> {
    let dir = gta_dir()?.join("adas-logs");
    let newest = fs::read_dir(&dir)
        .map_err(|e| format!("read {}: {e}", dir.display()))?
        .filter_map(Result::ok)
        .filter(|e| e.path().extension().is_some_and(|x| x == "log"))
        .max_by_key(|e| e.metadata().and_then(|m| m.modified()).ok())
        .ok_or_else(|| format!("no logs in {}", dir.display()))?;
    let path = newest.path();
    println!("== {}", path.display());
    // The game may hold the file open while running; read with sharing.
    let text = fs::read_to_string(&path).map_err(|e| format!("read {}: {e}", path.display()))?;
    print!("{text}");
    Ok(())
}
