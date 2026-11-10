//! Starting the core from inside the game (F2, or F10 with no core running).
//!
//! A convenience of the GTA adapter only; the core is the same process
//! `cargo run -p adas-core` starts. `cargo xtask deploy` writes
//! `adas-core.txt` next to the game: the first line is the repository, the
//! second the core's arguments (edit it to pick another feature). The core
//! runs from the repository (its recordings and vehicle profiles live
//! there), without a window, its output in `recordings/core-<time>.log`.
//! It is told to exit once the game has sent nothing for a while, so no core
//! outlives the game.

use std::fs::File;
use std::os::windows::process::CommandExt;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};

/// The file next to the game that says where the core is.
pub const CONFIG: &str = "adas-core.txt";
/// The core exits after this long without a sample from the game (s).
pub const EXIT_WHEN_IDLE_S: u32 = 60;
const CREATE_NO_WINDOW: u32 = 0x0800_0000;

#[derive(Default)]
pub struct Launcher {
    child: Option<Child>,
}

impl Launcher {
    /// A core this launcher started is still running.
    pub fn running(&mut self) -> bool {
        match self.child.as_mut().map(|c| c.try_wait()) {
            Some(Ok(None)) => true,
            _ => {
                self.child = None;
                false
            }
        }
    }

    /// Starts the core; `game_dir`: where `adas-core.txt` is. Returns what
    /// to tell the driver (the log file), or why it couldn't start.
    pub fn start(&mut self, game_dir: &Path) -> Result<String, String> {
        if self.running() {
            return Err("the core is already starting".into());
        }
        let cfg = game_dir.join(CONFIG);
        let text = std::fs::read_to_string(&cfg).map_err(|_| format!("no {CONFIG}: run `cargo xtask deploy`"))?;
        let mut lines = text.lines().map(str::trim).filter(|l| !l.is_empty() && !l.starts_with('#'));
        let root = PathBuf::from(lines.next().ok_or(format!("{CONFIG} is empty"))?);
        let args: Vec<String> = lines.next().unwrap_or("--feature autopilot").split_whitespace().map(String::from).collect();
        let exe = root.join("target").join("release").join("adas-core.exe");
        if !exe.exists() {
            return Err(format!("no core at {}: run `cargo build --release -p adas-core`", exe.display()));
        }
        let logs = root.join("recordings");
        let _ = std::fs::create_dir_all(&logs);
        let secs = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0);
        let log_path = logs.join(format!("core-{secs}.log"));
        let out = File::create(&log_path).map_err(|e| format!("{}: {e}", log_path.display()))?;
        let err = out.try_clone().map_err(|e| e.to_string())?;
        let child = Command::new(&exe)
            .args(&args)
            .args(["--exit-when-idle", &EXIT_WHEN_IDLE_S.to_string()])
            .current_dir(&root)
            .stdin(Stdio::null())
            .stdout(out)
            .stderr(err)
            .creation_flags(CREATE_NO_WINDOW)
            .spawn()
            .map_err(|e| format!("{}: {e}", exe.display()))?;
        let msg = format!("core started (pid {}, {}), log {}", child.id(), args.join(" "), log_path.display());
        self.child = Some(child);
        Ok(msg)
    }
}
