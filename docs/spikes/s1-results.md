# Spike S1 results: Rust .asi on Script Hook V (Enhanced)

2026-11-08 · **Pass**

## Question
Can a Rust plugin load on GTA V Enhanced through Script Hook V, call natives and draw text, and run stably? And how do the game's timers, pause, slow motion and raycasts behave, since the timing model (D10, D23) and the safe state (D24) depend on them?

## Setup

| | |
| --- | --- |
| Game | GTA V Enhanced, Steam. Exe file version 1.0.1158.16; reports `VER_EN_1_0_1158_13` internally |
| Script Hook V | v3889.0/1158.13, with its Enhanced ASI loader (`dinput8.dll`) |
| Toolchain | Rust 1.99.1, `x86_64-pc-windows-msvc`, linked with `rust-lld` against xwin-downloaded MSVC CRT and Windows SDK libraries |
| Code | [`crates/shv-sys`](../../crates/shv-sys/src/lib.rs) (bindings), [`spikes/s1-asi`](../../spikes/s1-asi/src/lib.rs) (the spike), [`xtask`](../../xtask/src/main.rs) (deploy and launch) |
| Machine | The target gaming PC: RTX 4070 Ti, Ryzen 7 7800X3D |

The spike samples every frame:
- game timer, frame time, frame counter;
- pause, loading and cutscene flags;
- the player's speed;
- one async line-of-sight ray (50 m forward), and one synchronous ray per second.

It draws a text readout and logs a CSV row per frame plus a summary per second.

## Runs

| Run | Duration | What happened |
| --- | --- | --- |
| 1 | 10 min | Standing, walking, pause menu, slow motion at 0.5 and 0.2, autosave alerts, cheat console, phone calls, blind driving with collisions |
| 2 | about 9 min | Carjack, death and respawn, then about 6 min of soak mode: GTA's AI driving the player's car through the city. Weather and clock changed every minute, through all six weather types and a full day and night. Top speed 87 km/h |

## Results

| Check | Result |
| --- | --- |
| Loads, calls natives, draws text | Yes |
| Crashes or panics | None in either run |
| Script runs every frame | Yes. The frame counter steps by exactly 1 between ticks; 0 gaps in 39,394 frames (run 2) |
| Frame rate in the world | About 72 fps (13.9 ms per frame, p99 14.7 ms) |
| Game-thread cost per frame (run 2) | p50 14 µs, p99 107 µs, p99.9 159 µs, max 545 µs. **No frame over the 1 ms budget** |
| Async shape test (`START_SHAPE_TEST_LOS_PROBE`) | Never ready in the same tick. Ready exactly 1 frame later in every case (about 30,000 rays). One ray in run 2 returned "no such test", most likely around the death and respawn |
| Synchronous shape test (`START_EXPENSIVE_SYNCHRONOUS_...`) | Ready immediately; about 15–40 µs each |
| Slow motion (`SET_TIME_SCALE`) | `GET_GAME_TIMER` and `GET_FRAME_TIME` both scale. At 0.5, both advance 500 ms per wall-clock second |
| Pause menu | Scripts are suspended and the game timer stops. `IS_PAUSE_MENU_ACTIVE` is never seen true, because our script doesn't run while the menu is open |
| System alerts (autosave prompts) | Same as the pause menu: scripts suspended, game timer stopped. One alert left unanswered suspended the script for 175 s |
| Cheat console, phone | Scripts keep running |
| Accumulated frame time vs game timer | They agree to within about 1 ms per second |

## What this means for the design

- **D10 (time model):** accumulating `GET_FRAME_TIME` gives µs-resolution game time that matches `GET_GAME_TIMER`, pauses with the game and scales with slow motion. Use it as planned.
- **D23 (timing):** ray data is always exactly 1 frame old, a fixed and predictable delay. "≤ 2 frames from sample to applied command" is achievable. Raycasts never return in the same tick, so line-of-sight checks must be pipelined across frames. Synchronous rays are cheap enough for a handful per frame, if ever needed.
- **D24 (safe state):** the watchdog must count game frames, not wall-clock time. Menus and alerts freeze scripts for any length of time, and no samples arrive during them. When the game resumes, the watchdog continues as if no time passed, which is the behavior we want.
- **Adapter robustness:** a pending shape test can disappear (death, respawn, streaming). Treat "no such test" as a miss, never as a hang.
- **Budget:** our sampling costs about a tenth of the 1 ms budget, leaving plenty of room for S6's line-of-sight checks.

## Gotchas found

- **GTA locks files in its folder at startup.** Any file that already exists in the game folder when GTA starts is held open by the game, and our plugin can't overwrite it. The spike's second launch failed silently because of this. Fix: write new, uniquely named files per run, in `<game folder>/adas-logs/`.
- **Script Hook V exports are C++-mangled.** Its export names are mangled, for example `?nativeInit@@YAX_K@Z`. `shv-sys` links to the exact names read from the DLL's export table, using `raw-dylib`, so the SDK's import library isn't needed.
- **Scripts start only in Story Mode.** Script Hook V registers the plugin at the landing page, but `script_main` first runs after Story Mode loads.
- **The game build label differs.** The exe's file version (1158.16) differs from the version Script Hook V checks (1158.13). Check `ScriptHookV.log` after every Steam update.

## Not covered by S1
- Reading vehicle memory (wheel speeds, steering angle): S5.
- Steering hook and pedal control: S4.
- Line-of-sight checks for many actors per frame: S6.
- Shared memory to the core process: S3.

## Raw data
Logs and per-frame CSVs are in `<game folder>/adas-logs/`, plus run 1's copy in the developer's scratch folder. Per the repo rules, they aren't committed.
