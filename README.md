# GTA 5 Autopilot

A realistic driver-assistance and autopilot stack, starting with **GTA V Enhanced** (single-player only). The car drives the way a real ADAS stack does:

sensors → vehicle buses (CAN / CAN FD) → ECUs → actuators

The autopilot never reads game state directly. It only sees what its sensors put on the bus. And the core never relies on a GTA trick: each game gets an adapter, so the stack can move to BeamNG, CARLA or another sim by writing a new one.

- **Core:** Rust, sim-neutral, in a separate process from the game, connected over shared memory.
- **First adapter:** Script Hook V (Enhanced). A thin C++ ReShade shim for camera frames comes in Phase 5.
- **Features:** ACC, AEB, LKA, blind-spot, rear cross-traffic and more, each toggleable.
- **Views:** in-game HUD, an engineering panel, Rerun 3D views, MCAP recordings.
- **Testing:** a closed-loop simulator tests the controllers without any game running.

## Status
Phases 0 and 1 are done (ACC, FCW, AEB on real bus traffic), and Phase 2 (corner radars, ultrasonics, BSD, RCTA, park assist) is built and tested in the simulator and in GTA. Phase 3 (the camera ECU, lane centering, LKA, pedestrian AEB) is built and passes its simulator scenarios; its GTA gate runs are in progress. See [docs/status/](docs/status/).

| Doc | What's in it |
| --- | --- |
| [docs/PRD.md](docs/PRD.md) | Goals, features and sensors, architecture, sim adapter, buses, timing, safety, tech stack, constraints, decision log D1–D29, spikes |
| [docs/implementation-plan.md](docs/implementation-plan.md) | Crate layout, contracts to freeze first (adapter, graphics shim, config), Phases 0–6 with checklists and gates |
| [docs/design-review.md](docs/design-review.md) | The first design review: a critique of every original decision, with alternatives |
| [docs/status/](docs/status/) | Progress reports per phase, with gate results |
| [docs/degradation.md](docs/degradation.md) | The degradation matrix (D21): what each missing capability or fault makes unavailable |

The second design review's questions were tracked as [issues](../../issues) #1–#11. Their decisions are now in `docs/` as D22–D29 plus revisions to earlier rows. The one still open, the hidden forward camera in GTA (#1, D29), isn't needed until Phase 5.

## Driving it yourself

Cargo isn't on the PATH on this machine. In Git Bash, run `export PATH="$USERPROFILE/.cargo/bin:$PATH"` first. In PowerShell, run `$env:Path = "$env:USERPROFILE\.cargo\bin;$env:Path"`.

**Once, and after pulling changes to the adapter** (with GTA closed):

```
cargo build --release -p adas-core       # the core the game will start
cargo xtask deploy                       # the mod (adas.asi) and adas-core.txt, into the game folder
```

**Then:**

1. Start GTA: `cargo xtask launch`, or from Steam.
2. Get in a car and press **F10**. With no core running, F10 starts one: "STARTING THE AUTOPILOT CORE" shows at the top, and the autopilot engages once the core answers. **F2** only starts the core, without engaging.
3. **F10** again disengages. Braking or steering also hands control back.

The core the game starts has no window. Its output goes to `recordings/core-<time>.log`, and its drive to `recordings/<time>-<feature>.mcap`. It exits 60 s after the game stops sending, so none is left running. It runs the feature named in `adas-core.txt` in the game folder (default `--feature autopilot`, which is ACC + lane centering). Edit that file's second line to pick `acc` or `test-driver`, or to add `--driver-monitor`.

Alternatively, run the core in a terminal to watch its log live. A newer core takes over from an older one:

```
cargo run --release -p adas-core -- --feature autopilot
```

**Keys in the game:**

| Key | Does |
| --- | --- |
| F10 | engage / disengage (starts the core if none is running) |
| F2 | start the core |
| F3 | cancel |
| Page Up / Page Down | set speed up / down |
| End | time gap |
| `,` / `.` | turn signal left / right |
| F9 | GTA's own AI drives (for surveys) |
| F7 | teleport to the next test spot |
| F8 | spawn a test car |
| F11 / F4 | a lead car 50 m ahead at 12 m/s / a stopped car 70 m ahead |
| F5 | the lead brakes hard |
| F6 | 60 fps cap on/off |

On the road, the HUD draws the lane lines the camera sees (green while lane centering steers). An **amber** pair of lines shows where the car will go in the next 3 s on its current steering. A **blue** line shows the lane centre the autopilot steers along. When amber and blue split, the car is about to change its steering.

**Views** (optional, in other terminals): `cargo run --release -p panel` for the engineering panel, and `cargo run --release -p viz` for the Rerun 3D views.

**Automated in-game tests:** `cargo xtask gta` (see [gta-tests/README.md](gta-tests/README.md)). **Closed-loop scenarios, no game:** `cargo run --release -p scenario -- scenarios/*.toml`. **Unit tests and the recorded regression drive:** `cargo xtask test`.

## Target machine
A gaming PC with an RTX 4070 Ti (12 GB) and a Ryzen 7 7800X3D. GTA and all builds and tests run there.

## Rules
- Story mode only. Launch with `-nobattleye`, and never go online with mods loaded.
- Pin the game build. Patches break memory patterns and offsets.
