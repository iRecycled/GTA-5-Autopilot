# Implementation plan

2026-11-08 · Nick Peterson

Seven phases, each ending in something drivable. Phase 0 starts with throwaway spikes before any production code. The plan is ordered by risk and by when you can first drive, not by dates.

## Repository layout

One Cargo workspace. Each crate owns one concern, so a phase adds crates instead of reworking old ones. Two rules hold the layout together:

- **Only the `gta-*` crates and `reshade-shim` know GTA exists.** A new host game gets its own adapter crates; nothing else changes (D22).
- **Only `scenario` and `sim` may see ground truth.** `sensors` reads ground truth only to model what a sensor reports. `features`, `fusion` and `control` see only bus messages, and the crate dependency graph enforces it.

| Crate | Kind | Owns | First phase |
| --- | --- | --- | --- |
| `adapter-api` | lib | The adapter contract: versioned messages, capabilities, mailboxes and rings, the clocks (D10, D22) | 0 |
| `geom` | lib | ISO 8855 frames, units, transforms (D16) | 0 |
| `adas-core` | bin | Connects to an adapter, runs `step()`, hosts the ECUs (D23) | 0 |
| `record` / `replay` | lib + bin | MCAP writing and reading; offline runs of the core (D20) | 0 |
| `shv-sys` | lib | Script Hook V bindings; native wrappers generated at build time | 0 |
| `gta-edition` | lib | Memory patterns, offsets and the steering write per game build (D2, D5) | 0 |
| `gta-plugin` | cdylib → `.asi` | The GTA adapter: sampling, standing sensor queries, actuation, safe state, HUD drawing, scenario primitives | 0 |
| `can` | lib | Topology, bus model, DBC code generation, faults (D9, D17) | 1 |
| `control` | lib | Speed and steering controllers, EPS model (D5, D19, D28) | 1 |
| `sensors` | lib | Sensor models and error model (D18, D25, D26) | 1 |
| `fusion` | lib | Tracking and prediction | 1 |
| `features` | lib | ACC, FCW, AEB, BSD, RCTA, park assist, LKA; engagement; degradation matrix (D11, D21) | 1 |
| `sim` | lib + bin | Closed-loop simulator implementing the adapter contract: bicycle-model ego car, scripted actors, no game (D20) | 1 |
| `scenario` | lib | Interprets `scenarios/*.toml` through adapter primitives; pass metrics (D27) | 1 |
| `viz` | lib | Rerun logging | 1 |
| `ui` | lib | egui engineering panel (D8) | 1 |
| `perception` | lib | GPU interop, `ort` + TensorRT inference, depth-buffer lidar | 5 |
| `reshade-shim` | C++ add-on | GTA colour, depth and stencil capture into a shared ring; the fixed C interface (D3, D4) | 5 |
| `planner` | lib | Routes and behavior | 6 |
| `tools/train` | Python | Fine-tuning pipeline → ONNX (D7) | 5 |

Data lives beside the code: `dbc/` (one file per CAN bus), `config/topology.toml`, `config/sensors/*.toml`, `config/vehicles/<host>/<model>.toml` and `scenarios/*.toml`.

## Contracts to freeze first

Three interfaces are expensive to change once code depends on them. They get written and reviewed before the code on either side. Each carries a version number, so a change is explicit, not silent.

### Adapter contract (Phase 0)

The contract is a set of versioned messages in shared memory, not a Rust trait, so an adapter can be written in any language. Every message starts with the same header:

```rust
#[repr(C)]
pub struct Header {
    pub kind: MsgKind,      // u16 message type
    pub version: u16,
    pub seq: u32,           // per channel; a gap means a dropped message
    pub frame: u64,         // host frame counter
    pub game_time_us: u64,  // accumulated from per-frame delta time
    pub wall_time_ns: u64,  // QueryPerformanceCounter, in ns
}
```

| Channel | Direction | Kind | Messages | Rate |
| --- | --- | --- | --- | --- |
| hello | both | Ring (lossless) | Contract version, epoch, adapter capabilities, sensor layout (standing queries) | On connect |
| sample | adapter → core | Mailbox (latest wins) | Vehicle state, actors, line-of-sight bits per sensor, range-fan hits, driver input, map tiles, pause state | Every frame |
| command | core → adapter | Mailbox (latest wins) | Steering angle, throttle, brake, engage state, the sample `seq` it answers | Every step |
| scenario | core → adapter | Ring (lossless) | Spawn, task, follow profile, teleport, weather and time, freeze, clear | On demand |
| scenario-reply | adapter → core | Ring (lossless) | Actor handles, errors | On demand |
| hud | core → adapter | Mailbox (latest wins) | HUD state: set speed, gap, lead car, warnings, spotter ring | 30 Hz |
| frames | adapter → core | Mailbox (latest wins) | GPU frame slot index, fence value, camera pose and projection (Phase 5) | Every camera frame |

All positions are ISO 8855 vehicle coordinates and SI units (D16). Each adapter does the one conversion from its game's frame.

**Mailbox:** a seqlock over one slot; the writer never waits and the reader always gets the newest complete value. **Ring:** single producer, single consumer, never drops. **Epoch:** incremented whenever the core starts; the adapter treats a new epoch as a fresh, disengaged core.

### Graphics shim C interface (drafted at the start of Phase 5, frozen after S7)

The shim is the only C++ in the project. It copies the game's frames into a ring of shared slots it owns, because the game's own resources can't be shared across processes (#5):

```c
typedef struct {
    uint64_t frame_id;        /* render-side frame counter */
    uint32_t slot;            /* index into the shared ring */
    uint64_t fence_value;     /* the core waits on this before reading */
    float    view[16];        /* sensor camera pose for this exact frame */
    float    proj[16];        /* projection, with TAA/DLSS jitter removed */
} gfx_frame;

/* At init, the shim creates N shared slots (colour RGBA8, depth R32F,
   stencil R8) and one shared fence, and duplicates their handles into
   the core process. */
int  gfx_init(const gfx_config *cfg, gfx_shared_handles *out);
int  gfx_latest_frame(gfx_frame *out);
void gfx_shutdown(void);
```

A pure-Rust DX12 hook (the D3 fallback) implements the same three functions. Hosts with native cameras (BeamNG, CARLA) fill the same `frames` channel directly.

### Configuration files (Phase 1)

| File | Holds |
| --- | --- |
| `config/topology.toml` | Buses (type, bit rate), which bus each ECU sits on, routes (D17) |
| `config/sensors/<sensor>.toml` | Mount pose, field of view, range, rate, error-model parameters (D18) |
| `config/vehicles/<host>/<model>.toml` | Dynamics (hand-tuned, later fitted), steering ratio, EPS limits (D19) |
| `config/core.toml` | Latency mode (`reactive` or `realistic`), watchdog frames (D23, D24) |
| `scenarios/<name>.toml` | Actors, start positions, triggers, profiles, pass criteria (D27) |

## Phase 0 · Spikes and skeleton

Goal: the core drives the car end to end through the GTA adapter, and the platform's unknowns are resolved. The spikes come first because each one can change what follows. Order: S1, then S4 and S5 (the biggest risk), then S3 and S6.

**Spikes**

- [ ] S1: `shv-sys` bindings; a Rust `.asi` loads on Enhanced, reads speed, draws text; record game timer and frame-time behavior in pause and slow motion; check whether shape tests can return in the same tick
- [ ] S4: hold and track a steering angle by the per-frame steering-input write, on 3 vehicles; measure the pedal mapping for simulated input; hold the car at a standstill without reversing
- [ ] S5: pattern-scan wheel speeds and steering angle into `gta-edition`; check across 3 vehicles
- [ ] S3: mailbox round trip between adapter and core, 99th percentile under 0.5 ms
- [ ] S6: line-of-sight cost for 5 radars in heavy traffic and ultrasonic fan cost, within 0.5 ms per frame; check whether parked vehicles appear in the actor list

**Build**

- [ ] Cargo workspace, MSVC toolchain, `cargo xtask deploy` copying the `.asi` into the GTA folder
- [ ] `adapter-api` v1: header, channels, mailboxes, rings, epoch, capabilities
- [ ] `geom` with ISO 8855 conversion, used inside the GTA adapter
- [ ] `adas-core`: connect, `step()` per sample, commands echo the sample `seq`
- [ ] GTA adapter: sampling, standing queries, actuation per D5, driver input, HUD text
- [ ] Safe state per D24: watchdog in frames, paused during menus and loading, steering write stopped at once
- [ ] Kill test: kill the core mid-turn and mid-brake; the player regains control within 2 frames
- [ ] Test driver: keyboard or gamepad input read by the core drives the car through the whole loop
- [ ] Raw recorder to MCAP, the `replay` binary, and `cargo xtask test` running replays (D20)

**Gate:** spikes pass; the core drives the car; adapter ≤ 1 ms per frame; the kill test passes; a recorded drive replays without GTA.

## Phase 1 · First drive: ACC, FCW, AEB

Goal: the first phase that feels like a modern car. It follows traffic smoothly and brakes decisively, on real bus traffic, and the controllers are tested without GTA.

**Buses**

- [ ] `config/topology.toml` (D17): chassis CAN and ADAS CAN FD; the ADAS ECU on both
- [ ] Chassis DBC from opendbc's Toyota TSS2 (confirm the exact file, e.g. `toyota_nodsu_pt_generated`, and that it has `STEERING_LTA`); our own ADAS DBC
- [ ] Code generation from DBC files into Rust message types
- [ ] Bus model: arbitration, bit stuffing, load, CAN FD data phase; counters and checksums; fault-injection API; `reactive` and `realistic` delivery (D23)
- [ ] Chassis ECUs publishing wheel speeds, yaw rate, steering angle and driver inputs at real cycle times
- [ ] Actuator ECU with the EPS model: angle commands, rate and range limits, fault states (D28)

**Sensing and control**

- [ ] Sensor framework: mount poses, fields of view, standing queries, error model with datasheet parameters (D18)
- [ ] Front radar ECU per D25, publishing object lists on ADAS CAN FD; ground truth recorded beside it as the oracle (D6)
- [ ] Tracker: a Kalman filter per object, track confirmation and deletion, lead-vehicle selection
- [ ] Hand-tuned vehicle profiles for two GTA cars (D19)
- [ ] Speed controller: PID with feedforward from the vehicle profile, jerk-limited
- [ ] ACC: set speed, time gap 1.0 to 2.5 s, stop-and-go, cut-in handling
- [ ] FCW and staged AEB: warn at about 2.5 s to collision, partial brake at 1.6 s, full at 0.9 s
- [ ] Engagement and override (D11): per-device thresholds; takeover escalation from visual to chime to a controlled stop with hazards
- [ ] Degradation matrix (D21), including missing adapter capabilities

**Testing and views**

- [ ] `sim`: bicycle-model ego car, scripted actors, implements the adapter contract; runs on every build with pass metrics (gap error, jerk, minimum TTC)
- [ ] `scenario` v1 and the GTA adapter's scenario primitives: lead car brakes hard, a car cuts in, a stopped car past a bend
- [ ] Recorder: both buses plus raw samples to MCAP; candump export
- [ ] HUD through the adapter: set speed, gap bars, lead-car icon, warnings
- [ ] `ui` engineering panel: toggles, fault injection, latency readout
- [ ] Rerun views: bus load per network, CAN trace, vehicle-state plots, bird's-eye view
- [ ] Borrow from openpilot (D13): engagement rules and controller tuning

**Gate:**
- ACC: gap error ≤ 0.3 s at steady state; jerk ≤ 2.5 m/s³; no AEB activation in 10 of 10 highway follow runs.
- AEB: stops before a stationary car from 50 km/h in 10 of 10 runs, in the sim and in GTA; no activation in any run that shouldn't trigger it (following, cut-in, lead braking, faults). The false-activation rate per 30 min moves to Phase 3, where lane keeping makes long drives possible.
- Radar path ≤ 2 frames at the 99th percentile; live CAN trace and bus load on both buses.
- Override and takeover (D11): brake cancels, gas overrides, steering doesn't cancel ACC; an unanswered takeover request ends in a controlled stop with hazards, in the sim and in GTA.

## Phase 2 · Surround: corner radars and ultrasonics

Goal: 360° awareness at short range, reusing Phase 1's sensor framework and tracker.

- [ ] Four corner radar ECUs per D25, on ADAS CAN FD (our own object-list DBC layout)
- [ ] Blind spot detection: zones, spotter ring and mirror icons, a chime when signaling into an occupied lane
- [ ] Rear cross-traffic alert: predicts crossing traffic while reversing; warns, then brakes
- [ ] Twelve ultrasonics on ray fans: park-assist distance arcs and tone; low-speed auto-brake
- [ ] Scenarios in the sim and in GTA: a car overtaking in the blind spot; reversing out of a parking space into traffic; parking against a wall

**Gate:** BSD detects 10 of 10 overtakes with ≤ 1 false alert per 30 min; RCTA brakes for crossing traffic in 10 of 10 runs; park assist stops ≥ 0.3 m from the obstacle in 10 of 10 runs. Then decide on a public release (D14).

## Phase 3 · Camera ECU and lanes: LKA, lane centering

Goal: the car steers itself within its lane, with a camera ECU that already behaves like a production one on the bus (D26).

- [ ] Map capability in the GTA adapter: road network from path nodes; local lane map (lane count, widths, centerlines)
- [ ] Camera ECU: objects, lanes and signs within its field of view, through a camera error model (worse range accuracy than radar, weather and night effects, misclassification); published on ADAS CAN FD
- [ ] Radar–camera fusion in `fusion`; pedestrian AEB
- [ ] Steering controller through the EPS model: pure pursuit first, then Stanley or MPC if pure pursuit can't hold the gate
- [ ] Lane keeping (a nudge at the lane edge), then continuous lane centering
- [ ] Curve speed: ACC slows ahead of bends from lane curvature
- [ ] Optional driver-monitoring toggle (D11)
- [ ] Scenarios: a long highway run, a tight bend, a lane that ends, a pedestrian stepping out

**Gate:** lane-centre error ≤ 0.3 m RMS on a named highway loop; ≤ 1 takeover per 10 km; pedestrian AEB stops in 10 of 10 runs at 30 km/h; ≤ 1 false AEB activation per 30 min of driving in traffic (moved from Phase 1).

## Phase 4 · Full network and a second host

Goal: the full vehicle network, fitted vehicle models, and proof that the core is portable.

- [ ] Body CAN and a gateway ECU routing only what each bus needs (D9, D17)
- [ ] Turn signals, hazards and cluster on body CAN
- [ ] Automatic calibration drive: throttle, brake and steering steps and sweeps fit a model per car (D19)
- [ ] A second host adapter (proposed: BeamNG.drive), using only the existing contract
- [ ] Optional: Foxglove layouts for replays

**Gate:** calibration fits 3 GTA cars; the Phase 1 ACC and AEB scenarios pass on the second host with no changes outside its adapter.

## Phase 5 · Vision and lidar

Goal: the car sees for itself. The camera ECU's eyes become a detector on real frames, aiming at what a current production camera does, and replay tests score it against ground truth.

- [ ] S2b: hidden forward sensor camera in GTA (D29); decide between interleaving, the player's own camera, or a hood view
- [ ] S2: depth and stencil through ReShade, with the frame-alignment check
- [ ] S7: shared frame ring to the core and CUDA; detector timing; freeze the shim interface
- [ ] Forward lidar from the depth buffer: real beam pattern, stencil labels, point frames on a modelled Ethernet channel
- [ ] Auto-label exporter: frames plus boxes and masks from the stencil buffer and ground truth
- [ ] `tools/train`: fine-tune lyuwenyu/RT-DETR or YOLOX (pick after a quick benchmark), export ONNX, build the TensorRT engine; version each model with its data
- [ ] Camera ECU on vision: vehicles, pedestrians, lanes, signs and lights; the same output messages as Phase 3; Windows screen capture as the colour-only fallback (D3)
- [ ] Fusion of radar, camera and lidar tracks
- [ ] Vision scoring against the ground-truth oracle in replay tests (D6)
- [ ] Experiment: openpilot's lane model against map lanes; keep whichever scores better

**Gate:** camera features run on vision, not ground truth; scores tracked per build; camera path ≤ 3 frames at the 99th percentile.

## Phase 6 · Full autopilot

Goal: point-to-point driving across the city, with the driver supervising.

- [ ] Route planner on the road graph, with destination input from the map
- [ ] Behavior planner as a state machine: follow lane, change lane, approach junction, stop, turn
- [ ] Lane change assist on top of blind spot detection
- [ ] Traffic lights and stop signs from the camera
- [ ] Optional: openpilot as an alternative brain under WSL2 on the gaming PC (D13)
- [ ] Optional: steering wheel and pedals with force feedback, using the driver-torque signal (D15)

**Gate:** a named city route at a named traffic density; takeovers per km recorded and tracked as a target, not a pass or fail.

## Development workflow

All code lives in one private Git repository and is built and run on the gaming PC.

| Loop | Command | When |
| --- | --- | --- |
| Core change | `cargo xtask core` rebuilds and restarts `adas-core`; the game keeps running | Most changes |
| Adapter or shim change | `cargo xtask deploy`, then restart GTA | Rare |
| Tests | `cargo xtask test`: unit tests, closed-loop sim scenarios, and every replay in `recordings/regression/` (D20) | Every build; no game needed |
| Phase gate | Scenario suite from `scenarios/`, run in the game | Before closing each phase |

**Rules for the machine**

- Launch story mode with BattlEye off (`-nobattleye`). Never go online with the mod installed.
- Pin the game build: back up the GTA folder once Phase 0 works, and update GTA only together with Script Hook V and `gta-edition`.
- A drive that finds a bug gets saved to `recordings/regression/`, so it becomes a test forever.
- Recordings, model files and TensorRT engines stay out of Git; only their names and versions are committed.
