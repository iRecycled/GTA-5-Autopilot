# Implementation plan

2026-11-08 · Nick Peterson

Seven phases, about 95 to 132 focused days of work in total (rough estimate). Every phase ends in something drivable. Phase 0 starts with six throwaway spikes before any production code.

## Repository layout

One Cargo workspace of 17 crates. Each crate owns one concern, so a phase adds crates instead of reworking old ones. Only `gta-plugin`, `gta-edition` and `reshade-shim` ever touch the game; everything else is testable without GTA.

| Crate | Kind | Owns | First phase |
| --- | --- | --- | --- |
| `shv-sys` | lib | Script Hook V bindings; native wrappers generated at build time | 0 |
| `gta-edition` | lib | Memory patterns and offsets per game build (D2) | 0 |
| `gta-plugin` | cdylib → `.asi` | Per-frame sampling, raycast scheduler, actuation, scenario spawning | 0 |
| `reshade-shim` | C++ add-on | Depth, stencil and color capture; overlay; the fixed C interface (D3) | 0 |
| `ipc` | lib | Shared-memory rings, message schema, the two clocks (D10) | 0 |
| `geom` | lib | ISO 8855 frames, units, GTA-to-vehicle transforms (D16) | 0 |
| `adas-core` | bin | 100 Hz scheduler, ECU host, watchdog | 0 |
| `record` / `replay` | lib + bin | MCAP writing and reading; offline runs of the core (D20) | 0 |
| `can` | lib | Topology, bus model, DBC code generation, gateway, faults (D9, D17) | 1 |
| `control` | lib | Speed and steering controllers, EPS model, vehicle calibration (D5, D19) | 1 |
| `viz` | lib | Rerun logging | 1 |
| `sensors` | lib | Sensor models, ray budget, error model (D18) | 2 |
| `fusion` | lib | Tracking and prediction | 2 |
| `features` | lib | ACC, AEB, BSD, RCTA, park assist, LKA; degradation matrix (D21) | 2 |
| `perception` | lib | GPU interop, `ort` + TensorRT inference, depth-buffer lidar | 1 (spike), 5 |
| `planner` | lib | Routes and behavior | 6 |
| `tools/train` | Python | Fine-tuning pipeline → ONNX (D7) | 5 |

Data lives beside the code: `dbc/` (one file per CAN bus), `config/topology.toml`, `config/sensors/*.toml`, `config/vehicles/<model>.toml` (written by the calibration drive) and `scenarios/*.toml`.

## Contracts to freeze first

Three interfaces are expensive to change once code depends on them. They get written and reviewed before the code on either side. Each carries a version number, so a change is explicit, not silent.

### IPC message schema (Phase 0)

Every message starts with the same header, so every sample and command carries both clocks (D10):

```rust
#[repr(C)]
pub struct Header {
    pub kind: MsgKind,      // u16 message type
    pub version: u16,
    pub seq: u32,           // per ring; a gap means a dropped message
    pub game_time_us: u64,  // GTA game clock
    pub wall_time_ns: u64,  // QueryPerformanceCounter, in ns
}
```

| Ring | Direction | Messages | Rate |
| --- | --- | --- | --- |
| samples | plugin → core | Vehicle state, object snapshot, driver input, ray results | Every frame |
| commands | core → plugin | Actuator command, ray requests, scenario commands | 100 Hz |
| frames | plugin → core | GPU frame descriptors: shared handles + fence value | Every frame |
| overlay | core → plugin | HUD state, boxes, points to draw | 30 Hz |

All positions in the schema are already in ISO 8855 vehicle coordinates and SI units (D16). The plugin does the one conversion from GTA's world frame.

### Graphics shim C interface (drafted Phase 0, frozen after S7 in Phase 1)

The shim is the only C++ in the project. The Rust plugin in the same process calls it and forwards frame descriptors to the core over the frames ring:

```c
typedef struct {
    uint64_t frame_id;
    uint64_t game_time_us;
    HANDLE   color;          /* shared D3D12 resources (D4) */
    HANDLE   depth_stencil;
    HANDLE   fence;
    uint64_t fence_value;    /* the core waits on this before reading */
    float    view_proj[16];  /* to turn depth back into 3D points */
} gfx_frame;

int  gfx_init(const gfx_config *cfg);
int  gfx_latest_frame(gfx_frame *out);
void gfx_submit_overlay(const overlay_cmd *cmds, size_t count);
void gfx_shutdown(void);
```

Handles are per process, so the plugin duplicates them into the core process before sending. A pure-Rust DX12 hook (the D3 fallback) implements the same four functions.

### Configuration files (Phase 1)

| File | Holds |
| --- | --- |
| `config/topology.toml` | Buses (type, bit rate), which bus each ECU sits on, gateway routes (D17) |
| `config/sensors/<sensor>.toml` | Mount pose, field of view, range, rate, error-model parameters (D18) |
| `config/vehicles/<model>.toml` | Dynamics fitted by the calibration drive, steering ratio, EPS limits (D19) |
| `scenarios/<name>.toml` | Actors, start positions, triggers, pass criteria (D20) |

## Phase 0 · Spikes and skeleton (12 to 16 days)

Goal: the core process drives the car end to end through the plugin, and the platform's unknowns are resolved. The spikes come first because each one can change what follows.

**Spikes**

- [ ] S1: `shv-sys` bindings; a Rust `.asi` loads on Enhanced, reads speed, draws text; record the game timer's behavior in pause and slow motion
- [ ] S2: ReShade add-on reads depth and stencil on Enhanced; overlay shows depth. Pass = ReShade stays; fail = Legacy fallback (D2) or own DX12 hook (D3)
- [ ] S3: shared-memory ring round trip, 99th percentile under 0.5 ms
- [ ] S4: steering and pedal fidelity, simulated input vs memory writes
- [ ] S5: pattern-scan steering angle and wheel speeds into `gta-edition`; check across 3 vehicles
- [ ] S6: raycast budget at 0.5 ms of game-thread cost

**Build**

- [ ] Cargo workspace, MSVC toolchain, `cargo xtask deploy` copying the `.asi` and shim into the GTA folder
- [ ] Native wrappers generated at build time from the native database
- [ ] `ipc` schema v1 and rings; `geom` with ISO 8855 conversion at the plugin boundary
- [ ] `adas-core` with the 100 Hz scheduler on game time and the safe-state watchdog (no command for 100 ms → release throttle, alert)
- [ ] Actuation path (D5): pedals by simulated input, steering angle write, player controls blocked and read as driver input
- [ ] Test driver: keyboard or gamepad input read by the core drives the car through the whole loop
- [ ] Raw recorder to MCAP, the `replay` binary, and `cargo xtask test` running replays (D20)

**Gate:** S1 to S6 pass; the core drives the car; game-thread cost ≤ 1 ms; a recorded drive replays without GTA.

## Phase 1 · Buses and tooling (12 to 16 days)

Goal: the car's networks exist and are observable, so every later feature is built and debugged on real bus traffic.

- [ ] `config/topology.toml` (D17): chassis CAN, ADAS CAN FD, body CAN, Ethernet channel, gateway routes
- [ ] Chassis-bus DBC taken from a real car in opendbc. Proposed: a Toyota RAV4 TSS2, openpilot's most mature platform; confirm at the start of the phase. Our own DBCs for ADAS and body
- [ ] Code generation from DBC files into Rust message types
- [ ] Bus model: arbitration, bit stuffing, load, CAN FD data phase; counters and checksums; fault-injection API
- [ ] Gateway ECU; chassis ECUs publishing wheel speeds, yaw rate, steering angle and driver inputs at real cycle times
- [ ] Actuator ECUs with the EPS model: rate limits, torque limits, fault states
- [ ] Recorder: every bus plus raw samples to MCAP; candump export
- [ ] Rerun views: bus load per network, CAN trace, vehicle-state plots, bird's-eye-view scaffold
- [ ] Overlay through ReShade: HUD skeleton and engineering panel (toggles, fault injection, latency readout)
- [ ] S7: cross-process GPU frame sharing, then detector timing on the RTX 4070 Ti; freeze the shim interface
- [ ] Borrow from openpilot (D13): its engagement rules and controller tuning notes, filed for Phase 2

**Gate:** live CAN trace and bus load per network; a recorded drive replays offline with identical bus traffic; S7 passes.

## Phase 2 · Front radar: ACC, FCW, AEB (14 to 18 days)

Goal: the first phase that feels like a modern car. It follows traffic smoothly and brakes decisively. It also lays the sensor, tracking and safety framework every later feature reuses.

- [ ] Sensor framework: mount poses, field of view, shared ray-budget scheduler, error model with datasheet parameters (D18)
- [ ] Front radar ECU publishing object lists on ADAS CAN FD; ground truth recorded beside it as the oracle (D6)
- [ ] Tracker: a Kalman filter per object, track confirmation and deletion, lead-vehicle selection
- [ ] Vehicle calibration drive (D19): throttle, brake and steering steps and sweeps fit a model per car into `config/vehicles/`
- [ ] Speed controller: PID with feedforward from the fitted model, jerk-limited
- [ ] ACC: set speed, time gap 1.0 to 2.5 s, stop-and-go, cut-in handling
- [ ] FCW and staged AEB: warn at about 2.5 s to collision, partial brake at 1.6 s, full at 0.9 s
- [ ] Engagement and override (D11): per-device thresholds; takeover escalation from visual to chime to a controlled stop with hazards
- [ ] Degradation matrix (D21), wired to the cluster
- [ ] Cluster ECU HUD: set speed, gap bars, lead-car icon, warnings
- [ ] Scenario mode v1: lead car brakes hard, a car cuts in, a stopped car past a bend

**Gate:** smooth stop-and-go in traffic; stops for the stopped car in every scenario run; radar path ≤ 30 ms at the 99th percentile.

## Phase 3 · Corner radars and ultrasonics (7 to 10 days)

Goal: 360° awareness at short range. This phase is fast because it reuses Phase 2's sensor framework and tracker.

- [ ] Four corner radar ECUs sharing the ray budget
- [ ] Blind spot detection: zones, mirror icons, a chime when signaling into an occupied lane
- [ ] Rear cross-traffic alert: predicts crossing traffic while reversing; warns, then brakes
- [ ] Twelve ultrasonics: park-assist distance arcs and tone; low-speed auto-brake
- [ ] Scenarios: a car overtaking in the blind spot; reversing out of a parking space into traffic

**Gate:** all three features pass their scenarios. Then decide on a public release (D14).

## Phase 4 · Map lanes: LKA and lane centering (10 to 14 days)

Goal: the car steers itself within its lane on the highway, using the road network as its map.

- [ ] Extract the road network from GTA's path nodes; build a local lane map (lane count, widths, centerlines)
- [ ] Steering controller through the EPS model: pure pursuit first, then Stanley or MPC if pure pursuit can't hold the gate
- [ ] Lane keeping (a nudge at the lane edge), then continuous lane centering
- [ ] Curve speed: ACC slows ahead of bends from map curvature
- [ ] Optional driver-monitoring toggle (D11)
- [ ] Scenarios: a long highway run, a tight bend, a lane that ends

**Gate:** holds a highway lane for 10 minutes with no takeover.

## Phase 5 · Vision and lidar (20 to 28 days)

Goal: the car sees for itself. The camera and lidar replace ground truth, and replay tests score how well they do.

- [ ] Forward lidar from the depth buffer: real beam pattern, stencil labels, point frames on the Ethernet channel
- [ ] Optional 360° raycast lidar (16 beams × 360 steps, 5 Hz)
- [ ] Camera path: shared frame → Apache-licensed detector (YOLOX or RT-DETR; pick after a quick benchmark) through `ort` + TensorRT
- [ ] Auto-label exporter: frames plus boxes and masks from the stencil buffer and ground truth
- [ ] `tools/train`: fine-tune, export ONNX, build the TensorRT engine; version each model with its data
- [ ] Camera ECU publishing objects and lanes on ADAS CAN FD; Windows screen capture as the fallback source (D3)
- [ ] Fusion of radar, camera and lidar tracks; pedestrian AEB
- [ ] Vision scoring against the ground-truth oracle in replay tests (D6)
- [ ] Experiment: openpilot's lane model against map lanes; keep whichever scores better

**Gate:** camera features run on vision, not ground truth; scores tracked per build; camera path ≤ 50 ms at the 99th percentile.

## Phase 6 · Full autopilot (20 to 30 days)

Goal: point-to-point driving across the city, with the driver supervising.

- [ ] Route planner on the road graph, with destination input from the map
- [ ] Behavior planner as a state machine: follow lane, change lane, approach junction, stop, turn
- [ ] Lane change assist on top of blind spot detection
- [ ] Traffic lights and stop signs from the camera
- [ ] Optional: openpilot as an alternative brain under WSL2 on the gaming PC (D13)
- [ ] Optional: steering wheel and pedals with force feedback, using the driver-torque signal (D15)

**Gate:** drives a city route with no takeover.

## Development workflow

All code lives in one private Git repository and is built and run on the gaming PC. For me to build, run and test it, Claude Code needs to run on that PC, not on the Docker server.

| Loop | Command | When |
| --- | --- | --- |
| Core change | `cargo xtask core` rebuilds and restarts `adas-core`; GTA keeps running | Most changes: seconds |
| Plugin or shim change | `cargo xtask deploy`, then restart GTA | Rare: about 2 minutes |
| Tests | `cargo xtask test`: unit tests plus every replay in `recordings/regression/` (D20) | Every build; no GTA needed |
| Phase gate | Scenario suite from `scenarios/`, run in the game | Before closing each phase |

**Rules for the machine**

- Launch story mode with BattlEye off (`-nobattleye`). Never go online with the mod installed.
- Pin the game build: back up the GTA folder once Phase 0 works, and update GTA only together with Script Hook V and `gta-edition`.
- A drive that finds a bug gets saved to `recordings/regression/`, so it becomes a test forever.
- Recordings, model files and TensorRT engines stay out of Git; only their names and versions are committed.
