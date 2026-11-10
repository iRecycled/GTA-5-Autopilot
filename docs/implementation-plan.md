# Implementation plan

2026-11-08 · Nick Peterson · reordered 2026-11-09 (the sensor boundary, D30)

Eight phases, each ending in something drivable. Phase 0 starts with throwaway spikes before any production code. The plan is ordered by risk and by when you can first drive, not by dates.

**The 2026-11-09 reorder.** Phase 3's camera read lanes laid out from GTA's road nodes, and its lane accuracy was scored against those same lanes. It drove, but it wasn't realistic, and it wouldn't pass the CARLA test (PRD): give the core a car's sensors in CARLA, and it drives as it does in GTA. Phase 3 closes without its gate. Its controllers, checks and junction AEB carry over. Next come the sensor boundary and a second host (Phase 4), then the camera on vision (Phase 5), which re-runs Phase 3's gate on real perception. Lidar, the full network and calibration follow (Phase 6), then the full autopilot (Phase 7).

## Repository layout

One Cargo workspace. Each crate owns one concern, so a phase adds crates instead of reworking old ones. Two rules hold the layout together:

- **Only the `gta-*` crates and `reshade-shim` know GTA exists.** A new host game gets its own adapter crates; nothing else changes (D22).
- **The world side and the car side (D30).** Only `scenario`, `sim` and `sensor-models` (the world side) may see ground truth, and `sensor-models` only to make raw sensor data for a sensor the host lacks. The car side (`sensors`' ECUs, `perception`, `fusion`, `features`, `control`) sees only raw sensor data and bus messages. The crate dependency graph enforces it. The one labelled exception: the closed-loop sim may hand the car side ideal camera output, as a controller bench, and no perception gate is scored on it.

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
| `sensors` | lib | Car side: radar ECUs (clustering and tracking detections), the ultrasonic ECU, the camera ECU's bus output (D25, D26). Phase 1–3 sensor models move out to `sensor-models` in Phase 4 | 1 |
| `sensor-models` | lib | World side: makes radar detections from ground truth for hosts without radar, with the error model (D18, D25, D30); the ideal camera for the sim's controller bench | 4 |
| `carla-adapter` | bin (any language) | The CARLA adapter: speaks the contract over shared memory, drives CARLA in synchronous mode (D31) | 4 |
| `fusion` | lib | Tracking and prediction | 1 |
| `features` | lib | ACC, FCW, AEB, BSD, RCTA, park assist, LKA; engagement; degradation matrix (D11, D21) | 1 |
| `sim` | lib + bin | Closed-loop simulator implementing the adapter contract: bicycle-model ego car, scripted actors, no game (D20) | 1 |
| `scenario` | lib | Interprets `scenarios/*.toml` through adapter primitives; pass metrics (D27) | 1 |
| `viz` | lib | Rerun logging | 1 |
| `ui` | lib | egui engineering panel (D8) | 1 |
| `perception` | lib | Car side: GPU interop, `ort` + TensorRT inference, lanes, road edges, the driving path and objects from images; depth-buffer lidar later | 4 / 5 |
| `reshade-shim` | C++ add-on | GTA colour (then depth and stencil) capture into a shared ring; the fixed C interface (D3, D4) | 4 |
| `planner` | lib | Routes and behavior | 7 |
| `tools/train` | Python | Fine-tuning pipeline → ONNX (D7); the lane-truth labelling tool (D32) | 5 |

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
| sample | adapter → core | Mailbox (latest wins) | Vehicle state, range-fan hits, radar detections (hosts with radar), driver input, navigation map, pause state | Every frame |
| oracle | adapter → core | Mailbox (latest wins) | Ground truth: actors, line-of-sight bits per sensor, lane truth where the host has it. Read only by the recorder, `sensor-models` and `scenario` (D30, from Phase 4) | Every frame |
| command | core → adapter | Mailbox (latest wins) | Steering angle, throttle, brake, engage state, the sample `seq` it answers | Every step |
| scenario | core → adapter | Ring (lossless) | Spawn, task, follow profile, teleport, weather and time, freeze, clear | On demand |
| scenario-reply | adapter → core | Ring (lossless) | Actor handles, errors | On demand |
| hud | core → adapter | Mailbox (latest wins) | HUD state: set speed, gap, lead car, warnings, spotter ring | 30 Hz |
| frames | adapter → core | Mailbox (latest wins) | GPU frame slot index, fence value, camera pose and projection (Phase 4) | Every camera frame |

All positions are ISO 8855 vehicle coordinates and SI units (D16). Each adapter does the one conversion from its game's frame.

**Mailbox:** a seqlock over one slot; the writer never waits and the reader always gets the newest complete value. **Ring:** single producer, single consumer, never drops. **Epoch:** incremented whenever the core starts; the adapter treats a new epoch as a fresh, disengaged core.

### Graphics shim C interface (drafted at the start of Phase 4, frozen after S7)

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

- [x] Four corner radar ECUs per D25, on ADAS CAN FD (our own object-list DBC layout)
- [x] Blind spot detection: zones, mirror icons, a chime when signaling into an occupied lane (the spotter ring is not drawn yet)
- [x] Rear cross-traffic alert: predicts crossing traffic while reversing; warns, then brakes
- [x] Twelve ultrasonics on ray fans: park-assist distance arcs and tone; low-speed auto-brake
- [x] Scenarios in the sim and in GTA: a car overtaking in the blind spot; reversing out of a parking space into traffic; parking against a wall

**Gate:** BSD detects 10 of 10 overtakes with ≤ 1 false alert per 30 min; RCTA brakes for crossing traffic in 10 of 10 runs; park assist stops ≥ 0.3 m from the obstacle in 10 of 10 runs. Then decide on a public release (D14): decided 2026-11-09, stays private for now.

## Phase 3 · Lane control on map lanes (closed without its gate)

Goal: the car steers itself within its lane, with a camera ECU that already behaves like a production one on the bus (D26).

**Closed 2026-11-09 without its gate (D30).** The camera's lanes were laid out from GTA's road nodes with one guessed lane width, and the lane-centre error was measured against those same lanes, so a good score didn't show the car was in GTA's painted lane. What carries over: pure pursuit through the EPS model, the steering response learned per speed band and the curvature loop, the lane checks (not in its lane, lane jumps, lane ends), LKA's rules, curve speed's rule, junction AEB, the driver monitor, the GTA test harness. What is retired on hosts: the ground-truth camera (lanes, objects, the speed limit). Its open issues move to Phase 4 or 5, as marked below. The gate moves to Phase 5, on vision.

- [x] Map capability in the GTA adapter: road network from path nodes; local lane map (lane count, widths, centerlines)
- [x] Camera ECU: objects, lanes and the speed limit within its field of view, through a camera error model (worse range accuracy than radar, misclassification, a vertical field of view, objects only on the road); published on ADAS CAN FD. Not yet: weather and night effects, other signs
- [x] Radar–camera fusion in `fusion`; pedestrian AEB
- [x] Junction AEB: braking for traffic crossing the path ahead, from the front corner radars, ignoring cars braking to give way (added after a GTA traffic car pulled out into the gate drive)
- [x] Steering controller through the EPS model: pure pursuit (it holds the gate's RMS so far: 0.18–0.20 m in GTA)
- [x] Lane keeping (a nudge at the lane edge), then continuous lane centering
- [x] Curve speed: ACC slows ahead of bends from lane curvature
- [x] Optional driver-monitoring toggle (D11)
- [x] Scenarios: a long highway run, a tight bend, a lane that ends, a pedestrian stepping out, junction pull-outs
- [x] HUD path lines: where the car is heading (3 s) and the lane centre it steers along
- [x] Start the core from the game (F2, or F10 with none running)

**Open issues, found driving in GTA:**
- [x] Lane centering loses the road and doesn't get back to the lane centre (seen driving manually, 2026-11-09; recording `recordings/1794280513-autopilot.mcap`). Cause: GTA's steering fades with speed (0.75 of the bicycle model at 35 km/h, 0.15 at 120) and one learned gain served all speeds. Fixed in simulation (steering learned per speed, a curvature loop, a GTA-like car in the simulator); to confirm in GTA
- [x] The learned brake gain ran away (43 to 85; a 9F to 110) while the driver braked over a small demand. Now learned only with the driver off the pedals and the demand steady, slower, and within 1.3x of a saved profile; to confirm in GTA
- [x] ACC ignored a car already stopped at a stop sign (never seen moving), and AEB braked for it. The camera now confirms stopped radar targets as vehicles, and ACC stops behind them; to confirm in GTA (`gta-tests/acc_stopped_car.toml`)
- [x] Lane centering steered for a lane the car wasn't in (after an AEB stop left it across the centre line, the camera's lane flipped to the oncoming one and the car followed it into the Alamo Sea). Now: no steering or engaging while the car is outside its lane ("car not in its lane"), and a lane centre that jumps over 1 m between camera frames isn't trusted
- [ ] → Phase 4. Save the learned steering response (per speed band) in the vehicle profile, next to the brake gain
- [ ] → Phase 4. The adapter's measured full steering lock reads 0.579 rad for nearly every car (and 0.728 and 0.596 for the same Sultan): find out whether it's a default or a measurement fault
- [ ] → Phase 4 (S9). Static obstacles: the radars see only actors, not walls or fences; a GTA test drove into the airport's perimeter at 30 km/h
- [ ] → Phase 4. Re-record the GTA regression drive, after the radar moves to detections
- [ ] → Phase 4. An impact at 31 km/h in that drive, just after blind-spot alerts on both sides (frame 70431): find out what hit what
- [ ] → Phase 5. Gate runs: lane centering on a named highway loop, pedestrian AEB 10 of 10, false AEB per 30 min, on vision

**Gate (not passed; moved to Phase 5):** lane-centre error ≤ 0.3 m RMS on a named highway loop; ≤ 1 takeover per 10 km; pedestrian AEB stops in 10 of 10 runs at 30 km/h; ≤ 1 false AEB activation per 30 min of driving in traffic (moved from Phase 1). The GTA runs gave 0.10–0.20 m, but against the map lanes the car itself followed, so they don't count.

## Phase 4 · The sensor boundary and a second host

Goal: hosts give the core raw sensor data only, and two hosts prove it. After this phase nothing on the car side can see game data, and the CARLA test can run (D30, D31).

**Spikes**

- [ ] S2b: hidden forward sensor camera in GTA (D29); decide between interleaving, the player's own view, or a hood view while engaged
- [ ] S7: shared frame ring to the core and CUDA; freeze the shim interface
- [ ] S9: static radar returns in GTA from a ray budget per radar (walls, guardrails, fences, parked cars)

**Contract and core**

- [ ] Contract v7: the `frames` channel live; radar detections per mount (range, azimuth, elevation, radial velocity, strength); ground truth moved to the `oracle` channel; `caps::MAP` becomes the navigation map (road graph, lane counts, speed limits, no lane geometry)
- [ ] Split the core (D30): `sensor-models` on the world side; the car side may not depend on it or on the oracle, enforced by the crate graph
- [ ] Radar ECUs on detections: cluster returns into objects, with no class from radar; the world-side model adds clutter, ghost returns and multipath; the tracker and features downstream unchanged
- [ ] Retire the ground-truth camera on hosts: lane centering, LKA, curve speed, pedestrian AEB and camera-confirmed stopped cars become unavailable without camera features ("camera not available" in the degradation matrix), and the autopilot runs as ACC; the ideal camera stays in `sim` only, labelled as the controller bench
- [ ] GTA's map lanes kept as a recorded overlay on the oracle channel, for comparison only
- [ ] Recording tags per frame, for slicing drives by where they went wrong: road class and lane count (navigation map), junction, curvature, speed, engaged features
- [ ] Carried from Phase 3: save the steering response in the vehicle profile; the steering-lock reading; the 31 km/h impact; re-record the regression drive after the radar change

**Second host (D31)**

- [ ] `carla-adapter`: vehicle state, controls (the steering angle converted to CARLA's normalised steer by the car's lock), driver input, camera frames, radar detections, ultrasonic range fans from short rays, the navigation map from CARLA's road graph, scenario primitives; CARLA in synchronous mode, so one tick is one sample (D23)
- [ ] Lane truth and object truth from CARLA on the oracle channel
- [ ] Vehicle profiles for two CARLA cars (D19)
- [ ] Phase 1–2 scenarios on CARLA: ACC follow and stop-and-go, AEB on a stopped car, BSD overtakes, RCTA, park assist

**Gate:** GTA and CARLA both deliver camera frames and radar detections; every Phase 1–2 scenario passes on both hosts with no change outside the adapters; no regressions in GTA (the re-recorded regression drive replays); GTA's sensor camera ≥ 20 Hz within its fps budget (S2b); the static returns find the airport fence (S9).

## Phase 5 · The camera on vision

Goal: the camera ECU sees lanes, road edges, vehicles and pedestrians in images, on both hosts, and lane centering comes back on it. Phase 3's gate runs here, on real perception, against truth the car didn't drive on.

- [ ] Lane truth: a hand-labelled frame set from a named GTA highway loop, with a labelling tool in `tools/train`, and AI traffic's positions as a second check; CARLA's lane truth from its map; a scorer in replay
- [ ] S8: lane perception candidates (openpilot's driving model, a pretrained lane detector, one trained on CARLA's lane-marking labels) scored on both sets; decide D32
- [ ] Camera ECU on vision: lane lines, road edges and a confidence per line on ADAS CAN FD (a DBC change), otherwise the same lane messages as Phase 3; lines are lost behind trucks, at crests and at junctions, as the image shows
- [ ] Driving path: the lane centre from lines and road edges, blended with the lead car's trail, with a confidence; lane centering steers along it, and asks the driver to take over when the confidence stays low
- [ ] Objects from images: a pretrained detector (D7's fallback) for vehicles and pedestrians, with range from the ground plane and radar fusion; fine-tuning comes with auto-labels in Phase 6
- [ ] Camera objects with heading and length (3D boxes): radars can't tell which way a stopped vehicle points
- [ ] Animals as their own class (GTA counts birds as pedestrians)
- [ ] Radar–camera fusion on vision objects: pedestrian AEB and camera-confirmed stopped cars come back
- [ ] Curve speed from the vision lanes; speed-limit hints from the navigation map
- [ ] Lateral comfort metrics in scenarios and GTA tests: lateral jerk, weaving about the lane centre, overshoot after bends
- [ ] Gate runs in GTA and CARLA

**Gate:** lane-centre error ≤ 0.3 m RMS against independent lane truth on a named highway loop, in GTA and in CARLA; lateral jerk ≤ 5 m/s³ and no sustained weaving; ≤ 1 takeover per 10 km; pedestrian AEB stops in 10 of 10 runs at 30 km/h; ≤ 1 false AEB per 30 min in traffic; camera path ≤ 3 frames at the 99th percentile.

## Phase 6 · Lidar, the full network and calibration

Goal: depth sensing, trained vision, the rest of the vehicle network, and fitted vehicle models.

- [ ] S2: depth and stencil through ReShade, with the frame-alignment check
- [ ] Forward lidar: points at beam angles from GTA's depth buffer with stencil labels, native in CARLA; point frames on a modelled Ethernet channel
- [ ] Auto-label exporter: frames plus boxes and masks from the stencil buffer (GTA) and semantic labels (CARLA)
- [ ] `tools/train`: fine-tune lyuwenyu/RT-DETR or YOLOX (pick after a quick benchmark), export ONNX, build the TensorRT engine; version each model with its data
- [ ] Vision scoring against the oracle in replay tests, per build (D6)
- [ ] Fusion of radar, camera and lidar tracks
- [ ] Body CAN and a gateway ECU routing only what each bus needs (D9, D17); turn signals, hazards and cluster on body CAN
- [ ] Automatic calibration drive: throttle, brake and steering steps and sweeps fit a model per car (D19)
- [ ] Optional: Foxglove layouts for replays

**Gate:** lidar scored against the oracle on both hosts; calibration fits 3 GTA cars and 2 CARLA cars; vision scores tracked per build.

## Phase 7 · Full autopilot

Goal: point-to-point driving across the city, with the driver supervising.

- [ ] Route planner on the navigation map, with destination input from the map
- [ ] Behavior planner as a state machine: follow lane, change lane, approach junction, stop, turn
- [ ] Right of way at junctions: give way where the car's road must, rather than leaving it to junction AEB
- [ ] Pass an obstruction: when a stopped vehicle sticks into the lane and shows no intent to move, shift within the lane, or over a dashed centre line when the oncoming lane is clear, at walking speed, then re-centre (uses the camera's 3D boxes from Phase 5)
- [ ] Lane change assist on top of blind spot detection
- [ ] Traffic lights, stop signs and speed-limit signs from the camera
- [ ] Optional: openpilot as an alternative brain under WSL2 on the gaming PC (D13)
- [ ] Optional: steering wheel and pedals with force feedback, using the driver-torque signal (D15)

**Gate:** a named city route at a named traffic density, in GTA and in CARLA; takeovers per km recorded and tracked as a target, not a pass or fail.

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
