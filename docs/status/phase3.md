# Progress: Phase 3 (camera ECU and lanes) built; its GTA gate is not passed yet

2026-11-09

Phase 3 adds a forward camera ECU and the features on it: lane keeping (LKA), lane centering (inside the autopilot, `--feature autopilot` = ACC + lane centering), curve speed and pedestrian AEB. Driving it in GTA added junction AEB, radar–camera fusion for stopped vehicles and a series of safety checks. Every item on the plan's checklist is built. The simulator scenarios pass. In GTA, lane centering meets its accuracy target, but no gate drive has yet run its full length.

## What changed

| Layer | Change |
| --- | --- |
| Contract (`adapter-api`, v6) | **Lane map** (`caps::MAP`): up to 8 lines as point lists in the ego frame, with kinds (dashed, solid, edge) and the speed limit. The HUD gains the camera's lane lines, the heading arc (curvature and length) and the plan (the lane centre being steered along), plus message codes for the lane reasons. The shared-memory link is named after the contract version (`Local\adas-link-c6`), so a core or adapter left running from another version can't share it. Recordings from v3–v5 still replay |
| GTA adapter | **Map capability:** the road's reference line is traced with `GET_CLOSEST_ROAD`, 10 m steps ahead in the direction of travel, with lane lines laid out from lane counts. `LANE_WIDTH` = 5.0 m comes from a survey of where AI traffic drives on the Del Perro freeway (`gta-tests/survey/`; `%LOCALAPPDATA%\adas\lane_survey` turns the logging on). Path lines on the road: amber for the heading arc, blue for the plan. **F2** starts the core from the game; **F10** with no core starts one and engages once it answers (`launch.rs`, configured by `adas-core.txt`, which `cargo xtask deploy` writes). Test tools: `pullout`, `walker`, `ai` |
| Sensors | **Camera ECU** (`sensors::camera`): the four nearest lane lines fitted as cubics, with noise and occasional misses; vehicles (to 120 m) and pedestrians (to 60 m), ±50°, with misclassification and worse range accuracy than radar. All sensors now have a **vertical field of view** (camera ±25°, front radar ±10°, corner radars ±15°) and report only objects **standing on the road** (2.5 m + 12 % of the range). Without that, GTA's birds (pedestrians to the game) and people on bridges triggered AEB |
| Fusion | Pedestrian tracks from the camera. **Radar–camera fusion:** a front-radar track that the camera reports a car or truck on is `camera_vehicle`, and ACC follows it even if it was never seen moving. Surround tracks carry an acceleration estimate |
| Features | `Lanes`: lane centering by pure pursuit, through the EPS model; LKA nudges; curve speed; "lane ends". **Steering response learned per speed band** (`SteerResponse`), and a curvature loop that scales the steering by what the car actually turns. **`NotInLane`** and the lane-jump check (below). `junction`: **junction AEB** for traffic crossing the path ahead, from the front corner radars, ignoring traffic that clears, waits, or brakes to give way. Forward AEB no longer brakes for a car sweeping through the lane that will clear before the car gets there |
| Core | Lane availability and takeover reasons (`docs/degradation.md`); the optional driver monitor (D11); the heading arc for the HUD. Recording and telemetry run on a writer thread, the core runs at high priority, and a core exits when a newer one takes the link. Brake-gain learning is guarded (below). `--exit-when-idle` |
| Sim | Roads from pieces (straights and arcs), lane ends, a lane-map host, pedestrians, crossing traffic. **`vehicle = "gta"`**: understeer that grows with speed (characteristic speed 15 m/s) and a 0.1 s yaw lag, as measured on GTA's Sultan. Timed set-speed changes (`[[ego.set]]`) |
| Scenarios | 37 files. New for Phase 3: a highway loop, a tight bend, a lane that ends, LKA drift and its control run, the driver monitor, pedestrians stepping out and waiting on the kerb, 30 minutes of autopilot in traffic, junction pull-outs from both sides with a control run, traffic that doesn't conflict, a car already stopped ahead, and GTA-like cars slowing after the highway and speeding up after town |
| GTA tests | Lane centering on a country road and for 5 minutes at Sandy Shores; pedestrian AEB; junction pull-out; ACC behind a stopped car; `regression_drive.toml` (scripts the regression recording). The runner fails runs that drop out (`min_engaged`) and can re-engage after an AEB stop like a driver (`resume_after_aeb`) |

## Gate

| Gate item | Sim | GTA (Sultan) |
| --- | --- | --- |
| **Lane-centre error ≤ 0.3 m RMS on a named highway loop** | 11 km highway loop passes; GTA-like car 0.031–0.041 m after speed changes | Country road: **0.10 m**. Sandy Shores: 0.18–0.20 m over the stretches driven, but no run has reached 5 minutes (below). Not yet run on a highway loop |
| **≤ 1 takeover per 10 km** | Highway loop: 0 | Not measured: no run covered 10 km |
| **Pedestrian AEB stops in 10 of 10 runs at 30 km/h** | 10/10; no braking for people on the kerb | `gta-tests/ped_aeb.toml` exists; not run 10 times |
| **≤ 1 false AEB per 30 min in traffic** | 30 min of autopilot in traffic: 0 | Not measured |
| No regressions | All 37 scenarios pass | The regression drive's brake commands differ slightly (the slower brake-gain learning): re-record with `gta-tests/regression_drive.toml` |

## What GTA taught

- **The core stalled behind I/O.** The first gate drive lost control after 2 minutes: the core missed 6 frames (97 ms) and the adapter's watchdog released the car. Replayed, the core's step never took more than 0.11 ms. The control thread was waiting, not working. Recording and telemetry moved to a writer thread, and the core runs at high priority. No stall since.
- **GTA's animals and bridges are pedestrians.** AEB braked for a seagull 47 m up, then for someone on a freeway bridge 14.6 m above the lane. Fixed by giving sensors a vertical view and a road-plane check (above). Animals as their own class are planned for Phase 5.
- **Junctions.** A quad from a side road drove into the car. Junction AEB now brakes for crossing traffic; replayed, it would have braked 1.6 s before that impact. In a later drive it stopped the car for an oncoming car turning across the lane, which it would otherwise have passed within about a metre. Its first false alarms came from cars slowing to give way, which looked set to cross at constant velocity; it now uses their deceleration.
- **GTA's steering fades with speed.** For the same wheel angle, the Sultan turns 0.75 of the bicycle model's curvature at 35 km/h, 0.27 at 80 and 0.15 at 120, with its yaw about 0.1 s behind the wheels. One learned gain for all speeds made the car weave out of its lane in town after the highway (a manual drive's crash) and run wide on the highway after town. Learning per speed band, plus a curvature loop that settles in about a second however wrong the gain is, fixed both in the simulator. In GTA the car then held within ±0.2 m for 4,000 frames.
- **Lane centering followed a lane the car wasn't in, into the Alamo Sea.** An AEB stop left the car across the centre line. The test driver re-engaged, the camera's lane flipped to the oncoming lane (its centre moved 5 m in one frame), and the car followed that lane off the road. Lane centering now neither steers nor engages with the car outside its lane, and doesn't trust a lane centre that jumps more than 1 m between frames.
- **Learning from messy driving.** Manual drives taught the Sultan's brake gain 85 (from 43) and a 9F's 110, because the core's small brake demand was credited with the driver's braking. Learning now needs the driver off the pedals and the demand steady, adapts more slowly, and stays within 1.3× of a saved profile.
- **Stopped cars.** ACC followed only vehicles seen moving, as radar-only ACC must, so a car already waiting at a stop sign was left to AEB. With camera confirmation, ACC now stops 4–5 m behind it, with no AEB (in the simulator and in GTA).
- **Test harness lessons.** One correct AEB stop used to end a whole gate drive, because the scripted driver never re-engaged. A test once drove off the airport taxiway, because our sensors don't see static geometry such as walls and fences. A core left running from an older build held the shared memory at the old layout, which is what led to versioning the link's name.

## Open

In `docs/implementation-plan.md` (Phase 3, open issues): confirm in GTA that lane centering and brake learning hold over full drives; run the gate on a highway loop; pedestrian AEB 10 of 10 and false AEB per 30 min in GTA; static obstacles; re-record the regression drive. Passing an obstruction and right of way at junctions are planned for Phase 6, and camera 3D boxes for Phase 5.
