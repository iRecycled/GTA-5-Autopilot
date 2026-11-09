# Progress: Phase 0 gate passed, Phase 1 underway

2026-11-09

This report covers the work since spike S3/S6: the Phase 0 build, and the first half of Phase 1 (ACC, FCW, AEB), tested in the closed-loop simulator and in GTA.

## What exists now

| Crate | What it does |
| --- | --- |
| `adapter-api` | Contract v3, in shared memory. Mailboxes for the vehicle sample (with driver inputs and button presses), the command (with AEB and FCW flags), the actor list (with line-of-sight bits), the sensor layout, the adapter info and the HUD. Also the core's process ID and epoch |
| `geom` | ISO 8855 conversions and ego-frame transforms |
| `control` | The speed controller: a comfort-limited reference, feed-forward plus PI, stop-and-hold, pedal hysteresis |
| `sensors` | The front radar model, with a deterministic error model (noise, misses, clutter, 20 Hz) |
| `fusion` | A Kalman filter per track, track confirmation and deletion, and lead selection along the predicted path |
| `features` | ACC (set speed, time gap, stop-and-go, gentle handling of cut-ins), plus FCW and staged AEB |
| `adas-core` | `Core::step`, which is deterministic. **Every input goes through the virtual vehicle's two buses** (`vbus`). Perception and AEB run every frame, and the engaged feature runs on top. It also has engagement rules, impact detection and an online-learned brake gain. Binaries: `adas-core` (live) and `replay` |
| `can` | DBC parsing, Intel and Motorola packing, Toyota checksums, the topology reader, and a bus timing model (real bitstream, FD data phase, arbitration, load, reactive and realistic delivery) |
| `record` | MCAP recordings of each whole core run. They survive the recorder being killed |
| `sim` | A closed-loop simulator that speaks the same contract: a road with a bend, a powertrain, brakes, scripted actors and occlusion |
| `scenario` | `scenarios/*.toml` with pass metrics, run by `cargo test` |
| `gta-edition` | Per-build offsets (1158.13 verified) and the pedal deadzone |
| `gta-plugin` | The GTA adapter. It handles: |
| | • **sensing:** actors and scheduled line-of-sight rays; |
| | • **control:** driver input read from Windows, angle-tracking steering, pedals, the handbrake hold, AEB; |
| | • **safety:** crash detection, the 6-frame watchdog and the D24 safe state; |
| | • **display:** the HUD and the test tools |

Driving it:
- `cargo xtask deploy` installs the adapter, and `adas-core --feature acc` runs the core.
- In the car: **F10** engages ACC, **F3** cancels, **Page Up / Page Down** change the set speed by ±5 km/h, and **End** cycles the time gap (not Home: that opens the Rockstar overlay). AEB is always armed.
- Test keys: **F11** spawns a lead car, **F5** makes it brake hard, and **F4** spawns a stopped car.

## Phase 0 gate

| Gate item | Result |
| --- | --- |
| Spikes pass | S1, S3, S4, S5 and S6 pass (see `docs/spikes/`) |
| The core drives the car | Yes: speed profile, test driver (I/J/K/L by-wire) and ACC |
| Adapter ≤ 1 ms per frame | Pass. Engaged and steering every frame: p50 about 123 µs, p99 177–328 µs. Not engaged: p50 126 µs, p99 341 µs. One frame of about 2.5 ms when a new vehicle is first steered (the memory check, cached after that) |
| Kill test: control back within 2 frames | Pass. Killed mid-turn (24 m/s, 0.14 rad steering) and mid-brake: released 1–2 frames after the last command, steering handed back at once |
| A recorded drive replays without GTA | Pass. 14,616 steps of a GTA ACC drive (radar model, tracker, ACC, AEB) replay bit for bit; killed recordings replay up to the kill |

## Phase 1 so far

### In the simulator (`cargo test -p scenario`): 46/46 runs pass, with every input and every command through the buses

| Scenario | Runs | Result |
| --- | --- | --- |
| ACC follows a slower lead at 2.0 s | 10 | Time-gap error 0.02 s (gate ≤ 0.3); jerk ≤ 2.5 m/s³ (gate ≤ 2.5); no AEB |
| A car cuts in close ahead | 5 | No AEB; jerk 2.9 m/s³ |
| The lead brakes hard to a stop | 10 | Stops 2–3 m behind it; jerk 2.7 m/s³ |
| Stop and go | 1 | Stops at the standstill gap, follows off again; no AEB; jerk 2.38 m/s³ |
| **AEB: stationary car from 50 km/h (gate: 10/10)** | 10 | **10/10**, stopped 6.3–6.7 m short |
| Stopped car past a bend at 70 km/h | 5 | 5/5 stopped (AEB) |

### In GTA

| Test | Result |
| --- | --- |
| ACC following a lead at 12 m/s | Settled at a **2.03–2.05 s** time gap (target 2.0 s) |
| The lead brakes hard (session 1) | ACC alone brought the car smoothly to a near-stop behind it, with no AEB needed |
| **AEB: stationary car from 50 km/h (gate: 10/10)** | **10/10** with the current tuning (15 stops in all, none hit). Stopped 6.6–7.6 m short at 5.1 m/s² average, about 1.8 s before impact. Peak deceleration is 6.8–6.9 m/s² |
| Vehicle profile (D19) | The learned brake gain (43.3 m/s² per unit for the Sultan) is saved to `config/vehicles/<host>/<model>.toml` and loaded at the next start. Without it, the first stop of a session peaked at 13.7 m/s²; with it, 6.9 m/s² |

### What the GTA runs taught (all fixed)

- **AEB requested pedal position, not deceleration.** GTA's brakes give about 11 m/s² at half pedal (the sim gives 4), so "partial" braking was a 1 g stop that ended 24 m short. AEB now requests a deceleration, and the core tracks it with feed-forward plus an integral on the measured deceleration. It also **learns the host's brake gain** on line: GTA reaches 4 m/s² at only about 5% brake.
- **AEB triggered too early,** at 3 m/s² required deceleration, and stopped 19 m short. Partial braking now starts at 4 m/s² required (requesting 5), full braking at 6.5 or less than 0.9 s.
- **AEB let go mid-stop.** Braking lowers the constant-speed TTC, so AEB stepped back down and the inattentive driver's foot took over. Once braking, AEB now brakes to a stop unless nothing is closing any more.
- **The AEB standstill hold never ended.** It now hands over after 3 s, or on either pedal.
- **ACC over-braked after a cut-in.** It now falls back at most 3 m/s below the lead and opens the gap gently.
- **Memory writes were slow in two ways.**
  - `WriteProcessMemory` costs about 2 ms a call: it changes page protection and flushes the instruction cache.
  - The `VirtualQuery` check that replaced it costs about 2 ms too, inside GTA's huge address space.

  The adapter now checks a memory region once and caches it, then writes directly at 0.2 µs per steering frame.
- **GTA's own AI shares the raycast pool.** When an async ray can't start, sync rays fill in.
- **`GetAsyncKeyState` doesn't work in another process while GTA has focus.** The core's test driver uses a low-level keyboard hook instead.
- **An impact while ACC was engaged.** In one GTA run the unsteered car drifted out of line with its lead and ran into the end of the taxiway, which the actor-list radar can't see. ACC then held full throttle at a standstill. An impact now cancels everything, as an airbag controller's crash signal would.
- **A false impact.** Spawning a car stalls GTA for about 70 ms, and in that frame the reported speed dropped 0.24 m/s in 14 ms of game time (17 m/s²). The first impact check (any frame beyond 1.5 g) took that for a crash and cancelled ACC. Impacts are now judged as crash sensing does, by speed lost: more than 1.5 m/s within 0.1 s. Hard braking loses about 1.1 m/s in that time; the glitch, 0.24.
- **The brake-gain learner learned from full braking.** At full brake the tyres, not the brakes, set the deceleration, so one AEB stop taught a gain of 26 instead of 43 and saved it to the profile. The gain is now learned only while the demand asks for less than 6 m/s², inside grip.
- **Police at the airport.** The taxiway is a restricted area; a one-off wanted-level clear at teleport didn't hold. Once a test tool is used (F7, F8), the adapter now keeps the wanted level at zero and the police ignoring the player.

## Open items

- **Recordings end "early"** because the core is always killed. A clean shutdown would finish the file (the reader already copes).
- **No test driver at the wheel.** ACC tests in GTA run unsteered on the airport taxiway (about 450 m usable), so a lead can drift out of line. Lane keeping (Phase 3) removes this; until then, keep runs short or let a human steer.
- **Static obstacles.** The actor-list radar doesn't see buildings, fences or the end of the taxiway. That's expected for D25; camera and lidar sensing (Phase 5) cover it.
- **Degradation matrix (D21): done.** See `docs/degradation.md`. Missing host capabilities, a silent chassis bus or a silent radar make features unavailable. Engaging is then refused, and the HUD says why ("ACC unavailable: …", "AEB OFF").
- **Buses: done so far:**
  - **Topology:** `config/topology.toml`, with the chassis bus and the ADAS CAN FD bus; the ADAS ECU sits on both.
  - **DBCs:** the TSS2 chassis DBC from opendbc, and our ADAS CAN FD DBC.
  - **Bus model:** the timing model.
  - **Chassis ECUs:** wheel speeds, yaw rate, steering angle, speed and pedals, at TSS2 cycle times.
  - **Radar ECU:** its object list goes out on CAN FD.
  - **ADAS ECU:** reads both buses.
  - **Load:** the chassis bus runs at about 8–9% load.
  - **Radar path:** a scan arrives in the same or the next frame, within the 2-frame gate.
  - **Actuation over the bus (D28).** Commands no longer go from the features to the adapter directly. The ADAS ECU sends three messages on chassis CAN, as a TSS2 car's ADAS does:
    - `ACC_CONTROL`, the acceleration request (0.001 m/s² steps) and the standstill hold;
    - `PRE_COLLISION_2`, AEB's deceleration request;
    - `STEERING_LTA`, the steering-wheel angle request.

    Two actuator ECUs act on them:
    - the **powertrain/brake ECU** turns the acceleration into pedals through the car's calibration (the vehicle profile), runs AEB's closed deceleration loop and holds the car at a standstill;
    - the **EPS** steers the road wheels within its rate (0.8 rad/s) and range (0.6 rad) limits, starting from the measured angle so engaging doesn't jerk the wheel.

    Requests not heard again for 0.1 s lapse, so a silent ADAS ECU releases the car. All the scenarios still pass. Replaying the old GTA drive through the new path changes pedals by at most 0.00012, which is the bus resolution, and changes no flags.
  - **Fault injection.** Each bus can drop, corrupt (a CAN CRC error: the frame takes its bus time and nobody receives it) or delay frames, by message, with a probability and a time window. Faults are deterministic (seeded per bus), so a faulted run replays exactly. Scenarios script them with `[[fault]]` tables, and health checks now count only valid frames. Three fault scenarios pass:
    - **radar silent for 2 s while following:** ACC disengages with "radar blocked";
    - **20% CRC errors on chassis CAN plus a radar 40 ms late:** ACC follows as before (time-gap error 0.01–0.02 s) and stays available;
    - **`ACC_CONTROL` lost while the lead brakes hard:** AEB's own message still gets through and stops the car 6.1 m short. This scenario found a bug: losing `ACC_CONTROL` timed out AEB's request too. Each message now has its own timeout.
  - **candump export:** `replay --candump` writes the regenerated traffic, including the actuation frames, in `candump -L` format.
- **Still to do in Phase 1:**
  - DBC code generation (the runtime DBC is used instead);
  - a live CAN trace and bus-load view;
  - D11 takeover escalation;
  - the egui panel and Rerun views.
