# Progress: Phase 2 (surround) built; its gates pass in the sim and in GTA

2026-11-09

Phase 2 adds 360° short-range sensing: four corner radars and twelve ultrasonic sensors, and three features on them: blind spot detection (BSD), rear cross-traffic alert and braking (RCTA) and park assist. Like AEB, all three run whenever they are available; nothing is engaged.

## What changed

| Layer | Change |
| --- | --- |
| Contract (`adapter-api`, v4) | **Range fans:** a standing `FanLayout` (core → adapter; `fan_flags::ACTIVE` only at parking speeds) and a `RangeReport` with the hit distance per ray. The sample gains `REVERSE`, `TURN_LEFT` and `TURN_RIGHT` flags. New capabilities: `RANGE_FANS`, `TURN_SIGNALS`, `GEAR`. The HUD gains surround flags, the twelve park distances and the park tone. Recordings from contract v3 still replay: none of the recorded layouts changed |
| Sensors | Corner radars use the front radar's model (`Radar`, `RadarConfig::corner()`): 80 m, ±75°, noisier, with more misses and clutter. They see an object where its box is nearest them and report its centre and size, with their own 16-bit object ids. Ultrasonics (`Ultrasonics`) give the nearest echo per fan at 25 Hz: 1 cm + 1 % noise, a 0.15 m blind zone, dropouts held for two cycles, and rare ghost echoes |
| Buses | ADAS CAN FD: each corner radar has its own status and object ids (0x310–0x317, signed `DIST_X`), and the park ECU sends `PARK_SENSORS_FRONT` and `PARK_SENSORS_REAR` (0x340–0x341). Chassis CAN: the TSS2 `GEAR_PACKET` (20 Hz) and `BLINKERS_STATE` (10 Hz). Topology: five new ECUs |
| Fusion | `SurroundTracker`: one track list from the four corner radars. It associates each radar's objects by (radar, id), new pairs by gating, so a car passing alongside stays one track as it is handed from radar to radar. Tracks are deleted by age, since the radars scan out of phase. Ego motion is signed (reversing) in both trackers |
| Features | `Bsd` (ISO 17387-style zones, closing vehicles, suppression of stationary and clearly-passed vehicles, a warning when signalling into an occupied side), `Rcta` (predicts each crossing vehicle's strip across the car's path; warns, then brakes at the last moment to stop short of that lane), `ParkAssist` (display, tone, low-speed braking on the four path sensors per bumper, two-cycle echo confirmation, hold, and re-arming so the driver can creep closer) |
| Core | All of the above through the buses. RCTA and park braking use AEB's deceleration request on `PRE_COLLISION_2`. Degradation rows are in `docs/degradation.md`. ACC is unavailable in reverse, and AEB ignores what's ahead while reversing. Per-feature toggles (`Core::set_toggles`). Events when the features act |
| Sim | Reversing, the turn signal, actors that drive in a fixed direction (crossing traffic), static obstacles (walls), ray-fan answers, oriented-box collisions, a traffic generator. Sensor coverage is judged by an actor's nearest box point |
| Scenarios | 12 new files, 82 runs (128 with Phase 1's), including two **control runs**: the same setups with RCTA or park assist switched off must all collide, so the scenarios really depend on the features. Runner metrics: BSD scored against ground truth (detections, latency, false alerts), RCTA and park activations, the room left to the obstacle |
| GTA adapter | Range fans by synchronous shape tests (round-robin, at most 10 rays per frame, only while asked). Reverse detection and pedal mapping (in reverse GTA's brake key drives backwards). The turn signal on `,` and `.` with the indicator lamps. The surround HUD (mirror icons, cross-traffic arrows, a park diagram with beeps). Corner sensor coverage by nearest box point. Test tools: `overtake`, `cross`, `obstacle`, `signal`, `creep` |
| Views | Telemetry carries surround tracks (flagged), the surround flags, the park distances and tone, and the ego box; older telemetry still decodes. `adas-viz` draws the BSD zones (lit while occupied), corner tracks by role and park arcs. `adas-panel` shows the surround state and park distances |

## Gate

| Gate item | Sim | GTA (Sultan) |
| --- | --- | --- |
| **BSD detects 10 of 10 overtakes** | 10/10 on the left (overtake, with the signal warning), 5/5 on the right; worst time to light up 0.03 s | **10/10**, each with the signal warning |
| **≤ 1 false BSD alert per 30 min** | **0** in 30 min of four-lane traffic at 100 km/h, with parked cars on the shoulder. Every one of the 63 vehicles truly in a blind spot was reported, worst latency 0.15 s | Not run: a 30-minute drive needs lane keeping (Phase 3), as with Phase 1's false-AEB rate |
| **RCTA brakes for crossing traffic in 10 of 10 runs** | 10/10 from the right (reversing out between two parked cars), 5/5 from the left at 25 km/h, no collisions; warns first. No braking for traffic 13 m behind (5/5). Control: RCTA off, 10/10 collide | **10/10**: warns, then brakes at 6 km/h; the crossing car is never hit |
| **Park assist stops ≥ 0.3 m from the obstacle in 10 of 10 runs** | Reversing toward a wall 2.5–5.5 m back: 10/10, stopped 0.80–0.86 m short. Forward: 10/10, 0.75–0.81 m. No braking reversing 60 s across open ground. Control: park assist off, 10/10 hit the wall | **11/11** toward a parked van, stopped 0.78–0.85 m short |
| No regressions | All Phase 1 scenarios pass; the GTA regression drive replays identically (6,264 steps) | All five Phase 1 in-game tests pass: time-gap error 0.04 s, stopped 5.1 m behind the braking lead, AEB 6.9 m short |
| Adapter ≤ 1 ms per frame | — | p50 about 136 µs; the median per-second p99 is 294 µs with the fans running. Teleports and model loads still stall GTA, as in Phase 1 |

## What the work taught

- **Reporting the nearest point of a box broke BSD.** The first corner radar model reported where the box was nearest the radar. A car alongside then looked to the front corner radar like it was level with the ego's front bumper, ahead of the B-pillar, while its tail was still in the zone. Over 30 minutes BSD reported only 36 of 63 truly occupied spells. Object-level radars report a reference point and a size, so the corner radars now report the centre, and BSD and RCTA reason about the object's box (`SurroundTrack::extent`, heading from the direction of travel). That also gives every radar the same point for one car. Since the change, 63/63 are reported.
- **Coverage by centre missed cars beside the rear corners.** The sim, and the GTA adapter with the same logic, decided which sensors covered an actor from its centre. A car right beside a rear corner had its centre just outside that radar's beam, so it got no line-of-sight bit. Corner sensors now judge coverage, and aim their ray, by the nearest point of the box. The front radar keeps the Phase 1 behaviour, so the regression replay is unchanged.
- **RCTA braked too late to avoid the crossing car's lane.** Braking 1.2 s before the crossing car arrived stopped the ego car standing in its lane. In 8 of 15 sim runs the crossing car then drove into it. RCTA now predicts the crossing car's strip across the path and brakes when the bumper is as close to that strip as the car needs to stop.
- **The GTA test driver was too fast.** Holding GTA's reverse key at 30 % reaches 16 km/h in two seconds, above the 10 km/h that RCTA and park assist work to, so both in-game tests failed at first. The test driver now creeps at a held parking speed (`creep back 1.5 SECS`).
- **The reverse latch outlived the manoeuvre.** After a reversing test the car stood still with reverse still latched, so every following ACC test was refused ("not available in reverse"). In GTA the key pressed at a standstill picks the direction, so the adapter now does the same, as a gear lever would. A new car or a teleport also clears the latch.

## Open items

- **D14, release:** decided at the end of Phase 2: the project stays private for now.
- **The spotter ring isn't drawn.** The HUD has mirror icons, cross-traffic arrows and the park diagram.
- **Only the Sultan in GTA.** The Phase 1 tests also ran on the Baller; the surround tests haven't.
- **Checked in GTA by hand:** the turn-signal keys (`,` and `.`) and their lamps work. Not yet checked by eye or ear: the HUD layout and the sound names (`ERROR` and `NAV_UP_DOWN` from `HUD_FRONTEND_DEFAULT_SOUNDSET`).
- **Ultrasonic rays on slopes.** The rays are horizontal at bumper height and include the map, so a steep slope could echo as an obstacle. The taxiway is flat; a ramp test would show it.
- **Pedestrians.** The actor list holds vehicles only, so the corner radars and RCTA don't see pedestrians. The ultrasonic rays do hit them.
- **A front corner radar going silent** costs BSD part of its view ahead of the B-pillar, but it isn't a row of its own in the degradation matrix.
