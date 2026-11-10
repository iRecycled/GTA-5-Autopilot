# Degradation matrix (D21)

One table, from each missing adapter capability or sensor fault to the features it makes unavailable, and to what the driver sees. The core evaluates it every frame (`crates/adas-core/src/degrade.rs`). If a feature is unavailable, engaging it is refused; if it becomes unavailable while engaged, it disengages. Either way, the HUD shows the reason.

| Condition | ACC | Stop-and-go | AEB | Steering features (test driver; LKA later) | Speed profile | HUD shows |
| --- | --- | --- | --- | --- | --- | --- |
| No adapter info yet | ✗ | ✗ | ✗ | ✗ | ✗ | "ACC unavailable: starting up" |
| Host lacks **pedals** (`caps::PEDALS`) | ✗ | ✗ | ✗ | — | ✗ | "no pedal control on this host" |
| Host lacks **actors** (`caps::ACTORS`) | ✗ | ✗ | ✗ | — | ✓ | "no radar data on this host" |
| Host lacks **line of sight** (`caps::LINE_OF_SIGHT`) | degraded | ✓ | degraded | — | ✓ | nothing. The radar treats everything in its beam as visible: no occlusion |
| Host lacks a **standstill hold** (`caps::HOLD`) | ✓ above 30 km/h | ✗ | ✓, no hold after the stop | — | ✓ | below 30 km/h: "not available below 30 km/h (no standstill hold)" |
| Host lacks **steering** (`caps::STEER`) | ✓ | ✓ | ✓ | ✗ | ✓ | "no steering control on this host" |
| **Chassis CAN silent** for over 0.25 s (18 frames) | ✗ | ✗ | ✗ | ✗ | ✗ | "vehicle data lost" |
| **Radar silent** for over 0.5 s (36 frames, 10 scans) | ✗ | ✗ | ✗ | — | ✓ | "radar blocked", plus "AEB OFF" |

AEB unavailable shows as **AEB OFF** on the HUD. A lost core, through a crash or a stall, is handled by the adapter's safe state (D24), not by this matrix. Inside the core, the actuator ECUs drop any request they haven't heard again for 0.1 s, so a silent ADAS ECU releases the car.

## Surround features (Phase 2)

BSD, RCTA and park assist aren't engaged: they run whenever they are available, as AEB does. Unavailable, they show greyed out with an "OFF" mark on the HUD (`surround_flags::BSD_OFF`, `RCTA_OFF`, `PARK_OFF`).

| Condition | BSD | RCTA | Park assist | HUD shows |
| --- | --- | --- | --- | --- |
| Host lacks **actors** (`caps::ACTORS`) | ✗ | ✗ | — | BSD OFF, RCTA OFF |
| Host lacks **gear** (`caps::GEAR`): it can't say when reverse is selected | — | ✗ | — | RCTA OFF |
| Host lacks **range fans** (`caps::RANGE_FANS`): no parking sensors | — | — | ✗ | PARK OFF |
| Host lacks **turn signals** (`caps::TURN_SIGNALS`) | icons only, no signal warning | — | — | nothing |
| Host lacks **pedals** (`caps::PEDALS`) | — | warns, doesn't brake | shows and sounds, doesn't brake | nothing |
| **Chassis CAN silent** for over 0.25 s | ✗ | ✗ | ✗ | all three OFF |
| A **rear corner radar** silent for over 0.5 s (36 frames) | ✗ | ✗ | — | BSD OFF, RCTA OFF ("side radar blocked") |
| A **parking sensor** gives no data, or the park ECU is silent for over 0.25 s, while the sensors run | — | — | ✗ | PARK OFF ("parking sensor fault") |

The parking sensors run only below 18 km/h (off again above 20 km/h), as on real cars; at speed an old park cycle isn't a fault. For the first 0.5 s after they start, missing data isn't a fault either: the host needs a few frames to answer the fans. A front corner radar going silent costs BSD only some of its view ahead of the B-pillar, so it isn't a row of its own.

ACC is also unavailable **in reverse** ("not available in reverse"), and AEB ignores what is ahead while reversing.

## Lanes and the camera (Phase 3)

Lane centering runs inside the autopilot (`--feature autopilot`, ACC + lane centering). If it can't continue, the whole autopilot asks the driver to take over (D11). While the camera still sees the lane, it keeps steering until the driver answers, as openpilot's soft disable does. LKA nudges only when nothing else steers.

| Condition | Lane centering (autopilot) | LKA | HUD shows |
| --- | --- | --- | --- |
| Host lacks **steering** (`caps::STEER`) | ✗ | ✗ | "no steering control on this host" |
| Host lacks **lane markings** (`caps::MAP`) | ✗ | ✗ | "no lane data on this host" |
| **Camera silent** for over 0.5 s (36 frames) | ✗ | ✗ | "camera blocked" |
| **No lane lines seen** for over 0.5 s (a frame without lines is bridged) | ✗ | ✗ | "lanes not detected" |
| **The lane ends** (narrower than 2.6 m within 4 s of driving) | ✗, takeover request | ✓ | "lane ends" |
| **Hands off the wheel** 25 s, with the optional driver monitor on (reminder at 15 s) | ✗, takeover request | — | "hands off the wheel too long" |
| **Chassis CAN silent** | ✗ | ✗ | "vehicle data lost" |

Pedestrian AEB needs the camera; without it, AEB still brakes for what the front radar sees.

Not yet in the matrix:
- bus faults short of silence, such as bad checksums or CRC errors: counted (`BusStats::errors`, `crc_errors`); the bad frames are discarded and the last good values held. A bus whose frames all fail counts as silent;
- weather and dirt on the sensors (Phase 1 error model).
