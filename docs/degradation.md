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

Not yet in the matrix:
- bus faults short of silence, such as bad checksums or CRC errors: counted (`BusStats::errors`, `crc_errors`); the bad frames are discarded and the last good values held. A bus whose frames all fail counts as silent;
- weather and dirt on the sensors (Phase 1 error model);
- the corner radars and ultrasonics (Phase 2).
