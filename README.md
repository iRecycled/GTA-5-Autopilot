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
Phases 0 and 1 are done (ACC, FCW, AEB on real bus traffic), and Phase 2 (corner radars, ultrasonics, BSD, RCTA, park assist) is built and tested in the simulator and in GTA. See [docs/status/](docs/status/).

| Doc | What's in it |
| --- | --- |
| [docs/PRD.md](docs/PRD.md) | Goals, features and sensors, architecture, sim adapter, buses, timing, safety, tech stack, constraints, decision log D1–D29, spikes |
| [docs/implementation-plan.md](docs/implementation-plan.md) | Crate layout, contracts to freeze first (adapter, graphics shim, config), Phases 0–6 with checklists and gates |
| [docs/design-review.md](docs/design-review.md) | The first design review: a critique of every original decision, with alternatives |
| [docs/status/](docs/status/) | Progress reports per phase, with gate results |
| [docs/degradation.md](docs/degradation.md) | The degradation matrix (D21): what each missing capability or fault makes unavailable |

The second design review's questions were tracked as [issues](../../issues) #1–#11. Their decisions are now in `docs/` as D22–D29 plus revisions to earlier rows. The one still open, the hidden forward camera in GTA (#1, D29), isn't needed until Phase 5.

## Target machine
A gaming PC with an RTX 4070 Ti (12 GB) and a Ryzen 7 7800X3D. GTA and all builds and tests run there.

## Rules
- Story mode only. Launch with `-nobattleye`, and never go online with mods loaded.
- Pin the game build. Patches break memory patterns and offsets.
