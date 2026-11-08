# GTA 5 Autopilot

A realistic driver-assistance and autopilot mod for **GTA V Enhanced** (single-player only). The car drives the way a real ADAS stack does:

sensors → vehicle buses (CAN / CAN FD) → ECUs → actuators

The autopilot never reads game state directly. It only sees what its sensors put on the bus.

- **Core:** Rust, in a separate process from the game, connected over shared-memory rings.
- **Game access:** Script Hook V (Enhanced), plus a thin C++ ReShade add-on shim for graphics.
- **Features:** ACC, AEB, LKA, blind-spot, rear cross-traffic and more, each toggleable.
- **Views:** in-game HUD, Rerun 3D views, MCAP recordings.

## Status
Architecture phase. **No code yet.**
- The design lives in the PRD (Claude Docs): https://claude.ai/artifact/GMXC6hxBEY8Vg7T42goYwU. It holds the decision log, the architecture, and the implementation plan with Phases 0–6.
- Open design questions are tracked as [issues](../../issues), each with possible solutions and a recommendation. Resolve the `blocker` issues first.

## Target machine
A gaming PC with an RTX 4070 Ti (12 GB) and a Ryzen 7 7800X3D. GTA and all builds and tests run there.

## Rules
- Story mode only. Launch with `-nobattleye`, and never go online with mods loaded.
- Pin the game build. Patches break memory patterns and offsets.
