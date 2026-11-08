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

| Doc | What's in it |
| --- | --- |
| [docs/PRD.md](docs/PRD.md) | Goals, features and sensors, architecture, buses, timing, tech stack, constraints, decision log D1–D21, spikes |
| [docs/implementation-plan.md](docs/implementation-plan.md) | Crate layout, contracts to freeze first (IPC, graphics shim, config), Phases 0–6 with checklists and gates |
| [docs/design-review.md](docs/design-review.md) | The first design review: a critique of every decision, with alternatives |

These were exported on 2026-11-08 from the [Claude Doc](https://claude.ai/artifact/GMXC6hxBEY8Vg7T42goYwU). **The copies in `docs/` are the ones to edit from now on.**

Open design questions are tracked as [issues](../../issues), each with possible solutions and a recommendation. They come from a second, independent review and **aren't reflected in `docs/` yet**. Resolve the `blocker` issues first, then update the docs as each one is decided.

## Target machine
A gaming PC with an RTX 4070 Ti (12 GB) and a Ryzen 7 7800X3D. GTA and all builds and tests run there.

## Rules
- Story mode only. Launch with `-nobattleye`, and never go online with mods loaded.
- Pin the game build. Patches break memory patterns and offsets.
