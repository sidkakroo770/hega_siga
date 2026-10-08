# Mission software

This repository contains the non-ROS mission software, Gazebo environments, corridor runtime, coverage/QR runtime, and Raspberry Pi integration work.

## Start here

- `world/integration/experimental_corridor_manager.py` — top-level non-ROS mission manager and authority handoffs.
- `approach/` — banner approach, camera perception, and staging tests.
- `corridor/` — corridor navigation FSM and native hardware-oriented corridor runtime.
- `coverage_mission/` — field mapping, red-zone avoidance, coverage planning, QR inspection, and return navigation.
- `world/` — Gazebo worlds, models, launch helpers, integration adapters, and hardware benchmarks.
- `config/` — mission and simulation configuration.
- `tools/` — one-off geometry and asset utilities.
- `docs/` — design notes, handoffs, reviews, and validation evidence.
- `archive/legacy/` — superseded prototypes and snapshots; not used by the active mission.

## Documentation map

- Current integration handoff: `world/HANDOFF.md`
- Pi hardware handoff: `world/integration/PI_HARDWARE_HANDOFF.md`
- QR implementation status: `docs/qr/mission_status.md`
- Return implementation status: `docs/return/mission_status.md`
- Validation evidence: `docs/validation/`
- Reviews and design history: `docs/reviews/` and `docs/`

## Main test areas

The active Python packages are intentionally kept at their current import paths so existing Gazebo and hardware test commands remain valid. Archived files are not part of the runtime path.
