# Isolated corridor test

Start with a clear corridor: 3.5 m between the inside wall faces, walls from
world X=-2 to X=12, floor at Z=0 and roof underside at Z=3.0. The drone starts
at X=-4, Y=0: exactly 2 m before the entrance, with a deliberate +15 degree
yaw error. Take off to 1.3 m.
The isolated model uses a top-mounted LiDAR at model Z=+0.18 m to keep its scan
clear of the body; at the test flight height it remains well below the roof.
There are no cameras, banners or obstacles in this baseline.

This runs the existing native corridor FSM. It starts directly at PRE_ENTRY,
aligns its yaw to the walls, advances 2.75 m during ENTER (2 m to the opening
plus the normal 0.75 m entry margin), cruises, and exits the open far end.
Altitude is held at the settled takeoff height. Landing remains a MAVProxy
command after success; native ABORT requests LAND. Ctrl+C in the runner sends
STOP.

The Gazebo partition and LiDAR topic are isolated, but this test uses the usual
SITL ports 9002/5760 and MAVProxy outputs 14550/14552. Close the old manager,
MAVProxy, SITL and Gazebo with Ctrl+C before starting this test. Do not run both
stacks together. No existing mission files or native controller defaults are
changed by these scripts.

## Terminal 1 — Gazebo

```bash
bash ~/[competition]_mission2/world/experiments/corridor_only/gazebo.sh
```

Wait for the world to load. The launch script includes the local multicast setup.

## Terminal 2 — SITL

```bash
bash ~/[competition]_mission2/world/experiments/corridor_only/sitl.sh
```

## Terminal 3 — MAVProxy

```bash
bash ~/[competition]_mission2/world/experiments/corridor_only/mavproxy.sh
```

Wait for initialization and EKF readiness. In the MAVProxy prompt:

```text
mode guided
arm throttle
takeoff 1.3
```

Wait until the drone is hovering steadily inside the corridor.

## Terminal 4 — inspection, then flight

First verify the actual airborne scan. This command sends no movement commands:

```bash
bash ~/[competition]_mission2/world/experiments/corridor_only/mission.sh --inspect
```

Expected: `strict_valid: true`, `width` near 3.5, confidence above 0.7 and valid
left/right fits. A centered headless Gazebo sensor test produced confidence
0.964, width 3.50038 m and 108 wall-fit candidates on each side. An invalid
inspection exits with status 2; inspect its log rather than starting flight.

When inspection passes:

```bash
bash ~/[competition]_mission2/world/experiments/corridor_only/mission.sh
```

Expected progression:

```text
PRE_ENTRY_GEOMETRY_LOCK
ENTER_CORRIDOR
CORRIDOR_CRUISE
EXIT_DETECTION
[PASS] CORRIDOR_EXITED
```

The runner stops after success. Land using Terminal 3:

```text
mode land
```

On an unexpected movement, Ctrl+C in Terminal 4 sends STOP; use `mode land`
in Terminal 3. Restart the world and SITL to repeat from the known start pose.

## Evidence and replay

- `artifacts/synthetic_fsm.json`: native FSM passed the whole baseline with
  analytical wall scans and an ideal velocity response.
- `artifacts/sensor_validation/live_geometry.json`: actual headless Gazebo
  wall scan passed geometric validation. This was an unarmed sensor test.
- `artifacts/scan_report.json` and `scan_plot.png`: prior mission scan analysis;
  only 5 left and 6 right candidates, insufficient to fit either wall.
- `artifacts/flight.log` and `artifacts/flight/live_scan.npz`: next flight evidence.

Full ArduPilot flight in this new world still requires the four-terminal run.
The headless sensor test and offline FSM test do not establish flight success.

Replay any captured scan:

```bash
source ~/[competition]_mission2/world/experiments/corridor_only/env.sh
python3 "$CORRIDOR_TEST_ROOT/replay.py" \
  --scan "$CORRIDOR_TEST_ROOT/artifacts/flight/live_scan.npz" \
  --output "$CORRIDOR_TEST_ROOT/artifacts/flight/replay"
```

Run the offline FSM test without Gazebo:

```bash
source ~/[competition]_mission2/world/experiments/corridor_only/env.sh
python3 "$CORRIDOR_TEST_ROOT/replay.py" --synthetic
```

