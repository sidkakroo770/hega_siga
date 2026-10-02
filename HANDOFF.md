# SAE Mission 2 — corridor integration and isolated corridor test handoff

## Latest update — 2026-09-22 exit timing

This update supersedes earlier test boundaries below. User's full-world run
completed measured entry (0.76 m), cruised with ~0.958 confidence, and reached
EXIT_DETECTION. Exit twice hit its original 15-second WALL-TIME deadline,
then recovery cycled into cruise with unreliable geometry and the loop guard
aborted. Full mission completion is still unverified.

Changed `corridor/native/controllers/exit_detection.py`: optional
`progress_clock` callable (default monotonic for existing/hardware callers)
drives exit's movement deadline. Pose freshness and continuous pose-loss
watchdog remain wall-time. Optional `ExitConfig.wall_timeout_s` provides an
independent wall cap. A progress clock reset below entry time stops/reassesses.
The full-world manager supplies `telemetry.position_boot_s` (SITL clock, also
used for descent), retains 15 simulation seconds, sets wall cap 180 seconds,
pose freshness 1 second and continuous pose-loss timeout 6 seconds. It logs
`exit_travel` and `sim_elapsed`. Exit still requires 1.20 m measured movement
at 0.15 m/s; front safety and recovery loop limits remain enabled.

Correction: prior statements calling the 60-second ENTER deadline simulation
time were incorrect. ENTER still uses monotonic wall time. Only EXIT now uses
the injected simulation clock in the full-world manager.

Validation: seven tests in `corridor/native/test_exit_clock.py` passed (slow
simulation, simulation deadline, frozen clock/pose loss, brief recovery,
wall cap, reset, default clock), existing `native.test_exit_pose` passed,
manager compiled, corridor diff whitespace check passed. No flight was run.
Next: restart the manager for another full-world flight and check for measured
EXIT completion. Do not weaken recovery guards if a different failure occurs.

Prepared on 2026-09-12 (Asia/Kolkata) for Siddharth. This document is intended
to be sufficient for a fresh Codex session without the original conversation.
Repository status, current code, launch scripts, and persisted test reports
were inspected while writing it. No implementation was continued after this
handoff was created.

## Save locations and directory instructions

- Canonical repository-root handoff: `/home/sid/sae_mission2/world/HANDOFF.md`.
- User-requested Desktop copy: `/home/sid/Desktop/HANDOFF.md`.
- Workspace-level convenience copy: `/home/sid/sae_mission2/HANDOFF.md`.
- The workspace `/home/sid/sae_mission2` combines THREE separate repositories;
  do not assume one Git repository contains all the code.
- Primary repository for the active experiment: `/home/sid/sae_mission2/world`.
- Current experiment directory:
  `/home/sid/sae_mission2/world/experiments/corridor_only`.
- Native controller repository: `/home/sid/sae_mission2/corridor`.
- Camera repository: `/home/sid/sae_mission2/approach`.
- ArduPilot source: `/home/sid/ardupilot`.
- Gazebo ArduPilot plugin and stock vehicle models: `/home/sid/ardupilot_gazebo`.
- Original imported conversation summary, if needed:
  `/home/sid/Downloads/SAE_Mission2_Codex_Handoff.txt`. It is historical and
  superseded by this document and current code.

Use absolute paths or `cd` explicitly. Shell scripts below source their own
environment, so they can be invoked from any working directory. Run them with
`bash`; executable permission is not required. Future edits should update the
canonical handoff and recopy it to Desktop/workspace if those copies are used.

## Overall objective and immediate task

Overall objective: integrate SAE AeroTHON autonomous drone Mission 2 in Gazebo
Harmonic and ArduPilot SITL. The mission uses camera/banner approach followed
by Siddharth's native non-ROS LiDAR corridor FSM, including entry, cruise,
obstacle decisions/avoidance and exit.

Immediate task: isolate and validate the LiDAR corridor states in a new simple
world. The full-world camera approach, descent, and hover now work in user
runs, but PRE_ENTRY cannot recognize both corridor walls and times out. The
user requested a corridor-only world and a correct pipeline requiring only
copy/paste commands into terminals. Those files and commands are ready.

**Current verified boundary:**

1. The existing native FSM passed an offline analytical-scan traversal through
   `CORRIDOR_EXITED`.
2. A REAL headless Gazebo sensor test in the new simple world produced valid
   two-wall geometry with confidence 0.964.
3. Full ArduPilot/SITL flight through this new world has NOT yet been verified.
   This is the exact next test. Do not claim that the full corridor flight or
   original Mission 2 is solved.
4. The isolated baseline intentionally has no obstacles. It validates
   PRE_ENTRY, ENTER, CRUISE and EXIT; obstacle-avoidance branches have not been
   exercised by this new experiment.

The user's final instruction for this turn was to create this detailed
handoff, save it on Desktop, and stop implementation. Respect that boundary.

## Later-session update — full-world LiDAR diagnosis and current state

This section supersedes the older "current verified boundary" above.

- The original full-world corridor covers were single-sided with their triangle
  winding facing outward. The GPU LiDAR therefore could not measure the inside
  walls: PRE_ENTRY had confidence 0 despite the same controller succeeding in
  the isolated corridor test.
- `experiments/full_world_lidar_ab/repair_corridor_mesh.py` added reversed,
  inward-facing copies of the two cover meshes to the production
  `models/models/miss2_env/miss2.glb`. The original mesh is backed up as
  `miss2.glb.before_two_sided_corridor_repair`.
- A controlled diagnostic at the full-mission handoff pose showed original
  winding: 6/5 wall candidates and confidence 0; reversed winding: 88/88
  candidates and confidence 0.882. The repaired production asset passed the
  same test.
- Full integrated runs now successfully descend 1 m, hover 2 simulation
  seconds, acquire strict LiDAR geometry (~0.88 confidence, ~3.75 m width),
  align/verify, and enter `ENTER_CORRIDOR`.
- MAVLink local-position delivery through MAVProxy has occasional gaps. The
  manager requests 20 Hz position/attitude via both message-interval and
  legacy stream requests. It refreshes that request only after a meaningful
  0.75-second delay, avoiding false warnings for normal ~0.25-second packet
  cadence. The native runner treats a missing pose as one continuous outage,
  stops during it, and aborts only after 6 seconds. Its permitted pose age is
  1 second in this integration.
- Latest run did **not** fail geometry or continuous pose loss. It reached
  `ENTER_CORRIDOR`, moved 0.62 m of the 0.75 m requirement, then hit its
  overall 40-second entry deadline. The manager now uses a 60-second entry
  deadline and prints `travelled=...` in entry diagnostics. It retains the
  0.75 m measured requirement, 0.20 m/s entry command, 0.5 m camera-to-
  corridor handoff range, and 1 m vertical descent.
- Validate the next full run before claiming full mission success. The next
  expected transition is `ENTER_CORRIDOR -> CORRIDOR_CRUISE`; a new failure
  must be investigated from its exact diagnostic reason.

## Repository state inspected at handoff

No commits, pushes, PRs, branch switches, or resets were made in this session.
Most integration/experiment files are untracked, so `git diff` alone omits
them. Inspect their contents directly.

### World repository

Directory: `/home/sid/sae_mission2/world`

- Branch: `main`.
- HEAD: `79fd525 Update README.md`.
- Earlier commits: `6b1b1cb Delete automation_ki_baat_cheet/temp`,
  `e627db3 Add files via upload`.
- Origin: `https://github.com/Prem-creator-arch/ajao-lelo-mera.git`.
- Status BEFORE adding this handoff:

```text
 M models/models/miss2_env/miss2.glb
?? experiments/
?? integration/
?? models/models/iris_miss2_full/
?? models/models/iris_miss2_integrated/
?? models/models/miss2_env/miss2.glb.before_corridor_recolor
?? worlds/miss2_full_world.sdf
?? worlds/miss2_lidar_world.sdf
?? worlds/miss2_preentry_test.sdf
```

The tracked GLB diff was binary: 18,516,456 -> 18,516,448 bytes. The actual
change is one material's name/base color; geometry and binary buffers were
verified unchanged when edited. `HANDOFF.md` will additionally be untracked.

### Native corridor repository

Directory: `/home/sid/sae_mission2/corridor`

- Branch: **`native-non-ros`**. Preserve it.
- HEAD: `b17b54b Add missing native abort and MAVLink runtime modules`.
- Earlier commits: `ee2b8c2 Add native Pixhawk runtime and measured exit detection`,
  `5e7fe5b Add native non-ROS corridor mission stack`.
- Origin: `https://github.com/sidkakroo770/something-secret.git`.
- `git status --short` was empty at handoff.
- Native source was inspected but not modified. Experimental timeout overrides
  are applied to runner/controller instances in the world repository.

### Approach repository

Directory: `/home/sid/sae_mission2/approach`

- Branch: `main`.
- HEAD: `d06f7f4 Delete nano`.
- Earlier commits: `fae7667 Update README.md`,
  `e5aabe7 Update README with mavlink terminal usage`.
- Origin: `https://github.com/thakurdhruv960/shhhh.git`.
- Modified: `autonomy/perception/hybrid_banner_detector.py` (this session).
- Also modified: `autonomy/behaviors/mission_runner.py`. This edit was not made
  by this session; preserve it. Inspected diff changes connection 14550 ->
  14552 and QR capture path to `~/sae_mission2/approach/qr_captures`.
- Untracked Python caches exist under `autonomy/` and `autonomy/perception/`.

No AGENTS.md or existing HANDOFF.md was found by the workspace search used for
this handoff. Recheck instructions if the workspace changes in a later session.

## Environment and assumptions

- Inspected OS: Ubuntu 22.04.5 LTS (Jammy).
- Inspected Python: 3.10.12.
- Gazebo Harmonic, transport bindings `gz.transport13`, protobuf bindings
  `gz.msgs10`; NumPy, OpenCV, pymavlink and matplotlib are installed locally.
- `gz` is `/usr/bin/gz`; `mavproxy.py` is `/home/sid/.local/bin/mavproxy.py`.
- ArduPilot vehicle: ArduCopter, frame `gazebo-iris`, JSON physics connection.
- Locally built plugin: `/home/sid/ardupilot_gazebo/build`.
- Simulated horizontal LiDAR approximates the D500 interface: 500 samples,
  -pi to +pi, 10 Hz, range 0.02–12 m.
- Python must set `PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python` before Gazebo
  imports; include `/usr/lib/python3/dist-packages` in PYTHONPATH.
- Local multicast workaround:

```bash
sudo ip link set dev lo multicast on
sudo ip route replace 239.255.0.7/32 dev lo src 127.0.0.1
```

- Both pipelines use `GZ_IP=127.0.0.1` and
  `GZ_DISCOVERY_MULTICAST_IP=239.255.0.7`.
- Full mission partition: `miss2_local`.
- Corridor-only partition: `sae_corridor_only`.
- Headless validation partition: `sae_corridor_validation`.
- Gazebo plugin/SITL physics port: 9002; SITL TCP: 5760; MAVProxy outputs:
  UDP 14550 and 14552. Runner listens on `udpin:0.0.0.0:14552`.
- A distinct Gazebo partition does NOT isolate those MAVLink/physics ports.
  Never run the full mission and corridor-only flight stacks concurrently.
- The simulation was observed running much slower than wall time. Reported
  velocities are in simulation seconds. This caused earlier timeout errors.
- No real hardware flight has been validated here. User explicitly requested
  comments warning that simulation speeds/settings need review before actual
  aircraft testing. The isolated runner contains a simulation-only warning.

### Tool/sandbox and background-process notes

The Codex sandbox could read/write the home workspace but blocked Gazebo
socket creation. A sandboxed headless test aborted with `Socket creation
failed` and an invalid-local-IP message. This was an execution-permission
problem, not evidence of bad Gazebo environment configuration.

A bounded headless test was run outside the sandbox with approval. It used a
validation model WITHOUT the ArduPilot plugin, its own partition, a 35-second
timeout, and an EXIT trap to stop its own process. It completed successfully.

After a model/session interruption, the old exec session IDs were invalid and
the old `/tmp/sae_corridor_validation.log` had disappeared. Do not attempt to
resume old session IDs. The test was rerun with persistent artifact paths.
The last host process check at completion found no `gz sim`, `gz-sim`,
ArduCopter, MAVProxy or sim_vehicle processes. This is a historical check;
recheck before launching if the user may have started another run.

## Architecture and interfaces

### Common control conventions — do not break

Native `BodyVelocity` fields:

```text
vx_m_s          positive forward
vy_m_s          positive LEFT
vz_m_s          positive UP
yaw_rate_rad_s  positive CCW
```

MAVLink BODY_NED velocity/yaw-rate conversion:

```text
mav_vx = native_vx
mav_vy = -native_vy
mav_vz = -native_vz
mav_yaw_rate = -native_yaw_rate
```

Native velocity sender uses type mask **1479**, preserving ACTIVE yaw-rate.
Do not substitute the camera sender's yaw-ignored mask for native commands.

`NativeScan` in `corridor/native/common/types.py` contains `angles_rad`,
`ranges_m`, `intensities`, monotonic reception `timestamp`, `range_min_m`,
`range_max_m`. Angles: 0 forward, +90 degrees left. Its `age_s` uses wall-time
monotonic age. `Attitude` contains roll/pitch/yaw and timestamp.

`VehiclePose` in `native/mission_runner.py` contains local X/Y, yaw and pose
timestamp. ENTER and EXIT use measured displacement; never replace this with
a time-only approximation.

`NativeMissionRunner.step(scan, attitude=..., pose=...)` returns a controller
output containing command, optional action, status, reason and confidence.
`public_state()` exposes `MissionState`. A `VehicleAction.LAND` must take
precedence over velocity commands.

### Native corridor FSM

Important files under `/home/sid/sae_mission2/corridor/native/`:

- `mission_runner.py`: coordinator, `MissionRunnerConfig`, `VehiclePose`,
  `NativeMissionRunner`; owns pre_entry/cruise/obstacle/exit/reassess/abort.
- `controllers/pre_entry.py`: `PreEntryConfig`, `PreEntryController`,
  `CorridorGeometry`, `extract_corridor_geometry`, `fit_line_ransac`.
- `controllers/corridor_cruise.py`: cruise, wall alignment and exit precursors.
- `controllers/obstacle_avoidance.py`: obstacle decision and bypass.
- `controllers/exit_detection.py`: measured forward exit commit.
- `controllers/hover_and_reassess.py`: recovery, including recovery timeout.
- `controllers/abort_corridor.py`: terminal abort/LAND request.
- `hardware/` and `run_corridor_real.py`: real-device runtime. Not used to
  launch this Gazebo experiment; do not rewrite them to fix experiment issues.

PRE_ENTRY waits for two valid roughly parallel wall fits at the expected
spacing. Important defaults inspected earlier:

- corridor width 3.5 m; strict tolerance 0.45 m; loose tolerance 0.80 m.
- geometry range <=8 m; fitting X window -0.2 to 5 m.
- left/right candidate Y magnitudes 0.25 to 3.5 m.
- at least 14 wall inliers, minimum span 0.8 m, max fit RMS 0.10 m.
- control confidence minimum 0.62; verify minimum 0.72.
- front stop 0.80 m; stale scan 0.30 s.
- internal phases ACQUIRE_GEOMETRY, ALIGN_YAW, CENTER_LATERALLY, VERIFY_LOCK,
  LOCKED and HOLD. A failed acquisition cannot recover by simply waiting in
  the same view forever.

`CorridorGeometry` can return with default zero confidence before filling its
inlier fields if either wall fit fails. Therefore zero reported inliers does
not mean the scan contained no points: explicitly count candidates as replay.py
does. Native geometry confidence is not a sensor connection status.

## Current active experiment: corridor_only

All paths in this section are relative to
`/home/sid/sae_mission2/world/experiments/corridor_only`.

### Files and geometry

- `corridor_only.sdf`: NEW simple box-based world. Interior wall faces at
  Y=+1.75/-1.75 m, length X=-2 through X=12, floor top Z=0, roof underside Z=3.
  Wall box centers X=5, Y=+1.85/-1.85, Z=1.5; size 14 x 0.2 x 3 m.
  Roof center Z=3.1, thickness 0.2. Ground size 40 x 20 x 0.2.
  Drone spawns at world `(0,0,0.25)`, yaw 0, facing +X, INSIDE the corridor.
- `models/iris_corridor_test/model.sdf` and `model.config`: NEW dedicated
  vehicle derived from the working integrated Iris model. Keeps stock Iris,
  flight physics, IMU and ArduPilot plugin. Removes forward/downward cameras.
  LiDAR is deliberately at **model Z=+0.18 m**, above its own body, and emits
  `/corridor_test/lidar/scan`. This is different from the full-world model.
  At takeoff 1.3 m it is comfortably below the corridor roof.
- `env.sh`: complete per-terminal environment, resource paths, dedicated
  partition and topic dependencies. Defines `CORRIDOR_TEST_ROOT`.
- `gazebo.sh`: multicast setup, then `gz sim -r -v3` for this world.
- `sitl.sh`: sim_vehicle JSON/gazebo-iris, no MAVProxy, separate state directory
  `artifacts/sitl` via `--use-dir` (verified supported by installed script).
- `mavproxy.sh`: TCP 5760 -> UDP 14550/14552; `--streamrate=20 --console`;
  logs/state under `artifacts/mavproxy`.
- `mission.sh --inspect`: read-only sensor geometry check; output under
  `artifacts/inspection`, tee log `artifacts/inspection.log`.
- `mission.sh` with no arguments: runs `run.py --fly`; output under
  `artifacts/flight`, tee log `artifacts/flight.log`. Shell uses `pipefail`.
- `README.md`: full user launch instructions and evidence boundaries.

### run.py behavior

Standalone sensor/MAVLink runner; does not import the camera integration
manager and does not perform banner approach, descent, or camera control.

- `receive_scan`: accepts a single horizontal plane; requires ranges size to
  equal message count and rejects multiple vertical planes. Converts to
  `NativeScan`; thread lock protects latest scan/sequence.
- `send_velocity`: native FLU -> MAVLink BODY_NED conversion with mask 1479.
- `request_streams`: requests LOCAL_POSITION_NED and ATTITUDE at 20 Hz.
- `save_scan`: NPZ raw scan plus JSON geometry report.
- `create_runner`: instance-specific simulation timeout overrides.
- Default (without `--fly`): subscribe, wait up to 20 wall seconds, save/report
  one actual scan, exit 0 if strict geometry valid, otherwise exit 2. It opens
  NO MAVLink connection and sends no motion commands.
- `--fly`: waits up to 30 seconds for an ArduPilot autopilot heartbeat and uses
  its source system/component; requires armed GUIDED, fresh finite scan/pose/
  attitude, and already settled takeoff. Initial EKF-relative height must be
  0.8–1.8 m and abs(vertical speed)<=0.1 m/s; instructed takeoff is 1.3 m.
- Captures a scan before starting a fresh native runner.
- Steps native FSM once per new scan, repeats bounded cached commands at
  about 20 Hz; command age >0.5 s sends STOP.
- Holds initial settled takeoff altitude with bounded +/-0.15 m/s correction.
- Heartbeat limit 3 s; scan/pose/attitude gates 0.5 s. Missing inputs send STOP
  and terminate after 5 s. Runtime watchdog 900 wall seconds.
- Requires MAVLink custom mode 4 (ArduCopter GUIDED) and armed bit. Exits on
  mode/arming loss so manual mode changes can take control.
- On native ABORT/LAND: sends STOP, requests LAND. On CORRIDOR_EXITED: sends
  STOP, reports PASS, returns; user lands through MAVProxy.
- Ctrl+C and finally send several STOP packets and close MAVLink.
- Debug lines show state, confidence, command, input ages, and PRE_ENTRY wall
  width/inliers/spans/sectors.

Timeout overrides for this experiment (wall seconds; native code unchanged):

| Setting | Value |
|---|---:|
| PRE_ENTRY acquire | 60 |
| PRE_ENTRY alignment | 90 |
| PRE_ENTRY hold | 60 |
| ENTER | 60 |
| Reassessment recovery | 60 |
| Reassessment hard cap | 65 |
| EXIT hard timeout | 90 |

### Replay and validation files

- `replay.py`: CLI `--scan PATH`, `--synthetic`, `--output DIR`.
- `synthetic_scan`: analytical rays against the two wall segments in the new
  world; 500 beams, same 0.02–12 m range and native angle convention.
- `geometry_report`: runs native geometry extraction and independently counts
  left/right fit candidates, avoiding misleading default zero-inlier reports.
- `synthetic_test`: monkey-patches monotonic time in an OFFLINE single-thread
  test, integrates ideal body velocity commands against analytical scans, and
  asserts the unchanged native runner reaches CORRIDOR_EXITED.
- `artifacts/synthetic_fsm.json`: PASS, final X=12.53 m, state sequence
  PRE_ENTRY_GEOMETRY_LOCK, ENTER_CORRIDOR, CORRIDOR_CRUISE, EXIT_DETECTION,
  CORRIDOR_EXITED. This is not a flight dynamics test.
- `artifacts/scan_report.json`, `artifacts/scan_plot.png`: saved full-mission
  scan analysis. It had 500 beams, 86 finite ranges, only 5 left/6 right fitting
  candidates, both fits invalid, confidence 0.0; L about 1.947 m and remaining
  diagnostic sectors empty.
- `validate_sensors.sh`: bounded headless real Gazebo sensor test with
  `GZ_PARTITION=sae_corridor_validation`. Starts `gz sim -s -r
  --headless-rendering` under `timeout 35s`, runs read-only inspection, and
  traps EXIT to stop/wait its own background process.
- `artifacts/validation_models/iris_corridor_test/{model.sdf,model.config}`:
  sensor-validation clone WITHOUT ArduPilot plugin. Kept at the front of the
  resource path only by validate_sensors.sh. Do not use it for SITL flight.
- `artifacts/gazebo_sensor_validation.log`: persisted headless log.
- `artifacts/sensor_validation/live_scan.npz` and `live_geometry.json`: actual
  Gazebo sensor evidence, inspected at handoff:

```text
500 beams; 362 finite beams
strict_valid = true
confidence = 0.9640000000000001
width = 3.500382300258868 m
left candidates = 108, right candidates = 108
both fits valid
left/right spans = ~4.241 m
left/right RMS = ~0.007545 m
L = ~1.751388 m; R = ~1.751388 m
FL = ~2.400711 m; FR = ~2.400711 m; F = null (open corridor)
```

At handoff there was NO `artifacts/flight.log` from an actual corridor-only
flight. If one exists in a later session, inspect its modification time and
contents rather than assuming these notes are current.

## Exact commands for the next corridor-only run

Close an older mission manager, MAVProxy, SITL and Gazebo with Ctrl+C first if
they are running. Last completion check found none running, but user actions
may have changed this. Do not issue blanket `pkill -9` cleanup.

Terminal 1:

```bash
bash ~/sae_mission2/world/experiments/corridor_only/gazebo.sh
```

Wait for world load. Terminal 2:

```bash
bash ~/sae_mission2/world/experiments/corridor_only/sitl.sh
```

Terminal 3:

```bash
bash ~/sae_mission2/world/experiments/corridor_only/mavproxy.sh
```

After initialization/EKF readiness, type in MAVProxy:

```text
mode guided
arm throttle
takeoff 1.3
```

Wait for steady hover. Terminal 4:

```bash
bash ~/sae_mission2/world/experiments/corridor_only/mission.sh --inspect
```

Require strict_valid true, width near 3.5 and confidence >0.7 before proceeding.
Then, in Terminal 4:

```bash
bash ~/sae_mission2/world/experiments/corridor_only/mission.sh
```

Success marker: `[PASS] CORRIDOR_EXITED`. Land in Terminal 3:

```text
mode land
```

On unexpected movement, Ctrl+C in Terminal 4 sends STOP, then use `mode land`.
Repeat tests from a fresh world/SITL start rather than assuming the same pose
after a prior traversal.

Useful offline commands:

```bash
source ~/sae_mission2/world/experiments/corridor_only/env.sh
python3 "$CORRIDOR_TEST_ROOT/replay.py" --synthetic
python3 "$CORRIDOR_TEST_ROOT/replay.py" \
  --scan "$CORRIDOR_TEST_ROOT/artifacts/flight/live_scan.npz" \
  --output "$CORRIDOR_TEST_ROOT/artifacts/flight/replay"
```

The second command requires a new flight capture to exist first. To replay the
already verified sensor test, substitute
`artifacts/sensor_validation/live_scan.npz` as its input.

## Full-world mission implementation retained for later integration

The active task is now the isolated test. Do not resume arbitrary full-world
tuning before checking the isolated test result.

### Main integration files

Paths relative to `/home/sid/sae_mission2/world`:

- `integration/experimental_corridor_manager.py`: existing manager extensively
  revised in this session; camera/Gazebo adapter, MAVLink cache/senders,
  approach handoff, relative descent, hover, then native runner.
- `integration/corridor_altitude.py`: NEW reusable altitude controller with
  bounded NED-down output, fresh-telemetry checks, gain, settling dwell,
  optional autopilot simulation clock and wall watchdog.
- `integration/corridor_handoff.py`: NEW `panel_front_distance(scan)` helper.
  Filters finite valid ranges in +/-10 degrees, requires at least 3 samples,
  returns median of the three shortest. This rejects one isolated short
  outlier. It measures a forward surface, NOT a classified panel.
- `integration/test_corridor_altitude.py`: NEW 11-test suite described below.
- `worlds/miss2_full_world.sdf`: full environment and integrated drone, raised
  green banner. This is NOT the corridor-only test world.
- `models/models/iris_miss2_full/model.sdf`: full mission vehicle, sensors and
  flight physics; LiDAR mounting was changed during this session.
- `models/models/miss2_env/miss2.glb`: one green corridor material recolored
  neutral gray. Geometry and binary mesh/image buffer bytes were preserved.

Existing read-only/staging scripts were inspected or retained, not newly
written by this session: `integration/test_fsm_readonly.py`,
`test_gz_lidar_adapter.py`, `stage_near_corridor.py`.
The old `worlds/miss2_preentry_test.sdf` is not the new test: it includes the
full GLB and a drone at Y=-65.50, so do not mistake it for corridor_only.sdf.

Historical manager backups created before selected edits:

```text
integration/experimental_corridor_manager.py.before_altitude
integration/experimental_corridor_manager.py.before_range_handoff
integration/experimental_corridor_manager.py.before_banner_loss_handoff
integration/experimental_corridor_manager.py.before_relative_descent
```

They are checkpoints, not current runnable recommendations. Preserve them.

### Current full-mission state flow

```text
BANNER_SEARCH
  -> CAMERA_CORRIDOR_CENTER
  -> APPROACH_CORRIDOR
  -> (forward range <=0.5 m OR 5 fresh frames without panel)
  -> DESCEND_BEFORE_PRE_ENTRY (1 m BELOW handoff height)
  -> HOVER_BEFORE_PRE_ENTRY (2 stable simulation seconds)
  -> fresh NativeMissionRunner / PRE_ENTRY_GEOMETRY_LOCK
  -> native ENTER / CRUISE / OBSTACLE / AVOID / EXIT states
  -> COMPLETE or ABORT
```

Current CLI defaults: MAVLink `udpin:0.0.0.0:14552`, enter distance 0.75 m,
search speed 0.50 m/s left, camera gain 0.003, camera max speed 0.50 m/s,
centering tolerance 20 px, 30 centered frames, approach speed 0.20 m/s,
banner-loss count 5, entrance-commit-range **0.5 m**.

No current `--corridor-altitude`, `--probe-timeout`, or `--handoff-scans`
arguments. Those belonged to superseded versions. Do not give the user old
commands containing removed options.

The user considered an extra forward-advance-after-panel-loss stage until the
range threshold. It was proposed and initially authorized, then explicitly
deferred BEFORE implementation to try relaxed camera filters. **It was never
implemented.** Current confirmed camera loss starts descent immediately even
if forward range is farther than 0.5 m. Do not silently add that stage as if
it were already part of the current behavior.

### Descent and hover details

- `Telemetry` caches LOCAL_POSITION_NED X/Y/Z and VX/VY/VZ, wall reception time,
  `time_boot_ms / 1000` as `position_boot_s`, and ATTITUDE plus its time.
- `drain_mavlink` updates this cache. Slow pose/attitude reception (>0.25 s)
  causes a bounded re-request of both 20 Hz streams no more than every 2 s.
- Relative descent snapshots fresh initial Z, sets target height `-z - 1.0`
  above EKF origin. Positive NED Z is down. It does not target 1.3 m AGL.
- Current descent controller configuration: max speed 0.50 m/s, gain 1.0,
  tolerance 0.10 m, settle dwell 0.5 simulation seconds, acquisition timeout
  60 simulation seconds, input max age 1.5 wall seconds.
- Fresh start-height acquisition requires <=0.5 s telemetry age and waits at
  most 2 wall seconds for an initial sample. Invalid/too-low relative targets
  abort; there is no trusted floor reference here.
- `AltitudeController.update(now,z,vz,timestamp,mission_time=...)` uses autopilot
  clock for descent progress timeout and settling. It does not extrapolate
  measured velocity across wall seconds in slow simulation.
- Stale/invalid telemetry sends zero vertical speed immediately; prolonged
  missing telemetry errors after 2 additional wall seconds. There is also a
  180-wall-second descent watchdog and clock-regression check.
- Dwell completion requires a new telemetry sample; repeatedly polling one
  sample cannot claim stability.
- Hover sends full STOP commands and waits for 2 simulation seconds with
  all measured axis speeds <=0.1 m/s. Stale input resets dwell; substantial
  altitude drift (>0.15 m) reacquires the SAME target, not another 1 m descent.
- Hover aborts on >3.5 s pose age, clock reset, or 180 wall seconds.
- Native runner is constructed only after hover completes. PRE_ENTRY timeouts
  therefore do not run while descending or hovering.
- Native states in the full manager receive their normal zero-VZ outputs;
  no earlier absolute-altitude override remains during native traversal.

`create_corridor_runner(enter_distance)` in the FULL manager uses these
instance-specific wall-time overrides (different from the isolated runner):
PRE_ENTRY acquire 32 s, alignment 60 s, hold 32 s, ENTER 40 s, reassessment
recovery 32 s, reassessment hard cap 48 s. Pose-loss guards are preserved.

### Camera detector state

`/home/sid/sae_mission2/approach/autonomy/perception/hybrid_banner_detector.py`
exports `HybridBannerDetector(panel_only=False)` and
`detect(frame, relaxed_approach=False)`.

- Current `min_area` is **700 square pixels** and comparison is strictly `>`.
- HSV lower [45,100,80], upper [75,255,255]; 5x5 morphological opening.
- Normal search/centering: old aspect ratio 0.8–5.0, polygon sides 4–16,
  extent >0.15. Chooses the largest valid contour.
- Approach calls `detect(..., relaxed_approach=True)`. It accepts partially
  visible/edge-clipped panels and bypasses shape/aspect/temporal tracking
  restrictions; retains minimum area and extent checks.
- `lock_target`, `reset_track`, `matches_panel` remain available, but the current
  full manager explicitly sets `panel_only=False` and resets tracking when
  approach starts. Do not mistake the presence of these methods for active
  strict tracking.
- Result fields: detected, bbox, center, area, largest_area, error_x/error_y,
  debug_frame, rejection_reason, clipped. Approach diagnostics include these
  plus LiDAR forward range, scan age and loss-frame count.
- Area threshold is NOT a distance estimate; the panel grows as it approaches.
  In user logs it filled the entire image (contour area 306081 px2) and stayed
  detected until the LiDAR distance trigger. Do not tune minimum pixels to
  claim an exact distance handoff.

### Full-world geometry edits

- Raised standalone `green_banner` box at X=-4, Y=-30. Box size 3.5 x 0.08 x
  1.0 m. Pose center Z changed from 2.5 -> **3.548 m**. Bottom is 3.048 m
  (10 feet WORLD Z), top 4.048. It has a visual but no explicit collision.
  This height is not measured from the mesh floor.
- Full model includes `miss2_env` GLB with link roll pi/2, so GLB Y corresponds
  to world height. Inspected mesh floor bounds near 0.084 m, roof near 3.05 m.
  These model bounds do not establish the live vehicle's height reference.
- GLB material formerly named `green` became `corridor_neutral_gray` with base
  color [0.45,0.45,0.45,1]. It was used by the green entrance/end-frame meshes.
  The raised standalone banner remains green; other materials, grass textures,
  geometry and collision mesh bytes were not changed.
- Backup: `models/models/miss2_env/miss2.glb.before_corridor_recolor`.
- Full model LiDAR mount history: +0.18 m -> 0 m (BAD: self-return) -> current
  **-0.25 m**. The body/leg geometry was inspected: leg bottoms at -0.195 m;
  current scan plane is below them. Moving to Z=0 caused a front reading about
  0.06 m and immediate false approach termination. After -0.25, user logs again
  showed correct 0.49–0.50 m approach cutoff, but PRE_ENTRY still failed.
- The new isolated model restores TOP mount +0.18 at a deliberately lower
  takeoff height. Do not make the two model files match automatically.

## Full-world launch commands (historical pipeline, not the next baseline test)

Terminal 1:

```bash
sudo ip link set dev lo multicast on
sudo ip route replace 239.255.0.7/32 dev lo src 127.0.0.1
export GZ_VERSION=harmonic
export GZ_IP=127.0.0.1
export GZ_PARTITION=miss2_local
export GZ_DISCOVERY_MULTICAST_IP=239.255.0.7
export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$HOME/sae_mission2/world/models/models:$HOME/ardupilot_gazebo/models:${GZ_SIM_RESOURCE_PATH:-}"
export SDF_PATH="$GZ_SIM_RESOURCE_PATH"
gz sim -r -v4 "$HOME/sae_mission2/world/worlds/miss2_full_world.sdf"
```

Terminal 2:

```bash
cd ~/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --no-mavproxy
```

Terminal 3:

```bash
mavproxy.py --master=tcp:127.0.0.1:5760 \
  --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14552 --console
```

MAVProxy prompt: `mode guided`, `arm throttle`, `takeoff 3` (one at a time;
wait for hover). `set streamrate 20` can be used to request faster streams.

Terminal 4:

```bash
cd ~/sae_mission2/world
export GZ_IP=127.0.0.1
export GZ_PARTITION=miss2_local
export GZ_DISCOVERY_MULTICAST_IP=239.255.0.7
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
export PYTHONPATH="$HOME/sae_mission2/approach:$HOME/sae_mission2/corridor:/usr/lib/python3/dist-packages"
gz topic -i -t /iris/camera_forward/image_raw
gz topic -i -t /iris/lidar/scan
python3 -u integration/experimental_corridor_manager.py \
  --mavlink udpin:0.0.0.0:14552 --banner-loss-frames 5 \
  2>&1 | tee /tmp/sae_corridor_test.log
```

Full sensor topics also include forward/downward camera_info, downward image,
and `/iris/lidar/scan/points`. Only forward image and scan feed the manager.

`No subscribers` BEFORE launching the manager is normal. A publisher listed
means a sensor publisher exists. The Python subscribe return `True` only means
subscription setup succeeded, not that frames are arriving. New terminals
must get the matching partition/IP environment; earlier shortened commands
omitted exports and caused a camera-wait issue.

## What was tested and what was not

Verified during this session:

1. Python syntax/import checks for changed detector, full manager, altitude
   helper and isolated runner/replay. Installed Gazebo bindings were used.
2. `integration/test_corridor_altitude.py`: 11 tests passed after timing work;
   later gain parameter changes retained all 11 passes. Tests cover direction,
   bounded speed, continuous dwell, nonfinite/stale/missing data, timeouts,
   1 Hz telemetry, repeated sample not completing dwell, disturbance recovery,
   invalid settings, slow simulation, clock reset and wall watchdog.
3. Additional ad-hoc executable checks (not all persisted as test files):
   native MAVLink signs/mask, altitude ingestion, approach loss transition,
   native runner construction only after relative descent/hover, 2-second
   hover delay, and fast descent (~3.24 s in an ideal-response test).
4. Synthetic OpenCV frames verified current relaxed approach accepts clipped
   panel/shape-size changes, retains 700 px2 threshold, and exposes missing/
   too-small rejection reasons. Earlier strict-filter synthetic checks passed
   too, but real rendering showed those rules were inappropriate; do not
   equate synthetic image passes with real-world detector validation.
5. User logs verified full-world approach at 0.5 m, descent ~1 m, stable hover,
   and then failed geometry acquisition. These are observations, not proof of
   corridor traversal success.
6. GLB edit validation: binary buffers and all non-material data unchanged;
   all other materials unchanged; standalone banner still green.
7. SDF XML checks; `gz sdf -k .../corridor_only.sdf` reported Valid. A stock IMU
   `gz_frame_id` extension warning was emitted, not a validation failure.
8. Shell syntax via `bash -n` for experiment launch scripts; installed
   sim_vehicle `--use-dir` and MAVProxy `--streamrate` support inspected.
9. Offline native FSM traversal PASS, with persisted report.
10. Real headless Gazebo scan validation PASS, with persisted raw scan/report.
11. Isolated runner scan adapter and FLU/NED velocity/yaw signs checked.

Re-run relevant checks after changing code, not merely because another turn
started. Full flight and obstacle branches remain unverified in the new world.

## Problems, unsuccessful attempts, and lessons

### Camera and handoff

- Original manager streamed zero vertical speed during native traversal;
  one-shot MAVProxy descent was overwritten. The initial altitude issue did
  not require a hardcoded 2 m target to exist.
- Multiple handoff designs were tried as user preferences changed: fixed
  operating altitude before approach; range-triggered descent; camera-loss
  handoff with no descent; then current relative descent+hover. Use CURRENT
  code/settings above, not chronological suggestions.
- Increasing minimum green area from 1000 to 1500 and later lowering to 700
  did not distinguish panel from larger green corridor frames.
- Strict shape/solid-fill/temporal tracking rejected real panel detections or
  lost them early. Some initially applied filters affected search too; that
  caused complete failure to acquire. They were backed out for approach.
- Recoloring corridor frames gray, then relaxing approach tracking allowed
  reliable panel detection up to the actual range threshold in user runs.
- A clipped bounding-box center is only the center of its visible portion.
  When the panel fills the image, pixel centering no longer proves true panel
  alignment; this is an unresolved measurement limitation, not a confirmed
  explanation for the LiDAR failure.

### Altitude, clocks, and sensor mounting

- A 0.5-wall-second stale threshold caused alternating +0.30/0 vertical
  commands and extremely slow descent. Descent now tolerates up to 1.5 s,
  retries streams and logs pose age.
- Even continuous descent timed out because 25 wall seconds did not represent
  enough simulator time. Descent/dwell switched to autopilot boot time;
  stale-data checks stayed on wall time. It now reaches the target in logs.
- User requested more lenient timeouts, faster descent and a two-second hover;
  these are implemented with fresh PRE_ENTRY timers only afterward.
- Moving LiDAR into the body at Z=0 produced ~6 cm forward self-returns. This
  was an assistant-introduced mounting mistake, corrected to -0.25 m in the
  full model. Do not hide self-returns by globally ignoring all short ranges;
  real obstacles must still be detected.
- Changing only model/world files requires restarting Gazebo. Restarting the
  Python manager does not reload material or sensor pose edits.

### Remaining full-world LiDAR failure

Concrete observed snapshot at failed PRE_ENTRY:

```text
EKF z approximately -2.58 m
confidence=0.0, strict_valid=false, loose_valid=false
front_clearance=12.0, width=Infinity
sector L ~=1.947 m; FL/F/FR/R = None
```

Offline replay established 5 left and 6 right fitting candidates and both
RANSAC fits invalid. This directly explains why PRE_ENTRY keeps STOP and
eventually enters HOVER_AND_REASSESS/ABORT. It does NOT yet establish the root
physical reason the scan lacks wall returns.

The callback's `LaserScan.world_pose` snapshot was identity (0,0,0, quaternion
identity). It is not trustworthy as the sensor's actual world pose in these
runs. Do not infer vehicle height or position from it. A future investigation
needs a real Gazebo model/link pose topic/service or other verified reference.

Height above EKF origin is not AGL or Gazebo world Z. Earlier statements that
the scanner might be above the roof were hypotheses; the logged identity pose
did not confirm them. Exact world/EKF offset remains unmeasured.

The plain-world real sensor test shows two valid walls with the same broad
scan convention, but full-world positioning, mesh visibility, orientation,
mounting/occlusion, and wall-fit window assumptions still need isolation. Do
not declare any one of these the root cause without additional evidence.

More timeouts alone cannot create missing geometry. The native source was
kept unchanged while investigating because the clean-scan FSM already works.

### Diagnostic artifacts and limitations

The full manager now writes a one-time capture at the first native step:
`/tmp/sae_preentry_scan.npz` and `/tmp/sae_preentry_geometry.json`, and prints
`[PRE_ENTRY SNAPSHOT]`. These are overwritten on later runs and /tmp is not
durable. They and `/tmp/sae_corridor_test.log` were no longer visible during
handoff inspection. The prior scan's plot/report persist under the experiment
artifacts; the real headless test has its own persisted raw scan.

The full manager's first-step snapshot can include a default/unsupported
world_pose. It reports geometry from the same step but its fields may stay at
defaults when fitting returns early. Treat missing sectors as no usable return
under current filters, not proof of an empty world.

## Constraints and things to preserve

- User wants direct diagnosis and complete copy/paste terminal commands, minimal
  unnecessary theory, and no repeated permission questions for authorized work.
- Inspect actual code and evidence. Several previous guesses did not solve
  the failure; avoid further blind threshold/altitude changes.
- Preserve working Gazebo/ArduPilot physics, IMU, camera topics, sensor adapters,
  native yaw-rate support and FLU -> NED signs.
- Preserve native `native-non-ros` branch and unrelated dirty edits.
- Do not run the legacy approach autonomous mission runner simultaneously with
  either experimental controller. One controller owns movement commands.
- Keep full-world and isolated-test model/resource/partition distinctions clear.
- Never automatically lower another 1 m every time hover reacquires altitude;
  keep the same latched target.
- Do not remove stale-sensor, flight-mode, arming, front-stop, or terminal LAND
  behavior merely to force progress through a state.
- Keep new simulation settings scoped to experimental instances rather than
  changing real-aircraft controller defaults.
- Full flight success must be established from measured logs/state progression,
  not SDF validity, subscription True, offline passes, or read-only scans.
- No deployment, commits or remote writes are required for the current task.
- User asked to stop implementation after writing this handoff. Future work
  resumes when the next session/user continues the experiment.

## Prioritized next steps

1. Read this file and `experiments/corridor_only/README.md`. Check actual file
   state and whether the user has already run the isolated test since handoff.
2. Inspect `artifacts/flight.log`, `artifacts/inspection.log`, and new NPZ/JSON
   captures if present. If absent, give/use the four-terminal commands above.
   The new task is full corridor-only SITL flight, not another camera edit.
3. Before motion, run `mission.sh --inspect` after takeoff 1.3. If it differs
   from the verified ground sensor geometry, compare airborne scan/height and
   vehicle attitude. Do not blindly start the FSM with bad geometry.
4. Run native corridor flight. Verify PRE_ENTRY lock, measured ENTER distance,
   cruise, exit commit, CORRIDOR_EXITED, and user-commanded LAND. Inspect exact
   failure point if it stops. Review scan/pose freshness, modes and timing
   rather than assuming the same previous failure.
5. If baseline passes, capture evidence and only then add a separate obstacle
   experiment if requested/needed. Baseline does not test avoidance branches.
6. Compare known-good isolated scans with freshly captured full-world scans.
   Obtain actual Gazebo vehicle/LiDAR pose through a verified source; do not
   reuse identity LaserScan.world_pose as truth. Investigate why the original
   mesh view lacks fitting candidates on both sides.
7. Apply a targeted fix to the full integration only after the root cause is
   supported. User's deferred forward-advance stage is an option, not current
   implementation and not a substitute for correct scan geometry.

## Update: full-world mesh repair, 2026-09-21

The full-world LiDAR failure was isolated to the corridor cover mesh's
outward-only triangle winding. The static production test at the inferred
handoff pose gave 87 finite beams, 6/5 wall candidates and confidence 0.
The same asset with triangle order reversed gave 210 beams, 88/88 candidates
and confidence 0.881613 with strict geometry valid. The full details and raw
reports are in `experiments/full_world_lidar_ab/DIAGNOSIS.md`.

The repair is now applied to
`models/models/miss2_env/miss2.glb`: it preserves each original outward face
and adds a reversed counterpart for both corridor covers. Wall positions,
width, material/color, banners, controller and vehicle models were unchanged.
Backup of the pre-repair current GLB:
`models/models/miss2_env/miss2.glb.before_two_sided_corridor_repair`.
The post-repair production-mesh control passed: 210 finite beams, 88/88 wall
candidates, confidence 0.881613, `strict_valid: true`.

The existing full-world Gazebo server was running before this asset edit, so
it still has the old mesh. It must be stopped and relaunched before a real
full-mission test. The dynamic flight model's LiDAR joint was not changed.
`--no-joint` applies only to the static diagnostic harness because that harness
has a separate fixed-joint artifact. No successful full-world flight yet.

## Update: ENTER_CORRIDOR MAVLink pose freshness, 2026-09-21

After the mesh repair, a real full-world run passed descent, PRE_ENTRY wall
lock and yaw alignment, then entered `ENTER_CORRIDOR`. It aborted because
`LOCAL_POSITION_NED` temporarily exceeded the native 0.5 s pose-age gate.
The project read-only monitor showed MAVProxy's default UDP fan-out delivered
this message at roughly 3.75 Hz, so a single delayed packet can exceed that
gate. This was separate from LiDAR geometry.

Changed `integration/experimental_corridor_manager.py` to request position and
attitude telemetry at 20 Hz through both `MAV_CMD_SET_MESSAGE_INTERVAL` and
the ArduPilot compatibility `REQUEST_DATA_STREAM` mechanism. Changed native
`MissionRunnerConfig` to make the enter pose-age limit configurable; the full
simulation manager explicitly uses a bounded 1.0 s value, while native default
remains 0.5 s. The existing 2 s no-pose abort remains. No vehicle-control,
mode, arm or land command was used to test streams.

Live disarmed SITL verification of these same telemetry-only requests measured
98 `LOCAL_POSITION_NED` messages in six seconds (17.1 Hz; max gap 0.324 s).
`test_exit_pose.py`, Python compilation, a pose-age boundary test and mocked
MAVLink request test passed. `test_pre_entry.py` and `test_cruise.py` could not
run because they require unavailable `/dev/ttyUSB0`; this is unrelated to the
changes. The existing simulator process need not be restarted for this Python
change: re-arm/take off and rerun the manager from Terminal 4.

## Update: handoff distance and descent, 2026-09-21

The camera-to-LiDAR handoff range remains 0.5 m before the corridor through
`--entrance-commit-range` (default 0.5). The confirmed full-flight log entered
descent at `front=0.49 m`, so that trigger was already correct. The relative
vertical descent remains the original 1.0 m through `--pre-entry-descent`
(default 1.0). Descent speed, altitude settling tolerance, two-second hover,
LiDAR mount and corridor FSM were not changed.

## Update: continuous position-loss timeout, 2026-09-21

The next full run again locked PRE_ENTRY and entered `ENTER_CORRIDOR`, but a
temporary position stream gap caused the native runner to abort. The original
runner incorrectly compared its 2 s pose timeout against the total age of
ENTER_CORRIDOR, even when earlier samples were valid. `native/mission_runner.py`
now tracks `enter_pose_missing_since` and times out only after a continuous
missing-pose interval. The integration manager sets that continuous-loss limit
to 6 s; it commands a stop while pose is unavailable and resumes only after a
fresh sample. `enter_corridor_timeout_s=40` remains the overall entry deadline.

The manager now logs position/attitude ages and position-message counts in
native diagnostics, and logs every stream-request retry. Python compilation,
the existing exit-pose test, and a new mocked continuous-loss boundary test
passed. This Python-only change needs no Gazebo/SITL restart; re-arm/take off
and rerun Terminal 4.

## Next Session

Start by reading `/home/sid/sae_mission2/world/HANDOFF.md` (this canonical file)
and `/home/sid/sae_mission2/world/experiments/corridor_only/README.md`. If starting
from the Desktop copy, `cd /home/sid/sae_mission2/world` before repository work.
Check the world, corridor and approach Git status independently; preserve the
native `native-non-ros` branch and unrelated dirty files.

Read `experiments/corridor_only/run.py`, `env.sh`, the four launch scripts and
the persisted `artifacts/sensor_validation/live_geometry.json` plus
`artifacts/synthetic_fsm.json`. The real headless scan and analytical FSM tests
passed. Full corridor-only SITL flight has not yet been verified. Continue
THAT task using the four-terminal commands, or inspect its new logs if the user
has already run it. Do not restart the old camera-debugging sequence.

Check background processes before launch; prior test exec session IDs are no
longer usable. The last completed test cleaned up its background Gazebo and
the last host check found no flight-stack processes, but do not assume that
remains true. Gazebo networking may require execution outside the Codex
sandbox; use the proper approval mechanism, not alternate tools to bypass it.

Keep logs/captures under `experiments/corridor_only/artifacts` so they survive
session changes. The handoff is also saved at `/home/sid/Desktop/HANDOFF.md`
and `/home/sid/sae_mission2/HANDOFF.md`; update those copies from the canonical
file when refreshing context. At the end of the handoff-writing turn, stop:
the user explicitly prohibited further implementation in that turn.
