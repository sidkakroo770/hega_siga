# Full mission: corridor to coverage (Gazebo)

## Current default mission — 2026-10-08

The default runner now includes initial/field QR inspection and a return extension
after the matched target's five-second 5 m hold. Return retains field safety,
acquires the orange banner, repeats a fresh native corridor FSM and requests
exterior landing with on-ground/disarmed confirmation. Current validation and
launch instructions are in `../../docs/return/mission_status.md`.
The older segment description below applies to the explicit coverage-only
regression profile, not the current full-mission endpoint.

`experimental_corridor_manager.py` now runs the original banner/approach/LiDAR
corridor stages unchanged through `EXIT_DETECTION`. On `CORRIDOR_EXITED` it
advances north into the registered field inset, climbs to 10 m HOME-relative
altitude, and hands the same MAVLink connection to the copied, validated
`coverage_mission` runtime. The manager sends no velocity commands during
coverage. Coverage completion ends this mission segment in a position hold;
there is no return, delivery, QR logic or automatic landing.

The fixed full-world field is local NED N=[-20,20], E=[-15,15]. This is
**not** spawn-relative: live SITL reported N=-30.4 at the banner, matching
Gazebo world Y=-30.4, and N≈-19.4 at corridor exit. The two red test zones
are N=[-9,-6], E=[-4,-2] and N=[8,11], E=[6,8]. The field height and
HOME/LOCAL-Z offset were checked in the full Gazebo world, not copied blindly
from the isolated fixture. The provisional downward camera is 640×480,
28.2° horizontal FOV at 15 Hz, 5 cm below the vehicle reference. The front
camera and LiDAR remain as before.

The original GLB also contains a red ground strip approximately
N=[-2.25,7.75], E=[-5.11,2.32]. It is detected by the downward camera and
can force a long, already-observed connector around both red areas. The
full-world config permits up to 120 source seconds without new map/traversal
credit for that connector; camera, clock, worker and localization watchdogs
remain much shorter. A live first run stopped at the isolated fixture's
40-second progress limit while safely routing around these areas, so the
full-world completion claim required a new run with this corrected profile.
That retry traversed 2,084 required points without a sampled red or fence
incursion, but hit its 3,600-second wall limit with 1,138 points still pending.
Independent truth confirmed projection and tracking remained within their
specified limits; it did **not** confirm full-field coverage. The subsequent
planner change prioritizes nearby reachable coverage over a distant unknown
ordered point. It passed the synthetic regression suite, but a new full-world
flight is still required before claiming complete Gazebo validation.

In the later accelerated full-world run (`artifacts/fullworld_oct2_third`),
the controller reached 3,161 measured waypoints and the independent truth
evaluator found zero permissible coverage gaps, zero sampled red/fence
incursions, and a 0.135 m maximum projected-corner error. The controller
nevertheless ended `BLOCKED`: 54 points and 2,519 unseen cells lay inside a
non-red island entirely enclosed by mapped red, while one outer-edge point
was missed by 0.282 m against the old 0.24 m arrival radius. The updated
controller exempts only islands enclosed by *confirmed* red; it does not
credit those points as traversed. The full-world arrival radius is now
0.30 m (the configured uncertainty), without changing red clearance. These
completion changes pass unit/synthetic tests but **have not yet had a fresh
end-to-end Gazebo flight**. Do not label the controller `COMPLETE` based only
on the earlier independent geometry result.

## Run the coverage-only regression simulation

The default manager now implements the initial/field QR mission and a 5 m
startup. For that complete sequence, use the repository-root
`../../docs/qr/mission_status.md` instructions. The following retains
the historical coverage-only regression and deliberately skips QR tasks.

Use separate terminals. First, with the usual Gazebo/SITL dependencies installed:

```bash
export GZ_IP=127.0.0.1
export GZ_PARTITION=miss2_integrated
export GZ_DISCOVERY_MULTICAST_IP=239.255.0.7
export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/sid/ardupilot_gazebo/build
export GZ_SIM_RESOURCE_PATH=/home/sid/[competition]_mission2/world/models/models:/home/sid/ardupilot_gazebo/models
gz sim -r -v2 /home/sid/[competition]_mission2/world/worlds/miss2_full_world.sdf
```

Second terminal:

```bash
cd /home/sid/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris \
  --model JSON --no-mavproxy \
  --use-dir /home/sid/[competition]_mission2/world/integration/artifacts/full_sitl_smoke
```

Third terminal:

```bash
mavproxy.py --master=tcp:127.0.0.1:5760 \
  --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14552 --streamrate=20
```

Wait for ArduPilot telemetry to settle. Leave the vehicle disarmed on the
ground. The manager confirms pre-arm health, requests GUIDED and normal arm,
takes off to the provisional Gazebo HOME-relative 3 m target, and checks a
continuous settled hover before starting banner search. It rejects an
already-armed restart or an FC command rejection. Do not manually arm or
take off before running it.

Fourth terminal:

```bash
export GZ_IP=127.0.0.1
export GZ_PARTITION=miss2_integrated
export GZ_DISCOVERY_MULTICAST_IP=239.255.0.7
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
export PYTHONPATH=/home/sid/[competition]_mission2:/home/sid/[competition]_mission2/approach:/home/sid/[competition]_mission2/corridor:/usr/lib/python3/dist-packages
python3 -u /home/sid/[competition]_mission2/world/integration/experimental_corridor_manager.py \
  --start-mission --coverage-only --takeoff-altitude 3 \
  --mavlink udpin:0.0.0.0:14552 --banner-loss-frames 5 \
  --coverage-max-wall-seconds 7200
```

The coverage GUI remains enabled. Logs and a machine-readable terminal
result are written under `world/integration/artifacts/coverage_runtime.*`.
The manager exits nonzero on abort.

For an independent pose trace, start this **before the manager** with the
same Gazebo environment and `PYTHONPATH=/home/sid/[competition]_mission2`:

```bash
python3 -m coverage_mission.truth_monitor \
  --config /home/sid/[competition]_mission2/config/full_mission_coverage.json \
  --zone -9 -6 -4 -2 --zone -2.25 7.75 -5.11 2.32 \
  --zone 8 11 6 8 \
  --model-name iris_miss2_full \
  --pose-topic /world/miss2_world/pose/info \
  --ground-z 0.0731022 \
  --output /home/sid/[competition]_mission2/world/integration/artifacts/full_truth.jsonl \
  --seconds 7400
```

After a completed trace, independent evaluation is:

```bash
PYTHONPATH=/home/sid/[competition]_mission2 python3 -m coverage_mission.evaluate_run \
  --mission /home/sid/[competition]_mission2/world/integration/artifacts/coverage_runtime.jsonl \
  --truth /home/sid/[competition]_mission2/world/integration/artifacts/full_truth.jsonl \
  --output /home/sid/[competition]_mission2/world/integration/artifacts/full_evaluation.json
```

This is still a Gazebo adapter: it uses Gazebo image/clock topics and a
provisional camera geometry. The Pi camera source, hardware time alignment,
intrinsic/mount calibration and on-device profiling remain required before
real flight.

## Post-corridor implementation update — 2026-10-06

This code is **not yet re-proven in Gazebo after these edits**. The development
sandbox denied socket creation (`PermissionError: Operation not permitted`) when
starting the isolated campaign. Run the existing full-world command above in a
normal terminal with a fresh owned Gazebo/SITL instance and independent truth
monitor before treating the updated mission as Gazebo validated.

- `CORRIDOR_EXITED` now accepts an optional ordered four-corner GPS geofence
  (`geofence_latlon` in coverage JSON, `[SW,NW,NE,SE]` in the field's own
  orientation). It registers it against the FC's `GPS_GLOBAL_ORIGIN` and rejects
  a missing origin/nonrectangular geofence. The old fixed full-world coordinates
  remain the Gazebo default. Advance follows the measured corridor-exit bearing
  and verifies the resulting climb point lies inside the field clearance inset.
  The coverage map/commands use a rigid field-frame transform. Actual surveyed
  geofence coordinates and a verified red-free exit/climb patch remain site inputs.
- The command service remains alive across handoff. The manager claims a new
  coverage token, revoking corridor proposals; the same leased sender issues
  coverage LOCAL_NED velocity. Its expired proposal becomes zero velocity, and
  shutdown transmits a final zero. Coverage's source-clock/pose/EKF supervision
  remains in the coverage runtime, while the top-level manager owns stage
  transitions and final command-authority release. No return runtime exists yet.
- Coverage GUI is in a separate optional process (latest-only, 1/5 decision
  refresh). `--no-gui` now propagates from manager. Flight decisions continue if
  GUI processing stalls. A `q` requests mission abort, not a hardware E-stop.
- Red detection uses configurable wider provisional HSV bounds; one hit blocks
  motion, five distinct admitted images confirm a persistent zone. A planar
  homography projects per-pixel red and usable masks onto the ground, so dark
  or saturated pixels cannot certify clear ground. This replaces per-contour
  full-grid raster allocation. Confirmed-red clearance inflation determines
  physically unreachable islands, including narrow openings; exclusion is not
  traversal credit.
- Pose displacement is checked against measured velocity. Coverage supervision
  independently advances a known red-residence warning/deadline while its worker
  stalls. Terminal position hold requires fresh heartbeat, EKF, synchronization,
  origin and position; transport failure no longer bypasses worker/log cleanup.
- Conservative 3x/2x checked-grid route search makes the benchmarked full-field
  rectangular detour ~73 ms on this desktop (formerly ~640 ms after basic
  simplification, ~856 ms in the review), with exact fine-grid fallback. The
  1,000-patch red-image benchmark is ~4.5 ms median (review: ~44 ms). These are
  *not* Raspberry Pi 5 or worst-case hard-deadline measurements.

Validation in this workspace: 76 current offline/synthetic/integration tests
passed, one inherited test in `/home/sid/[competition]_mission2_coverage/tests` was
deliberately deselected because it asserts the superseded two-frame confirmation
rule. The new five-frame rule has a local regression test. Gazebo campaign could
not start due sandbox networking, not because of a demonstrated flight failure.

Pending before physical flight: real IMX296 global-shutter camera adapter and
exposure/FC timestamp alignment; real intrinsics/crop/distortion and mount
measurement (nonzero distortion or mount rotation currently fails closed);
HSV/site trials, vehicle/dynamics margins, Pi worst-case/two-camera/LiDAR
benchmark, FC/link failsafe and terminal continuation tests. QR, delivery and
return remain out of scope. The existing `--coverage-max-wall-seconds` is a
test guard, not an optimized mission time budget.

## Gazebo continuation — 2026-10-06 (current run in progress)

Socket access is now available. The previous sandbox warning above is
historical, not the current test environment. The 1× full-world run proved
autonomous takeoff, corridor exit, field advance and the 10 m coverage handoff,
then exposed two integration defects: the dark grass texture fragmented the
old usable-pixel map at entry, and a later planner choice could route to its
own current cell when an ordered point was unknown. Neither partial run is a
full-field pass. Partial independent Gazebo traces had zero red/fence entries.

The full-world Gazebo profile now opts into a provisional 0.4 m local
green-texture evidence support, derived in pixels from camera geometry; the
generic profile remains unchanged. Black and dark neutral/red regions remain
unknown. An unroutable HOLD has its own source-time deadline independent of
hover-jitter pixels. Dense traversal points remain, while straight-lane
look-ahead selects only checked, unfinished points. Known reachable obligations
take priority over unknown ordered ones, and frontier viewpoints must require
meaningful motion rather than a route to self. These are software contracts,
not final outdoor color calibration.

Focused current-code tests: **46 passed**. A current-code isolated two-red-zone
Gazebo campaign completed and independently passed: zero red incursions, zero
fence violations, zero permissible coverage gaps, maximum projected-corner
error 0.103 m, maximum straight-leg cross-track 0.083 m, maximum truth sample
gap 0.035 s. Evidence:
`/tmp/[competition]-coverage-oct6-final-check2/evaluation.json`.

The earlier original 40 × 30 m full-mission re-run reached coverage and
independently sampled zero red/fence violations, but its `/tmp` evidence and
live processes were lost across an environment restart before completion.
A fresh re-run from autonomous takeoff is underway with persistent evidence at
`world/integration/artifacts/full_recheck_20261006/`. Do **not** claim
full-world completion until its mission result and independent evaluator both
pass.
Pi camera timing/calibration, concurrent real-sensor workload, site colors,
physical margins and FC failsafes remain unvalidated.

## Full-world liveness follow-up — 2026-10-06

The persistent `full_recheck_20261006` run reached coverage after autonomous
takeoff and corridor exit, but did **not** complete. Independent truth samples
showed zero red/fence incursions and at least 0.526 m body-edge red clearance;
the vehicle then repeatedly revisited roughly 20 m of safe ground while
pending/unseen counts stopped decreasing. This is a confirmed full-world
planner-liveness failure, not a successful mission. The trace and saved map are
under `world/integration/artifacts/full_recheck_20261006/`.

The map showed small enclosed dark-texture observation holes inside a much
larger still-unseen exterior. The route planner now prioritizes exterior
unknown components for frontier gain until exterior exploration is finished;
interior holes remain obligations. The progress watchdog now requires a new
traversal obligation or a configured area of new ground evidence, not one
jittering pixel. Full-world Gazebo uses a provisional 2.0 m² observation
progress threshold; the generic default is 0.25 m². Two regression tests were
added. Focused current-code suite: **48 passed**.

After this correction, a current-code isolated two-red-zone Gazebo campaign
independently passed: `COMPLETE`, zero red incursions/fence violations/gaps,
0.099 m maximum projected-corner error, 0.025 m maximum straight-leg
cross-track error and 0.227 m maximum credited-route cross-track error.
Evidence: `world/integration/artifacts/isolated_after_frontier_fix/evaluation.json`.
The original full-world mission is being rerun from autonomous takeoff with
persistent evidence under `world/integration/artifacts/full_frontier_20261006/`.
Its result remains **pending** until mission completion and independent truth
evaluation; isolated success cannot substitute for that result.

## Original-world second failure and fixture correction — 2026-10-06

The `full_frontier_20261006` run passed the previous ~900 s failure point but
eventually ended `BLOCKED` at source time 2559.55 s with 884 pending points.
The independent evaluator recorded the entire flight, zero sampled red/fence
incursions, 0.084 m maximum projected-corner error, 0.079 m maximum straight-leg
cross-track error and 0.296 m maximum credited-route cross-track error. It
also found **11,826 permissible grid cells with no admitted camera footprint**,
so this was not a full-field pass. Evidence and diagnostic maps are in
`world/integration/artifacts/full_frontier_20261006/`.

The saved map showed large unobserved sectors and no pending lane points in
the known-free reachable component. The environment GLB's central red-ground
mesh appears as partly red with a dark/unrendered interior from the downward
camera. The full-world SDF now adds a paint-only red visual over the *same*
central zone bounds (no collision and no zone coordinates supplied to the
controller). This corrects fixture observability rather than using truth in
navigation. The full-world-only local green texture support was enlarged from
0.4 to 1.0 m to cover dark grass patches; large pure-black regions remain
unknown. A regression test covers that distinction. **60 finite tests pass.**

A fresh 1× original-world run from autonomous takeoff is underway at
`world/integration/artifacts/full_paint_20261006/`. It must complete and pass
independent truth evaluation before any Gazebo full-mission success claim.

## Long-connector watchdog correction — 2026-10-07

`full_paint_20261006` also ended `BLOCKED`, but much later, at source time
3765.57 s with 95 pending points and 1,704 unknown permissible cells. The
independent evaluator found 1,577 geometrically unviewed permissible cells,
zero sampled red/fence incursions, 0.078 m maximum projected-corner error,
0.084 m maximum straight-leg cross-track error and 0.298 m maximum credited
route cross-track error. It was still a **failed** full mission. The last
150 s of the trace show a genuine long checked transit from the southeast
toward the remaining northeast corner, not a stationary loop; the fixed
120-source-second evidence-only watchdog stopped the vehicle before it
could reach that corner.

The watchdog now grants finite extra transit time based on the maximum *net*
displacement from the last measured traversal/observation progress point,
using a provisional 0.20 m/s lower-bound connector speed and a field-diagonal
cap. Circling or hovering cannot continually refresh it. One new regression
test checks that a long checked relocation survives while a repeated no-gain
position still times out. The full-world profile states the speed assumption
explicitly; **61 finite tests pass**. A fresh 1× original-world mission and
independent pose monitor are running under
`world/integration/artifacts/full_relocation_20261007/`. Full validation is
still pending. This software liveness change is not a physical-flight timing
or battery assessment.

## Saved-map replay instead of repeated full flights — 2026-10-07

The interrupted `full_relocation_20261007` run has no terminal result and is
**not** a pass. Its fresh retry, `full_relocation_retry_20261007`, reached 359
pending points, then aborted after repeated 200 ms route-search timeouts made
decisions stale at 2× simulation speed. Independent truth recorded zero
red/fence incursions, 0.098 m maximum projected-corner error, and 6,209
geometrically unviewed permissible cells. The saved terminal map reproduced
the timeout offline: an unnecessary soft inverse-clearance A* cost took over
200 ms for a long connector, while the same connector with the hard clearance
mask and exact segment checks took about 95 ms. The soft cost was removed;
hard body/uncertainty clearance was not reduced.

The next complete full-world run, `full_final_20261007`, no longer had planner
timeouts. It ended `BLOCKED` with 40 required lane points around tiny dark
printed ground patches. Independent geometry had zero permissible footprint
gaps and zero red/fence incursions, but controller completion remained false.
The GLB's two long grass strips also rendered almost black in a camera capture;
the SDF now has paint-only grass backing over those already-grassy strips.
Camera captures verified the backing is visible and the red-zone and QR
visuals remain visible above it. These visuals are a Gazebo fixture correction,
not controller map knowledge.

The `full_neutral_20261007` run, with provisional local neutral-print pixel
support and a route to reachable free cells inside an observed lane point's
arrival disk, reduced the unresolved obligations to 15. All 15 lay on one
lane through a printed QR patch: 22 unknown 0.1 m cells in four tiny enclosed
components had been enlarged by the conservative known-ground clearance
erosion. It safely ended `BLOCKED`, not `COMPLETE`. Independent truth recorded
zero red/fence incursions, 0.101 m maximum projected-corner error and only
eight geometrically unviewed cells. One of 3,161 credited points was 0.307 m
from its lane according to Gazebo truth against the existing 0.300 m criterion;
the controller measured 0.299 m and the FC/truth East difference was 8 mm.
Do not relabel this flight as a pass.

The full-world profile now allows at most 0.20 m² interior unknown components
to be *contextually* inferred clear only if the entire component is away from
confirmed/suspected red and the map boundary. Large, edge-connected or
red-adjacent unknown remains unknown. Direct observation and contextual
inference are stored separately. This is provisional for the known printed
ground scene, not a hardware-validated low-light policy. On the saved terminal
map it inferred exactly 22 cells; every one of the 15 pending lane points then
had a checked route. A kinematic saved-map replay traversed all 15 with two
checked trips, 30.14 m travel and a 33 ms worst route computation. The previous
40-point map also replayed to zero pending in six checked trips. These checks
take under a second and **do not** validate camera timing, flight dynamics,
red safety, or full mission completion. The revised code has **65 selected
offline tests passing**; full-world Gazebo remains **not yet proven COMPLETE**.

Use the quick regression loop before any further long flight:

```bash
cd /home/sid/[competition]_mission2
PYTHONPATH=/home/sid/[competition]_mission2 OPENBLAS_NUM_THREADS=1 \
  python3 -m coverage_mission.replay_saved_map \
  --config config/full_mission_coverage.json \
  --artifact-dir world/integration/artifacts/full_neutral_20261007
PYTHONPATH=/home/sid/[competition]_mission2:/home/sid/[competition]_mission2/approach:/home/sid/[competition]_mission2/corridor:/home/sid/[competition]_mission2/world/integration:/usr/lib/python3/dist-packages \
  OPENBLAS_NUM_THREADS=1 python3 -m pytest -q coverage_mission \
  world/integration/test_camera_pose_integrity.py \
  world/integration/test_corridor_altitude.py \
  world/integration/test_coverage_handoff.py \
  world/integration/test_startup_handoff_output.py
```

Reserve the next complete Gazebo flight for after these fast checks and a
short targeted camera/QR test. A 2× run is faster but has additional brief
frame-validity holds; it cannot replace final Pi camera/FC timing benchmarks.

### Accelerated full-world verification, 2026-10-07

Two autonomous-takeoff full-world flights ran with 2× simulation speed during
coverage. Neither is a controller pass. `full_verification_20261007` finished
`BLOCKED` at zero pending points but 33 unseen cells: those cells were already
separately marked contextually clear, yet completion counted them as unseen.
Completion and frontier accounting now consistently exclude only those
explicitly contextual-clear cells. Its saved-map replay passes at zero pending
and zero unseen. Independent truth found zero red/fence incursions, zero
geometrically unviewed permissible cells, and 0.104 m maximum corner error.
One separate credited-route metric exceeded its 0.300 m bound at 0.307 m, so
this flight is not relabeled a pass.

`full_completion_20261007` ended `ABORTED` with 1,837 pending points after
46 route-planning timeouts at the 200 ms wall budget. At 2× speed, each new
zero-command HOLD could already exceed the 0.2 s source-time decision age,
causing a ten-wall-second supervisor validity abort. Independent truth found
zero red/fence incursions and 0.099 m maximum corner error, but this was a
partial flight. No terminal map was saved, so the exact failing planner state
cannot be replayed. A fresh zero-command HOLD is now admitted without
authorizing stale motion; direct, fine-grid-checked frontier viewpoints are
tried before A*. These targeted liveness changes are **not yet verified by
another full Gazebo flight**. A synthetic direct-frontier test and 68 selected
offline/integration tests pass. Use short targeted scenarios or a reproducible
saved map before another long flight.

### Provisional dark-ground rule and cheap checks

At the user's direction, the full-mission coverage profile treats dark or
black pixels in a fresh, correctly shaped downward-camera frame as ordinary
ground unless the configured red detector marks them red. It skips the
green-texture and neutral-print support blurs and no longer rejects a wholly
black image on content alone. Missing/stale frames remain invalid, but a
fresh black image caused by an actual camera fault would be misclassified as
clear ground. This is a provisional Gazebo/field-color assumption, **not**
evidence that real dark-red or blacked-out imagery is safe.

No new full-world flight was launched. A short synthetic image/map scenario
confirmed that a dark patch becomes observed while a red patch remains a
hazard. The selected test suite passes 68 tests after removal of the dedicated
all-black-rejection test. Saved-map replays of `full_verification_20261007` and
`full_neutral_20261007` both finish with zero pending and zero unseen cells;
these replays predate the new image rule and are planner checks only, not a
fresh Gazebo validation.

### Accepted full-world Gazebo result — 2026-10-07

After a short 2× isolated timing campaign, a full autonomous-takeoff mission
ran the corridor at its previously proven 1× rate, then switched Gazebo and
SITL together to 2× after the coverage handoff. Artifacts are in
`artifacts/full_corridor1x_20261007`. The manager and coverage runtime both
ended `COMPLETE`; the saved map replay has zero pending and zero unseen cells.
The coverage planner recorded no timeouts. Two injected short-campaign faults
(350 ms camera delay and 230 ms worker delay) had independently measured
zero commands after 0.5 source seconds and stopping excursions below their
configured bounds. The separate 2×-from-takeoff candidate stopped in the
corridor recovery guard, so accelerating the proven corridor stage from the
start is **not** part of this accepted result.

Independent full-flight Gazebo truth covered the whole coverage interval
(89,211 samples; maximum gap 0.036 s): zero red-zone incursions, zero fence
violations, maximum projected-corner error 0.143 m against 0.25 m, and
maximum straight-leg cross-track error 0.119 m against 0.30 m. It found 25
geometrically unviewed 0.1 m cells (0.25 m² total), all in one 5×5 patch
beside the red zone at N=-7.75…-7.35, E=-4.45…-4.05. The saved controller map
marks all 25 as within its red clearance-inflated, excluded buffer; none is
directly observed, red, or a pending lane obligation. The independent evaluator
excludes only raw red and therefore still reports `sampled_geometry_coverage_passed=false`.
One of 3,172 credited points measured 0.307144 m East cross-track against
the existing 0.300 m diagnostic bound, at the first coverage timestamp;
the controller's measured East error there was 0.293 m. The evaluator still
reports `credited_route_tracking_passed=false`.

**User acceptance decision:** these two small deviations are accepted for the
current Gazebo milestone, and this run is recorded as a **Gazebo mission pass
with documented exceptions**. Neither evaluator threshold nor source code was
silently changed to manufacture a strict pass. Do not describe the independent
evaluator's two failed booleans as passing, and do not treat this as Raspberry
Pi/real-camera/real-flight certification. No second long coverage flight was
launched after this completion.
