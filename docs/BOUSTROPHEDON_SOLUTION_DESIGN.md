# Boustrophedon coverage segment — proposed solution

Date: 2026-09-23

Status: solution design only. No implementation, source changes, simulator launch, or flight commands are authorized by this document. Implementation awaits the user's next instruction.

This document records the proposed changes from the solution-design discussion and incorporates the user's subsequent clarifications. It supersedes the earlier assumption that any red-zone entry necessarily violates the competition rule. The saved audit remains a historical review, not the current scope or requirements.

## 1. Confirmed scope and user decisions

- Immediate target: an independent Gazebo coverage test, initially separate from corridor traversal.
- Keep two cameras: front-facing and downward-facing.
- Use the downward camera for this segment's ground perception and coverage. Keep the front camera available for viewing and future integration; do not require camera fusion.
- Provisional downward hardware: Raspberry Pi Global Shutter Camera with a 10 mm lens. Lens choice may change; geometry must be configurable through the camera model.
- Later deployment target: Raspberry Pi 5, real cameras, flight-controller/localization data, and real sensor latency.
- Keep the testing GUI.
- Open air at approximately 10 m altitude; airborne-obstacle and LiDAR concerns are out of scope for this segment.
- Hard-coded field/geofence coordinates are acceptable for now, provided they are easy to replace and have a defined coordinate frame.
- The existing X = -19.3...17.9 and Y = -13.4...14.3 values are arbitrary values supplied by a teammate. They are not surveyed bounds or an authoritative field definition.
- The user reports having proven takeoff and corridor behavior previously. Reuse the relevant existing setup and interfaces rather than redeveloping the corridor mission.
- The segment ends when its full required coverage path has been traversed. End in a verified hold; no return journey, payload delivery, QR logic, landing mission, or mission-time optimization is required.
- Do not build speculative organizer-geofence ingestion or finalize the physical lens now.

### Red-zone rule supplied by the user

> The drone cannot be inside the redzone for more than 10 seconds, 5 seconds given within the 10 for the drone to make its way out of the red zone.

Final user clarification: residence time starts at physical entry. After five seconds inside, the drone must autonomously raise its own warning and start the final five-second correction countdown. It must be outside by ten seconds total. There is no external warning or signal to wait for.

| Elapsed residence time | Required autonomous behavior |
|---|---|
| Entry, t = 0 | Start residence tracking; begin checked recovery as soon as entry is detected |
| 0 < t < 5 seconds | Continue immediate recovery; the initial allowance is not a reason to wait |
| t = 5 seconds, if still inside | Raise an onboard warning, show/log it in the GUI, and start the five-second correction countdown |
| 5 < t < 10 seconds | Prioritize exit; display remaining time as 10 seconds minus elapsed residence |
| t = 10 seconds | Exit must be confirmed by this deadline; otherwise record a deadline failure and continue the applicable recovery/contingency rather than declaring success |

Use the aircraft footprint conservatively for engineering boundary checks. That remains an engineering convention rather than a claim about an unspecified official centre-versus-footprint measurement rule.

Recommended behavior remains avoidance of deliberate red-zone entry. The rule gives a recovery allowance; it does not require planning shortcuts across red regions. Add explicit incursion recovery and residence-time monitoring. Do not wait for five seconds before trying to leave.

Detection delay counts against residence time. The warning/countdown is tied to entry plus five seconds, not detection plus five seconds. If an incursion is first recognized after that threshold, raise the warning immediately and use only the time remaining until entry plus ten seconds. No external operator, judge, or ground-station message is required to trigger these actions.

## 2. Existing project context inspected

The workspace contains separate world, corridor, and approach repositories. Avoid assuming one Git repository owns the entire workspace.

Relevant local references:

- [Mission context PDF](</home/sid/[competition]_mission2/autonomous mission_[competition]_[competition].pdf>)
- [Archived sweep prototype](../archive/legacy/boustrophedon_sweep_prototype.py)
- [Previous audit](reviews/boustrophedon_review.md)
- [Coverage generator](/home/sid/coverage_ws/generate_waypoints.py)
- [Canonical world handoff](/home/sid/[competition]_mission2/world/HANDOFF.md)
- [Independent corridor experiment](/home/sid/[competition]_mission2/world/experiments/corridor_only/README.md)
- [Existing camera mission startup](/home/sid/[competition]_mission2/world/automation_ki_baat_cheet/miss2_start.py:162)
- [Existing staging takeoff](/home/sid/[competition]_mission2/world/integration/stage_near_corridor.py:145)

The handoff and launch scripts establish the following existing setup:

| Item | Existing project configuration |
|---|---|
| Simulator | Gazebo Harmonic; gz.transport13 and gz.msgs10 |
| Autopilot | ArduCopter SITL, gazebo-iris frame, JSON physics connection |
| Plugin location | /home/sid/ardupilot_gazebo/build |
| Full-world file | /home/sid/[competition]_mission2/world/worlds/miss2_full_world.sdf |
| Full-world vehicle | iris_miss2_full, with both camera topics |
| Image topics | /iris/camera_forward/image_raw and /iris/camera_downward/image_raw |
| Documented full-world partition | miss2_local |
| Independent corridor partition | [competition]_corridor_only |
| Physics / SITL ports | 9002 / TCP 5760 |
| MAVProxy outputs | UDP 14550 and 14552; experimental runner convention uses 14552 |
| Python setup | PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python; Gazebo bindings available through /usr/lib/python3/dist-packages |
| Network setup | Existing loopback/multicast environment is recorded in world/HANDOFF.md and experiment scripts |

The independent corridor scripts already demonstrate separation of experiment configuration, SITL state/log directories, and launch stages. Reuse that pattern for coverage. The corridor-only vehicle has no cameras, so it cannot be used unchanged for this experiment.

Existing startup code explicitly requests GUIDED, arms, and sends MAV_CMD_NAV_TAKEOFF. Reuse that working command sequence and environment, with outcome verification and bounded waits appropriate to this segment. Do not copy the sweep script's velocity-only climb, and do not assume every fixed sleep in older startup code is an adequate readiness check.

The documentation records real-time-factor problems and MAVLink delivery gaps. Its September 22 update distinguishes simulation-time motion progress from wall-time data-health watchdogs. That experience directly informs the new timing design.

Some saved handoffs contain older incomplete-flight statements. They are historical evidence and do not override the user's report of subsequent proven takeoff/corridor behavior. No flight was rerun for this design task. The exact currently running process configuration was not established.

Keep one movement-command owner. Separate Gazebo partitions do not isolate shared SITL/MAVLink ports. Do not run the coverage controller concurrently with an approach or corridor controller commanding the same aircraft.

## 3. Overall engineering decision

Replace the current sweep/avoidance state machine while retaining Python, NumPy, OpenCV, pymavlink, Gazebo camera transport, and ArduPilot stabilization.

Build:

1. Camera-footprint-based sweep segments.
2. Timestamped downward-camera observations projected onto a flat ground plane.
3. A small ground map recording observed permissible, unknown, and restricted ground.
4. Checked connectors around red regions and field boundaries.
5. Explicit traversal and coverage progress that survives detours.
6. A vehicle supervisor handling motion limits, stale data, incursion recovery, and final hold.

The new red-zone rule does not make image-column avoidance reliable. A timed allowance still requires an estimate of entry, an exit route, and evidence that the aircraft has actually left.

## 4. Reassessment of the highlighted audit findings

| Finding | Current assessment | Proposed resolution |
|---|---|---|
| A14: largest contour only | Definite loss of relevant information when multiple red regions are visible | Use all relevant red regions in a common ground map |
| A15: image columns imply clearance | Image-left/right does not establish aircraft clearance or residence time | Project observations into metres and evaluate vehicle position/footprint against mapped regions |
| A16: disappearance means clearance | Leaving the image does not prove physical exit | Retain mapped regions and confirm exit using valid pose and geometry |
| A17: always move right | An unchecked detour can head toward another zone or the fence | Plan through observed permissible space; choose the route from geometry |
| A20: forward-projection skipping | Definite false-completion bug at row ends | Track identified sweep segments and remaining intervals; never skip rows by heading projection |
| A21: unchecked recovery | An endpoint is not evidence that its connecting route is safe or that intervening coverage is complete | Validate connectors, preserve unfinished work, and add completion passes where required |

The former one-camera mismatch is resolved by keeping both cameras. Lack of LiDAR, QR, payload, return, or landing logic is not a defect within the present segment scope.

Along-row waypoint spacing alone does not determine continuous camera coverage. Cross-track spacing, endpoints, valid observation frequency, and the actual flown path do. A fixed heading can be acceptable with a downward camera when the actual orientation is incorporated into projection and coverage checks.

## 5. Selective reuse of generate_waypoints.py

Extract and adapt resolve_coverage_geometry(), rather than importing the executable workflow.

For a level downward rectilinear camera over a flat plane:

    cross-track footprint W = 2 * h * tan(cross-track FOV / 2)
    lane spacing s = W * (1 - overlap_fraction)

These equations are valid when h is height above that ground plane and the FOV axis really is cross-track. The isolated calculation reproduces the generator's documented original result: 20 m altitude, 66 degrees FOV, and its default overlap give 18 m spacing.

Adaptations required:

- Use ground-relative camera height, not automatically HOME-relative altitude.
- Represent both footprint dimensions.
- Establish the camera-axis relationship to flight direction.
- Account for allowed altitude, attitude, tracking, and projection errors when selecting usable footprint and spacing.
- Derive endpoint placement and edge coverage separately from lane spacing.
- Regenerate spacing when camera geometry changes.

The generator's half-spacing partition inset has a legitimate adjacent-partition purpose. It is not a physical aircraft clearance. Its adapter separately intersects camera-inset planning geometry with safe route space; it does not simply add all offsets. Preserve that distinction, but do not assume the resulting polygon-area validation proves actual camera coverage.

Useful concepts to retain: geometric input validation, footprint/overlap calculation, separate physical and tracking margins, and checking complete connector segments.

Do not import: KML/KMZ ingestion, geographic conversions, multi-drone partitioning, ROS 2 service orchestration, QGC export, return-to-home/landing requirements, or Camera Module 3 defaults. The script delegates actual route planning to other packages and does not solve online red-zone discovery and recovery.

## 6. Provisional camera model

The Raspberry Pi Global Shutter Camera has 1456 x 1088 pixels of 3.45 micrometres, giving an approximate active area of 5.023 x 3.754 mm. With an ideal full-area 10 mm rectilinear lens:

| Quantity | Approximate calculated value |
|---|---:|
| Horizontal FOV | 28.2 degrees |
| Vertical FOV | 21.3 degrees |
| Footprint at 10 m, level downward camera | 5.02 x 3.75 m |
| Ideal cross-track spacing at 30% overlap | 3.52 m |
| Same calculation at 9.5 m | 3.34 m |

These are provisional estimates, not calibration of a specific physical lens. Source for sensor specifications: [Raspberry Pi camera documentation](https://www.raspberrypi.com/documentation/accessories/camera.html).

Start with 30% requested overlap, applied to a conservative usable footprint. Intrinsics, image dimensions/cropping, distortion parameters, and camera mounting transform are the actual geometry contract. A later lens change updates configuration/calibration rather than the algorithm.

Make the isolated Gazebo rendered camera agree with that model. Do not validate a nominal 10 mm configuration using the existing approximately 60-degree downward rendering. Scope any later simulator camera edits to the coverage experiment, preserving the working corridor setup.

## 7. Field definition, planning, and progress

### Field and coordinate frame

Replace the arbitrary sweep bounds. For the independent baseline, define an explicit rectangular test field, using the PDF's nominal 40 x 30 m as a test dimension rather than claiming it is a surveyed competition fence. Locate it at a declared pose in the test world and place the aircraft in a known permissible start area.

For subsequent testing against the existing full-world delivery area, derive bounds and placement from that actual scene and verify their transformation to local NED. Neither the old arbitrary values nor the diagram alone establishes the existing mesh's boundary.

Use the local NED north/east plane for planning. Define the flat ground-plane height explicitly and include the camera mounting offset in ground-relative height. Verify the world-to-NED mapping. A localization-origin reset invalidates the map and route until reinitialized.

### Coverage segments

Replace the 4 x 4 matrix with parallel segments along a configured field axis. Determine spacing from conservative camera coverage, fit the final lane without leaving an edge gap, and place endpoints using the along-track footprint.

Keep separate:

- Ground requiring coverage: field minus restricted red ground.
- Permissible aircraft-centre positions: field inset and exclusions expanded by aircraft size and uncertainty.
- Ground actually observed in valid images.
- Required route intervals and their traversal status.

Clearance buffers must not silently delete permissible ground from the coverage requirement. Ground near a red region may be observed from a safe offset. Unobservable/unreachable required ground remains explicitly unresolved.

Track each required segment interval as pending, partial, completed, or obstructed. Detours preserve the interrupted work. Split obstructed lanes, route to remaining portions, and add local completion passes for residual coverage. Do not use heading projection to discard future lanes.

## 8. Perception, map, and detours

Retain HSV segmentation as a baseline. Use all relevant red components and scale-aware filtering instead of a fixed 5000-pixel threshold. Partial contours at image edges are partial observations, not full zone outlines. Document and test the minimum feature size the detector can resolve.

For each valid frame, use intrinsics, distortion correction, camera mounting, and interpolated vehicle roll/pitch/yaw and position to project red regions and the observed footprint onto the ground plane. Reject invalid projections or unusable images. An invalid image is not free-ground evidence.

Use a conservative grid initially around 0.2 m per cell; a 40 x 30 m field is approximately 30,000 cells. Maintain unknown, observed permissible, suspected/confirmed restricted, clearance, and observed-coverage layers. Account for grid discretization in margins.

New red evidence may block movement provisionally immediately; persistent mapping requires independent valid observations. Reprocessing one frame is not multiple confirmations. Retain confirmed static exclusions when out of view. Contradictory observations require reobservation or a controlled hold, not automatic deletion.

Choose grid A* for connectors, without diagonal corner cutting. Check any simplified path along its full length. Exact polygon visibility graphs are viable but add complexity to maintaining partially observed shapes. Image-column wall following and blind rightward shifts are rejected.

Do not plan executable connectors through unknown ground. When a zone's far side is unknown, use reachable observation frontiers and short checked advances to reveal more ground. Report blocked/incomplete if no permissible observation/movement option exists.

Apply these checks to initial transit, normal sweeps, lane transitions, turns, detours, and recovery.

## 9. Red-zone incursion handling and clocks

Normal planning continues to avoid deliberate entry. Add an explicit incursion-recovery condition that takes priority over coverage when entry is detected or conservatively suspected.

- Suspend coverage progress and immediately seek a checked exit toward established permissible ground.
- Prefer a validated retreat along recent safe history when available; otherwise select a feasible exit from available geometry. A shortest geometric boundary crossing alone does not prove the route is acceptable.
- Do not blindly send a rightward command or spend three seconds hovering inside a known zone.
- Bound recovery by the remaining total residence allowance. At five seconds since entry, if still inside, autonomously raise and log a warning and start the final five-second correction countdown. Exit is due at ten seconds since entry; never grant a fresh five seconds from late detection.
- Estimate entry from timestamped pose/zone intersection, including recent history when a zone is discovered late. Record uncertainty and use the earliest supported possible entry time conservatively. Discovery time is not necessarily entry time.
- Confirm exit with fresh position and mapped geometry, conservatively including the aircraft footprint and margin. Disappearance from the image cannot reset the timer.
- Do not reset timing merely because detection flickers or the drone moves between touching red regions. Retain a union-of-red residence record and per-region diagnostics; exact official counting rules remain unconfirmed.
- If reliable localization or a trustworthy exit path is unavailable, do not fabricate an escape guarantee. Mark the segment faulted and invoke the configured flight-controller/operator contingency. Deadline expiry must remain visible in the result even if escape succeeds later.

In Gazebo, measure physical residence using simulation timestamps and independent ground truth in the evaluator. Keep wall-time watchdogs for process/sensor health. Slower real-time factor must not shorten the simulated ten-second interval, and a pause must not erase an incursion. On real hardware use an elapsed-time basis consistently associated with sensor/pose data.

Zero deliberate incursions is the baseline engineering objective, stricter than the stated allowance. Separate injected-incursion tests exercise recovery, the autonomous warning at five seconds, the remaining-time countdown, and the ten-second entry-to-exit deadline. An earlier successful exit does not require waiting for or emitting the five-second warning.

## 10. Motion and timing design

Keep ArduPilot responsible for stabilization. Use segment tracking, measured velocity/cross-track error, bounded acceleration requests, endpoint deceleration, altitude feedback, and stop-and-turn lane changes.

Start with a provisional 0.75 m/s maximum, reduced whenever the observed permissible stopping region is too short. Validate braking instead of assuming this speed is automatically safe.

Approximate delay-plus-braking distance:

    d_stop = v * latency + v^2 / (2 * braking_deceleration)

Add aircraft size, tracking/projection uncertainty, and a reserve. For illustrative latency 0.2 s and braking 0.5 m/s^2, this term is 0.71 m at 0.75 m/s but 2.55 m at 1.5 m/s. The nominal forward half-footprint can be only 1.88 m with the provisional lens. These are design illustrations, not measured dynamics.

Check the predicted stopping region against the fence, exclusions, and unknown ground. Use measured motion, including sideways drift. During turns continue safety monitoring and use actual projected geometry rather than a straight-flight footprint assumption.

### Camera/pose synchronization required now

- Preserve image acquisition and arrival timestamps and unique frame identity.
- Preserve telemetry source timestamps and a short position/attitude history.
- Establish and verify the relationship among Gazebo time, autopilot boot time, and host monotonic time.
- Interpolate pose at image acquisition time; do not pair latest image with latest pose unconditionally.
- Bound frame age, pose gaps, clock uncertainty, and worker-result age.
- Reject unmatched observations and stop advancing when valid motion authority expires.
- Reinitialize after simulation time reset or localization-origin changes.

At 1.5 m/s, 200 ms means 0.30 m of translation. At 10 m, a five-degree viewing-direction error can shift the ground intersection by approximately 0.87 m. Timing and full attitude therefore belong in the initial geometry design.

Use source/simulation time for observation association and physical progress; host monotonic time for process/link health. Existing corridor progress-clock experience should inform this design, but its relaxed corridor pose-age tolerances are not automatically acceptable for camera mapping.

Hardware triggering, cross-camera synchronization, and final Pi exposure timestamp characterization can wait. Timestamp interfaces and validity checks cannot.

## 11. Execution structure and direct fixes

Separate configuration/geometry, sensor adapters, perception/map, coverage/planning, vehicle supervision, and GUI/logging into small testable components.

Keep the control/I/O loop independent of slow image processing, planning, and rendering. Use a bounded worker, initially a separate process for heavy work, with latest-result communication. A cached motion authorization expires; a healthy publisher must not repeat stale motion indefinitely.

Reuse existing GUIDED/arm/NAV_TAKEOFF behavior for the independent launcher, adding verified outcomes and bounded waits. The coverage component itself accepts a verified airborne entry condition so later corridor integration does not repeat takeoff.

Direct corrections:

- Validate image format, stride, dimensions, and subscription success.
- Require valid telemetry before movement and request/measure stream rates.
- Check mode, arming, takeoff, altitude settling, and progress with deadlines.
- Replace the 90% climb threshold with an explicit altitude tolerance.
- Check segment completion before unnecessary yaw alignment.
- Replace state-only safety transitions with immediate appropriate motion intervention.
- Replace arbitrary pause durations with measured settling and a bounded timeout.
- Use meaningful timestamps and explicit heading behavior.
- Distinguish COMPLETE, BLOCKED, and ABORTED results.
- At success establish and verify a final hold; do not initiate return or landing.
- Log observations, matched poses, decisions, exclusions, commands, coverage, and red-zone residence evidence.

With stale images and trustworthy localization outside a known red region, hold. With a known incursion and adequate localization/map evidence, immediate checked egress takes priority. If localization is invalid, position hold or map-based escape cannot be assumed trustworthy; use the defined flight-controller contingency and report a fault.

## 12. What to keep, replace, and defer

| Part | Decision |
|---|---|
| Python / NumPy / OpenCV / pymavlink | Keep; benchmark before considering language changes |
| Existing working simulator/autopilot setup | Reuse; isolate coverage settings and preserve corridor behavior |
| Existing takeoff command sequence | Reuse with bounded outcome verification |
| Both cameras and GUI | Keep; downward drives coverage, front remains available |
| HSV segmentation | Keep as baseline, strengthen validity and multi-region handling |
| Fixed field configuration | Keep the approach; replace arbitrary values with an explicit verified test field |
| Boustrophedon lane ordering | Keep as nominal route pattern |
| 4 x 4 grid generator | Replace with footprint-based segments |
| Image-column EVADE/TRACE policy | Replace with ground geometry and checked routing |
| FIND_NEXT and waypoint skipping | Replace with explicit interval progress |
| Untimestamped telemetry/frame caches | Replace with validity/freshness contracts and pose history |
| generate_waypoints.py executable pipeline | Do not integrate; extract/adapt useful geometry |

Deliberately defer QR, payload, return, landing mission, time-limit optimization, LiDAR/airborne obstacles, organizer geofence ingestion, multi-drone planning, final lens choice, physical calibration, terrain reconstruction, SLAM, neural models, camera fusion, and hardware triggering.

Do not defer calibration interfaces, frame transforms, timing contracts, checked recovery, or coverage bookkeeping. Do not claim desktop simulation proves Pi throughput; measure actual Pi latency, cooling, power, buffering, and workload later.

## 13. Proposed Gazebo proof criteria

The following numerical values are initial test targets, not measured current performance or final flight settings.

### Geometry and coverage

- Rendering and processing use the same camera profile.
- Verify projected ground locations across position, heading, height, and allowed attitude; initially target maximum projection error at or below 0.25 m, contained within the allocated planning margin.
- Verify field-to-NED registration and known ground height.
- Cover the complete required field minus red ground, including edges/corners, at a declared evaluation resolution. Do not silently omit safety-buffer bands from required ground coverage.
- Credit only valid observed footprints; do not bridge frame gaps with invented observations.
- Regenerate and pass with a second FOV configuration without changing algorithm logic.

### Avoidance and progress

- Nominal supported scenarios have zero deliberate/actual red incursions and no geofence breach. The zero-incursion criterion is the baseline design target, not the official ten-second rule.
- Exercise no-zone, central-zone, multiple-zone, boundary-adjacent, row-end, partially visible, and newly revealed zone layouts.
- Reproduce the old row-end skipping case and verify later rows remain pending until actually handled.
- Every interrupted required route interval is completed, replaced with a justified coverage route, or explicitly unresolved.
- Supported layouts must complete; an impassable layout must report BLOCKED rather than COMPLETE.
- Use independent Gazebo truth for evaluation, not as an operational source of red-zone coordinates.

### Incursion recovery

- Inject a recoverable incursion separately from nominal avoidance tests.
- Record entry, detection, recovery command, autonomous warning/countdown, confirmed exit, and uncertainty using source timestamps.
- Demonstrate immediate recovery and exit by the ten-second total deadline in supported recovery cases, targeting earlier exit to retain margin.
- Test the five-second boundary and ten-second deadline with deterministic timing tests, plus bounded Gazebo recovery cases. If still inside at five seconds, the system must warn itself and show/log the final countdown without external input. Verify that late detection does not restart either allowance and that an early exit ends the active residence event.
- Test late detection, clipped/lost views, boundary jitter, and adjacent red regions; none may erase residence time or falsely assert exit.
- Unrecoverable/localization-invalid cases must produce an explicit fault and contingency, not a false compliance result.

### Motion, latency, and faults

Initial nominal envelope: altitude 10 m +/-0.5 m; speed cap 0.75 m/s; straight-segment cross-track error at or below 0.30 m; control supervision target 20 Hz; initial accepted image-age target 200 ms and camera/pose time-association uncertainty target 50 ms. Reconcile all limits with measured braking and uncertainty before crediting success.

Inject frame delay/dropout, telemetry loss, estimator/yaw perturbation, CPU load, and simulator pause/resume. The system must stay within its declared envelope or enter the appropriate non-success/recovery condition. Existing slow-simulation behavior must not turn a simulation-time residence or progress limit into an incorrect wall-time deadline.

### Completion and evidence

- COMPLETE requires all required route obligations resolved and no required coverage gaps remaining.
- Establish final settled hold with intended heading. No return, landing mission, QR action, or payload action follows.
- Repeat each supported scenario at least three times with varied valid start position and yaw.
- Retain timestamps, map evolution, planned/actual routes, coverage ledger, minimum clearances, residence times, and terminal reason.

## 14. Clarifications resolved and next implementation boundary

The user has resolved the timing question: five seconds of residence followed by a five-second correction countdown, with autonomous warning and no external trigger. No further user decision is needed to proceed with the proposed design. Use the documented conservative footprint and continuous-residence conventions for implementation/testing; do not present those engineering conventions as additional official rules.

The arbitrary old coordinates will not be reused as authoritative bounds. Establishing the isolated test field, selecting its known permissible start, and verifying the frame transform are engineering tasks for the implementation pass, not additional choices required from the user now.

Next implementation, when explicitly requested: establish the isolated two-camera fixture using the existing flight stack; implement/test geometry and timing contracts; replace coverage progress and routing; add perception mapping, residence monitoring and recovery; then execute the bounded Gazebo test matrix. Preserve existing takeoff/corridor behavior and unrelated files.

Session stopping point: design document updated with the final timing clarification. The user is done for today. Do not begin implementation until a subsequent request authorizes it.

