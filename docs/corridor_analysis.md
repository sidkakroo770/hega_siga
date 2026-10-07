# Pre-coverage mission: real-world readiness review

Date: 2026-10-03. Review target: Raspberry Pi 5 (8 GB), two simultaneous Raspberry Pi Global Shutter Cameras, one real 2D LiDAR, and a real flight controller/aircraft.

## Executive assessment

The corridor's basic perception and control approach is worth retaining. It uses small 2D geometric problems, bounded velocity commands, wall-fit validation, hysteresis, and explicit recovery states. Nothing in that approach inherently requires desktop-class compute, an AI model, ROS 2, or a wholesale replacement with SLAM.

However, **the successful Gazebo mission is not yet a hardware-ready flight application**. The highest risks are not the nominal wall-following mathematics. They are inconsistent treatment of missing information, gaps in state supervision, sensor-age accounting, and the difference between the tested simulation wrapper and the separate hardware runner.

I would permit supervised, motors-disabled Pi integration and sensor recording, but would not approve autonomous corridor flight with the current application unchanged. Several failure paths below are reproducible software defects, not speculative simulation-to-reality concerns.

The source was not modified. No aircraft connection, arm command, flight command, or new Gazebo flight was initiated during this review. The only project deliverable created is this report. Temporary PDF rendering and in-memory offline diagnostics were used for inspection.

## 1. Scope and evidence

### What was reviewed

The active full mission is `world/integration/experimental_corridor_manager.py`, not the old standalone approach script or the ROS corridor copies. I traced its imported native controllers, sensor callbacks, telemetry handling, output commands, altitude helper, and transitions through `ASCEND_FOR_COVERAGE`. Coverage planning, boustrophedon motion, and red-zone avoidance themselves are excluded.

I also inspected the separate native hardware runner and its LiDAR/MAVLink dependencies because those are the repository's existing hardware-facing implementation. Their safeguards must not be attributed to the full Gazebo manager unless actually connected to it.

The mission-context PDF was read, including the page-3 mission-profile diagram. It establishes an entrance/banner, a nominal 3.5 m corridor, static obstacles, and the transition to the outdoor field. Its text describes an initial 5 m stage and approximately 1 m forward movement; the diagram labels a 10-foot corridor height. The current full-world test instead starts from a manually established airborne condition. I treat that as a documented test boundary, not evidence that autonomous takeoff is integrated into this runner. I do not review QR functionality or count its absence as a defect.

Banner colour and dimensions are accepted as specified. Findings about exposure, clipping, camera mounting, and similar-coloured backgrounds remain relevant even with the correct physical banner.

### Reference key

All source references below are to the inspected files and line numbers; line ranges describe sections, not separate versions.

| Key | File / role |
| --- | --- |
| M | [world/integration/experimental_corridor_manager.py](/home/sid/[competition]_mission2/world/integration/experimental_corridor_manager.py) — active full-mission wrapper |
| A | [world/integration/corridor_altitude.py](/home/sid/[competition]_mission2/world/integration/corridor_altitude.py) — descent/climb controller |
| H | [world/integration/corridor_handoff.py](/home/sid/[competition]_mission2/world/integration/corridor_handoff.py) — front-range approach trigger |
| B | [approach/autonomy/perception/hybrid_banner_detector.py](/home/sid/[competition]_mission2/approach/autonomy/perception/hybrid_banner_detector.py) — active green-banner detector |
| N | [corridor/native/mission_runner.py](/home/sid/[competition]_mission2/corridor/native/mission_runner.py) — native state supervisor |
| P | [corridor/native/controllers/pre_entry.py](/home/sid/[competition]_mission2/corridor/native/controllers/pre_entry.py) — wall geometry and entry alignment |
| C | [corridor/native/controllers/corridor_cruise.py](/home/sid/[competition]_mission2/corridor/native/controllers/corridor_cruise.py) — cruise, correction, exit candidates |
| O | [corridor/native/controllers/obstacle_avoidance.py](/home/sid/[competition]_mission2/corridor/native/controllers/obstacle_avoidance.py) — obstacle classification, SHIFT/PASS |
| R | [corridor/native/controllers/hover_and_reassess.py](/home/sid/[competition]_mission2/corridor/native/controllers/hover_and_reassess.py) — recovery |
| E | [corridor/native/controllers/exit_detection.py](/home/sid/[competition]_mission2/corridor/native/controllers/exit_detection.py) — measured exit commit |
| T | [corridor/native/run_corridor_real.py](/home/sid/[competition]_mission2/corridor/native/run_corridor_real.py) — separate hardware runner |
| D | [corridor/native/hardware/d500_driver.py](/home/sid/[competition]_mission2/corridor/native/hardware/d500_driver.py) — serial LiDAR parser |
| I | [corridor/native/hardware/mavlink_io.py](/home/sid/[competition]_mission2/corridor/native/hardware/mavlink_io.py) — hardware telemetry reader |
| S | [corridor/native/hardware/mavlink_sender.py](/home/sid/[competition]_mission2/corridor/native/hardware/mavlink_sender.py) — gated hardware sender |
| SA | [corridor/native/common/scan_adapter.py](/home/sid/[competition]_mission2/corridor/native/common/scan_adapter.py) — raw scan to FLU conversion |
| VM | [world/models/models/iris_miss2_full/model.sdf](/home/sid/[competition]_mission2/world/models/models/iris_miss2_full/model.sdf) — tested simulated sensors |
| FW | [world/worlds/miss2_full_world.sdf](/home/sid/[competition]_mission2/world/worlds/miss2_full_world.sdf) — full-world placement/banner |
| Run notes | [world/integration/FULL_MISSION_COVERAGE.md](/home/sid/[competition]_mission2/world/integration/FULL_MISSION_COVERAGE.md) |
| Requirements | [autonomous mission_[competition]_[competition].pdf](</home/sid/[competition]_mission2/autonomous mission_[competition]_[competition].pdf>) |

### Interpretation of labels

- **Confirmed problem:** directly established by code or reproduced offline. This does not mean a real collision has been observed.
- **Likely real-world risk:** a concrete implementation assumption is exposed by plausible hardware behaviour; its actual frequency/magnitude is not measured.
- **Hardware validation needed:** no demonstrated defect, but a parameter or capability cannot be approved from this simulation alone.
- Severity describes consequence if encountered. **Critical** can invalidate motion safety or emergency handling; **High** can cause unsafe transitions, major mission failure, or prevent deployment; **Medium** primarily affects robustness, performance, or bounded completion.

## 2. Actual mission sequence and boundaries

```text
External airborne setup / takeoff
  -> BANNER_SEARCH
  -> CAMERA_CORRIDOR_CENTER
  -> APPROACH_CORRIDOR
  -> DESCEND_BEFORE_PRE_ENTRY
  -> HOVER_BEFORE_PRE_ENTRY
  -> LIDAR_CORRIDOR:
       PRE_ENTRY_GEOMETRY_LOCK
         ACQUIRE -> ALIGN_YAW / CENTER_LATERALLY -> VERIFY_LOCK
       ENTER_CORRIDOR
       CORRIDOR_CRUISE
         -> OBSTACLE_DECISION -> AVOID_LEFT / AVOID_RIGHT:
              SHIFT -> PASS -> CRUISE
         -> EXIT_DETECTION -> CORRIDOR_EXITED
       HOVER_AND_REASSESS -> revalidated state or ABORT_CORRIDOR
  -> ADVANCE_TO_FIELD
  -> ASCEND_FOR_COVERAGE
  -> COVERAGE [review stops here]
```

| Stage | What actually determines progress |
| --- | --- |
| Startup | M waits for MAVLink heartbeat; documented launch has the operator select GUIDED, arm, and take off to approximately 3 m first. M does not perform those actions. |
| Search | Body-left velocity, nominally 0.50 m/s, until a qualifying green contour appears. |
| Camera centering | Lateral and vertical velocity proportional to bounding-box pixel error; 30 new frames inside a 20 px tolerance. |
| Approach | Forward 0.20 m/s plus image-based lateral/vertical corrections. A front return at or below 0.50 m **or** five new frames without detection ends camera authority. |
| Descent / hover | Descend 1 m from the measured handoff height; settle, then wait for approximately two FC-clock seconds of low XYZ velocity and acceptable altitude. |
| Pre-entry | Fit two walls, validate width/parallelism/fit quality/sectors, align yaw and centre laterally, confirm lock. |
| Entry | Travel 0.75 m projected onto the initial heading, using local-position feedback. |
| Cruise | Wall-relative yaw/centering corrections, adaptive forward speed up to 0.35 m/s; obstacle and exit classification. |
| Avoidance | Recognise a transverse face attached to one wall, select the opposite passage, shift, pass at 0.18 m/s, confirm a matching face behind. |
| Exit | Commit another 1.20 m using measured projected displacement, normally at 0.15 m/s. |
| Pre-coverage boundary | Check the fixed registered field, advance body-forward until a north-coordinate threshold, then climb to nominal 10 m HOME-relative altitude via a local-Z offset. |

The downward image stream is not processed by M's pre-coverage states. The simulated downward sensor is enabled, but that is not evidence that the Pi has already sustained two real camera pipelines alongside this mission. The front callback remains subscribed after camera control ends.

## 3. Findings

### R01 — The tested full mission and the hardware entry point are not an executable, equivalent deployment path

**High — Confirmed problem.** Locations: M:8–95, 1028–1098; T:15–23, 320–400; S:15–18; N:69–92, 553–569.

M imports Gazebo transport/messages directly and has no real-camera or real-LiDAR source selection. T is corridor-only, not banner-to-corridor orchestration. Importing T fails in this checkout with `ModuleNotFoundError: No module named 'native.hardware.mavlink_commands'`: S imports that module, but it is absent. This was reproduced without opening any device.

Even after that import issue, T constructs `NativeMissionRunner()` without configuring `enter_corridor_distance_m`. The default is deliberately `None`, so `ENTER_CORRIDOR` stops with `WAITING_FOR_ENTER_DISTANCE_CONFIG`; T supplies no CLI option to resolve it. This is a safe placeholder, but a real deployment blocker, not completed hardware support.

**Why Gazebo passed:** M bypasses S and supplies its own entry distance and command functions. Neither hardware failure is exercised.

**Required before deployment:** establish one explicit hardware launch path and an airborne-entry/takeoff handoff contract, with import/startup checks in dry-run mode. Preserve the native controllers; do not assume transplanting the Gazebo script or invoking T already gives equivalent behaviour. The old `approach/autonomy/behaviors/mission_runner.py:55–87` also waits for operator takeoff; it is not the active full-mission orchestrator. The coverage takeoff utility explicitly identifies itself as a standalone SITL test.

### R02 — Flight-controller health and control ownership are not supervised throughout the active full mission

**Critical — Confirmed missing supervision; aircraft response depends on FC configuration.** Locations: M:334–395, 481–683, 1028–1045, 1194–1691; S:152–196; I:325–490, 532–574.

After startup, M caches position, attitude, and relative altitude, but does not continuously gate commands on heartbeat health, GUIDED mode, armed/airborne state, EKF validity, or pilot takeover. Its direct send functions bypass the hardware sender's gates. A received `LOCAL_POSITION_NED` value is not proof of a healthy position solution. Search/centering can operate with no usable localization. Commands normally rejected by an FC in another mode can become actionable again if GUIDED resumes while the old mission state continues.

The separate S has meaningful heartbeat/GUIDED/XY gates. However, I retains EKF flags without an EKF-message age; fresh position plus an old good flag can pass `horizontal_position_ok()`. Neither reader explicitly filters every incoming message by the intended aircraft/component. This matters on routed multi-source links, not necessarily a dedicated point-to-point wire.

**Why Gazebo passed:** the operator establishes the right mode, SITL supplies a stable estimator, and the expected vehicle is the dominant telemetry source.

**Required:** consistent command-authority and freshness/health checks across all motion states, and a verified RC/FC failsafe contract. Battery and link failsafes may properly reside in the FC; their actual configuration was not supplied, so I do not claim the aircraft has none.

### R03 — Camera loss has no explicit age watchdog; old motion can persist until the FC intervenes

**Critical — Confirmed problem.** Locations: M:161–202, 1194–1366, 1374–1632, 1636–1684.

The callback stores an image and sequence number, but no acquisition or reception timestamp. A decode failure leaves the last image in place. Camera states act only when the sequence changes. Once a valid image has existed, a stalled stream is not the `frame is None` case: those states can simply stop producing new commands without explicitly stopping or aborting for camera age.

This does **not** prove indefinite motion. ArduPilot documents a configurable `GUID_TIMEOUT`, default three seconds, after which velocity-controlled motion slows to a stop. At 0.50 m/s, three seconds corresponds to 1.5 m of nominal commanded travel before accounting for deceleration; the exact real response depends on firmware and settings. That is not an acceptable unexamined corridor safety margin. [ArduPilot Guided Mode](https://ardupilot.org/copter/docs/ac2_guidedmode.html)

**Why Gazebo passed:** regular rendered frames do not exercise disconnects, camera-service stalls, or long capture queues.

**Required:** distinguish no detection in a fresh image from no fresh image, and bound command age independently of perception progress. Sequence checks are good for avoiding duplicate evidence, but are not a sensor watchdog.

### R04 — Missing LiDAR bypasses the very recovery logic intended to handle missing LiDAR

**Critical — Confirmed problem.** Locations: M:1811–1817, 2212–2227; T:552–596; R:334–364; N:982–999.

In M's `LIDAR_CORRIDOR`, an absent or older-than-0.30 s scan causes `send_stop()` followed by `continue`. This skips `corridor.step()`, the GUI event handler, and the normal sleep. The result can be a CPU/MAVLink busy loop, with recovery deadlines never evaluated. A prolonged sensor failure does not autonomously complete the intended recovery/abort sequence.

T avoids that busy loop by blocking in `get_scan(timeout_s=0.25)`, and starts reassessment after a dropout. But it also continues without stepping the runner when no scan arrives. Once in reassessment, the no-scan recovery timeout still cannot run. Its command watchdog helps stop motion; it does not provide terminal failure handling.

**Why Gazebo passed:** the successful path keeps scans arriving. A stale check appearing in the controller is not enough when its caller suppresses the update.

**Required:** safety deadlines must advance without new sensor input; both wrappers need bounded dropout behaviour. This is a software correction to establish before powered hardware testing, not a threshold to tune in flight.

### R05 — Abort intent is not reliably delivered or confirmed

**Critical — Confirmed problem.** Locations: N:982–999 versus 1166–1188; M:654–683, 1888–1902, 2006–2016, 2182–2256; T:694–714, 745–807; I:193–219.

N normally translates a same-step abort into a LAND action. The reassessment hard-timeout branch returns early with `status='ABORT_CORRIDOR'` but **no action**. An offline check reproduced `ABORT_CORRIDOR, action=None`. M observes the terminal state and exits; it does not step again to retrieve the LAND action. T similarly assumes LAND was supplied and terminates after a short delay.

M's own descent, hover, registration, and climb aborts, GUI escape, and exception cleanup send a brief sequence of zero-velocity commands and close the link. They do not establish a confirmed sustained hold or landing. Normal native LAND dispatch is a one-shot request without ACK/mode/landed confirmation.

In T, exceptions unwind the `with MavlinkIO(...)` context before the outer exception handler tries `manager.land()`, so that fallback attempts to use an already-closed connection. Its normal output thread retries actions, which is useful, but is not a confirmed terminal-state protocol.

**Why Gazebo passed:** nominal missions do not test rejected/lost LAND messages or exception-order cleanup.

**Required:** make every terminal path produce a consistent, observable vehicle-level outcome and retain pilot authority. Whether landing inside a particular corridor is appropriate must be decided from the physical site; I am not asserting that LAND is universally safest.

### R06 — Banner search has no spatial or time bound and no local collision guard

**High — Confirmed problem.** Locations: M:1194–1366; defaults around 845–895.

For every fresh frame without a detection, M commands body-left motion, nominally 0.50 m/s. There is no search distance limit, search deadline, yaw-start validation, altitude envelope, or LiDAR lateral-clearance check in that branch. Camera centering can also persist without a mission-stage deadline. Losing the target during centering restarts search.

**Why Gazebo passed:** the known spawn, heading, and banner position put the banner along the programmed search direction. A small physical mounting/yaw error, temporary exposure failure, or a banner initially off the other side can make the drone continue across the launch area.

**Required:** bound search and alignment within the known operating area and define the no-acquisition outcome. This does not require a speculative general-purpose search planner or changing the assumed banner appearance.

### R07 — Detection loss and an unassociated close range both mean “safe to descend”

**High — Confirmed transition weakness; likely real-world false triggers.** Locations: M:1636–1683; H:5–15.

Five new images with no accepted panel trigger irreversible descent regardless of front range. At the modeled 30 FPS, this is roughly a fraction of a second, not evidence that the entrance was reached. A correct green banner can disappear through occlusion, motion blur, exposure changes, or clipping.

The alternative trigger is the median of the three smallest valid returns in a ±10° cone. It rejects one isolated small return but does not check adjacency despite its comment, and does not associate those returns with the banner. Two short outliers or airframe returns can produce a close trigger. Conversely, a fresh scan with no valid front range can still permit forward motion while the camera detects the panel.

**Why Gazebo passed:** the banner and LiDAR target surfaces have fixed placement and appearance; the history explicitly records LiDAR mounting changes to eliminate self-returns.

**Required:** entrance arrival must have explicit positive evidence; a perception failure must remain distinguishable from arrival. The current range and loss thresholds need validation against the mounted sensors, not just repetition of the clean simulation.

### R08 — Pixel centering is a valid lightweight technique, but its current geometry is under-specified

**High — Likely real-world risk.** Locations: B:52–154; M:1374–1632, 1667–1673; VM:944–979.

M turns bounding-box errors directly into lateral/vertical speed using 0.003 m/s per pixel, capped at 0.50 m/s, with 20 px acceptance. Its simulated front camera is 640×480 with approximately 60° horizontal FOV. The code does not compensate for front-camera boresight error, roll/pitch, distortion, or changing focal length/crop. Vertical image motion caused by pitch is therefore treated as height error. The real front-camera lens is not specified; the earlier downward-camera 10 mm assumption does not determine front-camera geometry.

Relaxed approach accepts clipped panels and still uses the visible bounding-box centre. That can cease to represent the panel centre; a panel filling the image can appear centred over a range of actual offsets. Thirty centred frames do not independently prove low aircraft velocity.

**Why Gazebo passed:** fixed optics, aligned mounting, and predictable attitude response.

**Required:** establish the front-camera processing mode and alignment/error budget, including clipped-target behaviour. Full 3D reconstruction is unnecessary. Basic image-based servoing can remain; calibration and measured closed-loop response determine whether attitude compensation beyond gating is necessary.

### R09 — Correct physical green does not guarantee correct thresholded pixels

**Medium — Likely real-world risk; final tuning needs hardware.** Locations: B:7–11, 65–124; M:1558–1563, 1662.

The active detector uses HSV bounds `[45,100,80]`–`[75,255,255]`, a 700 px area threshold, a 5×5 opening, and largest qualifying contour selection. A correctly manufactured banner can still lose saturation/value under glare, shade, white-balance changes, underexposure, or gain noise. Focus, vibration, and finite exposure affect its edges and area. Global shutter removes rolling-shutter skew, but does not make finite-exposure motion blur disappear. Raspberry Pi documents the camera's global exposure and short-exposure capability; exposure still has to be selected for available light. [Raspberry Pi camera documentation](https://www.raspberrypi.com/documentation/accessories/camera.html#global-shutter-camera)

There is no active target identity lock: M explicitly leaves `panel_only=False`; the presence of `lock_target()` is not evidence of tracking. A larger similarly green background can win even though the intended banner is exactly correct. This is a conditional background risk, not a demand to handle unknown banner colours.

The input contract also needs attention: M:170–185 assumes tightly packed, three-channel RGB, reshapes without checking pixel format or row stride, and converts RGB to BGR. That matches the simulated `R8G8B8` stream, but is not a generic Pi-camera buffer contract. A different channel order can silently break colour detection; padding or a different format can cause decode failures and leave the previous frame cached. B:73–75 also assumes the debug frame is at least 160×120 for its inset. These are straightforward adapter/processing-mode constraints, not evidence that the camera itself is unsuitable.

**Why Gazebo passed:** fixed rendering/materials; the documented world was also recoloured to remove a competing green corridor material.

**Required:** record this exact banner through the real front camera under expected lighting and motion. Retain HSV/contours unless evidence shows they are inadequate; there is no justification for adding an AI model now.

### R10 — Relative descent is controlled, but the flight-height envelope is not established

**High — Confirmed missing envelope; actual clearance needs hardware/site validation.** Locations: M:1694–1781, 2125–2139; A:constructor and `update`; native motion outputs.

The descent target is measured handoff height minus 1 m, not a verified floor/roof clearance. The handoff height itself was adjusted by camera centering. A positive altitude relative to EKF origin is not proof of clearance above the physical floor. During native traversal, the mission sends zero vertical velocity but no longer supervises actual height against a corridor envelope. That relies on the FC's vertical controller and height estimate; zero vertical velocity is not “no altitude control,” but it is also not independent roof/floor protection.

The altitude helper correctly rejects invalid/stale inputs, limits speed, requires low vertical velocity and new samples for settling, and has timeouts. Those protections should be retained.

**Why Gazebo passed:** fixed banner/roof/floor heights and stable vertical estimation. The diagram's 10-foot corridor height must not automatically be interpreted as a safe vehicle-centre altitude under a roof.

**Required:** relate HOME/EKF height to the known flat floor, measured airframe envelope, banner placement, and roof. Flat ground and a small FC/camera offset can be entirely acceptable; neither terrain mapping nor a new range sensor is automatically required. They do not eliminate barometric error, pitching airframe clearance, or an incorrect initial datum.

### R11 — ENTER_CORRIDOR ignores new obstacle and alignment information while moving

**Critical — Confirmed problem.** Locations: N:545–670, 1065–1077; M:776–797.

`step_enter_corridor()` receives only pose. It commands 0.20 m/s until projected displacement reaches 0.75 m, with no front-clearance, side-clearance, tilt, or ongoing wall-alignment check. M requires a fresh scan before calling the runner, but freshness alone is not geometric safety.

An offline call with a fresh scan containing 0.20 m returns in every direction still produced `vx=0.20 m/s` in ENTER. The clean pre-entry lock proves conditions at an earlier instant, not that they remain safe throughout the roughly 3.75 s nominal entry. M permits up to 60 wall seconds for this stage.

**Why Gazebo passed:** static entrance, repeatable alignment, no wind-driven cross-track excursion, and no changed obstruction during entry.

**Required:** preserve continuous safety supervision during the commit. The measured-displacement completion criterion is good and should stay; the defect is the absence of live constraints while executing it.

### R12 — PRE_ENTRY verification has an indefinite dead band

**High — Confirmed problem, reproduced offline.** Locations: P:958–1032; timeout checks at 823–828 and 895–901; N:1029–1059.

VERIFY_LOCK accepts lateral error up to 0.12 m, but only resumes lateral centering above 0.18 m. With valid, high-confidence geometry and a persistent 0.15 m error, it sends STOP, resets the successful-scan streak, and stays in VERIFY_LOCK. A similar gap exists between accepted yaw and yaw-realignment thresholds, and between control and verification confidence thresholds.

The alignment timeout is evaluated in ALIGN_YAW/CENTER_LATERALLY, not VERIFY_LOCK. The supervisor's HOLD timeout does not cover VERIFY_LOCK. Setting the alignment/state clocks 600 seconds old and supplying 100 such geometry updates still left the controller in VERIFY_LOCK with no failure.

**Why Gazebo passed:** clean geometry and repeatable dynamics carry the aircraft through the verification band. Real steady bias or small wind-induced offset can leave it stationary in the gap.

**Required:** every verification outcome must either converge, recover, or time out. Hysteresis itself is useful; an unbounded state between its thresholds is not.

### R13 — Missing side returns can be classified as the corridor exit

**Critical — Confirmed problem, reproduced offline.** Locations: C:370–441, 520–547, 659–686; E:422–565.

`classify_side_opening()` returns `True` when there are no valid finite returns in a side sector. This conflates an observed opening with missing/invalid/occluded sector data. If front clearance is large and both side sectors are missing for three scans after the 1.5 s guard, cruise requests EXIT_DETECTION before the ordinary geometry-confidence rejection path.

An offline scan with finite 5 m forward returns and NaNs elsewhere reached EXIT_DETECTION after three updates. Another exit route accepts repeated weakened strict geometry when loose geometry remains and the front is open; that is evidence of a possible end, not proof of an end.

E then commits 1.20 m and does not continue verifying that side-wall disappearance represents a genuine exit. A later fixed-field registration check can catch gross location errors, but cannot undo the intervening motion or guarantee a correct exit near the field boundary.

**Why Gazebo passed:** rendered no-return areas reliably correspond to the designed opening. Real reflectivity, occlusion, partial scans, and receiver failures need not have that meaning.

**Required:** keep “unknown sector” distinct from “observed free space,” and positively validate the exit interpretation. This is not solved merely by increasing the confirmation count.

### R14 — Front-range safety is inconsistent across states and can miss narrow/invalid evidence

**Critical — Confirmed invalid-data path; narrow-target detection is a hardware-validation risk.** Locations: P:341–376; O:764–802, 1836–1904; E:282–394, 620–632; H:5–15.

During avoidance PASS, no usable front returns produce `front_clearance_m=0`. The stop condition tests `0 < clearance <= 0.60`, so zero bypasses it. With a valid open-side wall, the controller continues forward. An offline observation reproduced `vx=0.18 m/s` with zero front clearance. E likewise continues its pose-based exit when front clearance is unavailable; `fresh_front_safety_available()` is not used as a movement gate. M blocks wholly stale scans, but not fresh packets whose front sector is unusable.

Several controllers convert positive infinity to maximum range and use the 10th percentile over a front cone. That is a reasonable noise filter for broad surfaces, but can suppress a narrow close object occupying less than about 10% of the cone. Zero, NaN, infinity, clipped range, and genuine no-return are not consistently distinguished. The D500 decoder emits raw numeric distances, not necessarily Gazebo's infinity convention.

**Why Gazebo passed:** complete, clean scans and broad planar obstacles.

**Required:** define a consistent validity/clearance contract and test the actual minimum obstacle width and angular coverage. Do not replace all percentiles with raw minima blindly; that trades missed obstacles for outlier-driven false stops.

### R15 — Obstacle avoidance is specialised for wall-attached, transverse faces

**High — Confirmed operating-envelope limitation; event applicability needs validation.** Locations: O:71–106, 856–928, 1112–1326, 1610–1646; R:1116–1336.

The recogniser fits longitudinal walls and a roughly transverse obstacle face, then accepts a face touching exactly one wall. Minimum face span, inliers, RMS, axis tolerance, and wall-touch distances are prescribed. A missing wall can be reconstructed using nominal 3.5 m width, but the open-side wall must actually be observed before selecting that passage—an important safeguard.

A free-standing post, sloping or curved face, sparse mesh, or two-sided blockage is not a supported normal bypass. The usual result should be ambiguity/stop/reassessment, not an invented free path. Pass completion identifies a rear transverse face on the same side; it does not establish full obstacle identity or explicitly model longitudinal aircraft extent. If the recognisable rear face is hidden, avoidance can time out; if an unrelated same-side face qualifies, it can finish early.

**Why Gazebo passed:** compatible wall-attached geometry with clean planar returns and predictable visibility. The mission drawing alone does not establish that every physical obstacle has these properties.

**Required:** confirm that physical obstacles fit this supported class and test their exact surfaces/visibility. Generalising to arbitrary obstacles should wait unless the event actually requires it. The controller's conservative refusal of unsupported shapes is preferable to guessing.

### R16 — Vehicle clearance numbers are not yet a verified aircraft safety envelope

**High — Hardware validation needed, with confirmed hard-coded assumptions.** Locations: O:61–68, 140–147, 1213–1256, 1714–1726; C:54–99; P:103–127.

Avoidance assumes vehicle width 0.65 m plus 0.25 m on each side: a minimum nominal passage of 1.15 m. The hard outer-wall limit is 0.45 m measured from the LiDAR origin. If that origin were at the centre of a 0.65 m aircraft, this would leave only 0.125 m to the nominal lateral edge at the stop threshold. That is an illustrative calculation, not a measurement of your aircraft. Rear-pass margin is 0.35 m without a supplied longitudinal propeller/airframe envelope.

Measured range error, LiDAR mounting offset, roll, yaw, propeller sweep, latency, tracking error, and stopping distance consume clearance. Commands are speed-limited, but no explicit mission-level acceleration/jerk bound is applied; actual acceleration/deceleration comes from the FC tuning and vehicle.

**Why Gazebo passed:** the Iris dynamics and geometry are repeatable; real inertia, wind response, and braking are unmeasured.

**Required:** measure the real envelope and tracking/braking errors before flight in narrow passages. There is no basis to label all present speeds too fast, or to assert a particular new clearance value from source alone.

### R17 — Missing attitude is accepted, and the 2D scan-plane assumption needs physical validation

**High — Confirmed supervision gap plus likely geometric risks.** Locations: P:108–109, 678–693, 1198–1202; C:115, 459; O:152–153, 357–374; M:444–474; T:602; SA:conversion; VM:888–938.

`require_imu=False` is the default. M converts attitude older than 0.50 s to `None`, which these controllers accept as safe. T passes the cached attitude object directly; the controller's tilt test does not inspect its timestamp. Consequently, absent/stale attitude can bypass the intended tilt limit. NaN roll/pitch normally fails the comparisons, which is better than treating it as level, but missing attitude remains permissive.

Scan points are treated as planar body FLU coordinates. The adapter offers yaw/sign correction but no translational mounting correction, per-beam attitude transformation, or scan deskew. Tilt limits reduce risk but do not prove that the scanner continues intersecting the walls/obstacles at the relevant height. A 2D sensor cannot observe an overhang, low object, or propeller-level hazard that misses its plane.

**Why Gazebo passed:** the simulated scanner is level, mounted 0.25 m below the model reference, and deliberately below the landing legs. Real mounts, vibration, self-occlusion, and wall/obstacle heights can differ.

**Required:** verify left/right/yaw signs physically, require usable attitude for the intended tilt guard, and establish the scan-plane operating envelope. Full scan deskew can wait for measured need at these low speeds; invalid/stale attitude handling should not.

### R18 — The hardware LiDAR pipeline can label old buffered measurements as fresh

**High — Confirmed timestamp semantics; overload occurrence is a likely risk.** Locations: D:231–260, 280–288, 350–378, 481–495; T:552–600; SA:conversion.

D reads serial data synchronously in the same loop that runs corridor perception. It decodes the sensor timestamp and rotation speed but does not use them. The assembled scan gets a new `time.monotonic()` stamp at build time; `scan_time_s` measures intervals between builds, not necessarily sensor revolutions. If perception/printing stalls and serial data backs up, an older physical scan can be decoded later and appear fresh.

The queue has `maxlen=3`, which is good, but does not bound the age of upstream serial/USB data. Sorting the scan also does not make its points simultaneous. SA drops the raw scan-duration field when producing the controller input. M has the analogous, though different, limitation of timestamping Gazebo scans at receipt rather than acquisition.

**Why Gazebo passed:** it bypasses this serial parser, and the local transport does not replicate USB buffering under Pi load.

**Required:** preserve acquisition-age information or a defensible bound and observe backlog. Separate prompt acquisition from expensive processing where necessary. Deskew is secondary to preventing stale scans from being marked current.

### R19 — CRC checking is good, but incomplete revolutions are not adequately identified

**High — Confirmed parser limitation; manifestation depends on sensor/link.** Locations: D:21–30, 292–339, 420–458; native range filters.

D correctly verifies CRC, resynchronises packet headers, discards the initial partial revolution, and converts millimetres to metres. However, a “complete” scan requires an observed wrap from above 340° to below 20° and at least 100 accumulated points. It does not verify angular coverage, missing sectors, point-age spread, plausible rotation speed, or packet continuity.

A dropout across the wrap can merge revolutions until a later recognised wrap. A scan with a large missing sector can still exceed 100 points. There is no explicit count/time cap on `current_scan_points` if malformed-but-accepted angle sequences never wrap. This is a conditional accumulation risk, not evidence of a normal memory leak. Intensity is preserved but not used to express return validity/confidence.

**Why Gazebo passed:** a 500-ray message arrives as one coherent scan; serial packet loss and parser assembly are absent.

**Required:** establish the exact real LiDAR model/protocol and invalid-return conventions, and test truncated/corrupt/missing-sector streams. The code is D500-specific; compatibility with an unspecified different 2D LiDAR is not established. Sunlight/reflectivity susceptibility must be measured for that device, not assumed from the simulator or an unidentified sensor datasheet.

### R20 — Acquisition time, receipt time, and evidence duration are mixed

**High — Confirmed timing limitations; impact depends on measured latency.** Locations: M:161–279, 334–474, 776–797; P:89–106, 130–136; C/O/R confirmation counts; I:325–393.

M timestamps MAVLink data while draining the queue and combines the latest position, attitude, and scan rather than observations at a common acquisition time. Delayed packets can therefore look newly received. The drain loop itself has no work budget. I also stamps telemetry at receipt and does not preserve FC sample time in its pose/attitude objects. Front images have no timestamp at all.

Many decisions use scan/frame counts rather than elapsed observation duration: six entry-verification scans, four bypass confirmations, three exit confirmations, five loss frames, and three-/five-sample filters. At the simulated 10 Hz LiDAR and 30 FPS camera these have one effective duration; drops, burst delivery, or different real rates change it. Unique-sample processing in M is a real strength, but does not detect repeated acquisition data delivered under new callbacks.

The simulation overrides explicitly allow 1 s pose age and 6 s pose-loss windows, plus extended stage deadlines. Do not transfer them to the aircraft as validated safety budgets.

**Why Gazebo passed:** fixed nominal rates, a common simulator, and known timing accommodations.

**Required now:** measurable sample age, sequence continuity, bounded processing/command latency, and rate-aware confirmation semantics. Hardware measurements should determine tolerances. Exact front/downward hardware shutter synchronisation is not required for these independent pre-coverage behaviours; there is no stereo triangulation here. Timestamped acquisition matters much more than forcing both cameras to expose together.

### R21 — Measured entry/exit travel is better than a timer, but pose validity is too weak

**High — Confirmed nonfinite-input gap; drift/reset effects are likely risks.** Locations: M:401–437; N:530–670; E:197–276, 477–563; I:532–561.

The projection `cos(start_yaw) * delta_north + sin(start_yaw) * delta_east` is correct for the stated NED yaw convention. It avoids the old assumption that speed multiplied by time equals displacement. Nevertheless, pose freshness does not check coordinate finiteness, estimator reset/jump, covariance/quality, or plausible displacement. A fresh pose with NaN X was accepted by E and still produced forward motion in the offline check. With NaN yaw, trigonometric processing can also propagate invalid values or raise for other nonfinite inputs.

A position jump can satisfy the 0.75 m entry or 1.20 m exit criterion early. Cross-track movement is not credited as forward progress, which is good, but these commit states do not bound cross-track drift or heading change while they execute. Velocity is body-forward now, whereas completion is projected along the starting heading.

**Why Gazebo passed:** smooth, consistent local position and heading. LiDAR wall-relative steering reduces dependence on global positioning during cruise; it does not remove the FC's need for a functioning estimator or the commits' dependence on local displacement.

**Required:** validate finite, continuous, healthy pose and the available positioning performance under the actual corridor. RTK/SLAM is not automatically required; the supplied sensor list does not establish what localization source the FC will have or how well it works near/under the corridor structure.

### R22 — The exit-to-field advance mixes body motion with a fixed north-coordinate condition

**High — Confirmed world-alignment assumption; unsafe outcome depends on registration error.** Locations: M:312–321, 2038–2057, 2080–2139.

The fixed field registration check is useful for catching gross origin errors. But `ADVANCE_TO_FIELD` decides progress from local N/X, then calls the body-frame sender with forward velocity. Its comment says “continue north”; that is only true at the expected heading. This branch does not continually check yaw, east-coordinate containment, side clearance, or total horizontal speed before climbing. It checks only north velocity for settling. The subsequent vertical climb does not independently prove roof clearance or maintain a checked field XY envelope.

The climb's HOME/local-Z offset conversion is a good correction; it should not be discarded. Its two altitude samples are only freshness-gated, not explicitly paired in time, so changing altitude/datum error still needs a budget.

**Why Gazebo passed:** the corridor, registered field, spawn, yaw, and roof endpoint are fixed together.

**Required:** establish a surveyed/known registration and a verified clear-to-climb condition for the real setup. Fixed geofence coordinates remain acceptable; no dynamic geofence input system is needed. The issue is validating their relationship to the aircraft's local frame, not whether they are hard-coded.

### R23 — Output timing, diagnostics, and unnecessary processing share the control path

**High for unbounded control delay; Medium for ordinary overhead — Confirmed coupling, Pi saturation unproven.** Locations: M:334–395, 1194–1684, 1840–1902, 1962–1999, 2197–2227; B:61–81; P:1468–1700; O:470–758; R:503–779.

M drains telemetry, processes images or scans, performs GUI calls, writes diagnostics, and sends commands in one loop. The nominal 50 Hz loop sleep is only a maximum-rate limiter, not a guaranteed deadline. A GUI stall, terminal backpressure, filesystem delay, or expensive fit delays both telemetry supervision and command output. The first pre-entry snapshot writes NPZ/JSON synchronously between calculation and dispatch. It is only once and normally occurs while stopped, so it is not the leading sustained-load problem.

Frames are copied under the shared sensor lock before checking whether their sequence is new; B makes another debug copy and prepares an inset on every detection. Forward-frame RGB conversion remains active after the native corridor takes over although those images no longer affect its control. LiDAR RANSAC has many Python-level hypotheses, with percentile operations inside them; its worst-case duration has not been measured on the Pi.

**Why Gazebo passed:** desktop compute and simulation pacing do not demonstrate bounded real-time service under two-camera Pi load.

**Required:** establish an independently supervised command-age budget and remove avoidable work from that budget. Keep the GUI for testing, but do not let display/I/O liveness determine aircraft safety. T's independent 20 Hz sender/0.35 s cache watchdog is a useful existing concept, not proof that M already has this protection.

T's output thread is not itself fully supervised: its `_loop()` calls the sender without an exception boundary, and the main mission loop does not check that the thread remains alive. It also discards ordinary velocity dispatch results. A transmit exception can therefore terminate the command thread while perception/state processing continues. A cache timeout only protects against old commands while the output thread is functioning; FC link-loss behaviour and thread liveness still need verification.

## 4. What I trust and would retain

| Component / decision | Assessment and limits |
| --- | --- |
| Native non-ROS controller boundary | Good separation of sensor datatypes, state logic, and semantic command output. Suitable to retain for Pi integration. |
| Wall-relative 2D geometry | Appropriate for a straight, bounded corridor with observable walls. RANSAC, residual/span/inlier tests, width/parallel checks, and sector consistency are substantive safeguards, not just ideal geometry assumptions. |
| Explicit conservative transitions | Obstacle decision and reassessment normally command STOP; unsupported transitions abort; avoidance is revalidated instead of blindly resuming an old PASS. Preserve these behaviours while repairing the gaps above. |
| Bounded commands and correction hysteresis | Useful damping and risk reduction. Tune on the real vehicle instead of replacing all P-control merely because PID exists. |
| Frame/sign convention in native output | Native FLU forward/left/up/CCW is explicitly mapped to MAVLink forward/right/down/clockwise yaw rate. No demonstrated wholesale ENU/NED or degrees/radians error in that mapping. Sensor installation signs still require bench checks. |
| Entry/exit displacement mathematics | Heading-projected measured travel is preferable to open-loop time-distance assumptions. Retain it with health/continuity checks. |
| Altitude helper | Saturated commands, finite/stale checks, low-velocity settling, fresh-sample dwell, clock-reset checks, and wall watchdogs are worthwhile. Reacquisition uses the same target, not repeated 1 m descents. |
| Once-per-new-scan/frame evidence | Prevents an ordinary fast polling loop from counting the same cached observation repeatedly. Preserve it alongside acquisition-age validation. |
| Latest-frame storage and bounded small histories | No evidence of a normal unbounded camera-history or controller-history memory leak. Eight GB RAM is not shown to be the bottleneck. Driver abnormal assembly is a separate R19 issue. |
| Lightweight banner detector | HSV, morphology, and contours are a reasonable baseline for the specified banner. The active detector does not run ORB or an AI model. Do not attribute unused detector costs to this mission. |
| Hardware safety intent | Explicit opt-in control, normal rather than force arming, gated sends, and an expiring command cache are good existing ideas in the separate hardware path, despite its current integration defects. |
| Fixed environment configuration | Known 3.5 m width and replaceable fixed field coordinates are acceptable current mission assumptions if measured/registered for the physical course. No speculative input framework is necessary. |

ArduPilot documents body-frame velocity semantics and metres/radians for these setpoints, supporting the sign/unit assessment. The camera sender's mask also sets the force bit while all acceleration components are ignored; that is a clarity/portability cleanup, not evidence of an observed force-control failure. Its constant timestamp is likewise not a demonstrated cause of this mission's behaviour. [ArduPilot Guided command reference](https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html)

## 5. Meaningful Pi optimisation review

### Actual workload, not assumed workload

The Pi can directly connect two cameras through its two MIPI connectors; the hardware arrangement is feasible in principle. That does not certify the desired simultaneous frame rates or this application's latency. [Raspberry Pi camera software: multiple cameras](https://www.raspberrypi.com/documentation/computers/camera_software.html#use-multiple-cameras)

Before coverage, M runs front-camera perception and native corridor processing in different mission phases. It does not run two vision algorithms simultaneously in these states. Both cameras may still be acquiring on the final Pi, and the front conversion continues after its control phase. Budget acquisition/ISP/memory costs separately from algorithm costs.

For scale, one 640×480 three-channel image is 921,600 bytes. Two such 30 FPS streams represent about **55.3 MB/s of image payload for one pass through memory**, before extra copies, conversion, sensor formats, preview, or logging. This is arithmetic, not a measured CSI bandwidth or Pi throughput limit. The GS sensor's native 1456×1088 image contains about 5.2 times as many pixels; processing native resolution by default would invalidate the existing cost and pixel-threshold assumptions. Native sensor specifications do not force native-resolution mission processing. [Raspberry Pi camera documentation](https://www.raspberrypi.com/documentation/accessories/camera.html#global-shutter-camera)

| Opportunity | Evidence / impact | Engineering recommendation, not an implementation |
| --- | --- | --- |
| Bound acquisition-to-command age | R03/R04/R18/R20/R23; average CPU alone can conceal dangerous latency tails. | Highest priority. Measure capture age, queue age, fit time, command age, missed deadlines, and watchdog actions. |
| Keep serial reception prompt | D500 serial reads wait for the mission loop to finish. | Prevent controller/diagnostic work from creating a stale-input backlog. Bounded/latest observations matter more than processing every old scan. |
| Remove duplicate frame copies | M copies before checking sequence; B copies for debug output. | Process only new observations; keep ownership/thread safety explicit. Do not remove copies blindly and introduce shared-array races. |
| Decouple diagnostic rendering rate | Mask inset, overlays, `imshow`, and `waitKey` share the mission loop. | Retain the test GUI, but permit lower-rate rendering and ensure display failure cannot bypass safety. |
| Avoid unnecessary post-handoff vision work | Forward callback continues colour conversion after camera authority ends. | If both streams must remain active, retain capture/health reporting but avoid unnecessary conversion/detection/rendering. No need to process the downward image for an excluded task. |
| Profile LiDAR fits before changing them | Pre-entry/cruise fit two walls with up to 2×120 hypotheses; obstacle extraction can fit two walls and front/rear faces, up to 2×140 + 2×180 = 640 hypotheses per update. Reassessment has its own fits. Candidate loops compute percentiles and medians. | Likely meaningful CPU hotspot. Benchmark representative and difficult/noisy scans with both cameras active. Consider bounded candidate work/early termination or sharing fit utilities only if measured necessary; do not weaken geometric validation to meet a guessed FPS. |
| Bound logging and storage delays | Synchronous snapshot plus terminal output; T prints each scan. | Keep useful diagnostics, with bounded buffering/rate and disk-error behaviour. Do not mandate full-resolution continuous video as a prerequisite. |
| Reduce duplicated geometry implementations cautiously | P, O, and R have related but different RANSAC and validity logic. | Common tested primitives can reduce inconsistent safety semantics. This is primarily a reliability/maintenance benefit; do not erase intentional state-specific thresholds. |
| Treat CRC and sorting as secondary | D uses a bitwise Python CRC and sorts points; SA sorts again after angle conversion. | Profile first. A lookup-table CRC is a possible bounded improvement, but not the first engineering issue. The second sort has a reason after angular remapping. |
| Keep small numerical operations readable | Small NumPy arrays, short deques, 2D SVD, scalar control calculations. | No evidence requiring a C++ rewrite, float32 conversion everywhere, GPU acceleration, or dropping confidence checks. |

**What remains unknown:** real resolutions/FPS/exposures, front lens, Pi OS/camera stack, cooling, power supply, LiDAR scan characteristics, FC firmware/link baud, preview/logging settings, and worst-case timing. I have not measured Pi CPU utilisation, thermal throttling, RAM use, USB behaviour, or sustained dual-camera throughput. It would be unjustified to declare either “the Pi cannot handle this” or “50 Hz is guaranteed.”

Thermal/power testing should use the final enclosure and both cameras/LiDAR/FC communications active. This is a validation requirement, not a claim that current hardware is throttling. The most useful outcome is bounded sensor/command age under sustained load, not a single high average FPS figure.

## 6. Important fixed assumptions to verify

| Location | Value / assumption | Assessment for hardware |
| --- | --- | --- |
| M startup / run notes | Already airborne, expected heading, correct mode | Must become an explicit verified handoff. Operator-managed takeoff is an acceptable current test boundary. |
| VM:961–976 | Front 640×480, ~60° horizontal FOV, 30 FPS | Fix an initial real processing mode; do not silently substitute the downward lens or native resolution. |
| B | HSV bounds, 700 px minimum, 5×5 opening, aspect/shape rules | Reasonable baseline; test the specified banner under real imaging conditions. |
| M camera control | 0.003 gain, ±0.50 m/s, 20 px, 30 centred frames | Resolution/optics/rate-dependent. Requires measured alignment and closed-loop response. |
| M search / approach | Left 0.50 m/s; forward 0.20 m/s | Search needs bounds; speed safety depends on latency/braking/clearance, not Gazebo success alone. |
| H / M approach | ±10°, three smallest returns; 0.50 m trigger; five lost frames | Fragile arrival semantics; treat independently from normal HSV tuning. |
| M descent | Handoff height minus 1 m; max 0.50 m/s | Useful relative move, not a proof of safe AGL or roof clearance. |
| M hover | ±0.15 m altitude drift; all axis speeds ≤0.10 m/s; two source seconds | Sensible settling idea; validate real velocity/height noise and sampling. |
| P/C/O/R | Nominal width 3.5 m, straight/near-parallel walls | Suitable if the real corridor supports it. Width reconstruction is a prior, not a measurement. |
| P/O/R fit gates | Roughly 0.07–0.09 m inlier distances, minimum spans/counts, ~0.09–0.13 m fit RMS | Fit to actual angular density, surface returns, and motion; do not lower thresholds merely to force acceptance. |
| VM / D | 500 simulated rays at 10 Hz; native protocol 230400 baud, nominal 0.02–12 m | Real angular coverage, rate, clipping, and invalid values must be confirmed. |
| P/C/O attitude | IMU optional; 8°/10°/12° tilt limits | Optional/freshness behaviour needs correction; angular limits need scan-plane testing. |
| N entry | 0.75 m at 0.20 m/s in M; distance `None` in T | Explicitly configure and supervise real entry; maintain live safety checks. |
| C motion / safety | Cruise 0.35 m/s; slowdown 2.50 m; trigger 1.35 m; emergency 0.75 m | Practical baseline, not a certified stopping envelope. |
| O passage | 0.65 m width, 0.25 m side margins; 0.45 m hard outer-wall clearance | Replace assumed aircraft dimensions with the actual envelope and error budget. |
| O bypass | Shift ≤0.20 m/s; pass 0.18 m/s; 0.35 m rear margin | Verify lateral settling and full-aircraft passage, not only scanner passage. |
| C/E exit | Side >2.40 m, front ≥3 m, three scans; 1.20 m commit at 0.15 m/s | Missing data cannot stand in for an opening; pose continuity and clear-path safety are required. |
| M simulation overrides | 1 s pose freshness; 6 s pose-loss; entry 60 s; reassessment 32/48 s | Explicitly simulation-tuned. Reassess for measured hardware latency and clearance. |
| M field entry | Fixed local N/E field and north-aligned corridor | Coordinates may remain fixed; physical/local-frame registration must be validated. |
| M climb | Nominal 10 m HOME relative; translated local-Z target | Reasonable on the known flat site if datum/clearance and ongoing localization are verified. |
| T output | 20 Hz; cached command expiry 0.35 s | Useful baseline concept, not yet an integrated or measured hardware guarantee. |

## 7. Where Gazebo gives confidence, and where it does not

### Genuine confidence from the successful simulation

The nominal state ordering, intended lateral/yaw signs, ideal wall geometry estimation, expected obstacle-side selection, and corridor-to-field sequence have meaningful integration evidence. Simulation is a good place to confirm these relationships. Source inspection also finds safeguards that are genuinely useful on hardware.

### False confidence to avoid

- A working **Gazebo sensor wrapper** does not validate the separate serial driver or hardware runner.
- A regular stream does not test freshness or terminal behaviour during a complete dropout.
- A rendered empty sector does not validate the interpretation of a real missing return.
- A colour-correct object in a renderer does not validate exposure, focus, ISP output format, or white balance.
- Stable SITL position does not validate continuity and accuracy of the physical FC's estimator under corridor conditions.
- A nominal desktop loop rate does not establish Pi worst-case command latency.
- A printed “LAND requested” does not prove LAND was dispatched, accepted, or completed.

### What can be checked before flight without new hardware capability

Use offline inputs and Gazebo faults to exercise camera freeze, decode failure, scan loss, sector loss, corrupted/incomplete scans, telemetry backlog, stale/missing attitude, invalid pose, EKF jumps, reduced sensor rate, GUI stalls, and every abort transition. Vary spawn heading/offset and apply wind/dynamics perturbations where the simulator supports them. Inspect the *actual outputs and bounded terminal result*, not just the state name or log message.

### What must be measured on the real system

Optics/focus/exposure, actual image format/stride, physical camera/LiDAR axes, LiDAR material and sunlight response, airframe self-returns, estimator quality, mounting vibration, vehicle stopping/tracking behaviour, dual-camera load, power integrity, sustained thermal behaviour, and FC failsafe/pilot-override response. Noise injection can test robustness to a chosen noise model; it cannot establish that the model matches the physical sensor.

## 8. Checks performed in this review

The following were offline, in-memory calls with no opened sensor/vehicle connection. Bytecode writing was disabled. They reproduce specific logic paths, not complete flight scenarios.

| Check | Result | Finding |
| --- | --- | --- |
| Import `native.run_corridor_real` | `ModuleNotFoundError: native.hardware.mavlink_commands` | R01 |
| VERIFY_LOCK, valid geometry, confidence 0.9, lateral error 0.15 m, front 5 m, clocks 600 s old, 100 updates | Remained VERIFY_LOCK; no failure; STOP throughout | R12 |
| Native reassessment past hard timeout | State ABORT_CORRIDOR, returned action `None` | R05 |
| ENTER with fresh pose and fresh 500-ray scan containing 0.20 m ranges | Forward command 0.20 m/s | R11 |
| Cruise with front 5 m returns and missing/NaN side sectors after guard time, three updates | Requested EXIT_DETECTION | R13 |
| Avoidance PASS with valid open-side wall and front clearance zero | Forward command 0.18 m/s | R14 |
| EXIT with fresh-timestamp pose but NaN X | Forward command 0.15 m/s; measured travel NaN | R21 |
| Existing `native.test_exit_clock` and `test_corridor_altitude` suites | 18 tests passed | Supports the valid helper behaviours; does not negate uncovered paths |

Existing tests were run with:

```bash
PYTHONDONTWRITEBYTECODE=1 \
PYTHONPATH=/home/sid/[competition]_mission2/corridor:/home/sid/[competition]_mission2/world/integration \
python3 -m unittest native.test_exit_clock test_corridor_altitude -q
```

No general test discovery was used: this repository also contains scripts called `test_*` that open real devices or perform hardware diagnostics. They are not automatically safe unit tests. No new real-flight or end-to-end Gazebo validation is claimed here.

## 9. Priority order

“Before Pi” below means before treating the Pi as a flight-control deployment. Safe, motors-disabled sensor logging on the Pi can proceed earlier and is useful.

| Rank | Work to address | Why it comes first | Timing |
| --- | --- | --- | --- |
| 1 | Consistent command authority, dropout supervision, and terminal action delivery — R02–R05 | Without these, a local perception failure or application stop has no reliably bounded aircraft outcome. | Software corrections before controlled hardware motion; verify FC/pilot failsafes on the bench. |
| 2 | Unknown versus free-space handling, especially false exits and invalid-front bypass — R13/R14 | Missing data can actively authorise movement or exit instead of stopping. | Correct and regression-test before flight. |
| 3 | Continuous safety during ENTER and pose integrity during commits — R11/R21 | Valid entry geometry can become unsafe; invalid/jumping pose can still drive or finish a commit. | Correct before flight; measure positioning quality on hardware. |
| 4 | Safe banner-to-LiDAR handoff and bounded search — R06/R07 | Camera faults currently imply descent or unbounded lateral search. | Correct transition/supervision semantics before flight. |
| 5 | Height, vehicle footprint, scan plane, and clear-to-climb envelope — R10/R16/R17/R22 | These determine whether an apparently correct path fits the actual aircraft and site. | Define assumptions before porting control; measure/calibrate before corridor flight. |
| 6 | Executable hardware path and explicit takeoff/airborne handoff — R01 | Tested and deployed paths must be identifiable and functionally comparable. | Resolve before hardware flight runtime integration; dry-run import tests first. |
| 7 | Real acquisition age, serial assembly/backlog, and timing contracts — R18–R20 | Old or incomplete observations can otherwise look fresh enough for control. | Establish semantics before live control; tune budgets from hardware recordings. |
| 8 | VERIFY_LOCK liveness — R12 | A reproducible state can hold indefinitely despite already-expired alignment time. | Straightforward software correction and offline regression before flight. |
| 9 | Front-camera optics/servo and supported obstacle class — R08/R09/R15 | Determines whether the nominal algorithm sees and controls the actual course correctly. | Real sensor bench recordings, then controlled vehicle tests. Do not generalise unknown future requirements. |
| 10 | Sustained Pi timing and diagnostic-cost optimisation — R23 / Section 5 | Ensures both camera streams and LiDAR do not erode the safety budgets under heat and I/O load. | Measure with final acquisition settings; optimise demonstrated hotspots before autonomous corridor flight. |

## 10. What should deliberately wait, and what should not change unnecessarily

### Can wait for hardware evidence

- Final HSV/exposure/white-balance settings and front-camera gains, once the exact banner, lens, crop, and mount are available.
- Exact LiDAR noise/fit/sector thresholds, invalid-return classification details, and whether deskew or more attitude compensation is necessary. The interfaces must support freshness/validity now; the final numerical tolerances need measurements.
- Final speed, acceleration, clearance, and confirmation-duration values after measuring FC response and tracking error.
- Aggressive RANSAC optimisation, CRC optimisation, thread/process topology beyond essential supervision, or a language rewrite. First measure the whole Pi workload, including latency tails.
- Hardware shutter synchronisation between front and downward cameras, unless a later algorithm genuinely requires simultaneous views. It is not a prerequisite for the current independent pre-coverage functions.
- More general obstacle recognition, arbitrary banner appearance, variable terrain, dynamic geofence input, or a new global planner without a demonstrated mission need.

### Should not wait for hardware

- Missing import/configuration blockers, missing-data branches that suppress their own timers, missing terminal actions, nonfinite pose acceptance, invalid-front continuation, false exit from absent sectors, and indefinite VERIFY_LOCK behaviour.
- A defined command-age/authority contract and clear separation of acquisition failure from successful mission progress.
- Identifying which runner is authoritative and what establishes safe airborne entry. The current manual Gazebo takeoff is a valid test setup, not a completed autonomous takeoff integration.

### Keep rather than prematurely replace

Keep the native state machine, geometry-based wall following, measured travel commits, small bounded histories, simple banner perception, conservative ambiguous-obstacle handling, and existing altitude helper. Keep the testing GUI but prevent it from becoming a flight-safety dependency. Keep fixed known field coordinates and flat-ground assumptions where physically verified. Do not introduce AI, dense reconstruction, ROS 2, or SLAM merely to make the system look more sophisticated.

## 11. Remaining information needed before flight approval

These do not prevent the software findings above from being actionable. They prevent unsupported claims about the final physical operating envelope:

1. Exact front-camera lens/FOV, both cameras' selected processing resolution/FPS/exposure policy, and rigid mounting axes. The downward 10 mm assumption does not answer the front-camera question.
2. Actual 2D LiDAR model, angular convention, scan rate, packet protocol, valid/no-return semantics, and mounted self-occlusion. The repository contains a D500 parser, but the request specifies only “real 2D LiDAR.”
3. FC model/firmware, localization sources, telemetry transport/rates, mode/failsafe parameters, and pilot takeover procedure.
4. Measured aircraft envelope including propellers, sensor offsets, and achieved braking/position/height accuracy.
5. Physical corridor width/height, wall and obstacle surfaces/heights, banner mounting height relative to the floor, and whether obstacles match the wall-attached face model.
6. Agreed airborne-start responsibility and the relationship between real HOME/EKF origin, corridor heading, and fixed field coordinates.
7. Pi power/cooling/enclosure, preview and recording requirements, and sustained dual-camera plus LiDAR timing results.

**Bottom line:** retain the core algorithm, but repair and test the safety/state/data-validity boundaries before flight. Gazebo has meaningfully validated the nominal corridor behaviour; it has not validated the hardware adapter, failure paths, physical sensing envelope, or Pi timing budget.


