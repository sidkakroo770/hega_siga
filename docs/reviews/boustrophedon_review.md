The current script is suitable for an early sweep-behavior experiment, but a successful run would **not yet demonstrate safe area coverage or completion of Mission 2**.

The most significant findings are a camera-configuration mismatch, an unenforced “geofence,” a recovery rule that can skip all remaining rows, and continued motion based on stale or geometrically ambiguous perception.

I read all 289 lines of [the archived sweep prototype](../../archive/legacy/boustrophedon_sweep_prototype.py) and all three pages of the [mission PDF](</home/sid/[competition]_mission2/autonomous mission_[competition]_[competition].pdf>), including Figure 3. I also checked relevant local camera definitions and ArduPilot behavior. I made no code changes and did not launch or command a vehicle. Validation here consists of static analysis, isolated synthetic-image checks, and numerical checks of the recovery condition—not an end-to-end Gazebo test.

## Mission intent versus implemented behavior

The PDF specifies this sequence:

1. Take off, climb to 5 m, move approximately 1 m forward, and decode the starting QR.
2. Identify the green corridor banner, align, descend to approximately 3 m, and traverse the 3.5 m-wide corridor.
3. Enter the delivery area, ascend to approximately 10 m, locate the matching QR, and avoid red restricted zones.
4. Descend to 5 m, lower the payload using a controlled pulley, and release only after it reaches the ground at the target.
5. Ascend, reacquire the return entrance, traverse the corridor, and land at the starting point.

Figure 3 depicts a nominal 40 × 30 m delivery area and a 10 m corridor, multiple QR locations, a red zone, and different outward/return paths. It explicitly warns that the layout can change. The document also requires supplied geofencing, completion within 15 minutes, and flight-data recording.

This script instead:

- Connects to ArduPilot and a downward-image topic.
- Arms and attempts a velocity-driven climb.
- Flies directly to the first corner of a fixed rectangle.
- Visits a 4 × 4 serpentine grid.
- Reacts to red image regions by moving right, tracing forward, pausing, and selecting a recovery waypoint.
- Sends one zero-velocity command and exits.

There is **no QR decoding, target matching, LiDAR processing, payload operation, corridor handoff, return, or landing** in this file. Given that your corridor mission is already proven, these are integration/scope gaps here—not evidence that the corridor implementation is defective.

For the classifications below:

- **Definite:** directly established by the code or an isolated reproduction.
- **Likely:** strong evidence, but the outcome depends on the active setup.
- **Potential:** requires a particular condition that has not been established.
- Severity describes the consequence if the issue is exercised, not its frequency.

## PART A — Gazebo / simulation audit

All script line references below refer to `archive/legacy/boustrophedon_sweep_prototype.py`.

### A1. Sensor and mission assumptions

| ID / Location | Assumption or problem and evidence | Gazebo relevance and possible failure | Severity / certainty |
|---|---|---|---|
| **A1 — CameraViewer, L28–34** | Subscribes only to `/iris/camera_downward/image_raw`. This contradicts the stated one-front-camera configuration. No mounting transform or alternate camera selection exists. | With only the forward topic, startup waits forever. With an available downward camera, testing can succeed using a sensor the intended aircraft does not have. | **Critical — definite configuration mismatch.** Whether the active world supplies this topic is unverified. |
| **A2 — main, L141–286** | The script treats completing/exhausting grid waypoints as completion. There is no QR identity, search-success condition, mission handoff, or delivery state. | It can print “SWEEP COMPLETE” without detecting any delivery target. This blocks interpreting the run as Mission 2 validation, though not as an isolated path experiment. | **High — definite.** |
| **A3 — entire file** | No LiDAR subscription, scan parsing, distance checking, or sensor-health handling. | Non-red obstacles do not influence this controller. Success in an open red-patch world does not validate obstacle avoidance. An external flight-controller avoidance configuration could provide protection, but this file neither establishes nor verifies it. | **High — definite absence; collision is conditional.** |
| **A4 — imports L11–12; L32–34, L143–144; L136–137** | Fixed Gazebo Transport/message versions, absolute topic, localhost UDP port 14550, and mandatory GUI display. Subscription success is not checked. | A different namespace, camera plugin, bridge, port arrangement, Gazebo version, or headless environment can prevent operation. An incompatible GUI backend can terminate the process. This is direct Gazebo Transport, not ROS. | **Medium — potential setup failure.** |

The mismatch is supported by more than the topic name: the local [full-world model](/home/sid/[competition]_mission2/world/models/models/iris_miss2_full/model.sdf:994) defines that camera pitched approximately 90° downward, at 640 × 480 and 30 Hz, with horizontal FOV 1.047 rad. That establishes what **that model** supplies; it does not establish which world/model you currently launch.

### A2. Field geometry, coverage, and navigation

| ID / Location | Assumption or problem and evidence | Gazebo relevance and possible failure | Severity / certainty |
|---|---|---|---|
| **A5 — constants L15–17; grid L93–106** | Coordinates are fixed at X = −19.3…17.9 and Y = −13.4…14.3. No relationship to the supplied geofence, field pose, or EKF origin is represented. | Translating/rotating the field or changing the vehicle’s localization origin can move the entire sweep off the intended area. The rectangle is 37.2 × 27.7 m, not the diagram’s nominal 40 × 30 m; that could be an intentional inset, but its registration is undocumented here. | **High — definite dependency; actual misregistration unverified.** |
| **A6 — L15–16 and all motion branches** | “GEOFENCE BOUNDARIES” are used only to generate waypoints. There is no boundary check on current position or commanded motion. | Evasion, tracing, recovery, overshoot, or initial transit can leave the rectangle. Boundary waypoints also have no explicit vehicle-size or stopping margin. | **Critical — definite missing enforcement.** External autopilot fencing remains unknown. |
| **A7 — generate_grid_4x4, L93–106** | Exactly four rows and four points per row, independent of camera footprint or target readability. Spacing is **12.4 m along rows** and **9.233 m between rows**. | Visiting every point does not establish that all ground was observable or that QR codes would be readable. Small FOV/altitude changes can create coverage gaps. | **High — definite absence of a coverage criterion; actual gaps depend on geometry.** |
| **A8 — initial transit L158–167; yaw loops L177–184** | Initial corner transit never calls vision. Per-waypoint yaw alignment also runs without checking red zones. | A red zone between spawn and the first corner is crossed without reaction. Braking drift during alignment is also unobserved by mission logic. | **High — definite blind phases.** |
| **A9 — waypoint checks L197–200, L272–280; TOLERANCE L17** | A waypoint is accepted within a 1.5 m disk; full 1.5 m/s speed is commanded until acceptance. No cross-track or coverage check. | The flown path can cut corners or miss strip endpoints. Localization noise can cause early acceptance. Braking and position accuracy are delegated to the flight controller. | **High — definite behavior; size of resulting errors is setup-dependent.** |
| **A10 — L177–184, L202–204, L207–222, L278–280** | `target_yaw` is computed in the alignment loop and then held throughout the inner motion/recovery loop. Translation direction is recomputed toward the waypoint, but camera heading is not. Evasion uses commanded yaw rather than measured yaw. | After lateral displacement, recovery can move diagonally outside the front camera’s useful view. Yaw lag/drift breaks the assumed relationship between image-left and commanded-right. | **High — definite decoupling; hazard depends on mounting and motion.** |
| **A11 — L177–184** | Alignment is checked before testing whether the waypoint is already reached. The bearing to a nearby point can change sharply with small position errors. No yaw-loop timeout exists. | This is especially relevant to waypoint 1, already approached within tolerance. The drone can rotate unnecessarily or remain in alignment while noisy bearing estimates move around. | **Medium — likely near reached waypoints.** |
| **A12 — velocities L20–22; motion branches** | Instant changes in requested velocity between sweep, zero, lateral evasion, tracing, and recovery. No mission-level braking distance, acceleration bound, or clearance calculation. | A responsive model can conceal overshoot. A heavier/slower vehicle model can enter a zone before lateral displacement develops. There is no explicit turn radius: turning behavior is stop-and-yaw, subject to vehicle dynamics. | **High — potential failure; controller delegation itself is acceptable.** |

There is no proven ENU/NED sign error inside the vector arithmetic. `LOCAL_POSITION_NED` and outgoing local-NED velocity use compatible conventions; the missing evidence is the transformation from the **field’s world coordinates** to that frame.

### A3. Perception and avoidance logic

| ID / Location | Assumption or problem and evidence | Gazebo relevance and possible failure | Severity / certainty |
|---|---|---|---|
| **A13 — process_vision, L114–123** | Fixed HSV red bands and a largest-contour threshold of `>5000` pixels². No normalization by resolution, range, altitude, or projected size. | Changing render resolution, zone size, altitude, lighting, material, or camera FOV changes detection. A visible red zone below the threshold becomes `CLEAR`. Clean saturated textures make performance look better than it is. | **High — definite sensitivity.** |
| **A14 — L121–130** | Only the largest red contour is considered. Other red regions are discarded, regardless of location. | A large left-hand zone can yield `TRACE` while a smaller but still substantial red zone occupies the path ahead. | **High — definite, reproduced with synthetic images.** |
| **A15 — L124–130** | State depends only on whether the bounding box’s right edge exceeds `W//3`. Vertical position, distance, vehicle footprint, and clearance are ignored. Both middle and right regions mean `EVADE`; only entirely-left regions mean `TRACE`. | “Red safely left” has no metric safety meaning. A far-right red object can command rightward motion toward it. A downward view can first detect a zone when the vehicle is already over it. | **Critical — definite missing geometric safety test; particular incursions conditional.** |
| **A16 — L112; L214–227** | No qualifying red contour means `CLEAR`; loss of sight is treated as having passed the zone. No distinction among clear scene, occlusion, small projected size, partial-frame clipping, and failed detection. | Turning, texture changes, or a zone leaving the image can trigger recovery while the aircraft remains beside or above the restricted region. | **High — definite state ambiguity.** |
| **A17 — L191–229** | Every avoidance maneuver starts to the right. There is no check of free space to the right, nearby fence, another zone, or navigability of the detour. | A zone near the right boundary, adjacent zones, or a concave arrangement can cause fence exit, repeated cycling, or collision. | **Critical — definite unchecked maneuver policy; outcome conditional.** |
| **A18 — transition branches L192–195, L209–217, L222–229, L264–266** | Detection often changes only the state. The sweep/recovery command is not immediately replaced on detection; EVADING/TRACING send movement before interpreting that frame’s next-state condition. | The previous command remains active through at least another processing iteration. The log “Halting sweep” is not an immediate stop command. Under load or low frame rate this delay grows. | **High — definite delay; incursion distance depends on timing.** |
| **A19 — HOVER_PAUSE, L231–236** | Three seconds of wall-clock time is treated as sufficient pause; actual velocity/stability is not checked. Newly detected red does not affect this state. | Recovery can start without settled motion or with red still visible. Re-detection is handled later in RECOVERING, so this is not permanent blindness, but the transition is not evidence-based. | **Medium — definite limitation.** |
| **A20 — FIND_NEXT, L238–257** | A waypoint is deemed passed when its projection onto the old heading is `<=0.5 m`. This is applied to every subsequent serpentine waypoint, including the next row in the opposite direction. | At a row end it can discard unvisited rows and falsely finish the sweep. | **High — definite, numerical counterexample below.** |
| **A21 — recovery L259–280** | “Selected safe coordinate” is selected only by forward projection. Neither destination nor connecting segment is established safe. Skipped path segments are never marked unsearched or revisited. | Recovery can repeatedly re-enter the same zone, stall in an evade/recover cycle, or leave permanent coverage holes. | **High — definite unsupported safety/coverage assumption.** |

Two concrete checks substantiate these findings:

- Using the actual `process_vision()` function with display suppressed, a large left red rectangle plus a smaller central red rectangle—**both exceeding 5000 pixels²**—returned `TRACE`. The central hazard was discarded.
- At `(18.0, −13.4)`, just beyond the first row’s endpoint, with heading `0` and recovery starting at waypoint 4, **all remaining waypoints fail the forward-distance test** because every remaining X coordinate is at most 17.9. The recovery index advances to 16 and the script declares completion. No later row needs to have been flown.

### A4. Telemetry, altitude, timing, and termination

| ID / Location | Assumption or problem and evidence | Gazebo relevance and possible failure | Severity / certainty |
|---|---|---|---|
| **A22 — Telemetry L48–62; main L153–154** | Position, yaw, and altitude start as zero. No “received valid sample” flag, message-age check, source filtering, EKF-health check, or requested telemetry rate. | Missing streams can appear to be valid zeros; interrupted streams remain frozen. The script can keep refreshing commands toward a waypoint it falsely believes it has not reached. | **Critical — definite missing validity gates.** |
| **A23 — CameraViewer L31–46; main L149–150, L187–188** | Camera readiness means one frame has arrived. Frame timestamps and sequence/freshness are ignored forever afterward. | A frozen clear image permits continued sweep; a frozen red image can drive indefinite evasion/tracing. Reprocessing the same frame is mistaken for ongoing observation. | **Critical — definite.** |
| **A24 — arm_and_takeoff L73–91** | Requests mode 4, waits a fixed second, arms, then sends upward velocity; no `MAV_CMD_NAV_TAKEOFF`, mode verification, or command-result handling. | On normal landed Copter behavior, velocity control may not initiate takeoff. The loop can remain stuck sending climb commands. It can appear to work if testing starts already airborne or another component initiated takeoff. | **High — likely fresh-start blocker; active firmware/state unverified.** |
| **A25 — L62, L86–89, L154; horizontal commands** | “10 m” climb exits at `0.90 × 10 = 9 m` home-relative altitude. Subsequently only `vz=0` is requested; altitude is never rechecked. | The actual camera footprint can differ from the assumed 10 m condition. Overshoot is uncontrolled at mission level. Sloped or elevated terrain changes height above ground despite constant home-relative altitude. | **High — definite altitude-reference/monitoring limitation.** |
| **A26 — L53–55; L80, L84–91, L144, L149–150, L159–167, L177–184, L186–282** | Unbounded waits for heartbeat, camera, arming, climb, arrival, alignment, and avoidance. Telemetry drains until the receive buffer is empty. | Missing data, unreachable waypoints, persistent red, or sustained message backlog can stall progress or starve command processing. There is no 15-minute budget enforcement. | **High — definite missing deadlines; telemetry starvation is potential.** |
| **A27 — sleeps L76–91, L150, L167, L184, L282; timer L216, L226, L234** | Scheduling and the three-second pause use wall time, independent of simulation time. Main-loop sleep is 0.05 s **in addition to** processing, not a guaranteed 20 Hz period. | Pause/unpause, real-time-factor changes, slow rendering, and CPU load alter the number of observations and commands per simulated second. The pause can finish while simulated physics is stopped. | **Medium — definite timing dependency.** |
| **A28 — image_callback L36–46** | Assumes tightly packed data; ignores `msg.step`. Unrecognized formats are treated as arbitrary uint8 channels. Grayscale is accepted despite color being essential. | Padded or different-depth images can fail reshape/conversion or produce wrong colors. Grayscale produces zero saturation and therefore no red detection: effectively `CLEAR`. | **High — definite unsupported-format behavior; default RGB path is valid.** |
| **A29 — process_vision L114–137; main L186–282** | Image processing, rendering, telemetry work, state updates, and command scheduling share the main control path. No exception handling or guaranteed shutdown action. | GUI errors, malformed images, OpenCV exceptions, or slow contour processing interrupt control. There is no application-level watchdog or controlled abort. | **High — definite architectural dependency; triggering fault is potential.** |
| **A30 — L284–286** | Normal completion sends one zero-velocity command with default yaw zero, destroys windows, and exits. No verified hover, return, landing, or explicit controller handoff. | An aircraft can remain airborne after “complete.” The stop also requests a north-facing yaw, potentially adding an unintended final rotation. | **High — definite termination behavior.** |

For A24, the local [ArduPilot velocity controller](/home/sid/ardupilot/ArduCopter/mode_guided.cpp:827) returns through safe ground handling when disarmed or landed. The [official Guided examples](https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html) also initiate takeoff before velocity control. I have not established that the locally checked source is the firmware your simulator currently runs.

Important qualification: missing application-level shutdown handling does **not** prove that the autopilot will execute the last velocity forever. ArduPilot documents a Guided velocity-command timeout. However, repeatedly refreshing commands using stale perception/telemetry can prevent that timeout from protecting the vehicle. Exact behavior depends on firmware and configuration. [ArduPilot Guided command documentation](https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html)

### Checks that did not reveal the suspected fault

- No explicit camera FOV or fixed image resolution exists in this Python file. Width and height come from the image. The **pixel-area threshold remains resolution-dependent**.
- No fixed physical QR/red-zone dimensions exist. QR processing is absent.
- No LiDAR range, angle, or scan-rate constants exist because LiDAR is unused.
- Yaw calculations use radians consistently.
- `relative_alt / 1000.0` correctly converts millimetres to metres.
- Negative NED vertical velocity correctly means upward.
- `(-sin(yaw), cos(yaw))` is the correct horizontal rightward direction relative to that yaw in NED.
- The type mask enables velocity and yaw and ignores position, acceleration, and yaw rate. It also sets the force-selection bit while acceleration is ignored; this is unnecessary, but not evidence that the normal velocity commands are invalid.
- A Gazebo-world/EKF-origin mismatch is a **risk to verify**, not a confirmed axis swap. MAVLink defines local-NED coordinates relative to the localization origin, not arbitrary Gazebo map coordinates. [MAVLink message reference](https://mavlink.io/en/messages/common)

## PART B — Real-world Raspberry Pi 5 audit

The critical Gazebo issues above carry directly into real flight. The following are the additional transfer risks and their practical implications.

### Camera and perception

| Issue / code evidence | Real-world consequence | Severity / certainty |
|---|---|---|
| **One front camera versus downward view — L32, L124–130** | A fixed front-facing camera does not provide the same ground coverage or overhead target visibility as the simulated downward camera. Red disappearing from a front image can mean it is now under the drone. A camera pointed toward the horizon may not see nearby ground targets at all. | **Critical — definite mismatch; exact visibility requires mounting angle and lens.** |
| **Unknown lens, no intrinsics/extrinsics — process_vision** | No focal length, principal point, distortion model, camera-body transform, or mounting-offset information is used. Image columns cannot establish ground clearance. The physical meaning of 5000 pixels and one-third image width changes with lens and mounting. | **High — definite missing geometry; real detection range unknown.** |
| **Altitude and perspective — L123–130, L154** | Apparent area varies with distance, pitch, roll, and target orientation. For a level nadir view, area scales approximately with inverse height squared; for a front view, slant range and foreshortening dominate. Partial clipping can push a hazard below threshold precisely when it is nearby. | **High — likely.** |
| **Fixed HSV bands — L25–26, L114–117** | Sunlight, shadows, glare, faded paint, automatic exposure, white balance, sensor color response, and compression can move genuine red outside the mask. Red objects unrelated to restricted zones can trigger avoidance. | **High — likely; outdoor dataset missing.** |
| **Single-frame, largest-contour decision — L119–130** | No temporal consistency, confidence, semantic validation, or multi-zone handling. Noise can fragment a zone; background red can dominate. Threshold flicker can repeatedly change behavior. | **High — definite limitation; frequency unknown.** |
| **Motion/vibration — no attitude compensation or image-quality check** | Aircraft tilt and vibration move image boundaries independently of ground clearance. Exposure-time blur reduces usable detail and can damage segmentation and eventual QR decoding. | **High — potential; depends on exposure, mounting, speed, and vibration.** |
| **Global-shutter interpretation** | The chosen camera avoids rolling-shutter readout distortion, but finite exposure can still produce motion blur. Global shutter does not solve camera vibration, lighting variation, calibration, or perspective. | **Medium — definite distinction; blur magnitude unknown.** |
| **No frame age or synchronization — L46, L187–189** | A frame can be paired with a later vehicle pose/yaw. Motion during sensor and processing latency makes the visual state inaccurate for the current aircraft position. | **Critical — definite missing checks; actual latency unmeasured.** |
| **No QR algorithm anywhere** | Target identity, pixel/module requirements, decoding range, target-center localization, and delivery alignment have not been exercised by this script. Red-mask performance cannot establish QR feasibility at 10 m. | **High — definite mission gap.** |

The Raspberry Pi Global Shutter Camera supports up to 1456 × 1088 at 60 Hz, but that is a device capability—not evidence that this pipeline runs at that resolution/rate. Its lens is separate, so there is no justified real-camera FOV to insert into this review. [Official camera specification](https://www.raspberrypi.com/products/raspberry-pi-global-shutter-camera/)

The code does not explicitly project pixels into world coordinates. Its problem is subtler: it makes **world-motion safety decisions from image columns without establishing their geometric meaning**.

### LiDAR

There is no LiDAR implementation in this file to audit for incorrect scan indices, units, clipping limits, or filtering.

The present definite gap is that **LiDAR cannot stop or alter any maneuver in this controller**, including sideward evasion and blind initial transit.

The following remain unvalidated hardware/interface requirements, not invented code bugs:

| Unvalidated property | Why it matters |
|---|---|
| Scan coverage and mounting height | A horizontal scan may see corridor walls but not ground-painted restricted zones; it may miss obstacles above or below its plane. |
| Noise, invalid/no returns, minimum/maximum range | A missing return is not necessarily free space. No validity policy is present here. |
| Scan rate, dropouts, latency | Clearance changes while measurements age; this file has no LiDAR health gate. |
| Extrinsic alignment and vehicle occlusion | Mounting offset, yaw error, propellers, and structure affect which direction/range a return represents. |
| Outdoor illumination/interference | Susceptibility depends on the unspecified LiDAR model and sensing technology. It cannot be quantified from “1 LiDAR.” |

**Severity:** High for an intended obstacle-avoidance capability. Particular sensor failure modes are potential until the sensor is specified and tested.

### Navigation, vehicle dynamics, and safety

| Issue / code evidence | Real-world consequence | Severity / certainty |
|---|---|---|
| **Trusted local position — L48–62** | GPS error, EKF drift/reset, localization latency, and incorrect field registration shift the path relative to physical hazards. A 1.5 m waypoint tolerance is not a safety margin against those errors. | **Critical — definite absence of quality checks; error magnitude unknown.** |
| **Trusted yaw — L60, L180, L207–222** | Compass bias and transient yaw error misalign the camera and commanded lateral direction. Measured yaw is only used during waypoint alignment, not to verify ongoing avoidance geometry. | **High — likely sensitivity.** |
| **No cross-track/coverage accounting — L196–204, L238–280** | Wind, tracking error, detours, and waypoint tolerance can produce gaps or excess overlap. Returning to a later waypoint does not recover the searched strip. | **High — definite coverage-accounting gap.** |
| **Altitude is home-relative, not measured AGL — L62, L87** | Terrain variation and altitude-estimation error change clearance and image scale. The code does not establish 10 m above the delivery ground or 5 m above the eventual payload target. | **High — definite reference limitation.** |
| **No dynamic clearance calculation — fixed velocities** | Braking distance and response delay depend on mass, wind, controller tuning, and payload. Restricted-zone avoidance has no allowance for aircraft dimensions or tracking uncertainty. | **Critical — definite missing safety basis; actual incursion conditional.** |
| **No progress/deadline/energy checks — main loops** | Repeated detours or stalled states can consume the mission window and battery indefinitely. No battery telemetry is processed. | **High — definite application-level omission.** |
| **No fault states — whole main loop** | Camera failure, bad localization, persistent red, or lost mode authority have no explicit abort/fallback path. Commands can continue after perception has stopped being useful. | **Critical — definite.** |
| **No runtime mode/arming/failsafe monitoring — Telemetry.update** | A failsafe or pilot mode change is not recognized by the mission state machine. Commands may be ignored while internal mission state continues evolving. | **High — definite.** |
| **No assured end state — L284–286** | A process ending is not a landed aircraft. External landing, takeover, and flight-controller failsafes are unspecified. | **Critical for standalone real operation — definite omission.** |
| **No structured mission/perception log — print statements only** | It is difficult to correlate a wrong decision with the image, pose, age, and command that produced it. Flight-controller logs may exist separately, but their configuration and linkage are not established. | **Medium — definite script limitation.** |

ArduPilot already supplies low-level stabilization and may supply fence, link-loss, battery, and EKF protections. Their absence from this Python file must not be confused with proof that they are disabled on the aircraft. Conversely, their protection cannot be credited without the actual parameter/configuration evidence.

### Raspberry Pi 5 / compute

| Issue / code evidence | Assessment | Severity / certainty |
|---|---|---|
| **Gazebo-only camera ingestion — L11–12, L28–46** | The unchanged script does not capture images from a Pi camera. A compatible acquisition/transport path would be required; none is represented here. | **High — definite deployment gap.** |
| **Full-frame processing and copies — L37–46, L114–119, L187** | RGB conversion, frame copy, HSV conversion, multiple masks, contours, and display all cost bandwidth and CPU. Cost grows with resolution and image complexity. | **Medium — potential performance risk.** |
| **Control scheduling coupled to perception — L186–282** | A slow frame, GUI stall, thermal slowdown, or CPU contention delays flight commands. The 0.05 s sleep does not provide a real-time deadline. | **High — definite timing dependency.** |
| **Callback/main-loop arrangement — L36–46, L187** | Asynchronous reception plus one latest-frame slot avoids an explicit unbounded application queue, which is good. It does not detect old frames, guarantee synchronization, or bound transport queues. | **High for freshness; no demonstrated memory leak.** |
| **Repeated processing of the same frame** | If the camera is slower than the main loop, CPU is wasted on duplicates and repeated decisions do not represent independent observations. | **Medium — definite possibility.** |
| **Mandatory desktop display — L136–137** | Headless onboard operation depends on a GUI stack unnecessarily; rendering also shares the critical loop. | **Medium — potential runtime failure.** |
| **Thermal, power, OS scheduling, concurrent workloads** | Sustained performance cannot be inferred from a desktop Gazebo run. Cooling, camera mode, other processes, and power quality are unspecified. | **Medium — potential, unmeasured.** |

There is **no evidence that Python or Pi 5 is inherently too slow for this algorithm**. Most image operations execute in native OpenCV. There is also no evidence that the desired sustained loop rate has been achieved on the Pi.

For scale only: a 1456 × 1088 three-channel image is about 4.75 MB. At 60 frames/s, one traversal of those pixels represents roughly 285 MB/s before additional conversions/copies. That is a workload estimate, not measured CPU utilization or physical CSI bandwidth.

## PART C — Hard-coded assumptions inventory

“Configurable” is not synonymous with “correct.” Several structural assumptions need validation or logic changes; exposing them as parameters would not resolve the underlying issue.

| Location | Value/Assumption | Current Purpose | Gazebo Risk | Real-World Risk | Should Become Configurable? |
|---|---|---|---|---|---|
| L15–16 | X: −19.3…17.9; Y: −13.4…14.3 | Sweep rectangle | World/origin mismatch | Wrong physical search area | **Yes**, tied to a defined frame |
| L93–106 | Axis-aligned rectangular area | Simple serpentine layout | Rotated/irregular area unsupported | Supplied fence may differ | **Eventually**; fixed fixture is acceptable now |
| L95–96 | Four X samples, four Y samples | 16-point grid | Spacing unrelated to visibility | Coverage gaps | **Yes**, based on coverage requirements |
| Derived | 37.2 × 27.7 m field; 12.4 m / 9.233 m spacing | Search geometry | World/FOV changes invalidate behavior | Lens/altitude dependence | **Yes**, preferably derived |
| L98–104 | Start at minimum X/Y; first leg toward +X | Ordering | Depends on entry location | Unsafe/inefficient entry transit | **Yes**, or explicit mission contract |
| L15–16 versus main | Waypoint rectangle acts as assumed fence | Implied containment | Evasion can leave it | Boundary violation | **Not solved by configuration** |
| L17 | 1.5 m tolerance | Waypoint acceptance | Corner cutting | Error budget may be inadequate | **Yes** |
| L20 | 1.5 m/s | Sweep/recovery/transit speed | Overshoot and reaction distance | Braking and wind sensitivity | **Yes** |
| L21 | 0.4 m/s | Tracing speed | Timing depends on model | Clearance not demonstrated | **Yes** |
| L22 | 0.3 m/s | Evasion speed | Slow lateral clearance | May not avoid zone in time | **Yes** |
| L86 | −1.5 m/s NED vertical velocity | Climb | Landed-controller incompatibility | Unbounded climb on stale altitude | **Yes**, but lifecycle issue remains |
| L154 | 10.0 m target altitude | Delivery-area scan | Footprint dependence | Not verified AGL | **Yes as mission input**; nominal value comes from PDF |
| L87 | 90% altitude acceptance | End climb | Starts sweep at ≥9 m | Scale/clearance mismatch | **Yes** |
| All horizontal commands | `vz=0`, no altitude revalidation | Nominal level flight | Terrain/estimator changes hidden | AGL and drift risks | **Structural validation required** |
| L25–26 | HSV [0,120,70]…[10,255,255] and [170,120,70]…[180,255,255] | Red segmentation | Render dependence | Lighting/color sensitivity | **Yes** |
| L123 | Contour area >5000 pixels² | Reject small red regions | Resolution/altitude dependence | Missed small/distant/clipped zones | **Yes**, with scale justification |
| L110, L127 | Image thirds; right edge >W/3 | EVADE versus TRACE | No metric clearance | Wrong maneuver from perspective | **Structural geometry issue** |
| L122 | Largest red contour only | Pick a zone | Ignores other hazards | Unsafe multi-zone scene | **Structural logic issue** |
| L112 | No accepted contour = CLEAR | Default state | Sensor/render failure looks safe | False-negative motion | **Structural state issue** |
| L207–208 | Always evade right | Detour policy | Fence/obstacle conflicts | Unsafe lateral motion | **Structural policy issue** |
| L220–222 | Trace along fixed target yaw | Pass alongside zone | No edge/distance tracking | Clearance can diverge | **Structural geometry issue** |
| L214–227 | Red disappearing means clearance | End avoidance | FOV loss mistaken for passing | Underflight/occlusion ambiguity | **Structural state issue** |
| L234 | Three-second pause | Settle before recovery | Wall time versus sim time | No proof of stability | **Yes**, but time alone is insufficient evidence |
| L249 | Forward projection >0.5 m | Select “next” waypoint | Skips later rows | False coverage/completion | **Logic defect, not tuning alone** |
| L259–280 | Straight-line recovery to selected waypoint | Rejoin sweep | Segment not validated | Can intersect hazard | **Structural assumption** |
| L182 | 0.15 rad ≈8.6° yaw tolerance | Start translation | Camera alignment error | Heading/clearance error | **Yes** |
| L177–280 | One target yaw per motion phase | Heading reference | Recovery direction diverges | Camera not watching travel direction | **Structural assumption** |
| L64 | Default yaw =0 | Default heading | Climb/end rotate north | Unexpected heading change | **Explicit contract needed** |
| L65 | Mask 3015; velocity+yaw control | MAVLink setpoint fields | Firmware compatibility | Unverified control semantics | Named protocol constant; **not arbitrary tuning** |
| L68 | `time_boot_ms=10` always | Message timestamp | Not a meaningful sender time | Poor timestamp/latency semantics | **Should be generated**, not configured |
| L69 | `MAV_FRAME_LOCAL_NED` | Command frame | Field registration required | EKF origin/frame errors | **Explicit documented contract** |
| L75 | Custom mode 4 | ArduCopter GUIDED | Autopilot-specific | Wrong platform/mode handling | Platform-specific configuration/validation |
| L143 | `udpin:127.0.0.1:14550` | MAVLink connection | Fixed routing | Onboard routing likely differs | **Yes** |
| L11–12 | `gz.transport13`, `gz.msgs10` | Simulator camera API | Version coupling | No native Pi capture | **Deployment dependency** |
| L32 | `/iris/camera_downward/image_raw` | Image source | Namespace/mount mismatch | Contradicts front camera | **Yes**, plus physical geometry validation |
| L38–45 | Format IDs 3/1; uint8; packed rows | Decode image | Other formats/stride fail | Camera output mismatch | Explicit validated format contract |
| L41–43 | Grayscale acceptable | Compatibility branch | Red becomes undetectable | Silent loss of safety perception | **Structural validity issue** |
| L136–137 | GUI enabled; `waitKey(1)` | Debug display | Headless failure | Onboard dependency/jitter | **Yes** |
| L76, L81, L89 | Sleeps 1 s, 2 s, 1 s | Startup/settling | Timing substitutes for state | Variable readiness/dynamics | **State validation needed** |
| L91 | 0.2 s sleep | Nominal climb cadence | Wall-time dependence | Jitter/stale altitude | **Yes**, with measured deadlines |
| L150, L167, L184 | 0.1 s sleeps | Camera polling/transit/alignment | Not guaranteed 10 Hz | Scheduling variability | **Yes where operationally relevant** |
| L282 | 0.05 s sleep | Nominal control cadence | Less than 20 Hz after processing | Compute stalls delay commands | **Yes**, with timing evidence |
| L48–62 | Zero-initialized, untimestamped telemetry | Cached vehicle state | Missing data appears valid | Stale/invalid localization | **Structural validity issue** |
| L46, L187 | Latest image usable indefinitely | Image cache | Frozen-image behavior | Flight after camera failure | **Structural freshness issue** |
| Entire file | Localization sufficiently accurate; origin stable | World navigation | Noise/reset untested | GPS/EKF error | Uncertainty/health contract required |
| Entire file | Camera axes match maneuver axes | Image-based steering | Mount changes break behavior | Calibration/mount errors | Extrinsics must be explicit |
| Entire file | Terrain/height variation irrelevant | Constant-altitude sweep | Flat world conceals weakness | Changing AGL | Operational constraint or validated geometry |
| Entire file | No non-red obstacle input | Open-area experiment | Limited scenario validity | Unseen hazards | **Structural sensing gap** |
| Main loops | No deadlines, progress limits, or energy budget | Run until grid ends | Infinite loops | Battery/time exhaustion | Limits should be configurable |
| L284–286 | One stop then process exit | Experiment termination | No managed flight handoff | Aircraft remains airborne | **Mission-state requirement** |
| L176, L284 | “16” in progress/completion text | Status reporting | Misleading after grid changes or early exhaustion | Poor operational feedback | Derive from mission state/count |

The companion model’s **1.047 rad FOV, 640 × 480 images, 30 Hz rate, fixed downward pitch, and 0.1–100 m clipping** are additional simulation-fixture values. They are not constants in this Python file and should not be mistaken for selected hardware specifications.

## PART D — Gazebo → real-world gap

### 1. Things Gazebo cannot realistically validate by itself

- Actual outdoor red segmentation or QR readability through the selected physical lens.
- Real exposure/white-balance behavior, glare, optical focus, contamination, and vibration.
- Actual GPS/compass interference and the site’s localization error distribution.
- Real LiDAR material response and sunlight/interference behavior.
- Sustained onboard timing under Pi temperature, power, camera-driver, and competing-workload conditions.
- Actual payload-lowering accuracy and aircraft response to the mechanism.

Simulation can model parts of these; it cannot establish that those models match your hardware.

### 2. Things Gazebo can validate if noise/perturbations are added

- Position/yaw bias, drift, delayed telemetry, missing streams, and estimator-origin changes.
- Camera latency, dropped/frozen images, resolution/FOV changes, mount-angle error, and imperfect rendering.
- LiDAR invalid returns and delayed scans once that sensor participates.
- Wind and tracking error within the fidelity of the vehicle model.
- Translated/rotated fields, varied spawn poses, multiple red zones, and zones near boundaries.
- Slow simulation, pause/unpause, CPU-induced timing variation, and controller response limits.

### 3. Things that should be tested in Gazebo before any real-world flight

- The actual one-front-camera configuration and its ability to observe the required ground regions.
- Fresh-start takeoff without hidden manual preparation.
- Field-to-NED registration and boundary containment.
- The row-end recovery counterexample and coverage after every detour.
- Initial-transit, turning, and recovery segments crossing red zones.
- Camera/telemetry failure during every moving state.
- Multiple zones, right-boundary conflicts, persistent red, and unreachable waypoints.
- A defined safe end/abort state and verified autopilot fallback behavior.
- Corridor-to-search and search-to-return handoffs, reusing the proven corridor behavior.
- Full-mission success criteria, including QR selection, delivery, return, and the time budget.

### 4. Things that must wait for hardware testing

- Final lens/FOV/focus selection and measured target readability.
- Intrinsic calibration and installed camera/LiDAR extrinsics.
- Outdoor image datasets across lighting, altitude, attitude, and speed.
- LiDAR performance against actual relevant surfaces.
- Pi end-to-end latency, sustained throughput, temperature, power, and buffering.
- Real localization accuracy, braking response, wind sensitivity, and payload dynamics.

The missing evidence that most limits this review is: the active launch/model, firmware and failsafe parameters, field-to-EKF transform, actual camera mount angle, proposed LiDAR model, target dimensions, and intended camera capture mode.

## PART E — Priority review

### Top 10 issues to investigate first

“Blocks Gazebo validation” below means blocks a meaningful safety/mission claim, not necessarily that the program cannot run.

| Rank | Issue | Why it matters | Blocks Gazebo validation? | Blocks real-world validation? |
|---|---|---|---|---|
| **1** | **Front-camera versus downward-camera mismatch** | The experiment may validate a sensor arrangement absent from the aircraft; ground visibility is foundational. | **Yes**, for the intended configuration | **Yes** |
| **2** | **No stale/invalid-data response or managed abort/end state** | A frozen clear frame or position can keep motion commands alive; completion does not land or hand off safely. | **Yes**, for robustness claims | **Yes** |
| **3** | **Image-column rules do not establish restricted-zone clearance** | Neither “left,” “gone,” nor “right evasion” proves the aircraft avoids the zone. | **Yes**, for avoidance claims | **Yes** |
| **4** | **Field registration and unenforced geofence** | The path may be misplaced, and detours are unconstrained even if registration is correct. | **Yes**, for containment claims | **Yes** |
| **5** | **Recovery skips unvisited rows** | There is a concrete case where the controller declares completion after the first row. | **Yes**, for coverage claims | **Yes** |
| **6** | **Fresh-start takeoff/mode lifecycle** | The mission may depend on an already-airborne vehicle or external preparation. | **Likely**, pending active firmware test | **Yes**, until verified |
| **7** | **Blind transit and direction/camera mismatch during recovery** | Important movement phases are unobserved or do not align with the camera’s view. | **Yes**, for path safety | **Yes** |
| **8** | **No demonstrated search coverage or QR feasibility** | Fixed spacing, altitude tolerance, and unknown optics provide no evidence that every target can be found and decoded. | **Yes**, for search success; not basic path demos | **Yes** |
| **9** | **No LiDAR contribution to this controller** | The intended sensor does not protect initial transit, sideward maneuvers, or non-red obstacle encounters. | **Yes** for obstacle-bearing tests; no for an explicitly empty fixture | **Yes** for claimed obstacle avoidance |
| **10** | **Unmeasured perception/control timing and vehicle response** | Reaction distance depends on frame age, processing jitter, yaw lag, and braking—not only commanded speed. | **Yes**, for performance claims | **Yes** |

### What I should NOT change yet

- **Do not rewrite the proven corridor navigation because this script lacks corridor handling.** Its integration contract needs review; its proven behavior should remain the baseline.
- **Keep a fixed rectangular Gazebo fixture.** Arbitrary polygon support is unnecessary for the current experiment. The fixture still needs a verified coordinate transform and containment checks.
- **Keep 10 m as the nominal search altitude.** It comes from the mission document. The concerns are the 9 m acceptance threshold, altitude reference, and actual visibility.
- **Keep conservative fixed speeds for initial repeatable tests.** A sophisticated speed optimizer is premature; measured response and clearance are the immediate questions.
- **Keep HSV red segmentation as a baseline.** There is no evidence yet that a neural detector is necessary. Its detection limits and unsafe state interpretations need to be understood first.
- **Keep direct MAVLink/ArduPilot velocity control.** There is no demonstrated reason to migrate to ROS or replace the flight controller.
- **Keep Python/OpenCV until measured Pi performance justifies otherwise.** There is no established CPU or memory bottleneck.
- **Keep a latest-frame approach.** The absence of an application FIFO is useful; freshness and timestamps matter more than adding buffering.
- **Keep the GUI for interactive Gazebo debugging.** Treat it as a debugging dependency, not evidence of onboard compatibility.
- **Do not finalize FOV-dependent tuning before choosing and measuring the lens.** Provisional simulated optics are acceptable if clearly labeled.
- **Do not add a fixed turn radius merely because none exists.** Stop-and-yaw is a reasonable multirotor test behavior; braking, drift, and visibility during it require validation.
- **Do not generalize protocol constants indiscriminately.** NED conventions and supported image formats need explicit contracts; they are not all user-adjustable “magic numbers.”
- **Do not treat every missing full-mission feature as a defect in an intentionally isolated sweep test.** QR delivery, return, and payload handling can remain outside this file during focused testing—but this file’s completion message cannot serve as evidence that those tasks succeeded.
