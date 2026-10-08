# [competition] QR mission integration plan

Date: 2026-10-07
Status: design and repository inspection only. Implementation and Gazebo flights have not started in this pass.

This plan adds the initial QR reference scan and delivery-field QR inspection to the current full mission. The immediate deliverable ends at a verified, stationary 5 m hover over the matching field marker. Payload handling, the return corridor, and landing remain outside this change.

Paths below are relative to the mission repository root. This file and its Desktop copy contain the same plan. Competition names have been replaced with [competition].

## 1. Requirements reread and engineering decisions

I read the complete three-page mission appendix, section 4.2.4, including the image in Figure 3, and cross-checked the corresponding mission section in the local rulebook, pages 35–37.

The specified sequence is autonomous takeoff, a start QR scan from 5 m after moving approximately 1 m forward, banner alignment and corridor traversal, a field search from approximately 10 m, and descent to 5 m at the correct marker. Figure 3 depicts multiple field QR markers, one corresponding to the initial target, and a red area. Its layout is illustrative, not a source of exact marker coordinates or a payload schema.

For this implementation:

- Use the downward camera for QR candidate detection, centering, and decoding. Keep the front camera for the existing banner/corridor functions and testing views.
- Interpret “do not scan mid-flight” as no decoding or identity confirmation while translating: the aircraft remains airborne but must center over a particular marker and settle before reading it. Detecting a QR-shaped candidate during coverage is necessary and is allowed; that discovery is not a decoded identity.
- Use decoded text equality as the provisional matching rule. A start payload such as TARGET-A matches a field marker with exactly the same decoded payload. This also works for an identical URL encoded in both markers without accessing the URL.
- Keep the reference immutable for the flight. Never infer it from field markers or load an old reference silently after restarting.
- Pause coverage to inspect candidates one at a time. Confirmed nonmatches release the inspection and resume unfinished coverage.
- A confirmed matching marker takes priority over every unfinished coverage obligation. No remaining sweep, coverage-repair, or ordinary red-detour objective can prevent target descent.
- Keep the global mission manager and leased command service as the authority. QR modules supply observations or proposed movement, never independent MAVLink commands.
- “Abandon red-zone logic” means terminate the coverage/search objective and its detour planning. Flight health, geofence checks, known red clearance, and any active residence deadline continue to protect the aircraft during target centering and descent.
- Use the existing flat-ground and HOME-relative altitude contracts. In the full-world fixture, preserve its explicit ground-above-HOME and camera-offset settings rather than treating EKF-origin Z as HOME altitude.
- Keep GUI diagnostics asynchronous. Use classical QR detection/decoding, with no AI model, network request, or dense reconstruction.
- Preserve the agreed non-red dark-ground behavior. A failed QR read is unreadable target evidence, not a new black-frame camera-failure rule; missing/stale transport remains a separate fault.

The appendix describes delivery information, not a published encoding or guaranteed identical payload format. Exact matching is therefore an explicit provisional policy for the present simulation. The future organizer mapping belongs in one small payload-matching function, not in navigation.

## 2. Actual repository findings

| File / section | What exists | Decision |
|---|---|---|
| approach/autonomy/perception/qr_detector.py:4–103, QRDetector.detect | OpenCV detectAndDecode on every frame, followed by a large edge-density fallback that labels the biggest blob PARTIAL_LOCK. Debug images are copied and drawn every call. | Refactor this existing detector into separate candidate-location and stationary-decoding operations. Remove edge-density blobs as valid QR locks or identities. Make preview rendering optional. |
| approach/autonomy/behaviors/qr_tracker.py:15–96 | Downward image subscription, fixed pixel gains, body-frame velocity commands, GUI, no acquisition timestamps, repeated processing of the latest image. | Retain the centering intent and useful preview conventions. Do not run this standalone commander alongside the full mission. Replace its control, freshness, and frame-count assumptions with the integrated inspection task. |
| approach/autonomy/behaviors/mission_runner.py:40–122 | Old takeoff boundary, timed 1 m move, QR_CENTERING, counting more than 30 loop iterations, saving an image, timed backward/descent maneuvers. It never retains a verified reference payload. | Reuse no open-loop movement or standalone state ownership. Saving a centered photograph is not reference acquisition. Do not introduce its extra backward maneuver into the active mission. |
| world/automation_ki_baat_cheet/miss2_start.py:82–119, 212–258 | Another QR cluster/centering prototype. It takes off to 10 m, contrary to the initial-scan requirement, and names detectAndDecode outputs success/bbox/data even though the first is decoded text and the third is the rectified QR image. The fallback is a corner-density cluster. | Preserve as historical code; do not use it as the mission entry point. It supplies neither a dependable decoded identity nor a compatible startup sequence. |
| world/integration/experimental_corridor_manager.py:1609–1630 | Active startup immediately enters BANNER_SEARCH after settled takeoff. The default takeoff height is 3 m. | Insert the reference-acquisition states before banner search and use a 5 m takeoff profile when QR mission mode is enabled. Start the banner phase deadline only after the QR phase finishes. |
| world/integration/experimental_corridor_manager.py:2606–2640 | Calls run_coverage synchronously using the shared MAVLink connection and a command token; interprets a zero return code as overall COMPLETE. | Extend the post-corridor outcome contract to distinguish matching-target completion, no-target exhaustion, and faults. Preserve one telemetry reader and one movement owner. |
| coverage_mission/runtime.py:64–111, 138–211, 231–349 | TIMESYNC fitting, telemetry history, exposure/pose matching, independent map/planning process, bounded image history and queues. | Reuse these timing and process boundaries. Start downward acquisition/reference synchronization earlier and pass the same sensor source into coverage instead of duplicating subscriptions. |
| coverage_mission/engine.py:193 onward | Safety/progress bookkeeping and coverage selection are combined. Its nominal-altitude guard requires approximately 10 m; its COMPLETE means traversal/coverage finished. | Add an explicit paused-inspection mode and a separate target-descent mode. Never attempt descent by feeding 5–10 m poses into an unchanged sweep branch. |
| coverage_mission/geometry.py:69–96; coverage_mission/field_frame.py | Existing calibrated camera rays, attitude-aware flat-ground intersection, and field/local-NED transforms. | Reuse for physical QR location and centering; do not copy the old minus-error-Y/body-gain shortcut. |
| coverage_mission/planning.py:16 onward | Known-free ground, red inflation, geofence inset, checked connectors, and red-enclosed exclusion. | Reuse for reaching QR inspection points and rejecting inaccessible markers. Keep one ground map. |
| world/integration/command_service.py:44–79 | Revocable generation tokens and a short setpoint lease. | Revoke obsolete stage proposals during QR transitions and after a match. Validate generations on observations and worker results as well as commands. |
| world/integration/corridor_altitude.py:17 onward | Feedback-based EKF-origin altitude acquisition with dwell, sample freshness, and bounded timeouts. | Reuse the control pattern with a correctly converted HOME-relative 5 m target and horizontal target hold. Its hysteresis alone is not a new final-success tolerance. |
| world/integration/pi_camera.py | Bounded IMX296 acquisition with exposure/receipt timestamps; current requested processing image is 640 × 480. | Use the same observation contract. Camera model alone cannot establish real QR readability. |

### Confirmed decoder issue in this environment

The Python interpreter used for the read-only diagnostic reports OpenCV 4.5.4. A decode attempt prints “Library QUIRC is not linked. No decoding is performed.” That interpreter cannot prove QR payload matching, regardless of centering accuracy.

Implementation must first verify an actual decode in the exact mission environment. Use an appropriate QR-capable OpenCV build and a known-payload startup self-check. Version strings or the existence of QRCodeDetector are insufficient. If that build is unsuitable for the Pi environment, a compatible small decoder backend is the fallback; its behavior must satisfy the same test contract. Do not silently simulate successful payload decoding.

### Confirmed world-fixture issues

The current environment is an embedded GLB. It contains six QR-named planes and five embedded screenshot textures. Those textures are only about 65–80 pixels per side. At their native resolution the current detector finds a quadrilateral in just one of the five textures. The broken decoder prevents concluding which, if any, have valid payloads. The visual appearance of a QR print is not a payload test.

The GLB initial marker has approximately 2 m sides and a center at world Y = -31.73 m after the environment transform. The vehicle spawns at Y = -36.50 m facing approximately positive world Y, so the marker is about 4.77 m ahead. After a 1 m forward move, it remains roughly 3.77 m ahead and outside the nominal 5 m downward view.

At the current downward FOV, the shorter footprint dimension at roughly 5 m is only about 1.9 m. A 2 m marker can also be clipped even after centering. These are fixture/visibility mismatches to address before flight validation.

## 3. Matching and perception contract

### Provisional matching policy

Use an exact nonempty decoded UTF-8 text value. Preserve case, whitespace, punctuation, URL path, query string, and fragments. Do not trim or normalize values speculatively. Empty decode output is unreadable, not a nonmatch. Unsupported binary/non-text payloads are an explicit unsupported format, not automatic authorization to continue.

A URL is treated as payload text. Do not browse it, follow redirects, download content, or guess which URL component represents the target. Identical encoded URL text matches directly.

Do not compare photographed bitmaps or apparent marker colors. Different QR versions, error-correction levels, or masks can encode the same value, while lighting and perspective alter image pixels. Successful decoding already performs the relevant QR reconstruction and error correction. If the organizers later define different start/field schemas, replace only the payload-to-target mapping.

### Candidate detection before decoding

Use OpenCV QR quadrilateral detection, including multiple candidates when present, separately from decoding. The documented API supports distinct detect/detectMulti and decode operations ([OpenCV QRCodeDetector documentation](https://docs.opencv.org/4.x/de/dc3/classcv_1_1QRCodeDetector.html)).

Return an observation containing frame sequence, exposure timestamp, receipt timestamp, camera model/resolution identity, quadrilateral, center, clipping status, task generation and associated exposure pose. Payload is absent in discovery mode.

Require finite, nondegenerate, geometrically plausible quadrilaterals. A partially clipped marker can be a tentative discovery, but cannot pass stationary decode readiness until the selected marker and its needed border are visible. A random edge blob must never authorize marker centering or matching.

When perspective matters, use the quadrilateral's projective center, such as diagonal intersection, rather than assuming the arithmetic mean of image corners is the physical marker center. Project through the existing Projector using the matched exposure pose.

Keep visual candidate location separate from identity. Each physical marker has a spatial track ID; payload is attached only after an approved stationary read. Multiple markers with the same value remain separate physical tracks.

## 4. State sequence to build

| Mission portion | Proposed states | Exit condition |
|---|---|---|
| Initial reference | TAKEOFF_5M → START_QR_FORWARD → START_QR_CENTER → START_QR_SETTLE → START_QR_READ | A stationary, centered start marker yields the same nonempty payload on three distinct approved exposures. |
| Existing transition | BANNER_SEARCH → existing banner/pre-entry/corridor states → CORRIDOR_EXITED → existing field advance/climb | Preserve corridor geometry readiness and command authority; field entry additionally requires the flight's valid reference. |
| Field search | FIELD_SEARCH → QR_BRAKE → QR_APPROACH → QR_CENTER → QR_SETTLE → QR_READ | Inspect exactly one physical candidate at a time; full QR decoding is permitted only in QR_READ. |
| Nonmatch | QR_READ → QR_NONMATCH → FIELD_SEARCH | Three consistent reads identify a different payload; record the spatial candidate and resume unfinished coverage. |
| Unreadable / lost candidate | QR_READ or centering → bounded recovery/retry → FIELD_SEARCH or explicit fault | Preserve identity/attempt history; no indefinite hover or repeated revisits. |
| Match | QR_READ → TARGET_CONFIRMED → TARGET_DESCEND → TARGET_HOLD_5M | Drop coverage obligations immediately, descend with horizontal target hold, and verify settled 5 m hover. |
| Search exhausted | FIELD_SEARCH → bounded pending-candidate inspection → TARGET_NOT_FOUND or TARGET_UNRESOLVED | No reachable, uninspected candidate remains. Report the outcome honestly rather than declaring target success. |

### Initial reference acquisition

1. Confirm the downward source and real decoder capability before arming in QR mission mode.
2. Use the existing autonomous startup checks and settle at 5 m HOME-relative altitude.
3. Record the vehicle's current local position and yaw. Compute a 1 m forward waypoint from that pose; do not multiply a nominal speed by two wall seconds.
4. Travel to it using current pose, bounded velocity/acceleration, and a defined clear launch envelope. The delivery-field geofence is not the launch envelope: the start is outside that rectangle.
5. Discover and lock the initial marker, center over it, and maintain 5 m.
6. Verify stationary conditions before submitting any reference decode job.
7. Commit the confirmed reference once, then begin banner search with fresh stage timing and explicit authority.
8. If the initial code is missing, unreadable, ambiguous or sensors fail, stop and follow the existing startup abort/contingency behavior. Never enter the corridor without a valid reference.
9. Verify the change from 3 m startup to 5 m against banner acquisition and the existing staging descent. Do not simply add a timed 2 m descent or copy an old backward maneuver.

The start QR controller uses local-NED coordinates. It does not register the delivery rectangle prematurely or assume the launch position is the field origin.

### Field discovery and candidate memory

- Detect at a bounded rate while the existing route is followed. Discovery may trigger braking; it does not determine the target payload.
- Continue red mapping and the existing safety supervisor throughout braking, route diversion, centering, settling and decoding.
- Once stopped, associate the observation to a ground position using an admitted exposure pose. An image lacking that pose may justify a brief discovery hold, but cannot authorize an unverified route to a guessed position.
- Select a reachable candidate deterministically using the known-free map and a checked connector. Do not fly straight toward its image center across red or unknown ground.
- Track candidates by spatial position and uncertainty plus recent visual association, never by the largest blob or by payload alone.
- A reviewed candidate stores position, last exposure, payload/status, attempt count and deferred reason. Temporary loss is not a confirmed nonmatch.
- Repeated sightings of a confirmed nonmatch do not interrupt coverage again. Two nearby markers must not merge because a coarse spatial threshold is convenient; ambiguous association forces reacquisition.
- If a QR is in red, inside a confirmed red-enclosed region, or too close to the operational fence for safe centering, mark it inaccessible. The requirement to center over it does not justify entering that region.
- A candidate initially blocked by unknown ground can be deferred and reconsidered after new observations/connectivity. Confirmed inaccessible and confirmed nonmatching markers do not receive endless retries.
- Preserve the existing traversed and pending route ledger across detours. Do not mark an interrupted lane complete or restart the entire field.

### Centering and stationary reads

Use the ground-projected marker center and current aircraft position to generate a capped planar correction through the existing frame transformations and motion limits. This automatically accounts for field orientation and yaw. Only issue movement based on fresh candidate tracking and valid localization.

Keep the same physical candidate locked while several markers share the image. Select and decode that marker's quadrilateral/crop; never accept an arbitrary first entry from a multi-code decoder.

Require all of the following for a continuous settle dwell:

- Marker and required border are in view and remain spatially consistent.
- Ground-center error is within the configured centering tolerance.
- Measured horizontal and vertical speeds are below their limits.
- Altitude is within the stage envelope.
- Attitude, clocks, localization and camera are current, and exposure/pose matching is valid.
- No red incursion or incompatible safety task is active.

Only exposures acquired after that dwell completes may support a read. Repeated processing of the same frame cannot supply extra confirmations. Drift, track loss, conflicting payloads, or task changes reset the confirmation sequence.

Capture admission is based on the frame's exposure/receipt freshness. A decode result has a separate bounded processing age: it can be accepted only for the same inspection generation and locked marker while the aircraft has remained settled. A delayed decode cannot authorize a movement proposal or revive a cancelled task. Do not force payload computation to masquerade as a fresh flight-control observation.

A nonempty consistent decode on three distinct approved exposures confirms the payload. Empty or conflicting results consume a bounded read attempt. No fallback label such as PARTIAL_LOCK can become a reference, nonmatch, or match.

### Match and final descent

On a confirmed match:

1. Latch the matched physical location and reference-comparison evidence.
2. Cancel pending coverage route proposals, ordinary red-detour objectives, scan requests for other markers and deferred candidate work. Revoke the old task generation.
3. Keep the sole telemetry receiver and mission command service active. Transition the post-corridor supervisor into target descent rather than returning into a manager with stale cached telemetry.
4. Revalidate horizontal centering and the clear descent footprint. Maintain target-relative horizontal corrections and descend with feedback to 5 m HOME-relative altitude.
5. Freeze all sweep credit/completion requirements. Use altitude-aware ground projection during the descent; the existing 10 m sweep-altitude guard must no longer govern this stage.
6. Keep camera/pose validity and retained ground clearance checks active as the field of view shrinks. If the target is lost or drift exceeds the limit, pause descent and use a bounded reacquisition at the current height. Never guess that continued vertical movement is safe.
7. Unexpected restricted-ground evidence or an active residence deadline interrupts descent under the existing safety rules. A target match does not erase an ongoing violation timer.
8. Require 5 m altitude, target-center tolerance and low measured speeds continuously for the terminal dwell before TARGET_HOLD_5M is successful.
9. End the simulation mission segment there. Keep a terminal target hold while the test is observed; the harness can stop the owned simulation afterward. No payload, landing, return or renewed field search follows.

Merely sending a 5 m setpoint, reaching it momentarily, or shutting down after a final zero-velocity packet is not evidence of a maintained target hover.

## 5. Implementation structure and ownership

Keep the existing full manager, CommandService, ground map and coverage planner. Add a small shared QR inspection task and an isolated perception worker. Do not run the old QR scripts beside the active manager.

| Owner | Responsibility |
|---|---|
| Global mission manager / active runtime supervisor | Flight session reference, mission/task state, safety precedence, stage generation, authority, telemetry reception and acceptance of read results. |
| Existing map/navigation worker | Single red/ground map, candidate reachability, checked routes, paused coverage ledger and proposed QR approach/centering movement. |
| QR perception worker | Rate-limited detection and approved decode requests; outputs observations and payloads only. It owns no MAVLink connection and no mission transition. |
| Shared QR inspection core | Pure candidate association, settle/read evidence, retry status, payload comparison and task decisions, reused at startup and in the field. |
| CommandService | Sole movement transmission, lease expiration and rejection of revoked-stage proposals. |
| Preview / asynchronous logging | Lossy GUI and bounded diagnostics, independent of command and sensor deadlines. |

The map worker must still process fresh red observations and safety bookkeeping while QR inspection is active. Refactor only the part of Engine.step that conflates safety/progress with sweep destination selection; inspection pauses that selection, not mapping or authority checks. During intentional inspection, suspend the ordinary coverage no-progress test and enforce an inspection-specific bounded deadline instead. Do not repeatedly reset a no-progress timer to hide a stuck inspection.

Use the existing exposure/pose-matching helpers from startup onward. Pass a shared downward sensor source into coverage through sensors_override. One caller at a time drains MAVLink; the QR worker never calls recv_match.

Keep the existing integer entry-point behavior compatible where needed, but provide a structured post-corridor outcome carrying reason, matched target, final pose and completion kind. Full field coverage without a match must not return the same target-success outcome as TARGET_HOLD_5M.

Use a QR-enabled full-mission profile and retain an explicit coverage-only regression/test mode. Production QR mission mode refuses a missing reference; only a named isolated test harness may inject one.

## 6. Pi-conscious performance design

- Begin with the existing 640 × 480 processing contract. Request ISP-scaled images through the camera adapter; do not resize and copy both native streams repeatedly in Python.
- Run candidate detection on fresh sequences at a provisional 5 source-Hz, with a wall-rate cap under accelerated simulation.
- Decode only the selected marker in the read state, using a crop with adequate border and bounded work. Begin at at most 2 decode jobs per source second.
- Use one separate QR process with one pending job and one latest result. Share bounded recent camera buffers where practical; avoid adding an unbounded full-frame queue.
- Reuse grayscale conversion within an admitted QR job. Avoid full-resolution edge maps, the old 25 × 25 morphology fallback, and exhaustive image-transform retries.
- Apply only a small bounded set of stationary decode variants if baseline decoding fails. Image enlargement is not new optical detail.
- Measure detection time, decode time, capture-to-result age, worker queue replacements, worker restarts, command send intervals and settle evidence. Keep logging/GUI outside flight-critical timing.
- A QR worker hang or native decoder crash expires its result. The supervisor holds and applies a bounded recovery/fault policy while red/pose supervision continues. Catching cv2.error alone cannot recover a native process crash.
- Limit worker recovery to one controlled restart per failed inspection, then end that attempt or fault the phase. No infinite decode or restart loop.
- Do not claim Pi throughput from this desktop. Extend the existing dual-camera benchmark with the approved QR workload when implementing; include simultaneous LiDAR, FC communication, map/planning and optional GUI during later Pi measurements.

At a 28.2-degree horizontal FOV and width 640, fx is approximately 1274 pixels. A 1 m marker viewed near 10 m has about 127 pixels across; a version-1 QR plus four-module quiet borders occupies 29 modules, giving roughly 4.4 pixels per module before rendering/noise losses. That is a fixture-design calculation, not a guaranteed decoding threshold.

Use the same calculation with actual marker dimensions, QR version, lens/crop and altitude. Unknown competition marker size and payload length remain real optical constraints. Higher-resolution capture may help if measured module detail is inadequate, but must preserve calibration and be benchmarked. Do not descend below the required search height to read an unknown field code before confirming the target, or pretend that upsampling fixes an undersampled marker.

## 7. Provisional settings and what remains to measure

These are conservative starting values to validate in Gazebo, not final hardware tuning. Keep them in explicit QR configuration.

| Setting | Proposed starting point | Reason / later evidence |
|---|---|---|
| Initial scan altitude | 5 m HOME-relative | Mission requirement; incorporate the fixture ground/HOME contract. |
| Initial forward travel | 1 m measured from settled pose/yaw | Mission requirement; feedback movement rather than a fixed-duration command. |
| Field scan altitude | 10 m HOME-relative | Existing mission coverage profile. |
| Final target altitude | 5 m HOME-relative | Requested terminal boundary. |
| QR approach/centering speed cap | 0.25 m/s | Provisional; constrained by stopping distance and observed free ground. |
| Center error | 0.15 m | Provisional target-relative tolerance; verify achievable tracking and geometry. |
| Settled horizontal/vertical speed | Each at most 0.10 m/s | Measured telemetry, not zero requested velocity. |
| Pre-read settle dwell | 1 source second | Prevent decoding during braking or residual drift. |
| Consistent payload observations | 3 distinct approved exposures | Avoid one-frame decisions and repeated-frame counts. |
| Candidate detector | At most 5 source-Hz | Discovery cost control; measure miss rate and Pi timing. |
| Stationary decode | At most 2 jobs/source second | Cost bounded to the selected target. |
| Read attempt deadline | 8 source seconds | Provisional finite wait; no dwell reset can extend it forever. |
| Attempts per field candidate | 2 total read/inspection attempts | Retry transient failure once, then retain unresolved status. |
| Whole initial reference phase | 45 source seconds after settled takeoff | Bounded movement, centering and decoding; hardware values remain provisional. |
| Final descent speed cap | 0.30 m/s, reduced near target | Reuse feedback descent pattern; verify actual vertical dynamics. |
| Final altitude tolerance / dwell | 0.15 m for 2 source seconds | Align with the existing climb tolerance and verified stationary terminal behavior. |

Retain existing pose-gap, frame-age, geofence-clearance, command-lease and red-residence contracts rather than loosening them just to obtain QR success. Add separate explicit wall watchdogs for camera/worker failure and a stalled simulator. Phase deadlines must not reset whenever a candidate is rediscovered or a worker retries.

Things requiring hardware/site data: final lens/crop/intrinsics, print dimensions and density, exposure/lighting and color contrast, QR readability, centering noise and controller tuning, camera latency, FC clock alignment, and the concurrent Pi performance budget.

Things requiring competition clarification: payload schema, whether start/field payloads are identical or contain different delivery fields, raw binary/character encoding, number and dimensions of markers, duplicate targets, and marker placement relative to red zones. These do not prevent the stated provisional text-equality simulation.

## 8. Gazebo fixture changes needed during implementation

Do not rewrite the existing corridor environment to add QR behavior.

- Create deterministic, genuinely decodable markers with explicit payloads: one start reference, at least two nonmatching field markers, and one reachable matching marker.
- Include a second fixture with an identical URL encoded at start and target to prove that no network access is required.
- Replace or conceal the old screenshot QR visuals for these test fixtures so false candidates do not distract from the intended scenario. Preserve terrain and corridor collision geometry.
- Place the initial marker about 1 m forward of the takeoff pose, with its full print visible after the specified move. Begin with a declared 1 m square including a quiet zone; this is a simulation fixture, not an inferred competition dimension.
- Use sufficiently detailed source textures, correct UV orientation, no z-fighting, and visible borders. Validate each source texture by decoding it before rendering.
- Verify a rendered overhead view at both 5 m and 10 m through the actual mission camera, not just the texture's source image.
- Place a confirmed nonmatch along the initial field search before an early reachable target so one integration flight can exercise rejection, resumption and early completion without traversing all 40 × 30 m.
- Put a separate candidate inside the large red region for exclusion tests. No truth/model identifier or configured marker coordinates may be used by the mission's detector.
- Keep fixture target identities/positions in an independent evaluator, separate from controller input.

The actual installed decoder must successfully decode known fixtures before any flight result is called a QR pass.

## 9. Implementation order

1. Verify decoder capability in the intended mission interpreter and create valid QR assets/identity fixtures.
2. Refactor the existing QRDetector into detect-only observations and explicitly authorized stationary decoding. Add structured results and optional debug rendering.
3. Build the pure inspection/identity controller and candidate ledger; exercise its state and timing rules offline.
4. Move shared downward acquisition, pose history and synchronization to the startup boundary without duplicating the MAVLink reader.
5. Add the 5 m initial reference sequence and bounded measured 1 m advance; verify the subsequent banner/corridor handoff.
6. Integrate rate-limited QR discovery into the post-corridor runtime and QR diversion into its single map/navigation owner.
7. Add stable nonmatch resumption, inaccessible/unreadable handling, and truthful search-exhaustion outcomes.
8. Add atomic target acceptance, cancellation of obsolete work, altitude-aware descent and verified terminal hover.
9. Update GUI overlays, mission/QR timing metrics, result schemas, handoffs and the independent test evaluator.
10. Run the focused validation sequence below. Extend the Pi benchmark and record the remaining hardware measurements.

No automatic repository push or deployment is part of this planning task.

## 10. Validation strategy and acceptance criteria

### First: offline and replay checks

Use focused tests before any long flight:

- Decoder self-check with known text, identical URLs, mismatches, empty output and explicitly unsupported data.
- Same payload encoded with different QR masks/versions still matches after decoding; similar images with different payloads do not.
- Candidate detection produces geometry without issuing a decode request while the aircraft moves.
- Multiple markers retain distinct spatial identities; a delayed observation cannot switch the selected physical marker.
- Repeated/stale images cannot satisfy dwell or three-read confirmation. Exposures before settle completion are ineligible.
- Drift, conflicting reads, clipping, camera loss and localization loss interrupt readiness deterministically.
- Projection and centering signs are correct at rotated field orientations and vehicle yaw, including a nonzero HOME/EKF offset.
- Confirmed nonmatches do not cause repeated stops; unreadable/temporarily inaccessible markers obey finite attempts and deferred status.
- A candidate inside the large red zone cannot authorize entry, centering or descent.
- QR inspection does not manufacture sweep credit or trigger the ordinary coverage no-progress timer. Resumption keeps remaining obligations.
- A confirmed match cancels every pending sweep/repair and competing QR task; revoked results and command tokens cannot restore them.
- Intentional descent can progress through 10–5 m without the sweep altitude gate holding it at 10 m.
- Sensor/worker failure during descent produces hold or the documented contingency; it cannot claim 5 m success.
- Complete coverage without a match yields TARGET_NOT_FOUND or TARGET_UNRESOLVED, never target completion.
- Decoder delay/crash, preview freeze and logging backpressure do not interrupt the command watchdog.
- Re-run the existing corridor/coverage regression tests appropriate to the integration changes.

Do not use the old coverage evaluator unchanged for early target completion: unseen field and unfinished coverage are expected after a correct match. Keep its safety checks; add explicit target-identity, centering and descent evidence.

### Then: three short targeted Gazebo scenarios

| Scenario | What it must demonstrate |
|---|---|
| Initial reference and missing-reference variant | Takeoff at 5 m, measured 1 m advance, center/hover/read, immutable reference and safe bounded failure when the reference cannot be read. Stop before a long corridor/field flight. |
| Field inspection with early match | From an explicit isolated airborne test boundary at 10 m: inspect a nonmatch, resume, safely reach a matching marker, cancel unfinished coverage, descend and settle at 5 m. |
| Field safety/failure variant | Include a red-excluded marker and a lost/stale QR observation or stalled decoder; verify no unsafe detour and finite recovery/failure. Reuse short fixtures and saved-map replay for combinatorial cases. |

After these pass, run one integrated mission from autonomous takeoff through the existing corridor to the early field target and 5 m hold. One clean flight is sufficient for the current Gazebo acceptance checkpoint; collect additional repetitions only if a failure or unexplained timing result warrants them. A complete field sweep is not required to validate early target success.

### Evidence required for the integrated pass

- Actual source/rendered QR assets decode successfully in the mission environment.
- The initial reference is obtained at settled 5 m after a feedback-based approximately 1 m move.
- Initial identity is unchanged across corridor and field handoffs.
- Every accepted field payload is associated with its selected marker and fresh post-settle exposures; decoding is never accepted during translation.
- At least one confirmed nonmatch is rejected and unfinished coverage resumes.
- The final marker's payload matches the reference, independently of the controller's selected ID or world-model names.
- Search/coverage commands cease on confirmation even though sweep points remain.
- Independent Gazebo truth shows no sampled red or fence violations and verifies horizontal alignment, descent to 5 m and low final velocities over the terminal dwell. Report truth sampling gaps and retain the existing safety margins.
- The final result is TARGET_HOLD_5M with the matched location, reference identity/hash, confirmation exposures, altitude/centering/velocity evidence and transition log.
- No payload action, return, landing or renewed sweep follows.
- GUI/logging failures and a delayed QR worker cannot produce stale movement authority.
- Any retained exception is stated explicitly; a QR success must not be inferred merely from a photograph or a COMPLETE string.

## 11. Boundaries preserved

The current Gazebo coverage pass is an existing baseline, not proof of the new QR stages. Keep its regression mode and review history. The existing Pi camera adapter is preparation, not measured Pi flight performance.

This plan deliberately adds no AI, online URL resolution, speculative organizer schema, real payload mechanism, return mission, battery/time optimization, dynamic geofence feed or final lens selection. It changes no mission source code in this pass.

## Implementation follow-up — 2026-10-08

The implementation and the GUI-enabled early-target Gazebo checkpoint are now recorded in `docs/qr/mission_status.md`, with measured independent safety/centering/descent evidence, retained failed development runs and remaining hardware/organizer work. The plan above remains the historical planning record. The original collision environment was preserved while all six coloured screenshot QR visuals were replaced by valid black-and-white fixtures.
