# Return Mission Implementation Plan

Date: 2026-10-08  
Status: design only; no mission source changes made for this task.

## 1. Recommended behaviour

Extend the current full mission with:

Matching QR confirmed → descend to 5 m → remain settled for 5 continuous seconds → ascend to the field transit altitude of 10 m → route toward the orange corridor entrance while avoiding red zones → acquire and centre the orange banner → establish safe corridor-entry height and positive LiDAR wall readiness → run a fresh instance of the existing corridor FSM → establish that the entire aircraft has cleared the far end → land at a verified exterior landing location → confirm touchdown and disarming.

The five-second hold is the current delivery surrogate. This milestone does not add a payload-release actuator or pretend delivery has been verified.

The mission PDF specifies a post-delivery ascent to 10 m, a green return banner and landing at the original takeoff point. For this milestone, the user's explicit orange-banner and “anywhere outside the corridor” instructions override those last two requirements. The planned implementation is therefore not a claim of complete competition compliance.

The return must start only from a confirmed target match and a successfully completed delivery-height hold. Coverage completion without a target, unreadable targets, an aborted descent or a safety fault must not start the return.

## 2. Evidence from the current implementation

| Location | Current behaviour | Consequence for the return |
| --- | --- | --- |
| `coverage_mission/qr.py: QRConfig.final_dwell` | Default dwell is 2 seconds, despite the state name `TARGET_HOLD_5M`; “5M” means altitude, not duration. | Set the full-return profile to a measured 5-second settled dwell. Do not add a blind sleep. |
| `coverage_mission/qr.py: Inspection.step` | Target identity is latched, coverage is cancelled, horizontal centring is maintained during descent, and dwell resets when measured settling conditions fail. | Keep these mechanisms and make the successful hold an explicit return-start event. |
| `coverage_mission/runtime.py: run` | `TARGET_HOLD_5M` currently ends the session; final cleanup terminates workers. | Keep the live ground map and safety session through field return instead of exiting and rebuilding them. |
| `world/integration/experimental_corridor_manager.py: COVERAGE branch` | A successful field runtime currently leads to `COMPLETE`. | Add explicit target-hold → return dispatch; generic success must not imply permission to return. |
| `world/integration/entrance_readiness.py: StagingEnvelope` | Fixed outbound bounds: N [-31.5,-29], E [-6,-2], HOME altitude [1.5,5] m. | These bounds cannot admit the orange entrance. Use separate outbound and return approach profiles. |
| `experimental_corridor_manager.py: CORRIDOR_EXITED branch` | Every corridor exit currently advances into the field and starts ascent/coverage. | Dispatch by traversal role: outbound exit enters the field; return exit proceeds to exterior landing. |
| `corridor/native/mission_runner.py` | Existing runner owns entry, cruise, obstacle handling, reassessment and exit controllers. | Reuse the runner implementation with a fresh return instance, rather than copying an entire FSM. |
| `corridor/native/controllers/exit_detection.py: projected_travel` | Exit progress is actual displacement projected onto the latched start heading, not always northward displacement. | This mechanism supports southbound return; reverse obstacle observations still need testing. |
| `coverage_mission/engine.py: Engine.step` | Field inset, observed-free map, red residence, pose continuity, freshness and stopping-region checks precede inspection commands. | Keep these protections for return routing and vertical phases; do not restart sweep scheduling. |
| `coverage_mission/planning.py: route` | Routes use reachable observed-free cells and can choose checked frontier viewpoints when the goal is not yet reachable; planning has a wall-time budget. | Useful return-routing foundation, but the coverage-oriented frontier ranking and arrival-disk substitution need return-specific acceptance rules. |
| `experimental_corridor_manager.py: request_land / confirm_terminal_land` | Normal LAND request stops the velocity service; confirmation checks fresh LAND mode, not touchdown. | Reuse the normal command path, then add a separate touchdown/disarm monitor. |
| `experimental_corridor_manager.py: drain_mavlink` | Leaving GUIDED or becoming disarmed revokes mission authority. | Explicitly distinguish authorized LAND/touchdown from pilot takeover without allowing further velocity commands. |
| `world/worlds/miss2_full_world.sdf: orange_banner` | Orange visual already exists at world (4.1,-20,3.548), with the green banner's dimensions and orientation. | Use it as the return fixture; its placement has XML checks, not return-flight validation. |

The native corridor runner's relevant states include `PRE_ENTRY_GEOMETRY_LOCK`, `ENTER_CORRIDOR`, `CORRIDOR_CRUISE`, `OBSTACLE_DECISION`, `HOVER_AND_REASSESS`, `EXIT_DETECTION` and `CORRIDOR_EXITED`.

## 3. Ownership and reuse decision

Keep the global mission manager as the owner of overall state, safety escalation and command authority. Corridor and field runtimes remain subordinate proposal producers.

Reuse implementations through shared components and configuration. Copying mutable runtime instances, whole command loops or a second mission manager would create avoidable ownership and maintenance problems.

### Field session

- Keep the current map/engine worker alive from coverage through target hold and field return.
- Add explicit field-session modes: search/inspection, delivery hold, return ascent, return routing and return approach.
- Replace the terminal target-hold break with a phase event for full-return runs. Retain an explicit QR-only test endpoint for existing regression tests.
- Give the manager an incremental session pump or equivalent supervisor hook so it continues to own phase transitions, fault decisions and leases. Do not bury the new return behind an independently commanding, blocking mission loop.
- Retain one MAVLink receive owner at any instant. Share already decoded telemetry with subordinate controllers; never let two consumers compete for the same connection.
- Continue existing source-clock alignment, camera-pose history, localization continuity checks and live downward observations throughout the field phases.
- Exchange small decisions/events across process boundaries. Do not stream the entire grid back to the manager every control tick.
- Once the corridor takes over, stop field planning and release unnecessary worker resources through an explicit, zero-command handoff.

### Corridor session

- Instantiate a fresh native runner, entrance-readiness estimator, banner tracker, phase guards and altitude supervisor for the return lap.
- Preserve the underlying scan adapter, LiDAR mounting convention and body-command conversion. Facing the opposite direction changes which physical features are on the vehicle's left/right; it does not justify another sign inversion.
- Configure approach geometry and expected travel bearing through a return profile.
- Preserve outbound behaviour through its existing profile and regression tests.
- Add a traversal-role marker so the second `CORRIDOR_EXITED` cannot restart coverage.

### Authority transfer

Each handoff must revoke the old proposal token, command zero, validate fresh state and grant a new token only to the next active runtime. Old sweep, QR-centering or corridor commands must never become valid again.

A successful QR match remains latched for the remainder of the flight. Shut down QR decoding after the match; do not rescan field targets, repeat the initial QR, arm again or repeat takeoff.

## 4. State sequence and acceptance gates

Names below are proposed manager-level phases, not a demand to duplicate the native FSM.

| Phase | Behaviour | Required transition evidence |
| --- | --- | --- |
| `DELIVERY_HOLD` | Remain centred over the matched target at 5 m, with all field safety checks active. | Five continuous source-time seconds within the existing configured altitude, position and measured-velocity limits; fresh telemetry and safety evidence throughout. |
| `RETURN_ASCEND` | Hold horizontal position and climb to 10 m. | Measured altitude and low vertical speed settle; current location and stopping region remain permissible. |
| `RETURN_TO_ENTRY_REGION` | Follow a checked route toward an interior stand-off region facing the orange entrance. | Reach a safe, observed stand-off region with braking room; never use the banner coordinate itself as a field waypoint. |
| `RETURN_BANNER_SEARCH` | Establish the configured acquisition height at a clear stand-off, then use bounded yaw search and, if necessary, checked viewpoint changes. | Orange detection persists over distinct fresh exposures, within the expected entrance region; no clipped/unreliable target accepted for approach. |
| `RETURN_BANNER_CENTER` | Reuse camera centring and tracking with an orange colour profile. | Stable image alignment and low measured motion; recent tracked banner remains valid. |
| `RETURN_APPROACH_AND_STAGE` | Approach within a bounded entrance envelope, establish the proven corridor-entry height, and settle. | Supported approach geometry, fresh pose/attitude/scan, safe front range, roof/airframe clearance and valid staging bounds. A close-range stop alone is not arrival. |
| `RETURN_ENTRY_READY` | Require repeated positive LiDAR two-wall readiness before granting native control. | At least the existing required number of distinct fresh supported scans; correct entry envelope and height. |
| `RETURN_CORRIDOR` | Fresh native runner executes its existing entry/cruise/obstacle/reassessment/exit FSM. | Native `CORRIDOR_EXITED` with valid measured progress and no unresolved fault. |
| `RETURN_EGRESS` | If needed, move a bounded additional distance along the measured exit bearing to clear the full airframe and roof footprint. | Exterior landing-envelope position, adequate geometric clearance and measured settle. |
| `LAND_REQUESTED` | Revoke velocity authority and request normal FC LAND. | Fresh mode feedback confirms LAND; accepted command ACK alone is insufficient. |
| `LANDING` | Monitor FC landing; do not send competing velocity commands. | Fresh on-ground evidence and disarmed heartbeat, with consistent height/velocity evidence. |
| `COMPLETE` | Persist terminal outcome and clean up owned resources. | Confirmed landing, not merely LAND mode or low altitude. |

Five seconds means a continuous measured dwell after settling, not five seconds of host sleep or a timer started during descent. Loss of settling resets the dwell; loss of valid evidence invalidates completion. A separate bounded phase deadline prevents endless repeated settling.

The PDF's 10 m return transit is retained. Banner acquisition need not occur at 10 m: the banner is near 3.55 m and a horizontal front camera may not see it from a close overhead position. Route to a safe stand-off first, then establish a configurable acquisition height, initially reusing the outbound 5 m acquisition profile where Gazebo supports it. Determine the stand-off and staging sequence from actual fixture visibility and roof clearance before coding fixed flight commands.

## 5. Red-zone-safe return routing

Use the existing ground map, projector, inflation and checked route follower. Return is a goal-directed task, not another coverage sweep.

- Retain all accumulated red evidence, uncertainty margins and observations; match confirmation does not erase hazards.
- Continue detecting red and updating the map during hold, ascent and field transit using timestamp-matched pose and actual altitude.
- Keep first-hit blocking and the existing distinct-frame confirmation rule.
- Keep the existing rule that received valid non-red dark/black pixels are permissible ground evidence. Do not reintroduce black-image-content failure logic; missing/stale transport still fails freshness checks.
- Maintain the same red-residence clock across return phase transitions: warn after 5 seconds inside, exit before 10 seconds. A new state or command token must not reset continuous residence.
- Safety escape and braking take precedence over the nominal return goal.
- Replan when new red evidence invalidates the current route. Check stopping room against measured velocity as well as the requested command.
- If the entry stand-off is unobserved, move only to checked frontier viewpoints that expose ground toward the return goal. Do not mark unknown cells free, follow an unchecked straight line or run the whole-field sweep again.
- Adapt the existing frontier scoring toward return progress. Its large-field coverage ranking prefers local observation gain; that alone is not a guarantee of timely progress toward an entrance.
- Track meaningful progress, frontier visits and bounded planning retries. An unreachable entrance produces an explicit blocked/abort outcome, not a loop of repeated identical replans.
- Planner budget exhaustion is different from proven absence of a route. Both command zero while unresolved; a bounded supervisor decides escalation.
- Near entry, arrival is acceptance of a safe stand-off region followed by positive banner/geometry gates. Do not inherit coverage waypoint “arrival disk” substitution as permission to enter a corridor.
- Ascent/descent do not simply set a permanent altitude-bypass flag. Each phase has a target, acceptable envelope, measured progress and deadline.

The initial implementation can keep the current grid resolution and conservative clearance settings. Return does not need a second map, a global optimization solver, dense reconstruction or an AI model.

## 6. Entrance geometry and field-fence transition

This is the main integration risk.

The field's operational inset ends before the corridor mouth. The orange entrance is on that boundary, and the return corridor and exterior landing location lie outside the field grid. Sending a waypoint to the banner while applying the field-only fence will correctly fail; globally disabling the fence would be unsafe.

Use explicit mission-region profiles:

1. Field region: existing registered geofence, inset and live red-map authority.
2. Entrance transition: narrow, bounded approach/staging region around the selected corridor mouth, with appropriate altitude and heading constraints.
3. Corridor region: native LiDAR/pose/altitude supervision.
4. Exterior egress/landing region: bounded clear test-site region beyond the return exit.

These regions must also fit within the overall authorized mission operating area. A phase change cannot authorize departure from a real whole-mission geofence.

An entrance permit is granted only after fresh localization, a correctly associated orange banner, a valid approach profile and checked field-side approach/vertical clearance. Crossing from the field inset into this narrow approach region is an explicit transition, not a silent map exception. Positive LiDAR wall readiness is still required before native entry motion.

LiDAR wall readiness cannot be required at 10 m if the corridor walls are below its scan plane. Establish the safe exterior staging height first, while remaining outside the roof footprint, then evaluate the wall geometry.

Use corridor-local axes for approach bounds and egress distance. Transform them through the registered FC local frame; do not assume the field is north-aligned or reuse the outbound rectangle for the orange entrance.

For the current Gazebo fixture:

- Orange banner: world X ≈ 4.1 m, Y = -20 m, Z = 3.548 m.
- Return corridor runs toward decreasing world Y; local NED heading is approximately π radians.
- The far mouth is around world Y = -30.1 m.
- The field-side stand-off must be inside the safe field inset, not directly under the banner.
- Outbound entry/exit distances of 0.75/1.20 m remain starting regression values, not proof that the return landing footprint clears the roof.

Keep fixture coordinates in replaceable configuration. Do not couple navigation to Gazebo model-name lookups or derive the real corridor entrance from geofence corners alone. A field boundary does not identify a corridor mouth.

The current orange fixture may overlap the red detector's low-hue threshold. Also, the raised banner is not on the assumed ground plane. Test the actual downward view and projected evidence during approach. Do not globally exempt orange-looking ground from red-zone safety to solve a fixture/transition issue.

## 7. Banner and corridor reuse

Reuse `HybridBannerDetector` with a separate configurable orange HSV profile and reset its track on entering the return phase. The current parameter names mention green, but the detector already accepts colour bounds; a second detector implementation is unnecessary.

Use the same resolution-normalized area/shape checks, clipped-target rejection, track association and bounded servo behaviour. Orange thresholds and acquisition gains are provisional until camera/site testing.

Search with yaw from a checked hover first. Any translation requires a safe mapped route and stopping room. Do not copy the outbound “move left” search into the field without those checks.

Loss of detection must command hold/reacquisition within a finite deadline; it must not count as arrival. Keep the provisional approximately 0.50 m approach range as a hard stop only until the return geometry test supports a better value.

The specialized static rectangular side-obstacle model remains in scope. Reverse traversal exposes obstacle corners in a different order, so test both left and right protrusions from the return side. Unsupported geometry must fail conservatively, not be inferred clear.

Keep HOME-relative altitude and the existing flat-ground datum convention. Make acquisition, staging and corridor heights explicit profile values, with the existing physical airframe/clearance assumptions visible. Do not use “10 feet” as permission to fly at the roof height without clearance checks.

## 8. Landing decision

“Anywhere outside the corridor” means an accepted exterior landing region, not an arbitrary position immediately after the exit state.

For Gazebo, choose a deterministic, flat, clear landing location ahead of the orange corridor's outward mouth, along the measured exit bearing. Use the existing scene geometry to define the bounded test region; stop and settle there before LAND.

Ensure the entire vehicle footprint is beyond the roof/walls with the configured uncertainty margin. Native exit progress alone does not certify a landing site.

Reuse normal FC LAND and existing bounded ACK/mode checks. Add fresh on-ground and disarmed confirmation. Handle expected LAND/disarm as terminal mission transitions while keeping pilot takeover authoritative.

Do not force-disarm in the air, continue issuing GUIDED velocities during FC LAND, or claim completion from an accepted ACK.

With a 2D LiDAR and colour cameras, this implementation cannot certify arbitrary ground suitability. Physical testing requires a surveyed clear, flat exterior landing region. Automated terrain/landing-site classification is not part of this milestone.

## 9. Fault handling and liveness

Every new motion phase needs sensor requirements, an absolute deadline, a measured progress requirement and an explicit safe failure outcome.

- Stale pose, attitude, clock alignment, camera data or worker decisions cannot authorize motion.
- Camera loss during field motion commands a bounded hold; no indefinite banner search is permitted.
- Missing/unsupported LiDAR at entry or inside the corridor uses the existing conservative hold/reassessment/abort contract.
- An origin reset, pose discontinuity or pilot mode takeover invalidates active proposals immediately.
- A blocked route, lost banner, failed climb/descent or failed readiness gate does not fall back to unchecked travel.
- Keep red escape priority while localization/control remain usable. Do not choose a blind nominal LAND over a red zone or beneath a roof as the ordinary recoverable-fault response.
- Severe link/localization failures still depend on configured FC failsafes and pilot intervention. Software cannot guarantee map-safe escape without valid state/control.
- Landing mode rejection or lack of touchdown confirmation reports a landing failure, not mission completion.
- Phase deadlines use flight/source time for measured progress and independent monotonic wall watchdogs for stalled processes/transport. Accelerated Gazebo must not corrupt either.

Maintain GUI and black-box diagnostics asynchronously; neither can extend a command lease. Persist phase, traversal role, active authority generation, route/progress, source ages, worker timing and landing evidence.

## 10. Pi-conscious implementation choices

- One live ground map and one acquisition path per camera; no second return camera pipeline.
- Reuse latest-frame/bounded-buffer acquisition and existing timestamp history.
- Stop QR decoding once matched. Only run front-banner processing when it is needed; keep acquisition health available.
- Rate-limit route planning and replan on meaningful map/goal changes, with the current wall-time budget and motion watchdog intact.
- Cache map-derived reachability/clearance data by map revision where useful; do not weaken hazard-update responsiveness for speed.
- Keep large arrays inside the map worker. Send compact decisions, health and phase events.
- Release inactive field planning/preview resources when corridor supervision takes over; do not run both planners continuously.
- Continue asynchronous bounded logging and preview. Record drops and timing percentiles.
- No neural detector, SLAM subsystem, dense point cloud, high-resolution mandatory processing or extra nested mission process is needed.

Rates, camera exposure, calibrated FOV, exact entry gains, thermal limits and worst-case planner latency remain provisional until Pi measurements. Gazebo success cannot establish Pi timing margins.

## 11. Implementation order and files

| Order | Work | Primary locations |
| --- | --- | --- |
| 1 | Add return configuration, traversal role, phase contracts and offline tests before motion changes. | `world/integration/experimental_corridor_manager.py`, new return configuration/module as needed, `config/` |
| 2 | Make the 5-second hold a supervised event; retain the live map session; suppress QR/sweep work after match. | `coverage_mission/qr.py`, `coverage_mission/runtime.py`, `coverage_mission/engine.py` |
| 3 | Add bounded goal-directed return routing, ascent/stand-off phases and manager-owned authority transitions. | Shared field runtime, `coverage_mission/planning.py`, `coverage_mission/geometry.py` only where required |
| 4 | Parameterize orange acquisition and outbound/return staging; implement narrow entrance-region handoff. | `approach/autonomy/perception/hybrid_banner_detector.py`, `world/integration/entrance_readiness.py`, manager |
| 5 | Reuse a fresh native FSM and role-specific exit dispatch; fix demonstrated reverse-direction defects only. | `corridor/native/mission_runner.py` and existing controllers if tests reveal defects |
| 6 | Add exterior egress, expected LAND authority transition and actual touchdown/disarm confirmation. | Manager and shared telemetry/command helpers |
| 7 | Add independent truth validation and update launch instructions/handoffs. | `world/integration/qr_gazebo_validation.py`, new return evaluator/tests, `HANDOFF.md`, QR status and integration handoffs |

Implementation should preserve the existing outbound mission, coverage-only mode and QR-only endpoint as explicit regression profiles. The default full mission changes from stopping at target hold to finishing the return and landing.

Do not modify the nested corridor repository broadly or erase existing unrelated changes. Any native changes must be minimal, test-driven and documented.

## 12. Validation plan

Start cheaply; do not repeat a long whole-field coverage run to prove return logic.

### Offline and saved-map tests

- Existing QR, coverage, command-lease, freshness, corridor and startup regressions.
- Five-second continuous dwell: jitter, interruptions, stale telemetry and paused/accelerated source clock.
- Match-only return trigger; no repeat takeoff/initial QR/sweep, and no restart of red residence across phases.
- Saved-map return routes around a large red barrier, a newly discovered barrier, an initially unknown entrance approach and a genuinely disconnected entrance.
- Route budget expiry versus no-route classification; bounded search/replanning without infinite loops.
- Rotated corridor/field frames and southbound entry/exit displacement.
- Staging permit/fence gates, roof exclusion, camera loss, stale attitude/scan and old-token rejection.
- Outbound versus return exit dispatch.
- LAND ACK without mode change; mode change without touchdown; on-ground without fresh disarmed heartbeat; expected LAND versus pilot takeover.

### Short targeted Gazebo scenarios

1. Return-only field transit using an initialized/saved map, including live red updates and the entrance-region handoff.
2. Orange acquisition/staging and complete reverse corridor traversal, with both side-obstacle cases covered across targeted fixtures.
3. Exterior egress, normal LAND and independently observed touchdown/disarming.
4. Short fault scenarios for missing orange banner, blocked entry route and stale required sensor data, stopping before a long mission is needed.

A targeted return-only fixture must still enforce fresh telemetry, camera timestamps and airborne admission. Test initialization must not become a permissive default restart path.

### One end-to-end nominal flight

Run the full mission with an early matching target, preserving normal initial QR, green corridor, field target centring/descent, five-second hold, red-safe return, orange corridor and final landing. Stop after one independently verified pass. Additional full flights are justified only by a concrete failure or subsequent material change.

Existing long coverage tests remain evidence for coverage; this return test need not traverse every lane.

### Pass criteria

- Independent Gazebo truth confirms 5 m target altitude and continuous five-second settled dwell using declared tolerances.
- Return climb reaches the configured 10 m transit altitude before field transit; any later acquisition descent occurs in the authorized clear stand-off.
- No new red-zone incursion, unauthorized mission-region departure, collision or roof crossing. If an injected incursion is tested, warning/deadline behaviour is separately verified.
- Return routes and approach gates are supported by current observations/geometry, not simply expected coordinates.
- Reverse native FSM completes with measured entry/exit progress and no unsupported-geometry bypass.
- No stale/revoked command is applied at a handoff, and no velocity sender competes with LAND.
- Independent truth places the full airframe outside the corridor/roof at descent and confirms ground contact; fresh FC evidence confirms on-ground and disarmed.
- Logs expose source/receipt ages, authority transitions, phase deadlines and final outcome.
- Existing outbound and QR-only regression results remain valid under their explicit profiles.

These are Gazebo acceptance criteria, not physical-flight certification.

## 13. Deliberately deferred

- Real orange HSV/exposure tuning and front-camera lens/FOV measurement.
- Hardware timestamp/latency calibration, dual-camera throughput and Pi thermal/timing benchmarks.
- Site-specific entry and landing-region registration, actual roof clearance checks and FC LAND/failsafe validation.
- Changes to the obstacle model unless reverse tests expose a genuine supported-geometry defect.
- Payload actuation and verified delivery.
- Exact-home landing, pending the later requirement to restore full competition behaviour.
- Speculative QR formats, dynamic geofence input and autonomous arbitrary-terrain landing classification.

## 14. Status at the end of this planning task

Only this plan and its identical Desktop copy are created. No return code, new flight validation or source changes are claimed. The existing orange banner is present, but the return journey remains unimplemented.

