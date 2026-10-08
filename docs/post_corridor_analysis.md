# Post-corridor mission review — [competition] [competition]

Review date: 2026-10-05. Target: Raspberry Pi 5, 8 GB RAM, real cameras, flight controller and localization. Immediate validation environment: Gazebo/ArduPilot SITL, non-ROS.

## Engineering assessment

The coverage algorithm is worth keeping. The ideal-camera projection, measured coverage accounting, conservative treatment of unknown ground, checked connectors, and separation between the planning process and MAVLink supervisor are useful foundations. The current code also has credible nominal Gazebo evidence, including the user's latest full-field completion.

**I would not put the current post-corridor implementation on an unrestricted physical mission yet.** The main blockers are software contracts around perception and control: partly unusable images can certify unseen ground as clear; terminal handling can command a position hold with an invalid estimator; a failed terminal transmission can bypass cleanup; the flight map can survive an undetected position discontinuity; and planning/GUI stalls can interrupt red-zone deadline handling. The Gazebo acquisition/clock adapter is explicitly not a hardware adapter.

**A Pi 5 with 8 GB is a plausible platform for this algorithm.** There is no need for an AI model or dense reconstruction. The measured grid memory is small. The important compute problem is worst-case latency: a difficult route takes hundreds of milliseconds on the desktop, despite ordinary frames taking only a few milliseconds. More RAM will not fix that serial planning delay.

## Scope, assumptions and evidence integrity

- Inspected the actual `/home/sid/[competition]_mission2` working tree, including uncommitted changes. Repository HEAD is `5f9573ce9e88e9fec3d7d065c7d69994e7cd58b5`; HEAD alone does not identify the reviewed working tree. Test manifests retain coverage source hashes.
- Reviewed the mission PDF's text and page-3 diagram. The diagram gives a nominal 40 × 30 m delivery field; the text requires ascent to approximately 10 m, avoidance of restricted zones, supplied geofence coordinates, and a 15-minute competition mission. The diagram explicitly allows field layout changes. It does not specify the detailed 5+5-second red-zone rule.
- Applied the user's red-zone interpretation: detect entry autonomously, warn after five seconds, and exit by ten seconds total. Begin recovery immediately; waiting five seconds before attempting escape would be inappropriate.
- Scope begins at `CORRIDOR_EXITED`, includes field advance, ascent, coverage startup, mapping, coverage/red-zone decisions, command supervision and termination. The archived `archive/legacy/boustrophedon_sweep_prototype.py` is **not** the implementation the integrated manager invokes.
- QR interpretation/scanning, delivery and return are deliberately deferred. Their absence is not counted as a defect. `COMPLETE` currently means completion of this coverage segment, not completion of the competition mission.
- Flat terrain, no airborne obstacles in the open field, two cameras retained, and replaceable fixed field coordinates remain accepted project assumptions. No speculative geofence input system or airborne LiDAR avoidance is requested by this review.
- **Camera configuration needs reconciliation:** the design report specifies two Camera Module 3 NoIR cameras, and the user subsequently said to adopt the new camera. The older Desktop corridor task list still says IMX296. This review provisionally follows that later instruction and discusses Module 3 NoIR-specific risks. A clarification was requested during review but not received before writing. If IMX296 remains the chosen camera, the Module 3 rolling-shutter/autofocus/NoIR findings become conditional; the software, synchronization and calibration findings remain applicable. Neither the earlier 10 mm lens nor the retracted f/10 statement is treated as a current measured calibration.
- No mission source or configuration was edited. Audit probes were separate programs with synthetic inputs or mocked I/O. Gazebo campaigns used an owned instance `-I2`, ports 5780/9022, and a separate partition. All processes created for these campaigns were stopped.

Detailed evidence is retained in [post_corridor_review_evidence_2026-10-05](/home/sid/Desktop/post_corridor_review_evidence_2026-10-05). Distinguish **confirmed software behavior**, **likely physical risk**, and **hardware/site validation needed** throughout this report. Synthetic probes demonstrate branches and mathematical counterexamples; they do not measure a real aircraft.

## What actually runs after the corridor

| Stage | Current behavior and reference | Important boundary |
| --- | --- | --- |
| `CORRIDOR_EXITED` | Manager stops motion, checks a fixed local-NED field bounding box, then enters `ADVANCE_TO_FIELD`. `experimental_corridor_manager.py:2491–2521`, helper at `:360`. | A bounding-box check is not registration of the physical field or proof of roof clearance. |
| `ADVANCE_TO_FIELD` | Commands 0.15 m/s north, correctly converted into body forward/right using measured yaw. Stops after reaching `n_min + clearance + 0.4`, then waits for horizontal speed below 0.10 m/s. `:2530–2568`. | Fixed northward entry and 12 source-second / 30 wall-second limits assume this world layout. |
| `ASCEND_FOR_COVERAGE` | Converts the measured HOME/local-Z offset into a local target, climbs with bounded vertical speed, requires altitude/speed dwell and a clear-to-climb position envelope. `:2572–2593`, `corridor_altitude.py:36–95`. | The offset is sampled once; physical field elevation and safe ascent location remain external assumptions. |
| Coverage startup | Stops the corridor command service, passes the same MAVLink connection to `coverage_mission.runtime.main`. `:2595–2621`. | One command owner is intentional and correct. There is a fresh synchronization/entry wait, not immediate sweep motion. |
| Runtime admission | Subscribes to downward image and Gazebo clock; joins attitude, position and HOME-relative altitude by source time; checks EKF flags, armed/GUIDED state, timing and worker results. `runtime.py:289–510`. | This source adapter depends on Gazebo; it is not a Pi camera source. |
| Mapping and sweep | Detects red contours, projects them onto flat ground, updates a 10 cm grid, follows measured serpentine obligations using checked connectors and observation frontiers. `geometry.py`, `planning.py`, `engine.py`. | Ground marked observed is trusted as non-red unless the color detector says otherwise. |
| Recovery | Stops for expired evidence; `ESCAPE` can follow the frozen known map during camera loss if localization is healthy. Residence warning/deadline is updated in `Engine.step`. | Timer and escape planning depend on the same worker receiving valid pose jobs. |
| End | `COMPLETE` requires resolved obligations, no remaining required unseen cells, and two seconds of measured settling. Runtime attempts a position hold, exits; manager closes the connection. `engine.py:204–213`, `runtime.py:533–554`, manager `:2745–2748`. | No return or automatic landing is implemented, as agreed. A terminal hold is not a verified indefinite contingency. |

## Tests run and results

### Existing offline regression suite

**60 tests passed in 21.00 seconds.** Tests were imported from the old isolated project's `tests/` directory but explicitly executed against `/home/sid/[competition]_mission2/coverage_mission`, not against its older copy. Also included the current integrated handoff and asynchronous-log tests.

Reproduction command:

```bash
cd /home/sid/[competition]_mission2
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
PYTHONPATH=/home/sid/[competition]_mission2:/home/sid/[competition]_mission2/approach:/home/sid/[competition]_mission2/corridor:/home/sid/[competition]_mission2/world/integration:/usr/lib/python3/dist-packages \
python3 -c 'import coverage_mission, pytest; print("TESTED PACKAGE:", coverage_mission.__file__); raise SystemExit(pytest.main(["-q", "-p", "no:cacheprovider", "--rootdir=/home/sid/[competition]_mission2", "/home/sid/[competition]_mission2_coverage/tests", "/home/sid/[competition]_mission2/world/integration/test_coverage_handoff.py", "/home/sid/[competition]_mission2/coverage_mission/test_async_logs.py"]))'
```

### Fresh Gazebo campaigns against current coverage code

These use a 12 × 10 m fixture, the existing smoke configuration, ideal nadir camera and speedup 2. The fixture intentionally removes LiDAR (`fixture.py:61–62`), so these runs are not a two-camera-plus-LiDAR Pi workload benchmark. The unchanged full mission's retained user run supplies separate desktop integration evidence.

| New test | Observed outcome | Independent measurements / limits |
| --- | --- | --- |
| `gazebo_multiple` | `COMPLETE`; two red rectangles avoided | Approximately 202.48 source seconds from first to terminal decision; 6,150 truth samples; maximum truth gap 35 ms; zero sampled physical red incursions/fence violations; zero unseen permissible cells; maximum projected-corner error **0.10655 m**; minimum clearance beyond the assumed body radius **0.47443 m**. All applicable existing evaluator checks passed. |
| `gazebo_recoverable` | `COMPLETE` after six injected outages | Camera drop, 350 ms delayed imagery, black frames, telemetry drop, invalid-EKF flag, and 350 ms worker delay. All six stopping/settling checks passed. Maximum measured excursions for those cases were 0.063, 0.139, 0.138, 0.123, 0.110 and 0.132 m, respectively. Zero sampled red/fence incursions and coverage gaps; peak corner error **0.09606 m**. No long straight-leg samples were available, so that metric is unavailable, not a passed tracking test. |
| `gazebo_deadline` | Deliberate overstay ended `ABORTED`, after physically leaving red | Warning at **5.000 s** after estimated entry; deadline failure at **10.0285 s**; estimated residence 14.648 s; independently measured physical residence **14.483 s**. The fault forcibly drove/held the vehicle inside red. No fence violation. Evaluator `passed=false` and campaign exit code 1 correctly preserve the failed mission; only detection and post-fault response behaved as expected. |

The recoverable EKF test suppresses software estimator validity; it does **not** corrupt the flight controller's actual state estimator. The worker-delay test sleeps the worker; it does **not** prove recovery from a blocked supervisor or serial driver. Physical wind, camera materials, rolling-shutter artifacts, CSI timing and power/thermal behavior were not simulated by these campaigns.

Commands used the current package and existing harness, for example:

```bash
cd /home/sid/[competition]_mission2
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
GZ_IP=127.0.0.1 GZ_DISCOVERY_MULTICAST_IP=239.255.0.23 \
GZ_SIM_SYSTEM_PLUGIN_PATH=/home/sid/ardupilot_gazebo/build \
GZ_SIM_RESOURCE_PATH=/home/sid/[competition]_mission2/world/models/models:/home/sid/ardupilot_gazebo/models \
PYTHONPATH=/home/sid/[competition]_mission2:/usr/lib/python3/dist-packages \
python3 -u -m coverage_mission.campaign \
  --config /home/sid/[competition]_mission2_coverage/config/smoke.json \
  --scenario multiple --output /tmp/post-corridor-review/gazebo_multiple \
  --limit 420 --speedup 2 --instance 2
```

Use a new output directory for a rerun; the harness refuses existing output folders and occupied instance ports. For the recoverable case, use `--scenario clear --faults /home/sid/[competition]_mission2_coverage/config/fault_recoverable.json`; for the deadline case, use `--scenario incursion --faults /home/sid/[competition]_mission2_coverage/config/fault_deadline.json --limit 150`. These are simulation harness commands, not flight launch commands.

### Retained full-world run found during this review

The user's `/tmp/[competition]-fullmission-check.zFu2Yn/coverage.result.json` reports **`COMPLETE`, pending 0, unseen 0**, with no JSONL loss. All coverage-module hashes recorded in its manifest match the current files. The manifest does not hash the manager or entire world, so that match is specifically evidence about the coverage package.

The logged coverage interval is **3,347.237 source seconds, approximately 55.8 minutes**. There are 63,361 logged decisions. Worker time: median **4.145 ms**, p95 **7.301 ms**, p99 **9.030 ms**, maximum **827.364 ms**. The final map records 3,163 measured-done points and 548 excluded points out of 3,710 obligations; these counts can overlap when a previously visited point is subsequently excluded. There are 4,153 enclosed grid cells. Exclusion is not physical traversal.

This is stronger evidence than the stale documentation claiming the enclosure-completion change had never completed a new full-world run. However, no independent truth recording accompanied that user run in its folder. Do **not** turn its self-reported completion into a new claim of zero physical incursions or globally accurate coverage. The earlier `fullworld_oct2_third` truth trace had zero sampled incursions and good geometry but ended `BLOCKED` on its earlier revision; those are separate results.

### Adversarial offline probes added outside the repository

| Probe | Actual result from current code | Interpretation |
| --- | --- | --- |
| Image 96.875% black, 3.125% usable gray | Accepted; **1,900 cells** marked observed; vehicle-center cell becomes free | Confirms overclaim of usable ground, not merely a hypothetical lighting concern. |
| Dark red patch `(B,G,R)=(0,0,30)` amid normally exposed terrain | No red cells; patch center observed and free | Local unobservable/dark pixels are not kept unknown. Final real-camera HSV values still need testing. |
| Two false red observations followed by 100 clear observations | **441 red cells remain confirmed** | Map confirmation is irreversible during the mission. |
| Position moves 0.8 m in 0.4 s with reported zero velocity | `SWEEP`, no discontinuity terminal state | The guard admits this inconsistency and keeps the existing map. |
| Altitude becomes 10.6 m | `HOLD`, vertical command zero | The envelope protection stops correction too; not evidence of altitude recovery. |
| Confirmed red ring with a 0.4 m raw opening | Interior is not classified enclosed, yet no route exists through the inflated map | Physical unreachability is broader than the implemented raw-red enclosure test. |
| Synthetic calibrated lens: 70° HFOV, radial `k1=-0.1` | Four-corner footprint overclaims the top midpoint by **0.381 m**; a point beyond the true edge is marked observed | A mathematical limitation of the distorted-footprint representation. These are test parameters, not the measured camera lens. |
| Sustained invalid EKF, with fresh position packets — mocked runtime | Sends **10 terminal position-hold packets** | Freshness is used without estimator validity at shutdown. |
| Lost heartbeat, with fresh position packets — mocked runtime | Sends **10 terminal position-hold packets** using cached armed/GUIDED state | Command authority is not revalidated at terminal dispatch. |
| Terminal hold transmission throws `OSError` — mocked runtime | Exception escapes; worker join not reached | Cleanup is not exception-safe. |

The runtime mocks isolate supervision and do not exercise actual optics, worker computation, or vehicle behavior. Their script and results are retained alongside the image/geometry probes.

## Problems to fix before physical testing

The software work below does not need final Pi benchmarks, final HSV thresholds or final vehicle dimensions. Where a physical input is needed to complete commissioning, that dependency is stated separately.

### P01 — Terminal hold trusts a fresh but invalid position

**Critical; confirmed software behavior.** [runtime.py:533](/home/sid/[competition]_mission2/coverage_mission/runtime.py:533), especially `:534–540`; estimator and authority checks are earlier at `:447–465`.

On shutdown, the runtime tests a cached heartbeat and the age of the latest position packet. It does not require valid EKF flags, valid synchronization, an unchanged origin, or a fresh heartbeat. Both invalid-EKF and heartbeat-loss probes emitted ten position-hold commands. An origin/discontinuity exception can enter this same terminal block.

Recent telemetry is not necessarily trustworthy telemetry. A real EKF failure may leave numerically plausible position packets while horizontal position is unusable. Reasserting a position/yaw target then is not an established safe action. Gazebo's invalid-EKF injection leaves its actual estimator healthy, so it readily holds and hides this distinction.

**Fix now:** distinguish normal completion hold from localization/authority failure; use current verified authority and localization for any position command, and establish an explicit bounded contingency outcome when that contract is absent. Do not equate “sent ten packets” with FC acceptance or physical hold. **Later:** validate the chosen FC contingency, pilot takeover, touchdown/hold behavior and installed firmware/settings. This does not require inventing the return mission.

### P02 — Terminal send errors bypass cleanup and confuse handoff ownership

**High; confirmed by fault probe.** `runtime.py:533–554`; [manager handoff:2604](/home/sid/[competition]_mission2/world/integration/experimental_corridor_manager.py:2604).

The final `hold()`/`velocity()` sends are outside a protective cleanup layer. A transport exception skips worker termination, log finalization and result writing. The probe raised an uncaught `OSError` and never joined its mock worker.

The outer manager catches any escaping runtime exception under a comment asserting coverage failed before sending motion. That assertion is not generally true: an exception during coverage's final send can escape after a whole flight. The manager can then reclaim abort authority using its pre-coverage telemetry snapshot, since its `drain_mavlink()` loop has been suspended throughout coverage.

**Fix now:** make cleanup unconditional, make runtime outcomes explicitly report whether control was acquired/released, and refresh authority before any outer fallback. **Gazebo masking:** loopback UDP sends usually succeed. A disappearing serial/USB link is more relevant to the real aircraft.

### P03 — The post-handoff command path loses the independent command lease

**High; confirmed design gap, physical effect requires fault/FC testing.** Manager `:2597` stops `CommandService`; runtime `:409–431`, `:444–446`, `:507–529` directly performs receive, synchronization requests, stream requests and command writes on its supervisor thread.

The supervisor correctly sends zero velocity when its worker result expires, provided that supervisor is still running. It has no separate coverage-stage sender/watchdog equivalent to the corridor service. A stalled supervisor or blocking output prevents its own expiry handling. The full manager does supply asynchronous console output, and JSONL has been moved off this loop; those are genuine mitigations. Standalone runtime console prints remain synchronous.

ArduPilot's Guided command timeout can stop a vehicle when velocity updates cease, but it is a configurable FC behavior, not proof of the Python reaction-time assumption. Its documented default is three seconds. GCS failsafe monitors GCS heartbeat; a still-running MAVProxy can outlive a failed mission process. The coverage runtime itself does not establish a mission-liveness heartbeat contract. [Guided timeout](https://ardupilot.org/copter/docs/ac2_guidedmode.html), [GCS failsafe](https://ardupilot.org/copter/docs/gcs-failsafe.html).

**Fix now:** bound flight-critical dispatch and test supervisor stalls/output failure, preserve one command owner, and define how FC watchdogs detect mission-process loss. **Later:** verify installed FC parameters and real link behavior. Do not assume the design report's proposed failsafes are installed.

### P04 — Partly unusable imagery becomes free-ground evidence

**Critical; confirmed by image probes.** [geometry.py:103](/home/sid/[competition]_mission2/coverage_mission/geometry.py:103), `:113–125`; [planning.py:53](/home/sid/[competition]_mission2/coverage_mission/planning.py:53).

`validated_hsv()` accepts a frame if only 2% of pixels satisfy its brightness/saturation test. `GroundMap.observe()` then marks the whole eroded geometric footprint observed. All non-red pixels effectively become permissible evidence, including locally black, clipped, obscured or otherwise unclassifiable pixels. The 96.875%-black probe and dark-patch probe demonstrate that exact behavior.

On hardware, a shadowed red zone, lens obstruction or local exposure clipping can therefore create a checked route over restricted ground. A uniformly black-frame test passes because it rejects the whole frame, but does not cover this local failure. Gazebo's uniformly lit textured floor rarely exercises it.

**Fix now:** represent unusable parts as unknown and distinguish evidence of clear ground from absence of a color detection. Keep this lightweight; no learned detector is required. **Later:** determine usable-pixel criteria, exposure/color settings and detection performance with actual outdoor images.

### P05 — Distortion support does not make the four-corner footprint conservative

**High; confirmed mathematical counterexample, conditional on distorted delivered imagery.** `geometry.py:85–96`, `planning.py:58–62`.

Individual image points are undistorted before ray projection, which is correct. However, the whole visible footprint is represented by only four projected corners. Under radial distortion, the projected image boundary need not be the straight quadrilateral joining those corners. The valid synthetic 70°/`k1=-0.1` profile marks ground beyond the true top-edge midpoint as observed; the overclaim is 0.381 m before grid erosion, larger than the one-cell visibility reserve.

**Fix now:** make footprint visibility conservative for supported calibrated models, or explicitly constrain the accepted input to a rectified ideal-camera image with matching calibration. The same reasoning applies to approximating distorted contour edges with sparse vertices. **Later:** measure real distortion/crop and select the supported image path. Gazebo's zero-distortion camera cannot expose this case.

### P06 — Two color hits become an irreversible hazard and coverage exemption

**High; confirmed persistence rule, likely outdoor reliability risk.** `planning.py:66–90`, `CoveragePlan.update():162–163`.

Two distinct timestamps mark a cell permanently confirmed red. No separation by time/viewpoint, geometric consistency or localization quality is required. Repeated correlated errors are easy with a stationary camera, red objects that are not ground zones, NoIR color shifts, vibration or a wrong projection. One hundred later clear observations did not remove the probe's false patch.

Persistent red is appropriate for genuine static zones; casually forgetting them would be worse. But permanently accumulating errors can close routes, thicken zones, falsely form enclosing rings, and exclude required work. Conversely, a repeatedly missed dark zone receives no protection. Gazebo's stable pure-red material makes two frames appear much stronger evidence than they are.

**Fix now:** make confirmation and contradiction semantics explicit, use valid ground evidence and geometric/pose consistency, and distinguish exclusion confidence from a provisional avoidance mark. Do not implement unconditional time-based hazard deletion. **Later:** determine evidence thresholds from measured false positives/negatives and the competition's actual zone material.

### P07 — Map continuity is weaker than the physical registration contract

**High; confirmed weak guard, likely physical risk.** `engine.py:151–161`, `runtime.py:419–423`, `:462–465`.

The horizontal jump bound is `max(0.3, 3*dt)`. A pose change of 0.8 m over 0.4 s with reported zero velocity was accepted as `SWEEP`; the map survived. Larger gaps allow larger changes. There is no comparison against measured velocity or tracked uncertainty. Origin changes are checked only after an origin message has already been received; the runtime starts with `origin=None` and does not explicitly request the origin in `request_streams()`. HOME changes are not tracked.

A modest estimator reset, compass correction or changing localization bias can shift the aircraft relative to a permanently accumulated map without triggering the tested one-metre-jump case. **A constant common translation shared by the red map and vehicle may cancel for local avoidance**; it still matters to the independently registered geofence. Time-varying bias and old-map revisits do not enjoy that cancellation.

**Fix now:** carry/validate the registration and estimator continuity contract across handoff, detect physically inconsistent position/heading/altitude changes, and stop using the old map after a detected reset. **Later:** measure actual drift, heading bias and uncertainty; do not claim the current 0.30 m reserve covers ordinary GPS merely because EKF flags are healthy.

### P08 — The field transition is registered to this Gazebo world, not to the real course

**High; confirmed portability limitation.** Manager `:360–384`, `:2499–2586`; `config/full_mission_coverage.json`.

The accepted field is N=[−20,20], E=[−15,15]. Advance is always north and climb clearance is inferred from crossing a particular north coordinate. The code does correctly rotate the north command into body coordinates; this is not an ENU/NED sign bug. It also correctly distinguishes local-Z from HOME-relative altitude. Those mechanics should remain.

Knowing the corridor origin does not, by itself, make the physical field use these local coordinates, bearing or HOME datum. No downward red map exists during advance/ascent: its acquisition starts inside the coverage runtime. A red area at the exit/ascent point would be discovered only after committing to that point. The current world provides a benign entry patch.

**Fix before site flight:** make the manual field/corridor registration and known safe transition/ascent region explicit and verifiable, including their altitude datum. Fixed coordinates can remain a simple configuration. **Site/competition input:** actual boundary, corridor heading, clear roof/exit envelope and whether red is excluded from that entry patch. No dynamic organizer-input system is needed now.

### P09 — A hardware camera/clock contract is still missing

**High; confirmed integration limitation.** `runtime.py:27–111`, `:289–319`, `:325–329`, `:462–479`; `geometry.py:128–137`.

The adapter imports Gazebo transport/messages and uses `/world/miss2_world/clock`. Hardware cannot supply those streams as written. The decoder accepts specific Gazebo RGB/BGR encodings, and coverage requires exactly the configured frame shape. The current projection uses one timestamp/pose per whole image.

The improved runtime actually uses **`RoundTripClock`/MAVLink TIMESYNC**, not just the older reception-paired `SimClock` estimate. That is a good foundation. It still needs an explicit relationship between real camera exposure metadata, host monotonic time and FC boot time. A stable offset residual does not measure exposure duration, rolling readout, buffering, FC estimator latency or asymmetrical link delay.

**Fix now:** define the real adapter interface and reject absent/mismatched timestamp/calibration metadata; bind effective image size/crop to its intrinsics. **Hardware work:** measure timestamps, latency distribution and synchronized image-to-pose error. Keep the two-sided interpolation and stale-data rejection. Merely relaxing the 0.20 s budget until a Pi runs is not a defensible calibration.

### P10 — Red warning/deadline updates stop with the worker or usable localization

**High; confirmed control-flow limitation.** `runtime.py:472–479`, `:487–504`; `engine.py:146–168`, `Residence.update():20–43`.

Residence advances only when `Engine.step()` runs. With an active incursion and unhealthy localization, no new pose job is submitted; with blocked planning/GUI, no step occurs. The supervisor stops motion and can abort after ten wall seconds, but it does not independently advance the active five/ten-second residence alarm. On recovery, the engine may backdate/latch failure correctly; that does not provide the warning at five seconds while the outage is occurring.

**Fix now:** keep elapsed incursion/deadline accounting alive in the supervising layer, retain uncertainty about whether exit occurred, and never reset residence from missing data. This does not authorize blind escape without localization. **Later:** test the complete contingency when localization is lost inside red; no software can promise a geometrically safe exit with no trustworthy state and no known route.

### P11 — GUI responsiveness still affects coverage planning and recovery

**High for recovery availability; confirmed design coupling.** `runtime.py:259–279`; manager `:2604–2610`, `:2655–2665`.

Coverage `imshow`, map drawing, resize and `waitKey` execute inside the perception/planning worker after publishing its decision. A stalled display leaves the supervisor able to stop, which is good, but also prevents new mapping, escape updates and residence events. The corridor preview had been separated; coverage has not.

The manager's `--no-gui` controls its own preview but is not passed into `run_coverage()`. Its old preview escape event is not polled while the synchronous coverage call runs. Coverage uses `q` in its own windows. This is an inconsistent operator interface, not a guaranteed emergency-stop mechanism.

**Fix now:** retain the requested testing GUI as an optional observer, propagate the GUI setting and unify operator cancellation/ownership. Avoid letting a dead preview interrupt the only process able to compute escape. Final refresh rate is a Pi benchmark decision.

### P12 — Route planning has unbounded work relative to the decision lifetime

**High; directly measured on the desktop.** `planning.py:111–134`, `:207–250`; `engine.py:239–242`; `runtime.py:487–492`.

A full-grid detour around one large rectangle took **855.9 ms** after the Gazebo campaigns stopped; the earlier concurrent measurement was 992.4 ms. The existing user's integrated run independently recorded a **827.4 ms** worker peak. The valid decision window is 200 ms. Whole-grid Python A*, repeated line-of-sight simplification and frontier scans have no deadline/cancellation boundary within that call.

The latest-only queue prevents an unlimited backlog, and the supervisor rejects stale results, so this is not evidence that an old velocity must be applied. It is evidence of lost control availability: repeated holds, missed mapping windows, and worse red-recovery responsiveness on slower hardware. Nominal-frame median timing conceals it.

**Fix now:** bound planning work, preserve a valid checked path while replanning when possible, cancel obsolete computations, and measure planning separately from frame work. Cache map-derived structures when their inputs have not changed. **Later:** choose final grid/planner rates from Pi worst-case measurements without weakening clearance or completeness.

### P13 — Fragmented red imagery scales processing with full-field raster work

**Medium–High; confirmed scaling, outdoor frequency unknown.** `geometry.py:113–124`, `engine.py:95–99`, `planning.py:45–65`.

Every contour is projected separately and rasterized into a fresh full-field grid. At 640×480 and a 400×300 map, desktop median `observe()` times were **3.08 ms** for no red contours, **7.47 ms** for 100 small contours, and **44.28 ms** for 1,000 small contours. These were four iterations per case, not a production benchmark distribution. A textured/NoIR/noisy image can create much more fragmented evidence than a Gazebo rectangle.

**Fix now:** bound and instrument this work using shared/batched polygon processing or local raster regions; do not simply discard small hazards or truncate evidence and call the rest clear. **Later:** characterize typical and worst-case segmentation on real imagery and benchmark the resulting load.

### P14 — Unreachable coverage is handled for only one topology

**Medium; confirmed gap in completion semantics.** `planning.py:79–90`, `:160–175`, `engine.py:189–196`.

The recent change correctly excludes a non-red island enclosed by confirmed raw red, without pretending it was traversed. However, it does not represent an island accessible only through a gap narrower than the aircraft/clearance envelope, or an area disconnected by red plus the fence. The 0.4 m opening probe left the interior non-enclosed and route-less even though the inflated map could not reach it. Such obligations remain pending and can produce `BLOCKED`.

Conservative `BLOCKED` is preferable to entering red. It still falls short of the user's broader “do not try impossible internal coverage” intent. **Fix before relying on full-coverage completion:** explicitly distinguish unknown, temporarily inaccessible and proven physically unreachable requirements relative to the vehicle's reachable component; retain evidence and exclusions separately. Do not exempt work just because a search ran out of time. Competition acceptance of exclusions remains a separate question.

### P15 — Physical energy feasibility is not established by full Gazebo completion

**High before a full-field physical attempt; confirmed timing mismatch, endurance estimate unverified.** `config.py:62–92`, `planning.py:145–153`, `runtime.py:400`, `:447–510`; design report §2.5 and Table 15.

The latest integrated coverage takes about **55.8 source minutes**. Even the nominal ten-lane empty-field polyline is about **395.15 m**, or **8.78 minutes at a continuous 0.75 m/s**, before acceleration, inspection holds, corridor traversal and recovery. The design report estimates roughly 7–9 minutes of aircraft endurance. Its endurance is not an as-built measurement, but it is enough to reject the assumption that a long desktop run is physically feasible with the current narrow-camera profile.

The runtime does not inspect battery status or enforce an energy reserve. A two-hour command-line wall cap is a simulation test limit, not a flight budget. Battery failsafes might eventually act if configured; this code does not verify them.

**Before physical flight:** establish a bounded test area/duration and a verified reserve/terminal contingency. Full mission time optimization can remain deferred, as previously agreed. **Later:** measure actual endurance and the selected camera footprint; a wider calibrated camera and eventual early target identification change the task cost. This finding does not prescribe faster flight or a battery change without data.

### P16 — Calibration and configuration accept combinations the planner/oracle do not fully support

**Medium; confirmed structural limitation.** `config.py:9–38`, `planning.py:141–148`, `:217–219`, `evaluate_run.py:85–104`.

Projection supports mount rotation and asymmetric intrinsics. Nominal sweep spacing still treats the camera's width as east-west and length as north-south, independent of configured heading/mount yaw; frontier window sizing also omits `ground_above_home`. For a substantially rotated camera or changed lens, this can create gaps requiring repair or unnecessary overlap. Actual observed-cell completion prevents a simple claim that all such cases falsely complete, provided visibility itself is correct.

The independent oracle assumes the ideal centered nadir camera and constructs rays from `fx` for both axes; it does not validate arbitrary distortion/asymmetric calibration/mounts. The isolated fixture explicitly rejects unsupported calibration, which is good. **Fix now:** validate supported configuration combinations and make test coverage match them. **Later:** use measured intrinsics/mounts; do not extrapolate the old oracle's accuracy result to a different physical optical model.

## Hardware/site tests required before choosing final values

These are not reasons to postpone the software corrections above.

| Area | Evidence and current assumption | Educated assessment / required measurement |
| --- | --- | --- |
| Camera model and effective intrinsics | Full-world profile is 640×480, HFOV 28.2°, zero distortion, ideal nadir, 5 cm downward offset. This is still the previous narrow-lens simulation profile. | Measure the selected module/variant, processing crop/resize, focus setting, distortion and mount. A Module 3 NoIR has different optics; do not transplant the old 10 mm geometry. Sensor native resolution need not be the mission resolution. |
| Module 3 NoIR color and focus, if selected | Module 3 has autofocus; NoIR lacks an IR-cut filter. Code uses fixed HSV red intervals H=0–12/168–180, S≥100, V≥55, with no exposure/AWB/focus contract. | Outdoor infrared contribution and automatic camera adjustments can change apparent color and focus. Evaluate actual red material, vegetation/backgrounds, shadows and glare. Stabilize/calibrate focus and camera settings as appropriate. Keep the simple detector initially; a neural model is not required. [Official Module 3 specifications](https://www.raspberrypi.com/products/camera-module-3/), [camera tuning guidance](https://www.raspberrypi.com/documentation/computers/camera_software.html). |
| Rolling shutter or global shutter | Whole-frame projection uses one pose. Module 3 uses rolling shutter; IMX296 is global shutter. | For Module 3, measure readout/exposure timing and motion distortion; one frame timestamp is not identical to simultaneous exposure of all rows. Existing low-motion admission helps but does not prove a bound under vibration. Global shutter removes row skew, not exposure blur or pose latency. [Official shutter explanation](https://www.raspberrypi.com/documentation/accessories/camera.html). |
| Optical-motion envelope | Combined projection tilt ≤5°, angular rate ≤0.10 rad/s with ±50 ms brackets; control envelope uses per-axis 8°; map grace 1 s. | Conservative for this Iris model. A real drone can need persistent tilt to hold in wind, leaving no accepted observations. Test calm/windy hover and transitions; do not blindly widen gates. Positive evidence: rejected frames do not earn coverage. |
| Altitude and flat ground | Projection uses HOME-relative altitude minus `ground_above_home` and camera down offset. Configured ground is −0.207 m, specific to Gazebo. | HOME-relative altitude is a reasonable source on a verified flat, same-elevation site; a terrain sensor is not automatically necessary. Measure the ground datum and barometric drift. At the current ~3.2 m footprint corner radius, a 0.5 m height error produces roughly 0.16 m scale error. A 5 cm offset has much smaller impact. |
| Altitude-envelope recovery | At 10.6 m the engine returns zero vertical velocity because it is outside the ±0.5 m coverage envelope. | Holding/mapping rejection is conservative. It cannot correct that error itself; stale-map expiry eventually ends the run. Decide whether bounded reacquisition or explicit abort is appropriate after testing real altitude behavior. Do not enlarge the mapping envelope merely to suppress aborts. |
| Position and heading accuracy | Fixed map reserve 0.30 m; small synthetic stress uses 4 cm translation and 2° heading bias. EKF flags do not quantify that bound. | About 1° pitch/roll error shifts the nadir ray ~0.175 m at 10 m; 2° yaw error moves a 3.2 m corner ~0.11 m. Budget these together with timing, localization drift and grid error. The report's claimed ~50 cm GPS accuracy is not proof of 30 cm geofence/map integrity. |
| Vehicle envelope | Coverage assumes a 0.40 m circular body radius. Report gives ~419.6 mm wheelbase and 7-inch props: conditional circumscribed prop diameter ~597.4 mm if wheelbase means opposing motor centers. | The current circle is not obviously too small against that nominal calculation, but measure the assembled propeller/gear/payload envelope. Keep a circle for yaw-independent conservative clearance unless data justify refinement. Do not confuse the report's three-inch inter-prop gap with boundary clearance. |
| Dynamics and escape feasibility | Speed 0.75 m/s, acceleration/braking 0.5 m/s², reaction 0.25 s; stopping lookahead uses measured and commanded velocity. | At top speed the assumed reaction-plus-braking distance is 0.75 m. Measure worst-case stopping and crosswind drift with the actual vehicle/load/FC. A large enough incursion cannot physically be exited within ten seconds at these limits; detection before entry remains primary. Escape omits normal turn-braking/stopping checks, so validate overshoot near a fence after escape, not only ordinary sweep turns. |
| Field transition | Northward advance, known safe ascent location, rectangular registered field. | Survey/test field origin, heading, actual exit location and any ground elevation offset. Verify the advance and ascent are outside restricted ground. No new aerial-obstacle system is required under the stated envelope. |
| FC transport throughput | Requests position/attitude at 50 Hz, relative altitude at 20 Hz, EKF at 10 Hz, plus TIMESYNC and command traffic. | Those four telemetry streams alone are roughly 5.1–5.2 kB/s with unsigned MAVLink 2 framing, before other messages/signatures (exact size depends on extensions/truncation). A 57,600-baud serial link has only about 5.76 kB/s byte throughput with 8N1. USB or a faster configured UART may be fine; measure actual delivered rates/latency, rather than assuming all requests are honored. |
| FC failsafes and end state | Normal completion ends in a position hold; no terminal hold acknowledgement or ongoing supervisor. No battery/RC/whole-process contingency is configured by this code. | Confirm actual FC settings and test companion death, MAVProxy-only survival, unplugged link, RC takeover and invalid localization. Retaining the scoped hold endpoint is acceptable only with an explicit tested operator/FC continuation for physical trials. |
| Thermal, power and interference | Pi, both camera streams and USB LiDAR will share power and compute resources. | Run a sustained simultaneous workload with cooling and the flight power supply; record throttling, voltage events and frame age. Official camera documentation also notes possible Module 3 interference near GPS L1: treat it as a specific bench/site comparison with cameras active, not proof this aircraft has that fault. [Camera documentation](https://www.raspberrypi.com/documentation/accessories/camera.html). |

## Raspberry Pi 5 feasibility and meaningful optimization

### What I expect the Pi to handle

The Pi 5 has a 2.4 GHz quad-core Cortex-A76 and two MIPI camera/display interfaces. Its documentation supports two directly connected cameras and recommends active cooling for sustained performance. These capabilities make simultaneous acquisition plausible, but do not certify this application's timings. [Pi 5 specifications](https://www.raspberrypi.com/products/raspberry-pi-5/), [multiple-camera documentation](https://www.raspberrypi.com/documentation/computers/camera_software.html).

At the current 400×300 grid, all `GroundMap` NumPy arrays together use **1,440,000 bytes (~1.37 MiB)**. The 3,710 path obligations are also small. A VGA BGR frame is 921,600 bytes; eight buffered frames are ~7.37 MB. RAM capacity is not the present limiting resource.

Two VGA BGR streams at 15 fps represent **27.65 MB/s** of application image bytes before copies. Two 1920×1080 BGR streams at 15 fps represent **186.62 MB/s**. Two full 4608×2592 BGR streams at 15 fps would be **1.075 GB/s** before copies; this last number is a sizing calculation, not a supported simultaneous mode/FPS claim. Native 12 MP processing is unnecessary for red-zone segmentation. Sensor/ISP raw bandwidth and processed-buffer bandwidth are distinct.

**Provisional engineering estimate, not a benchmark:** ordinary VGA HSV/sparse-projection/grid updates should plausibly fit a 10–15 Hz observation path on a cooled Pi 5, with a separate 20 Hz command/supervision path. If this desktop's routine 4–9 ms work costs 2–6 times as much on the Pi, it becomes roughly 8–54 ms before acquisition, IPC and scheduling. That is a planning range, not a guaranteed CPU ratio or FPS. The 0.83–0.99 s desktop route spikes could become multi-second interruptions, and the 1,000-contour workload could consume much of the entire latency allowance. Those must be addressed and then measured.

No change to an AI model, dense point cloud, 3D mapping stack or wholesale C++ rewrite is justified by the evidence. Optimize the specific latency sources first.

### Optimization priorities

| Priority | Change worth considering | Why it matters / what to preserve |
| --- | --- | --- |
| 1 | Put a work budget/cancellation boundary around A*, frontier scoring and path simplification; reuse valid routes and map-derived connected components/distance fields by map revision. | Measured desktop route spikes already exceed decision freshness. Preserve checked connectors and no-corner-cutting. Do not enlarge freshness windows just to accept old routes. |
| 2 | Separate coverage GUI rendering from perception/planning, with latest-only frames and a lower configurable refresh rate. | Prevent diagnostic stalls from suppressing escape/timer updates. Keep the GUI for testing. |
| 3 | Batch red contour projection and rasterization; use bounded local regions instead of one full-grid allocation per contour. | Removes the demonstrated contour-count × field-size cost. Preserve small hazards and partial-frame evidence semantics. |
| 4 | Avoid repeated full-frame HSV validation. | `worker()` validates the latest raw frame, then `engine.observe()` converts/validates an admitted frame again. Cache per sequence where the same image is used, without confusing raw-camera liveness with accepted projection evidence. |
| 5 | Bound camera buffering/copies; prefer calibrated low-resolution ISP output for this detector. | The current multiprocessing job can carry an admitted frame and a newer raw frame, potentially ~1.84 MB per 20 Hz job, plus serialization/copies. Repeated frames are sent even when no new mapping is needed. A bounded shared buffer/index design is worth considering after measuring IPC, but do not add complexity without evidence. |
| 6 | Use indexed/bounded-neighborhood pose lookup rather than repeated full history scans. | `PoseHistory.at()` copies and linearly scans a 30-second history; main matching and angular gates call it repeatedly. It is bounded, not a leak, but search work grows with stream rate. Keep true bracketing and quaternion interpolation. |
| 7 | Reuse map masks/morphology kernels and avoid unnecessary full refreshes. | `GroundMap.refresh()` relabels the field, dilates twice and erodes observed ground after every admitted frame. `_escape()` also computes a full distance transform on every recovery step, including unused returned nearest-point indices. Useful caching is preferable to reducing safety margins. |
| 8 | Stage-aware sensor processing at handoff. | Front-camera decoding and LiDAR array preparation continue in manager callbacks during coverage, even though their results are not used there. Keep acquisition if desired, but avoid unnecessary full processing/preview work. The corridor RANSAC loop itself is not continuing during the synchronous coverage call, so it should not be blamed for all post-corridor load. |
| 9 | Explicitly control native-library thread counts. | `OPENBLAS_NUM_THREADS=1` / `OMP_NUM_THREADS=1` did not make OpenCV single-threaded in the audit process: `cv2.getNumThreads()` reported 12 on this desktop. Check the actual Pi build and prevent accidental oversubscription across camera/planner/GUI processes. |
| 10 | Extend timing/health instrumentation to the whole post-corridor pipeline. | Current `processing_ms` starts after the artificial worker delay and excludes queue transit, GUI work after publication, capture/decode and command transport. Record source exposure-to-command age, queue replacements, maximum supervisor gap, decode/map/plan timings, admitted-frame fraction, RSS, throttling and disk backlog. |
| 11 | Preserve bounded asynchronous diagnostics, but improve evidence accounting. | JSONL logging no longer blocks ordinary control and reports dropped records. `newly_traversed_points` is computed before a latest-only result queue can discard it, so overwritten worker results can lose incremental validation evidence even with zero disk-writer drops. Track result sequence gaps/cumulative counters or a reliable bounded evidence channel. |
| 12 | Consolidate tests/configuration into the actual mission project. | Current isolated campaign defaults refer to `config/smoke.json` / `coverage.json`, absent from this repo, and most coverage regressions live in `[competition]_mission2_coverage/tests`. This review supplied explicit paths and verified imports. Make future validation reproducible without accidental use of the older package; this is a tooling issue, not an in-flight failure. |

### Pi acceptance measurements

Use both selected cameras, real LiDAR acquisition, the intended FC transport, diagnostics enabled and sustained flight-power/cooling conditions. Treat the following as provisional acceptance targets to test, not measured capabilities:

- The supervisor/command path must remain responsive independently of GUI/storage/planning stalls; validate the maximum command gap against the measured stopping-distance contract.
- Track the complete 0.20 s source-age budget: capture exposure/readout, ISP/copy, telemetry brackets (currently including 50 ms future attitude evidence), queueing, compute and command transmission. Processing time alone is insufficient.
- Measure p50/p95/p99 **and maximum** latency under ordinary ground, highly fragmented red texture, long detours, repair, escape, dual-camera load and log storage contention.
- Confirm a sustainable accepted-observation rate during moving flight and wind-loaded hover. High camera FPS with all frames rejected by the motion gate is not useful throughput.
- Keep resident memory and queue sizes bounded; ensure no thermal throttling or power-induced camera/USB failures over the intended sortie and contingency duration. Do not assume 8 GB solves CPU scheduling.
- Re-run independent geometry/clearance checks with the calibrated image path before trying to reduce margins or increase speed.

## Competition clarifications that should remain separate

| Unknown | Current working interpretation | What depends on clarification |
| --- | --- | --- |
| Restricted-zone material and marking | Red visual ground regions; static during a segment | Real hue/reflectance, white borders, whether other red objects count, and whether a zone is a filled polygon or just a border. Keep thresholds configurable; do not invent a recognition protocol. |
| Incursion definition | Current timer uses map red dilated by the assumed aircraft radius | Whether organizers count vehicle center, any propeller/body overlap, projection onto the ground, uncertainty buffers, repeated entries or cumulative residence. Keep the user-agreed 5+5 rule provisionally; it is not specified in the supplied three-page excerpt. |
| Unreachable interior | User wants geometrically inaccessible internal work excluded | Whether such coverage/target exclusions are acceptable to organizers, and whether targets can be placed there. Do not claim excluded areas were inspected or traversed. |
| Field boundary and entry | Replaceable fixed rectangular coordinates for now | Coordinate reference/format, actual corridor bearing, entry patch and boundary geometry. A survey/configuration step is sufficient until format is known. |
| Camera/target altitude and definition of completion | Coverage at ~10 m; current segment ends after coverage | How target acquisition will later stop search, delivery sequencing and required evidence. QR encoding, return and delivery remain intentionally outside this review. |
| Competition time versus test time | PDF says 15 minutes for the entire mission; full-coverage segment currently has a generous simulation cap | Final mission strategy can wait, but physical energy reserve and bounded test scope cannot be replaced by that cap. |

## Parts I trust and would leave alone for now

1. **The basic projection method.** Calibrated sparse rays intersecting a verified flat plane is appropriate and inexpensive. FRD/body-to-NED rotation, yaw-aware projection and source-time interpolation are present; no obvious metre/centimetre or degree/radian error was found in these paths. Keep them, with the footprint/model limitations above corrected.
2. **HOME-relative altitude on verified flat ground.** This is a workable input with a measured ground datum and error budget. Do not add a terrain reconstruction system just to replace it. Centimetre camera offsets are lower priority than incorrect intrinsics, pose timing or metre-scale localization error.
3. **Measured coverage credit and separate exclusions.** Points are credited from actual pose travel, not elapsed waypoint time. Unknown ground is not a normal traversable route. Enclosed exclusions are not marked done. Preserve that accounting distinction.
4. **Conservative geometry checks.** Clearance inflation, field inset, no-corner-cutting A*, sampled segment checks, braking lookahead and measured turn settling are sensible. Their physical constants need measurement, not automatic deletion.
5. **Latest-only bounded work queues and a separate supervisor.** These already prevent ordinary worker delay from issuing indefinitely stale commands. The fresh outage campaign provides real evidence for that limited claim. Extend isolation to GUI and supervisor/output failure rather than discarding the architecture.
6. **Source versus host time separation.** Source time governs motion/residence in simulation; host watchdogs catch a stopped producer. Matching brackets, rejecting resets and rejecting invalid imagery are good foundations. Hardware must provide a defensible equivalent clock contract.
7. **Persistent static-zone memory and immediate escape intent.** A confirmed real zone should not disappear just because it leaves the image. Recovery starts immediately, warning/deadline logic latches failure, and crossing two overlapping zones does not reset the residence timer. Improve evidence quality, not these intentions.
8. **Simple classical perception and bounded map size.** A 1.37 MiB map and VGA color processing are compatible with the Pi target. Keep Python/NumPy/OpenCV/SciPy until profiling identifies a specific routine needing replacement.
9. **No airborne LiDAR avoidance in this segment.** This follows the agreed open-space envelope. The relevant LiDAR issue here is concurrent resource use, not a missing obstacle system.
10. **The scoped endpoint.** Ending this segment in a verified hold with a defined physical-test continuation is reasonable. Missing QR, delivery and autonomous return are not counted as bugs.

## Order of work before physical deployment

1. Correct terminal validity, exception-safe cleanup and handoff ownership (P01–P02).
2. Establish coverage command expiry/FC contingency and independent residence timing during faults (P03, P10). Test stalled supervisor/transport, not only delayed worker input.
3. Correct the usable-ground evidence contract and distorted-footprint overclaim (P04–P05).
4. Make map confirmation and estimator continuity robust enough for physical data (P06–P07).
5. Implement/commission the real camera-time adapter and bind the correct camera profile, effective crop and mount (P09 and hardware table). Final tuning still waits for data.
6. Make field registration and the safe advance/ascent region explicit; verify them on the site (P08).
7. Decouple coverage GUI, fix option/cancel propagation, and bound worst-case planning/segmentation work (P11–P13).
8. Complete unreachable-area/configuration contracts and reproducible validation tooling (P14, P16).
9. Benchmark the simultaneous Pi workload, then set justified operating rates and uncertainty/stopping budgets. Preserve margins until the evidence supports a change.
10. Establish battery reserve and a bounded physical test plan before a complete-field attempt (P15); extend area only after the shorter sorties pass.

## What would count as ready for a bounded physical trial

- The confirmed software counterexamples above have regression tests and no longer manufacture free-space evidence, command invalid terminal targets or skip cleanup.
- Hardware capture timestamps, effective intrinsics, distortion treatment, ground datum and mounting are verified; a printed/measured ground target validates the entire image-to-ground chain under the permitted movement envelope.
- Localization continuity and the manually registered fence are tested, including a reset and gradual drift case; the map is not silently reused after registration loss.
- Stale camera, stale pose, lost heartbeat, invalid EKF, GUI stall, planner overrun, storage stall, transport failure and mission-process death produce their declared bounded outcomes on the installed FC stack.
- Red entry warns at five seconds and latches failure by ten under the declared operating envelope, including degraded sensing. Inability to find a known safe exit is reported as a failure, never as completion. Site geometry and dynamics make the intended recovery physically possible.
- A simultaneous, thermally steady Pi workload meets the measured latency/stopping contract with margin. Reports include command timing and rejected/dropped observations, not only mean FPS.
- An independently evaluated current-code Gazebo regression covers the chosen calibrated camera model and the full-world field transition/coverage. The latest self-reported full-world `COMPLETE` is retained as useful evidence, while independent physical clearance/coverage remains to be re-established for that exact run/profile.
- The trial has a measured energy reserve, a safe terminal continuation and functioning pilot/FC takeover. This is compatible with leaving QR and return implementation for the next development stage.

The requested review is complete. The repository's mission code and configuration were left unchanged; the report and retained test evidence are the outputs of this pass.

---

## Implementation addendum — 2026-10-06

This section records the subsequent coding pass. The review and its original
evidence above remain a historical baseline, **not a description of the new
working tree**. The user confirmed that the intended cameras are global-shutter
IMX296 units; Module 3 NoIR/rolling-shutter discussion above is conditional and
does not define the current camera choice. The downward camera's 10 mm lens is
provisional; the front lens is unknown. No AI detector was introduced.

| Finding / decision | Implemented now | Still open / reason |
| --- | --- | --- |
| P01–P02 terminal validity and cleanup | Terminal position hold now requires current authority, EKF, time synchronization, origin and fresh position. Transport exceptions are caught; worker join, log close and result writing still run. Manager no longer assumes a coverage exception occurred before motion. | Installed FC mode/failsafe and pilot/landing continuation must be exercised on hardware. Integrated zero-velocity endpoint is not a guaranteed indefinite hover after mission-process exit. |
| P03 single command authority | One leased `CommandService` remains live from corridor into coverage. Manager claims a coverage-stage token, revoking corridor proposals. Sender supports BODY_NED and LOCAL_NED, expires stale motion to zero, and sends a final zero on shutdown. | A complete return stage is not implemented. Actual link/process-death behavior and FC Guided timeout require hardware tests. Standalone isolated coverage harness still owns and sends directly to its own SITL vehicle; the integrated mission uses the shared sender. |
| P04 red/usable evidence | Widened configurable provisional red HSV to H 0–15/165–179, S≥60, V≥25. Per-pixel usable and red masks are ground-projected by one pinhole/flat-ground homography. Unreadable pixels remain unknown; a largely black frame no longer manufactures free ground. | Sun/shade/material HSV limits and exposure settings are not finalized. This is deliberately a simple color detector, not an AI model. |
| P05/P16 camera calibration | Zero-distortion nadir pinhole configuration is explicit. Unsupported nonzero distortion/mount rotation now fails configuration validation instead of allowing a four-corner footprint to overclaim coverage. Current 640×480, ~28.2° Gazebo profile is consistent with the nominal 10 mm IMX296-sized sensor, but is **not measured calibration**. | Measure the effective crop/intrinsics/distortion/mount and extend the footprint method only if measured values require it. Do not infer the front lens from the downward profile. |
| P06 confirmation | Changed irreversible red confirmation from two to five distinct admitted observations; first hit remains an immediate motion barrier. New local regression tests cover this. | Final threshold remains provisional until outdoor false-positive/negative trials. An old test in the previous isolated repository still asserts two hits and is intentionally not changed there. |
| P07 pose continuity | Engine compares pose increments with integrated reported velocity, including altitude, and aborts on inconsistency; origin continuity is checked at integrated geofence handoff and by coverage runtime. | Drift within the allowance and absolute geofence registration need site measurements. No claim that EKF flags alone certify 0.30 m accuracy. |
| P08 field geometry/transition | Config supports four GPS geofence corners ordered `[SW,NW,NE,SE]`; at corridor exit, FC `GPS_GLOBAL_ORIGIN` registers the rectangle to local NED. Nonrectangular/unregistered geometry fails closed. The map/velocity are transformed into the derived field frame. Advance follows the measured corridor-exit yaw, not fixed north, and checks the resulting climb point against field inset. Existing Gazebo bounds are unchanged by default. | Actual organizer coordinates, surveyed corridor exit orientation, red-free transition/climb patch and ground elevation are site inputs. The current planner intentionally supports rectangular geofences only. |
| P09 camera timing / Pi | Kept latest-only image job transport, source timestamp/pose bracketing and IMX296 whole-frame global-shutter model; eliminated duplicate HSV conversion when a raw/admitted frame is the same. | Real libcamera acquisition, exposure timestamp to FC boot-clock alignment, drop/latency data and simultaneous two-camera bandwidth have not been measured or implemented. Current adapter still imports Gazebo transport. |
| P10 red deadline | Coverage supervisor now keeps the known-entry 5 s warning and 10 s failure active from source clock while the planning worker stalls. No missing observation can clear a known residence. | With stale clock/localization the aircraft can only stop and invoke a tested FC/operator contingency; software cannot promise a known exit route then. |
| P11 GUI | Coverage preview runs as an optional, lossy separate process; `--no-gui` propagates from manager. Rendering cannot block the planning worker's `imshow/waitKey`. | Preview refresh and camera-copy cost should be measured on Pi. GUI `q` is an abort request, not certified E-stop. |
| P12 route latency | Conservative 3× then 2× coarse search, original-grid connector checks, fine fallback, vectorized supercover checks and bounded-by-path-length binary-search simplification. The review's full-grid rectangular detour now measures ~73 ms on the desktop, versus ~856 ms reviewed. | An adversarial fine-grid fallback is still not a hard real-time bound; benchmark worst cases on Pi and retain the supervisor's stale-result rejection/leased zero command. |
| P13 fragmented red load | Replaced per-contour projection/full-grid allocation with one homography warp of image masks. At 1,000 synthetic red patches, median `observe()` is ~4.5 ms on this desktop (review ~44 ms). | Benchmark real image texture, two cameras and thermal throttling on Pi. |
| P14 unreachable obligations | Proven-unreachable connectivity now uses clearance-inflated **confirmed** red; a raw 0.4 m opening no longer falsely implies physical reachability. Unknown ground never itself forms an exclusion barrier. Exclusion remains distinct from traversal credit. | Site geometry and accepted competition treatment of excluded red regions remain external. |
| P15 energy/time | No QR/battery/time optimization added per user decision. | Before any full-field physical sortie, establish a measured bounded test duration and FC/pilot contingency. QR-driven early stop is deferred until encoding rules arrive. |

### Verification and limits

- `git diff --check` and Python compilation passed.
- **76 offline/synthetic/integration tests passed, one obsolete inherited test deselected.** The inherited test in `/home/sid/[competition]_mission2_coverage/tests/test_commissioning.py` expects two red frames; the current mission requires five and has a new local regression test in `coverage_mission/test_post_corridor_changes.py`. All synthetic replay cases in the executed selection passed.
- Benchmarks on this desktop, not Pi 5: 0/100/1000 synthetic red patches took about 4.26/4.31/4.50 ms median for `Engine.observe`; a checked full-grid rectangular detour took about 72–74 ms in three repeats. No hard worst-case latency claim follows from these small samples.
- A new isolated Gazebo campaign was attempted, but this workspace's sandbox rejected socket creation before simulator startup (`PermissionError: Operation not permitted`). **No new Gazebo validation is claimed.** Run the updated full mission with the independent truth monitor and evaluate fence/red incursions, actual coverage, handoff and completion in a normal terminal. Existing older Gazebo results apply only to the earlier code revision.
- Not yet ready for physical flight: the real Pi camera/time adapter is absent; optics/exposure/mount, FC failsafes, localization/geofence accuracy, vehicle margins and simultaneous Pi workload remain unmeasured. The flat-ground and open-air operating envelope remains explicit.

The integrated source of truth is `[competition]_mission2/world/integration/FULL_MISSION_COVERAGE.md`; the manager and world handoffs carry the same current-status warning. QR, delivery and return remain out of scope.

### Validation continuation — 2026-10-06

The sandbox socket restriction cited above no longer applies. A current-code
isolated Gazebo two-red-zone run passed the independent evaluator: `COMPLETE`,
zero sampled red incursions/fence violations/permissible gaps, 0.103 m maximum
projected-corner error and 0.083 m maximum long-leg cross-track error. Evidence:
`/tmp/[competition]-coverage-oct6-final-check2/evaluation.json`.

Full-world 1× commissioning additionally exposed dark grass texture yielding
no continuous free cells with the previous per-pixel brightness rule, plus a
target-selection loop: an unknown first ordered obligation displaced known
reachable work, then the planner selected the vehicle's current cell as a
frontier. The full-world profile now uses provisional geometry-scaled local
green-texture evidence; black/dark neutral/red remains unknown. Known reachable
work takes priority, frontier viewpoints require displacement, checked lane
look-ahead excludes completed targets, and a no-route HOLD has its own bounded
deadline. These changes have 46 focused passing tests. They are not evidence
that real outdoor exposure or color thresholds are finalized.

The first original 40 × 30 m re-run was interrupted by an environment restart
before completion; its `/tmp` evidence was lost, so it cannot be counted as a
pass. A new run from autonomous takeoff is underway with persistent evidence
at `[competition]_mission2/world/integration/artifacts/full_recheck_20261006/`. Its
full-field result is **pending**; partial safe traces are not a completion
claim. The latest executable status is in
`[competition]_mission2/world/integration/FULL_MISSION_COVERAGE.md`.

### Full-world liveness correction — 2026-10-06

The persistent original-world rerun at
`[competition]_mission2/world/integration/artifacts/full_recheck_20261006/` reached
coverage but did **not** complete: pending/unseen counts stopped improving
while the aircraft repeated a safe route. The saved map showed many small
enclosed dark-texture observation holes inside a large unexplored exterior.
There were zero independently sampled red/fence incursions in that partial
run, but safety alone does not constitute mission success.

The planner now gives exterior unknown components frontier priority until
exterior exploration is complete, while retaining interior holes as later
obligations. The progress watchdog requires a new traversed point or a
meaningful area of newly observed ground, rather than one flickering pixel.
The full-world threshold is provisionally 2.0 m²; generic default 0.25 m².
This is a liveness safeguard, not final outdoor color/exposure calibration.

The corrected current-code isolated two-red-zone Gazebo campaign passed the
independent evaluator: `COMPLETE`, zero red incursions, zero fence violations,
zero permissible gaps, 0.099 m maximum projected-corner error, 0.025 m
maximum straight-leg cross-track error and 0.227 m maximum credited-route
cross-track error. Evidence:
`[competition]_mission2/world/integration/artifacts/isolated_after_frontier_fix/evaluation.json`.
The finite current-code test selection passed **59 tests**. The original-world
rerun from autonomous takeoff remains in progress at
`[competition]_mission2/world/integration/artifacts/full_frontier_20261006/` and is
**not yet a validated complete mission**. Real Pi camera timing/calibration,
simultaneous hardware load and FC failsafes remain hardware work.

### Second full-world result and current rerun — 2026-10-06

The `full_frontier_20261006` original-world run did **not** pass. It safely
progressed beyond the earlier stall, but stopped `BLOCKED` at 884 pending
points. Independent truth recorded zero red/fence incursions and a 0.084 m
maximum projected-corner error, but 11,826 permissible 0.1 m cells had no
accepted camera footprint. The saved map and diagnostic images identify
unobserved sectors and a central GLB red strip whose interior renders dark.

The Gazebo fixture now contains a paint-only visual at the existing central
zone bounds; it adds no collision and is not fed to the controller. The
full-world green-texture evidence scale is provisionally 1.0 m instead of
0.4 m; a large pure-black patch still remains unknown in a new test.
**60 finite tests pass.** A 1× full-world rerun from autonomous takeoff is
currently collecting evidence under
`[competition]_mission2/world/integration/artifacts/full_paint_20261006/`. Its result is
pending; neither earlier partial run is a completed mission pass.

### Checked-relocation watchdog follow-up — 2026-10-07

The next full-world run (`full_paint_20261006`) reduced pending points to 95,
but still ended `BLOCKED` during a long checked connector to the remaining
corner. Independent truth found zero red/fence incursions and a 0.078 m
maximum projected-corner error, but 1,577 permissible grid cells still lacked
an accepted camera footprint; it was not a complete pass. The trace showed
steady vehicle displacement toward the remote target. The 120 s evidence-only
watchdog had no provision for that legitimate transit.

The watchdog now gives bounded extra time based on maximum net displacement
from the last coverage/map progress point, with a provisional 0.20 m/s
connector assumption and a field-diagonal cap. A repeated loop cannot reset
the allowance indefinitely. **61 finite tests pass.** A new 1× original-world
flight plus independent truth monitor is running in
`[competition]_mission2/world/integration/artifacts/full_relocation_20261007/`.
Completion and evaluator status remain pending; Pi timing and hardware
calibration remain separate unvalidated work.

### Faster validation and late-stage printed-ground issue — 2026-10-07

The original full-world flight is still **not** controller-validated as
`COMPLETE`. A restarted 2× flight aborted at 359 pending points because a
soft A* clearance preference repeatedly exceeded the 200 ms planning budget.
The hard clearance mask and checked route segments remain; removing only that
soft cost made the exact saved-map connector finish in about 95 ms. A later
full-world flight reached 40 pending points with zero independently sampled
red/fence incursions and zero geometric footprint gaps. The next flight
reduced that to 15 points with zero incursions, but still ended `BLOCKED`.
The remaining points were on a single lane through the printed QR patch,
where 22 tiny unreadable 0.1 m cells enlarged into an impassable band after
clearance erosion. QR decoding/target behaviour remains deferred.

The full-world profile now permits bounded contextual inference for at most
0.20 m² isolated unknown components, only away from red and map edges;
inferred cells remain separate from actual observed cells. The saved final map
then offered checked routes to all 15 remaining lane points. A new
`coverage_mission.replay_saved_map` command completed them in two kinematic
trips (30.14 m; 33 ms worst route search) in under a second. It also resolved
the previous 40-point saved map. **65 offline tests pass.** This replay
checks planner/map liveness, not camera timing, flight dynamics, red safety,
or end-to-end mission completion. Therefore the next full Gazebo run remains
necessary, but should occur only after cheap map/replay and focused camera
tests; repeated 30-minute flights are no longer the debugging loop.

The last truth evaluation also had one of 3,161 credited lane points at
0.307 m against the existing 0.300 m test bound, with an 8 mm FC-vs-Gazebo
pose difference at that instant. It is recorded as a failed criterion, not
silently rounded away. Real camera calibration, Pi profiling, site color
validation and FC failsafes remain outstanding for physical flight.

Two further 2× full-world flights have not established a pass. The first
stopped `BLOCKED` with zero pending obligations but 33 contextually clear
cells wrongly counted as unseen; truth showed zero red/fence incursions and
zero geometrically unviewed permissible cells, but one 0.307 m credited-route
error exceeded the 0.300 m criterion. The second stopped `ABORTED` after 46
200 ms planner timeouts and accelerated source-time expiry of fresh
zero-command HOLD decisions. Truth again showed zero red/fence incursions,
but the flight was incomplete. Completion accounting, direct checked frontier
selection and zero-HOLD admission received targeted changes; 68 selected
offline/integration tests pass. No confirming full Gazebo flight has occurred.
Details and artifacts: `[competition]_mission2/world/integration/FULL_MISSION_COVERAGE.md`.

Current provisional Gazebo rule: dark/black pixels in fresh, correctly shaped
downward-camera frames are clear if the red detector does not mark them red.
The full-mission profile disables the older green/print support blurs and no
longer rejects wholly black received frames on content alone, per user
decision. Missing/stale frames remain invalid. This trades conservative
unknown-ground handling for simpler mapping, but a fresh camera-fault black
frame or undetected dark red could be mistaken for clear ground. 68 selected
tests and two saved-map replays pass; integrated Gazebo completion remains
unproven.

The later staged original-world flight in
`[competition]_mission2/world/integration/artifacts/full_corridor1x_20261007` reached
manager and coverage `COMPLETE` with zero planner timeouts and zero sampled
red/fence incursions. Corridor ran at the proven 1× rate; synchronized
Gazebo/SITL coverage ran at 2×. Independent truth found a 0.143 m maximum
corner error and 0.119 m maximum straight-leg cross-track error. The user
accepted this as a **Gazebo mission pass with documented exceptions**: the
unchanged evaluator still reports 25 unviewed 0.1 m cells (all in one
red-clearance-excluded 0.25 m² patch) and one 0.307 m credited-point error
against the former 0.300 m bound. These are not silently recoded as evaluator
passes. Pi, real-camera and outdoor flight validation remain outstanding.

