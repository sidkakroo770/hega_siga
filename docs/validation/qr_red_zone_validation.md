# Shared downward-camera QR and red-zone validation

Latest corrective checkpoint: the subsequent main-world approach exposed false
QR detection/selection over grass, which these initial near-target fixtures did
not exercise. The fix and new 18-check textured-ground approach are documented in
[qr_false_candidate_fix.md](qr_false_candidate_fix.md). Earlier results below are
retained with their original scope; they were not proof of textured-ground search.

Date: 2026-10-08. Scope: targeted [competition] field-phase validation, not physical flight approval.

## Answer and command hierarchy

One downward-camera acquisition supplies both algorithms. They do not open separate camera streams. `coverage_mission/runtime.py:358` owns acquisition and retains at most eight frames. The same timestamped image/pose pair is supplied to mapping and to the separate QR perception process. The Gazebo fixture uses 640×480 images at 15 Hz; mission decisions are scheduled at up to 20 Hz, without treating repeated exposures as new observations.

QR discovery is capped at 5 Hz. Payload decoding is capped at 2 Hz and authorized only after centering and settling. The QR worker has one image in flight and bounded queues. It cannot send vehicle commands. After a matching target is confirmed, its worker is stopped: further QR searching is unnecessary. Red observation and safety continue during descent and hold.

Hierarchy:

1. The global mission manager owns phase transitions and hands over a leased command token (`world/integration/experimental_corridor_manager.py:2768`). The field runtime temporarily becomes the sole MAVLink reader; the manager does not run a competing telemetry/control loop during that call.
2. The field supervisor validates FC authority, pose, clock, camera and decision freshness before admitting motion (`coverage_mission/runtime.py`, main control loop).
3. The engine arbitrates field fence, red-zone residence/escape, attitude, observation validity and stopping clearance (`coverage_mission/engine.py:194`). It can veto a QR proposal.
4. QR inspection takes priority over ordinary sweep navigation only when those safety checks admit it. Sweep motion and coverage credit pause while centering/reading/descending; red mapping and residence timing do not pause.
5. `world/integration/command_service.py:11` is the single velocity sender. Its stage token rejects old publishers, and its 0.30 s command lease expires to zero translation. FC/pilot mode changes remain above autonomous navigation authority.

The important distinction is **simultaneous perception, not simultaneous conflicting steering**. QR centering/descent and boustrophedon steering never independently command the aircraft at the same time.

Within each field worker job, live image mapping occurs before QR inspection, and engine safety arbitration occurs after the inspection proposal (`coverage_mission/runtime.py:273–299`). If red escape is required during descent, downward velocity is forced to zero. Safety interruptions clear read/settle/terminal-dwell continuity.

## Short Gazebo tests actually run

These are owned field-start fixtures using the real global manager, autonomous takeoff, Gazebo camera images, QR decoding, SITL, field engine and leased sender. They intentionally inject the known fixture reference `REF-001` and skip initial QR/corridor/return. They are not new end-to-end validations of those stages.

The existing red rectangle is N ∈ [−9,−6], E ∈ [−4,−2]. The test QR is the existing 1 m black-and-white printed marker, moved only in a private copied fixture. Production worlds and QR assets were not changed. Aircraft starts at N=−10.2, E=−3.8 and takes off to 10 m. The configured planner clearance is 0.70 m, comprising 0.40 m body radius plus 0.30 m provisional uncertainty.

| Case | Target centre | Expected behaviour | Result |
| --- | --- | --- | --- |
| Reachable QR near red | N=−10.2, E=−3; centre 1.20 m from physical red edge | Center, settle, confirm three distinct matching reads, cancel sweep and descend to 5 m while red safety remains active | PASS: 15/15 independent checks |
| QR too close to red | N=−9.55, E=−3; centre 0.55 m from physical red edge | Exclude target because required 0.70 m clearance is unavailable; do not center/read/descend to it | PASS: 13/13 independent checks |

The unsafe-target scenario ends by **controlled harness interruption** after observing exclusion for four wall seconds. Its retained manager exit is 124 and runtime terminal state is ABORTED/Keyboard interrupt. Those are expected fixture shutdown, not a successful completed mission and not a hidden timeout failure. The evaluation explicitly requires `controlled_stop_after_exclusion`. There is no reason to fly the whole field to test this exclusion decision.

Independent Gazebo model poses were checked at all field altitudes, including the entire descent below 9 m; the truth monitor's generic above-9 m summary alone was not used as proof.

| Evidence | Reachable target | Too-close target |
| --- | --- | --- |
| Independent trajectory samples | 1,558 | 369 |
| Maximum truth sample gap | 0.018 s | 0.018 s |
| Minimum physical body-to-red clearance | 0.780 m | 0.792 m |
| Physical body red incursions / field fence violations | 0 / 0 | 0 / 0 |
| Exact source timestamps shared by QR and mapping | 21 | 14 |
| Confirmed red map cells | 120 | 220 |
| QR decode jobs / worker restarts | 3 / 0 | 0 / 0 |
| Distinct mapped exposures during target descent | 255 | No target descent authorized |
| Worst terminal horizontal error from true QR centre | 0.0225 m | Not applicable |

The reachable case also passed independent HOME-relative altitude and stationary-hold checks, confirmed three distinct stationary matching source exposures, and never resumed sweep after matching. Mapping observation poses continued below 7 m. Both cases reported zero dropped diagnostic records. The too-close target remained excluded, generated zero payload decode jobs and stayed at coverage altitude.

Evidence directories:

- `world/integration/artifacts/qr_near_red_20261008_01/`
- `world/integration/artifacts/qr_too_close_red_20261008_01/`

Each contains fixture/world copies, independent truth, camera/QR results, decisions, supervision commands, final map and `qr_red_evaluation.json`. The evaluator is `coverage_mission/evaluate_qr_red_run.py`.

## Offline adversarial safety regressions

Seven focused tests in `coverage_mission/test_qr_red_zone.py` cover:

- Safe target descent beside a red boundary, with no mistaken coverage credit.
- Exclusion of a target inside required clearance while the aircraft itself is safe.
- Newly observed red over a selected target during reading.
- New red at the vehicle/target before an otherwise eligible third confirmation: escape/safety wins and the target does not latch.
- Newly unsafe matched target during descent: downward continuation is rejected.
- Red escape overriding a stale latched descent proposal.
- Stale camera preventing continued descent.

The selected broader regression suite passed **148 tests**, including these seven plus existing QR, return, coverage, handoff and corridor safety tests. New-red appearance during inspection/descent was tested offline against the real inspection and arbitration classes; it was **not** dynamically injected into a Gazebo flight. The two live fixtures validate static nearby red, actual shared camera processing, real decoding and vehicle response.

## Pi implications and remaining limits

Measured on the development computer during the reachable fixture: mapping median 5.82 ms / p95 9.16 ms; QR processing p95 15.92 ms; peak QR request-to-result wall time 25.65 ms. The too-close fixture had mapping p95 10.24 ms and QR p95 15.48 ms. These are worker timings, not total frame-to-actuation latency, and **not Raspberry Pi measurements**.

The architecture is suitable for Pi benchmarking: one camera owner, bounded buffering, resized processing images, separate bounded QR work, limited decode rate, no AI model, and a control sender independent of perception/GUI/logging. These tests do not establish native-resolution processing throughput, dual physical camera bandwidth, thermal sustainability, real exposure timestamps, outdoor QR readability or calibrated camera-to-ground accuracy. Those still need Pi/camera testing with both cameras and LiDAR acquiring.

No mission priority, safety margin, threshold or normal flight behaviour needed changing to pass these cases. Added implementation consists of focused tests, a read-only evaluator and a hidden, owned-harness-only field-start entry guarded by environment, explicit QR-only mode, 10 m takeoff and registered starting pose. Normal mission still acquires its initial reference; it does not use the fixture injection.

## Reproduce

Run from the repository root, with no existing simulator occupying port 5760; use a fresh output directory each time:

```bash
python3 -m world.integration.qr_gazebo_validation --scenario qr-near-red --seconds 160 --gui --output world/integration/artifacts/qr_near_red_manual_01
python3 -m coverage_mission.evaluate_qr_red_run world/integration/artifacts/qr_near_red_manual_01

python3 -m world.integration.qr_gazebo_validation --scenario qr-too-close-red --seconds 100 --gui --output world/integration/artifacts/qr_too_close_red_manual_01
python3 -m coverage_mission.evaluate_qr_red_run world/integration/artifacts/qr_too_close_red_manual_01
```

The `--gui` option enables mission camera/map diagnostics; the Gazebo server remains headless. The harness stops only the processes it creates. All owned processes from the recorded runs have been stopped.
