# QR mission implementation and validation

## Latest correction: grass false candidates

The user's full-world failure exposed invalid QR finder polygons and stale
candidate selection. These are fixed with geometry validation, three fresh stable
discovery exposures and bounded lost/progress/retry handling. A recorded-geometry
replay, 168 regressions and an actual textured-ground field approach through
matching QR descent passed. One real-marker loss recovered on its second attempt;
no false grass candidate was selected. Details and the limits of this evidence:
[../validation/qr_false_candidate_fix.md](../validation/qr_false_candidate_fix.md).

## Shared camera / red-boundary checkpoint

The targeted shared-downward-camera tests passed: actual centering, three
stationary matching reads and 5 m descent beside red; a second fixture excluded
an unsafe target without reading or descending. Seven offline safety cases and
the selected 148-test regression suite passed. See
[../validation/qr_red_zone_validation.md](../validation/qr_red_zone_validation.md) for evidence and limits.
No complete corridor flight was repeated; Pi performance is not yet validated.

## Return continuation

The results below validate the earlier QR-only endpoint. The default full mission
now continues through the return extension; see
[../return/mission_status.md](../return/mission_status.md).
Use `--qr-only` for the original endpoint. The QR validation harness preserves
QR-only behaviour unless `--return-mission` is supplied explicitly.

Implementation date: 2026-10-08. This extends the [competition] mission; physical deployment remains unvalidated.

## Implemented

| Area | Implementation |
|---|---|
| Printed markers | Six genuine high-resolution black-and-white prints, including quiet borders. All six old coloured screenshot markers are absent from the environment visual. The original terrain/corridor collision mesh remains unchanged. Prints are declared 1 m test fixtures, not assumed competition dimensions. |
| Initial reference | Autonomous takeoff to 5 m, feedback-based approximately 1 m advance along the settled heading, centering, measured-velocity settle, then three distinct decoded exposures. Both cameras must acquire before arming. Missing/unreadable reference forbids corridor entry. |
| Decoder | A verified ZBar-backed decoder, since the installed OpenCV locates codes but cannot decode them. No network requests, image-pattern comparison, AI model or speculative payload schema. |
| Identity | Exact nonempty UTF-8 text equality, preserving case/whitespace/URL parameters. Different QR masks/versions can represent the same identity. Non-UTF-8 binary is explicitly unsupported. |
| Perception | Timestamp-matched downward camera observations project through the existing calibrated ground-ray geometry. Discovery is decode-free at 5 source-Hz; stationary selected-marker decoding is capped at 2 source-Hz. Rates and all inspection limits are provisional configuration. |
| Ownership | Existing leased command sender remains the only navigation transmitter. Startup/field/descent stage transfers revoke old authority. A bounded telemetry refresh restores the manager's read ownership before banner search. |
| Field inspection | Spatial marker ledger, one selected marker, center → settle → read. Confirmed nonmatches resume unfinished coverage and are not repeatedly inspected. Two bounded attempts per unreadable marker; unknown locations defer, red/clearance/fence-excluded locations cannot authorize an approach. |
| Safety during inspection | Same map, camera/pose freshness, fence, stopping-region and red-residence checks remain active. Only coverage selection/progress timers pause; inspection does not manufacture traversal credit. |
| Match and descent | Exact match latches its physical location, cancels coverage/repair, revokes queued sweep authority, then feedback-descends to 5 m only while aligned. Actual low velocities, altitude and a two-second dwell are required for `TARGET_HOLD_5M`. No delivery, return or landing follows success. |
| Failure handling | One bounded native-worker restart per inspection generation; further failure aborts. A separate provisional 5 s startup-readiness budget admits no queued image during cold imports; admitted jobs retain the 1 s watchdog. Suspended workers are forcibly reaped after bounded teardown. Read/inspection deadlines cannot be reset by rediscovery, repeated frames or delayed confirmations. |
| Diagnostics | Separate bounded asynchronous QR/decision/supervision logs; QR job counts, processing and result-latency peaks, restarts, selected-marker statuses and confirmation exposures. Camera GUI remains available. |
| Pi preparation | Dual-camera ground benchmark can include the identical isolated QR workload via `--qr`; `--qr-stationary` explicitly enables test-only stationary decoding. It does not connect to the flight controller. |

## Important files

- `coverage_mission/qr_detector.py`: shared detector/selected-marker decoder. The earlier approach detector imports this implementation.
- `coverage_mission/qr.py`: configuration, isolated worker/service, inspection state machine and marker ledger.
- `coverage_mission/runtime.py`, `coverage_mission/engine.py`: integration with the existing supervision/map/control path.
- `world/integration/experimental_corridor_manager.py`: initial scan and reference/command/read-owner handoffs.
- `config/qr_mission.json`: provisional centering, rate, confirmation, timeout and descent limits.
- `world/integration/build_qr_assets.py`: reproducible texture generation and environment visual stripping; preserves the original collision asset.
- `world/integration/qr_gazebo_validation.py`: owned-process validation harness; never kills unrelated processes.
- `coverage_mission/evaluate_qr_run.py`: independent early-target evaluator. Full-field completion is deliberately not required after a match.
- `requirements/qr.txt`: pinned runtime decoder and fixture-generator packages. Runtime also needs `libzbar0`.

## Running the simulation

Run from the repository root, with Gazebo, the existing ArduPilot build and MAVProxy installed. No other simulator may occupy TCP 5760 / the default JSON physics ports. The harness owns and cleans up only processes it starts.

```bash
python3 -m pip install --user -r requirements/qr.txt
# If libzbar is missing: sudo apt install libzbar0
python3 world/integration/build_qr_assets.py
python3 -m world.integration.qr_gazebo_validation \
  --output world/integration/artifacts/qr_manual_run --seconds 600 --gui
python3 -m coverage_mission.evaluate_qr_run \
  world/integration/artifacts/qr_manual_run
```

Use a fresh output directory for each run. `--seconds` is the test harness watchdog, not a competition time-optimization policy. The camera/map GUI is shown with `--gui`; press the displayed mission quit key only to request an abort. This harness closes the simulator after the segment finishes. It does not run a return or payload sequence.

For the bounded missing-reference test:

```bash
python3 -m world.integration.qr_gazebo_validation \
  --output world/integration/artifacts/qr_missing_reference \
  --scenario missing-reference --seconds 150
```

The full manager defaults to the QR mission and a 5 m startup. Existing coverage-only regressions must explicitly pass `--coverage-only`; their previous 3 m takeoff remains available with `--takeoff-altitude 3`. A reference may be injected only into the explicitly isolated runtime CLI, not the integrated manager.

## Validation record

The GUI-enabled integrated mission in `world/integration/artifacts/qr_full_20261008_05` passed the independent early-target evaluator. Sequence: autonomous takeoff → measured startup advance → settled initial reference → banner/corridor → field inspection of a nonmatch → coverage resumption → matching marker → cancellation of remaining coverage → descent and verified `TARGET_HOLD_5M`.

| Measured result | Evidence |
|---|---|
| Reference and field identity | `REF-001`, confirmed on three different approved exposures at both phases; exact equality. A different field marker decoded `OTHER-001` and was rejected. |
| Target alignment | Maximum independent horizontal error during terminal dwell: **0.04281 m**. |
| Terminal height | FC HOME-relative altitude is within 0.15 m of 5 m. Independent model height above its median settled pre-takeoff datum: **5.023–5.127 m** throughout the sampled terminal window. The datum method is a simulator oracle, not a real altitude calibration. |
| Terminal motion | Maximum independent finite-difference horizontal/vertical speed: **0.09509 m/s**, below the unchanged 0.10 m/s limit. The evaluator uses actual baselines of at least 50 ms so adjacent 60 Hz samples are not inadvertently discarded. |
| Safety | Zero sampled field red incursions and zero sampled fence violations, including the descent. Maximum truth sample gap: **0.019 s**. Sampling is not a continuous-time safety proof. |
| Cancellation | **3,684 coverage points** remained; no sweep/repair or further inspection resumed after `TARGET_MATCH`. |
| GUI / terminal result | GUI enabled; manager exit 0; `TARGET_HOLD_5M`. No delivery, return or successful-terminal landing. Owned test processes were stopped. |
| Missing-reference variant | `qr_missing_20261008_01`: healthy camera but reference print removed. Bounded approximately 1 m advance, deadline abort, no corridor/coverage invocation, confirmed LAND. Harness marked the expected failure as passing. |
| Offline / subprocess checks | **101 automated tests passed** in the final QR/coverage/corridor-handoff/camera/command regression selection. QR checks cover masks/versions, URLs, selected-marker-only decoding, clipping/stale/repeated/conflicting observations, motion and deadline rejection, nonmatch resumption, finite unreadable attempts, red exclusion, descent safety, cancellation, truthful search exhaustion, bounded cold startup and actual suspended-worker recovery. |
| Desktop QR workload, not Pi performance | Field worker: 122 admitted jobs, 6 authorized decode jobs, zero restarts; peak processing **22.95 ms**, peak submission-to-result **26.75 ms**. QR discovery/decoding stopped after matching. These are one desktop run's maxima, not guaranteed hardware budgets. |

Run artifacts retain `.qr.jsonl`, `.jsonl`, `.supervision.jsonl`, `.result.json`, independent `truth.jsonl`, fixture/config manifests, and `qr_evaluation.json`. The evaluator is separate from controller perception and uses fixture identities only as test truth. No configured marker coordinates or world model identities enter mission control.

This is a nominal Gazebo acceptance checkpoint plus focused failure/offline evidence, not a claim that every QR/red/noise combination has flown. Red-excluded markers and native-worker faults are covered by focused offline/subprocess checks rather than another long full-world campaign. The separate cold-start readiness extension is subprocess-tested; real Pi startup latency and all performance budgets remain provisional. No additional full-field flight was run after the verified early-target pass.

Retained development runs are not silently counted as passes:

1. `_01`: startup stopped on an unregistered asynchronous QR log channel. Registered the QR channel and tested it; failed startup authority handback was corrected.
2. `_02`: actual rendered startup QR decoded and confirmed, but the corridor manager held stale heartbeat/EKF receipt times after the QR runtime had drained telemetry. Added bounded zero-motion telemetry handback.
3. `_03`: startup, corridor and field nonmatch inspection/resumption worked. Stopped deliberately when the supposed early target was found to be outside the actual first sweep lane. Adjusted only the test marker placement to make early-match integration reproducible; did not change coverage routing to chase fixture coordinates.
4. `_04`: functional full mission with GUI reached verified FC 5 m hold, but the independent recorder selected the wrong model name and captured no pose trace. It is **not** the independent validation pass. Corrected the recorder name and added a no-truth-samples preflight gate before `_05`.

The first evaluator pass over `_05` also exposed an evaluator-only sampling bug: every adjacent truth interval was shorter than its 20 ms derivative cutoff, leaving an empty velocity list. Corrected the finite-difference baseline and unit-tested it without relaxing the 0.10 m/s limit or rerunning a flight. All evaluator checks then passed.

## Deliberately still pending

Final competition payload semantics, arbitrary binary encoding, print dimensions/density, duplicate-target rules and red-zone placement policy require organizer clarification. Exact text identity is provisional.

Real lens/crop intrinsics, distortion/mount calibration, optical readability at 10 m, exposure/lighting, concurrent Pi CPU/latency/thermal measurements, real FC synchronization and real LiDAR/backend integration still require hardware/site work. Desktop processing metrics are not Pi measurements. The Gazebo-only manager must not be flown on hardware.

No return, payload release, URL fetching, AI inference, dynamic geofence input system or final lens selection has been added.
