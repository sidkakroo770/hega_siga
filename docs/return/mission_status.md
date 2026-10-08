# Return Mission Implementation Status

Date: 2026-10-08

## Current checkpoint

Latest corrective checkpoint: staging admission survives sensor holds without
authorising motion, and the front feed/preview stops after outbound exit then
resumes with an explicit orange-mask window. 175 regressions, a failed-position
map check and a targeted return-to-landing Gazebo flight passed. See
[sensor_hold_fix.md](sensor_hold_fix.md). The complete
initial-QR-to-landing success recorded below predates this correction; that
entire flight was not repeated. Pi/hardware timing remains unmeasured.

## Implemented

- Default full-mission continuation after matching QR descent and a five-second continuous settled hold.
- The same live field map, red evidence, residence timer, camera-pose matching and command supervisor are retained during return.
- Bounded ascent to 10 m, checked goal/frontier routing, entrance-lane survey at 10 m, return to a front-camera stand-off and acquisition-height descent.
- Orange detection reuses the existing lightweight banner detector with a separate configurable colour profile.
- Camera approach proposals pass through field freshness, stopping-region and red-map checks.
- Narrow configured mouth admission replaces the field-only inset only during an explicitly authorized approach. It is not a global fence bypass.
- Positive repeated LiDAR wall-readiness, fresh read-owner handback and a new native corridor FSM instance for the return lap.
- Role-specific second corridor exit leading to exterior egress and normal FC LAND.
- Expected LAND/disarm distinguished from pilot takeover; completion requires fresh on-ground/disarmed feedback, low measured motion and low HOME-relative altitude.
- QR decoder released after match; no repeat QR search or sweep scheduler work.
- Return yaw is rate-limited below the projection angular-rate gate; safety HOLD retains the return heading. Invalid supervision stops yaw as well as translation.
- Return corridor faults cannot authorize an unverified landing underneath the roof. The ordinary abort stops output and requires the FC/operator contingency.
- Existing coverage-only and QR-only test endpoints remain explicit options.

## Additional geometry finding

A lower-altitude forward approach cannot necessarily see enough new ground to certify the existing 0.70 m clearance envelope: the configured downward footprint shrinks with altitude. Return first surveys the entrance lane from a safe interior viewpoint at 10 m, then backs out to the acquisition stand-off. This retains actual observations for the lower approach and avoids reducing clearance or assuming unobserved ground is clear.

## Evidence so far

| Check | Outcome |
| --- | --- |
| Selected coverage/QR/manager/native offline regression suite | 141 passed at the final implementation checkpoint |
| Focused return state/geometry/approach tests | Included in the final suite; distinct frames, stale data, bounded search, heading, region and touchdown/disarm gates covered |
| Saved-map return replay | Passed: ascent → entrance survey → stand-off route → acquisition descent → front approach admission |
| Replay source duration | 71.9 seconds, 720 synthetic samples |
| Replay evidence | `world/integration/artifacts/return_replay_20261008_01/result.json` |
| First full flight | Retained failure before return: outbound scan/pose freshness expired; supervisor stopped and confirmed LAND |
| First failure evidence | `world/integration/artifacts/return_full_20261008_01/` |
| Second full flight | Retained failure: safety HOLD restored outbound yaw during return ascent; pose-continuity supervision aborted |
| Second failure evidence | `world/integration/artifacts/return_full_20261008_02/` |
| Targeted return-only Gazebo flight, GUI enabled | Passed orange acquisition, positive wall handoff, reverse corridor, exterior LAND and on-ground/disarmed confirmation |
| Targeted flight evidence | `world/integration/artifacts/return_entry_20261008_01/return_evaluation.json` |
| Complete default mission, GUI enabled | Passed all 18 independent evaluator checks, including initial/target QR, five-second hold, return routing, both exits and landing |
| Full-flight evidence | `world/integration/artifacts/return_full_20261008_03/return_evaluation.json` |
| Full-flight hold | 5.0007545 measured source seconds; 296 independent truth samples |
| Worst independent target-hold position error | 0.04166 m, below the unchanged 0.25 m limit |
| Peak independent target-hold speed | 0.09683 m/s, below the unchanged 0.10 m/s limit |
| Independent truth maximum sampling gap | 0.019 s |
| Sampled red incursions / transit fence violations | Zero |
| Minimum return roof-to-model-origin gap | 0.49916 m; not a measurement of the physical aircraft's upper envelope |
| Final FC landing feedback | N -32.1934 m, E 4.1123 m; on-ground and disarmed |

Replay uses synthetic planar observations and a modest actuator lag. It proves map/routing contracts, not camera rendering, real dynamics or physical safety. No freshness, pose-integrity, target-alignment or speed threshold was relaxed to bypass the retained failures. The second failure was fixed by preserving heading authority and limiting return yaw, then proving the return-only fixture before repeating the full mission. Test processes were stopped after the successful full flight.

The full flight validates the current fixture. The existing specialized obstacle controller is reused; this pass does not claim a new reverse-direction physical obstacle-shape matrix or arbitrary landing-site certification.

## Desktop timing evidence

Across 3,227 logged return-worker samples in the full flight, processing time was median 1.341 ms, p95 7.795 ms and maximum 31.901 ms. These are desktop worker measurements, not Pi benchmarks, not the front-controller's separate readiness-fit cost, and not end-to-end exposure-to-actuation latency.

## Running the mission

From the repository root, use the owned-process harness:

```bash
python3 -m world.integration.qr_gazebo_validation \
  --return-mission --gui \
  --output world/integration/artifacts/return_manual_01 \
  --seconds 1000
```

Use a fresh output directory. The harness refuses an occupied SITL TCP port and stops only its own process groups; it does not kill unrelated user sessions. The seconds value is an execution watchdog, not a competition mission-time optimization.

Independent return evaluation, after a completed run:

```bash
python3 -m coverage_mission.evaluate_return_run \
  world/integration/artifacts/return_manual_01
```

For a shorter return-only fixture with fresh autonomous takeoff (not a permissive restart of the normal mission):

```bash
python3 -m world.integration.qr_gazebo_validation \
  --scenario return-entry --gui \
  --output world/integration/artifacts/return_entry_manual_01 \
  --seconds 700
```

The hidden manager fixture flag requires the owned harness environment and the registered stand-off position. Normal launches still require the initial QR and matching target before return.

The harness without `--return-mission` deliberately preserves the earlier QR-only endpoint. Direct manager launches default to the return mission; `--qr-only` stops at the matched 5 m hold and `--coverage-only` retains the coverage regression profile.

## Configuration and deployment limits

`config/return_mission.json` supplies replaceable local N/E entrance coordinates, heading, stand-off, orange thresholds and landing distance. Additional conservative limits live in the validated `ReturnConfig` defaults and can be supplied in the JSON.

The entrance/exterior strip and final landing region are surveyed-clear Gazebo operating envelopes. They require site registration and a verified flat clear landing region before physical use. They do not override a whole-mission geofence.

Camera HSV/FOV calibration, real timing/latency, LiDAR mounting/returns, as-built clearance, Pi dual-camera throughput/thermal behaviour and FC LAND/failsafe tests remain hardware/site work. No AI model or dense reconstruction was added.

The current milestone uses the orange return banner, a five-second delivery surrogate and exterior landing. Payload release and exact-home landing are not implemented.

## Important files

- `coverage_mission/return_mission.py`: bounded field return and narrow entrance permit.
- `world/integration/return_approach.py`: manager-owned front-camera/LiDAR proposal producer.
- `coverage_mission/runtime.py`: retained map/safety session and explicit return handoff.
- `world/integration/experimental_corridor_manager.py`: traversal dispatch, corridor reuse, exterior egress and landing.
- `coverage_mission/replay_return_map.py`: saved-map regression.
- `coverage_mission/evaluate_return_run.py`: independent truth evaluation.
- `coverage_mission/test_return_mission.py`, `world/integration/test_return_approach.py`: offline return contracts.
