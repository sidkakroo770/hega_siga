# QR false-candidate correction and validation

Date: 2026-10-08. Applies to the [competition] field QR mission.

## Confirmed failure

The user's full run is retained under `world/integration/artifacts/full_near_red_manual_01/`. Initial reference acquisition succeeded; the field runtime correctly read `OTHER-001`, then repeatedly selected spurious ground detections. There were 14 candidate IDs at shutdown, 3,097 logged `QR_CENTER` decisions and no matching target confirmation. The run ended by keyboard interruption.

The detector accepted polygons based on finite coordinates and area alone. For example, a logged candidate had corners `(345,477), (213,317), (637,-394), (637,329)` in a 640×480 image. It was marked incomplete but nevertheless projected onto the ground, entered the candidate ledger and became eligible for centering. Reading required a complete live marker, so this candidate could not finish its inspection. Timers bounded individual attempts, but different false IDs bypassed the intended two-attempt limit across the scene. Stale candidates could also be selected without fresh evidence.

Clock error during the original field trace remained between 0.002 and 0.01842 s, within the clock gate. This was a detection/selection/liveness failure, not evidence that 0.005–0.006 s clock variation should be corrected or its threshold weakened.

## Implemented changes

| Location | Correction | Purpose |
| --- | --- | --- |
| `coverage_mission/qr_detector.py`, `QRDetector.valid_quad()` / `candidates()` | Require all corners inside the image margin, a convex quadrilateral, sufficient side length, bounded side ratio and broad corner-angle limits | Reject extrapolated corners, triangles, thin strips and heavily distorted finder geometry before projection and debug drawing |
| `coverage_mission/qr.py`, `Inspection.ingest()` | Reject incomplete/nonfinite observations; count distinct, consecutive, spatially stable exposures | A single unstable appearance cannot acquire navigation authority |
| `Inspection.step()`, selection | Require fresh observations and three stable exposures; honour candidate retry and global resume times | Prevent stale candidates and new false IDs from immediately interrupting coverage |
| `Inspection.failed_attempt()` / `step()` | Release a selected marker after 1 s without observations, or 5 s without meaningful centering progress; retain total inspection/read deadlines | Prevent prolonged zero-motion centering or ineffective pursuit |
| Failed attempt recovery | Candidate cooldown 10 s, field search resumes for at least 3 s, existing two-attempt bound retained | Make room for coverage and fresh rediscovery instead of immediate repeated inspection |
| Centering progress | Evaluate remaining route length when a detour exists | Moving around red can temporarily increase direct distance without being falsely classified as no progress |
| `config/qr_mission.json` / `QRConfig` | Explicit conservative discovery, geometry and recovery parameters | Keep hardware-dependent tuning visible and adjustable |

Provisional defaults: three discovery exposures, 0.20 m spatial stability, 3 px image margin, 8 px minimum side, maximum side ratio 1.8 and corner angles 45–135 degrees. These broad geometry gates apply to the existing near-level downward-camera operating envelope; they do not assume a QR version, physical print size, focal length or payload. Payload decoding still requires stationary, centered, explicitly authorised observations, and a match still requires three distinct decoded exposures.

The existing camera acquisition, bounded queues, one-in-flight QR worker, red safety hierarchy and single leased command sender are retained. Geometry checking operates on four points; no additional full-image filtering, model or reconstruction was added. The QR worker receives the configured limits from `QRService.submit()`.

## Verification

### Failed-run geometry replay

`coverage_mission/replay_qr_candidates.py` rechecks the actual retained quadrilaterals:

- 154 logged observations examined.
- 83 invalid quadrilaterals rejected.
- 71 genuine marker observations retained; every retained projected centre lies within 0.30 m of a physical fixture marker.
- All three actual stationary `OTHER-001` payload exposures retained.

Result: `world/integration/artifacts/full_near_red_manual_01/qr_geometry_fix_replay.json`, PASS.

This is geometry replay, not a rerun of OpenCV on saved pixels or a simulation of the changed trajectory. An exploratory state replay is retained as `qr_fix_replay.json`: it failed to reproduce the nonmatch confirmation. That experiment advanced the inspection only at recorded QR-result times, changed selection timing and could not reconstruct the actual faster worker/command delivery sequence. It is not used as closed-loop proof. The live Gazebo test below supplies that evidence.

### Automated regressions

The selected regression suite passed **168 tests**, including 20 new cases in `coverage_mission/test_qr_false_candidates.py`. Cases cover actual malformed failed-run polygons, triangular/crossed/thin/skewed geometry, rotated valid markers, QR versions, incomplete observations, duplicate exposures, missed-frame confirmation reset, unstable positions, lost target release, fresh rediscovery, cooldown across different candidate IDs, no centering progress and exhausted retries.

Existing reference immutability, stationary read eligibility, target descent, red safety arbitration, return behaviour and command/handoff checks passed. Existing test helpers now supply the three distinct discovery exposures required by the corrected policy.

### Gazebo approach across the actual textured ground

The owned `qr-textured-approach` fixture starts after the corridor at N=−18.2, E=−4, with fresh autonomous takeoff to 10 m and an injected test reference. It retains all production QR markers and the unchanged grass/red world. It travels through the area that caused the user's failure, inspects the real nonmatch at N=−17, E=−4, then finds the matching target at N=−10.2, E=−3 beside red.

GUI diagnostics were enabled. Evidence: `world/integration/artifacts/qr_textured_fix_20261008_01/`. Independent evaluator: `coverage_mission/evaluate_qr_red_run.py`. **18/18 checks passed; manager exit 0; terminal state `TARGET_HOLD_5M`.**

| Measurement | Result |
| --- | --- |
| Field source duration | 95.19 s |
| Independent trajectory samples / maximum gap | 5,757 / 0.018 s |
| Physical body red incursions / field fence violations | 0 / 0 |
| Minimum physical body-to-red clearance | 0.790 m |
| Selected inspection events | 3, all belonging to the two real markers |
| Matching and nonmatching payload decode jobs | 6 total; three reads for each |
| Worst terminal horizontal error from actual target centre | 0.0693 m |
| Worst terminal independent speed | 0.0945 m/s |
| Distinct mapped exposures during target descent | 259 |
| Shared mapping/QR exposure timestamps | 224 |
| QR worker restarts / diagnostic record loss | 0 / 0 |
| Mapping processing median / p95 on development computer | 5.66 / 8.69 ms |
| QR processing p95 / peak request-to-result wall time | 20.08 / 38.66 ms |

One real nonmatching-marker loss occurred during its first approach. The runtime released it promptly, resumed coverage, reacquired fresh observations after cooldown and confirmed its identity on the second attempt. The independent checks require failed inspections to correspond to real markers and respect attempt bounds; they do not incorrectly label a bounded recovery as a false-grass inspection. No false candidate was selected and no QR-centering churn occurred over the grass.

Red observations continued during descent, three stationary matching exposures were verified, coverage did not resume after matching, and the true target alignment, 5 m altitude and stationary endpoint met unchanged limits. The camera/pose clock varied normally, including around 0.005–0.006 s.

## Scope and remaining validation

This test validates the affected field approach, candidate lifecycle, actual camera decoding and descent near red. It intentionally skips initial QR acquisition, corridor traversal and return; a complete start-to-landing flight was not repeated. Existing offline reference/return/corridor checks passed. The previous full-mission success record predates this correction and must not be presented as a new complete-flight result.

Pi execution time, real image calibration/exposure and outdoor readability remain to be measured. The listed CPU timings are development-computer worker timings, not Pi throughput or total frame-to-actuation latency. All owned simulator/test processes from this validation have been stopped.

## Reproduce the targeted live regression

From the repository root, with no simulator already occupying port 5760, choose a fresh output directory:

```bash
python3 -m world.integration.qr_gazebo_validation --scenario qr-textured-approach --seconds 220 --gui --output world/integration/artifacts/qr_textured_manual_02
python3 -m coverage_mission.evaluate_qr_red_run world/integration/artifacts/qr_textured_manual_02
```

The regular full-mission launch commands still use the normal initial reference and full return/landing. They do not use the private test reference injection.
