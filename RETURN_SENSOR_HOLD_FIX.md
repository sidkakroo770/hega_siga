# Return staging sensor hold and camera lifecycle correction

Date: 2026-10-08. [competition] mission.

## Failure and correction

The retained manual run aborted at N=−19.60325, E=4.10327, HOME altitude 2.30 m with `outside operational fence`. The ordinary field inset begins at approximately N=−19.229, but this aircraft position is inside the configured orange entrance region and passes its saved-map clearance checks.

The entrance allowance previously depended on each individual front-camera/LiDAR approach proposal. Missing or stale proposals, or a fresh sensor-hold proposal with `entrance_permit=False`, could remove the allowance while the aircraft was already legitimately staging near the mouth. The engine then applied the ordinary field fence and aborted. The manual logs do not identify which individual sensor/proposal freshness condition triggered the final update.

`coverage_mission/return_mission.py`, `ReturnNavigator`, now retains admission for the active approach after a fresh explicitly authorised entrance proposal. Missing/stale/unauthorised subsequent proposals produce zero-motion HOLD while retaining the narrow geometric allowance. Initial acquisition/centering before admission continues to use the ordinary field fence.

The allowance still requires the configured entrance position, width, altitude and heading on every update. It does not widen the field fence globally, override red or unknown ground, permit old commands, or reset residence timers. Source/host freshness checks and finite approach deadlines remain active. Explicit aborts still abort.

## Front camera and orange-mask display

`world/integration/experimental_corridor_manager.py` suspends forward-image acquisition after outbound `CORRIDOR_EXITED`, i.e. completion of exit detection, and closes the green preview. It unsubscribes from the forward image topic, clears cached frames/timestamps and rejects in-flight callbacks using a capture generation. No front-image decoding continues in the mission during the coverage/QR/transit phases.

The return approach lazily resumes the same front-camera subscription when `RETURN_FRONT` starts and waits for newly received frames. Old cached green-camera frames cannot be reused. The clock-continuity gate is preserved across this deliberate acquisition gap. LiDAR and downward-camera acquisition continue independently.

Orange detection already generated the same inset mask as green but used the shared, generically titled preview service. `PreviewService` now supports restart; the green window closes and an `Orange Banner Camera — MASK` window opens for return acquisition. Its inset is explicitly labelled `ORANGE MASK`. Preview generation/display remains optional and separate from command authority. This window and mask were observed live and captured in the evidence below.

This turns off **mission subscription, decoding and preview** during the unused interval. The Gazebo sensor's configured rendering/physical-camera power is not controlled by that subscription. When the IMX296 driver replaces the Gazebo adapter, connect the same lifecycle to camera-driver stop/start and measure restart/exposure settling on hardware.

## Validation

- **175 selected regression tests passed**, including new cases for missing, stale and falsely moving unauthorised staging proposals; retained red/geometry constraints; disabled callbacks; cache clearing; idempotent unsubscribe/resubscribe; and orange preview/mask restart.
- Replaying the retained failed position and map with a missing front proposal now gives `HOLD`, reason `Waiting for fresh orange approach proposal`, with all three velocity components zero. The ordinary field fence and entrance clearance were not changed.
- A GUI-enabled targeted return Gazebo flight passed **9/9 independent checks**, including positive orange entry readiness, reverse native corridor exit, red clearance, roof/wall clearance, independent exterior touchdown and fresh FC on-ground/disarmed confirmation.

Evidence: `world/integration/artifacts/return_hold_fix_20261008_02/`.

| Measurement | Result |
| --- | --- |
| Manager exit | 0 |
| Coverage-runtime endpoint | `RETURN_ENTRY_READY` |
| Full return endpoint | `COMPLETE`, LAND confirmed, on ground and disarmed |
| Landing local N/E | −32.19341 / 4.13050 m |
| Landing HOME altitude | −0.069 m |
| Maximum independent truth sample gap | 0.019 s |
| Minimum model-origin roof gap through corridor | 0.49967 m |

`orange_preview.png` retains the observed orange-mask display. `manager.log` records forward suspension, return resumption, staging descent, positive entry readiness and confirmed landing.

The user's existing simulator occupied TCP 5760. It was left untouched. The harness now accepts a separate owned `--instance`, which selects distinct SITL TCP/JSON/MAVProxy ports and a Gazebo partition. The successful test used instance 1. The earlier `_01` directory records preparation only: launch was refused before flight because port 5760 was occupied. All processes owned by the successful validation have stopped; the user's earlier simulator remains theirs.

This is a targeted return validation with an injected fixture reference; it skips initial QR acquisition and outbound corridor/coverage. Missing/stale front proposals were exercised offline against the actual navigation/engine classes and saved failure map. No sensor fault was deliberately injected into this live Gazebo flight, and no new complete start-to-landing flight was performed.

## Reproduce

From the repository root, choose a fresh output directory and unused instance:

```bash
python3 -m world.integration.qr_gazebo_validation --scenario return-entry --instance 1 --seconds 320 --gui --output world/integration/artifacts/return_hold_manual_03
python3 -m coverage_mission.evaluate_return_run world/integration/artifacts/return_hold_manual_03
```

Normal full-mission commands remain unchanged. Hardware restart behaviour, camera timing and Pi throughput remain to be measured.
