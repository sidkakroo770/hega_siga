# Pi 5 + IMX296 integration handoff

## Return extension checkpoint — 2026-10-08

The full mission now includes a return implementation; see
`../../docs/return/mission_status.md` for current test evidence.
It reuses the map worker, bounded sensor history and leased sender, and releases
QR decoding after match. Entrance, exterior landing and colour profiles remain
Gazebo/site assumptions. This does not change the hardware-flight-ready status.

Status: camera acquisition and the dual-camera ground benchmark are implemented, but the full mission is **not yet hardware-flight ready**. The existing full-mission runner still subscribes to Gazebo camera and LiDAR topics. No physical Pi, IMX296 pair, LiDAR or flight controller was available in this development environment, so these numbers have not been measured on the target.

## Implemented now

- `pi_camera.py` opens two explicitly assigned IMX296 cameras through Picamera2, requests 640×480 BGR-compatible output at a provisional 15 fps, retains only a bounded recent-frame history, and exposes camera health. It rejects missing/regressing/implausibly old sensor timestamps rather than substituting Python receipt time.
- The exposure timestamp is converted from libcamera's boot clock to the host monotonic clock and shifted back by half the exposure duration. This is the timestamp to pair with flight-controller pose; receipt time is retained separately for watchdogs.
- `PiCoverageSensors` supplies the existing coverage runtime's frame/clock contract. `coverage_mission.runtime.main` accepts a supplied sensor adapter while keeping its Gazebo default unchanged.
- `pi_camera_benchmark.py` exercises both streams together and runs the existing banner/red-region colour processing. It reports camera counts, processing rate/cost, CPU, RSS, temperature, timestamp age, exposure, gain, crop and camera errors. It does not command the aircraft.
- Headless banner detection skips debug-frame copies and annotations; the Gazebo GUI path remains unchanged. The full-mission runner no longer copies each forward frame before detection because published frames are immutable to consumers.

## Run on the Pi, with propellers removed

Install the Raspberry Pi OS Picamera2 package and the project's Python dependencies in the Pi environment. First identify which numbered camera is front and which is downward; numbering can change between boots:

```bash
rpicam-hello --list-cameras
python3 -c 'from picamera2 import Picamera2; print(Picamera2.global_camera_info())'
```

From the `[competition]_mission2` repository root, substitute the observed indexes:

```bash
python3 -m world.integration.pi_camera_benchmark \
  --front-index 0 --downward-index 1 --seconds 120 \
  --output artifacts/pi_camera_benchmark.json
```

Repeat once with `--gui` to measure the diagnostic-display overhead separately. The default, no-GUI run is the relevant control-path baseline. Record the actual camera IDs, processing crop, lens, lighting, exposure/gain, OS/Picamera2 versions, power supply and cooling. Repeat in sunlight and shade, then with flight-controller and LiDAR communication running. The GUI must remain optional and must not be required for command timing.

## Provisional measurement gates, not final calibration

- Both streams should sustain their configured rate for the run without camera exceptions, progressive timestamp age, thermal throttling or prolonged frame gaps. `received` counts camera acquisition; `processed_fps` is how many frames the benchmark actually processed.
- The mission's current coverage frame-age gate is 0.20 s and its worker watchdog is 1 s. The Pi measurements must show adequate margin under concurrent load, not merely average values below those limits. If not, lower the acquisition/processing rate or revise the timing budget based on measured data.
- Validate lens/crop-specific downward intrinsics and mounting rotation before claiming metre-scale ground projection accuracy. The current 10 mm lens is provisional; the front-camera lens/FOV is unknown. Camera model and pixel count alone do not calibrate either lens.
- Determine the real LiDAR model, transport, scan format/rate, invalid-return semantics and time source. Do not map invalid returns to clear space by copying the Gazebo adapter.
- Determine the flight-controller connection/baud and test TIMESYNC quality, telemetry freshness, control authority and fail-safe behaviour on the Pi. Autonomous takeoff and the complete state sequence require restrained ground/HIL tests before propellers-on testing.

## Next integration boundary

### QR workload added — 2026-10-08

The same isolated QR perception worker can now be included in the ground-only
dual-camera benchmark. Install `requirements/qr.txt` and distribution
`libzbar0`, then add `--qr` to the benchmark command. Use
`--qr --qr-stationary` with a visible test print to include selected-marker
decoding; this is explicitly a synthetic-pose, stationary ground workload,
not permission to decode in translating flight. Results include admitted QR
jobs, decoder jobs, restarts, processing and result-latency peaks. Discovery
is provisionally 5 Hz; decoding is capped at 2 Hz. The worker has a separate
bounded cold-start readiness window and does not queue old frames during it.

QR identity, centering, timing and descent integration are documented in the
repository `docs/qr/mission_status.md`. Actual Pi performance,
10 m optical readability, final print/lens calibration and physical sensor
backend integration remain unmeasured. No AI model was added.

The hardware runner still needs a physical LiDAR adapter and a sensor-backend selection for the full-mission manager. `PiCoverageSensors` is an injection seam, not evidence that the entire mission can presently run on the Pi. Once the LiDAR and FC interfaces are known, connect both physical camera streams and LiDAR through that backend, keep one mission command authority, and re-run mission/fault tests with real timing. Do not use the Gazebo-only runner as a flight program.
