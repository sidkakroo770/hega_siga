# Full mission: corridor to coverage (Gazebo)

`experimental_corridor_manager.py` now runs the original banner/approach/LiDAR
corridor stages unchanged through `EXIT_DETECTION`. On `CORRIDOR_EXITED` it
advances north into the registered field inset, climbs to 10 m HOME-relative
altitude, and hands the same MAVLink connection to the copied, validated
`coverage_mission` runtime. The manager sends no velocity commands during
coverage. Coverage completion ends this mission segment in a position hold;
there is no return, delivery, QR logic or automatic landing.

The fixed full-world field is local NED N=[-20,20], E=[-15,15]. This is
**not** spawn-relative: live SITL reported N=-30.4 at the banner, matching
Gazebo world Y=-30.4, and N≈-19.4 at corridor exit. The two red test zones
are N=[-9,-6], E=[-4,-2] and N=[8,11], E=[6,8]. The field height and
HOME/LOCAL-Z offset were checked in the full Gazebo world, not copied blindly
from the isolated fixture. The provisional downward camera is 640×480,
28.2° horizontal FOV at 15 Hz, 5 cm below the vehicle reference. The front
camera and LiDAR remain as before.

The original GLB also contains a red ground strip approximately
N=[-2.25,7.75], E=[-5.11,2.32]. It is detected by the downward camera and
can force a long, already-observed connector around both red areas. The
full-world config permits up to 120 source seconds without new map/traversal
credit for that connector; camera, clock, worker and localization watchdogs
remain much shorter. A live first run stopped at the isolated fixture's
40-second progress limit while safely routing around these areas, so the
full-world completion claim required a new run with this corrected profile.
That retry traversed 2,084 required points without a sampled red or fence
incursion, but hit its 3,600-second wall limit with 1,138 points still pending.
Independent truth confirmed projection and tracking remained within their
specified limits; it did **not** confirm full-field coverage. The subsequent
planner change prioritizes nearby reachable coverage over a distant unknown
ordered point. It passed the synthetic regression suite, but a new full-world
flight is still required before claiming complete Gazebo validation.

In the later accelerated full-world run (`artifacts/fullworld_oct2_third`),
the controller reached 3,161 measured waypoints and the independent truth
evaluator found zero permissible coverage gaps, zero sampled red/fence
incursions, and a 0.135 m maximum projected-corner error. The controller
nevertheless ended `BLOCKED`: 54 points and 2,519 unseen cells lay inside a
non-red island entirely enclosed by mapped red, while one outer-edge point
was missed by 0.282 m against the old 0.24 m arrival radius. The updated
controller exempts only islands enclosed by *confirmed* red; it does not
credit those points as traversed. The full-world arrival radius is now
0.30 m (the configured uncertainty), without changing red clearance. These
completion changes pass unit/synthetic tests but **have not yet had a fresh
end-to-end Gazebo flight**. Do not label the controller `COMPLETE` based only
on the earlier independent geometry result.

## Run the full simulation

Use separate terminals. First, with the usual Gazebo/SITL dependencies installed:

```bash
export GZ_IP=127.0.0.1
export GZ_PARTITION=miss2_integrated
export GZ_DISCOVERY_MULTICAST_IP=239.255.0.7
export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/sid/ardupilot_gazebo/build
export GZ_SIM_RESOURCE_PATH=/home/sid/sae_mission2/world/models/models:/home/sid/ardupilot_gazebo/models
gz sim -r -v2 /home/sid/sae_mission2/world/worlds/miss2_full_world.sdf
```

Second terminal:

```bash
cd /home/sid/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris \
  --model JSON --no-mavproxy \
  --use-dir /home/sid/sae_mission2/world/integration/artifacts/full_sitl_smoke
```

Third terminal:

```bash
mavproxy.py --master=tcp:127.0.0.1:5760 \
  --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14552 --streamrate=20
```

At the MAVProxy prompt, after ArduPilot is ready, issue `mode guided`,
`arm throttle`, then `takeoff 3` one at a time. Wait until the aircraft
reaches roughly 3 m before starting the manager.

Fourth terminal:

```bash
export GZ_IP=127.0.0.1
export GZ_PARTITION=miss2_integrated
export GZ_DISCOVERY_MULTICAST_IP=239.255.0.7
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
export PYTHONPATH=/home/sid/sae_mission2:/home/sid/sae_mission2/approach:/home/sid/sae_mission2/corridor:/usr/lib/python3/dist-packages
python3 -u /home/sid/sae_mission2/world/integration/experimental_corridor_manager.py \
  --mavlink udpin:0.0.0.0:14552 --banner-loss-frames 5 \
  --coverage-max-wall-seconds 7200
```

The coverage GUI remains enabled. Logs and a machine-readable terminal
result are written under `world/integration/artifacts/coverage_runtime.*`.
The manager exits nonzero on abort.

For an independent pose trace, start this **before the manager** with the
same Gazebo environment and `PYTHONPATH=/home/sid/sae_mission2`:

```bash
python3 -m coverage_mission.truth_monitor \
  --config /home/sid/sae_mission2/config/full_mission_coverage.json \
  --zone -9 -6 -4 -2 --zone -2.25 7.75 -5.11 2.32 \
  --zone 8 11 6 8 \
  --model-name iris_miss2_full \
  --pose-topic /world/miss2_world/pose/info \
  --ground-z 0.0731022 \
  --output /home/sid/sae_mission2/world/integration/artifacts/full_truth.jsonl \
  --seconds 7400
```

After a completed trace, independent evaluation is:

```bash
PYTHONPATH=/home/sid/sae_mission2 python3 -m coverage_mission.evaluate_run \
  --mission /home/sid/sae_mission2/world/integration/artifacts/coverage_runtime.jsonl \
  --truth /home/sid/sae_mission2/world/integration/artifacts/full_truth.jsonl \
  --output /home/sid/sae_mission2/world/integration/artifacts/full_evaluation.json
```

This is still a Gazebo adapter: it uses Gazebo image/clock topics and a
provisional camera geometry. The Pi camera source, hardware time alignment,
intrinsic/mount calibration and on-device profiling remain required before
real flight.
