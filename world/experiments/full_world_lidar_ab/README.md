# Full-world LiDAR A/B test

> **Superseded experiment: read [DIAGNOSIS.md](DIAGNOSIS.md) first.**
> The old top/bottom pose is outside corridor range, and its static fixed joint
> independently breaks a box-wall control. The claims below about omitting all
> cameras and interpreting mount-height results are incorrect: the base model
> includes a camera. Keep these commands only as historical context.
>
> The corrected stationary comparison, requiring no SITL or flight commands:
>
> ```bash
> cd ~/[competition]_mission2/world/experiments/full_world_lidar_ab
> source ./env.sh
> python3 diagnose_winding.py original --no-joint
> python3 diagnose_winding.py inward --no-joint
> ```
>
> Each command starts/stops its own headless server on a unique partition/topic.
> Results are in `artifacts/winding_diagnosis/{original,inward}_no_joint/`.
> Expected confidence: original 0; reversed triangle winding approximately 0.882.
> Production models remain unchanged. `--no-joint` is for this static probe;
> do not remove the dynamic flight model's joint.

This is a stationary, read-only test of the Mission 2 environment mesh at the
existing `miss2_preentry_test` X/Y pose: `0, -65.50`. That test pose predates
this A/B harness; it is not yet verified to be the exact camera-handoff pose.
The probe body is held static at world Z=2.90 so it cannot fall without SITL.
No MAVLink connection, SITL process, camera state, or velocity command is used.

The only difference between the variants is the vehicle model:

- `top`: an Iris body with the integrated model's LiDAR at model Z=+0.18 m.
- `bottom`: the same Iris body with the full model's LiDAR at model Z=-0.25 m.

The probe models deliberately omit the ArduPilot plugin and cameras. That keeps
this measurement independent of SITL's shared physics port and of camera state.

Run these sequentially after closing all other Gazebo simulations:

```bash
bash ~/[competition]_mission2/world/experiments/full_world_lidar_ab/probe.sh top
bash ~/[competition]_mission2/world/experiments/full_world_lidar_ab/probe.sh bottom
```

Each command exits after capturing one scan. Compare:

```bash
jq '{confidence, strict_valid, width, sectors, left, right}' \
  ~/[competition]_mission2/world/experiments/full_world_lidar_ab/artifacts/top/live_geometry.json
jq '{confidence, strict_valid, width, sectors, left, right}' \
  ~/[competition]_mission2/world/experiments/full_world_lidar_ab/artifacts/bottom/live_geometry.json
```

Interpretation:

- Top valid, bottom invalid: the full vehicle's lower LiDAR mounting is the
  cause at this pose.
- Both invalid: the full environment collision mesh or this physical probe
  pose cannot provide two wall fits.
- Both valid: the failure depends on the actual camera-to-LiDAR handoff pose,
  altitude, or moving vehicle state. Capture that exact state next; do not
  repeat the whole mission blindly.

