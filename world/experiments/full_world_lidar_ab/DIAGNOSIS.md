# Full-world corridor failure: measured diagnosis, 2026-09-21

## Finding

The full environment's two `corridor cover` meshes have zero-thickness walls
with outward-facing triangle winding. In the installed Gazebo Harmonic/Ogre2
GPU LiDAR pipeline, their interior faces do not produce wall returns. The
native controller therefore cannot fit both corridor walls.

A controlled rendered-scan comparison demonstrates this: changing ONLY triangle
index order in a copy of the original GLB restores both wall fits. Material,
positions, normals, probe pose, sensor and controller settings are identical.
Reversing normals as well gives the same result. The GLB material already has
`doubleSided: true`; it did not make the interior detectable in this pipeline.
Which importer/rendering stage fails to honor that flag was not traced.

The isolated corridor uses solid SDF box walls. It supplies valid wall returns,
so the same controller works. The user's successful isolated flights, including
the 2 m stand-off/yaw test, do not validate the full-world mesh or handoff.

## Evidence

Controls use a 500-beam horizontal scan, 12 m range, the existing native
`PreEntryController`, and no MAVLink or flight commands.

| Input | Finite beams | Left/right wall candidates | Confidence | Strict valid |
| --- | ---: | ---: | ---: | --- |
| Actual failed mission capture | 86 | 5 / 5 | 0 | false |
| Original mesh, corrected stationary probe | 87 | 6 / 5 | 0 | false |
| Same mesh, triangle winding reversed ONLY | 210 | 88 / 88 | 0.881613 | true |
| Same mesh, winding and normals reversed | 210 | 88 / 88 | 0.881613 | true |
| Original mesh + box wall visuals, bare sensor | 210 | 88 / 88 | 0.881614 | true |
| Box walls alone, corrected stationary probe | 193 | 83 / 83 | 0.849583 | true |

The controller requires 14 wall inliers per side; the failed scan cannot meet
that even before RANSAC. Modified-mesh scans measure width 3.751 m, wall spans
3.855/3.949 m and RMS about 0.008 m. Width fits the unchanged 3.5 +/- 0.45 m
strict configuration. No confidence threshold was relaxed.

Artifacts relative to this directory:

- `artifacts/winding_diagnosis/original_no_joint/{report.json,scan.npz,world.sdf}`
- `artifacts/winding_diagnosis/inward_no_joint/{report.json,scan.npz,world.sdf}`
- `artifacts/winding_diagnosis/inward_normals_no_joint/report.json`
- `artifacts/winding_diagnosis/box_control_bare/report.json`
- `artifacts/winding_diagnosis/boxes_only_no_joint/report.json`
- `artifacts/winding_diagnosis/comparison.png`: rendered-scan comparison.
- `artifacts/winding_diagnosis/failed_mission.log`: preserved mission log.

Original flight evidence: `../../integration/artifacts/preentry_capture/`.
Its `replay/scan_report.json` and `replay/scan_plot.png` were regenerated from
`preentry_scan.npz`. Successful isolated 2 m/yaw scan:
`../corridor_only/artifacts/flight/live_scan.npz`.

## Pose and geometry

First corridor world bounds from GLB vertices and link +90 degree X rotation:
X [-5.869633,-2.119633], Y [-30.109434,-20.109434], Z [-0.004763,3.05] metres.
The interior width is 3.75 m.

Saved local NED pose: X=-30.445704, Y=-4.003409, Z about -2.61 m,
yaw=0.010378 rad. The probe infers world XY/yaw from the configured ENU/NED
transform: X=-4.003409, Y=-30.445704, yaw=1.560418 rad. Body Z is controlled
at 2.61 m and LiDAR at 2.36 m. EKF-origin altitude is not guaranteed to equal
world altitude: this is near the inferred handoff pose, not an exact measured
world-pose replay. LaserScan's all-zero/identity world pose is not usable truth.

Each cover has 12 triangles: six nondegenerate (two per wall and roof), six
degenerate. Side normals point away from the interior, roof normals upward.
Gazebo MeshManager loads these submeshes; the GLB is not wholly missing.

## Why Terminal 4 ended

The log shows normal failure handling, not a Python crash: approach front
range reached about 0.49 m, descent and two-second hover completed, then
PRE_ENTRY had confidence zero despite fresh scans. LOW_CONFIDENCE_GEOMETRY
led to reassessment, timeout, ABORT_CORRIDOR, LAND requested, STOP, and manager
shutdown. The log proves a LAND request, not a verified completed landing.

More timeout or a lower image-area threshold cannot create missing wall
returns. Fresh scans prove message receipt, not usable geometry. `No
subscribers` before starting the manager is not a broken-publisher diagnosis.

## Unsuccessful attempts and diagnostic pitfalls

1. Old `probe.sh top/bottom` used `(0,-65.5)`, far beyond the 12 m range of
   the corridor entrance near `(-4,-30.109)`. Zero returns there do not identify
   a mounting-height problem.
2. The static probe initially kept its fixed joint to nested Iris. With that
   joint, even box-only controls returned zero beams. Removing ONLY that joint
   restored 193 returns with the body and camera present. The precise static
   joint/render-pose mechanism was not traced. Do NOT infer that the dynamic
   flight joint should be removed. Initial body self-obstruction suspicion
   was not established.
3. Removing only the inherited camera did not fix the static-joint result.
4. Both covers share index accessor 18. The first reversal implementation
   flipped it twice, undoing itself. The script now de-duplicates accessor IDs.
   Early `inward/`, `inward_normals/`, `inward_normals_bare/` reports are
   superseded, invalid negative tests. Use final `*_no_joint/` reports above.
5. Lowering the probe or adding boxes with the bad static joint still present
   did not help. Those results are confounded by the joint.
6. GLB loading prints metallic/roughness channel extraction errors. Reversed
   winding still produces valid geometry without changing materials, so those
   warnings were not necessary to fix for this diagnosis.

## Repair applied and current state

Added `diagnose_winding.py`: generates separate experiment assets, collects
bounded static scans, saves reports and stops its own server process group.
Added `inspect_loaded_mesh.cc`: uses Gazebo MeshManager to inspect imported
submeshes/materials. `repair_corridor_mesh.py --apply` was then run on the
production `models/models/miss2_env/miss2.glb`.

The repair retains each original outward primitive and adds an otherwise
identical primitive whose triangle winding is reversed. It changes neither
wall positions, corridor dimensions, material/color, banner, vehicle model,
controller, launch scripts nor mission FSM. The production mesh grew by
444 bytes. Its pre-repair copy is
`models/models/miss2_env/miss2.glb.before_two_sided_corridor_repair`.

The post-repair `original --no-joint` verification passed with 210 finite
beams, 88 candidates per wall, confidence 0.881613 and strict validity true.
The currently running full-world Gazebo server was started before the file
change, so it still has the old mesh. It must be restarted before full-flight
verification. No SITL movement commands were sent in this work.

At final process check the original full-world Gazebo server/gui and ArduCopter
SITL remained running; all diagnostic servers had stopped. Recheck processes
next session; do not assume PIDs persist.

World Git: `main`, HEAD `79fd525`. Existing experiments/integration are mostly
untracked; GLB already dirty from recoloring. Corridor repo remains clean.
No commits, branch switches or resets made.

## Next Session

Read this report and README; inspect final original/inward `_no_joint` reports.
Working directory: `/home/sid/[competition]_mission2/world`. The two README commands
reproduce the diagnosis without SITL, arming or a flight.

The full mesh now has inward and outward corridor faces. Restart Gazebo to load
it and test full approach, 0.5 m handoff, 1 m descent,
2 s hover, PRE_ENTRY, entry, cruise and exit. Full-world flight success is
still unverified. Preserve freshness checks, confidence gates and abort rules.

