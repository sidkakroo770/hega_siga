# Corridor obstacle Gazebo fixtures

These small static boxes are regression fixtures for the **existing full-world
non-ROS mission**, not claims about competition obstacle dimensions. Spawn
**one** into a fresh `miss2_full_world.sdf` instance before autonomous takeoff:

| Fixture | Expected corridor result |
| --- | --- |
| `left_wall_protrusion.sdf` | `AVOID_RIGHT`, SHIFT, PASS, recenter, `CORRIDOR_EXITED` |
| `right_wall_protrusion.sdf` | `AVOID_LEFT`, SHIFT, PASS, recenter, `CORRIDOR_EXITED` |
| `blocked_passage.sdf` | STOP/reassessment, bounded `ABORT_CORRIDOR`, LAND; **no bypass** |

With Gazebo running under the desired `GZ_PARTITION` and `GZ_IP`, spawn using:

```bash
gz service -s /world/miss2_world/create \
  --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 5000 \
  --req 'sdf_filename: "/home/sid/[competition]_mission2/world/integration/fixtures/left_wall_protrusion.sdf"'
```

Substitute the other fixture path for the other cases. Require `data: true`,
then run the normal full-world manager with `--start-mission`. Use an isolated
Gazebo partition and SITL instance/ports if another simulation is active.
In the 2026-10-05 tests, a copied Iris model used ArduPilot plugin port 9012,
SITL `-I1` used TCP 5770, and MAVProxy forwarded to UDP 14562; the user's
existing SITL `-I0` was not stopped. The copies and logs were kept under
`/tmp/[competition]-corridor-verify.0EQBVc`, `/tmp/[competition]-corridor-obstacle.7esG7F`,
`/tmp/[competition]-corridor-right.elAzVm` and `/tmp/[competition]-corridor-blocked.4Yhihp`
while those temporary directories remain.

The manager's coverage runtime was deliberately wall-time capped after the
corridor tests. Its later `ABORTED: Wall-time execution limit` is not a
corridor failure. One left-fixture run printed an intermittent native-thread
fatal error after the capped coverage runtime shut down; this post-coverage
teardown is not yet diagnosed. All isolated test vehicles were landed and
disarmed before their Gazebo/SITL processes were stopped.

