#!/usr/bin/env python3

import os

# Must be set before protobuf / Gazebo message imports.
os.environ.setdefault(
    "PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION",
    "python",
)

import time
import numpy as np

from gz.transport13 import Node
from gz.msgs10.laserscan_pb2 import LaserScan

from native.common.types import NativeScan
from native.mission_runner import NativeMissionRunner


TOPIC = "/iris/lidar/scan"

latest_scan = None
scan_sequence = 0


def lidar_callback(msg):
    global latest_scan
    global scan_sequence

    ranges = np.asarray(
        msg.ranges,
        dtype=np.float64,
    )

    if ranges.size == 0:
        return

    angles = (
        float(msg.angle_min)
        + np.arange(
            ranges.size,
            dtype=np.float64,
        ) * float(msg.angle_step)
    )

    try:
        intensities = np.asarray(
            msg.intensities,
            dtype=np.float64,
        )
    except Exception:
        intensities = np.empty(0)

    if intensities.size != ranges.size:
        intensities = np.zeros(
            ranges.size,
            dtype=np.float64,
        )

    latest_scan = NativeScan(
        angles_rad=angles,
        ranges_m=ranges,
        intensities=intensities,
        timestamp=time.monotonic(),
        range_min_m=float(msg.range_min),
        range_max_m=float(msg.range_max),
    )

    scan_sequence += 1


print()
print("==========================================")
print(" SAE MISSION 2 - FSM READ-ONLY TEST")
print(" NO MAVLINK CONTROL COMMANDS ARE SENT")
print("==========================================")
print()

node = Node()

result = node.subscribe(
    LaserScan,
    TOPIC,
    lidar_callback,
)

print(
    f"[GAZEBO] subscribe({TOPIC}) -> {result}"
)

if not result:
    raise SystemExit(
        "[ERROR] Could not subscribe to LiDAR."
    )


# ------------------------------------------------------------
# This is the ACTUAL native corridor mission runner.
#
# We intentionally use its default config.
# enter_corridor_distance_m remains None.
#
# Therefore even if PRE_ENTRY succeeds, ENTER_CORRIDOR
# will command STOP rather than actually proceed.
# ------------------------------------------------------------

runner = NativeMissionRunner()

print()
print(
    "[SAFE] BodyVelocity outputs will only be PRINTED."
)
print(
    "[SAFE] Nothing in this script opens MAVLink."
)
print()


last_sequence = -1
last_print = 0.0
start_time = time.monotonic()


try:
    while True:

        time.sleep(0.01)

        if latest_scan is None:
            continue

        if scan_sequence == last_sequence:
            continue

        last_sequence = scan_sequence

        # ----------------------------------------------------
        # REAL FSM STEP
        # ----------------------------------------------------

        output = runner.step(
            scan=latest_scan,
            attitude=None,
            pose=None,
        )

        now = time.monotonic()

        if now - last_print >= 0.5:

            last_print = now

            cmd = output.command
            state = runner.public_state()

            print()
            print(
                f"[FSM] state={state.value}"
            )

            print(
                f"      status={output.status}"
            )

            if output.reason:
                print(
                    f"      reason={output.reason}"
                )

            if output.confidence is not None:
                print(
                    f"      confidence="
                    f"{output.confidence:.3f}"
                )

            print(
                "      WOULD COMMAND:"
            )

            print(
                f"        vx="
                f"{cmd.vx_m_s:+.3f} m/s"
            )

            print(
                f"        vy="
                f"{cmd.vy_m_s:+.3f} m/s"
            )

            print(
                f"        vz="
                f"{cmd.vz_m_s:+.3f} m/s"
            )

            print(
                f"        yaw_rate="
                f"{cmd.yaw_rate_rad_s:+.3f} rad/s"
            )

            print(
                f"      scan_age="
                f"{latest_scan.age_s:.3f}s"
            )

        # Useful explicit milestone.
        if (
            runner.public_state().value
            == "ENTER_CORRIDOR"
        ):
            print()
            print("==========================================")
            print(" PRE_ENTRY SUCCESSFULLY LOCKED")
            print(" FSM REACHED ENTER_CORRIDOR")
            print()
            print(
                "No movement was sent to ArduPilot."
            )
            print(
                "ENTER_CORRIDOR distance is still "
                "intentionally unconfigured."
            )
            print("==========================================")

            # Give runner one more iteration so we can prove
            # the safe WAITING_FOR_ENTER_DISTANCE_CONFIG state.
            time.sleep(0.2)

            output = runner.step(
                scan=latest_scan,
                attitude=None,
                pose=None,
            )

            print()
            print(
                f"[FSM] status={output.status}"
            )
            print(
                f"[FSM] command="
                f"vx={output.command.vx_m_s:+.3f} "
                f"vy={output.command.vy_m_s:+.3f}"
            )

            break


except KeyboardInterrupt:
    print()
    print("[TEST] Stopped by user.")

finally:
    print()
    print(
        "[TEST] Read-only test finished. "
        "No MAVLink commands transmitted."
    )
