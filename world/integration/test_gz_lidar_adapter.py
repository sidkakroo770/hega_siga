#!/usr/bin/env python3

import os
import sys
import time
import math
import importlib

import numpy as np


# ------------------------------------------------------------
# Make Ubuntu Gazebo Python packages visible even if the user
# happens to be inside another Python environment.
# ------------------------------------------------------------

for p in (
    "/usr/lib/python3/dist-packages",
    "/usr/local/lib/python3/dist-packages",
    "/usr/lib/python3/site-packages",
):
    if os.path.exists(p) and p not in sys.path:
        sys.path.insert(0, p)


# ------------------------------------------------------------
# Load Gazebo Transport dynamically.
# Same idea used by the coworker's camera mission.
# ------------------------------------------------------------

Node = None

for name in (
    "gz.transport15",
    "gz.transport14",
    "gz.transport13",
    "gz.transport12",
    "gz.transport",
):
    try:
        mod = importlib.import_module(name)
        if hasattr(mod, "Node"):
            Node = mod.Node
            print(f"[OK] Gazebo transport: {name}")
            break
    except Exception:
        pass


# ------------------------------------------------------------
# Load LaserScan protobuf dynamically.
# ------------------------------------------------------------

LaserScan = None
loaded_scan_module = None

candidates = []

for version in (15, 14, 13, 12, 11, 10):
    candidates += [
        f"gz.msgs{version}.laserscan_pb2",
        f"gz.msgs{version}.laser_scan_pb2",
    ]

candidates += [
    "gz.msgs.laserscan_pb2",
    "gz.msgs.laser_scan_pb2",
]

for name in candidates:
    try:
        mod = importlib.import_module(name)

        if hasattr(mod, "LaserScan"):
            LaserScan = mod.LaserScan
            loaded_scan_module = name
            print(f"[OK] Gazebo LaserScan: {name}")
            break

    except Exception:
        pass


if Node is None:
    raise SystemExit(
        "[ERROR] Gazebo Transport Python Node not found."
    )

if LaserScan is None:
    raise SystemExit(
        "[ERROR] Gazebo LaserScan protobuf binding not found."
    )


# ------------------------------------------------------------
# Import the real corridor datatype.
# ------------------------------------------------------------

from native.common.types import NativeScan


TOPIC = "/iris/lidar/scan"

latest_native_scan = None
scan_count = 0


def callback(msg):
    global latest_native_scan
    global scan_count

    scan_count += 1

    ranges = np.asarray(
        msg.ranges,
        dtype=np.float64,
    )

    if ranges.size == 0:
        return

    angle_min = float(msg.angle_min)
    angle_step = float(msg.angle_step)

    angles = (
        angle_min
        + np.arange(ranges.size, dtype=np.float64)
        * angle_step
    )

    try:
        raw_intensities = np.asarray(
            msg.intensities,
            dtype=np.float64,
        )
    except Exception:
        raw_intensities = np.empty(0)

    if raw_intensities.size != ranges.size:
        intensities = np.zeros(
            ranges.size,
            dtype=np.float64,
        )
    else:
        intensities = raw_intensities

    latest_native_scan = NativeScan(
        angles_rad=angles,
        ranges_m=ranges,
        intensities=intensities,
        timestamp=time.monotonic(),
        range_min_m=float(msg.range_min),
        range_max_m=float(msg.range_max),
    )


def nearest_range(scan, wanted_angle):
    delta = np.arctan2(
        np.sin(scan.angles_rad - wanted_angle),
        np.cos(scan.angles_rad - wanted_angle),
    )

    idx = int(np.argmin(np.abs(delta)))

    return float(scan.ranges_m[idx])


node = Node()

ok = node.subscribe(
    LaserScan,
    TOPIC,
    callback,
)

print(f"[INFO] subscribe({TOPIC}) -> {ok}")
print("[INFO] Waiting for scans...")


last_print = 0.0

while True:
    time.sleep(0.05)

    scan = latest_native_scan

    if scan is None:
        continue

    now = time.monotonic()

    if now - last_print < 1.0:
        continue

    last_print = now

    front = nearest_range(scan, 0.0)
    left = nearest_range(scan, math.pi / 2)
    right = nearest_range(scan, -math.pi / 2)
    rear = nearest_range(scan, math.pi)

    print()
    print(
        f"[SCAN #{scan_count}] "
        f"points={scan.size} "
        f"age={scan.age_s:.3f}s"
    )

    print(
        f"range limits: "
        f"{scan.range_min_m:.2f} .. "
        f"{scan.range_max_m:.2f} m"
    )

    print(
        f"FRONT  0°   : {front:.2f} m"
    )

    print(
        f"LEFT  +90°   : {left:.2f} m"
    )

    print(
        f"RIGHT -90°   : {right:.2f} m"
    )

    print(
        f"REAR ±180°   : {rear:.2f} m"
    )
