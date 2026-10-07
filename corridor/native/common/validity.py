"""Small, hardware-independent validity contracts.

Sector support thresholds are provisional. Unknown returns are never free space.
Adapters must explicitly opt in to a documented positive-infinity no-return
convention (Gazebo does; the raw serial adapter does not).
"""
import math
import time

import numpy as np


def fresh(timestamp, max_age, now=None):
    now = time.monotonic() if now is None else now
    return math.isfinite(timestamp) and 0 <= now - timestamp <= max_age


def pose_valid(pose, max_age):
    return bool(pose is not None and all(math.isfinite(v) for v in
                (pose.x_m, pose.y_m, pose.yaw_rad))
                and fresh(pose.timestamp, max_age))


def attitude_valid(attitude, max_age=0.50):
    return bool(attitude is not None and all(math.isfinite(v) for v in
                (attitude.roll_rad, attitude.pitch_rad, attitude.yaw_rad))
                and fresh(attitude.timestamp, max_age))


def scan_valid(scan, max_age=0.30):
    if scan is None or not fresh(scan.timestamp, max_age):
        return False
    r, a = np.asarray(scan.ranges_m), np.asarray(scan.angles_rad)
    return bool(r.ndim == a.ndim == 1 and r.size == a.size and r.size >= 20
                and np.all(np.isfinite(a))
                and math.isfinite(scan.range_min_m)
                and math.isfinite(scan.range_max_m)
                and 0 <= scan.range_min_m < scan.range_max_m)


def sector_clearance(scan, bearing_deg=0.0, half_cone_deg=18.0,
                     max_age=0.30, min_support=0.80, max_gap_deg=4.0):
    """Minimum supported range, or None for an unknown/incomplete sector.

The minimum deliberately stops on narrow close returns. Do not silently trim
them as percentile outliers. Filtering false returns requires sensor evidence.
"""
    if not scan_valid(scan, max_age):
        return None
    delta = np.arctan2(np.sin(scan.angles_rad - math.radians(bearing_deg)),
                       np.cos(scan.angles_rad - math.radians(bearing_deg)))
    half = math.radians(half_cone_deg)
    select = np.abs(delta) <= half
    angles = delta[select]
    ranges = np.asarray(scan.ranges_m)[select].copy()
    if angles.size < 3:
        return None
    if scan.no_return_is_clear:
        ranges[np.isposinf(ranges)] = scan.range_max_m
    valid = (np.isfinite(ranges) & (ranges >= scan.range_min_m)
             & (ranges <= scan.range_max_m))
    if np.count_nonzero(valid) < 3 or np.mean(valid) < min_support:
        return None
    supported = np.sort(angles[valid])
    gaps = np.diff(np.concatenate(([-half], supported, [half])))
    if np.max(gaps) > math.radians(max_gap_deg):
        return None
    return float(np.min(ranges[valid]))
