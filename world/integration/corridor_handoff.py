"""Forward range trigger independent of corridor wall geometry and camera lock."""
import numpy as np


def panel_front_distance(scan):
    angles = np.arctan2(np.sin(scan.angles_rad), np.cos(scan.angles_rad))
    ranges = scan.ranges_m
    valid = (np.isfinite(ranges) & np.isfinite(angles)
             & (ranges > scan.range_min_m) & (ranges < scan.range_max_m)
             & (np.abs(angles) <= np.deg2rad(10)))
    front = ranges[valid]
    if front.size < 3:
        return None
    # Reject isolated short returns; the panel spans several adjacent beams.
    return float(np.median(np.sort(front)[:3]))
