"""Provisional local-pose envelope for straight corridor commitments.

This is an integrity guard, not a certified wall clearance.  LiDAR geometry
continues to decide whether space ahead is actually traversable.
"""

import math


def straight_path_error(start, current):
    """Return (forward, signed cross-track, wrapped yaw error) in NED metres/rad."""
    return straight_path_error_xy(
        start.x_m, start.y_m, start.yaw_rad, current)


def straight_path_error_xy(start_x, start_y, start_yaw, current):
    dx = float(current.x_m) - float(start_x)
    dy = float(current.y_m) - float(start_y)
    heading = float(start_yaw)
    yaw_error = math.atan2(
        math.sin(float(current.yaw_rad) - heading),
        math.cos(float(current.yaw_rad) - heading),
    )
    return (
        math.cos(heading) * dx + math.sin(heading) * dy,
        -math.sin(heading) * dx + math.cos(heading) * dy,
        yaw_error,
    )


def path_within_envelope(start, current, max_cross_track_m, max_yaw_error_deg):
    return path_within_envelope_xy(
        start.x_m, start.y_m, start.yaw_rad, current,
        max_cross_track_m, max_yaw_error_deg)


def path_within_envelope_xy(start_x, start_y, start_yaw, current,
                            max_cross_track_m, max_yaw_error_deg):
    forward, cross_track, yaw_error = straight_path_error_xy(
        start_x, start_y, start_yaw, current)
    if not all(math.isfinite(v) for v in (forward, cross_track, yaw_error)):
        return False, forward, cross_track, yaw_error
    return (abs(cross_track) <= max_cross_track_m
            and abs(yaw_error) <= math.radians(max_yaw_error_deg)), \
        forward, cross_track, yaw_error
