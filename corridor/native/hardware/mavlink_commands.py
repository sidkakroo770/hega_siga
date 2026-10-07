"""Pure command planning: FLU -> MAVLink body forward/right/down.

No connections or transmissions occur in this module.
"""
import math
from dataclasses import dataclass

from native.common.types import VehicleAction


@dataclass(frozen=True)
class VelocityPlan:
    vx_m_s: float
    vy_m_s: float
    vz_m_s: float
    yaw_rate_rad_s: float
    coordinate_frame: int = 8  # MAV_FRAME_BODY_NED
    type_mask: int = 1479  # velocity + yaw rate; position/acceleration/yaw ignored


@dataclass(frozen=True)
class ActionPlan:
    command: int = 21  # MAV_CMD_NAV_LAND
    confirmation: int = 0
    params: tuple = (0., 0., 0., 0., 0., 0., 0.)


def body_velocity_to_mavlink(command):
    values = (command.vx_m_s, command.vy_m_s, command.vz_m_s,
              command.yaw_rate_rad_s)
    if not all(math.isfinite(v) for v in values):
        raise ValueError("Nonfinite velocity command")
    return VelocityPlan(values[0], -values[1], -values[2], -values[3])


def vehicle_action_to_mavlink(action):
    if action != VehicleAction.LAND:
        raise ValueError(f"Unsupported vehicle action: {action}")
    return ActionPlan()
