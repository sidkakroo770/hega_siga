"""Bound the entire camera-guided phase, including reacquisition attempts.

Budgets are provisional configuration, not certified aircraft clearances.
This guard uses local receipt age; acquisition-clock alignment is separate.
"""
from dataclasses import dataclass
import math

from native.common.validity import fresh


@dataclass(frozen=True)
class BannerGuardConfig:
    timeout_s: float = 120.0
    max_displacement_m: float = 10.0
    frame_max_age_s: float = 0.5
    health_failure_s: float = 2.0
    pose_max_age_s: float = 0.5

    def __post_init__(self):
        if not all(math.isfinite(v) and v > 0 for v in vars(self).values()):
            raise ValueError("Banner guard budgets must be finite and positive")


class BannerGuard:
    def __init__(self, started, config=None):
        self.config = config or BannerGuardConfig()
        self.started = started
        self.origin = None
        self.unhealthy_since = None
        self.abort_reason = None

    def check(self, now, frame_stamp, position, pose_stamp, attitude_ok):
        """Return (may_move, terminal_reason); failures are latched."""
        if self.abort_reason:
            return False, self.abort_reason
        c = self.config
        if now - self.started >= c.timeout_s:
            self.abort_reason = "camera phase deadline"
            return False, self.abort_reason
        position_ok = (position is not None and len(position) == 3
                       and all(math.isfinite(v) for v in position)
                       and fresh(pose_stamp, c.pose_max_age_s, now))
        if position_ok:
            if self.origin is None:
                self.origin = position
            if math.dist(position, self.origin) >= c.max_displacement_m:
                self.abort_reason = "camera phase travel envelope exceeded"
                return False, self.abort_reason
        healthy = (position_ok and attitude_ok
                   and fresh(frame_stamp, c.frame_max_age_s, now))
        if not healthy:
            if self.unhealthy_since is None:
                self.unhealthy_since = now
            if now - self.unhealthy_since >= c.health_failure_s:
                self.abort_reason = "camera phase sensor health timeout"
            return False, self.abort_reason
        self.unhealthy_since = None
        return True, None
