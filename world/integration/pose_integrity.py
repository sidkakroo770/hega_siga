"""Conservative LOCAL_POSITION_NED continuity check for corridor progress.

Limits are provisional software guards, not measured EKF accuracy. A violation
must stop mission authority; it must never be credited as travelled distance.
"""

from dataclasses import dataclass
import math


@dataclass
class PoseIntegrity:
    max_speed_m_s: float = 5.0
    jump_slack_m: float = 2.0
    max_clock_backstep_s: float = 0.1
    last_boot_s: float | None = None
    last_xyz_m: tuple[float, float, float] | None = None
    failure: str | None = None

    def observe(self, boot_s, xyz_m, velocity_m_s):
        values = (boot_s, *xyz_m, *velocity_m_s)
        if not all(math.isfinite(float(value)) for value in values):
            self.failure = "nonfinite local-position feedback"
            return False
        if boot_s < 0:
            self.failure = "negative FC boot time"
            return False
        if self.last_boot_s is not None:
            elapsed = boot_s - self.last_boot_s
            if elapsed < -self.max_clock_backstep_s:
                self.failure = "FC local-position clock reset"
                return False
            if elapsed <= 0:
                return False  # Duplicate or slightly reordered packet, not new progress.
            distance = math.dist(self.last_xyz_m, xyz_m)
            allowed = self.jump_slack_m + self.max_speed_m_s * elapsed
            if distance > allowed:
                self.failure = (f"local-position jump {distance:.2f} m "
                                f"exceeds {allowed:.2f} m provisional bound")
                return False
        self.last_boot_s = boot_s
        self.last_xyz_m = tuple(float(v) for v in xyz_m)
        return True
