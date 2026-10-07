"""Bounded, feedback-driven startup for the full non-ROS mission.

All times are host monotonic receipt times. Physical clearances and initial
takeoff altitude are deployment settings, not properties inferred from images.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class StartupConfig:
    takeoff_altitude_m: float = 3.0  # Gazebo world profile, provisional for hardware
    preflight_timeout_s: float = 25.0
    mode_timeout_s: float = 8.0
    arm_timeout_s: float = 10.0
    takeoff_timeout_s: float = 45.0
    settle_s: float = 2.0
    altitude_tolerance_m: float = 0.20
    speed_tolerance_m_s: float = 0.15
    feedback_max_age_s: float = 0.75

    def __post_init__(self):
        if not all(math.isfinite(v) and v > 0 for v in vars(self).values()):
            raise ValueError("Startup settings must be finite and positive")


@dataclass(frozen=True)
class StartupFeedback:
    heartbeat_time: float = 0.0
    mode: str = "UNKNOWN"
    armed: bool = False
    landed_state: int | None = None
    landed_time: float = 0.0
    ekf_flags: int = 0
    ekf_time: float = 0.0
    position_time: float = 0.0
    attitude_time: float = 0.0
    relative_alt_time: float = 0.0
    relative_alt_m: float | None = None
    velocity_m_s: tuple[float, float, float] | None = None
    position_m: tuple[float, float, float] | None = None
    camera_time: float = 0.0
    lidar_time: float = 0.0
    rejected_command: int | None = None
    prearm_ok: bool = False
    prearm_time: float = 0.0


def recent(stamp, now, limit):
    return math.isfinite(stamp) and 0 < stamp <= now and now - stamp <= limit


def sensor_ready(f, now, age):
    return (recent(f.heartbeat_time, now, 2.5)
            and recent(f.position_time, now, age)
            and recent(f.attitude_time, now, age)
            and recent(f.relative_alt_time, now, age)
            and recent(f.ekf_time, now, 1.5)
            and recent(f.camera_time, now, age)
            and recent(f.lidar_time, now, age)
            and bool(f.ekf_flags & (8 | 16))
            and f.relative_alt_m is not None
            and math.isfinite(f.relative_alt_m)
            and f.position_m is not None
            and all(math.isfinite(v) for v in f.position_m)
            and f.velocity_m_s is not None
            and all(math.isfinite(v) for v in f.velocity_m_s))


class StartupController:
    """Actions are single-shot. State changes require separate observed feedback."""

    def __init__(self, started, config=None):
        self.config = config or StartupConfig()
        self.state = "PREFLIGHT"
        self.state_since = started
        self.settled_since = None
        self.reason = ""
        self.armed_by_mission = False

    def _enter(self, state, now):
        self.state, self.state_since = state, now
        self.settled_since = None

    def _abort(self, reason, now):
        self.reason = reason
        self._enter("ABORT", now)
        return None

    def step(self, f, now):
        """Return a one-shot MAVLink action: MODE, ARM, TAKEOFF, or None."""
        c = self.config
        if self.state in ("READY", "ABORT"):
            return None
        if self.state == "PREFLIGHT":
            if f.armed:
                return self._abort("already armed; refusing restart", now)
            if (recent(f.landed_time, now, 2.5) and f.landed_state != 1):
                return self._abort("vehicle not confirmed on ground", now)
            if (sensor_ready(f, now, c.feedback_max_age_s)
                    and f.prearm_ok and recent(f.prearm_time, now, 2.5)
                    and recent(f.landed_time, now, 2.5)
                    and f.landed_state == 1
                    and abs(f.relative_alt_m) <= c.altitude_tolerance_m
                    and f.mode != "GUIDED"):
                self._enter("WAIT_MODE", now)
                return "MODE"
            if (sensor_ready(f, now, c.feedback_max_age_s)
                    and f.prearm_ok and recent(f.prearm_time, now, 2.5)
                    and recent(f.landed_time, now, 2.5)
                    and f.landed_state == 1
                    and abs(f.relative_alt_m) <= c.altitude_tolerance_m
                    and f.mode == "GUIDED"):
                self._enter("WAIT_ARM", now)
                return "ARM"
            if now - self.state_since >= c.preflight_timeout_s:
                return self._abort("preflight feedback timeout", now)
            return None
        if self.state == "WAIT_MODE":
            if f.rejected_command == 176:
                return self._abort("GUIDED mode rejected", now)
            if f.mode == "GUIDED" and recent(f.heartbeat_time, now, 2.5):
                self._enter("WAIT_ARM", now)
                return "ARM"
            if now - self.state_since >= c.mode_timeout_s:
                return self._abort("GUIDED mode not confirmed", now)
            return None
        if self.state == "WAIT_ARM":
            if f.rejected_command == 400:
                return self._abort("arming rejected", now)
            if f.mode != "GUIDED" and recent(f.heartbeat_time, now, 2.5):
                return self._abort("mode changed during arming", now)
            if f.armed and recent(f.heartbeat_time, now, 2.5):
                self.armed_by_mission = True
                self._enter("WAIT_TAKEOFF", now)
                return "TAKEOFF"
            if now - self.state_since >= c.arm_timeout_s:
                return self._abort("arming not confirmed", now)
            return None
        if self.state == "WAIT_TAKEOFF":
            if f.rejected_command == 22:
                return self._abort("takeoff rejected", now)
            if f.mode != "GUIDED" or not f.armed:
                return self._abort("authority lost during takeoff", now)
            if not recent(f.heartbeat_time, now, 2.5):
                return self._abort("heartbeat lost during takeoff", now)
            if (recent(f.relative_alt_time, now, c.feedback_max_age_s)
                    and f.relative_alt_m is not None
                    and math.isfinite(f.relative_alt_m)
                    and f.relative_alt_m >= c.takeoff_altitude_m - c.altitude_tolerance_m):
                self._enter("SETTLING", now)
            elif now - self.state_since >= c.takeoff_timeout_s:
                return self._abort("takeoff altitude timeout", now)
            return None
        if self.state == "SETTLING":
            if not f.armed or f.mode != "GUIDED" or not recent(f.heartbeat_time, now, 2.5):
                return self._abort("authority lost while settling", now)
            healthy = (sensor_ready(f, now, c.feedback_max_age_s)
                       and abs(f.relative_alt_m - c.takeoff_altitude_m) <= c.altitude_tolerance_m
                       and math.sqrt(sum(v*v for v in f.velocity_m_s)) <= c.speed_tolerance_m_s)
            if healthy:
                if self.settled_since is None:
                    self.settled_since = now
                elif now - self.settled_since >= c.settle_s:
                    self._enter("READY", now)
            else:
                self.settled_since = None
            if now - self.state_since >= c.takeoff_timeout_s:
                return self._abort("takeoff settling timeout", now)
            return None
        raise RuntimeError(f"Unknown startup state {self.state}")
