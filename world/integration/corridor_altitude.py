"""Altitude control in the EKF-origin frame; positive output is NED down."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class AltitudeResult:
    vz_down: float = 0.0
    ready: bool = False
    error: str = ""


class AltitudeController:
    def __init__(self, target=1.3, max_speed=0.30, tolerance=0.12,
                 dwell=1.0, timeout=20.0, telemetry_max_age=0.50, gain=0.6):
        values = (target, max_speed, tolerance, dwell, timeout, telemetry_max_age, gain)
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError("Altitude settings must be finite and positive")
        if tolerance >= target or timeout <= dwell:
            raise ValueError("Require tolerance < target and timeout > dwell")
        self.target, self.max_speed, self.tolerance = target, max_speed, tolerance
        self.dwell, self.timeout = dwell, timeout
        self.telemetry_max_age = telemetry_max_age
        self.gain = gain
        self.start(0.0)

    def start(self, now):
        self.started = now
        self.stable_since = None
        self.missing_since = None
        self.ready = False
        self.last_sample_time = None
        self.mission_started = None
        self.last_mission_time = None

    def update(self, now, z, vz, timestamp, mission_time=None):
        fresh = (z is not None and vz is not None
                 and math.isfinite(z) and math.isfinite(vz)
                 and 0 <= now - timestamp <= self.telemetry_max_age)
        if not fresh:
            self.stable_since = None
            self.ready = False
            if self.missing_since is None:
                self.missing_since = now
            return AltitudeResult(error="altitude telemetry unavailable" if
                                  now - self.missing_since >= 2.0 else "")
        self.missing_since = None
        control_now = now
        if mission_time is not None:
            if not math.isfinite(mission_time) or (
                self.last_mission_time is not None and mission_time < self.last_mission_time
            ):
                return AltitudeResult(error="autopilot clock invalid or reset")
            if self.mission_started is None:
                self.mission_started = mission_time
            self.last_mission_time = mission_time
            control_now = self.started + mission_time - self.mission_started
            # A paused or extremely slow simulation must not wait forever.
            if now - self.started >= 180.0:
                return AltitudeResult(error="descent wall-time watchdog expired")
        new_sample = timestamp != self.last_sample_time
        self.last_sample_time = timestamp
        error = -z - self.target
        # Account for travel since the last received sample so slower telemetry
        # does not keep a full-speed command active close to the target.
        # Wall seconds are not simulation seconds. With an autopilot clock,
        # use measured error rather than extrapolating by wall-time age.
        predicted_error = error if mission_time is not None else error - vz * (now - timestamp)
        if error * predicted_error <= 0:
            predicted_error = 0.0
        elif abs(predicted_error) > abs(error):
            predicted_error = error
        speed = max(-self.max_speed, min(self.max_speed, self.gain * predicted_error))
        settled = abs(error) <= self.tolerance and abs(vz) <= 0.10
        # Hysteresis avoids interrupting traversal on minor measurement noise.
        if self.ready and abs(error) <= 2 * self.tolerance:
            return AltitudeResult(speed, True)
        if self.ready:
            self.start(now)
            if mission_time is not None:
                self.mission_started = mission_time
            control_now = now
            self.stable_since = None
            self.ready = False
        if settled:
            if self.stable_since is None:
                self.stable_since = control_now
            if new_sample and control_now - self.stable_since >= self.dwell:
                self.ready = True
                return AltitudeResult(speed, True)
        else:
            self.stable_since = None
        if control_now - self.started >= self.timeout:
            return AltitudeResult(error="altitude acquisition timed out")
        return AltitudeResult(speed)
