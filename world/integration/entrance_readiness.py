"""Read-only corridor entrance evidence before granting native control."""
from dataclasses import dataclass
import math

from native.common.validity import attitude_valid, pose_valid, scan_valid
from native.controllers.pre_entry import PreEntryController


@dataclass(frozen=True)
class StagingEnvelope:
    # Fixed Gazebo world local N/E, based on the successful PRE_ENTRY capture.
    # Replace with measured site geometry before physical corridor flight.
    n_min: float = -31.5
    n_max: float = -29.0
    e_min: float = -6.0
    e_max: float = -2.0
    min_home_alt_m: float = 1.5
    max_home_alt_m: float = 5.0

    def contains(self, n, e, home_alt):
        values = (n, e, home_alt)
        return (all(v is not None and math.isfinite(v) for v in values)
                and self.n_min <= n <= self.n_max
                and self.e_min <= e <= self.e_max
                and self.min_home_alt_m <= home_alt <= self.max_home_alt_m)


class EntranceReadiness:
    def __init__(self, required_scans=3):
        if required_scans < 1:
            raise ValueError("Require at least one distinct scan")
        self.estimator = PreEntryController()
        self.required_scans = required_scans
        self.count = 0
        self.last_sequence = None
        self.last_stamp = 0.0
        self.last_geometry = None

    def observe(self, sequence, scan, attitude, pose):
        if sequence == self.last_sequence:
            return False
        self.last_sequence = sequence
        if not (scan_valid(scan) and attitude_valid(attitude)
                and pose_valid(pose, 0.5)
                and self.estimator.attitude_is_safe(attitude)):
            self.count = 0
            return False
        if scan.timestamp <= self.last_stamp:
            self.count = 0
            return False
        self.last_stamp = scan.timestamp
        geometry = self.estimator.extract_corridor_geometry(scan)
        self.last_geometry = geometry
        c = self.estimator.config
        ready = (geometry.strict_valid
                 and geometry.confidence >= c.control_confidence_min
                 and math.isfinite(geometry.front_clearance)
                 and geometry.front_clearance >= c.front_stop_m)
        self.count = self.count + 1 if ready else 0
        return self.count >= self.required_scans

    def reset(self):
        self.count = 0
        self.last_sequence = None
        self.last_stamp = 0.0
