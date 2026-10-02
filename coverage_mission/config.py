"""Explicit test-field and camera contracts; metres, radians, seconds, NED."""
from dataclasses import dataclass, field, asdict
import json
import math
from pathlib import Path


@dataclass(frozen=True)
class Camera:
    width: int = 640
    height: int = 480
    hfov: float = math.radians(28.1975597684)
    # Optical right -> body right; optical down -> body backward; optical Z -> down.
    mount_yaw: float = 0.0
    mount_roll: float = 0.0
    mount_pitch: float = 0.0
    down_offset: float = 0.0  # Neglect centimetres initially, as agreed.
    distortion: tuple = (0., 0., 0., 0., 0.)
    calibrated_fx: float | None = None
    calibrated_fy: float | None = None
    calibrated_cx: float | None = None
    calibrated_cy: float | None = None

    @property
    def fx(self):
        return self.calibrated_fx or self.width / (2 * math.tan(self.hfov / 2))

    @property
    def fy(self): return self.calibrated_fy or self.fx

    @property
    def cx(self): return self.width/2 if self.calibrated_cx is None else self.calibrated_cx

    @property
    def cy(self): return self.height/2 if self.calibrated_cy is None else self.calibrated_cy

    def footprint(self, altitude):
        # Conservative centred footprint when the principal point is off-centre.
        return 2*altitude*min(self.cx,self.width-self.cx)/self.fx, 2*altitude*min(self.cy,self.height-self.cy)/self.fy


@dataclass(frozen=True)
class Config:
    # 40 x 30 m isolated fixture. Home/entry is N=E=0, yaw north.
    n_min: float = -1.6
    n_max: float = 38.4
    e_min: float = -2.2
    e_max: float = 27.8
    altitude: float = 10.
    altitude_tolerance: float = .5
    # HOME and delivery ground share the same flat elevation.
    ground_above_home: float = 0.
    camera: Camera = field(default_factory=Camera)
    overlap: float = .30
    resolution: float = .1
    # Iris rotor centres (.13, .22) plus .10 m propeller radius: < .36 m.
    body_radius: float = .40
    # Commissioning observed .226 m peak projection error; .20 m was insufficient.
    # Provisional reserve, not a general bound on hardware/localization errors.
    uncertainty: float = .30
    coverage_reserve: float = .30
    speed: float = .75
    acceleration: float = .5
    braking: float = .5
    reaction_time: float = .25
    arrival: float = .24
    frame_age: float = .20  # source/simulation time
    mapping_motion_grace: float = 1.0  # fresh usable camera, frozen known map only
    viewpoint_settle: float = .5  # stationary source seconds before leaving a repair viewpoint
    pose_gap: float = .15
    projection_motion_window: float = .05
    # 0.1 rad/s over 50 ms is 0.005 rad (~5 cm at 10 m) of angular
    # timing sensitivity. Admit steady observations, not rapid attitude changes.
    max_projection_angular_rate: float = .10
    # Nominal 0.5 m/s^2 horizontal acceleration needs ~2.9 degrees of tilt.
    # Extra allowance for tracking, without accepting poorly conditioned views
    # at combined roll/pitch peaks (where angular rate alone can be near zero).
    max_projection_tilt: float = math.radians(5)
    decision_period: float = .05  # Source seconds: 20 Hz in simulation or real time.
    minimum_worker_wall_period: float = .02  # Bound producer CPU at accelerated RTF.
    max_tilt: float = math.radians(8)
    heading: float = 0.
    heartbeat_wall_age: float = 3.
    sensor_wall_age: float = 1.
    worker_wall_age: float = 1.
    no_progress: float = 40.  # source time, not mission optimization
    downward_topic: str = '/coverage/down/image'
    clock_topic: str = '/world/coverage_test/clock'

    def __post_init__(self):
        for key, value in asdict(self).items():
            if isinstance(value, (int, float)) and not math.isfinite(value):
                raise ValueError(f"Non-finite {key}")
        if not (self.n_min < self.n_max and self.e_min < self.e_max):
            raise ValueError("Empty field")
        if not 0 <= self.overlap < 1 or not 0 < self.camera.hfov < math.pi/2:
            raise ValueError("Invalid overlap/FOV")
        if min(self.resolution, self.speed, self.braking, self.acceleration,
               self.arrival, self.frame_age, self.pose_gap, self.no_progress,self.mapping_motion_grace,self.viewpoint_settle,
               self.projection_motion_window,self.max_projection_angular_rate,self.max_projection_tilt,
               self.decision_period,self.minimum_worker_wall_period) <= 0:
            raise ValueError("Positive resolution, motion and timing limits required")
        if self.projection_motion_window>=self.frame_age:
            raise ValueError('Projection motion window must fit within image freshness')
        if self.max_projection_tilt>=math.pi/2:
            raise ValueError('Projection tilt must remain below the horizon')
        if min(self.body_radius, self.uncertainty, self.coverage_reserve) < 0:
            raise ValueError("Negative margin")
        if self.altitude <= self.altitude_tolerance + self.ground_above_home:
            raise ValueError("Invalid ground-relative altitude")
        if self.camera.width < 8 or self.camera.height < 8:
            raise ValueError("Invalid camera dimensions")
        for v in (self.camera.calibrated_fx,self.camera.calibrated_fy):
            if v is not None and (not math.isfinite(v) or v<=0): raise ValueError('Invalid focal calibration')
        if not (0<self.camera.cx<self.camera.width and 0<self.camera.cy<self.camera.height):
            raise ValueError('Invalid principal point')
        if not all(math.isfinite(x) for x in (*self.camera.distortion,
                    self.camera.mount_yaw, self.camera.mount_roll,
                    self.camera.mount_pitch, self.camera.down_offset)):
            raise ValueError("Invalid camera calibration")

    @property
    def clearance(self):
        return self.body_radius + self.uncertainty

    @classmethod
    def load(cls, path):
        data = json.loads(Path(path).read_text())
        data['camera'] = Camera(**data.get('camera', {}))
        return cls(**data)

    def as_dict(self):
        return asdict(self)
