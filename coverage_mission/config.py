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
    # Rectangle coordinates are in a surveyed field frame. Zero is the
    # original Gazebo NED alignment; yaw is field-N relative to local NED-N.
    field_origin_n: float = 0.
    field_origin_e: float = 0.
    field_yaw: float = 0.
    # Optional four GPS corners, ordered field SW -> NW -> NE -> SE.
    # Registration against FC GPS_GLOBAL_ORIGIN happens after takeoff.
    geofence_latlon: tuple | None = None
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
    planning_wall_budget: float = .20  # Provisional per-replan wall budget; benchmark on Pi.
    max_tilt: float = math.radians(8)
    heading: float = 0.
    heartbeat_wall_age: float = 3.
    sensor_wall_age: float = 1.
    worker_wall_age: float = 1.
    no_progress: float = 40.  # source time, not mission optimization
    minimum_observation_progress_area: float = .25  # m² of new ground per liveness reset
    relocation_credit_speed: float = .20  # provisional lower bound for a checked connector
    red_confirm_frames: int = 5  # Distinct admitted observations; first hit still blocks.
    red_saturation_min: int = 60
    red_value_min: int = 25
    red_hue_low_max: int = 15
    red_hue_high_min: int = 165
    # Provisional full-field rule: every received, correctly shaped frame is
    # ground evidence; only detected red marks a red hazard. This deliberately
    # does not distinguish a wholly black image from dark ground. Missing or
    # stale frames still fail the independent transport/freshness checks.
    assume_nonred_ground_clear: bool = False
    # Optional, provisional green-ground evidence for dark textured grass.
    # Disabled by default; the full Gazebo field opts in. Revalidate on site.
    green_texture_support_m: float = 0.
    green_texture_mean_min: int = 15
    # Printed neutral targets can have black pixels below the red HSV floor.
    # Require nearby bright neutral pixels; a uniformly dark patch stays unknown.
    neutral_print_support_m: float = 0.
    # Optional tiny dark/print-hole inference, never applied near red or field
    # edges. It is not direct camera evidence and is kept separate in the map.
    small_unknown_hole_area: float = 0.
    downward_topic: str = '/coverage/down/image'
    clock_topic: str = '/world/coverage_test/clock'

    def __post_init__(self):
        for key, value in asdict(self).items():
            if isinstance(value, (int, float)) and not math.isfinite(value):
                raise ValueError(f"Non-finite {key}")
        if not (self.n_min < self.n_max and self.e_min < self.e_max):
            raise ValueError("Empty field")
        if self.geofence_latlon is not None:
            corners=self.geofence_latlon
            if (len(corners)!=4 or any(len(c)!=2 or not all(math.isfinite(v) for v in c)
                                       or not (-90<=c[0]<=90 and -180<=c[1]<=180)
                                       for c in corners)):
                raise ValueError('Geofence requires four finite lat/lon corners')
        if not 0 <= self.overlap < 1 or not 0 < self.camera.hfov < math.pi/2:
            raise ValueError("Invalid overlap/FOV")
        if min(self.resolution, self.speed, self.braking, self.acceleration,
               self.arrival, self.frame_age, self.pose_gap, self.no_progress,self.mapping_motion_grace,self.viewpoint_settle,
               self.relocation_credit_speed,
               self.projection_motion_window,self.max_projection_angular_rate,self.max_projection_tilt,
               self.decision_period,self.minimum_worker_wall_period,
               self.planning_wall_budget) <= 0:
            raise ValueError("Positive resolution, motion and timing limits required")
        if self.minimum_observation_progress_area <= 0:
            raise ValueError('Positive observed-area progress required')
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
        if not (2 <= self.red_confirm_frames <= 255
                and 0 <= self.red_saturation_min <= 255
                and 0 <= self.red_value_min <= 255
                and 0 <= self.red_hue_low_max < self.red_hue_high_min <= 179):
            raise ValueError('Invalid red evidence thresholds')
        if (self.green_texture_support_m < 0 or self.neutral_print_support_m < 0
                or self.small_unknown_hole_area < 0
                or not 0 <= self.green_texture_mean_min <= 255):
            raise ValueError('Invalid ground evidence thresholds')
        # A four-corner footprint only bounds a pinhole camera. Real distortion
        # needs a calibrated, edge-sampled footprint before it can certify ground.
        if any(abs(v) > 1e-12 for v in self.camera.distortion):
            raise ValueError('Nonzero lens distortion is not supported by the coverage map')
        if any(abs(v)>1e-8 for v in (self.camera.mount_roll,
                                      self.camera.mount_pitch,self.camera.mount_yaw)):
            raise ValueError('Rotated camera mount is not supported by the sweep footprint')
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

    def register_origin(self,latitude,longitude):
        """Derive a local-NED rectangular field from surveyed GPS corners.

        WGS-84 tangent approximation is much smaller than a centimetre over
        this 40 m mission when the FC origin is accurate.
        Nonrectangular or wrongly ordered boundaries fail closed rather than
        silently using a bounding box that extends outside the geofence.
        """
        if self.geofence_latlon is None:
            return self
        if (not all(math.isfinite(v) for v in (latitude,longitude))
                or not (-90<=latitude<=90 and -180<=longitude<=180)):
            raise ValueError('Valid FC local GPS origin required for geofence')
        import numpy as np
        lat0=math.radians(latitude)
        equatorial_radius=6378137.
        eccentricity_squared=0.00669437999014
        denominator=1-eccentricity_squared*math.sin(lat0)**2
        meridian_radius=equatorial_radius*(1-eccentricity_squared)/denominator**1.5
        prime_vertical_radius=equatorial_radius/math.sqrt(denominator)
        corners=np.array([[(lat-latitude)*math.pi/180*meridian_radius,
                           (lon-longitude)*math.pi/180*prime_vertical_radius*math.cos(lat0)]
                          for lat,lon in self.geofence_latlon])
        north=corners[1]-corners[0]; east=corners[2]-corners[1]
        length=float(np.linalg.norm(north)); width=float(np.linalg.norm(east))
        if min(length,width)<2*self.clearance or np.linalg.det(np.stack([north,east]))<=0:
            raise ValueError('Geofence corners are too small or not in field order')
        if (abs(float(north@east))/(length*width)>.02
                or np.linalg.norm(corners[3]-(corners[0]+east))>.20
                or np.linalg.norm(corners[2]-(corners[0]+north+east))>.20):
            raise ValueError('Coverage currently requires a surveyed rectangle')
        from dataclasses import replace
        return replace(self,field_origin_n=float(corners[0,0]),
                       field_origin_e=float(corners[0,1]),
                       field_yaw=math.atan2(north[1],north[0]),
                       n_min=0.,n_max=length,e_min=0.,e_max=width)
