"""Bounded return proposals. No vehicle, GUI, camera or simulator ownership."""
from dataclasses import dataclass, asdict
import json
import math
import time
import cv2
import numpy as np

from .engine import Decision
from .field_frame import FieldFrame
from .planning import disk, route


@dataclass(frozen=True)
class ReturnConfig:
    # Surveyed local N/E fixture, not field-aligned coordinates. Provisional.
    entrance_n: float = -20.
    entrance_e: float = 4.1
    heading: float = math.pi
    stand_off: float = 6.
    survey_stand_off: float = 1.2
    acquisition_altitude: float = 5.
    delivery_dwell: float = 5.
    speed: float = .5
    vertical_speed: float = .5
    yaw_rate: float = .08  # Below the provisional projection angular-rate gate.
    settle_seconds: float = 1.
    altitude_tolerance: float = .15
    settled_speed: float = .10
    arrival: float = .20
    route_seconds: float = 240.
    vertical_seconds: float = 90.
    approach_seconds: float = 180.
    no_progress_seconds: float = 60.
    entry_half_width: float = .9
    entry_outside_limit: float = .8
    staging_min_altitude: float = 1.5
    staging_max_altitude: float = 5.3
    far_mouth_distance: float = 10.1
    landing_clearance: float = 2.
    landing_seconds: float = 90.
    orange_hsv_low: tuple = (5, 100, 80)
    orange_hsv_high: tuple = (25, 255, 255)

    def __post_init__(self):
        for key, value in asdict(self).items():
            if isinstance(value, (int, float)):
                if not math.isfinite(value): raise ValueError(f'Nonfinite return {key}')
                if key not in ('entrance_n', 'entrance_e', 'heading') and value <= 0:
                    raise ValueError(f'Return {key} must be positive')
        if self.staging_min_altitude >= self.staging_max_altitude:
            raise ValueError('Invalid return staging heights')
        if self.acquisition_altitude > self.staging_max_altitude:
            raise ValueError('Acquisition height outside staging envelope')
        if not (len(self.orange_hsv_low) == len(self.orange_hsv_high) == 3 and
                all(0 <= a <= b <= limit for a, b, limit in
                    zip(self.orange_hsv_low, self.orange_hsv_high, (179,255,255)))):
            raise ValueError('Invalid orange HSV bounds')

    @classmethod
    def load(cls, path):
        return cls(**json.loads(path.read_text()))

    @property
    def axis(self): return np.array([math.cos(self.heading), math.sin(self.heading)])

    @property
    def entrance(self): return np.array([self.entrance_n, self.entrance_e])

    @property
    def stand_off_point(self): return self.entrance - self.stand_off*self.axis

    def approach_contains(self, local_ne, alt):
        delta=np.asarray(local_ne)-self.entrance
        along=float(delta @ self.axis)
        across=float(delta @ np.array([-self.axis[1], self.axis[0]]))
        return (-self.stand_off-.5 <= along <= self.entry_outside_limit and
                abs(across) <= self.entry_half_width and
                self.staging_min_altitude <= alt <= self.staging_max_altitude)


class EntrancePermit:
    """Narrow surveyed mouth admission, never a global geofence bypass.

    Field-side ground still needs observed body-clearance support and no red.
    Only the configured exterior strip is admitted without a field map. That
    strip requires a surveyed clear operating envelope and front LiDAR checks.
    """
    def __init__(self, cfg, profile):
        self.cfg=cfg; self.profile=profile; self.frame=FieldFrame(cfg)
        self.last_map_t=None; self.known=None

    def allows(self, pose):
        error=math.atan2(math.sin(pose.yaw+self.cfg.field_yaw-self.profile.heading),
                         math.cos(pose.yaw+self.cfg.field_yaw-self.profile.heading))
        return (abs(error)<math.radians(15) and
                self.profile.approach_contains(self.frame.local_point(pose.xy), pose.alt))

    def line_clear(self, ground, a, b, altitude):
        if self.last_map_t != ground.last_t:
            self.known=cv2.erode((ground.observed|ground.contextual_clear).astype(np.uint8),
                disk(self.cfg.clearance,self.cfg.resolution),
                borderType=cv2.BORDER_CONSTANT,borderValue=1).astype(bool)
            self.last_map_t=ground.last_t
        count=max(1,math.ceil(np.linalg.norm(np.asarray(b)-a)/(self.cfg.resolution*.25)))
        for point in np.linspace(a,b,count+1):
            if not self.profile.approach_contains(self.frame.local_point(point), altitude):
                return False
            cell=ground.cell(point)
            if ground.contains(cell):
                if not self.known[cell] or ground.inflated[cell]: return False
            else:
                # Exterior admission is only through the surveyed mouth edge,
                # never through an unrelated side of the field.
                mouth=self.frame.point(self.profile.entrance)
                if not (self.cfg.n_min-.2 <= mouth[0] <= self.cfg.n_max+.2 and
                        self.cfg.e_min-.2 <= mouth[1] <= self.cfg.e_max+.2): return False
        return True


class ReturnNavigator:
    def __init__(self, cfg, profile):
        self.cfg=cfg; self.profile=profile; self.frame=FieldFrame(cfg)
        self.stand_off_goal=self.frame.point(profile.stand_off_point)
        self.goal=self.frame.point(profile.entrance-profile.survey_stand_off*profile.axis)
        if not (cfg.n_min+cfg.clearance+cfg.resolution*.71 <= self.goal[0] <= cfg.n_max-cfg.clearance-cfg.resolution*.71
                and cfg.e_min+cfg.clearance+cfg.resolution*.71 <= self.goal[1] <= cfg.e_max-cfg.clearance-cfg.resolution*.71):
            raise ValueError('Return survey viewpoint outside field inset')
        self.yaw=profile.heading-cfg.field_yaw
        self.state='RETURN_ASCEND'; self.started=None; self.phase_started=None
        self.anchor=None; self.settled=None; self.path=[]; self.last_route=-math.inf
        self.progress_t=None; self.best_distance=math.inf; self.observed=0
        self.permit=EntrancePermit(cfg,profile)
        self.entrance_active=False
        self.command_yaw=None; self.yaw_t=None

    def _change(self, state, pose):
        self.state=state; self.phase_started=pose.t; self.settled=None

    def interrupt(self): self.settled=None

    def step(self, pose, ground, front=None):
        decision,permit=self._step(pose,ground,front)
        if self.command_yaw is None: self.command_yaw=pose.yaw; self.yaw_t=pose.t
        dt=max(0.,min(.2,pose.t-self.yaw_t)); self.yaw_t=pose.t
        error=math.atan2(math.sin(self.yaw-self.command_yaw),math.cos(self.yaw-self.command_yaw))
        rate=min(self.profile.yaw_rate,self.cfg.max_projection_angular_rate*.8)
        self.command_yaw+=float(np.clip(error,-rate*dt,rate*dt))
        decision.yaw=self.command_yaw
        return decision,permit

    def _step(self, pose, ground, front=None):
        p=self.profile; cfg=self.cfg
        if self.started is None:
            self.started=self.phase_started=self.progress_t=pose.t
            self.anchor=pose.xy.copy(); self.observed=int(ground.observed.sum())
        d=Decision(self.state,'Returning to orange corridor',yaw=self.yaw)
        limit=p.route_seconds if self.state in ('RETURN_ROUTE','RETURN_SURVEY_ENTRY') else (
            p.approach_seconds if self.state=='RETURN_FRONT' else p.vertical_seconds)
        if pose.t-self.phase_started>limit:
            d.state='ABORTED'; d.reason=f'{self.state} deadline exceeded'; return d, None
        if self.state in ('RETURN_ASCEND','RETURN_ACQUISITION_HEIGHT'):
            altitude=cfg.altitude if self.state=='RETURN_ASCEND' else p.acquisition_altitude
            delta=self.anchor-pose.xy
            v=delta*min(.8,p.speed/max(np.linalg.norm(delta),1e-9))
            d.vn,d.ve=map(float,v)
            d.vd=float(np.clip((pose.alt-altitude)*.8,-p.vertical_speed,p.vertical_speed))
            settled=(np.linalg.norm(delta)<=p.arrival and abs(pose.alt-altitude)<=p.altitude_tolerance
                     and max(math.hypot(pose.vn,pose.ve),abs(pose.vd))<=p.settled_speed
                     and abs(math.atan2(math.sin(pose.yaw-self.yaw),math.cos(pose.yaw-self.yaw)))<math.radians(10))
            if settled:
                if self.settled is None: self.settled=pose.t
                if pose.t-self.settled>=p.settle_seconds:
                    self._change('RETURN_SURVEY_ENTRY' if self.state=='RETURN_ASCEND' else 'RETURN_FRONT',pose)
                    d.vn=d.ve=d.vd=0.
            else: self.settled=None
            return d,None
        if self.state=='RETURN_FRONT':
            # Surveyed admission belongs to this approach phase, not to the
            # lifetime of an individual camera/LiDAR proposal. A stale proposal
            # may stop motion without changing the aircraft's valid location.
            retained=self.permit if self.entrance_active and self.permit.allows(pose) else None
            if not front or not 0<=pose.t-front.get('source_t',-1)<=cfg.frame_age:
                if retained is not None: d.state='HOLD'
                d.reason='Waiting for fresh orange approach proposal'; return d,retained
            d=Decision(**{k:v for k,v in front.items() if k in Decision.__dataclass_fields__})
            d.yaw=self.yaw
            if front.get('entrance_permit'):
                self.entrance_active=True
            permit=self.permit if self.entrance_active and self.permit.allows(pose) else None
            if self.entrance_active and not front.get('entrance_permit') and d.state!='ABORTED':
                d.vn=d.ve=d.vd=0.
                d.state='HOLD'
            return d,permit
        if abs(pose.alt-cfg.altitude)>cfg.altitude_tolerance:
            d.reason='Reacquiring transit altitude'
            d.vd=float(np.clip((pose.alt-cfg.altitude)*.8,-p.vertical_speed,p.vertical_speed))
            return d,None
        distance=float(np.linalg.norm(pose.xy-self.goal)); seen=int(ground.observed.sum())
        if distance<self.best_distance-.20 or seen>self.observed+int(.5/cfg.resolution**2):
            self.progress_t=pose.t; self.best_distance=min(distance,self.best_distance); self.observed=seen
        if pose.t-self.progress_t>p.no_progress_seconds:
            d.state='ABORTED'; d.reason='Return route made no measured progress'; return d,None
        if distance<=p.arrival and math.hypot(pose.vn,pose.ve)<=p.settled_speed:
            if self.state=='RETURN_SURVEY_ENTRY':
                self.goal=self.stand_off_goal.copy(); self.path=[]; self.last_route=-math.inf
                self.best_distance=math.inf; self.progress_t=pose.t
                self._change('RETURN_ROUTE',pose)
            else:
                self.anchor=pose.xy.copy(); self._change('RETURN_ACQUISITION_HEIGHT',pose)
            return d,None
        while self.path and np.linalg.norm(self.path[0]-pose.xy)<p.arrival:
            self.path.pop(0)
        valid=bool(self.path and ground.line_clear(pose.xy,self.path[0]))
        if not valid and pose.t-self.last_route>=.5:
            begin=time.monotonic(); self.path=route(ground,pose.xy,self.goal,cfg.planning_wall_budget) or []
            self.last_route=pose.t
            d.reason=f'Return replan {(time.monotonic()-begin)*1000:.1f} ms'
        if self.path and ground.line_clear(pose.xy,self.path[0]):
            delta=self.path[0]-pose.xy
            speed=min(p.speed,math.sqrt(max(0.,2*cfg.braking*np.linalg.norm(delta))))
            v=delta*min(.8,speed/max(np.linalg.norm(delta),1e-9))
            d.vn,d.ve=map(float,v)
        else: d.reason='Holding for a checked return route/frontier'
        return d,None
