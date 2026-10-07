"""Pure mission logic. No simulator, sockets, vehicle commands, or GUI imports."""
from collections import deque
from dataclasses import dataclass, asdict
import math
import time
import cv2
import numpy as np
from scipy import ndimage
from .geometry import Projector, validated_hsv, red_mask
from .planning import GroundMap, CoveragePlan, route, astar


class Residence:
    """Continuous union-of-red residency; invalid data never resets a timer."""
    def __init__(self):
        self.entered=None
        self.warned=False
        self.failed=False
        self.events=[]
        self.last_t=None

    def update(self,t,inside,entry_hint=None):
        if self.last_t is not None and t<self.last_t:
            raise ValueError('Residence clock reset')
        self.last_t=t
        if inside is True and self.entered is None:
            self.entered=min(t,entry_hint if entry_hint is not None else t)
            self.warned=False
            self.events.append({'event':'ENTER','t':self.entered,'detected':t})
        if self.entered is not None:
            elapsed=t-self.entered
            if elapsed>=5 and inside is not False and not self.warned:
                self.warned=True
                self.events.append({'event':'AUTONOMOUS_WARNING','t':t,'remaining':max(0.,10-elapsed)})
            if elapsed>=10 and inside is not False:
                if not self.failed: self.events.append({'event':'DEADLINE_FAILURE','t':t})
                self.failed=True
            if inside is False:
                # Exit first confirmed after deadline still counts as a failure.
                if elapsed>10:
                    if not self.failed:
                        self.events.append({'event':'DEADLINE_FAILURE','t':t,'reason':'Late exit confirmation'})
                    self.failed=True
                self.events.append({'event':'EXIT','t':t,'duration':elapsed})
                self.entered=None; self.warned=False
        return self.remaining(t)

    def remaining(self,t):
        return None if self.entered is None else max(0.,10-(t-self.entered))


@dataclass
class Decision:
    state: str
    reason: str
    vn: float = 0.
    ve: float = 0.
    vd: float = 0.
    yaw: float = 0.
    source_t: float = 0.
    frame_t: float = -1.
    camera_t: float = -1.
    pending: int = 0
    unseen: int = 0
    entered: float | None = None
    warned: bool = False
    failed: bool = False

    def as_dict(self): return asdict(self)


class Engine:
    def __init__(self,cfg):
        self.cfg=cfg
        self.projector=Projector(cfg)
        self.ground=GroundMap(cfg)
        self.plan=CoveragePlan(cfg)
        self.residence=Residence()
        self.history=deque(maxlen=2000)
        self.path=[]
        width,length=cfg.camera.footprint(cfg.altitude-cfg.altitude_tolerance-cfg.ground_above_home)
        many_swaths=max(cfg.n_max-cfg.n_min,cfg.e_max-cfg.e_min)>4*max(width,length)
        self.replan_period=max(width,length)/cfg.speed if many_swaths else .5
        self.last_plan=-math.inf
        self.next_route_attempt=-math.inf
        self.last_planning_wall_ms=0.
        self.planning_timeouts=0
        self.no_route_since=None
        self.last_pose=None
        self.last_velocity=np.zeros(2)
        self.last_progress_t=None
        self.progress_done=0
        self.progress_observed=0
        self.progress_origin=None
        self.max_relocation_distance=0.
        self.settle_t=None
        self.terminal=None
        self.last_frame_pose=None
        self.turn_braking=False
        self.turn_settle_t=None
        self.repair_viewpoint=False
        self.repair_settle_t=None
        self.inspection_pause=None

    def observe(self,frame,pose,hsv=None):
        hsv=validated_hsv(frame,self.cfg) if hsv is None else hsv
        mask=red_mask(hsv,self.cfg)
        if self.cfg.assume_nonred_ground_clear:
            # Provisional mission rule: dark/black imagery is ground unless
            # the red detector marks it red. Transport freshness remains
            # separate; image content no longer rejects a black frame.
            usable=np.full(mask.shape,255,np.uint8)
        else:
            usable=((hsv[:,:,2]>=self.cfg.red_value_min)&
                    ((hsv[:,:,2]<=250)|(hsv[:,:,1]>=self.cfg.red_saturation_min))).astype(np.uint8)*255
        if not self.cfg.assume_nonred_ground_clear and self.cfg.neutral_print_support_m:
            # The known ground prints are black/white, not red. Accept a dark
            # pixel only where nearby bright neutral print is actually visible;
            # a large black patch or dark frame cannot become clear evidence.
            # Red-dominated neighbours veto that inference. This does not
            # decode or locate QR targets.
            height=pose.alt-self.cfg.ground_above_home-self.cfg.camera.down_offset
            support=max(3,int(round(self.cfg.neutral_print_support_m*self.cfg.camera.fx/height)))
            support=min(support|1,min(frame.shape[:2])//2*2-1)
            bright_neutral=((hsv[:,:,1]<=40)&(hsv[:,:,2]>=80)).astype(np.uint8)*255
            usable[bright_neutral>0]=255
            nearby_white=cv2.blur(bright_neutral,(support,support))
            nearby_red=cv2.blur(mask,(support,support))
            printed_dark=(hsv[:,:,2]<self.cfg.red_value_min)&(nearby_white>=50)&(nearby_red<128)
            usable[printed_dark]=255
        if not self.cfg.assume_nonred_ground_clear and self.cfg.green_texture_support_m:
            # Dark grass blades have unreadable individual pixels, but a small
            # ground patch can still carry a stable green signal. Keep this
            # scene-specific evidence separate from the red detector. A black
            # patch or a dark neutral/red patch remains unknown. The support
            # is in metres, so changing lens/resolution does not bake in 51 px.
            height=pose.alt-self.cfg.ground_above_home-self.cfg.camera.down_offset
            support=max(3,int(round(self.cfg.green_texture_support_m*self.cfg.camera.fx/height)))
            support=min(support|1,min(frame.shape[:2])//2*2-1)
            mean=cv2.blur(frame,(support,support))
            blue=mean[:,:,0].astype(np.int16)
            green=mean[:,:,1].astype(np.int16)
            red=mean[:,:,2].astype(np.int16)
            green_ground=(green>=self.cfg.green_texture_mean_min)&(green*5>=red*6)&(green*5>=blue*6)
            usable[green_ground]=255
        footprint=self.projector.footprint(pose)
        if self.ground.observe_pixels(footprint,mask,usable,pose.t):
            self.last_frame_pose=pose
        return mask

    def _entry_hint(self):
        # Backdate late discoveries through pose history to the last possible crossing.
        start=None
        for p in reversed(self.history):
            if not self.ground.inside_red(p.xy):
                if start is not None: start=p.t
                break
            start=p.t
        return start

    def _escape(self,xy):
        g=self.ground; start=g.cell(xy)
        if not g.contains(start) or not g.inset[start] or not g.observed[start]: return []
        if not np.any(g.free): return []
        # Escape can traverse the currently occupied red region, but only toward free
        # space, within already observed ground and the fence. Never launch into unknown.
        dist,indices=ndimage.distance_transform_edt(~g.free,return_indices=True)
        allowed=g.observed & g.inset
        heap=deque([start]); parent={start:None}; goal=None
        while heap:
            p=heap.popleft()
            if g.free[p]: goal=p; break
            for di,dj in ((1,0),(-1,0),(0,1),(0,-1)):
                q=(p[0]+di,p[1]+dj)
                if not g.contains(q) or q in parent or not allowed[q]: continue
                if dist[q]>dist[p]+1e-6: continue
                parent[q]=p; heap.append(q)
        if goal is None: return []
        path=[]
        while goal!=start:
            path.append(g.point(goal)); goal=parent[goal]
        path=path[::-1]
        # Grid resolution must not impose crawl-speed recovery at every cell.
        # Use a checked look-ahead only when distance to known free space never
        # increases along the entire shortcut.
        for index in range(len(path)-1,-1,-1):
            target=path[index]
            samples=np.linspace(xy,target,max(2,int(np.linalg.norm(target-xy)/(g.cfg.resolution*.25))+1))
            values=[dist[g.cell(p)] for p in samples]
            if g.line_clear(xy,target,allowed) and np.all(np.diff(values)<=1e-6):
                return path[index:]
        return path

    def step(self,pose,camera_t=-1.,inspection=None,descent_active=False,transition=None):
        cfg=self.cfg; g=self.ground
        d=Decision('HOLD','Waiting for valid observation',yaw=inspection.yaw if inspection is not None else cfg.heading,
                   source_t=pose.t,frame_t=g.last_t if math.isfinite(g.last_t) else -1.,camera_t=camera_t,
                   pending=len(self.plan.pending()),
                   unseen=int((~g.observed & ~g.contextual_clear & ~g.red & ~g.enclosed).sum()))
        if not pose.valid(): return d
        if self.last_pose:
            dt_pose=pose.t-self.last_pose.t
            horizontal_residual=(np.linalg.norm(
                pose.xy-self.last_pose.xy-
                np.array([pose.vn+self.last_pose.vn,pose.ve+self.last_pose.ve])*dt_pose*.5)
                if dt_pose>=0 else math.inf)
            altitude_residual=(abs(pose.alt-self.last_pose.alt+
                                   (pose.vd+self.last_pose.vd)*dt_pose*.5)
                               if dt_pose>=0 else math.inf)
            if (dt_pose<0 or horizontal_residual>.30+.25*dt_pose
                    or altitude_residual>.40+.25*dt_pose
                    or abs(math.atan2(math.sin(pose.yaw-self.last_pose.yaw),
                                      math.cos(pose.yaw-self.last_pose.yaw)))>
                       max(math.radians(5),dt_pose)):
                self.terminal='ABORTED: clock/origin/pose discontinuity'
        if self.last_pose is None or pose.t>self.last_pose.t:
            self.history.append(pose)
        while self.history and pose.t-self.history[0].t>30: self.history.popleft()
        dt=.05 if self.last_pose is None else max(.001,min(.2,pose.t-self.last_pose.t))
        if self.last_pose and pose.t-self.last_pose.t>.5: self.last_velocity[:]=0
        self.last_pose=pose
        cell=g.cell(pose.xy)
        admitted=bool(transition is not None and transition.allows(pose))
        if (not g.contains(cell) or not g.inset[cell]) and not admitted:
            self.terminal='ABORTED: outside operational fence'
        inside=g.inside_red(pose.xy)
        self.residence.update(pose.t,inside,self._entry_hint())
        d.entered=self.residence.entered; d.warned=self.residence.warned; d.failed=self.residence.failed
        if self.terminal:
            d.state,d.reason=self.terminal.split(': ',1); return d
        intentional_descent=descent_active or (inspection is not None and inspection.state in ('TARGET_DESCEND','TARGET_HOLD_5M'))
        if (not intentional_descent and abs(pose.alt-cfg.altitude)>cfg.altitude_tolerance):
            d.reason='Altitude outside coverage envelope'; return d
        if abs(pose.roll)>cfg.max_tilt or abs(pose.pitch)>cfg.max_tilt:
            d.reason='Attitude outside projection envelope'; return d
        fresh=0<=pose.t-g.last_t<=cfg.frame_age
        # Rejected high-motion projections are not a dead camera. Briefly follow
        # only the already observed, clearance-checked map with fresh usable pixels.
        # No new map evidence or traversal credit is manufactured during this grace.
        camera_live=0<=pose.t-camera_t<=cfg.frame_age
        mapping_grace=camera_live and 0<=pose.t-g.last_t<=cfg.mapping_motion_grace
        escape=(not transition.line_clear(g,pose.xy,pose.xy,pose.alt) if admitted
                else not g.free[cell])
        if admitted and escape:
            d.state='ABORTED'; d.reason='Entrance transition ground/clearance invalid'; return d
        if not (fresh or mapping_grace) and not (escape and np.any(g.red)):
            # Timer still runs. Mapping is frozen; runtime treats this as a hold/fault.
            self.last_velocity[:]=0
            d.reason='Stale/unmatched camera frame'; return d
        g.visits[cell]=min(65535,int(g.visits[cell])+1)
        if self.residence.failed and not inside:
            self.terminal='ABORTED: red-zone deadline exceeded'
            d.state='ABORTED'; d.reason='Red-zone deadline exceeded'; return d
        if inspection is not None and not escape:
            # Only selection/progress timers pause; observation, residency and
            # all safety gates above continue. Never credit inspection traversal.
            if self.inspection_pause is None: self.inspection_pause=pose.t
            self.path=[]; self.turn_braking=False; self.turn_settle_t=None
            self.repair_viewpoint=False; self.repair_settle_t=None
            inspection.source_t=pose.t; inspection.frame_t=d.frame_t
            inspection.camera_t=camera_t; inspection.pending=d.pending; inspection.unseen=d.unseen
            inspection.entered=d.entered; inspection.warned=d.warned; inspection.failed=d.failed
            v=np.array([inspection.vn,inspection.ve])
            for checked in (v,np.array([pose.vn,pose.ve])):
                speed=np.linalg.norm(checked)
                end=pose.xy+checked*(cfg.reaction_time+speed/(2*cfg.braking))
                clear=(transition.line_clear(g,pose.xy,end,pose.alt) if admitted
                       else g.line_clear(pose.xy,end))
                if not clear:
                    inspection.vn=inspection.ve=inspection.vd=0.
                    inspection.state='BRAKE'; inspection.reason='QR stopping region not observed permissible'
                    break
            self.last_velocity[:]=0
            return inspection
        if self.inspection_pause is not None:
            paused=pose.t-self.inspection_pause
            if self.last_progress_t is not None: self.last_progress_t+=paused
            if self.no_route_since is not None: self.no_route_since+=paused
            self.inspection_pause=None
        if fresh and not escape: self.plan.update(pose,g)
        pending=self.plan.pending()
        unseen=~g.observed & ~g.contextual_clear & ~g.red & ~g.enclosed
        d.pending=len(pending); d.unseen=int(unseen.sum())
        done_count=int(self.plan.done.sum())
        observed_count=int(g.observed.sum())
        required_cells=max(1,math.ceil(cfg.minimum_observation_progress_area/
                                       (cfg.resolution*cfg.resolution)))
        if (self.last_progress_t is None or done_count>self.progress_done or
                observed_count-self.progress_observed>=required_cells):
            self.last_progress_t=pose.t
            self.progress_done=done_count
            self.progress_observed=observed_count
            self.progress_origin=pose.xy
            self.max_relocation_distance=0.
        else:
            # A long, checked connector between the last credited lane and a
            # distant remaining patch is not a coverage stall. Credit only net
            # displacement from that last evidence point, not distance flown
            # around a loop, and cap it at the field diagonal. Thus repeated
            # circling still reaches a finite no-progress deadline.
            diagonal=math.hypot(cfg.n_max-cfg.n_min,cfg.e_max-cfg.e_min)
            self.max_relocation_distance=max(self.max_relocation_distance,
                min(diagonal,float(np.linalg.norm(pose.xy-self.progress_origin))))
        progress_allowance=(cfg.no_progress+
                            self.max_relocation_distance/cfg.relocation_credit_speed)
        if not escape and (len(pending) or unseen.any()) and pose.t-self.last_progress_t>progress_allowance:
            self.terminal='BLOCKED: no measured traversal/observation progress'
            d.state='BLOCKED'; d.reason='No measured traversal/observation progress'; return d
        if self.residence.failed and not inside:
            self.terminal='ABORTED: red-zone deadline exceeded'
            d.state='ABORTED'; d.reason='Red-zone deadline exceeded'; return d
        if escape:
            self.repair_viewpoint=False; self.repair_settle_t=None
            self.path=self._escape(pose.xy)
            d.state='ESCAPE'; d.reason='Exit incursion/clearance violation immediately'
        elif len(pending)==0 and not unseen.any():
            self.last_velocity[:]=0
            d.state='SETTLING'; d.reason='Coverage traversed; verifying stationary hold'
            if math.hypot(pose.vn,pose.ve)<.1 and abs(pose.vd)<.1:
                if self.settle_t is None: self.settle_t=pose.t
                if pose.t-self.settle_t>=2:
                    self.terminal='COMPLETE: full required coverage traversed'
                    d.state='COMPLETE'; d.reason='Full required coverage traversed'
            else: self.settle_t=None
            return d
        else:
            d.state='SWEEP' if len(pending) else 'COVERAGE_REPAIR'
            d.reason='Follow checked route to unfinished coverage'
            repair=not len(pending)
            if not repair:
                self.repair_viewpoint=False; self.repair_settle_t=None
            elif self.repair_viewpoint and not self.path:
                # A frontier is a viewing destination, not a continuously changing
                # pursuit target. Settle and acquire a new admitted image there.
                self.last_velocity[:]=0
                if math.hypot(pose.vn,pose.ve)>=.1 or abs(pose.vd)>=.1:
                    self.repair_settle_t=None
                elif self.repair_settle_t is None:
                    self.repair_settle_t=pose.t
                if (self.repair_settle_t is None or
                        pose.t-self.repair_settle_t<cfg.viewpoint_settle or
                        g.last_t<self.repair_settle_t+cfg.viewpoint_settle):
                    d.state='OBSERVE'; d.reason='Settle at repair viewpoint for a valid observation'
                    return d
                self.repair_viewpoint=False; self.repair_settle_t=None
            # Do not delete or mark obstructed rows as traversed.
            # The large field cannot afford a whole-field A* search every
            # half-second. Replan at most once per camera-swath travel time,
            # or immediately when a leg is reached/invalidated. Every current
            # leg is still checked against fresh red evidence below.
            if (((not repair and pose.t-self.last_plan>=self.replan_period) or not self.path)
                    and pose.t>=self.next_route_attempt):
                target=(self.plan.reachable_target(g,pose.xy,pending) if len(pending)
                        else g.point(np.argwhere(unseen)[0]))
                plan_started=time.monotonic()
                candidate=route(g,pose.xy,target,cfg.planning_wall_budget)
                self.last_planning_wall_ms=(time.monotonic()-plan_started)*1000
                if candidate is None:
                    self.planning_timeouts+=1
                    # Retain only a still-checked leg. If none exists, stop and
                    # retry after a bounded delay instead of spinning on A*.
                    if self.path and not g.line_clear(pose.xy,self.path[0]):
                        self.path=[]
                    self.next_route_attempt=pose.t+.25
                else:
                    self.path=candidate
                    self.last_plan=pose.t
                self.repair_viewpoint=repair and bool(self.path)
        while self.path and np.linalg.norm(self.path[0]-pose.xy)<min(cfg.resolution/3,cfg.arrival):
            # A recovery endpoint's tolerance disk may still overlap the unsafe
            # cell. Exit is established by actual geometry, not waypoint proximity.
            if escape and len(self.path)==1 and not g.free[cell]: break
            # Reaching a tolerance disk must not cut the following corner.
            if len(self.path)>1 and not escape and not g.line_clear(pose.xy,self.path[1]): break
            if escape and len(self.path)>1 and not g.line_clear(pose.xy,self.path[1],g.observed & g.inset): break
            self.path.pop(0)
        if not self.path:
            if self.no_route_since is None: self.no_route_since=pose.t
            self.last_velocity[:]=0
            d.state='HOLD'; d.reason='No checked route; acquiring ground observations'
            # New pixels from hover jitter are not navigation progress. Do not
            # let them keep an unroutable aircraft in HOLD indefinitely.
            if pose.t-self.no_route_since>cfg.no_progress:
                self.terminal='BLOCKED: no observable/reachable coverage progress'
                d.state='BLOCKED'; d.reason='No observable/reachable coverage progress'
            return d
        self.no_route_since=None
        delta=self.path[0]-pose.xy; distance=np.linalg.norm(delta)
        direction=delta/max(distance,1e-9)
        speed=min(cfg.speed,math.sqrt(2*cfg.braking*distance),distance*.9)
        desired=direction*speed
        if escape:
            self.turn_braking=False  # Never defer an incursion exit for lane alignment.
            self.turn_settle_t=None
        else:
            measured=np.array([pose.vn,pose.ve])
            # A new connector can turn sharply while the aircraft still carries
            # lateral momentum. Settle before starting that leg, instead of
            # sweeping a curved transient outside its tracking allowance.
            for v in (measured,self.last_velocity):
                along=float(v@direction)
                lateral=float(np.linalg.norm(v-along*direction))
                stop=lateral*cfg.reaction_time+lateral*lateral/(2*cfg.acceleration)
                if stop>cfg.uncertainty/2 or along<-.1:
                    self.turn_braking=True
            if self.turn_braking:
                self.last_velocity[:]=0
                if np.linalg.norm(measured)>=.1: self.turn_settle_t=None
                elif self.turn_settle_t is None: self.turn_settle_t=pose.t
                if self.turn_settle_t is not None and pose.t-self.turn_settle_t>=.3:
                    self.turn_braking=False
                    self.turn_settle_t=None
                else:
                    desired[:]=0
                    d.state='TURN_BRAKE'; d.reason='Settle lateral momentum before changing route direction'
        change=desired-self.last_velocity
        change*=min(1.,cfg.acceleration*dt/max(np.linalg.norm(change),1e-9))
        velocity=self.last_velocity+change
        # Safety overrides acceleration ramp: brake immediately if trajectory unsafe.
        if not escape:
            for v in (velocity,np.array([pose.vn,pose.ve])):
                s=np.linalg.norm(v)
                end=pose.xy+v*(cfg.reaction_time+s/(2*cfg.braking))
                if not g.line_clear(pose.xy,end):
                    velocity[:]=0; d.state='BRAKE'; d.reason='Stopping region not observed permissible'
                    break
            if not g.line_clear(pose.xy,self.path[0]):
                velocity[:]=0; self.path=[]; self.repair_viewpoint=False
                self.repair_settle_t=None
                d.state='BRAKE'; d.reason='Route invalidated by new observation'
        self.last_velocity=velocity
        d.vn,d.ve=map(float,velocity)
        d.vd=float(np.clip((pose.alt-cfg.altitude)*.6,-.3,.3))
        return d
