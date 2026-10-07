"""Manager-owned orange approach proposals; never reads MAVLink or sends commands."""
import math
import numpy as np

from autonomy.perception.hybrid_banner_detector import HybridBannerDetector
from native.common.types import Attitude
from native.common.validity import sector_clearance, scan_valid
from native.mission_runner import VehiclePose
from entrance_readiness import EntranceReadiness
from coverage_mission.engine import Decision
from coverage_mission.field_frame import FieldFrame


class ReturnApproach:
    def __init__(self, cfg, profile, args, snapshot, preview=None, activate=None):
        self.cfg=cfg; self.profile=profile; self.args=args; self.snapshot=snapshot
        self.frame=FieldFrame(cfg); self.preview=preview
        self.activate=activate; self.activated=False
        self.detector=HybridBannerDetector(lower_green=profile.orange_hsv_low,
            upper_green=profile.orange_hsv_high,
            min_area_at_640x480=args.banner_min_area_640x480,
            morph_kernel_size=args.banner_morph_kernel,
            aspect_ratio_bounds=args.banner_aspect_range,min_extent=args.banner_min_extent)
        self.readiness=EntranceReadiness()
        self.state='RETURN_BANNER_SEARCH'; self.started=None; self.wall_started=None
        self.last_seq=-1; self.last_seen=-math.inf; self.centered=0
        self.last_result=None; self.target_alt=None; self.settled=None
        self.last_pose_t=None

    def interrupt(self):
        self.centered=0; self.settled=None; self.readiness.reset()

    def __call__(self, pose, now, source_now):
        p=self.profile; a=self.args
        if self.last_pose_t is not None and pose.t-self.last_pose_t>self.cfg.pose_gap*2:
            self.interrupt()
        self.last_pose_t=pose.t
        if not self.activated:
            if self.activate: self.activate()
            if self.preview and self.preview.enabled:
                self.preview.start('Orange Banner Camera — MASK')
            self.activated=True
        if self.started is None: self.started=pose.t; self.wall_started=now
        d=Decision(self.state,'Orange banner approach',yaw=p.heading-self.cfg.field_yaw,
                   source_t=pose.t)
        out=d.as_dict(); out['entrance_permit']=False
        if pose.t-self.started>p.approach_seconds or now-self.wall_started>360:
            out.update(state='ABORTED',reason='Orange approach deadline exceeded'); return out
        frame,seq,receipt,stamp,scan,scan_seq,fault=self.snapshot()
        if fault:
            out.update(state='ABORTED',reason=f'Approach sensor clock: {fault}'); return out
        local=self.frame.local_point(pose.xy); yaw=pose.yaw+self.cfg.field_yaw
        if not p.approach_contains(local,pose.alt):
            out.update(state='ABORTED',reason='Orange approach envelope lost'); return out
        if (frame is None or stamp is None or not 0<=now-receipt<=a.camera_max_age
                or not 0<=source_now-stamp*1e-9<=self.cfg.frame_age):
            self.centered=0; self.settled=None
            out['reason']='Waiting for fresh forward camera'; return out
        fresh_scan=bool(scan is not None and scan_valid(scan) and scan.age_s<=.30
            and scan.source_timestamp is not None and
            0<=source_now-scan.source_timestamp*1e-9<=self.cfg.frame_age)
        if not fresh_scan:
            self.readiness.reset(); self.settled=None
            out['reason']='Waiting for fresh approach LiDAR'; return out
        if seq!=self.last_seq:
            self.last_seq=seq
            self.last_result=self.detector.detect(frame,
                relaxed_approach=self.state!='RETURN_BANNER_SEARCH',
                debug=bool(self.preview and self.preview.enabled))
            if self.preview:
                debug_frame=self.last_result['debug_frame']
                if debug_frame is not None:
                    import cv2
                    cv2.rectangle(debug_frame,(2,2),(145,20),(0,0,0),-1)
                    cv2.putText(debug_frame,'ORANGE MASK',(5,15),cv2.FONT_HERSHEY_SIMPLEX,
                                .4,(255,255,255),1)
                self.preview.submit(debug_frame)
            if self.last_result['detected']:
                self.last_seen=source_now
                ex=self.last_result['error_x']*640/frame.shape[1]
                ey=self.last_result['error_y']*480/frame.shape[0]
                self.centered=self.centered+1 if max(abs(ex),abs(ey))<a.center_tolerance_px else 0
            else: self.centered=0
        result=self.last_result
        yaw_error=math.atan2(math.sin(yaw-p.heading),math.cos(yaw-p.heading))
        if abs(yaw_error)>math.radians(10):
            out['reason']='Settling at registered orange approach heading'; return out
        if self.state in ('RETURN_BANNER_SEARCH','RETURN_BANNER_CENTER','RETURN_APPROACH'):
            if not result or not result['detected'] or source_now-self.last_seen>self.cfg.frame_age:
                # No unchecked lateral search. The registered stand-off faces
                # the entrance; persistent non-detection has a finite deadline.
                out['reason']='Holding for orange banner reacquisition'; return out
            ex=result['error_x']*640/frame.shape[1]; ey=result['error_y']*480/frame.shape[0]
            right=float(np.clip(ex*a.camera_gain,-a.camera_max_speed,a.camera_max_speed))
            down=float(np.clip(ey*a.camera_gain,-a.camera_max_speed,a.camera_max_speed))
            forward=0.
            if self.state=='RETURN_BANNER_SEARCH': self.state='RETURN_BANNER_CENTER'
            if self.state=='RETURN_BANNER_CENTER' and self.centered>=a.center_frames:
                self.state='RETURN_APPROACH'; self.detector.panel_only=False; self.detector.reset_track()
            if self.state=='RETURN_APPROACH':
                front=sector_clearance(scan,half_cone_deg=10.)
                if front is None:
                    out['reason']='Unknown forward clearance'; return out
                out['entrance_permit']=True
                if front<=a.entrance_commit_range:
                    target=pose.alt-a.pre_entry_descent
                    if target<p.staging_min_altitude or target>3.0:
                        out.update(state='ABORTED',reason='Unsafe return staging height'); return out
                    self.target_alt=target; self.state='RETURN_STAGE_DESCEND'
                    self.readiness.reset(); self.settled=None
                    forward=right=down=0.
                else: forward=abs(a.approach_speed)
            ne=np.array([forward*math.cos(yaw)-right*math.sin(yaw),
                         forward*math.sin(yaw)+right*math.cos(yaw)])
            out['vn'],out['ve']=map(float,self.frame.vector(ne)); out['vd']=down
        else:
            out['entrance_permit']=True
            # Once staged, the vertical marker may no longer be in the front
            # view. Its identity/entrance remain latched; readiness uses LiDAR.
            error=pose.alt-self.target_alt
            out['vd']=float(np.clip(error,-.5,.5))
            settled=(abs(error)<=.10 and
                     max(math.hypot(pose.vn,pose.ve),abs(pose.vd))<=p.settled_speed)
            if settled:
                if self.settled is None: self.settled=pose.t
                attitude=Attitude(pose.roll,pose.pitch,yaw,now)
                vehicle_pose=VehiclePose(float(local[0]),float(local[1]),yaw,now)
                self.readiness.observe(scan_seq,scan,attitude,vehicle_pose)
                if pose.t-self.settled>=2. and self.readiness.count>=self.readiness.required_scans:
                    self.state='RETURN_ENTRY_READY'; out['vd']=0.
            else:
                self.settled=None; self.readiness.reset()
        out['state']=self.state
        return out
