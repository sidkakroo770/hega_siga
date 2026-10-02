"""Gazebo + MAVLink adapter. Non-ROS, airborne entry; never arms or takes off.

Simulation-only commissioning adapter. Pi camera acquisition/clock calibration must
be substituted and measured before hardware deployment. HOST watchdogs and SOURCE
physical-progress time are deliberately distinct.
"""
import os
os.environ.setdefault('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION','python')
import argparse
import hashlib
from collections import deque
from dataclasses import replace
import json
import math
import multiprocessing as mp
from pathlib import Path
import queue
import threading
import time
import numpy as np
from .config import Config
from .geometry import Pose, PoseHistory, decode_image, UnusableImage, validated_hsv
from scipy.spatial.transform import Rotation


class SimClock:
    """Calibrate Gazebo minus SITL boot-time offset from paired reception samples.

Both source clocks advance with JSON physics, with a small measured rate drift.
Track epoch drift at no more than 2 ms per source second, retaining a 50 ms
maximum residual gate. Abrupt changes invalidate data rather than moving time.
This reception-based estimate is NOT a hardware synchronization protocol.
An optional independently measured offset remains fixed, but is still validated.
"""
    def __init__(self,offset=None):
        self.offset=offset
        self.samples=deque(maxlen=100)
        self.error=math.inf
        self.last_boot=None
        self.fixed=offset is not None

    def add(self,boot,sim):
        if self.last_boot is not None and boot<self.last_boot:
            raise ValueError('Autopilot clock reset')
        dt=0 if self.last_boot is None else boot-self.last_boot
        self.last_boot=boot
        self.samples.append((boot,sim-boot))
        if len(self.samples)<20 or self.samples[-1][0]-self.samples[0][0]<1: return
        offsets=np.array([s[1] for s in self.samples])
        centre=float(np.median(offsets))
        if self.offset is None and np.max(np.abs(offsets-centre))<=.05:
            self.offset=centre
        if self.offset is not None:
            if not self.fixed:
                self.offset+=float(np.clip(centre-self.offset,-.002*dt,.002*dt))
            self.error=float(np.max(np.abs(offsets-self.offset)))

    @property
    def ready(self): return self.offset is not None and self.error<=.05


class RoundTripClock(SimClock):
    """MAVLink TIMESYNC midpoint fit with bounded source-time round-trip error.

    Unlike pairing a queued telemetry packet with reception time, the responder
    timestamp is enclosed by a measured request/response interval. No fitted
    latency correction or simulator truth is used by the flight controller.
    """
    def __init__(self,offset=None):
        super().__init__(offset)
        self.pending={}
        self.last_reply=0.
        self.rtt=math.inf
        self.last_requested_sim=None

    def request(self,master,sim):
        if self.last_requested_sim is not None and sim<=self.last_requested_sim: return
        self.last_requested_sim=sim
        token=time.monotonic_ns()
        self.pending[token]=(sim,time.monotonic())
        self.pending={k:v for k,v in self.pending.items() if time.monotonic()-v[1]<2}
        master.mav.timesync_send(0,token)

    def reply(self,msg,sim):
        sent=self.pending.pop(msg.ts1,None)
        if sent is None or msg.tc1<=0: return
        # A paused simulator can have a tiny SOURCE RTT despite seconds of host
        # delay. Such an exchange does not bound when the responder timestamped it.
        if time.monotonic()-sent[1]>1: return
        rtt=sim-sent[0]
        if not 0<rtt<=.04: return
        boot=msg.tc1*1e-9
        # TIMESYNC uses a UART receive-time estimate, not the ordered telemetry
        # stream. Delayed/reordered replies cannot declare an autopilot reboot.
        # Actual ATTITUDE/POSITION boot-time regression is checked independently.
        if self.last_boot is not None and boot<=self.last_boot: return
        candidate=(sent[0]+sim)/2-boot
        if self.offset is not None and abs(candidate-self.offset)>.025:
            # Reject an exchange outside the alignment contract, not all following
            # good exchanges for a whole rolling-window duration. Persistent shifts
            # still invalidate synchronization and reach the normal outage abort.
            self.error=math.inf
            return
        self.rtt=rtt
        self.add(boot,(sent[0]+sim)/2)
        self.last_reply=time.monotonic()

    @property
    def ready(self):
        return super().ready and self.error<=.025 and time.monotonic()-self.last_reply<2


def put_latest(q,value):
    try: q.put_nowait(value); return
    except queue.Full: pass
    try: q.get_nowait()
    except queue.Empty: pass
    try: q.put_nowait(value)
    except queue.Full: pass


def clock_diagnostics(sync):
    # Unknown/rejected synchronization is a normal HOLD condition. JSONL uses
    # null, not non-standard Infinity/NaN that could itself abort supervision.
    return {name:value if value is not None and math.isfinite(value) else None
            for name,value in (('clock_error',sync.error),('clock_offset',sync.offset),('clock_rtt',sync.rtt))}


class TelemetryHistory:
    """Join independent MAVLink streams at position SOURCE time, not reception time."""
    def __init__(self):
        self.attitudes=PoseHistory(seconds=3)
        self.altitudes=PoseHistory(seconds=3)
        self.pending=deque(maxlen=150)
        self.poses=PoseHistory()

    def add(self,msg):
        t=msg.time_boot_ms*.001
        if msg.get_type()=='ATTITUDE':
            self.attitudes.add(Pose(t,0,0,0,msg.roll,msg.pitch,msg.yaw))
        elif msg.get_type()=='GLOBAL_POSITION_INT':
            self.altitudes.add(Pose(t,0,0,msg.relative_alt*.001))
        elif msg.get_type()=='LOCAL_POSITION_NED':
            if self.pending and t<self.pending[-1].time_boot_ms*.001:
                raise ValueError('Position clock reset')
            self.pending.append(msg)

    def assemble(self,offset,max_gap):
        if not self.attitudes.samples or not self.altitudes.samples: return
        latest=min(self.attitudes.samples[-1].t,self.altitudes.samples[-1].t)
        while self.pending and self.pending[0].time_boot_ms*.001<=latest:
            msg=self.pending.popleft(); t=msg.time_boot_ms*.001
            a=self.attitudes.at(t,max_gap); h=self.altitudes.at(t,max_gap)
            if a is not None and h is not None:
                self.poses.add(Pose(t+offset,msg.x,msg.y,h.alt,a.roll,a.pitch,a.yaw,
                                    msg.vx,msg.vy,msg.vz))


def projection_motion_valid(history,pose,cfg):
    """Bound angular timing sensitivity using telemetry, never simulator truth.

    Require evidence on both sides of exposure. A single latest attitude or
    constant heading bias cannot manufacture that evidence. Work is constant-size.
    """
    tilt=math.acos(float(np.clip(math.cos(pose.roll)*math.cos(pose.pitch),-1.,1.)))
    if tilt>cfg.max_projection_tilt: return False
    w=cfg.projection_motion_window
    a=history.at(pose.t-w,cfg.pose_gap); b=history.at(pose.t+w,cfg.pose_gap)
    if a is None or b is None: return False
    rots=Rotation.from_euler('xyz',[[p.roll,p.pitch,p.yaw] for p in (a,pose,b)])
    rates=(rots[:-1].inv()*rots[1:]).magnitude()/w
    return bool(np.max(rates)<=cfg.max_projection_angular_rate)


def matched_frame(frames,history,now_source,cfg,cache=None):
    """Newest bracketed image; a just-arrived unbracketed image cannot starve work."""
    cache={} if cache is None else cache
    live={(image[3],image[0]) for image in frames}
    for key in list(cache):
        if key not in live: del cache[key]
    for image in reversed(frames):
        if not 0<=now_source-image[0]<=cfg.frame_age: continue
        key=(image[3],image[0])
        if key not in cache:
            if not history.samples or history.samples[-1].t<image[0]+cfg.projection_motion_window-1e-8:
                continue  # Future bracketing data may still arrive; do not cache absence.
            pose=history.at(image[0],cfg.pose_gap)
            cache[key]=pose if pose is not None and projection_motion_valid(history,pose,cfg) else None
        if cache[key] is not None: return image,cache[key]
    return None,None


def current_pose(history,now_source,max_gap):
    if not history.samples: return None
    latest=history.samples[-1]
    # A bounded epoch-estimation error can put the latest received sample a few
    # milliseconds ahead of the Gazebo clock. Interpolate within the available
    # history instead of either using a future pose or spuriously dropping health.
    return history.at(now_source,max_gap) if latest.t>now_source else latest


def job_due(wall,last_wall,source,last_source,cfg):
    return wall-last_wall>=cfg.minimum_worker_wall_period and source-last_source>=cfg.decision_period


def worker(cfg,inputs,outputs,gui,artifact):
    import cv2
    from .engine import Engine
    engine=Engine(cfg)
    last_seq=-1
    last_camera_seq=-1
    camera_t=-1.
    tracked_segment=None
    credited=np.zeros(len(engine.plan.points),bool)
    view_image=np.zeros((cfg.camera.height,cfg.camera.width,3),np.uint8)
    try:
        while True:
            job=inputs.get()
            if job is None: return
            seq,image,frame_pose,current,delay,raw_frame=job
            if delay: time.sleep(delay)
            processing_started=time.monotonic()
            try:
                if raw_frame is not None and raw_frame[3]!=last_camera_seq:
                    last_camera_seq=raw_frame[3]
                    try:
                        validated_hsv(raw_frame[2],cfg)
                        camera_t=raw_frame[0]
                    except UnusableImage:
                        pass
                if frame_pose is not None and seq!=last_seq and (abs(frame_pose.alt-cfg.altitude)<=cfg.altitude_tolerance
                        and abs(frame_pose.roll)<=cfg.max_tilt and abs(frame_pose.pitch)<=cfg.max_tilt):
                    try:
                        engine.observe(image,frame_pose); last_seq=seq
                        view_image=image
                    except UnusableImage:
                        pass  # No observations/coverage credited; freshness expires.
                decision=engine.step(current,camera_t)
                payload=decision.as_dict()
                payload['events']=engine.residence.events[-10:]
                payload['position']=[current.n,current.e]
                payload['altitude']=current.alt
                payload['attitude']=[current.roll,current.pitch,current.yaw]
                payload['measured_velocity']=[current.vn,current.ve,current.vd]
                payload['newly_traversed_points']=engine.plan.points[engine.plan.done & ~credited].tolist()
                credited=engine.plan.done.copy()
                # Diagnostics only: keep a fixed reference for each commanded
                # straight leg, rather than measuring error against a moving ray.
                if decision.state=='SWEEP' and engine.path:
                    target=engine.path[0].tolist()
                    if tracked_segment is None or tracked_segment['end']!=target:
                        tracked_segment={'start':current.xy.tolist(),'end':target,'since':current.t}
                    payload['tracking_segment']=tracked_segment
                else:
                    tracked_segment=None
                payload['observation_pose']=vars(engine.last_frame_pose) if engine.last_frame_pose else None
                payload['processing_ms']=(time.monotonic()-processing_started)*1000
                put_latest(outputs,payload)
                if gui:
                    view=view_image.copy()
                    cv2.putText(view,decision.state,(12,25),cv2.FONT_HERSHEY_SIMPLEX,.7,(0,255,255),2)
                    if decision.entered is not None:
                        text=f'RED: {max(0,10-(current.t-decision.entered)):.1f}s remaining'
                        cv2.putText(view,text,(12,55),cv2.FONT_HERSHEY_SIMPLEX,.7,(0,0,255),2)
                    g=engine.ground
                    map_image=np.full((*g.shape,3),45,np.uint8)
                    map_image[g.observed]=(90,125,90)
                    map_image[g.inflated]=(20,90,180)
                    map_image[g.red]=(0,0,255)
                    for xy in engine.path:
                        c=g.cell(xy)
                        if g.contains(c): map_image[c]=(255,255,0)
                    c=g.cell(current.xy)
                    if g.contains(c): cv2.circle(map_image,(c[1],c[0]),2,(255,255,255),-1)
                    cv2.imshow('Coverage downward camera',view)
                    cv2.imshow('Coverage ground map (north up)',cv2.resize(map_image[::-1],(600,600),interpolation=cv2.INTER_NEAREST))
                    if cv2.waitKey(1)&255==ord('q'):
                        put_latest(outputs,{'state':'ABORTED','reason':'GUI quit'}); return
            except Exception as exc:
                put_latest(outputs,{'state':'ABORTED','reason':f'Worker: {exc}'}); return
    finally:
        np.savez_compressed(artifact,observed=engine.ground.observed,red=engine.ground.red,
                            confirmed=engine.ground.confirmed,enclosed=engine.ground.enclosed,
                            points=engine.plan.points,done=engine.plan.done,excluded=engine.plan.excluded)
        cv2.destroyAllWindows()


class Sensors:
    def __init__(self,cfg=None):
        from gz.transport13 import Node
        from gz.msgs10.image_pb2 import Image
        from gz.msgs10.clock_pb2 import Clock
        self.lock=threading.Lock()
        self.image=None; self.frames=deque(maxlen=8); self.seq=0; self.clock=None; self.error=None
        self.node=Node()
        cfg=cfg or Config()
        if not self.node.subscribe(Image,cfg.downward_topic,self.image_cb):
            raise RuntimeError('Downward image subscription failed')
        if not self.node.subscribe(Clock,cfg.clock_topic,self.clock_cb):
            raise RuntimeError('Clock subscription failed')

    def image_cb(self,msg):
        try:
            frame=decode_image(msg)
            t=msg.header.stamp.sec+msg.header.stamp.nsec*1e-9
            with self.lock:
                if self.image is not None and t<self.image[0]: self.error='Image clock reset'
                if self.image is not None and t==self.image[0]: return
                self.seq+=1; self.image=(t,time.monotonic(),frame,self.seq)
                self.frames.append(self.image)
        except Exception as exc:
            with self.lock: self.error=str(exc)

    def clock_cb(self,msg):
        t=msg.sim.sec+msg.sim.nsec*1e-9
        with self.lock:
            if self.clock is not None and t<self.clock[0]: self.error='Gazebo clock reset'
            self.clock=(t,time.monotonic())

    def snapshot(self):
        with self.lock: return list(self.frames),self.clock,self.error


def request_streams(master):
    from pymavlink import mavutil
    for message,us in ((32,20000),(30,20000),(33,50000),(193,100000)):
        master.mav.command_long_send(master.target_system,master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,0,message,us,0,0,0,0,0)


def velocity(master,vn,ve,vd,yaw):
    from pymavlink import mavutil
    master.mav.set_position_target_local_ned_send(int(time.monotonic()*1000)&0xffffffff,
        master.target_system,master.target_component,mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        2503,0,0,0,vn,ve,vd,0,0,0,yaw,0)


def hold(master,position,yaw):
    from pymavlink import mavutil
    master.mav.set_position_target_local_ned_send(int(time.monotonic()*1000)&0xffffffff,
        master.target_system,master.target_component,mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        2552,position.x,position.y,position.z,0,0,0,0,0,0,yaw,0)


def main(argv=None, master=None):
    from pymavlink import mavutil
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=Path(__file__).parents[1]/'config/coverage.json')
    parser.add_argument('--mavlink',default='udpin:127.0.0.1:14652')
    parser.add_argument('--fly',action='store_true',help='Control this isolated SITL vehicle after takeoff')
    parser.add_argument('--no-gui',action='store_true')
    parser.add_argument('--clock-offset',type=float,help='Verified Gazebo time minus autopilot boot time')
    parser.add_argument('--max-wall-seconds',type=float,default=1800)
    parser.add_argument('--entry-wall-seconds',type=float,default=30,
                        help='Abort if synchronized airborne entry is not achieved')
    parser.add_argument('--log',type=Path,default=Path(__file__).parents[1]/'artifacts/runtime.jsonl')
    parser.add_argument('--faults',type=Path,help='Owned SITL harness only; never use for flight')
    args=parser.parse_args(argv); cfg=Config.load(args.config)
    faults=None
    if args.faults:
        from .faults import Faults
        faults=Faults(args.faults)
    args.log.parent.mkdir(parents=True,exist_ok=True)
    manifest={'config':cfg.as_dict(),'fly':args.fly,'gui':not args.no_gui,'faults':faults.events if faults else [],
        'source_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest()
                         for p in Path(__file__).parent.glob('*.py')}}
    args.log.with_suffix('.manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    sensors=Sensors(cfg); sync=RoundTripClock(args.clock_offset)
    owns_master=master is None
    if owns_master:
        master=mavutil.mavlink_connection(args.mavlink,source_system=245)
    hb=master.wait_heartbeat(timeout=20)
    if hb is None: raise SystemExit('No simulator heartbeat')
    if hb.autopilot!=mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
        raise SystemExit('Expected ArduPilot heartbeat')
    master.target_system=hb.get_srcSystem()
    master.target_component=hb.get_srcComponent()
    request_streams(master)
    ctx=mp.get_context('spawn'); jobs=ctx.Queue(maxsize=1); results=ctx.Queue(maxsize=1)
    args.log.parent.mkdir(parents=True,exist_ok=True)
    proc=ctx.Process(target=worker,args=(cfg,jobs,results,not args.no_gui,args.log.with_suffix('.map.npz')),daemon=True)
    proc.start()
    position=attitude=altitude=None
    ekf_flags=0; ekf_wall=0; origin=None
    heartbeat_wall=time.monotonic(); pos_wall=att_wall=alt_wall=0
    telemetry=TelemetryHistory(); history=telemetry.poses
    frame_matches={}
    latest_result=None; result_wall=0; last_job=last_print=0
    last_job_source=-math.inf
    last_request=time.monotonic(); last_sync=0; started=time.monotonic(); active=False; entry_t=None
    state='WAITING'; reason='Awaiting synchronized airborne entry'; hold_pos=None
    missing_since=None; exit_code=1; last_logged=None; last_command=0
    args.log.parent.mkdir(parents=True,exist_ok=True)
    print(f'[COVERAGE] {"FLIGHT" if args.fly else "INSPECT ONLY"}; never arms/takes off',flush=True)
    try:
        with args.log.open('w') as log, args.log.with_suffix('.supervision.jsonl').open('w') as supervision:
            last_supervision=0
            while time.monotonic()-started<args.max_wall_seconds:
                now=time.monotonic()
                if not active and now-started>args.entry_wall_seconds:
                    raise RuntimeError('Synchronized coverage entry timed out')
                frames,clock,error=sensors.snapshot()
                injected=faults.active(clock[0]-entry_t) if faults and clock and entry_t is not None else {}
                image=frames[-1] if frames else None
                if error: raise RuntimeError(error)
                if not proc.is_alive(): raise RuntimeError('Perception/planning worker stopped')
                for _ in range(300):
                    msg=master.recv_match(blocking=False)
                    if msg is None: break
                    if msg.get_srcSystem()!=master.target_system or msg.get_srcComponent()!=master.target_component: continue
                    kind=msg.get_type()
                    if kind in ('ATTITUDE','GLOBAL_POSITION_INT','LOCAL_POSITION_NED'):
                        if 'telemetry_drop' in injected: continue
                        if 'clock_reset' in injected: msg.time_boot_ms=max(0,msg.time_boot_ms-5000)
                    if kind=='HEARTBEAT': hb=msg; heartbeat_wall=now
                    elif kind=='EKF_STATUS_REPORT': ekf_flags=msg.flags; ekf_wall=now
                    elif kind=='GPS_GLOBAL_ORIGIN':
                        new_origin=(msg.latitude,msg.longitude,msg.altitude)
                        if active and origin is not None and new_origin!=origin:
                            raise RuntimeError('EKF origin changed; registration invalid')
                        origin=new_origin
                    elif kind=='TIMESYNC':
                        _,reply_clock,_=sensors.snapshot()
                        if reply_clock: sync.reply(msg,reply_clock[0])
                    elif kind=='ATTITUDE': attitude=msg; att_wall=now; telemetry.add(msg)
                    elif kind=='GLOBAL_POSITION_INT': altitude=msg; alt_wall=now; telemetry.add(msg)
                    elif kind=='LOCAL_POSITION_NED':
                        position=msg; pos_wall=now
                        telemetry.add(msg)
                if sync.ready: telemetry.assemble(sync.offset,cfg.pose_gap)
                # Telemetry drain/assembly takes time. Compare against a fresh clock
                # snapshot, not the clock sampled before reading these messages.
                frames,clock,error=sensors.snapshot()
                if 'camera_drop' in injected: frames=[]
                if 'camera_delay' in injected and clock:
                    frames=[f for f in frames if f[0]<=clock[0]-injected['camera_delay']]
                if 'black_frame' in injected:
                    frames=[(f[0],f[1],np.zeros_like(f[2]),f[3]) for f in frames]
                image=frames[-1] if frames else None
                if error: raise RuntimeError(error)
                now=time.monotonic()
                if clock and now-clock[1]<cfg.sensor_wall_age and now-last_sync>.05:
                    sync.request(master,clock[0]); last_sync=now
                if now-last_request>5: request_streams(master); last_request=now
                armed=bool(hb.base_mode & 128); guided=hb.custom_mode==4
                # ATTITUDE, horizontal/vertical velocity, absolute horizontal/vertical
                # position; reject constant-position, uninitialized and GPS-glitch flags.
                estimator_ok=(ekf_flags & 55)==55 and not (ekf_flags & (128|1024|32768)) and now-ekf_wall<cfg.sensor_wall_age
                if 'ekf_invalid' in injected: estimator_ok=False
                if active and (not armed or not guided):
                    state='ABORTED'; reason='Vehicle disarmed or mode authority changed'; break
                heartbeat_ok=now-heartbeat_wall<=cfg.heartbeat_wall_age
                if not active and not heartbeat_ok: raise RuntimeError('Heartbeat lost before entry')
                pose=current_pose(history,clock[0],cfg.pose_gap) if clock else None
                def noise(p):
                    return replace(p,n=p.n+.04*math.sin(p.t),e=p.e+.04*math.cos(p.t),yaw=p.yaw+math.radians(2))
                if pose and 'pose_noise' in injected: pose=noise(pose)
                if pose and 'yaw_jump' in injected: pose=replace(pose,yaw=pose.yaw+math.pi/2)
                if pose and 'position_jump' in injected: pose=replace(pose,n=pose.n+1)
                pose_healthy=bool(pose and heartbeat_ok and estimator_ok and sync.ready and clock and
                    now-max(pos_wall,0)<cfg.sensor_wall_age and now-att_wall<cfg.sensor_wall_age and
                    now-alt_wall<cfg.sensor_wall_age and
                    now-clock[1]<cfg.sensor_wall_age and 0<=clock[0]-pose.t<=cfg.pose_gap*2)
                healthy=bool(pose_healthy and image and now-image[1]<cfg.sensor_wall_age)
                raw_frame=image if healthy and 0<=clock[0]-image[0]<=cfg.frame_age else None
                image,frame_pose=matched_frame(frames,history,clock[0],cfg,frame_matches) if healthy else (None,None)
                if frame_pose and 'pose_noise' in injected: frame_pose=noise(frame_pose)
                airborne=bool(pose and abs(pose.alt-cfg.altitude)<=cfg.altitude_tolerance)
                due=bool(clock and job_due(now,last_job,clock[0],last_job_source,cfg))
                if healthy and frame_pose and (active or airborne) and due:
                    put_latest(jobs,(image[3],image[2],frame_pose,pose,injected.get('worker_delay',0),raw_frame))
                    last_job=now; last_job_source=clock[0]
                elif active and pose_healthy and due:
                    # Keep residence/recovery alive with fresh localization and the
                    # frozen known map. No image means no new coverage evidence.
                    put_latest(jobs,(-1,None,None,pose,injected.get('worker_delay',0),raw_frame))
                    last_job=now; last_job_source=clock[0]
                try:
                    while True: latest_result=results.get_nowait(); result_wall=now
                except queue.Empty: pass
                if latest_result and latest_result.get('state')=='ABORTED':
                    state='ABORTED'; reason=latest_result['reason']; break
                if not active and healthy and frame_pose and armed and guided and abs(pose.alt-cfg.altitude)<=cfg.altitude_tolerance and abs(pose.vd)<.15:
                    active=True; entry_t=clock[0]; print('[ENTRY] Settled airborne state accepted',flush=True)
                valid=bool(active and pose_healthy and latest_result and
                    now-result_wall<=cfg.worker_wall_age and
                    0<=clock[0]-latest_result.get('source_t',-100)<=cfg.frame_age and
                    ((healthy and 0<=clock[0]-latest_result.get('camera_t',-100)<=cfg.frame_age
                      and 0<=clock[0]-latest_result.get('frame_t',-100)<=cfg.mapping_motion_grace)
                     or latest_result.get('state')=='ESCAPE'))
                command=[0.,0.,0.]
                if valid:
                    state=latest_result['state']; reason=latest_result['reason']; missing_since=None
                    if state=='COMPLETE':
                        hold_pos=position; exit_code=0
                    if state in ('COMPLETE','BLOCKED','ABORTED'): break
                    command=[latest_result['vn'],latest_result['ve'],latest_result['vd']]
                else:
                    if active:
                        state='HOLD'; reason='Sensor/clock/worker validity expired'
                        if missing_since is None: missing_since=now
                        if now-missing_since>10: raise RuntimeError(reason+' for 10 wall seconds')
                if 'push_north' in injected: command=[injected['push_north'],0.,0.]
                if 'actuator_hold' in injected: command=[0.,0.,0.]
                if args.fly and active and guided and armed and now-last_command>=.02:
                    # Keep one GUIDED control submode during supervision. Repeated
                    # position/velocity switching destabilized the Iris test model.
                    velocity(master,*command,cfg.heading)
                    last_command=now
                if active and now-last_supervision>=.05:
                    supervision.write(json.dumps({'t':clock[0],'wall':now,'state':state,'valid':valid,'command':command,
                        'control':'velocity',
                        'faults':injected,'position':[position.x,position.y] if position else None,
                        'velocity':[position.vx,position.vy,position.vz] if position else None})+'\n')
                    supervision.flush(); last_supervision=now
                if latest_result and latest_result!=last_logged:
                    record=dict(latest_result,wall=now,**clock_diagnostics(sync),applied=valid and args.fly,
                        supervisor_state=state,healthy=healthy,
                        pose_age=clock[0]-pose.t if clock and pose else None,
                        decision_age=clock[0]-latest_result.get('source_t',0) if clock else None,
                        frame_age=clock[0]-latest_result.get('frame_t',0) if clock else None,
                        worker_age=now-result_wall)
                    log.write(json.dumps(record,allow_nan=False)+'\n'); log.flush(); last_logged=latest_result.copy()
                if now-last_print>=2:
                    print(f'[{state}] {reason}; clock_error={sync.error:.3f} pending={latest_result.get("pending") if latest_result else None}',flush=True)
                    last_print=now
                time.sleep(.005)
            else: reason='Wall-time execution limit'; state='ABORTED'
    except (Exception,KeyboardInterrupt) as exc:
        state='ABORTED'; reason=str(exc) or 'Keyboard interrupt'
    finally:
        if args.fly and hb.custom_mode==4 and hb.base_mode&128:
            # Latch one hold position only with fresh localization. No automatic LAND.
            if position and time.monotonic()-pos_wall<cfg.sensor_wall_age:
                hold_pos=hold_pos or position
                for _ in range(10): hold(master,hold_pos,cfg.heading); time.sleep(.05)
            else:
                velocity(master,0,0,0,cfg.heading)
                print('[FAULT] Localization stale: hover not assured; autopilot/operator contingency required',flush=True)
        put_latest(jobs,None); proc.join(timeout=2)
        if proc.is_alive(): proc.terminate(); proc.join(timeout=2)
        if owns_master:
            master.close()
        print(f'[{state}] {reason}',flush=True)
        args.log.with_suffix('.result.json').write_text(json.dumps({'state':state,'reason':reason,
            'exit_code':exit_code,'last_decision':latest_result},indent=2,allow_nan=False)+'\n')
    return exit_code


if __name__=='__main__': raise SystemExit(main())
