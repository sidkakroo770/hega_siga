"""Bounded QR worker and pure inspection state machine. Limits are provisional."""
from dataclasses import dataclass, asdict
import hashlib
import math
import time
import multiprocessing as mp
import queue
import numpy as np
from .geometry import Projector
from .engine import Decision
from .planning import route


@dataclass(frozen=True)
class QRConfig:
    center_tolerance: float = .15
    center_speed: float = .25
    settled_speed: float = .10
    settle_seconds: float = 1.
    confirmation_frames: int = 3
    detect_period: float = .20
    decode_period: float = .50
    observation_age: float = .50
    result_wall_age: float = 1.
    startup_wall_seconds: float = 5.
    attempt_seconds: float = 8.
    inspect_seconds: float = 30.
    attempts: int = 2
    initial_seconds: float = 45.
    final_altitude: float = 5.
    altitude_tolerance: float = .15
    final_dwell: float = 2.
    descent_speed: float = .30
    descent_seconds: float = 45.
    association_radius: float = .45
    max_candidates: int = 128
    candidate_frames: int = 3
    candidate_stability: float = .20
    lost_seconds: float = 1.
    retry_seconds: float = 10.
    search_resume_seconds: float = 3.
    center_progress_seconds: float = 5.
    center_progress_distance: float = .05
    quad_frame_margin_px: float = 3.
    quad_min_side_px: float = 8.
    quad_max_side_ratio: float = 1.8
    quad_min_angle_deg: float = 45.

    def __post_init__(self):
        if any(not math.isfinite(v) or v <= 0 for v in asdict(self).values()):
            raise ValueError('QR limits must be finite and positive')
        if any(not isinstance(v,int) for v in (self.confirmation_frames,self.attempts,self.max_candidates,self.candidate_frames)):
            raise ValueError('QR counts must be integers')
        if self.confirmation_frames < 2: raise ValueError('QR confirmation needs distinct exposures')
        if self.candidate_frames<2: raise ValueError('QR discovery needs distinct exposures')
        if self.candidate_stability>self.association_radius:
            raise ValueError('Candidate stability exceeds association radius')
        if not 0<self.quad_min_angle_deg<90 or self.quad_max_side_ratio<=1:
            raise ValueError('Invalid QR shape limits')

    @classmethod
    def load(cls, path):
        import json
        return cls(**json.loads(path.read_text())) if path else cls()


def identity_hash(payload):
    return hashlib.sha256(payload.encode('utf-8')).hexdigest()


def decoder_self_check():
    import cv2
    from pathlib import Path
    from .qr_detector import QRDetector
    path=Path(__file__).parents[1]/'world/models/models/qr_markers/materials/textures/qr_start.png'
    image=cv2.imread(str(path))
    if image is None: raise RuntimeError('Missing QR self-check fixture')
    found=QRDetector().candidates(image)
    if len(found)!=1 or QRDetector.decode_selected(image,found[0]['quad'])[0]!='REF-001':
        raise RuntimeError('QR decoder capability self-check failed')


def qr_worker(jobs, results,ready=None):
    import cv2
    from .qr_detector import QRDetector
    cv2.setNumThreads(1)
    detector=QRDetector()
    if ready is not None: ready.set()
    while True:
        job=jobs.get()
        if job is None: return
        started=time.monotonic()
        seq,image,pose,request=job
        try:
            detector.limits=request.get('limits')
            observations=detector.candidates(image)
            projector=Projector(request['config'])
            for candidate in observations:
                candidate['xy']=projector.project([candidate['center']],pose)[0].tolist()
                candidate['payload']=None; candidate['decode_status']='not authorized'
            if request['decode'] and observations:
                selected=min(observations,key=lambda c:np.linalg.norm(np.array(c['xy'])-request['target']))
                if (selected['complete'] and np.linalg.norm(np.array(selected['xy'])-request['target'])
                        <=request['association_radius']):
                    selected['payload'],selected['decode_status']=detector.decode_selected(image,selected['quad'])
            result={'seq':seq,'t':pose.t,'pose':asdict(pose),'generation':request['generation'],
                    'decode':request['decode'],'observations':observations,
                    'processing_ms':1000*(time.monotonic()-started)}
        except Exception as exc:
            result={'seq':seq,'t':pose.t,'generation':request['generation'],'error':str(exc)}
        try: results.put_nowait(result)
        except queue.Full: pass


class QRService:
    """One native worker, one in-flight image, no accumulation or command I/O."""
    def __init__(self,cfg,limits):
        self.cfg=cfg; self.limits=limits; self.ctx=mp.get_context('spawn')
        self.last_seq=-1; self.last_source=-math.inf; self.last_decode_source=-math.inf; self.last_wall=0.
        self.inflight=None; self.restarts=0; self.last_generation=None
        self.metrics={'submitted':0,'decode_jobs':0,'results':0,'restarts':0,
                      'processing_ms_peak':0.,'result_wall_ms_peak':0.}
        self.start()

    def start(self):
        self.jobs=self.ctx.Queue(1); self.results=self.ctx.Queue(1)
        self.ready=self.ctx.Event(); self.start_wall=time.monotonic()
        self.proc=self.ctx.Process(target=qr_worker,args=(self.jobs,self.results,self.ready),daemon=True,name='qr-perception')
        self.proc.start()

    def submit(self,image,pose,decision):
        now=time.monotonic(); generation=decision.get('qr_generation',0)
        if generation!=self.last_generation:
            self.restarts=0; self.last_generation=generation
        if self.inflight is not None or not self.ready.is_set(): return
        decode=bool(decision.get('qr_read_since') is not None and
            pose.t>=decision['qr_read_since'] and
            math.hypot(pose.vn,pose.ve)<=self.limits.settled_speed and
            abs(pose.vd)<=self.limits.settled_speed and
            pose.t-self.last_decode_source>=self.limits.decode_period)
        period=self.limits.detect_period
        if image[3]==self.last_seq or pose.t-self.last_source<period or now-self.last_wall<.05: return
        request={'config':self.cfg,'limits':self.limits,'generation':generation,'decode':decode,
                 'target':decision.get('qr_target'), 'association_radius':self.limits.association_radius}
        self.jobs.put_nowait((image[3],image[2],pose,request))
        self.last_seq=image[3]; self.last_source=pose.t; self.last_wall=now; self.inflight=now
        if decode: self.last_decode_source=pose.t
        self.metrics['submitted']+=1; self.metrics['decode_jobs']+=int(decode)

    def poll(self):
        result=None
        try:
            result=self.results.get_nowait()
            self.metrics['results']+=1
            self.metrics['processing_ms_peak']=max(self.metrics['processing_ms_peak'],result.get('processing_ms',0.))
            self.metrics['result_wall_ms_peak']=max(self.metrics['result_wall_ms_peak'],
                1000*(time.monotonic()-self.inflight) if self.inflight is not None else 0.)
            self.inflight=None
        except queue.Empty: pass
        if result: return result
        # Cold Python/OpenCV imports are setup, not admitted-frame processing.
        # Do not queue an image until ready, or grant old exposures a long grace.
        startup_failed=not self.ready.is_set() and time.monotonic()-self.start_wall>self.limits.startup_wall_seconds
        if not self.proc.is_alive() or startup_failed or (self.inflight is not None and
                                       time.monotonic()-self.inflight>self.limits.result_wall_age):
            if self.restarts>=1: raise RuntimeError('QR worker failed after bounded restart')
            self.close(); self.restarts+=1; self.metrics['restarts']+=1; self.start(); self.inflight=None
        return None

    def close(self):
        try: self.jobs.put_nowait(None)
        except queue.Full: pass
        self.proc.join(.2)
        if self.proc.is_alive(): self.proc.terminate(); self.proc.join(.5)
        if self.proc.is_alive(): self.proc.kill(); self.proc.join(.5)
        for q in (self.jobs,self.results): q.close(); q.cancel_join_thread()


class Inspection:
    """Spatial identity, finite reads, immutable reference and terminal target latch."""
    def __init__(self,cfg,limits=None,reference=None,initial=False):
        self.cfg=cfg; self.q=limits or QRConfig(); self.reference=reference; self.initial=initial
        self.started=None; self.origin=None; self.initial_target=None
        self.candidates=[]; self.selected=None; self.generation=0
        self.state='QR_ADVANCE' if initial else 'SEARCH'
        self.settle_since=None; self.read_since=None; self.read_deadline=None
        self.inspect_deadline=None; self.reads=[]; self.events=[]; self.last_result=None
        self.match=None; self.descent_started=None; self.final_since=None
        self.failure=None; self.path=[]; self.last_route=-math.inf
        self.resume_after=-math.inf
        self.center_best=math.inf; self.center_progress_since=None

    def event(self,t,state,**details):
        self.events.append(dict(t=t,state=state,**details)); self.events=self.events[-256:]

    def interrupt(self):
        self.settle_since=None; self.read_since=None; self.reads=[]
        self.final_since=None  # Delivery dwell requires continuous admitted evidence.

    def ingest(self,result,pose):
        if not result or self.last_result==(result['seq'],result['t']): return
        self.last_result=(result['seq'],result['t'])
        if result.get('error'): self.failure='QR perception: '+result['error']; return
        if not 0<=pose.t-result['t']<=self.q.observation_age: return
        if result['generation']!=self.generation: return
        seen=set()
        for obs in result['observations']:
            if not obs['complete']: continue
            xy=np.asarray(obs['xy'],float)
            if xy.shape!=(2,) or not np.isfinite(xy).all(): continue
            nearby=[c for c in self.candidates if np.linalg.norm(c['xy']-xy)<=self.q.association_radius]
            if len(nearby)>1: continue
            c=nearby[0] if nearby else None
            if c is None:
                if len(self.candidates)>=self.q.max_candidates:
                    self.failure='QR candidate ledger bound exceeded'; return
                c=dict(id=len(self.candidates),xy=xy,status='NEW',attempts=0,t=-math.inf,complete=False,
                       sightings=0,stable_origin=xy.copy(),seq=None,next_try=-math.inf)
                self.candidates.append(c)
            if c['seq']==result['seq']: continue
            seen.add(c['id'])
            stable=(0<=result['t']-c['t']<=self.q.observation_age and
                    np.linalg.norm(xy-c['stable_origin'])<=self.q.candidate_stability)
            if not stable: c['sightings']=0; c['stable_origin']=xy.copy()
            c['sightings']+=1; c['seq']=result['seq']
            if c is not self.selected: c['xy']=xy
            c.update(t=result['t'],complete=obs['complete'])
            if c is not self.selected or not result['decode'] or self.read_since is None: continue
            if (not obs['complete'] or result['t']<self.read_since or self.state!='QR_READ' or
                    (self.read_deadline is not None and pose.t>self.read_deadline) or
                    (self.inspect_deadline is not None and pose.t>self.inspect_deadline) or
                    np.linalg.norm(pose.xy-c['xy'])>self.q.center_tolerance or
                    math.hypot(pose.vn,pose.ve)>self.q.settled_speed or abs(pose.vd)>self.q.settled_speed or
                    math.hypot(result['pose']['vn'],result['pose']['ve'])>self.q.settled_speed or
                    abs(result['pose']['vd'])>self.q.settled_speed or
                    np.linalg.norm(np.array([result['pose']['n'],result['pose']['e']])-c['xy'])>self.q.center_tolerance):
                continue
            text=obs['payload']
            c['decode_status']=obs.get('decode_status','unreadable' if text is None else 'text')
            if text is None: continue
            if self.reads and self.reads[-1]['payload']!=text: self.reads=[]
            if not any(r['seq']==result['seq'] for r in self.reads):
                self.reads.append(dict(seq=result['seq'],t=result['t'],payload=text))
            if len(self.reads)>=self.q.confirmation_frames:
                if self.initial:
                    self.reference=text; self.state='REFERENCE_READY'
                    self.event(pose.t,self.state,identity_hash=identity_hash(text),reads=list(self.reads))
                elif text==self.reference:
                    c['status']='MATCH'; self.match=c['xy'].copy(); self.state='TARGET_DESCEND'
                    self.descent_started=pose.t; self.generation+=1
                    self.event(pose.t,'TARGET_MATCH',identity_hash=identity_hash(text),xy=self.match.tolist(),reads=list(self.reads))
                else:
                    c['status']='NONMATCH'; self.event(pose.t,'NONMATCH',id=c['id'],reads=list(self.reads)); self.release()
        for c in self.candidates:
            if c['id'] not in seen and c['status'] in ('NEW','DEFERRED'):
                c['sightings']=0

    def release(self):
        self.selected=None; self.state='SEARCH'; self.generation+=1
        self.interrupt(); self.read_deadline=None; self.inspect_deadline=None; self.path=[]
        self.center_best=math.inf; self.center_progress_since=None

    def failed_attempt(self,pose,reason):
        c=self.selected
        c['status']='UNREADABLE' if c['attempts']>=self.q.attempts else 'NEW'
        c['next_try']=pose.t+self.q.retry_seconds
        c['sightings']=0
        self.event(pose.t,reason,id=c['id'],attempt=c['attempts'])
        if self.initial and c['status']=='UNREADABLE': self.failure='Initial QR unreadable after finite attempts'
        self.resume_after=pose.t+self.q.search_resume_seconds
        self.release()

    def metadata(self):
        return dict(qr_generation=self.generation,qr_read_since=self.read_since if self.state=='QR_READ' else None,
                    qr_target=self.selected['xy'].tolist() if self.selected is not None else None,
                    qr_reference=self.reference,qr_reference_hash=identity_hash(self.reference) if self.reference else None,
                    qr_events=list(self.events),qr_candidates=[dict(id=c['id'],xy=c['xy'].tolist(),
                        status=c['status'],attempts=c['attempts'],sightings=c.get('sightings',0),
                        decode_status=c.get('decode_status')) for c in self.candidates],
                    qr_match=self.match.tolist() if self.match is not None else None)

    def search_outcome(self):
        # An excluded marker's identity is still unknown, not a proven nonmatch.
        return ('TARGET_UNRESOLVED' if any(c['status']!='NONMATCH' for c in self.candidates)
                else 'TARGET_NOT_FOUND')

    def step(self,pose,result=None,ground=None):
        q=self.q
        if self.started is None:
            self.started=pose.t; self.origin=pose.xy.copy()
            self.initial_target=self.origin+np.array([math.cos(pose.yaw),math.sin(pose.yaw)])
        if ground is not None:
            cell=ground.cell(pose.xy)
            if not ground.contains(cell) or not ground.free[cell]:
                self.interrupt()
                return None  # Engine escape/fence safety retains priority.
            if self.selected is not None:
                cell=ground.cell(self.selected['xy'])
                if not ground.contains(cell) or not ground.inset[cell] or ground.inflated[cell]:
                    if self.match is not None:
                        self.failure='Matched target newly unsafe'
                    else:
                        self.selected['status']='EXCLUDED'; self.release()
        self.ingest(result,pose)
        d=Decision('HOLD','QR inspection',yaw=self.cfg.heading,source_t=pose.t)
        if self.failure or (self.initial and pose.t-self.started>q.initial_seconds):
            d.state='ABORTED'; d.reason=self.failure or 'Initial reference deadline exceeded'; return d
        if self.state=='REFERENCE_READY':
            d.state=self.state; d.reason='Immutable initial QR confirmed'; return d
        if self.state=='QR_ADVANCE':
            delta=self.initial_target-pose.xy
            if np.linalg.norm(delta)>.10:
                v=delta*min(.8,q.center_speed/max(np.linalg.norm(delta),1e-9))
                d.vn,d.ve=map(float,v); d.state='QR_ADVANCE'; d.reason='Measured 1 m startup advance'
                d.vd=float(np.clip((pose.alt-q.final_altitude)*.6,-.3,.3)); return d
            self.state='SEARCH'; self.event(pose.t,'INITIAL_ADVANCE_COMPLETE',distance=float(np.linalg.norm(pose.xy-self.origin)))
        if self.match is not None:
            if pose.t-self.descent_started>q.descent_seconds:
                d.state='ABORTED'; d.reason='Target descent deadline exceeded'; return d
            delta=self.match-pose.xy
            if ground is not None and not ground.line_clear(pose.xy,self.match):
                d.state='ABORTED'; d.reason='Matched target no longer safe for descent'; return d
            v=delta*min(.8,q.center_speed/max(np.linalg.norm(delta),1e-9))
            d.vn,d.ve=map(float,v); d.state='TARGET_DESCEND'; d.reason='Match latched; coverage cancelled'
            centered=np.linalg.norm(delta)<=q.center_tolerance
            d.vd=float(np.clip((pose.alt-q.final_altitude)*.6,-q.descent_speed,q.descent_speed)) if centered else 0.
            settled=centered and abs(pose.alt-q.final_altitude)<=q.altitude_tolerance and math.hypot(pose.vn,pose.ve)<=q.settled_speed and abs(pose.vd)<=q.settled_speed
            if settled:
                if self.final_since is None: self.final_since=pose.t
                if pose.t-self.final_since>=q.final_dwell:
                    d.state='TARGET_HOLD_5M'; d.reason='Matching target; verified stationary 5 m hold'; d.vn=d.ve=d.vd=0.
            else: self.final_since=None
            return d
        if self.selected is None:
            for c in sorted(self.candidates,key=lambda c:np.linalg.norm(c['xy']-pose.xy)):
                if c['status'] in ('NONMATCH','UNREADABLE','EXCLUDED'): continue
                if ground is not None:
                    cell=ground.cell(c['xy'])
                    if not ground.contains(cell) or not ground.inset[cell] or ground.inflated[cell]:
                        c['status']='EXCLUDED'; continue
                    if not ground.free[cell]: c['status']='DEFERRED'; continue
                if (pose.t<self.resume_after or pose.t<c['next_try'] or
                        not 0<=pose.t-c['t']<=q.observation_age or
                        c['sightings']<q.candidate_frames): continue
                self.selected=c; c['attempts']+=1; c['status']='INSPECTING'
                self.generation+=1; self.inspect_deadline=pose.t+q.inspect_seconds
                self.interrupt(); self.path=[]; self.state='QR_CENTER'
                self.center_best=math.inf; self.center_progress_since=pose.t
                self.event(pose.t,'QR_SELECTED',id=c['id']); break
        if self.selected is None: return d if self.initial else None
        c=self.selected
        if self.initial and np.linalg.norm(c['xy']-self.initial_target)>1.:
            self.failure='Initial QR outside bounded reference area'; return d
        if pose.t>self.inspect_deadline or (self.read_deadline is not None and pose.t>self.read_deadline):
            self.failed_attempt(pose,'READ_ATTEMPT_EXPIRED'); return d if self.initial else None
        if pose.t-c['t']>q.lost_seconds:
            self.failed_attempt(pose,'QR_LOST'); return d if self.initial else None
        delta=c['xy']-pose.xy; live=0<=pose.t-c['t']<=q.observation_age
        centered=np.linalg.norm(delta)<=q.center_tolerance
        if not centered:
            distance=float(np.linalg.norm(delta))
            # Route length, rather than direct distance, permits a legitimate
            # detour around red to move temporarily away from the QR.
            if self.path:
                distance=float(np.linalg.norm(self.path[0]-pose.xy))+sum(
                    float(np.linalg.norm(b-a)) for a,b in zip(self.path,self.path[1:]))
            if distance<self.center_best-q.center_progress_distance:
                self.center_best=distance; self.center_progress_since=pose.t
            elif pose.t-self.center_progress_since>q.center_progress_seconds:
                self.failed_attempt(pose,'QR_CENTER_NO_PROGRESS'); return d if self.initial else None
        else:
            self.center_best=math.inf; self.center_progress_since=pose.t
        settled=centered and live and c['complete'] and math.hypot(pose.vn,pose.ve)<=q.settled_speed and abs(pose.vd)<=q.settled_speed
        if settled:
            if self.settle_since is None: self.settle_since=pose.t
            if pose.t-self.settle_since>=q.settle_seconds:
                self.state='QR_READ'
                if self.read_since is None:
                    self.read_since=pose.t
                    if self.read_deadline is None: self.read_deadline=pose.t+q.attempt_seconds
            else: self.state='QR_SETTLE'
        else: self.interrupt(); self.state='QR_CENTER'
        d.state=self.state; d.reason='Center, settle, then read selected physical marker'
        if not centered and live:
            target=c['xy']
            if ground is not None and not ground.line_clear(pose.xy,target):
                if pose.t-self.last_route>1:
                    had_path=bool(self.path)
                    self.path=route(ground,pose.xy,target,self.cfg.planning_wall_budget) or []; self.last_route=pose.t
                    if self.path and not had_path:
                        self.center_best=math.inf; self.center_progress_since=pose.t
                while self.path and np.linalg.norm(self.path[0]-pose.xy)<.08: self.path.pop(0)
                if not self.path: return d
                target=self.path[0]
            v=(target-pose.xy)*min(.8,q.center_speed/max(np.linalg.norm(target-pose.xy),1e-9)); d.vn,d.ve=map(float,v)
        d.vd=float(np.clip((pose.alt-(q.final_altitude if self.initial else self.cfg.altitude))*.6,-.3,.3))
        return d
