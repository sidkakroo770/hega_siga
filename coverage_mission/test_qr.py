"""QR identity, authority and state safety tests; no aircraft connection."""
from dataclasses import replace
from pathlib import Path
import math
import queue
from unittest.mock import Mock
import cv2
import numpy as np
import pytest
import qrcode
from coverage_mission.config import Config
from coverage_mission.engine import Engine, Decision
from coverage_mission.geometry import Pose, Projector
from coverage_mission.qr import Inspection, QRConfig, QRService, decoder_self_check
from coverage_mission.qr_detector import QRDetector


def cfg():
    return replace(Config(),n_min=-5,n_max=5,e_min=-5,e_max=5,assume_nonred_ground_clear=True)


def code(text,mask=None,version=None):
    q=qrcode.QRCode(box_size=8,border=4,mask_pattern=mask,version=version)
    q.add_data(text); q.make(fit=True)
    return cv2.cvtColor(np.array(q.make_image().convert('RGB')),cv2.COLOR_RGB2BGR)


def observation(i,t,xy=(0,0),payload=None,generation=0,decode=False,vn=0,complete=True):
    return dict(seq=i,t=t,generation=generation,decode=decode,
        pose=vars(Pose(t,*xy,10,vn=vn)),observations=[dict(xy=list(xy),complete=complete,payload=payload)])


def select(ins,t=0,xy=(0,0),ground=None):
    p=Pose(t,*xy,5 if ins.initial else 10)
    # Initial advance already verified in separate tests.
    if ins.initial: ins.state='SEARCH'
    for i,at in enumerate((t-.4,t-.2,t),1):
        ins.step(Pose(at,*xy,p.alt),observation(i,at,xy,generation=ins.generation),ground)
    return p


def settle(ins,t=0,xy=(0,0),ground=None):
    select(ins,t,xy,ground)
    for i,dt in enumerate((.2,.4,.6,.8,1.,1.2),4):
        ins.step(Pose(t+dt,*xy,5 if ins.initial else 10),
                 observation(i,t+dt,xy,generation=ins.generation),ground)
    assert ins.state=='QR_READ'


def confirm(ins,payload,t=2,xy=(0,0),ground=None):
    for i in range(3):
        at=t+i*.2
        ins.step(Pose(at,*xy,5 if ins.initial else 10),
            observation(30+i,at,xy,payload,ins.generation,True),ground)


def test_installed_decoder_capability():
    decoder_self_check()


@pytest.mark.parametrize('text',['REF-001','https://example.org/task?x=1&y=2',' Text with Spaces '])
def test_exact_text_with_distinct_patterns(text):
    for mask in (0,3):
        frame=code(text,mask=mask)
        found=QRDetector().candidates(frame)
        assert len(found)==1
        assert QRDetector.decode_selected(frame,found[0]['quad'])[0]==text


def test_same_text_in_different_qr_versions_matches():
    for version in (1,3):
        image=code('REF-001',version=version)
        found=QRDetector().candidates(image)
        assert QRDetector.decode_selected(image,found[0]['quad'])[0]=='REF-001'


def test_discovery_does_not_decode(monkeypatch):
    monkeypatch.setattr(QRDetector,'decode_selected',Mock(side_effect=AssertionError('moving decode')))
    result=QRDetector().detect(code('REF-001'),debug=False)
    assert result['detected'] and not result['qr_data'] and result['debug_frame'] is None


def test_initial_advance_uses_measured_position_and_heading():
    ins=Inspection(cfg(),initial=True)
    d=ins.step(Pose(0,2,1,5,yaw=math.pi/2))
    assert abs(d.vn)<1e-8 and d.ve>0
    d=ins.step(Pose(10,2,1,5,yaw=math.pi/2))
    assert d.state=='QR_ADVANCE'  # elapsed time alone cannot declare arrival
    ins.step(Pose(11,2,1.95,5,yaw=math.pi/2))
    assert any(e['state']=='INITIAL_ADVANCE_COMPLETE' for e in ins.events)


def test_initial_no_camera_candidate_deadline_is_bounded():
    ins=Inspection(cfg(),initial=True)
    ins.step(Pose(0,0,0,5))
    assert ins.step(Pose(46,1,0,5)).state=='ABORTED'


def test_reference_requires_three_distinct_eligible_reads_and_is_immutable():
    ins=Inspection(cfg(),initial=True); settle(ins)
    read=observation(100,2,payload='REF-001',generation=ins.generation,decode=True)
    ins.step(Pose(2,0,0,5),read)
    for _ in range(5): ins.step(Pose(2.01,0,0,5),read)
    assert ins.reference is None and len(ins.reads)==1
    confirm(ins,'REF-001',2.1)
    assert ins.reference=='REF-001' and ins.state=='REFERENCE_READY'
    ins.step(Pose(2.7,0,0,5),observation(200,2.7,payload='EVIL',generation=ins.generation,decode=True))
    assert ins.reference=='REF-001'


@pytest.mark.parametrize('change',['old','generation','moving','before_settle','clipped'])
def test_ineligible_read_never_confirms(change):
    ins=Inspection(cfg(),reference='REF-001'); settle(ins)
    r=observation(100,2,payload='REF-001',generation=ins.generation,decode=True)
    if change=='old': r['t']=0
    elif change=='generation': r['generation']-=1
    elif change=='moving': r['pose']['vn']=.5
    elif change=='before_settle': r['t']=ins.read_since-.1
    elif change=='clipped': r['observations'][0]['complete']=False
    ins.step(Pose(2,0,0,10),r)
    assert ins.match is None
    if change!='clipped': assert not ins.reads


def test_nonmatch_is_spatially_remembered_and_resumes_search():
    ins=Inspection(cfg(),reference='REF-001'); settle(ins); confirm(ins,'OTHER')
    assert ins.match is None and ins.selected is None
    assert ins.candidates[0]['status']=='NONMATCH'
    assert ins.step(Pose(4,0,0,10),observation(100,4,generation=ins.generation)) is None


def test_unreadable_has_finite_attempts():
    ins=Inspection(cfg(),reference='REF-001'); select(ins)
    ins.step(Pose(31,0,0,10))
    select(ins,t=42)
    ins.step(Pose(73,0,0,10))
    assert ins.candidates[0]['attempts']==2 and ins.candidates[0]['status']=='UNREADABLE'
    assert ins.selected is None


def test_multiple_markers_cannot_switch_active_identity():
    ins=Inspection(cfg(),reference='REF-001'); settle(ins)
    r=observation(99,2,xy=(1,0),payload='REF-001',generation=ins.generation,decode=True)
    ins.step(Pose(2,0,0,10),r)
    assert len(ins.candidates)==2 and ins.selected['id']==0 and not ins.reads


def test_new_red_excludes_active_candidate_without_reading():
    eng=Engine(cfg()); g=eng.ground; g.observed[:]=True; g.refresh()
    ins=Inspection(cfg(),reference='REF-001'); select(ins,xy=(1,0),ground=g)
    g.red[g.cell((1,0))]=True; g.refresh()
    ins.step(Pose(1,0,0,10),ground=g)
    assert ins.selected is None and ins.candidates[0]['status']=='EXCLUDED'


def test_match_cancel_and_descent_not_blocked_by_coverage_altitude():
    conf=cfg(); eng=Engine(conf); g=eng.ground; g.observed[:]=True; g.last_t=1; g.refresh()
    ins=Inspection(conf,reference='REF-001'); settle(ins,ground=g); confirm(ins,'REF-001',ground=g)
    assert ins.match is not None
    assert ins.step(Pose(3,0,0,8),ground=g).vd>0
    g.last_t=3
    d=eng.step(Pose(3,0,0,8),3,ins.step(Pose(3,0,0,8),ground=g))
    assert d.state=='TARGET_DESCEND' and d.vd>0 and not eng.path
    ins.step(Pose(20,0,0,5),ground=g)
    assert ins.step(Pose(22.1,0,0,5),ground=g).state=='TARGET_HOLD_5M'


def test_descent_drift_and_vertical_motion_prevent_success():
    ins=Inspection(cfg(),reference='REF-001'); settle(ins); confirm(ins,'REF-001')
    assert ins.step(Pose(4,.5,0,8)).vd==0
    ins.step(Pose(20,0,0,5,vd=.2))
    assert ins.step(Pose(22,0,0,5,vd=.2)).state=='TARGET_DESCEND'


def test_inspection_does_not_credit_coverage_or_stall_progress():
    eng=Engine(cfg()); g=eng.ground; g.observed[:]=True; g.refresh()
    for t in (1,2,3):
        g.last_t=t
        eng.step(Pose(t,0,0,10),t,Decision('QR_SETTLE','test'))
    assert not eng.plan.done.any() and eng.inspection_pause==1
    g.last_t=4; eng.step(Pose(4,0,0,10),4)
    assert eng.inspection_pause is None


def test_red_residence_and_escape_override_inspection():
    eng=Engine(cfg()); g=eng.ground; g.observed[:]=True
    g.red[g.cell((0,0))]=True; g.last_t=1; g.refresh()
    d=eng.step(Pose(1,0,0,10),1,Decision('QR_READ','must not read'))
    assert d.state!='QR_READ' and d.entered==1


@pytest.mark.parametrize('yaw',[0,math.pi/2,-.7])
def test_projection_is_vehicle_heading_aware(yaw):
    proj=Projector(cfg()); p=Pose(0,2,3,10,yaw=yaw)
    center=proj.project([[320,240]],p)[0]
    assert np.linalg.norm(center-p.xy)<1e-8
    right=proj.project([[420,240]],p)[0]-p.xy
    assert np.dot(right,[-math.sin(yaw),math.cos(yaw)])>0


def test_invalid_configuration_rejected():
    with pytest.raises(ValueError): QRConfig(attempts=0)
    with pytest.raises(ValueError): QRConfig(confirmation_frames=1)


def test_current_drift_rejects_delayed_stationary_read():
    ins=Inspection(cfg(),reference='REF-001'); settle(ins)
    read=observation(100,2,payload='REF-001',generation=ins.generation,decode=True)
    ins.step(Pose(2.05,.3,0,10),read)
    assert not ins.reads and ins.match is None


def test_late_confirmation_cannot_bypass_attempt_deadline():
    ins=Inspection(cfg(),reference='REF-001'); settle(ins)
    for i in range(2):
        ins.step(Pose(2+i*.2,0,0,10),observation(40+i,2+i*.2,payload='REF-001',
                                               generation=ins.generation,decode=True))
    deadline=ins.read_deadline
    ins.step(Pose(deadline+.01,0,0,10),observation(100,deadline-.01,
                    payload='REF-001',generation=ins.generation,decode=True))
    assert ins.match is None


def test_native_worker_is_bounded_and_decodes_only_when_authorized():
    import time
    conf=cfg(); service=QRService(conf,QRConfig())
    image=np.full((480,640,3),255,np.uint8)
    stamp=code('REF-001')
    image[120:120+stamp.shape[0],200:200+stamp.shape[1]]=stamp
    try:
        assert service.ready.wait(5)
        p=Pose(1,0,0,10,vn=.5)
        service.submit((1,0,image,1),p,{'qr_read_since':0.,'qr_target':[0,0]})
        def receive():
            deadline=time.monotonic()+5
            while time.monotonic()<deadline:
                # Initial Python import startup is not decoder job latency.
                try: result=service.results.get_nowait()
                except queue.Empty: time.sleep(.01); continue
                service.inflight=None
                return result
            raise AssertionError('native QR process did not return')
        r=receive(); assert not r['decode'] and r['observations'][0]['payload'] is None
        target=r['observations'][0]['xy']
        time.sleep(.06)  # The real service deliberately caps admission by wall time too.
        service.submit((2,0,image,2),Pose(2,0,0,10),{'qr_read_since':1.,'qr_target':target})
        r=receive(); assert r['decode'] and r['observations'][0]['payload']=='REF-001'
        assert service.jobs._maxsize==1
    finally: service.close()


def test_worker_fault_restart_is_bounded():
    service=QRService.__new__(QRService)
    service.results=Mock(); service.results.get_nowait.side_effect=queue.Empty
    service.proc=Mock(); service.proc.is_alive.return_value=False
    service.inflight=None; service.restarts=0; service.limits=QRConfig()
    service.ready=Mock(); service.ready.is_set.return_value=True
    service.start_wall=0.
    service.metrics={'restarts':0}; service.close=Mock(); service.start=Mock()
    assert service.poll() is None and service.restarts==1
    with pytest.raises(RuntimeError,match='bounded restart'): service.poll()


def test_incursion_during_descent_can_escape_below_sweep_altitude():
    eng=Engine(cfg()); g=eng.ground; g.observed[:]=True
    g.red[g.cell((0,0))]=True; g.last_t=1; g.refresh()
    d=eng.step(Pose(1,0,0,7),1,descent_active=True)
    assert d.reason!='Altitude outside coverage envelope'
    assert d.entered==1


def test_unsupported_binary_and_ambiguous_crop_rejected(monkeypatch):
    from types import SimpleNamespace
    import pyzbar.pyzbar as backend
    rect=SimpleNamespace(left=20,top=20,width=80,height=80)
    # Selected crop origin is (0,0), making the decoded centre (60,60).
    quad=np.array([[20,20],[100,20],[100,100],[20,100]],np.float32)
    frame=np.zeros((150,150,3),np.uint8)
    entry=SimpleNamespace(rect=rect,data=b'\xff\xfe')
    monkeypatch.setattr(backend,'decode',lambda *a,**k:[entry])
    assert QRDetector.decode_selected(frame,quad)[1]=='unsupported binary payload'
    monkeypatch.setattr(backend,'decode',lambda *a,**k:[entry,entry])
    assert QRDetector.decode_selected(frame,quad)[1]=='ambiguous'


def test_no_empty_decode_and_conflicting_reads_cannot_match():
    ins=Inspection(cfg(),reference='REF-001'); settle(ins)
    for i,text in enumerate(('REF-001','OTHER','REF-001','OTHER')):
        ins.step(Pose(2+i*.2,0,0,10),observation(99+i,2+i*.2,
            payload=text,generation=ins.generation,decode=True))
    assert ins.match is None and len(ins.reads)==1


def test_new_red_during_descent_aborts_without_descending():
    conf=cfg(); eng=Engine(conf); g=eng.ground; g.observed[:]=True; g.refresh()
    ins=Inspection(conf,reference='REF-001'); settle(ins,ground=g); confirm(ins,'REF-001',ground=g)
    g.red[g.cell((0,0))]=True; g.last_t=3; g.refresh()
    assert ins.step(Pose(3,1,0,8),ground=g).state=='ABORTED'


def test_delayed_coverage_pause_resume_preserves_obligations():
    eng=Engine(cfg()); g=eng.ground; g.observed[:]=True; g.refresh()
    g.last_t=1; eng.step(Pose(1,0,0,10),1)
    progress=eng.last_progress_t; done=eng.plan.done.copy()
    for t in (2,10,40,70):
        g.last_t=t; eng.step(Pose(t,0,0,10),t,Decision('QR_READ','inspection'))
    assert np.array_equal(eng.plan.done,done)
    g.last_t=71; result=eng.step(Pose(71,0,0,10),71)
    assert result.state!='BLOCKED' and eng.last_progress_t>=progress+69


def test_suspended_native_process_restarts_once_without_leaking():
    import os
    import signal
    import time
    service=QRService(cfg(),QRConfig(result_wall_age=.05))
    first=service.proc
    try:
        assert service.ready.wait(5)
        os.kill(first.pid,signal.SIGSTOP)
        service.inflight=time.monotonic()-.1
        service.poll()
        assert not first.is_alive() and service.restarts==1 and service.proc.pid!=first.pid
        assert service.ready.wait(5)
        os.kill(service.proc.pid,signal.SIGSTOP)
        service.inflight=time.monotonic()-.1
        with pytest.raises(RuntimeError,match='bounded restart'): service.poll()
    finally: service.close()


def test_search_exhaustion_is_not_target_success():
    ins=Inspection(cfg(),reference='REF-001')
    assert ins.search_outcome()=='TARGET_NOT_FOUND'
    ins.candidates=[{'status':'NONMATCH'}]
    assert ins.search_outcome()=='TARGET_NOT_FOUND'
    for status in ('EXCLUDED','UNREADABLE','DEFERRED'):
        ins.candidates=[{'status':status}]
        assert ins.search_outcome()=='TARGET_UNRESOLVED'


def test_no_image_queued_during_cold_start_and_startup_is_bounded():
    import time
    service=QRService.__new__(QRService)
    service.ready=Mock(); service.ready.is_set.return_value=False
    service.last_generation=None; service.restarts=0; service.inflight=None
    service.metrics={'submitted':0,'restarts':0}; service.limits=QRConfig()
    service.submit((1,0,None,1),Pose(1,0,0,10),{})
    assert service.metrics['submitted']==0
    service.proc=Mock(); service.proc.is_alive.return_value=True
    service.results=Mock(); service.results.get_nowait.side_effect=queue.Empty
    service.start_wall=time.monotonic()-6
    service.close=Mock(); service.start=Mock()
    service.poll()
    assert service.restarts==1


def test_truth_evaluator_handles_60hz_samples_without_losing_velocity_evidence():
    from coverage_mission.evaluate_qr_run import truth_velocities
    records=[dict(t=i/60,n=.03*i/60,e=0,z=5-.05*i/60) for i in range(120)]
    speeds=truth_velocities(records)
    assert speeds and max(max(s) for s in speeds)<.10
    assert abs(speeds[0][0]-.03)<1e-8 and abs(speeds[0][1]-.05)<1e-8
