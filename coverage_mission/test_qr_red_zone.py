"""Focused arbitration regressions: QR never outranks live red-zone safety."""
from dataclasses import replace
import numpy as np
import pytest
from coverage_mission.config import Config
from coverage_mission.engine import Engine, Decision
from coverage_mission.geometry import Pose
from coverage_mission.qr import Inspection
from coverage_mission.test_qr import observation, settle, confirm


def setup_scene(distance):
    cfg=replace(Config(),n_min=-5,n_max=5,e_min=-5,e_max=5,
                assume_nonred_ground_clear=True)
    engine=Engine(cfg)
    ground=engine.ground
    ground.observed[:]=True
    for index in np.ndindex(ground.shape):
        if ground.point(index)[0]>=distance:
            ground.red[index]=True
    ground.last_t=0.
    ground.refresh()
    return cfg,engine,Inspection(cfg,reference='REF-001')


def test_safe_nearby_target_centers_reads_and_descends_with_safety_active():
    cfg,engine,ins=setup_scene(1.2)
    ground=engine.ground
    settle(ins,ground=ground)
    confirm(ins,'REF-001',ground=ground)
    assert ins.match is not None
    # Small steps retain normal pose-continuity checking throughout descent.
    decisions=[]
    for i in range(181):
        t=3+i*.1
        alt=max(5.,10-i*.03)
        pose=Pose(t,0,0,alt,vd=.3 if alt>5 else 0.)
        ground.last_t=t
        decisions.append(engine.step(pose,t,ins.step(pose,ground=ground),descent_active=True))
    assert any(d.state=='TARGET_DESCEND' and d.vd>0 for d in decisions)
    assert not any(d.state in ('ESCAPE','ABORTED') for d in decisions)
    assert not engine.plan.done.any()  # QR manoeuvres are not coverage credit.


def test_target_inside_clearance_is_excluded_without_centering_or_matching():
    _,engine,ins=setup_scene(1.2)
    # Vehicle is safe, but target at N=.7 is only .5 m from red.
    pose=Pose(0,0,0,10)
    proposal=ins.step(pose,observation(1,0,xy=(.7,0)),engine.ground)
    assert proposal is None
    assert ins.candidates[0]['status']=='EXCLUDED'
    assert ins.selected is None and ins.match is None


@pytest.mark.parametrize('phase',['reading','final_read','matched'])
def test_new_red_over_target_cancels_reads_or_matched_descent(phase):
    _,engine,ins=setup_scene(2.)
    ground=engine.ground
    settle(ins,xy=(1,0),ground=ground)
    if phase=='matched': confirm(ins,'REF-001',xy=(1,0),ground=ground)
    if phase=='final_read':
        for i in range(2):
            t=2+i*.2
            ins.step(Pose(t,1,0,10),observation(30+i,t,xy=(1,0),
                payload='REF-001',generation=ins.generation,decode=True),ground)
        assert len(ins.reads)==2
    # New red appears on the selected target; vehicle remains outside inflation.
    ground.red[ground.cell((1,0))]=True
    ground.refresh(); ground.last_t=3
    pose=Pose(3,1 if phase=='final_read' else 0,0,8 if phase=='matched' else 10)
    result=(observation(32,3,xy=(1,0),payload='REF-001',
                        generation=ins.generation,decode=True) if phase=='final_read' else None)
    proposal=ins.step(pose,result,ground=ground)
    decision=engine.step(pose,3,proposal,descent_active=ins.match is not None)
    assert decision.vd<=0 and decision.state!='TARGET_DESCEND'
    if phase=='final_read':
        # This third read is otherwise eligible: the vehicle is centred and
        # stationary. New red at its feet must win even over confirmation.
        assert not ins.reads and ins.match is None
        assert decision.state not in ('QR_READ','TARGET_DESCEND')
        assert decision.entered is not None
    elif phase=='reading':
        assert ins.selected is None and not ins.reads
        assert ins.candidates[0]['status']=='EXCLUDED'
        assert ins.match is None
    else: assert decision.state=='ABORTED'


def test_escape_overrides_a_latched_descent_proposal():
    _,engine,ins=setup_scene(1.2)
    ground=engine.ground
    ground.red[ground.cell((0,0))]=True; ground.refresh(); ground.last_t=3
    pose=Pose(3,0,0,8)
    decision=engine.step(pose,3,Decision('TARGET_DESCEND','stale descent',vd=.3),descent_active=True)
    assert decision.state!='TARGET_DESCEND'
    assert decision.vd<=0 and decision.entered is not None


def test_stale_camera_cannot_continue_target_descent():
    _,engine,_=setup_scene(1.2)
    engine.ground.last_t=0
    decision=engine.step(Pose(3,0,0,8),0,
                         Decision('TARGET_DESCEND','otherwise safe',vd=.3),descent_active=True)
    assert decision.state=='HOLD' and decision.vd==0
