"""Return state, region and map contracts, without a flight connection."""
from dataclasses import replace
from pathlib import Path
import math
import numpy as np
import pytest

from coverage_mission.config import Config
from coverage_mission.engine import Engine
from coverage_mission.geometry import Pose
from coverage_mission.planning import GroundMap
from coverage_mission.return_mission import ReturnConfig, ReturnNavigator, EntrancePermit
from coverage_mission.qr import Inspection, QRConfig


def cfg():
    return Config.load(Path(__file__).parents[1]/'config/full_mission_coverage.json')


def ground():
    g=GroundMap(cfg()); g.observed[:]=True; g.refresh(); g.last_t=10.
    return g


def test_profile_validates_and_is_heading_relative():
    with pytest.raises(ValueError): ReturnConfig(speed=0)
    with pytest.raises(ValueError): ReturnConfig(heading=math.nan)
    with pytest.raises(ValueError): ReturnConfig(orange_hsv_low=(30,100,80))
    p=ReturnConfig()
    assert np.allclose(p.stand_off_point,[-14,4.1])
    assert p.approach_contains([-19.6,4.1],2.7)
    assert not p.approach_contains([-19.6,7.],2.7)
    assert not p.approach_contains([-21.,4.1],2.7)
    rotated=replace(p,entrance_n=0,entrance_e=0,heading=math.pi/2)
    assert np.allclose(rotated.stand_off_point,[0,-6])


def test_delivery_dwell_is_five_continuous_seconds():
    ins=Inspection(cfg(),replace(QRConfig(),final_dwell=5),reference='reference')
    ins.match=np.array([-14.,-4.]); ins.descent_started=0
    assert ins.step(Pose(1,-14,-4,5)).state=='TARGET_DESCEND'
    assert ins.step(Pose(5.9,-14,-4,5)).state=='TARGET_DESCEND'
    # A real movement interrupts the dwell, not its absolute descent deadline.
    ins.step(Pose(6,-14,-4,5,vn=.2))
    assert ins.step(Pose(7,-14,-4,5)).state=='TARGET_DESCEND'
    assert ins.step(Pose(11.9,-14,-4,5)).state=='TARGET_DESCEND'
    assert ins.step(Pose(12,-14,-4,5)).state=='TARGET_HOLD_5M'


def test_return_yaw_is_rate_limited_and_preserved_during_safety_hold():
    c=cfg(); g=ground(); r=ReturnNavigator(c,ReturnConfig())
    first,_=r.step(Pose(0,-14,-4,5,yaw=0),g)
    second,_=r.step(Pose(.1,-14,-4,5,yaw=0),g)
    assert first.yaw==0 and abs(second.yaw)<=.0081
    e=Engine(c)
    # The ground is deliberately stale/unobserved: HOLD must not turn back
    # toward the outbound heading while return acquisition is in progress.
    second.yaw=1.
    d=e.step(Pose(.1,-14,-4,5),inspection=second,descent_active=True)
    assert d.state=='HOLD' and d.yaw==1.


def test_return_climbs_then_routes_then_acquires_at_five_metres():
    g=ground(); r=ReturnNavigator(cfg(),ReturnConfig())
    first,_=r.step(Pose(0,-14,-4,5,yaw=math.pi),g)
    assert first.state=='RETURN_ASCEND' and first.vd<0
    r.step(Pose(10,-14,-4,10,yaw=math.pi),g)
    r.step(Pose(11.1,-14,-4,10,yaw=math.pi),g)
    d,_=r.step(Pose(11.2,-14,-4,10,yaw=math.pi),g)
    assert d.state=='RETURN_SURVEY_ENTRY' and d.ve>0
    r.step(Pose(25,-18.8,4.1,10,yaw=math.pi),g)
    assert r.state=='RETURN_ROUTE'
    r.step(Pose(30,-14,4.1,10,yaw=math.pi),g)
    d,_=r.step(Pose(30.1,-14,4.1,10,yaw=math.pi),g)
    assert d.state=='RETURN_ACQUISITION_HEIGHT' and d.vd>0
    r.step(Pose(40,-14,4.1,5,yaw=math.pi),g)
    r.step(Pose(41.1,-14,4.1,5,yaw=math.pi),g)
    d,_=r.step(Pose(41.2,-14,4.1,5,yaw=math.pi),g)
    assert d.state=='RETURN_FRONT' and d.vn==d.ve==d.vd==0


def test_return_route_does_not_cross_red_barrier():
    g=ground(); g.red[45:75,110:130]=True; g.refresh()
    r=ReturnNavigator(cfg(),ReturnConfig()); r.state='RETURN_ROUTE'
    p=Pose(0,-14,-6,10,yaw=math.pi)
    r.step(p,g)
    assert r.path
    start=p.xy
    for end in r.path:
        assert g.line_clear(start,end)
        start=end


def test_stalled_return_and_vertical_phases_are_bounded():
    g=ground(); p=ReturnConfig(no_progress_seconds=2,route_seconds=10)
    r=ReturnNavigator(cfg(),p); r.state='RETURN_ROUTE'
    r.step(Pose(0,-14,-4,10,yaw=math.pi),g)
    d,_=r.step(Pose(2.1,-14,-4,10,yaw=math.pi),g)
    assert d.state=='ABORTED'
    r=ReturnNavigator(cfg(),replace(p,vertical_seconds=1))
    r.step(Pose(0,-14,-4,5),g)
    assert r.step(Pose(1.1,-14,-4,5),g)[0].state=='ABORTED'


def test_stale_front_proposal_cannot_move():
    r=ReturnNavigator(cfg(),ReturnConfig()); r.state='RETURN_FRONT'
    d,permit=r.step(Pose(10,-14,4.1,5,yaw=math.pi),ground(),
        {'state':'RETURN_APPROACH','source_t':0.,'vn':-1.,'entrance_permit':True})
    assert d.vn==0 and permit is None


@pytest.mark.parametrize('proposal',[None,
    {'state':'RETURN_STAGE_DESCEND','source_t':0.,'vd':.4,'entrance_permit':True},
    {'state':'RETURN_STAGE_DESCEND','source_t':10.,'vd':.4,'entrance_permit':False}])
def test_staging_sensor_hold_retains_only_geometry_not_motion(proposal):
    r=ReturnNavigator(cfg(),ReturnConfig()); r.state='RETURN_FRONT'
    pose=Pose(10,-19.6032486,4.10327,2.3,yaw=math.pi)
    r.step(replace(pose,t=9.9),ground(),{'state':'RETURN_STAGE_DESCEND',
        'source_t':9.9,'entrance_permit':True,'vd':.03,'reason':'Staging'})
    if proposal is not None: proposal=dict(proposal,reason='Sensor hold')
    d,permit=r.step(pose,ground(),proposal)
    assert d.state=='HOLD' and d.vn==d.ve==d.vd==0 and permit is not None
    e=Engine(cfg()); e.ground=ground()
    assert e.step(pose,10,d,True,permit).state=='HOLD'
    for invalid in (replace(pose,e=7),replace(pose,yaw=0),replace(pose,alt=1)):
        assert not permit.allows(invalid)


def test_retained_staging_permit_still_rejects_new_red():
    r=ReturnNavigator(cfg(),ReturnConfig()); r.state='RETURN_FRONT'; r.entrance_active=True
    pose=Pose(10,-19.6,4.1,2.3,yaw=math.pi)
    e=Engine(cfg()); e.ground=ground()
    e.ground.red[e.ground.cell(pose.xy)]=True; e.ground.refresh()
    d,permit=r.step(pose,e.ground,None)
    result=e.step(pose,10,d,True,permit)
    assert result.state=='ABORTED' and 'clearance' in result.reason


def test_transition_requires_explicit_narrow_permit():
    c=cfg(); p=Pose(10,-19.8,4.1,2.7,yaw=math.pi)
    e=Engine(c); e.ground.observed[:]=True; e.ground.refresh(); e.ground.last_t=10.
    from coverage_mission.engine import Decision
    external=Decision('RETURN_APPROACH','Test',vn=-.1)
    permit=EntrancePermit(c,ReturnConfig())
    assert permit.allows(p)
    assert e.step(p,10,external,True,permit).state=='RETURN_APPROACH'
    e=Engine(c); e.ground.observed[:]=True; e.ground.refresh(); e.ground.last_t=10.
    assert e.step(p,10,external,True).state=='ABORTED'
    assert not permit.allows(replace(p,e=7.))
    assert not permit.allows(replace(p,yaw=0.))


def test_transition_cannot_ignore_red_or_unknown_ground():
    g=ground(); permit=EntrancePermit(cfg(),ReturnConfig())
    p=np.array([-19.8,4.1])
    assert permit.line_clear(g,p,p,2.7)
    g.red[g.cell(p)]=True; g.refresh(); g.last_t=11
    assert not permit.line_clear(g,p,p,2.7)
    g.red[:]=False; g.observed[:]=False; g.refresh(); g.last_t=12
    assert not permit.line_clear(g,p,p,2.7)


def test_transition_does_not_reset_existing_red_residence():
    e=Engine(cfg()); e.ground.observed[:]=True; e.ground.refresh(); e.ground.last_t=5
    e.residence.update(0,True)
    from coverage_mission.engine import Decision
    e.step(Pose(5,-20.1,4.1,2.7,yaw=math.pi),5,
           Decision('RETURN_STAGE_DESCEND','Test'),True,EntrancePermit(cfg(),ReturnConfig()))
    assert e.residence.entered==0 and e.residence.warned
