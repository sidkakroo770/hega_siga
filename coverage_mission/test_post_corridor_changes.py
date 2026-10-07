"""Contracts for the post-corridor safety and Pi-cost changes."""
from dataclasses import replace
import math

import numpy as np
import pytest

from coverage_mission.config import Camera, Config
from coverage_mission.engine import Engine
from coverage_mission.field_frame import FieldFrame
from coverage_mission.geometry import Pose, red_regions
from coverage_mission.planning import GroundMap
from coverage_mission.planning import CoveragePlan, route
import coverage_mission.planning as planning_module
from coverage_mission.runtime import safe_zero_hold
from command_service import CommandService


def test_five_distinct_red_observations_confirm_but_first_hit_blocks():
    cfg=Config()
    ground=GroundMap(cfg)
    foot=[[-1,-2],[5,-2],[5,5],[-1,5]]
    red=[[1,0],[2,0],[2,1],[1,1]]
    for t in range(1,5):
        assert ground.observe(foot,[red],t)
        assert ground.red.any()
        assert not ground.confirmed.any()
    assert not ground.observe(foot,[red],4)
    assert ground.observe(foot,[red],5)
    assert ground.confirmed.any()


def test_unreadable_part_of_image_cannot_become_clear_ground():
    cfg=replace(Config(),n_max=10.4,e_max=7.8)
    engine=Engine(cfg)
    image=np.full((cfg.camera.height,cfg.camera.width,3),120,np.uint8)
    image[:, :cfg.camera.width//2]=0
    pose=Pose(1,2,2,cfg.altitude)
    engine.observe(image,pose)
    projected=engine.projector.project([[cfg.camera.width*.25,cfg.camera.height*.5],
                                        [cfg.camera.width*.75,cfg.camera.height*.5]],pose)
    assert not engine.ground.observed[engine.ground.cell(projected[0])]
    assert engine.ground.observed[engine.ground.cell(projected[1])]


def test_dark_red_is_provisional_hazard_with_configurable_thresholds():
    cfg=Config()
    image=np.full((cfg.camera.height,cfg.camera.width,3),120,np.uint8)
    image[100:150,100:150]=(0,0,30)
    mask,polygons=red_regions(image,cfg)
    assert mask[120,120] and polygons


def test_provisional_green_texture_evidence_fills_dark_grass_not_black_or_red():
    cfg=replace(Config(),n_max=10.4,e_max=7.8,
                green_texture_support_m=.4,green_texture_mean_min=15)
    engine=Engine(cfg)
    image=np.zeros((cfg.camera.height,cfg.camera.width,3),np.uint8)
    image[:,:cfg.camera.width//2:2]=(1,46,4)
    image[:,cfg.camera.width//2:]=(0,0,0)
    image[160:220,100:160]=(0,0,70)
    pose=Pose(1,2,2,cfg.altitude)
    engine.observe(image,pose)
    grass,black,red=engine.projector.project(
        [[200,360],[520,240],[130,190]],pose)
    ground=engine.ground
    assert ground.free[ground.cell(grass)]
    assert not ground.observed[ground.cell(black)]
    assert ground.red[ground.cell(red)]


def test_full_world_nonred_dark_patch_is_clear_but_red_still_blocks():
    cfg=Config.load('config/full_mission_coverage.json')
    engine=Engine(cfg)
    image=np.full((cfg.camera.height,cfg.camera.width,3),(1,46,4),np.uint8)
    image[80:250,350:600]=0
    image[150:200,100:150]=(0,0,100)
    pose=Pose(1,0,0,cfg.altitude)
    engine.observe(image,pose)
    grass,black,red=engine.projector.project([[200,300],[450,160],[125,175]],pose)
    assert engine.ground.observed[engine.ground.cell(grass)]
    assert engine.ground.observed[engine.ground.cell(black)]
    assert engine.ground.red[engine.ground.cell(red)]


def test_neutral_print_evidence_fills_local_black_modules_not_dark_ground():
    cfg=replace(Config(),n_max=10.4,e_max=7.8,neutral_print_support_m=.5)
    engine=Engine(cfg)
    image=np.full((cfg.camera.height,cfg.camera.width,3),(20,100,20),np.uint8)
    image[120:200,100:180]=255
    image[140:160,130:150]=0  # A black module surrounded by white print.
    image[300:420,400:540]=0  # A broad black area has no neutral support.
    image[200:280,250:330]=(0,0,80)
    image[230:250,280:300]=0  # Red-dominated dark interior stays unknown.
    pose=Pose(1,2,2,cfg.altitude)
    engine.observe(image,pose)
    print_pixel,black_pixel,red_dark_pixel=engine.projector.project(
        [[140,150],[470,360],[290,240]],pose)
    ground=engine.ground
    assert ground.observed[ground.cell(print_pixel)]
    assert not ground.observed[ground.cell(black_pixel)]
    assert not ground.observed[ground.cell(red_dark_pixel)]


def test_tiny_enclosed_nonred_unknown_holes_do_not_block_lane():
    cfg=replace(Config(),n_max=10.4,e_max=7.8,small_unknown_hole_area=.2)
    ground=GroundMap(cfg)
    ground.observed[:]=True
    tiny=(55,35)
    large=(85,55)
    near_red=(105,35)
    ground.observed[54:57,34:37]=False
    ground.observed[80:91,50:61]=False
    ground.observed[104:107,34:37]=False
    ground.red[100:104,34:37]=True
    ground.refresh()
    assert ground.contextual_clear[tiny] and ground.free[tiny]
    assert not ground.observed[tiny]  # Inference is not direct observation.
    assert not ground.contextual_clear[large]
    assert not ground.contextual_clear[near_red]


def test_contextually_clear_hole_does_not_hold_completed_mission_in_repair():
    cfg=replace(Config(),n_max=10.4,e_max=7.8,small_unknown_hole_area=.2)
    engine=Engine(cfg)
    ground=engine.ground
    ground.observed[:]=True
    ground.observed[54:57,34:37]=False
    ground.refresh()
    assert ground.contextual_clear[55,35]
    engine.plan.done[:]=True
    for t in (1.,3.1):
        ground.last_t=t
        decision=engine.step(Pose(t,2.,2.,cfg.altitude),camera_t=t)
    assert decision.state=='COMPLETE'
    assert decision.unseen==0


def test_hover_jitter_cannot_keep_no_route_hold_alive():
    cfg=replace(Config(),n_max=10.4,e_max=7.8,no_progress=1.)
    engine=Engine(cfg)
    for t in (1.,1.4,1.8,2.2):
        cell=engine.ground.cell((2.,2.))
        engine.ground.observed[cell]=True
        engine.ground.observed[0,int(t*10)]=True
        engine.ground.last_t=t
        engine.ground.refresh()
        decision=engine.step(Pose(t,2.,2.,cfg.altitude),camera_t=t)
    assert decision.state=='BLOCKED'
    assert 'no observable/reachable' in decision.reason.lower()


def test_tiny_new_pixels_cannot_keep_a_looping_route_alive():
    cfg=replace(Config(),n_max=10.4,e_max=7.8,no_progress=1.,
                minimum_observation_progress_area=1.)
    engine=Engine(cfg)
    engine.ground.observed[:]=True
    engine.ground.observed[0,:]=False
    engine.path=[np.array([3.,2.])]
    engine.last_plan=100.
    for t in (1.,1.4,1.8,2.2):
        engine.ground.observed[0,int(t*10)]=True
        engine.ground.last_t=t
        engine.ground.refresh()
        decision=engine.step(Pose(t,2.,2.,cfg.altitude),camera_t=t)
    assert decision.state=='BLOCKED'
    assert decision.reason=='No measured traversal/observation progress'


def test_long_checked_relocation_gets_finite_progress_credit():
    cfg=replace(Config(),n_max=10.4,e_max=7.8,no_progress=2.,
                relocation_credit_speed=.2)
    engine=Engine(cfg)
    engine.ground.observed[:]=True
    engine.ground.observed[90,70]=False
    engine.ground.refresh()
    engine.plan.done[:]=True
    engine.path=[np.array([6.,2.])]
    for t,n in ((1.,2.),(4.,3.)):
        engine.ground.last_t=t
        decision=engine.step(Pose(t,n,2.,cfg.altitude),camera_t=t)
    assert decision.state!='BLOCKED'
    # Continuing to hover or loop within the same net displacement cannot
    # refresh the finite deadline without new map or traversal evidence.
    engine.ground.last_t=9.
    decision=engine.step(Pose(9.,3.,2.,cfg.altitude),camera_t=9.)
    assert decision.state=='BLOCKED'
    assert decision.reason=='No measured traversal/observation progress'


def test_checked_lane_lookahead_does_not_change_dense_traversal_obligations():
    cfg=replace(Config(),n_max=10.4,e_max=7.8)
    ground=GroundMap(cfg)
    plan=CoveragePlan(cfg)
    ground.observed[:]=True
    ground.refresh()
    start=plan.points[0]
    target=plan.reachable_target(ground,start)
    assert np.linalg.norm(target-start)>1.
    assert ground.line_clear(start,target)
    assert len(plan.pending())==len(plan.points)
    # Looking ahead must never turn a nearby completed point into a pursuit
    # target; that once stalled the integrated mission at the field entrance.
    plan.done[5]=True
    target_before_done=plan.reachable_target(ground,start)
    assert np.allclose(target_before_done,plan.points[4])
    plan.done[5]=False
    ground.red[ground.cell((start+target)/2)]=True
    ground.refresh()
    blocked=plan.reachable_target(ground,start)
    assert ground.line_clear(start,blocked) or np.allclose(blocked,start)


def test_unknown_ordered_point_cannot_displace_a_known_reachable_obligation():
    cfg=replace(Config(),n_max=10.4,e_max=7.8)
    ground=GroundMap(cfg)
    plan=CoveragePlan(cfg)
    ground.observed[:]=True
    first=tuple(plan.cells[0])
    ground.observed[first]=False
    ground.refresh()
    current=plan.points[50]
    assert ground.free[ground.cell(current)]
    target=plan.reachable_target(ground,current)
    assert ground.free[ground.cell(target)]


def test_unknown_goal_cannot_generate_self_frontier_route():
    cfg=replace(Config(),n_max=10.4,e_max=7.8)
    ground=GroundMap(cfg)
    current=np.array([2.,2.])
    centre=ground.cell(current)
    ground.observed[centre[0]-20:centre[0]+21,
                    centre[1]-20:centre[1]+21]=True
    ground.refresh()
    assert ground.free[centre]
    path=route(ground,current,np.array([8.,5.]),budget_s=1.)
    minimum=max(2*cfg.arrival,min(cfg.camera.footprint(cfg.altitude-cfg.altitude_tolerance))/4)
    assert path is None or not path or np.linalg.norm(path[-1]-current)>=minimum


def test_exterior_unknown_frontier_outranks_dark_interior_hole():
    cfg=replace(Config(),n_max=10.4,e_max=7.8)
    ground=GroundMap(cfg)
    ground.observed[20:100,12:52]=True
    ground.observed[48:52,29:33]=False
    ground.refresh()
    current=np.array([5.,1.5])
    assert ground.free[ground.cell(current)]
    path=route(ground,current,np.array([9.,6.]),budget_s=1.)
    assert path and path[-1][0]>6.


def test_long_connector_keeps_hard_clearance_without_soft_search_penalty(monkeypatch):
    cfg=Config.load('config/full_mission_coverage.json')
    ground=GroundMap(cfg)
    ground.observed[:]=True
    # A red barrier forces a checked connector around one end of the field.
    ground.red[100:300,140:160]=True
    ground.refresh()
    start=np.array([-15.,-10.])
    goal=np.array([15.,10.])
    searched=[]
    original=planning_module.astar

    def record_search(mask,source,target,clearance=None,preference=.3,deadline=None):
        searched.append(preference)
        return original(mask,source,target,clearance,preference,deadline)

    monkeypatch.setattr(planning_module,'astar',record_search)
    path=route(ground,start,goal,budget_s=1.)
    assert path and searched and all(preference==0 for preference in searched)
    assert all(ground.line_clear(a,b) for a,b in zip([start]+path[:-1],path))


def test_observed_unfree_lane_point_uses_reachable_arrival_disk():
    cfg=replace(Config(),n_max=10.4,e_max=7.8,arrival=.3)
    ground=GroundMap(cfg)
    ground.observed[:]=True
    target=np.array([5.,2.])
    cell=ground.cell(target)
    ground.observed[cell[0],cell[1]+5]=False
    ground.refresh()
    assert ground.observed[cell] and not ground.free[cell]
    start=np.array([2.,2.])
    path=route(ground,start,target,budget_s=1.)
    assert path and np.linalg.norm(path[-1]-target)<=cfg.arrival-cfg.resolution/3
    assert all(ground.line_clear(a,b) for a,b in zip([start]+path[:-1],path))


def test_rotated_field_registration_from_gps_origin_round_trips():
    lat,lon=12.,77.
    yaw=.38
    c,s=math.cos(yaw),math.sin(yaw)
    corners=[(0.,0.),(40*c,40*s),(40*c-30*s,40*s+30*c),(-30*s,30*c)]
    a=6378137.; e2=.00669437999014; latitude_rad=math.radians(lat)
    denominator=1-e2*math.sin(latitude_rad)**2
    meridian=a*(1-e2)/denominator**1.5
    prime_vertical=a/math.sqrt(denominator)
    gps=[(lat+n/meridian*180/math.pi,
          lon+e/(prime_vertical*math.cos(latitude_rad))*180/math.pi)
         for n,e in corners]
    cfg=replace(Config(),geofence_latlon=tuple(gps)).register_origin(lat,lon)
    assert cfg.n_min==cfg.e_min==0.
    assert cfg.n_max==pytest.approx(40,abs=.01)
    assert cfg.e_max==pytest.approx(30,abs=.01)
    frame=FieldFrame(cfg)
    point=np.array([10.,6.])
    assert np.allclose(frame.point(frame.local_point(point)),point)
    assert np.allclose(frame.vector(frame.local_vector(point)),point)


def test_nonrectangular_geofence_fails_closed():
    lat,lon=12.,77.
    cfg=replace(Config(),geofence_latlon=((lat,lon),(lat+.0003,lon),
                                          (lat+.0003,lon+.0003),(lat+.00001,lon+.0003)))
    with pytest.raises(ValueError,match='rectangle'):
        cfg.register_origin(lat,lon)


def test_pose_jump_inconsistent_with_measured_velocity_invalidates_map():
    cfg=replace(Config(),n_max=10.4,e_max=7.8)
    engine=Engine(cfg)
    engine.ground.observed[:]=True
    engine.ground.refresh()
    engine.ground.last_t=1.4
    engine.step(Pose(1.,2.,2.,cfg.altitude),camera_t=1.)
    decision=engine.step(Pose(1.4,2.8,2.,cfg.altitude),camera_t=1.4)
    assert decision.state=='ABORTED'


def test_unsupported_distortion_rejected_before_mapping():
    with pytest.raises(ValueError,match='distortion'):
        replace(Config(),camera=replace(Camera(),distortion=(-.1,0.,0.,0.,0.)))


def test_stage_claim_revokes_old_command_and_final_zero_is_transmitted():
    sent=[]
    def transmit(*command,frame='body'):
        sent.append((frame,command))
    service=CommandService(transmit,period_s=.005,lease_s=.02)
    service.start()
    token=service.claim()
    with pytest.raises(RuntimeError,match='Stale'):
        service.publish(1.,0.,0.)
    service.publish(.2,.1,0.,.4,token=token,frame='local')
    service.stop()
    assert sent[-1]==('local',(0.,0.,0.,.4))


def test_planner_budget_is_distinct_from_no_safe_route(monkeypatch):
    cfg=replace(Config(),n_max=10.4,e_max=7.8)
    ground=GroundMap(cfg)
    ground.observed[:]=True
    ground.red[40:80,40:60]=True
    ground.refresh()
    assert route(ground,np.array([2.,-1.]),np.array([8.,5.]),budget_s=1e-9) is None


def test_direct_frontier_avoids_astar_when_unknown_goal_is_distant(monkeypatch):
    cfg=Config.load('config/full_mission_coverage.json')
    ground=GroundMap(cfg)
    centre=ground.cell((0.,0.))
    ground.observed[centre[0]-45:centre[0]+46,
                    centre[1]-45:centre[1]+46]=True
    ground.refresh()
    monkeypatch.setattr(planning_module,'astar',
                        lambda *args,**kwargs: pytest.fail('unneeded A*'))
    start=np.array([0.,0.])
    path=route(ground,start,np.array([18.,12.]),budget_s=.2)
    assert path and len(path)==1 and ground.line_clear(start,path[0])
    assert np.linalg.norm(path[0]-start)>cfg.arrival


def test_stale_motion_never_uses_zero_hold_exception():
    hold={'state':'HOLD','vn':0.,'ve':0.,'vd':0.,'yaw':0.}
    assert safe_zero_hold(hold,decision_recent=True,sensors_healthy=True)
    assert not safe_zero_hold(hold,decision_recent=False,sensors_healthy=True)
    assert not safe_zero_hold(hold,decision_recent=True,sensors_healthy=False)
    assert not safe_zero_hold(dict(hold,vn=.01),decision_recent=True,sensors_healthy=True)
    assert not safe_zero_hold(dict(hold,state='SWEEP'),decision_recent=True,sensors_healthy=True)


def test_over_budget_replan_preserves_only_checked_old_leg(monkeypatch):
    import coverage_mission.engine as engine_module
    cfg=replace(Config(),n_max=10.4,e_max=7.8)
    engine=Engine(cfg)
    engine.ground.observed[:]=True
    engine.ground.refresh()
    engine.ground.last_t=1.
    engine.path=[np.array([1.,0.])]
    monkeypatch.setattr(engine_module,'route',lambda *args: None)
    decision=engine.step(Pose(1.,0.,0.,cfg.altitude),camera_t=1.)
    assert engine.path and engine.planning_timeouts==1
    assert decision.state in ('SWEEP','COVERAGE_REPAIR')
    assert engine.next_route_attempt>1.
