"""Front acquisition, read ownership and return landing authority regressions."""
import math
import time
from types import SimpleNamespace
from pathlib import Path
from unittest.mock import Mock
import cv2
import numpy as np
import pytest

from return_approach import ReturnApproach
from coverage_mission.config import Config
from coverage_mission.geometry import Pose
from coverage_mission.return_mission import ReturnConfig
from native.common.types import NativeScan


def fixture():
    cfg=Config.load(Path(__file__).parents[2]/'config/full_mission_coverage.json')
    args=SimpleNamespace(banner_min_area_640x480=700,banner_morph_kernel=5,
        banner_aspect_range=(.8,5),banner_min_extent=.15,camera_max_age=.30,
        center_tolerance_px=20,camera_gain=.003,camera_max_speed=.5,
        center_frames=2,entrance_commit_range=.5,pre_entry_descent=1.,approach_speed=.2)
    hsv=np.zeros((480,640,3),np.uint8); hsv[190:290,145:495]=(15,255,255)
    image=cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)
    now=time.monotonic()
    scan=NativeScan(np.linspace(-math.pi,math.pi,721),np.full(721,12.),
                    np.zeros(721),now,no_return_is_clear=True,source_timestamp=10_000_000_000)
    data=[image,1,now,10_000_000_000,scan,1,None]
    ctrl=ReturnApproach(cfg,ReturnConfig(),args,lambda:tuple(data))
    return ctrl,data,now


def test_orange_detection_centres_and_approaches_in_southbound_ned():
    c,data,now=fixture(); pose=Pose(10,-14,4.1,3.7,yaw=math.pi)
    first=c(pose,now,10)
    assert first['state']=='RETURN_BANNER_CENTER'
    data[1]=2
    second=c(pose,now,10)
    assert second['state']=='RETURN_APPROACH'
    assert second['vn']<0 and abs(second['ve'])<.01
    assert second['entrance_permit']


def test_duplicate_camera_exposure_does_not_count_as_centred():
    c,data,now=fixture(); pose=Pose(10,-14,4.1,3.7,yaw=math.pi)
    for _ in range(20): c(pose,now,10)
    assert c.centered==1 and c.state=='RETURN_BANNER_CENTER'


@pytest.mark.parametrize('fault',['missing-camera','old-camera','missing-lidar','old-source','wrong-heading'])
def test_sensor_and_heading_failures_cannot_authorize_motion(fault):
    c,data,now=fixture(); pose=Pose(10,-14,4.1,5,yaw=math.pi)
    if fault=='missing-camera': data[0]=None
    if fault=='old-camera': data[2]=now-1
    if fault=='missing-lidar': data[4]=None
    if fault=='old-source': data[3]=8_000_000_000
    if fault=='wrong-heading': pose=Pose(10,-14,4.1,5,yaw=0)
    d=c(pose,now,10)
    assert d['vn']==d['ve']==d['vd']==0 and not d['entrance_permit']


def test_camera_loss_is_not_arrival_and_search_is_bounded():
    c,data,now=fixture(); data[0][:]=0
    d=c(Pose(10,-14,4.1,5,yaw=math.pi),now,10)
    assert d['state']=='RETURN_BANNER_SEARCH' and d['vn']==0
    d=c(Pose(191,-14,4.1,5,yaw=math.pi),now,191)
    assert d['state']=='ABORTED'


def test_close_lidar_is_only_staging_stop_not_entry_success():
    c,data,now=fixture(); pose=Pose(10,-19.5,4.1,3.7,yaw=math.pi)
    c(pose,now,10); data[1]=2; data[4].ranges_m[:]=.4
    d=c(pose,now,10)
    assert d['state']=='RETURN_STAGE_DESCEND'
    assert d['vn']==d['ve']==d['vd']==0
    assert c.readiness.count==0 and c.target_alt==pytest.approx(2.7)


def test_orange_preview_is_restarted_and_contains_mask():
    c,data,now=fixture()
    c.preview=Mock(enabled=True); c.activate=Mock()
    pose=Pose(10,-14,4.1,3.7,yaw=math.pi)
    c(pose,now,10); data[1]+=1; c(pose,now,10)
    c.activate.assert_called_once()
    c.preview.start.assert_called_once_with('Orange Banner Camera — MASK')
    debug=c.preview.submit.call_args.args[0]
    assert debug is not None and debug.shape==data[0].shape
    # The orange rectangle becomes white in the inset mask, the background black.
    assert np.array_equal(debug[60,70],np.array([255,255,255]))
    assert np.array_equal(debug[30,10],np.array([0,0,0]))
    assert c.preview.submit.call_count==2


def test_suspended_forward_callback_does_not_decode_or_cache(monkeypatch):
    import experimental_corridor_manager as manager
    monkeypatch.setattr(manager,'forward_camera_active',False)
    decoder=Mock(side_effect=AssertionError('disabled camera decoded'))
    monkeypatch.setattr(manager,'decode_gazebo_bgr',decoder)
    manager.on_forward_image(Mock())
    decoder.assert_not_called()


def test_forward_suspend_resume_discards_old_cache_and_subscribes_once(monkeypatch):
    import experimental_corridor_manager as manager
    monkeypatch.setattr(manager,'forward_camera_active',True)
    monkeypatch.setattr(manager,'forward_camera_generation',0)
    monkeypatch.setattr(manager,'latest_forward_frame',np.ones((4,4,3),np.uint8))
    monkeypatch.setattr(manager,'camera_receipt_time',99.)
    monkeypatch.setattr(manager,'camera_source_stamp_ns',99)
    node=Mock(); node.subscribe.return_value=True
    manager.set_forward_camera_active(node,False)
    assert not manager.forward_camera_active and manager.latest_forward_frame is None
    manager.set_forward_camera_active(node,False)
    node.unsubscribe.assert_called_once_with('/iris/camera_forward/image_raw')
    manager.set_forward_camera_active(node,True)
    assert manager.forward_camera_active and manager.camera_receipt_time==0
    manager.set_forward_camera_active(node,True)
    assert node.subscribe.call_count==1


def test_expected_land_does_not_revoke_but_pilot_takeover_does(monkeypatch):
    import experimental_corridor_manager as manager
    from pymavlink import mavutil
    t=manager.Telemetry(authority_started=True,landing_expected=True)
    monkeypatch.setattr(manager,'telemetry',t)
    master=Mock(target_system=1,target_component=1)
    def heartbeat(mode,armed):
        msg=Mock(base_mode=128 if armed else 0)
        msg.get_srcSystem.return_value=1; msg.get_srcComponent.return_value=1
        msg.get_type.return_value='HEARTBEAT'
        master.recv_match.side_effect=[msg,None]
        monkeypatch.setattr(mavutil,'mode_string_v10',lambda _:mode)
        manager.drain_mavlink(master)
    heartbeat('LAND',True); assert not t.authority_revoked
    heartbeat('LAND',False); assert not t.authority_revoked
    heartbeat('LOITER',True); assert t.authority_revoked


def test_landing_requires_fresh_touchdown_disarm_and_exterior_region():
    from experimental_corridor_manager import Telemetry,landing_confirmed_feedback
    p=ReturnConfig()
    t=Telemetry(x_m=-32.1,y_m=4.1,relative_alt_m=0.,vx_m_s=0.,vy_m_s=0.,vz_m_s=0.,
        mode='LAND',armed=False,landed_state=1,heartbeat_time=10.,landed_time=10.,
        position_time=10.,relative_alt_time=10.)
    assert landing_confirmed_feedback(t,10.1,9.,p)
    t.armed=True; assert not landing_confirmed_feedback(t,10.1,9.,p)
    t.armed=False; t.x_m=-29.; assert not landing_confirmed_feedback(t,10.1,9.,p)
    t.x_m=-32.1; t.landed_time=5.; assert not landing_confirmed_feedback(t,10.1,9.,p)
