"""Regressions for grass-derived detections and inspection starvation."""
from dataclasses import replace
import math
from unittest.mock import Mock
import numpy as np
import pytest
from coverage_mission.geometry import Pose
from coverage_mission.qr import Inspection, QRConfig
from coverage_mission.qr_detector import QRDetector
from coverage_mission.test_qr import cfg, observation, select, code


@pytest.mark.parametrize('quad',[
    [[345,477],[213,317],[637,-394],[637,329]],  # Actual failed-run extrapolation.
    [[311,132],[403,424],[403,684],[72,451]],    # Another actual failed-run detection.
    [[100,100],[180,100],[180,100],[100,180]],  # Triangle masquerading as a quad.
    [[100,100],[180,180],[180,100],[100,180]],  # Crossing edges.
    [[100,100],[220,100],[220,112],[100,112]],  # Thin grass strip.
    [[100,100],[180,100],[300,180],[220,180]],  # Acute skew.
    [[100,100],[180,100],[180,math.nan],[100,180]],
])
def test_invalid_geometry_never_reaches_candidate_projection(quad):
    detector=QRDetector()
    detector.detector=Mock()
    detector.detector.detectMulti.return_value=(True,np.asarray([quad],np.float32))
    assert detector.candidates(np.zeros((480,640),np.uint8))==[]


@pytest.mark.parametrize('angle',[0,25,90,155])
def test_shape_gate_preserves_rotated_and_modestly_distorted_markers(angle):
    quad=np.array([[-40,-35],[40,-40],[42,40],[-39,42]],float)
    a=math.radians(angle); rotation=np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]])
    assert QRDetector().valid_quad(quad@rotation.T+[320,240],(480,640))


def test_one_detection_or_duplicate_exposure_cannot_interrupt_sweep():
    ins=Inspection(cfg(),reference='REF-001')
    result=observation(1,0,xy=(1,0))
    assert ins.step(Pose(0,0,0,10),result) is None
    for t in (.05,.1,.2):
        assert ins.step(Pose(t,0,0,10),result) is None
    assert ins.candidates[0]['sightings']==1 and not ins.selected


def test_incomplete_observations_are_not_candidates():
    ins=Inspection(cfg(),reference='REF-001')
    for i in range(8):
        assert ins.step(Pose(i*.2,0,0,10),observation(i,i*.2,complete=False)) is None
    assert not ins.candidates


def test_missed_exposure_resets_discovery_confirmation():
    ins=Inspection(cfg(),reference='REF-001')
    for i,t in enumerate((0,.2)):
        ins.step(Pose(t,0,0,10),observation(i,t))
    absent=observation(3,.4); absent['observations']=[]
    ins.step(Pose(.4,0,0,10),absent)
    assert ins.candidates[0]['sightings']==0
    assert ins.step(Pose(.6,0,0,10),observation(4,.6)) is None


def test_unstable_world_positions_do_not_confirm_despite_association():
    ins=Inspection(cfg(),reference='REF-001')
    for i in range(12):
        xy=(0 if i%2 else .3,0)
        assert ins.step(Pose(i*.2,0,0,10),observation(i,i*.2,xy=xy)) is None
    assert not ins.selected


def test_stale_candidate_is_not_reselected_after_failure():
    ins=Inspection(cfg(),reference='REF-001'); select(ins)
    assert ins.selected
    assert ins.step(Pose(1.01,0,0,10)) is None
    assert ins.selected is None and ins.events[-1]['state']=='QR_LOST'
    for t in (2,11,30):
        assert ins.step(Pose(t,0,0,10)) is None
    assert ins.candidates[0]['attempts']==1


def test_fresh_other_candidates_cannot_bypass_global_coverage_resume():
    ins=Inspection(cfg(),reference='REF-001'); select(ins)
    ins.step(Pose(1.01,0,0,10))
    for i,t in enumerate((1.2,1.4,1.6),100):
        assert ins.step(Pose(t,0,0,10),observation(i,t,xy=(1,0),generation=ins.generation)) is None
    assert ins.selected is None
    for i,t in enumerate((4.1,4.3,4.5),110):
        decision=ins.step(Pose(t,0,0,10),observation(i,t,xy=(1,0),generation=ins.generation))
    assert ins.selected and decision.state=='QR_CENTER'


def test_persistent_candidate_with_no_motion_is_released_for_coverage():
    ins=Inspection(cfg(),reference='REF-001')
    select(ins,xy=(1,0))
    # Hold aircraft away from target while supplying genuinely fresh sightings.
    for i in range(1,30):
        t=i*.2
        result=observation(20+i,t,xy=(1,0),generation=ins.generation)
        ins.step(Pose(t,0,0,10),result)
    assert ins.selected is None
    assert any(e['state']=='QR_CENTER_NO_PROGRESS' for e in ins.events)
    assert ins.candidates[0]['attempts']==1


def test_lost_candidate_has_only_two_attempts_even_after_rediscovery():
    ins=Inspection(cfg(),reference='REF-001'); select(ins)
    ins.step(Pose(1.01,0,0,10))
    for i,t in enumerate((11.1,11.3,11.5),50):
        ins.step(Pose(t,0,0,10),observation(i,t,generation=ins.generation))
    assert ins.selected and ins.selected['attempts']==2
    ins.step(Pose(12.51,0,0,10))
    assert ins.candidates[0]['status']=='UNREADABLE'
    for i,t in enumerate((30,30.2,30.4),60):
        assert ins.step(Pose(t,0,0,10),observation(i,t,generation=ins.generation)) is None


def test_current_real_qr_patterns_still_discover_and_decode():
    for version in (1,3,5):
        frame=code('REF-001',version=version)
        found=QRDetector().candidates(frame)
        assert len(found)==1
        assert QRDetector.decode_selected(frame,found[0]['quad'])[0]=='REF-001'
