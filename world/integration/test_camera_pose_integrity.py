"""Offline tests for the forward-camera adapter and corridor pose gate."""

from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from camera_frame import decode_gazebo_bgr, gazebo_source_stamp_ns
from pose_integrity import PoseIntegrity
from source_clock import SourceClockGate
from autonomy.perception.hybrid_banner_detector import HybridBannerDetector


def message(width, height, step, data, pixel_format):
    return SimpleNamespace(width=width, height=height, step=step,
                           data=data, pixel_format_type=pixel_format)


def test_decoder_respects_stride_format_and_processing_width():
    rgb = bytes([255, 0, 0, 0, 255, 0, 99, 99] * 2)
    image = decode_gazebo_bgr(message(2, 2, 8, rgb, 3))
    assert image.shape == (2, 2, 3)
    assert image[0, 0].tolist() == [0, 0, 255]
    assert image[0, 1].tolist() == [0, 255, 0]
    bgr = bytes([1, 2, 3] * 4)
    small = decode_gazebo_bgr(message(2, 2, 0, bgr, 8), 1)
    assert small.shape == (1, 1, 3)
    assert small[0, 0].tolist() == [1, 2, 3]


def test_imx296_size_rgb_frame_can_be_processed_at_baseline_width():
    # Image transport is still Gazebo here; no front-lens FOV is assumed.
    native_width, native_height = 1440, 1080
    stride = native_width * 3 + 16
    raw = np.zeros((native_height, stride), dtype=np.uint8)
    raw[:, :native_width * 3] = np.array([0, 255, 0] * native_width,
                                        dtype=np.uint8)
    image = decode_gazebo_bgr(
        message(native_width, native_height, stride, raw.tobytes(), 3), 640)
    assert image.shape == (480, 640, 3)
    assert image[240, 320].tolist() == [0, 255, 0]


@pytest.mark.parametrize("msg", [
    message(2, 2, 5, bytes(10), 3),
    message(2, 2, 6, bytes(11), 3),
    message(2, 2, 6, bytes(12), 0),
])
def test_decoder_rejects_bad_layout(msg):
    with pytest.raises(ValueError):
        decode_gazebo_bgr(msg)


def test_gazebo_source_timestamp_is_preserved_but_not_host_age():
    class Header:
        stamp = SimpleNamespace(sec=12, nsec=345)
        def HasField(self, name):
            return name == "stamp"
    assert gazebo_source_stamp_ns(SimpleNamespace(header=Header())) == 12_000_000_345
    assert gazebo_source_stamp_ns(SimpleNamespace()) is None


def test_source_clock_replay_and_reset_do_not_refresh_sensor():
    gate = SourceClockGate("camera")
    assert gate.accept(None)
    assert gate.accept(100)
    assert not gate.accept(100)
    assert gate.duplicates == 1
    assert gate.accept(101)
    assert not gate.accept(99)
    assert "backward" in gate.failure
    assert not gate.accept(102)


def test_banner_area_scales_with_processing_resolution():
    for width, height in ((640, 480), (1280, 960)):
        frame = np.zeros((height, width, 3), dtype=np.uint8)
        scale = width / 640
        cv2.rectangle(frame, (round(250*scale), round(180*scale)),
                      (round(300*scale), round(230*scale)), (0, 255, 0), -1)
        result = HybridBannerDetector().detect(frame)
        assert result["detected"]
    with pytest.raises(ValueError):
        HybridBannerDetector(lower_green=(90, 0, 0), upper_green=(80, 255, 255))
    for options in ({"min_area_at_640x480": float("nan")},
                    {"morph_kernel_size": 4},
                    {"aspect_ratio_bounds": (5., .8)},
                    {"min_extent": float("nan")}):
        with pytest.raises(ValueError):
            HybridBannerDetector(**options)
    detector = HybridBannerDetector(morph_kernel_size=3)
    kernel = detector.morph_kernel
    detector.detect(np.zeros((480, 640, 3), dtype=np.uint8))
    assert detector.morph_kernel is kernel


def test_pose_integrity_rejects_jump_reset_and_nonfinite():
    gate = PoseIntegrity()
    assert gate.observe(10., (0., 0., -2.), (0., 0., 0.))
    assert not gate.observe(10., (50., 0., -2.), (0., 0., 0.))
    assert gate.failure is None  # Duplicate packets are ignored, not credited.
    assert gate.observe(10.1, (.1, 0., -2.), (1., 0., 0.))
    assert not gate.observe(10.2, (10., 0., -2.), (1., 0., 0.))
    assert "jump" in gate.failure
    gate = PoseIntegrity()
    assert gate.observe(10., (0., 0., 0.), (0., 0., 0.))
    assert not gate.observe(9., (0., 0., 0.), (0., 0., 0.))
    assert "clock reset" in gate.failure
    gate = PoseIntegrity()
    assert not gate.observe(1., (float("nan"), 0., 0.), (0., 0., 0.))
    assert "nonfinite" in gate.failure
