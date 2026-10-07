"""Offline full-world integration contracts; no MAVLink or Gazebo connections."""
from dataclasses import replace
from pathlib import Path
import math
import time

import numpy as np

from command_service import CommandService
from async_log import AsyncLogStream
from health_metrics import HealthMetrics
from preview_service import PreviewService
from snapshot_service import SnapshotService
from entrance_readiness import EntranceReadiness, StagingEnvelope
from startup_control import StartupController, StartupFeedback
from native.common.types import Attitude, NativeScan
from native.mission_runner import VehiclePose


def feedback(now, **changes):
    base = StartupFeedback(
        heartbeat_time=now, mode="STABILIZE", armed=False,
        landed_state=1, landed_time=now, ekf_flags=8, ekf_time=now,
        position_time=now, attitude_time=now, relative_alt_time=now,
        relative_alt_m=0., position_m=(0., 0., 0.),
        velocity_m_s=(0., 0., 0.), camera_time=now, lidar_time=now)
    base = replace(base, prearm_ok=True, prearm_time=now)
    return replace(base, **changes)


def test_autonomous_startup_requires_confirmation_and_continuous_settle():
    c = StartupController(100.)
    assert c.step(feedback(100.1), 100.1) == "MODE"
    assert c.step(feedback(100.2), 100.2) is None
    assert c.step(feedback(101., mode="GUIDED"), 101.) == "ARM"
    assert c.step(feedback(102., mode="GUIDED", armed=True), 102.) == "TAKEOFF"
    assert c.step(feedback(105., mode="GUIDED", armed=True,
                           relative_alt_m=2.9), 105.) is None
    assert c.state == "SETTLING"
    assert c.step(feedback(106., mode="GUIDED", armed=True,
                           relative_alt_m=3., velocity_m_s=(.2, 0, 0)), 106.) is None
    assert c.settled_since is None
    assert c.step(feedback(107., mode="GUIDED", armed=True,
                           relative_alt_m=3.), 107.) is None
    assert c.step(feedback(109.1, mode="GUIDED", armed=True,
                           relative_alt_m=3.), 109.1) is None
    assert c.state == "READY"


def test_startup_rejects_already_armed_stale_feedback_and_command_denial():
    c = StartupController(100.)
    c.step(feedback(100.1, armed=True), 100.1)
    assert c.state == "ABORT" and "already armed" in c.reason
    c = StartupController(100.)
    c.step(feedback(100.1, camera_time=0.), 100.1)
    assert c.state == "PREFLIGHT"
    c.step(feedback(126., camera_time=0.), 126.)
    assert c.state == "ABORT"
    c = StartupController(100.)
    c.step(feedback(100.1), 100.1)
    c.step(feedback(101., rejected_command=176), 101.)
    assert c.state == "ABORT"


def test_takeoff_requires_altitude_feedback_and_stops_on_mode_loss():
    c = StartupController(100.)
    c.step(feedback(100.1, mode="GUIDED"), 100.1)
    c.step(feedback(101., mode="GUIDED", armed=True), 101.)
    c.step(feedback(102., mode="GUIDED", armed=True,
                    relative_alt_m=3., relative_alt_time=100.), 102.)
    assert c.state == "WAIT_TAKEOFF"
    c.step(feedback(103., mode="LOITER", armed=True), 103.)
    assert c.state == "ABORT"


def captured_scan(stamp):
    source = Path(__file__).parent / "artifacts/preentry_capture/preentry_scan.npz"
    with np.load(source) as data:
        return NativeScan(data["angles_rad"], data["ranges_m"],
                          np.zeros(len(data["ranges_m"])), stamp,
                          range_min_m=float(data["range_min_m"]),
                          range_max_m=float(data["range_max_m"]),
                          no_return_is_clear=True)


def test_handoff_requires_distinct_supported_scans_and_settled_scene_region():
    envelope = StagingEnvelope()
    assert envelope.contains(-30.45, -4., 2.6)
    assert not envelope.contains(-33., -4., 2.6)
    assert not envelope.contains(-30.45, -4., .8)
    readiness = EntranceReadiness()
    for seq in (1, 2, 3):
        stamp = time.monotonic()
        valid = readiness.observe(seq, captured_scan(stamp),
                                  Attitude(timestamp=stamp),
                                  VehiclePose(-30.45, -4., 0., stamp))
        assert valid == (seq == 3)
        if seq == 1:
            assert not readiness.observe(seq, captured_scan(stamp),
                                         Attitude(timestamp=stamp),
                                         VehiclePose(-30.45, -4., 0., stamp))
    stamp = time.monotonic()
    bad = captured_scan(stamp)
    bad.ranges_m[:] = math.nan
    assert not readiness.observe(4, bad, Attitude(timestamp=stamp),
                                 VehiclePose(-30.45, -4., 0., stamp))
    assert readiness.count == 0


def test_command_lease_zeroes_stale_motion_and_worker_failure_visible():
    sent = []
    service = CommandService(lambda *v: sent.append(v), period_s=.005, lease_s=.03)
    service.start()
    service.publish(.2, 0, 0, source_timestamp=time.monotonic())
    time.sleep(.02)
    assert (.2, 0., 0., 0.) in sent
    time.sleep(.06)
    service.stop()
    assert (0., 0., 0., 0.) in sent
    assert service.expirations > 0
    assert service.send_duration_peak_s >= 0
    assert service.decision_to_send_peak_s > 0
    assert service.observation_to_send_peak_s > 0
    broken = CommandService(lambda *v: 1 / 0, period_s=.005)
    broken.start()
    time.sleep(.02)
    assert not broken.healthy()
    assert isinstance(broken.failure, ZeroDivisionError)
    broken.stop()
    slow = CommandService(lambda *_: time.sleep(.01), period_s=.005)
    slow.start()
    time.sleep(.04)
    slow.stop()
    assert slow.deadline_misses > 0


def test_noncritical_diagnostics_are_bounded_and_optional(tmp_path, monkeypatch):
    import threading
    import snapshot_service

    metrics = HealthMetrics()
    for _ in range(1000):
        metrics.observe("loop", .01)
    result = metrics.snapshot_and_reset()
    assert result["loop"]["count"] == 1000
    assert result["loop"]["max_ms"] == 10.
    assert metrics.snapshot_and_reset() == {}

    preview = PreviewService(enabled=False)
    preview.submit(np.zeros((2, 2, 3), dtype=np.uint8))
    assert not preview.escape_pressed()
    preview.stop()

    started = threading.Event()
    release = threading.Event()
    def slow_writer(*_args):
        started.set()
        release.wait(1.)
    monkeypatch.setattr(snapshot_service, "_write_snapshot", slow_writer)
    service = SnapshotService()
    s = captured_scan(time.monotonic())
    begun = time.monotonic()
    assert service.submit(tmp_path, s, {})
    assert time.monotonic() - begun < .1
    assert started.wait(1.)
    assert not service.submit(tmp_path, s, {})
    release.set()
    service.join(1.)
    assert service.error is None


def test_land_ack_alone_does_not_confirm_terminal_mode(monkeypatch):
    import experimental_corridor_manager as manager
    from pymavlink import mavutil

    now = time.monotonic()
    state = manager.Telemetry()
    state.mode = "GUIDED"
    state.heartbeat_time = now
    state.last_land_request = now
    state.ack_command = mavutil.mavlink.MAV_CMD_NAV_LAND
    state.ack_result = mavutil.mavlink.MAV_RESULT_ACCEPTED
    state.ack_time = now
    monkeypatch.setattr(manager, "telemetry", state)
    monkeypatch.setattr(manager, "drain_mavlink", lambda _master: None)
    assert not manager.confirm_terminal_land(object(), timeout_s=.06)
    state.mode = "LAND"
    state.heartbeat_time = time.monotonic()
    assert manager.confirm_terminal_land(object(), timeout_s=.06)


def test_blocked_console_cannot_hold_control_thread():
    import threading

    entered = threading.Event()
    release = threading.Event()
    class BlockedSink:
        def write(self, _line):
            entered.set()
            release.wait(1.)
        def flush(self):
            pass
    console = AsyncLogStream(BlockedSink(), max_lines=2)
    console.write("first\n")
    assert entered.wait(1.)
    begun = time.monotonic()
    for _ in range(100):
        console.write("diagnostic\n")
    assert time.monotonic() - begun < .1
    assert console.dropped_lines > 0
    release.set()
    console.stop(1.)


def test_direct_mavlink_send_does_not_deadlock_on_stalled_worker_lock():
    import threading
    from types import SimpleNamespace
    from unittest.mock import Mock
    import experimental_corridor_manager as manager

    entered = threading.Event()
    release = threading.Event()
    def hold_lock():
        with manager.mav_tx_guard(timeout_s=1.):
            entered.set()
            release.wait(1.)
    holder = threading.Thread(target=hold_lock)
    holder.start()
    assert entered.wait(1.)
    master = SimpleNamespace(target_system=1, target_component=1, mav=Mock())
    begun = time.monotonic()
    try:
        import pytest
        with pytest.raises(RuntimeError, match="transmit lock busy"):
            manager.send_stop(master)
        assert time.monotonic() - begun < .4
        master.mav.send.assert_not_called()
    finally:
        release.set()
        holder.join(1.)
