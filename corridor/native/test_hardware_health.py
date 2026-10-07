"""Fake telemetry only: never opens a port or arms a vehicle."""
import math
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from native.common.banner_guard import BannerGuard, BannerGuardConfig
from native.common.types import Attitude, BodyVelocity, VehicleAction
from native.hardware.mavlink_io import MavlinkIO, LocalPositionNED, MavlinkStatus
from native.hardware.mavlink_sender import MavlinkCommandSender


class HealthTests(unittest.TestCase):
    def setUp(self):
        self.fc = MavlinkIO("unused")
        self.fc.master = SimpleNamespace(target_system=1, target_component=1, mav=Mock())
        self.now = time.monotonic()
        self.fc._local_position = LocalPositionNED(0, 0, -2, 0, 0, 0, self.now)
        self.fc._attitude = Attitude(timestamp=self.now)
        self.fc._ekf_timestamp = self.now
        self.fc._status = MavlinkStatus(connected=True, armed=True, mode="GUIDED",
                                      ekf_flags=8, heartbeat_timestamp=self.now)

    def message(self, kind, system=1, component=1, **fields):
        return SimpleNamespace(get_type=lambda: kind, get_srcSystem=lambda: system,
                               get_srcComponent=lambda: component, **fields)

    def test_connect_uses_heartbeat_source_not_broadcast_component(self):
        fc = MavlinkIO("unused")
        master = Mock(target_system=1, target_component=0)
        master.wait_heartbeat.return_value = self.message("HEARTBEAT", base_mode=0)
        with patch("native.hardware.mavlink_io.mavutil.mavlink_connection",
                   return_value=master), patch.object(fc, "_request_telemetry"), \
                patch.object(fc, "_reader_loop"), \
                patch("native.hardware.mavlink_io.mavutil.mode_string_v10",
                      return_value="STABILIZE"):
            fc.connect()
            self.assertEqual(master.target_component, 1)
            fc.close()

    def test_foreign_telemetry_cannot_refresh_health(self):
        for system, component in ((255, 1), (1, 190)):
            self.fc._handle_message(self.message("EKF_STATUS_REPORT", system, component, flags=0))
            self.assertEqual(self.fc.status().ekf_flags, 8)

    def test_ekf_requires_recent_report(self):
        self.assertTrue(self.fc.horizontal_position_ok())
        self.fc._ekf_timestamp = self.now - 2
        self.assertFalse(self.fc.horizontal_position_ok())
        self.fc._handle_message(self.message("EKF_STATUS_REPORT", flags=8))
        self.assertTrue(self.fc.horizontal_position_ok())

    def test_finite_position_required(self):
        self.fc._local_position = LocalPositionNED(math.nan, 0, 0, 0, 0, 0, self.now)
        self.assertFalse(self.fc.horizontal_position_ok())

    def test_mode_takeover_is_latched(self):
        heartbeat = self.message("HEARTBEAT", base_mode=128)
        with patch("native.hardware.mavlink_io.mavutil.mode_string_v10",
                   side_effect=["GUIDED", "LOITER", "GUIDED"]):
            for _ in range(3):
                self.fc._handle_message(heartbeat)
        self.assertTrue(self.fc.authority_revoked())
        sender = MavlinkCommandSender(self.fc)
        sender._control_enabled = True
        self.assertTrue(sender.send_velocity(BodyVelocity()).blocked)
        self.assertTrue(sender.send_action(VehicleAction.LAND).blocked)
        self.fc.master.mav.command_long_send.assert_not_called()

    def test_sender_requires_arm_and_attitude(self):
        sender = MavlinkCommandSender(self.fc)
        sender._control_enabled = True
        self.assertTrue(sender.send_velocity(BodyVelocity()).transmitted)
        self.fc._attitude = None
        self.assertTrue(sender.send_velocity(BodyVelocity()).blocked)
        self.fc._attitude = Attitude(timestamp=self.now)
        self.fc._status = MavlinkStatus(connected=True, mode="GUIDED", heartbeat_timestamp=self.now)
        self.assertTrue(sender.send_velocity(BodyVelocity()).blocked)


class BannerTests(unittest.TestCase):
    def test_freeze_stops_then_aborts(self):
        guard = BannerGuard(10)
        self.assertEqual(guard.check(10, 10, (0, 0, 0), 10, True), (True, None))
        self.assertEqual(guard.check(11, 10, (0, 0, 0), 11, True), (False, None))
        self.assertIsNotNone(guard.check(13, 10, (0, 0, 0), 13, True)[1])
        self.assertFalse(guard.check(14, 14, (0, 0, 0), 14, True)[0])

    def test_deadline_not_reset_by_reacquisition(self):
        guard = BannerGuard(10, BannerGuardConfig(timeout_s=5))
        guard.check(10, 10, (0, 0, 0), 10, True)
        guard.check(11, 10, (0, 0, 0), 11, True)
        self.assertTrue(guard.check(12, 12, (0, 0, 0), 12, True)[0])
        self.assertIsNotNone(guard.check(15, 15, (0, 0, 0), 15, True)[1])

    def test_travel_and_pose_health(self):
        guard = BannerGuard(10)
        self.assertFalse(guard.check(10, 10, None, 10, True)[0])
        self.assertTrue(guard.check(11, 11, (0, 0, 0), 11, True)[0])
        self.assertFalse(guard.check(12, 12, (10, 0, 0), 12, True)[0])

    def test_configuration_validation(self):
        for v in (0, -1, math.inf, math.nan):
            with self.assertRaises(ValueError):
                BannerGuardConfig(timeout_s=v)


if __name__ == "__main__":
    unittest.main()
