"""Exit progress time must not weaken wall-time telemetry supervision."""
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from native.common.types import MissionState
from native.controllers.exit_detection import ExitConfig, ExitDetectionController


class ExitClockTest(unittest.TestCase):
    def setUp(self):
        self.wall = 100.0
        self.sim = 10.0
        self.mock_clock = patch(
            "native.controllers.exit_detection.time.monotonic",
            side_effect=lambda: self.wall,
        )
        self.mock_clock.start()
        self.addCleanup(self.mock_clock.stop)
        self.controller = ExitDetectionController(
            # Clock-only fixture: front safety has independent regressions.
            ExitConfig(use_front_safety=False, pose_fresh_s=1.0, pose_loss_timeout_s=6.0,
                       wall_timeout_s=180.0),
            progress_clock=lambda: self.sim,
        )
        self.controller.enter()
        self.step(0.0)

    def step(self, x):
        return self.controller.step(pose=SimpleNamespace(
            x_m=x, y_m=0.0, yaw_rad=0.0, timestamp=self.wall))

    def test_slow_sim_completes_by_distance(self):
        self.wall += 60.0
        self.sim += 8.0
        out = self.step(0.9)
        self.assertIsNone(out.next_state)
        self.assertGreater(out.command.vx_m_s, 0)
        self.sim += 3.0
        out = self.step(1.21)
        self.assertEqual(out.next_state, MissionState.CORRIDOR_EXITED)
        self.assertEqual(out.command.vx_m_s, 0)

    def test_sim_deadline(self):
        self.sim += 15.1
        out = self.step(0.5)
        self.assertEqual(out.next_state, MissionState.HOVER_AND_REASSESS)
        self.assertIn("progress timeout", out.reason)

    def test_frozen_sim_still_detects_pose_loss(self):
        self.assertEqual(self.controller.step(pose=None).command.vx_m_s, 0)
        self.wall += 6.1
        out = self.controller.step(pose=None)
        self.assertEqual(out.next_state, MissionState.HOVER_AND_REASSESS)
        self.assertIn("no fresh", out.reason)

    def test_brief_pose_loss_recovers(self):
        self.controller.step(pose=None)
        self.wall += 2.0
        self.assertGreater(self.step(0.2).command.vx_m_s, 0)
        self.assertIsNone(self.controller.pose_missing_since)

    def test_wall_cap_with_frozen_sim_and_fresh_pose(self):
        self.wall += 180.1
        out = self.step(0.2)
        self.assertEqual(out.next_state, MissionState.HOVER_AND_REASSESS)
        self.assertIn("wall-clock", out.reason)

    def test_sim_reset_stops(self):
        self.sim = 0.0
        out = self.step(0.2)
        self.assertEqual(out.next_state, MissionState.HOVER_AND_REASSESS)
        self.assertEqual(out.command.vx_m_s, 0)

    def test_default_clock_retains_real_time_deadline(self):
        controller = ExitDetectionController(ExitConfig(use_front_safety=False))
        controller.enter()
        self.wall += 15.1
        out = controller.step(pose=None)
        self.assertIn("progress timeout", out.reason)


if __name__ == "__main__":
    unittest.main()
