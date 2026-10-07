"""Offline safety regressions. No device connections or vehicle commands."""
import math
import time
import unittest
from unittest.mock import patch

import numpy as np

from native.common.types import Attitude, BodyVelocity, MissionState, NativeScan, VehicleAction
from native.common.validity import attitude_valid, pose_valid, scan_valid, sector_clearance
from native.controllers.pre_entry import PreEntryController, CorridorGeometry, LockState
from native.controllers.corridor_cruise import CorridorCruiseController
from native.controllers.obstacle_avoidance import (
    ObstacleAvoidanceController, ObstacleObservation, ObstacleState, AvoidPhase)
from native.controllers.exit_detection import ExitDetectionController, ExitConfig
from native.hardware.mavlink_commands import body_velocity_to_mavlink, vehicle_action_to_mavlink
from native.mission_runner import NativeMissionRunner, MissionRunnerConfig, VehiclePose


def scan(distance=5.0, trusted=False):
    angles = np.linspace(-math.pi, math.pi, 500)
    return NativeScan(angles, np.full(500, distance), np.zeros(500),
                      time.monotonic(), no_return_is_clear=trusted)


def attitude():
    return Attitude(timestamp=time.monotonic())


class ValidityTests(unittest.TestCase):
    def test_finite_pose_and_attitude(self):
        now = time.monotonic()
        self.assertTrue(pose_valid(VehiclePose(1, 2, 0, now), .5))
        for value in (math.nan, math.inf, -math.inf):
            self.assertFalse(pose_valid(VehiclePose(value, 2, 0, now), .5))
            self.assertFalse(attitude_valid(Attitude(roll_rad=value, timestamp=now)))
        self.assertFalse(attitude_valid(None))
        self.assertFalse(attitude_valid(Attitude(timestamp=now-1)))

    def test_future_and_stale_scans_rejected(self):
        s = scan()
        s.timestamp = time.monotonic()+10
        self.assertFalse(scan_valid(s))
        s.timestamp = time.monotonic()-1
        self.assertIsNone(sector_clearance(s))

    def test_malformed_arrays_rejected(self):
        s = scan()
        s.angles_rad = s.angles_rad[:-1]
        self.assertFalse(scan_valid(s))

    def test_unknown_is_not_free(self):
        for value in (math.nan, math.inf, -math.inf, 0., 13.):
            self.assertIsNone(sector_clearance(scan(value)))

    def test_only_explicit_adapter_trusts_infinity(self):
        self.assertEqual(sector_clearance(scan(math.inf, True)), 12.)
        self.assertIsNone(sector_clearance(scan(math.nan, True)))

    def test_sector_hole_is_unknown(self):
        s = scan()
        s.ranges_m[np.abs(s.angles_rad) < math.radians(5)] = math.nan
        self.assertIsNone(sector_clearance(s))

    def test_narrow_close_return_is_not_percentile_trimmed(self):
        s = scan()
        s.ranges_m[250] = .2
        self.assertEqual(sector_clearance(s), .2)

    def test_exit_diagnosis_uses_supported_forward_opening(self):
        """At the captured exit, 90-degree walls remain while both 45s are open."""
        s = scan(12.)
        for bearing in (-90, 90):
            s.ranges_m[np.abs(np.degrees(s.angles_rad) - bearing) < 8] = 1.9
        controller = CorridorCruiseController()
        geometry = CorridorGeometry(strict_valid=False, loose_valid=True,
                                    confidence=.85, front_clearance=12.)
        with patch.object(PreEntryController, "extract_corridor_geometry",
                          return_value=geometry), patch.object(controller, "session_age",
                                                                return_value=3.):
            result = controller.extract_cruise_geometry(s)
        self.assertTrue(result.exit_candidate)
        self.assertFalse(result.side_open_left)
        self.assertFalse(result.side_open_right)
        s.ranges_m[np.abs(np.degrees(s.angles_rad) - 45) < 8] = math.nan
        with patch.object(PreEntryController, "extract_corridor_geometry",
                          return_value=geometry), patch.object(controller, "session_age",
                                                                return_value=3.):
            result = controller.extract_cruise_geometry(s)
        self.assertFalse(result.exit_candidate)


class ControllerSafetyTests(unittest.TestCase):
    def test_verify_dead_band_restarts_centering(self):
        c = PreEntryController(); c.enter()
        c.state = LockState.VERIFY_LOCK
        c.alignment_start_time = time.monotonic()
        g = CorridorGeometry(strict_valid=True, loose_valid=True, confidence=.9,
                             front_clearance=5., lateral_error=.15)
        c.step_fsm(g, attitude())
        self.assertEqual(c.state, LockState.CENTER_LATERALLY)

    def test_verification_timeout(self):
        c = PreEntryController(); c.enter()
        c.state = LockState.VERIFY_LOCK
        c.alignment_start_time = time.monotonic()-600
        c.step_fsm(CorridorGeometry(front_clearance=5.), attitude())
        self.assertEqual(c.failure_reason, "ENTRY_VERIFICATION_TIMEOUT")

    def test_missing_attitude_not_safe(self):
        for c in (PreEntryController(), CorridorCruiseController(), ObstacleAvoidanceController()):
            self.assertFalse(c.attitude_is_safe(None))
            self.assertFalse(c.attitude_is_safe(Attitude(timestamp=time.monotonic()-2)))

    def test_missing_side_sectors_not_exit(self):
        c = CorridorCruiseController(); c.enter()
        c.session_start_time = time.monotonic()-10
        s = scan(math.nan)
        s.ranges_m[np.abs(s.angles_rad) < math.radians(36)] = 5.
        for _ in range(3):
            out = c.step(s, attitude())
        self.assertNotEqual(out.next_state, MissionState.EXIT_DETECTION)
        self.assertEqual(out.command.vx_m_s, 0)

    def test_observed_open_sides_remain_exit_candidate(self):
        c = CorridorCruiseController(); c.enter()
        c.session_start_time = time.monotonic()-10
        s = scan(math.inf, True)
        for _ in range(3):
            out = c.step(s, attitude())
        self.assertEqual(out.next_state, MissionState.EXIT_DETECTION)

    def test_pass_unknown_front_stops(self):
        c = ObstacleAvoidanceController(); c.enter()
        c.state = ObstacleState.AVOID_RIGHT; c.phase = AvoidPhase.PASS
        c.target_outer_clearance_m = 1.2; c.latched_obstacle_side = "LEFT"
        c.state_enter_time = time.monotonic()
        c.step_avoid(ObstacleObservation(right_wall_valid=True, d_right_m=1.2,
                                        right_wall_yaw_rad=0, front_clearance_m=0))
        self.assertEqual(c.latest_command.vx_m_s, 0)
        self.assertEqual(c.transition_target, MissionState.HOVER_AND_REASSESS)

    def test_exit_requires_valid_front(self):
        for s in (None, scan(math.nan), scan(math.inf)):
            c = ExitDetectionController(); c.enter()
            out = c.step(s, VehiclePose(0, 0, 0, time.monotonic()))
            self.assertEqual(out.command.vx_m_s, 0)
            self.assertEqual(out.next_state, MissionState.HOVER_AND_REASSESS)

    def test_exit_rejects_nonfinite_pose(self):
        c = ExitDetectionController(); c.enter()
        out = c.step(scan(), VehiclePose(math.nan, 0, 0, time.monotonic()))
        self.assertEqual(out.command.vx_m_s, 0)
        self.assertIsNone(c.start_x_m)

    def test_exit_stops_first_close_scan(self):
        c = ExitDetectionController(); c.enter()
        out = c.step(scan(.2), VehiclePose(0, 0, 0, time.monotonic()))
        self.assertEqual(out.command.vx_m_s, 0)
        self.assertEqual(out.next_state, MissionState.HOVER_AND_REASSESS)

    def test_hard_timeout_includes_land(self):
        c = NativeMissionRunner()
        c.start_reassess(MissionState.CORRIDOR_CRUISE, "test")
        c.state_enter_time = time.monotonic()-100
        self.assertEqual(c.step(None).action, VehicleAction.LAND)

    def test_missing_sensor_ticks_eventually_abort(self):
        c = NativeMissionRunner()
        self.assertEqual(c.step(None).command.vx_m_s, 0)
        c.reassess.state_enter_time = time.monotonic()-100
        out = c.step(None)
        self.assertEqual(c.state, MissionState.ABORT_CORRIDOR)
        self.assertEqual(out.action, VehicleAction.LAND)

    def test_enter_does_not_ignore_obstacle(self):
        c = NativeMissionRunner(MissionRunnerConfig(enter_corridor_distance_m=.75))
        c.transition(MissionState.ENTER_CORRIDOR, "test")
        out = c.step(scan(.2), attitude(), VehiclePose(0, 0, 0, time.monotonic()))
        self.assertEqual(out.command.vx_m_s, 0)
        self.assertEqual(c.state, MissionState.HOVER_AND_REASSESS)

    def test_enter_with_supported_geometry_moves(self):
        c = NativeMissionRunner(MissionRunnerConfig(enter_corridor_distance_m=.75))
        c.transition(MissionState.ENTER_CORRIDOR, "test")
        g = CorridorGeometry(strict_valid=True, confidence=.9, front_clearance=5.)
        with patch.object(c.pre_entry, 'extract_corridor_geometry', return_value=g):
            out = c.step(scan(), attitude(), VehiclePose(0, 0, 0, time.monotonic()))
        self.assertEqual(out.command.vx_m_s, .2)

    def test_enter_pose_path_cannot_credit_sideways_displacement(self):
        c = NativeMissionRunner(MissionRunnerConfig(enter_corridor_distance_m=.75))
        c.transition(MissionState.ENTER_CORRIDOR, "test")
        now = time.monotonic()
        c.step_enter_corridor(VehiclePose(0, 0, 0, now))
        out = c.step_enter_corridor(VehiclePose(.8, 1., 0, now))
        self.assertEqual(out.command.vx_m_s, 0)
        self.assertEqual(c.state, MissionState.ABORT_CORRIDOR)
        self.assertIn("cross-track", c.terminal_reason)

    def test_exit_pose_path_rejects_cross_track_and_yaw(self):
        for pose in (VehiclePose(.5, 1., 0, time.monotonic()),
                     VehiclePose(.5, 0, math.radians(65), time.monotonic())):
            c = ExitDetectionController(); c.enter()
            c.step(scan(), VehiclePose(0, 0, 0, time.monotonic()))
            out = c.step(scan(), pose)
            self.assertEqual(out.command.vx_m_s, 0)
            self.assertEqual(out.next_state, MissionState.HOVER_AND_REASSESS)

    def test_provisional_vehicle_envelope_has_single_runner_source(self):
        c = NativeMissionRunner(MissionRunnerConfig(vehicle_width_m=.71,
                                                    passage_side_margin_m=.31))
        self.assertEqual(c.obstacle.config.vehicle_width_m, .71)
        self.assertEqual(c.reassess.config.vehicle_width_m, .71)
        self.assertEqual(c.obstacle.config.passage_side_margin_m, .31)
        self.assertEqual(c.reassess.config.passage_side_margin_m, .31)
        with self.assertRaises(ValueError):
            MissionRunnerConfig(vehicle_width_m=math.nan)
        with self.assertRaises(ValueError):
            ExitConfig(max_yaw_error_deg=0)


class CommandPlanTests(unittest.TestCase):
    def test_flu_conversion_and_mask(self):
        p = body_velocity_to_mavlink(BodyVelocity(.1, .2, .3, .4))
        self.assertEqual((p.vx_m_s, p.vy_m_s, p.vz_m_s, p.yaw_rate_rad_s), (.1, -.2, -.3, -.4))
        self.assertEqual((p.coordinate_frame, p.type_mask), (8, 1479))
        self.assertEqual(vehicle_action_to_mavlink(VehicleAction.LAND).command, 21)

    def test_invalid_command_rejected(self):
        with self.assertRaises(ValueError):
            body_velocity_to_mavlink(BodyVelocity(vx_m_s=math.nan))
        with self.assertRaises(ValueError):
            vehicle_action_to_mavlink('ARM')


if __name__ == '__main__':
    unittest.main()
