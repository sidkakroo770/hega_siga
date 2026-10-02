import math
import unittest
from corridor_altitude import AltitudeController


class AltitudeTests(unittest.TestCase):
    def test_descent_ascent_and_speed_limit(self):
        c = AltitudeController()
        self.assertEqual(c.update(0, -3, 0, 0).vz_down, 0.30)
        self.assertEqual(c.update(0, -0.5, 0, 0).vz_down, -0.30)

    def test_requires_continuous_dwell_and_low_vertical_speed(self):
        c = AltitudeController()
        self.assertFalse(c.update(0, -1.3, 0.2, 0).ready)
        self.assertFalse(c.update(1, -1.3, 0, 1).ready)
        self.assertFalse(c.update(1.5, -1.6, 0, 1.5).ready)
        self.assertFalse(c.update(2, -1.3, 0, 2).ready)
        self.assertTrue(c.update(3, -1.3, 0, 3).ready)
        self.assertFalse(c.update(4, -1.8, 0, 4).ready)

    def test_missing_stale_nonfinite_stop_and_abort(self):
        for z, vz, stamp in [(None, 0, 1), (-1.3, 0, 0),
                             (math.nan, 0, 1), (-1.3, math.inf, 1)]:
            c = AltitudeController()
            r = c.update(1, z, vz, stamp)
            self.assertEqual(r.vz_down, 0)
            self.assertFalse(r.ready)
            self.assertTrue(c.update(3, z, vz, stamp).error)

    def test_timeout(self):
        c = AltitudeController()
        self.assertTrue(c.update(21, -3, 0, 21).error)

    def test_slow_telemetry_does_not_pulse_stop(self):
        c = AltitudeController(target=2.57, telemetry_max_age=1.5)
        for t in (0, 0.5, 0.8, 1.0):
            self.assertGreater(c.update(t, -3.57, 0.1, 0).vz_down, 0)
        r = c.update(1.51, -3.57, 0.1, 0)
        self.assertEqual(r.vz_down, 0)
        self.assertFalse(r.ready)
        self.assertTrue(c.update(3.6, -3.57, 0.1, 0).error)

    def test_repeated_sample_cannot_complete_dwell(self):
        c = AltitudeController(telemetry_max_age=1.5)
        self.assertFalse(c.update(0, -1.3, 0, 0).ready)
        self.assertFalse(c.update(1.2, -1.3, 0, 0).ready)
        self.assertTrue(c.update(1.3, -1.3, 0, 1.3).ready)

    def test_relative_descent_with_one_hz_telemetry(self):
        c = AltitudeController(target=2.57, tolerance=0.10, dwell=1.5,
                               timeout=25, telemetry_max_age=1.5)
        z, vz = -3.57, 0.0
        for step in range(500):
            now = step * 0.05
            if step % 20 == 0:
                sample_z, sample_vz, timestamp = z, vz, now
            r = c.update(now, sample_z, sample_vz, timestamp)
            self.assertFalse(r.error)
            vz += (r.vz_down - vz) * 0.15
            z += vz * 0.05
            if r.ready:
                break
        self.assertTrue(r.ready)
        self.assertAlmostEqual(z, -2.57, delta=0.10)

    def test_closed_loop_descent_and_disturbance_recovery(self):
        c = AltitudeController()
        z, vz = -3.0, 0.0
        for step in range(400):
            now = step * 0.05
            r = c.update(now, z, vz, now)
            self.assertFalse(r.error)
            vz += (r.vz_down - vz) * 0.15
            z += vz * 0.05
        self.assertTrue(r.ready)
        self.assertAlmostEqual(-z, 1.3, delta=0.12)
        self.assertFalse(c.update(20, z - 0.5, 0, 20).ready)

    def test_reject_invalid_config(self):
        for kwargs in [dict(target=math.nan), dict(max_speed=-1),
                       dict(tolerance=2), dict(timeout=0.5)]:
            with self.assertRaises(ValueError):
                AltitudeController(**kwargs)

    def test_slow_simulation_uses_autopilot_time(self):
        c = AltitudeController(target=2.57, tolerance=.10, dwell=1.5,
                               timeout=25, telemetry_max_age=1.5)
        z, vz = -3.57, 0.0
        for step in range(3000):
            wall = step * .05
            sim = wall * .15
            r = c.update(wall, z, vz, wall, mission_time=sim)
            self.assertFalse(r.error)
            vz += (r.vz_down - vz) * .04
            z += vz * .0075
            if r.ready:
                break
        self.assertTrue(r.ready)
        self.assertGreater(wall, 25)
        self.assertLess(sim, 25)
        self.assertAlmostEqual(z, -2.57, delta=.10)

    def test_sim_clock_does_not_bypass_safety(self):
        c = AltitudeController()
        c.update(0, -3, 0, 0, mission_time=10)
        self.assertTrue(c.update(1, -3, 0, 1, mission_time=9).error)
        c = AltitudeController()
        c.update(0, -3, 0, 0, mission_time=10)
        self.assertTrue(c.update(181, -3, 0, 181, mission_time=10).error)


if __name__ == '__main__':
    unittest.main()
