"""Synthetic serial assembly tests, with no physical serial port."""
import math
import unittest
from unittest.mock import patch, Mock

from native.hardware.d500_driver import D500Driver, LidarPoint
from native.common.scan_adapter import ScanAdapter


def points(angles=range(360)):
    return [LidarPoint(float(a), 2., 100) for a in angles]


class AssemblyTests(unittest.TestCase):
    def test_complete_scan_preserves_start_receipt_and_sensor_clock(self):
        driver = D500Driver()
        driver.assembly_started = 10.
        driver.sensor_timestamp_ms = 123
        with patch("native.hardware.d500_driver.time.monotonic", return_value=10.1):
            scan = driver._build_scan(points())
        self.assertEqual(scan.stamp_monotonic, 10.)
        self.assertAlmostEqual(scan.scan_time_s, .1)
        self.assertEqual(scan.sensor_timestamp_ms, 123)
        corrected = ScanAdapter().convert(scan)
        self.assertEqual(corrected.source_timestamp, 123)
        self.assertEqual(corrected.source_clock, "d500_device_ms")
        self.assertAlmostEqual(corrected.scan_duration_s, .1)

    def test_hole_rejected_despite_many_points(self):
        driver = D500Driver()
        driver.assembly_started = 10.
        with patch("native.hardware.d500_driver.time.monotonic", return_value=10.1):
            self.assertIsNone(driver._build_scan(points(range(20, 360))))
        self.assertEqual(driver.rejected_scans, 1)

    def test_expired_assembly_cannot_be_refreshed(self):
        driver = D500Driver()
        driver.assembly_started = 10.
        with patch("native.hardware.d500_driver.time.monotonic", return_value=11.):
            self.assertIsNone(driver._build_scan(points()))

    def test_missing_wrap_cannot_grow_forever(self):
        driver = D500Driver(max_scan_points=100)
        with patch("native.hardware.d500_driver.time.monotonic", return_value=10.):
            for _ in range(20):
                driver._consume_points(points(range(20)))
                self.assertLessEqual(len(driver.current_scan_points), 100)
        self.assertGreater(driver.rejected_scans, 0)
        self.assertFalse(driver.scan_queue)

    def test_initial_partial_revolution_is_discarded(self):
        driver = D500Driver()
        with patch("native.hardware.d500_driver.time.monotonic", return_value=10.):
            driver._consume_points(points(range(200, 360)))
            driver._consume_points(points())
            self.assertFalse(driver.scan_queue)
            driver._consume_points(points(range(12)))
        self.assertEqual(len(driver.scan_queue), 1)

    def test_backlog_is_dropped_not_retimestamped(self):
        driver = D500Driver()
        driver.serial = Mock(in_waiting=5000)
        with patch("native.hardware.d500_driver.time.monotonic", side_effect=[10., 12.]):
            self.assertIsNone(driver._read_packet(11.))
        driver.serial.reset_input_buffer.assert_called_once()
        self.assertEqual(driver.backlog_resets, 1)

    def test_invalid_configuration(self):
        for value in (0, -1, math.inf, math.nan):
            with self.assertRaises(ValueError):
                D500Driver(max_assembly_s=value)


if __name__ == "__main__":
    unittest.main()
