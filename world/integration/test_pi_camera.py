"""Hardware-independent checks for Pi camera timing and bounded frame handoff."""

import time
import unittest

import numpy as np

from approach.autonomy.perception.hybrid_banner_detector import HybridBannerDetector
from world.integration.pi_camera import PiCameraStream, PiCoverageSensors, exposure_monotonic


class PiCameraTests(unittest.TestCase):
    def test_exposure_midpoint_and_boottime_conversion(self):
        # BOOTTIME is 5 seconds ahead of the monotonic epoch in this fixture.
        stamp = 20_000_000_000
        metadata = {'SensorTimestamp': stamp, 'ExposureTime': 20_000}
        result = exposure_monotonic(metadata, receipt_monotonic_ns=15_050_000_000,
                                    boot_now_ns=20_040_000_000,
                                    mono_now_ns=15_040_000_000)
        self.assertAlmostEqual(result, 14.99)

    def test_missing_and_future_timestamps_fail_closed(self):
        with self.assertRaisesRegex(ValueError, 'SensorTimestamp'):
            exposure_monotonic({'ExposureTime': 1000}, 10, 10, 10)
        with self.assertRaisesRegex(ValueError, 'implausible'):
            exposure_monotonic({'SensorTimestamp': 3_000_000_000, 'ExposureTime': 1000},
                               1_000_000_000, 1_000_000_000, 1_000_000_000)

    def test_downward_snapshots_bounded_and_owned(self):
        stream = PiCameraStream(1, 'downward', size=(4, 3), history=2)
        for offset_ms in (30, 20, 10):
            stamp = time.clock_gettime_ns(time.CLOCK_BOOTTIME) - offset_ms * 1_000_000
            frame = np.zeros((3, 4, 3), np.uint8)
            stream.accept_frame({'SensorTimestamp': stamp, 'ExposureTime': 1000}, frame)
        frames, error = stream.snapshot()
        self.assertIsNone(error)
        self.assertEqual(len(frames), 2)
        self.assertEqual([frame[3] for frame in frames], [2, 3])
        self.assertEqual(stream.health()['overwritten'], 1)
        coverage_frames, clock, coverage_error = PiCoverageSensors(stream).snapshot()
        self.assertEqual(coverage_frames, frames)
        self.assertLess(abs(clock[0] - clock[1]), .001)
        self.assertIsNone(coverage_error)

    def test_wrong_shape_or_role_rejected(self):
        stream = PiCameraStream(0, 'front', size=(4, 3))
        with self.assertRaisesRegex(ValueError, 'downward'):
            PiCoverageSensors(stream)
        with self.assertRaisesRegex(ValueError, 'shape'):
            stream.accept_frame({'SensorTimestamp': time.clock_gettime_ns(time.CLOCK_BOOTTIME)-10_000_000,
                                 'ExposureTime': 1000}, np.zeros((4, 3, 3), np.uint8))

    def test_headless_banner_detection_keeps_decision(self):
        frame = np.zeros((480, 640, 3), np.uint8)
        frame[150:300, 220:400] = (0, 255, 0)
        full = HybridBannerDetector().detect(frame)
        lean = HybridBannerDetector().detect(frame, debug=False)
        self.assertEqual(lean['detected'], full['detected'])
        self.assertEqual(lean['bbox'], full['bbox'])
        self.assertEqual(lean['error_x'], full['error_x'])
        self.assertIsNone(lean['debug_frame'])


if __name__ == '__main__':
    unittest.main()
