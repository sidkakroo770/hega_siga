"""One-shot PRE_ENTRY capture that cannot block flight-critical control."""

import json
import threading

import numpy as np


def _write_snapshot(output_dir, angles, ranges, range_min, range_max, details):
    output_dir.mkdir(parents=True, exist_ok=True)
    np.savez(output_dir / "preentry_scan.npz", angles_rad=angles,
             ranges_m=ranges, range_min_m=range_min, range_max_m=range_max)
    (output_dir / "preentry_geometry.json").write_text(
        json.dumps(details, indent=2))


class SnapshotService:
    def __init__(self):
        self._thread = None
        self.error = None

    def submit(self, output_dir, scan, details):
        if self._thread is not None:
            return False
        # Own the arrays before returning; the sensor callback may replace its
        # cache while the background writer is still active.
        args = (output_dir, scan.angles_rad.copy(), scan.ranges_m.copy(),
                float(scan.range_min_m), float(scan.range_max_m), details)

        def run():
            try:
                _write_snapshot(*args)
            except Exception as exc:
                self.error = exc

        self._thread = threading.Thread(target=run, name="preentry-snapshot",
                                        daemon=True)
        self._thread.start()
        return True

    def join(self, timeout_s=0.2):
        if self._thread is not None:
            self._thread.join(timeout_s)
