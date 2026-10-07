"""Constant-memory timing counters for later Pi/IMX296 benchmarking."""

import math
import threading


class HealthMetrics:
    def __init__(self):
        self._lock = threading.Lock()
        self._data = {}

    def observe(self, name, seconds):
        if not math.isfinite(seconds) or seconds < 0:
            return
        with self._lock:
            count, total, peak = self._data.get(name, (0, 0., 0.))
            self._data[name] = (count + 1, total + seconds, max(peak, seconds))

    def snapshot_and_reset(self):
        with self._lock:
            data, self._data = self._data, {}
        return {name: {"count": count, "mean_ms": 1000 * total / count,
                       "max_ms": 1000 * peak}
                for name, (count, total, peak) in data.items()}
