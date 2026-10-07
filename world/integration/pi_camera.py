"""Non-ROS IMX296 acquisition with exposure-time timestamps and bounded storage.

This module can be imported on a development PC; Picamera2 is imported only
when a physical camera is started. Images are OpenCV BGR at processing size.
"""

from collections import deque
import math
import threading
import time

import numpy as np


def exposure_monotonic(metadata, receipt_monotonic_ns=None,
                       boot_now_ns=None, mono_now_ns=None):
    """Map libcamera CLOCK_BOOTTIME exposure midpoint into host monotonic.

    The paired clock reads bound the conversion error to normal scheduling
    jitter. A missing or implausible sensor timestamp is never replaced with
    receipt time: doing that would corrupt image/vehicle-pose matching.
    """
    stamp = metadata.get("SensorTimestamp")
    if not isinstance(stamp, int) or stamp <= 0:
        raise ValueError("missing SensorTimestamp")
    exposure_us = metadata.get("ExposureTime")
    if not isinstance(exposure_us, (int, float)) or not math.isfinite(exposure_us) or exposure_us < 0:
        raise ValueError("missing or invalid ExposureTime")
    if boot_now_ns is None:
        boot_now_ns = time.clock_gettime_ns(time.CLOCK_BOOTTIME)
    if mono_now_ns is None:
        mono_now_ns = time.monotonic_ns()
    if receipt_monotonic_ns is None:
        receipt_monotonic_ns = mono_now_ns
    # Picamera2 reports start of *readout*, so exposure midpoint is earlier.
    midpoint = stamp - round(exposure_us * 500)
    mapped = midpoint + mono_now_ns - boot_now_ns
    age = (receipt_monotonic_ns - mapped) * 1e-9
    if not -0.01 <= age <= 2.0:
        raise ValueError(f"implausible exposure timestamp age: {age:.3f}s")
    return mapped * 1e-9


class PiCameraStream:
    """One dedicated capture thread; release each libcamera request promptly."""

    def __init__(self, camera_index, name, size=(640, 480), fps=15,
                 history=8, expected_model="imx296"):
        if camera_index < 0 or name not in ("front", "downward"):
            raise ValueError("explicit non-negative camera index and role required")
        if any(v <= 0 for v in size) or not 0 < fps <= 60 or history < 1:
            raise ValueError("invalid camera processing configuration")
        self.camera_index = camera_index
        self.name = name
        self.size = tuple(size)
        self.fps = fps
        self.expected_model = expected_model.lower()
        self.frames = deque(maxlen=history)
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.camera = None
        self.thread = None
        self.seq = 0
        self.error = None
        self.metadata = {}
        self.received = 0
        self.overwritten = 0
        self.processing_total_s = 0.0
        self.processing_peak_s = 0.0
        self.last_source = None

    def start(self):
        if self.camera is not None:
            raise RuntimeError("camera already started")
        try:
            from picamera2 import Picamera2
        except ImportError as exc:
            raise RuntimeError("Picamera2 is required on the Raspberry Pi") from exc
        info = Picamera2.global_camera_info()
        selected = info[self.camera_index] if self.camera_index < len(info) else None
        if selected is None:
            raise RuntimeError(f"camera index {self.camera_index} absent: {info}")
        if self.expected_model not in str(selected.get("Model", "")).lower():
            raise RuntimeError(f"{self.name} camera is not {self.expected_model}: {selected}")
        self.camera = Picamera2(self.camera_index)
        try:
            configuration = self.camera.create_video_configuration(
                main={"size": self.size, "format": "RGB888"},
                controls={"FrameRate": self.fps}, buffer_count=4, queue=False)
            self.camera.configure(configuration)
            self.camera.start()
            self.thread = threading.Thread(target=self._capture_loop,
                                           name=f"{self.name}-imx296", daemon=True)
            self.thread.start()
        except Exception:
            self.camera.close()
            self.camera = None
            raise
        return selected

    def _capture_loop(self):
        while not self.stop_event.is_set():
            request = None
            started = time.monotonic()
            try:
                request = self.camera.capture_request()
                metadata = request.get_metadata()
                receipt_ns = time.monotonic_ns()
                # RGB888 has B,G,R byte order in Picamera2's NumPy interface.
                frame = request.make_array("main")
                self.accept_frame(metadata, frame, receipt_ns)
                elapsed = time.monotonic() - started
                with self.lock:
                    self.processing_total_s += elapsed
                    self.processing_peak_s = max(self.processing_peak_s, elapsed)
            except Exception as exc:
                with self.lock:
                    self.error = str(exc)
                return
            finally:
                if request is not None:
                    request.release()

    def accept_frame(self, metadata, frame, receipt_ns=None):
        """Validate and publish one owned frame; also supports fake-camera tests."""
        receipt_ns = time.monotonic_ns() if receipt_ns is None else receipt_ns
        source_t = exposure_monotonic(metadata, receipt_ns)
        if (not isinstance(frame, np.ndarray) or frame.dtype != np.uint8
                or frame.shape != (self.size[1], self.size[0], 3)):
            raise ValueError(f"unexpected {self.name} image shape or format")
        # libcamera's make_array owns a copy; never retain a request-backed view.
        frame = np.ascontiguousarray(frame)
        with self.lock:
            if self.last_source is not None and source_t <= self.last_source:
                raise ValueError(f"non-increasing {self.name} sensor timestamp")
            self.last_source = source_t
            self.seq += 1
            self.received += 1
            if len(self.frames) == self.frames.maxlen:
                self.overwritten += 1
            self.frames.append((source_t, receipt_ns * 1e-9, frame, self.seq))
            self.metadata = {"ExposureTime": metadata.get("ExposureTime"),
                             "AnalogueGain": metadata.get("AnalogueGain"),
                             "ScalerCrop": str(metadata.get("ScalerCrop"))}

    def snapshot(self):
        with self.lock:
            return list(self.frames), self.error

    def health(self):
        with self.lock:
            latest = self.frames[-1] if self.frames else None
            return {"camera": self.name, "index": self.camera_index,
                    "received": self.received, "overwritten": self.overwritten,
                    "last_exposure_age_s": time.monotonic() - latest[0] if latest else None,
                    "last_receipt_age_s": time.monotonic() - latest[1] if latest else None,
                    "capture_cycle_mean_ms": 1000 * self.processing_total_s / self.received if self.received else None,
                    "capture_cycle_peak_ms": 1000 * self.processing_peak_s,
                    "metadata": dict(self.metadata), "error": self.error}

    def close(self):
        self.stop_event.set()
        if self.camera is not None:
            self.camera.stop()
        if self.thread is not None:
            self.thread.join(timeout=2)
        if self.camera is not None:
            self.camera.close()
            self.camera = None


class PiCoverageSensors:
    """Provide the existing coverage worker with real exposure/host clock data."""

    def __init__(self, downward_stream):
        if downward_stream.name != "downward":
            raise ValueError("coverage requires an explicitly mapped downward camera")
        self.stream = downward_stream

    def snapshot(self):
        frames, error = self.stream.snapshot()
        now = time.monotonic()
        return frames, (now, now), error
