"""Optional, lossy Gazebo preview isolated from the mission control loop.

Frames are latest-only. A stopped or wedged GUI must never block navigation.
The ESC event remains an operator abort request, not a flight-safety watchdog.
"""

import multiprocessing as mp
from queue import Full


def _run_preview(frames, escape, title):
    import cv2

    try:
        cv2.namedWindow(title, cv2.WINDOW_NORMAL)
        while True:
            frame = frames.get()
            if frame is None:
                break
            cv2.imshow(title, frame)
            if cv2.waitKey(1) & 0xFF == 27:
                escape.set()
                break
    finally:
        cv2.destroyAllWindows()


class PreviewService:
    def __init__(self, enabled=True, title="Experimental Corridor Camera"):
        self.enabled = bool(enabled)
        self.dropped = 0
        self._process = None
        self.title=title
        self.start(title)

    def start(self,title=None):
        if self.enabled:
            if self.alive: return
            if self._process is not None: self.stop()
            self.title=title or self.title
            ctx = mp.get_context("spawn")
            self._frames = ctx.Queue(maxsize=1)
            self._escape = ctx.Event()
            self._process = ctx.Process(
                target=_run_preview, args=(self._frames, self._escape, self.title),
                name="corridor-preview", daemon=True)
            self._process.start()

    def submit(self, frame):
        if self._process is None or not self._process.is_alive():
            return
        try:
            self._frames.put_nowait(frame)
        except Full:
            self.dropped += 1

    def escape_pressed(self):
        return self._process is not None and self._escape.is_set()

    @property
    def alive(self):
        return self._process is not None and self._process.is_alive()

    @property
    def exitcode(self):
        return self._process.exitcode if self._process is not None else None

    def stop(self):
        if self._process is None:
            return
        try:
            self._frames.put_nowait(None)
        except Full:
            pass
        self._process.join(timeout=0.5)
        if self._process.is_alive():
            self._process.terminate()
            self._process.join(timeout=0.5)
        self._frames.close()
        self._process = None
