"""Bounded best-effort JSONL writer for the coverage flight supervisor.

Disk I/O and JSON encoding run outside the command loop. Losing diagnostics is
reported, but cannot hold or authorize vehicle motion.
"""

import json
from pathlib import Path
import queue
import threading


class AsyncJsonlWriter:
    def __init__(self, decision_path: Path, max_records=512):
        if max_records < 1:
            raise ValueError("max_records must be positive")
        self.decision_path = Path(decision_path)
        self.supervision_path = self.decision_path.with_suffix(".supervision.jsonl")
        self.qr_path = self.decision_path.with_suffix('.qr.jsonl')
        self._records = queue.Queue(maxsize=max_records)
        self._closing = threading.Event()
        self.dropped = 0
        self.failure = None
        self._thread = threading.Thread(target=self._run, name="coverage-jsonl",
                                        daemon=True)
        self._thread.start()

    def submit(self, channel, record):
        if channel not in ("decision", "supervision", "qr"):
            raise ValueError("unknown coverage log channel")
        if self._closing.is_set() or self.failure is not None:
            self.dropped += 1
            return
        try:
            self._records.put_nowait((channel, record))
        except queue.Full:
            # Preserve the most recent diagnostic state under backpressure.
            try:
                self._records.get_nowait()
            except queue.Empty:
                pass
            self.dropped += 1
            try:
                self._records.put_nowait((channel, record))
            except queue.Full:
                self.dropped += 1

    def _write(self, streams, channel, record):
        streams[channel].write(json.dumps(record, allow_nan=False) + "\n")

    def _run(self):
        try:
            self.decision_path.parent.mkdir(parents=True, exist_ok=True)
            with self.decision_path.open("w", buffering=65536) as decision, \
                    self.supervision_path.open("w", buffering=65536) as supervision, \
                    self.qr_path.open('w', buffering=65536) as qr:
                streams = {"decision": decision, "supervision": supervision,'qr':qr}
                while not self._closing.is_set() or not self._records.empty():
                    try:
                        channel, record = self._records.get(timeout=0.1)
                    except queue.Empty:
                        continue
                    self._write(streams, channel, record)
        except Exception as exc:
            self.failure = exc

    def close(self, timeout_s=0.0):
        self._closing.set()
        if timeout_s > 0:
            self._thread.join(timeout_s)
        return self.drained

    @property
    def drained(self):
        return not self._thread.is_alive()

    def __enter__(self):
        return self

    def __exit__(self, *_exc):
        self.close()  # Never wait for storage before terminal vehicle action.
