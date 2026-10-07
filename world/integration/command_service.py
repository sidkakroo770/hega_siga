"""Single corridor-stage MAVLink velocity owner with a short command lease.

The worker never performs GUI, disk or perception work. The FC's own link and
mode failsafes remain necessary for process failure or a broken MAVLink link.
"""
import math
import threading
import time


class CommandService:
    def __init__(self, transmit, period_s=0.05, lease_s=0.30):
        if not all(math.isfinite(v) and v > 0 for v in (period_s, lease_s)):
            raise ValueError("Invalid command service timing")
        self.transmit = transmit
        self.period_s = period_s
        self.lease_s = lease_s
        self._lock = threading.Lock()
        self._running = False
        self._thread = None
        self._command = (0.0, 0.0, 0.0, 0.0)
        self._updated = 0.0
        self._source_timestamp = None
        self._frame = "body"
        self._generation = 0
        self.failure = None
        self.last_send = 0.0
        self.started = 0.0
        self.expirations = 0
        self.send_duration_peak_s = 0.0
        self.deadline_misses = 0
        self.decision_to_send_peak_s = 0.0
        self.observation_to_send_peak_s = 0.0

    def start(self):
        with self._lock:
            if self._running:
                return
            self._running = True
            self.started = time.monotonic()
        self._thread = threading.Thread(target=self._run, name="mission-command", daemon=True)
        self._thread.start()

    def claim(self):
        """Revoke all earlier stage publishers and immediately command zero."""
        with self._lock:
            if not self._running or self.failure is not None:
                raise RuntimeError('Command worker unavailable')
            self._generation += 1
            self._command = (0.0, 0.0, 0.0, 0.0)
            self._frame = 'body'
            self._updated = 0.0
            return self._generation

    def publish(self, vx, vy, vz, yaw_rate=0.0, source_timestamp=None,
                token=None, frame='body'):
        command = (float(vx), float(vy), float(vz), float(yaw_rate))
        if not all(math.isfinite(v) for v in command):
            raise ValueError("Nonfinite velocity command")
        if (source_timestamp is not None
                and (not math.isfinite(source_timestamp) or source_timestamp <= 0)):
            raise ValueError("Invalid observation receipt timestamp")
        if frame not in ('body', 'local'):
            raise ValueError('Unsupported command frame')
        with self._lock:
            if not self._running or self.failure is not None:
                raise RuntimeError("Command worker unavailable")
            if token != (self._generation if self._generation else None):
                raise RuntimeError('Stale mission-stage command authority')
            self._command = command
            self._frame = frame
            self._updated = time.monotonic()
            self._source_timestamp = source_timestamp

    def healthy(self):
        return (self._thread is not None and self._thread.is_alive()
                and self.failure is None
                and time.monotonic() - (self.last_send or self.started)
                <= max(0.5, 4*self.period_s))

    def _run(self):
        try:
            next_send = time.monotonic()
            while True:
                with self._lock:
                    if not self._running:
                        return
                    command = self._command
                    frame = self._frame
                    generation = self._generation
                    updated = self._updated
                    source_timestamp = self._source_timestamp
                now = time.monotonic()
                if now - updated > self.lease_s:
                    if command[:3] != (0.0, 0.0, 0.0):
                        self.expirations += 1
                    command = (0.0, 0.0, 0.0,
                               command[3] if frame == 'local' else 0.0)
                # Keep claim/stop serialized with the actual send. Otherwise a
                # corridor packet already copied by this thread could be sent
                # *after* coverage has claimed the vehicle.
                with self._lock:
                    if not self._running:
                        return
                    if generation != self._generation:
                        continue
                    send_started = time.monotonic()
                    if frame == 'local':
                        self.transmit(*command, frame='local')
                    else:
                        self.transmit(*command)
                    self.last_send = time.monotonic()
                duration = self.last_send - send_started
                self.send_duration_peak_s = max(self.send_duration_peak_s, duration)
                if duration > self.period_s:
                    self.deadline_misses += 1
                if updated > 0 and command != (0.0, 0.0, 0.0, 0.0):
                    self.decision_to_send_peak_s = max(
                        self.decision_to_send_peak_s, self.last_send - updated)
                    if source_timestamp is not None:
                        self.observation_to_send_peak_s = max(
                            self.observation_to_send_peak_s,
                            self.last_send - source_timestamp)
                next_send += self.period_s
                time.sleep(max(0.0, next_send - time.monotonic()))
                if next_send < time.monotonic() - self.period_s:
                    next_send = time.monotonic()
        except Exception as exc:
            self.failure = exc

    def stop(self, timeout_s=1.0):
        with self._lock:
            frame=self._frame
            heading=self._command[3] if frame == 'local' else 0.0
            self._command = (0.0, 0.0, 0.0, heading)
            self._running = False
        if self._thread is not None:
            self._thread.join(timeout_s)
            if self._thread.is_alive():
                raise RuntimeError("Command worker did not terminate")
        # Explicit final zero after joining, so an arbitrary old setpoint is
        # never the final packet merely because shutdown raced the send tick.
        if self.failure is None:
            if frame == 'local': self.transmit(0.,0.,0.,heading,frame='local')
            else: self.transmit(0.,0.,0.,0.)
