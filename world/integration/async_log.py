"""Bounded best-effort console diagnostics; never a command or safety channel."""

import queue
import threading


class AsyncLogStream:
    def __init__(self, stream, max_lines=256):
        self.stream = stream
        self.encoding = getattr(stream, "encoding", None)
        self._lines = queue.Queue(maxsize=max_lines)
        self._local = threading.local()
        self.dropped_lines = 0
        self.failure = None
        self._thread = threading.Thread(target=self._run, name="mission-console",
                                        daemon=True)
        self._thread.start()

    def _submit(self, line):
        try:
            self._lines.put_nowait(line)
        except queue.Full:
            self.dropped_lines += 1

    def write(self, text):
        if not isinstance(text, str):
            raise TypeError("Console writes must be text")
        pending = getattr(self._local, "pending", "") + text
        while "\n" in pending:
            line, pending = pending.split("\n", 1)
            self._submit(line + "\n")
        if len(pending) > 4096:
            self._submit(pending[:4096] + "\n")
            pending = pending[4096:]
        self._local.pending = pending
        return len(text)

    def flush(self):
        pending = getattr(self._local, "pending", "")
        if pending:
            self._submit(pending)
            self._local.pending = ""

    def isatty(self):
        return bool(getattr(self.stream, "isatty", lambda: False)())

    def fileno(self):
        return self.stream.fileno()

    def _run(self):
        try:
            while True:
                line = self._lines.get()
                if line is None:
                    return
                self.stream.write(line)
                self.stream.flush()
        except Exception as exc:
            self.failure = exc

    def stop(self, timeout_s=0.2):
        self.flush()
        try:
            self._lines.put_nowait(None)
        except queue.Full:
            pass
        self._thread.join(timeout_s)
