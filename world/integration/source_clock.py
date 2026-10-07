"""Detect repeated/backward source stamps without mixing clock domains."""


class SourceClockGate:
    def __init__(self, source):
        self.source = source
        self.last_stamp = None
        self.failure = None
        self.duplicates = 0

    def accept(self, stamp):
        if self.failure is not None:
            return False
        if stamp is None:
            return True  # Receipt-age still applies; source continuity unavailable.
        if self.last_stamp is not None:
            if stamp < self.last_stamp:
                self.failure = f"{self.source} source clock moved backward"
                return False
            if stamp == self.last_stamp:
                self.duplicates += 1
                return False
        self.last_stamp = stamp
        return True
