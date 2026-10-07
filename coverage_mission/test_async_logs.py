"""Coverage diagnostic I/O must not block its flight-control supervisor."""

import json
import threading
import time

from coverage_mission.async_logs import AsyncJsonlWriter


def test_jsonl_writer_preserves_both_streams(tmp_path):
    path = tmp_path / "run.jsonl"
    logs = AsyncJsonlWriter(path)
    logs.submit("decision", {"state": "SWEEP"})
    logs.submit("supervision", {"valid": True})
    logs.submit('qr',{'seq':42})
    assert logs.close(timeout_s=2.)
    assert logs.failure is None and logs.dropped == 0
    assert json.loads(path.read_text().strip()) == {"state": "SWEEP"}
    assert json.loads(path.with_suffix(".supervision.jsonl").read_text().strip()) == {"valid": True}
    assert json.loads(path.with_suffix('.qr.jsonl').read_text().strip())=={'seq':42}


def test_slow_disk_drops_records_without_stalling_supervisor(tmp_path, monkeypatch):
    entered = threading.Event()
    release = threading.Event()
    original = AsyncJsonlWriter._write

    def slow(self, streams, channel, record):
        entered.set()
        release.wait(1.)
        original(self, streams, channel, record)

    monkeypatch.setattr(AsyncJsonlWriter, "_write", slow)
    logs = AsyncJsonlWriter(tmp_path / "slow.jsonl", max_records=2)
    logs.submit("decision", {"sequence": 0})
    assert entered.wait(1.)
    begun = time.monotonic()
    for sequence in range(100):
        logs.submit("decision", {"sequence": sequence + 1})
    assert time.monotonic() - begun < .1
    assert logs.dropped > 0
    release.set()
    assert logs.close(timeout_s=2.)
    assert logs.failure is None
