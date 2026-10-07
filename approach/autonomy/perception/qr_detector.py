"""Compatibility import for the shared geometry-only QR detector.

Legacy standalone trackers are not the full mission command owner. Decoding now
requires explicit allow_decode=True; flight inspection uses the isolated worker.
"""
import sys
from pathlib import Path

# Also support the historical standalone launch from the approach directory.
root=str(Path(__file__).resolve().parents[3])
if root not in sys.path: sys.path.insert(0,root)
from coverage_mission.qr_detector import QRDetector
