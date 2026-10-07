"""Bench-test both IMX296 streams and mission colour processing on a Pi 5.

Run from the repository root with ``python3 -m world.integration.pi_camera_benchmark``.
This is a ground-only diagnostic: it never connects to or commands the FC.
"""

import argparse
import json
from pathlib import Path
import resource
import time

import cv2

from approach.autonomy.perception.hybrid_banner_detector import HybridBannerDetector
from coverage_mission.config import Config
from coverage_mission.geometry import red_regions
from coverage_mission.geometry import Pose
from coverage_mission.qr import QRService, QRConfig, decoder_self_check
from .pi_camera import PiCameraStream


def temperature_c():
    try:
        return int(Path('/sys/class/thermal/thermal_zone0/temp').read_text()) / 1000
    except (OSError, ValueError):
        return None


def cpu_seconds():
    usage = resource.getrusage(resource.RUSAGE_SELF)
    return usage.ru_utime + usage.ru_stime


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--front-index', type=int, required=True)
    parser.add_argument('--downward-index', type=int, required=True)
    parser.add_argument('--seconds', type=float, default=120)
    parser.add_argument('--fps', type=float, default=15)
    parser.add_argument('--output', type=Path, default=Path('artifacts/pi_camera_benchmark.json'))
    parser.add_argument('--gui', action='store_true', help='Preview only; benchmark again without it')
    parser.add_argument('--qr',action='store_true',help='Include the isolated 5 Hz QR discovery worker')
    parser.add_argument('--qr-stationary',action='store_true',
                        help='GROUND TEST ONLY: also exercise 2 Hz selected-marker decoding')
    args = parser.parse_args(argv)
    if args.front_index == args.downward_index or args.seconds <= 0:
        parser.error('camera indexes must differ and duration must be positive')
    cv2.setNumThreads(1)
    streams = [PiCameraStream(args.front_index, 'front', fps=args.fps, history=2),
               PiCameraStream(args.downward_index, 'downward', fps=args.fps, history=2)]
    detector = HybridBannerDetector()
    cfg = Config()
    if args.qr_stationary and not args.qr: parser.error('--qr-stationary requires --qr')
    if args.qr: decoder_self_check()
    qr_service=QRService(cfg,QRConfig()) if args.qr else None
    qr_result=None
    seen = {'front': 0, 'downward': 0}
    processed = {'front': 0, 'downward': 0}
    processing_s = {'front': 0., 'downward': 0.}
    processing_peak_s = {'front': 0., 'downward': 0.}
    samples = []
    initial_temp = temperature_c()
    try:
        identities = [stream.start() for stream in streams]
        print('Camera mapping:', identities, flush=True)
        start = time.monotonic()
        cpu_start = cpu_seconds()
        next_report = start + 1
        while time.monotonic() - start < args.seconds:
            for stream in streams:
                frames, error = stream.snapshot()
                if error:
                    raise RuntimeError(f'{stream.name} camera failed: {error}')
                if not frames or frames[-1][3] == seen[stream.name]:
                    continue
                source_t, receipt_t, frame, seq = frames[-1]
                seen[stream.name] = seq
                processing_start = time.monotonic()
                if stream.name == 'front':
                    detector.detect(frame, debug=False)
                else:
                    red_regions(frame, cfg)
                    if qr_service:
                        observed=qr_service.poll()
                        if observed: qr_result=observed
                        request={}
                        # Synthetic pose only for this ground-only CPU workload;
                        # never fed to flight control or claimed as calibration.
                        if args.qr_stationary and qr_result and qr_result.get('observations'):
                            request={'qr_read_since':0.,'qr_target':qr_result['observations'][0]['xy']}
                        qr_service.submit(frames[-1],Pose(source_t,0,0,10),request)
                elapsed = time.monotonic() - processing_start
                processed[stream.name] += 1
                processing_s[stream.name] += elapsed
                processing_peak_s[stream.name] = max(processing_peak_s[stream.name], elapsed)
                if args.gui:
                    cv2.imshow(stream.name, frame)
                    cv2.waitKey(1)
            now = time.monotonic()
            if now >= next_report:
                snapshot = {'elapsed_s': now - start, 'temperature_c': temperature_c(),
                            'cpu_cores_equivalent': (cpu_seconds() - cpu_start) / (now - start),
                            'rss_max_mib': resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024,
                            'cameras': [stream.health() for stream in streams],
                            'processed': dict(processed)}
                samples.append(snapshot)
                print(json.dumps(snapshot), flush=True)
                next_report = now + 1
            time.sleep(.002)
        duration = time.monotonic() - start
        result = {'duration_s': duration, 'requested_fps': args.fps,
                  'identities': identities, 'temperature_start_c': initial_temp,
                  'temperature_end_c': temperature_c(),
                  'cpu_cores_equivalent': (cpu_seconds() - cpu_start) / duration,
                  'rss_max_mib': resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024,
                  'cameras': [stream.health() for stream in streams],
                  'processed_fps': {name: count / duration for name, count in processed.items()},
                  'processing_mean_ms': {name: 1000 * processing_s[name] / count if count else None
                                         for name, count in processed.items()},
                  'processing_peak_ms': {name: 1000 * value for name, value in processing_peak_s.items()},
                  'samples': samples,'qr_worker':qr_service.metrics if qr_service else None,
                  'qr_synthetic_ground_benchmark':bool(qr_service)}
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + '\n')
        print(f'Wrote {args.output}', flush=True)
        return result
    finally:
        if qr_service: qr_service.close()
        for stream in streams:
            stream.close()
        if args.gui:
            cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
