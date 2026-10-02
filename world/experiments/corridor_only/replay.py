#!/usr/bin/env python3
"""Offline saved-scan diagnosis and native FSM test with analytical box walls."""
import argparse
import json
import math
from pathlib import Path
import sys
import time
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path.home() / 'sae_mission2/corridor'))
from native.common.types import NativeScan
from native.controllers.pre_entry import PreEntryController
from native.mission_runner import NativeMissionRunner, MissionRunnerConfig, VehiclePose


def synthetic_scan(x, y=0.0, yaw=0.0, timestamp=None):
    angles = np.linspace(-math.pi, math.pi, 500)
    bearings = angles + yaw
    ranges = np.full(500, np.inf)
    for wall_y in (-1.75, 1.75):
        with np.errstate(divide='ignore', invalid='ignore'):
            distance = (wall_y - y) / np.sin(bearings)
            hit_x = x + distance * np.cos(bearings)
        valid = (distance > 0) & (distance < 12) & (hit_x >= -2) & (hit_x <= 12)
        ranges[valid] = np.minimum(ranges[valid], distance[valid])
    return NativeScan(angles, ranges, np.zeros(500),
                      time.monotonic() if timestamp is None else timestamp, .02, 12.)


def geometry_report(scan):
    controller = PreEntryController()
    geometry = controller.extract_corridor_geometry(scan)
    valid = (np.isfinite(scan.ranges_m) & (scan.ranges_m >= .05)
             & (scan.ranges_m <= 8))
    points = np.column_stack((scan.ranges_m[valid] * np.cos(scan.angles_rad[valid]),
                              scan.ranges_m[valid] * np.sin(scan.angles_rad[valid])))
    report = {'beams': scan.size, 'finite_beams': int(np.isfinite(scan.ranges_m).sum()),
              'confidence': geometry.confidence, 'strict_valid': bool(geometry.strict_valid),
              'width': geometry.width, 'sectors': geometry.sectors}
    for label, sign in [('left', 1), ('right', -1)]:
        selected = points[(points[:, 0] >= -.2) & (points[:, 0] <= 5)
                          & (sign * points[:, 1] >= .25) & (sign * points[:, 1] <= 3.5)]
        fit = controller.fit_line_ransac(selected)
        report[label] = {'candidate_points': len(selected), 'fit_valid': bool(fit.valid),
                         'span': fit.span, 'rms': fit.rms}
    return report, points


def synthetic_test():
    clock = [0.]
    states = []
    with patch('time.monotonic', side_effect=lambda: clock[0]):
        runner = NativeMissionRunner(MissionRunnerConfig(enter_corridor_distance_m=.75))
        x, y, yaw = 0., 0., 0.
        for step in range(1000):
            clock[0] = step * .1
            output = runner.step(synthetic_scan(x, y, yaw, clock[0]),
                                 pose=VehiclePose(x, y, yaw, clock[0]))
            state = runner.public_state().value
            if not states or state != states[-1]:
                states.append(state)
            command = output.command
            x += (command.vx_m_s * math.cos(yaw) - command.vy_m_s * math.sin(yaw)) * .1
            y += (command.vx_m_s * math.sin(yaw) + command.vy_m_s * math.cos(yaw)) * .1
            yaw += command.yaw_rate_rad_s * .1
            if state in ('CORRIDOR_EXITED', 'ABORT_CORRIDOR'):
                break
    assert states[-1] == 'CORRIDOR_EXITED', states
    return {'result': 'PASS', 'states': states, 'final_x_m': x,
            'scope': 'Analytical scan and ideal velocity response, not a Gazebo flight'}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scan', type=Path)
    parser.add_argument('--synthetic', action='store_true')
    parser.add_argument('--output', type=Path, default=Path(__file__).parent / 'artifacts')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    if args.synthetic:
        report = synthetic_test()
        (args.output / 'synthetic_fsm.json').write_text(json.dumps(report, indent=2))
        print(json.dumps(report, indent=2))
    if args.scan:
        with np.load(args.scan) as data:
            scan = NativeScan(data['angles_rad'], data['ranges_m'],
                              np.zeros_like(data['ranges_m']), time.monotonic(),
                              float(data['range_min_m']), float(data['range_max_m']))
        report, points = geometry_report(scan)
        (args.output / 'scan_report.json').write_text(json.dumps(report, indent=2))
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(7, 7))
        ax.scatter(points[:, 0], points[:, 1], s=12, label='Saved scan')
        ax.plot(0, 0, 'ro', label='LiDAR')
        ax.add_patch(plt.Rectangle((-.2, .25), 5.2, 3.25, fill=False, color='green'))
        ax.add_patch(plt.Rectangle((-.2, -3.5), 5.2, 3.25, fill=False, color='green'))
        ax.set(xlabel='Forward (m)', ylabel='Left (m)', title='PRE_ENTRY scan and wall-fit regions')
        ax.set_aspect('equal'); ax.grid(); ax.legend()
        fig.savefig(args.output / 'scan_plot.png', dpi=150); plt.close(fig)
        print(json.dumps(report, indent=2))
    if not args.scan and not args.synthetic:
        parser.error('Choose --scan PATH or --synthetic')


if __name__ == '__main__':
    main()
