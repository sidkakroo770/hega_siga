"""Fast planner-liveness check from a stopped coverage run's saved map.

This kinematic replay never substitutes for Gazebo truth, camera timing,
vehicle tracking, or physical-flight validation.
"""
import argparse
import json
import time
from pathlib import Path

import numpy as np

from .config import Config
from .geometry import Pose
from .planning import CoveragePlan, GroundMap, route


def replay(config_path: Path, artifact_dir: Path, max_trips: int = 100) -> dict:
    cfg = Config.load(config_path)
    saved = np.load(artifact_dir / 'coverage.map.npz')
    result = json.loads((artifact_dir / 'coverage.result.json').read_text())
    xy = np.asarray(result['last_decision']['position'], dtype=float)
    ground = GroundMap(cfg)
    ground.observed = saved['observed'].copy()
    ground.red = saved['red'].copy()
    ground.confirmed = saved['confirmed'].copy()
    ground.refresh()
    plan = CoveragePlan(cfg)
    plan.done = saved['done'].copy()
    plan.excluded = saved['excluded'].copy()
    initial = len(plan.pending())
    unresolved = lambda: int((~ground.observed & ~ground.contextual_clear &
                              ~ground.red & ~ground.enclosed).sum())
    initial_unseen = unresolved()
    tick = 1.0
    plan.last = Pose(tick, *xy, cfg.altitude)
    worst_route_ms = 0.0
    length_m = 0.0
    failure = None
    trips = 0
    while len(plan.pending()) and trips < max_trips:
        before = len(plan.pending())
        target = plan.reachable_target(ground, xy, plan.pending())
        started = time.monotonic()
        path = route(ground, xy, target, cfg.planning_wall_budget)
        worst_route_ms = max(worst_route_ms, (time.monotonic() - started) * 1000)
        if not path:
            failure = 'route budget expired' if path is None else 'no checked route'
            break
        for destination in path:
            if not ground.line_clear(xy, destination):
                failure = 'unchecked route segment'
                break
            origin = xy.copy()
            distance = float(np.linalg.norm(destination - origin))
            length_m += distance
            for fraction in np.linspace(0, 1, max(2, int(distance / .05))):
                tick += .1
                point = origin + fraction * (destination - origin)
                plan.update(Pose(tick, *point, cfg.altitude), ground)
            xy = destination
        if failure:
            break
        trips += 1
        if len(plan.pending()) >= before:
            failure = 'no measured traversal progress'
            break
    return {'kind': 'offline_saved_map_kinematic_replay',
            'initial_pending': initial, 'remaining_pending': len(plan.pending()),
            'initial_unseen': initial_unseen, 'remaining_unseen': unresolved(),
            'contextually_inferred_cells': int(ground.contextual_clear.sum()),
            'checked_trips': trips, 'checked_distance_m': round(length_m, 3),
            'worst_route_ms': round(worst_route_ms, 3), 'failure': failure,
            'passed': failure is None and not len(plan.pending()) and unresolved()==0}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--artifact-dir', type=Path, required=True)
    args = parser.parse_args()
    result = replay(args.config, args.artifact_dir)
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result['passed'] else 1)


if __name__ == '__main__':
    main()
