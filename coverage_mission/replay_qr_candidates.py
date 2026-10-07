"""Replay recorded QR geometry without images or flight commands.

Fixture truth is used only by the final test oracle. This replay cannot recreate
changed vehicle motion, worker result delivery or rerun OpenCV on old pixels.
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
from .config import Config
from .qr import QRConfig
from .qr_detector import QRDetector


def replay(directory, output):
    manifest=json.loads((directory/'coverage.manifest.json').read_text())
    config=Config.load(Path(__file__).parents[1]/'config/full_mission_coverage.json')
    limits=QRConfig.load(Path(__file__).parents[1]/'config/qr_mission.json')
    detector=QRDetector(limits)
    truth=json.loads((Path(__file__).parents[1]/'world/models/models/qr_markers/truth.json').read_text())
    counts=Counter(); accepted=[]; rejected=[]; reads=[]
    for line in (directory/'coverage.qr.jsonl').open():
        original=json.loads(line)
        for observation in original.get('observations',[]):
            counts['observations']+=1
            if detector.valid_quad(observation['quad'],(config.camera.height,config.camera.width)):
                accepted.append(observation['xy'])
                counts['accepted']+=1
                if original['decode'] and observation['payload'] is not None:
                    reads.append({'seq':original['seq'],'payload':observation['payload']})
            else:
                counts['rejected']+=1; rejected.append(observation['xy'])
    def distance(xy):
        return min(math.dist(xy,m['world_enu'][1::-1]) for m in truth)
    checks={
        'actual_false_geometry_rejected':bool(rejected) and not any(distance(xy)>.30 for xy in accepted),
        'real_marker_geometry_retained':bool(accepted),
        'three_actual_nonmatch_decode_exposures_retained':len({r['seq'] for r in reads})>=3
            and all(r['payload']=='OTHER-001' for r in reads),
    }
    report={'pass':all(checks.values()),'checks':checks,'geometry_counts':dict(counts),
        'retained_authorized_reads':reads,
        'original_manifest_qr_mode':manifest['qr_mode'],
        'scope':'Recorded geometry filtering only; does not replay image pixels, result delivery, control or aircraft trajectory.'}
    output.write_text(json.dumps(report,indent=2)+'\n')
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('directory',type=Path)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    report=replay(args.directory,args.output)
    print(json.dumps(report,indent=2))
    raise SystemExit(0 if report['pass'] else 1)
