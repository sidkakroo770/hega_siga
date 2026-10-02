"""Read-only Gazebo test oracle. NEVER imported by the mission/controller.

Logs actual model poses and footprint-vs-zone checks independently of its map.
Start before takeoff/controller. A partial trace does not validate the whole flight.
"""
import os
os.environ.setdefault('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION','python')
import argparse
import json
import math
from pathlib import Path
import threading
import time


def evaluate(n,e,cfg,zones):
    radius=cfg['body_radius']
    outside=not (cfg['n_min']+radius<=n<=cfg['n_max']-radius and
                 cfg['e_min']+radius<=e<=cfg['e_max']-radius)
    distance=min((math.hypot(n-min(max(n,a),b),e-min(max(e,c),d))
                  for a,b,c,d in zones),default=math.inf)
    return distance<radius,outside,distance-radius


def main():
    from gz.transport13 import Node
    from gz.msgs10.pose_v_pb2 import Pose_V
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--truth',type=Path,default=Path('artifacts/gazebo_fixture/truth.json'))
    p.add_argument('--config',type=Path,help='Use this camera/field config instead of a fixture truth file')
    p.add_argument('--zone',action='append',nargs=4,type=float,metavar=('N_MIN','N_MAX','E_MIN','E_MAX'),
                   help='Independent physical red rectangle; repeat for each zone')
    p.add_argument('--model-name',default='iris_coverage')
    p.add_argument('--pose-topic',default='/world/coverage_test/pose/info')
    p.add_argument('--ground-z',type=float,default=.003)
    p.add_argument('--output',type=Path,default=Path('artifacts/truth.jsonl'))
    p.add_argument('--seconds',type=float,default=600)
    args=p.parse_args()
    if args.config:
        from .config import Config
        truth={'config':Config.load(args.config).as_dict(), 'zones':args.zone or [],
               'ground_z':args.ground_z}
    else:
        truth=json.loads(args.truth.read_text())
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.with_suffix('.fixture.json').write_text(json.dumps(truth,indent=2)+'\n')
    lock=threading.Lock(); records=[]
    def callback(msg):
        for pose in msg.pose:
            if pose.name!=args.model_name: continue
            t=msg.header.stamp.sec+msg.header.stamp.nsec*1e-9
            n,e,z=pose.position.y,pose.position.x,pose.position.z
            inside,outside,clearance=evaluate(n,e,truth['config'],truth['zones'])
            record={'t':t,'n':n,'e':e,'z':z,'inside':inside,'outside':outside,
                    'quaternion_xyzw':[pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w],
                    'red_clearance':clearance if math.isfinite(clearance) else None}
            with lock: records.append(record)
    node=Node()
    if not node.subscribe(Pose_V,args.pose_topic,callback):
        raise SystemExit('Truth pose subscription failed')
    start=time.monotonic(); count=incursions=outside=0; first=last=None
    closest=math.inf; max_gap=0
    try:
        with args.output.open('w') as out:
            while time.monotonic()-start<args.seconds:
                with lock: batch=records[:]; records.clear()
                for r in batch:
                    # Include takeoff in the trace; score only aircraft above 9 m.
                    out.write(json.dumps(r)+'\n')
                    if r['z']<9: continue
                    count+=1; incursions+=r['inside']; outside+=r['outside']
                    if first is None: first=r['t']
                    if last is not None: max_gap=max(max_gap,r['t']-last)
                    last=r['t']
                    if r['red_clearance'] is not None: closest=min(closest,r['red_clearance'])
                out.flush(); time.sleep(.05)
    except KeyboardInterrupt: pass
    report={'kind':'independent_gazebo_truth','airborne_samples':count,
            'start_source_t':first,'end_source_t':last,'max_sample_gap':max_gap,
            'incursion_samples':incursions,'outside_samples':outside,
            'min_red_clearance':closest if math.isfinite(closest) else None}
    args.output.with_suffix('.report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__': main()
