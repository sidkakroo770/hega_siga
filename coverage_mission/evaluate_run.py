"""Offline independent Gazebo trace checks; never used for navigation.

The optical oracle uses Gazebo model orientation and camera +X/-Y/-Z rays,
not the mission's FRD optical transform, HOME altitude, map or pose estimator.
Only accepted observation timestamps in the runtime log earn coverage.
"""
import argparse
import json
import math
from pathlib import Path
import cv2
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from .config import Config, Camera
from .geometry import Pose, Projector
from .truth_monitor import evaluate


def read_rows(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def evaluate_run(mission_path,truth_path,truth_extra=()):
    rows=read_rows(mission_path); truth=read_rows(truth_path)
    fixture=json.loads(truth_path.with_suffix('.fixture.json').read_text())
    for extra in truth_extra:
        if json.loads(extra.with_suffix('.fixture.json').read_text())!=fixture:
            raise ValueError('Truth segments have different field/zones/camera contracts')
        truth.extend(read_rows(extra))
    truth.sort(key=lambda row:row['t'])
    data=dict(fixture['config']); data['camera']=Camera(**data['camera']); cfg=Config(**data)
    result=json.loads(mission_path.with_suffix('.result.json').read_text())
    if result.get('last_decision'): rows.append(result['last_decision'])
    if not rows or not truth: raise ValueError('Empty mission/truth trace')
    first=min(r['source_t'] for r in rows); last=max(r['source_t'] for r in rows)
    covered_interval=truth[0]['t']<=first and truth[-1]['t']>=last
    flight=[p for p in truth if first<=p['t']<=last]
    sample_gap=max((b['t']-a['t'] for a,b in zip(flight,flight[1:])),default=math.inf)
    incursions=outside=0; min_clearance=math.inf
    for p in flight:
        inside,out,clearance=evaluate(p['n'],p['e'],fixture['config'],fixture['zones'])
        incursions+=inside; outside+=out; min_clearance=min(min_clearance,clearance)
    report={'kind':'offline_independent_gazebo_evaluation','controller_state':result['state'],
            'mission_start':first,'mission_end':last,'whole_interval_recorded':covered_interval,
            'truth_samples':len(flight),'body_radius':cfg.body_radius,
            'incursion_samples':incursions,'outside_samples':outside,
            'max_truth_gap':sample_gap if math.isfinite(sample_gap) else None,
            'sampled_flight_safety_passed':bool(covered_interval and sample_gap<=.05 and
                result['state']=='COMPLETE' and not incursions and not outside),
            'min_red_clearance':min_clearance if math.isfinite(min_clearance) else None}
    if not all('quaternion_xyzw' in p for p in truth):
        report['geometry_evaluation']='Unavailable: this older trace has no orientation'
        return report
    timestamps=np.array([p['t'] for p in truth]); xyz=np.array([[p['e'],p['n'],p['z']] for p in truth])
    quats=np.array([p['quaternion_xyzw'] for p in truth])
    # Remove duplicate simulator pose messages without manufacturing missing data.
    timestamps,indices=np.unique(timestamps,return_index=True); xyz=xyz[indices]; quats=quats[indices]
    rotations=Slerp(timestamps,Rotation.from_quat(quats))
    cross_track=[]
    for row in rows:
        leg=row.get('tracking_segment'); t=row['source_t']
        if not leg or t-leg['since']<.5 or not timestamps[0]<=t<=timestamps[-1]: continue
        a=np.array(leg['start']); b=np.array(leg['end']); delta=b-a
        length=np.linalg.norm(delta)
        if length<2: continue  # Short turns/frontiers are checked by clearance instead.
        point=np.array([np.interp(t,timestamps,xyz[:,i]) for i in (1,0)])
        along=np.dot(point-a,delta)/length
        if 0<=along<=length:
            cross_track.append(float(abs(delta[0]*(point-a)[1]-delta[1]*(point-a)[0])/length))
    report.update(straight_leg_samples=len(cross_track),
                  max_straight_leg_cross_track=max(cross_track) if cross_track else None,
                  straight_leg_tracking_passed=bool(cross_track and max(cross_track)<=.30))
    credited_errors=[]
    for row in rows:
        t=row['source_t']
        if not timestamps[0]<=t<=timestamps[-1]: continue
        east=float(np.interp(t,timestamps,xyz[:,0]))
        credited_errors.extend(abs(east-p[1]) for p in row.get('newly_traversed_points',[]))
    report.update(credited_route_samples=len(credited_errors),
                  max_credited_route_cross_track=max(credited_errors) if credited_errors else None,
                  credited_route_tracking_passed=bool(credited_errors and max(credited_errors)<=.30))
    observations={r['frame_t']:r['observation_pose'] for r in rows if r.get('observation_pose')}
    shape=(math.ceil((cfg.n_max-cfg.n_min)/cfg.resolution),math.ceil((cfg.e_max-cfg.e_min)/cfg.resolution))
    coverage=np.zeros(shape,np.uint8); errors=[]; skipped=0
    c=cfg.camera; fx=c.fx
    rays=np.array([[1,c.width/2/fx,c.height/2/fx],
                   [1,-c.width/2/fx,c.height/2/fx],
                   [1,-c.width/2/fx,-c.height/2/fx],
                   [1,c.width/2/fx,-c.height/2/fx]])
    camera_mount=Rotation.from_euler('y',math.pi/2).as_matrix()
    projector=Projector(cfg)
    for t,p in observations.items():
        idx=np.searchsorted(timestamps,t)
        if not 0<idx<len(timestamps) or timestamps[idx]-timestamps[idx-1]>.05:
            skipped+=1; continue
        orientation=rotations(t).as_matrix()
        origin=np.array([np.interp(t,timestamps,xyz[:,i]) for i in range(3)])
        origin+=orientation @ np.array([0.,0.,-cfg.camera.down_offset])
        directions=rays @ (orientation @ camera_mount).T
        if np.any(directions[:,2]>=0): skipped+=1; continue
        # Use independent Gazebo field height, never the controller's HOME datum.
        ground_z=fixture.get('ground_z',.003)
        points=origin+directions*((ground_z-origin[2])/directions[:,2,None])
        footprint=points[:,[1,0]]
        estimated=projector.footprint(Pose(**p))
        errors.extend(np.linalg.norm(footprint-estimated,axis=1).tolist())
        # Grid-centre footprint sampling, with a one-cell inward visibility reserve.
        image=np.zeros((shape[0]+6,shape[1]+6),np.uint8)
        pixels=(footprint-[cfg.n_min,cfg.e_min])/cfg.resolution-.5+3
        cv2.fillPoly(image,[np.rint(pixels[:,::-1]).astype(np.int32)],1)
        image=cv2.erode(image,np.ones((3,3),np.uint8),borderType=cv2.BORDER_CONSTANT,borderValue=0)
        coverage |= image[3:-3,3:-3]
    nn,ee=np.indices(shape)
    nn=cfg.n_min+(nn+.5)*cfg.resolution; ee=cfg.e_min+(ee+.5)*cfg.resolution
    restricted=np.zeros(shape,bool)
    for a,b,c,d in fixture['zones']: restricted|=(nn>=a)&(nn<=b)&(ee>=c)&(ee<=d)
    missing=(coverage==0)&~restricted
    report.update(accepted_observation_times=len(observations),unmatched_truth_observations=skipped,
                  max_projected_corner_error=max(errors) if errors else None,
                  p95_projected_corner_error=float(np.percentile(errors,95)) if errors else None,
                  independently_unseen_permissible_cells=int(missing.sum()),grid_resolution=cfg.resolution,
                  original_025m_projection_target_passed=bool(errors and max(errors)<=.25),
                  sampled_geometry_coverage_passed=bool(errors and not skipped and not missing.any()
                      and max(errors)<=cfg.uncertainty))
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mission',required=True,type=Path)
    parser.add_argument('--truth',required=True,type=Path)
    parser.add_argument('--truth-extra',action='append',type=Path,default=[],
                        help='Overlapping later Gazebo truth segment with identical fixture')
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args(); report=evaluate_run(args.mission,args.truth,args.truth_extra)
    args.output.write_text(json.dumps(report,indent=2)+'\n'); print(json.dumps(report,indent=2))


if __name__=='__main__': main()
