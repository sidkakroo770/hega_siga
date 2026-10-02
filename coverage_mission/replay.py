"""Independent synthetic image/kinematic test, NOT a Gazebo flight claim."""
import argparse
from dataclasses import replace
import json
from pathlib import Path
import time
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from .config import Config
from .geometry import Pose
from .engine import Engine


def render(cfg,pose,zones):
    image=np.full((cfg.camera.height,cfg.camera.width,3),(65,125,70),np.uint8)
    # Forward rendering is independent of the inverse ground-projector code.
    optical_to_body=np.array([[0,-1,0],[1,0,0],[0,0,1.]])
    R=Rotation.from_euler('xyz',[pose.roll,pose.pitch,pose.yaw]).as_matrix()@optical_to_body
    K=np.array([[cfg.camera.fx,0,cfg.camera.width/2],[0,cfg.camera.fx,cfg.camera.height/2],[0,0,1.]])
    for n0,n1,e0,e1 in zones:
        world=np.array([[n0,e0,0],[n1,e0,0],[n1,e1,0],[n0,e1,0]],float)
        camera=(world-[pose.n,pose.e,-pose.alt])@R
        if np.any(camera[:,2]<=0): continue
        pix=(camera@K.T); pix=pix[:,:2]/pix[:,2,None]
        cv2.fillPoly(image,[np.rint(pix).astype(np.int32)],(0,0,255))
    return image


def run(cfg,zones,max_seconds=1600,dt=.15):
    engine=Engine(cfg); pose=Pose(0,0,0,cfg.altitude)
    trail=[]; start=time.monotonic(); incursions=0; outside=0
    for k in range(int(max_seconds/dt)):
        image=render(cfg,pose,zones)
        engine.observe(image,pose)
        d=engine.step(pose)
        trail.append([pose.t,pose.n,pose.e,d.vn,d.ve])
        for a,b,c,e in zones:
            near=np.array([np.clip(pose.n,a,b),np.clip(pose.e,c,e)])
            if np.linalg.norm(pose.xy-near)<cfg.body_radius: incursions+=1
        if not(cfg.n_min+cfg.body_radius<=pose.n<=cfg.n_max-cfg.body_radius and cfg.e_min+cfg.body_radius<=pose.e<=cfg.e_max-cfg.body_radius): outside+=1
        if d.state in ('COMPLETE','ABORTED','BLOCKED'): break
        pose=replace(pose,t=pose.t+dt,n=pose.n+d.vn*dt,e=pose.e+d.ve*dt,vn=d.vn,ve=d.ve)
    report={'kind':'synthetic_camera_kinematics','state':d.state,'reason':d.reason,
            'config':cfg.as_dict(),
            'sim_seconds':pose.t,'wall_seconds':time.monotonic()-start,
            'incursion_samples':incursions,'outside_samples':outside,
            'pending':len(engine.plan.pending()),'unseen_cells':int((~engine.ground.observed & ~engine.ground.red).sum()),
            'residence_events':engine.residence.events,'steps':len(trail)}
    return report,np.array(trail),engine


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--full',action='store_true')
    p.add_argument('--scenario',choices=['clear','central','multiple','row_end','row_end_open'],default='central')
    p.add_argument('--output',type=Path)
    args=p.parse_args()
    cfg=Config() if args.full else replace(Config(),n_max=10.4,e_max=7.8)
    zones={'clear':[], 'central':[(4,6,-.5,2)],
           'multiple':[(3,4.5,-.5,1),(6,8,3,4.5)],
           'row_end':[(8,9.5,-.4,1.2)],'row_end_open':[(8,9.5,0,1.2)]}[args.scenario]
    report,trail,engine=run(cfg,zones)
    print(json.dumps(report,indent=2))
    if args.output:
        args.output.mkdir(parents=True,exist_ok=True)
        (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        np.savez_compressed(args.output/'trace.npz',trail=trail,red=engine.ground.red,observed=engine.ground.observed,done=engine.plan.done)
    return 0 if report['state']=='COMPLETE' and not report['incursion_samples'] and not report['outside_samples'] else 1


if __name__=='__main__': raise SystemExit(main())
