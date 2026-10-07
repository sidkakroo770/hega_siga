"""Saved-map return regression with synthetic planar observations, no aircraft."""
import argparse
from dataclasses import replace
import json
import math
from pathlib import Path
import numpy as np
from .config import Config
from .engine import Engine
from .geometry import Pose
from .return_mission import ReturnConfig, ReturnNavigator


def replay(map_path, cfg, profile):
    engine=Engine(cfg)
    with np.load(map_path) as saved:
        for key in ('observed','red','confirmed','enclosed'):
            getattr(engine.ground,key)[:]=saved[key]
    engine.ground.red_hits[:]=engine.ground.red.astype(np.uint8)*cfg.red_confirm_frames
    engine.ground.refresh()
    navigator=ReturnNavigator(cfg,profile)
    p=Pose(0,-14,-4,5,yaw=profile.heading-cfg.field_yaw)
    # Independent static fixture zones; ground evidence is synthetic, not a
    # claim of camera/rendering or flight-dynamics validation.
    zones=[(-9,-6,-4,-2),(-2.25,7.75,-5.11,2.32),(8,11,6,8)]
    polygons=[np.array([[a,c],[a,d],[b,d],[b,c]]) for a,b,c,d in zones]
    phases=[]; visited=[]; peak_ms=0.; state='WAITING'
    for i in range(4000):
        footprint=engine.projector.footprint(p)
        engine.ground.observe(footprint,polygons,p.t)
        proposed,permit=navigator.step(p,engine.ground)
        decision=engine.step(p,p.t,proposed,descent_active=True,transition=permit)
        if not phases or phases[-1]['state']!=decision.state:
            phases.append({'state':decision.state,'t':p.t,'reason':decision.reason})
        visited.append([p.n,p.e,p.alt])
        if engine.ground.inside_red(p.xy):
            return {'pass':False,'reason':'Synthetic red incursion','phases':phases}
        if decision.state=='RETURN_FRONT':
            return {'pass':True,'phases':phases,'source_seconds':p.t,
                    'final_position':visited[-1],'samples':len(visited)}
        if decision.state in ('ABORTED','BLOCKED'):
            return {'pass':False,'reason':decision.reason,'phases':phases,'position':visited[-1]}
        if decision.state in ('HOLD','BRAKE','ESCAPE'): navigator.interrupt()
        # Modest actuator lag, not perfect instantaneous waypoint tracking.
        vn=p.vn+.4*(decision.vn-p.vn); ve=p.ve+.4*(decision.ve-p.ve); vd=p.vd+.4*(decision.vd-p.vd)
        p=replace(p,t=round((i+1)*.1,6),n=p.n+vn*.1,e=p.e+ve*.1,alt=p.alt-vd*.1,vn=vn,ve=ve,vd=vd)
    return {'pass':False,'reason':'Replay bounded step budget','phases':phases}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    root=Path(__file__).parents[1]
    parser.add_argument('--map',type=Path,required=True)
    parser.add_argument('--config',type=Path,default=root/'config/full_mission_coverage.json')
    parser.add_argument('--profile',type=Path,default=root/'config/return_mission.json')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args(); result=replay(args.map,Config.load(args.config),ReturnConfig.load(args.profile))
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2)+'\n'); print(json.dumps(result,indent=2))
    return 0 if result['pass'] else 1


if __name__=='__main__': raise SystemExit(main())
