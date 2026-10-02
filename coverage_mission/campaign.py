"""Own an isolated Gazebo/SITL test stack, record truth, and clean up only its PIDs."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import threading
import time
from .fixture import build, ROOT
from .config import Config
from .evaluate_run import evaluate_run


def stop(process):
    if process.poll() is not None: return
    os.killpg(process.pid,signal.SIGINT)
    try: process.wait(timeout=4)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid,signal.SIGTERM)
        try: process.wait(timeout=4)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGKILL); process.wait(timeout=4)


def run(config,scenario,output,limit=600,speedup=1,yaw=0,faults=None,pause_at=None,instance=0,entry=(0.,0.)):
    if not 0<=instance<=9: raise ValueError('SITL instance must be in [0,9]')
    tcp=5760+10*instance; physics=9002+10*instance
    takeoff_port=14650+100*instance; mission_port=14652+100*instance
    # Never attach to or kill an existing user's aircraft/physics server.
    for port,kind in ((tcp,socket.SOCK_STREAM),(physics,socket.SOCK_DGRAM),
                      (takeoff_port,socket.SOCK_DGRAM),(mission_port,socket.SOCK_DGRAM)):
        with socket.socket(socket.AF_INET,kind) as sock:
            sock.bind(('127.0.0.1',port))
    output.mkdir(parents=True,exist_ok=False)
    (output/'run_spec.json').write_text(json.dumps({'scenario':scenario,'config':str(config),
        'speedup':speedup,'spawn_yaw':yaw,'entry_north_east':entry,'instance':instance,
        'faults':str(faults) if faults else None,'pause_at':pause_at,'wall_limit':limit},indent=2)+'\n')
    fixture=output/'fixture'; build(config,fixture,scenario,yaw,speedup,instance)
    env=os.environ.copy()
    env['GZ_PARTITION']='sae_coverage_campaign_'+str(instance)
    if faults: env['COVERAGE_TEST_INJECTION']='1'
    env['GZ_SIM_RESOURCE_PATH']=str(fixture/'models')+':'+env.get('GZ_SIM_RESOURCE_PATH','')
    env['SDF_PATH']=env['GZ_SIM_RESOURCE_PATH']
    processes=[]; files=[]
    def launch(name,command,cwd=ROOT):
        log=(output/(name+'.log')).open('w'); files.append(log)
        p=subprocess.Popen(command,cwd=cwd,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(p); return p
    try:
        gz=launch('gazebo',['gz','sim','-r','-s','--headless-rendering',str(fixture/'coverage.sdf')])
        sitl_dir=output/'sitl'; sitl_dir.mkdir()
        ap='/home/sid/ardupilot'
        launch('sitl',[ap+'/build/sitl/bin/arducopter','--model','JSON','--speedup',str(speedup),
            '--defaults',ap+'/Tools/autotest/default_params/copter.parm,'+ap+'/Tools/autotest/default_params/gazebo-iris.parm',
            '--sim-address=127.0.0.1','-I'+str(instance)],sitl_dir)
        time.sleep(2)
        launch('mavproxy',['mavproxy.py',f'--master=tcp:127.0.0.1:{tcp}',f'--out=udp:127.0.0.1:{takeoff_port}',
            f'--out=udp:127.0.0.1:{mission_port}','--streamrate=30','--non-interactive'],output)
        truth=launch('truth',[sys.executable,'-m','coverage_mission.truth_monitor',
            '--truth',str(fixture/'truth.json'),'--output',str(output/'truth.jsonl'),'--seconds',str(limit+180)])
        takeoff=launch('takeoff',[sys.executable,'-m','coverage_mission.takeoff_test',
            '--mavlink',f'udpin:127.0.0.1:{takeoff_port}',
            '--altitude',str(Config.load(config).altitude),
            '--entry-north',str(entry[0]),'--entry-east',str(entry[1])])
        if takeoff.wait(timeout=150)!=0: raise RuntimeError('Takeoff failed; see takeoff.log')
        command=[sys.executable,'-m','coverage_mission.runtime','--config',str(config),
            '--mavlink',f'udpin:127.0.0.1:{mission_port}',
            '--fly','--no-gui','--max-wall-seconds',str(limit),'--log',str(output/'mission.jsonl')]
        if faults: command.extend(['--faults',str(faults)])
        mission=launch('mission',command)
        def pause_test():
            trace=output/'mission.supervision.jsonl'
            deadline=time.monotonic()+limit
            while mission.poll() is None and time.monotonic()<deadline:
                try:
                    rows=trace.read_text().splitlines()
                    a=json.loads(rows[0]); b=json.loads(rows[-1])
                    if b['t']-a['t']>=pause_at: break
                except (FileNotFoundError,IndexError,json.JSONDecodeError): pass
                time.sleep(.1)
            else: return
            def control(value):
                return subprocess.run(['gz','service','-s','/world/coverage_test/control',
                    '--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','2000',
                    '--req','pause: '+value],env=env,capture_output=True,text=True,timeout=4)
            first=control('true')
            wall_start=time.monotonic()
            try: time.sleep(3)
            finally:
                wall_end=time.monotonic()
                second=control('false')
            (output/'pause.json').write_text(json.dumps({'source_start':b['t'],
                'wall_start':wall_start,'wall_end':wall_end,
                'wall_pause':3,'pause_response':first.stdout,'resume_response':second.stdout},indent=2)+'\n')
        if pause_at is not None: threading.Thread(target=pause_test,daemon=True).start()
        mission.wait(timeout=limit+30)
        time.sleep(1); stop(truth)
        evaluation=evaluate_run(output/'mission.jsonl',output/'truth.jsonl')
        (output/'evaluation.json').write_text(json.dumps(evaluation,indent=2)+'\n')
        print(json.dumps(evaluation,indent=2),flush=True)
        return evaluation
    finally:
        for p in reversed(processes): stop(p)
        for f in files: f.close()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,default=ROOT/'config/smoke.json')
    p.add_argument('--scenario',default='multiple')
    p.add_argument('--output',required=True,type=Path)
    p.add_argument('--limit',type=float,default=600)
    p.add_argument('--speedup',type=float,default=1)
    p.add_argument('--yaw',type=float,default=0,help='Initial NED yaw radians')
    p.add_argument('--faults',type=Path)
    p.add_argument('--pause-at',type=float)
    p.add_argument('--instance',type=int,default=0,help='Isolated Gazebo partition and SITL/MAVLink ports (0..9)')
    a=p.parse_args(); r=run(a.config.resolve(),a.scenario,a.output.resolve(),a.limit,a.speedup,a.yaw,a.faults.resolve() if a.faults else None,a.pause_at,a.instance)
    return 0 if r.get('sampled_flight_safety_passed') and r.get('sampled_geometry_coverage_passed') and r.get('original_025m_projection_target_passed') else 1


if __name__=='__main__': raise SystemExit(main())
