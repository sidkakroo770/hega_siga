"""Owned-process QR integration harness. Never kills unrelated user processes.

Run as a module from repository root. One early-target mission, retained logs,
independent pose oracle. This file is testing infrastructure, not flight control.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import shutil
import socket
import subprocess
import time
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[2]


def environment():
    env=os.environ.copy()
    env.update(GZ_IP='127.0.0.1',GZ_PARTITION='qr_validation',
        GZ_DISCOVERY_MULTICAST_IP='239.255.0.7',
        GZ_SIM_SYSTEM_PLUGIN_PATH='/home/sid/ardupilot_gazebo/build',
        GZ_SIM_RESOURCE_PATH=f'{ROOT}/world/models/models:/home/sid/ardupilot_gazebo/models',
        PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION='python',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',
        PYTHONPATH=f'{ROOT}:{ROOT}/world/integration:{ROOT}/approach:{ROOT}/corridor:/usr/lib/python3/dist-packages')
    return env


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seconds',type=float,default=600)
    p.add_argument('--gui',action='store_true')
    p.add_argument('--instance',type=int,choices=range(10),default=0,
                   help='Owned SITL instance; separate TCP, JSON and MAVProxy ports')
    p.add_argument('--return-mission',action='store_true',
                   help='Run the new complete return/landing; otherwise preserve QR-only endpoint')
    p.add_argument('--scenario',choices=['nominal','missing-reference','return-entry',
                                       'qr-near-red','qr-too-close-red','qr-textured-approach'],default='nominal')
    a=p.parse_args(); output=a.output.resolve(); output.mkdir(parents=True,exist_ok=True)
    if (output/'manager.log').exists(): raise SystemExit('Use a fresh evidence directory')
    shutil.copyfile(ROOT/'world/models/models/qr_markers/truth.json',output/'qr_fixture.json')
    tcp_port=5760+10*a.instance; output_port=14552+10*a.instance
    for port in (tcp_port,):
        with socket.socket() as s:
            if s.connect_ex(('127.0.0.1',port))==0: raise SystemExit(f'Port {port} occupied; stop the existing simulator explicitly')
    env=environment(); processes=[]; streams=[]
    env['GZ_PARTITION']=f'qr_validation_{a.instance}'
    world_path=ROOT/'world/worlds/miss2_full_world.sdf'
    field_fixture=a.scenario in ('qr-near-red','qr-too-close-red','qr-textured-approach')
    textured_approach=a.scenario=='qr-textured-approach'
    if field_fixture:
        if a.return_mission: raise SystemExit('Field fixtures intentionally stop before the return')
        env['MISSION_OWNED_FIELD_QR_FIXTURE']='1'
        target_n=-9.55 if a.scenario=='qr-too-close-red' else -10.2
        marker_tree=ET.parse(ROOT/'world/models/models/qr_markers/model.sdf')
        container=marker_tree.getroot().find('model')
        for marker in list(container.findall('model')):
            if marker.attrib['name']=='qr_target_early': marker.find('pose').text=f'-3 {target_n} 0.12 0 0 0'
            elif not textured_approach: container.remove(marker)
        marker_dir=output/'models/qr_field'; marker_dir.mkdir(parents=True)
        marker_tree.write(marker_dir/'model.sdf',encoding='unicode',xml_declaration=True)
        shutil.copyfile(ROOT/'world/models/models/qr_markers/model.config',marker_dir/'model.config')
        world_tree=ET.parse(world_path)
        for include in world_tree.getroot().findall('.//world/include'):
            if include.findtext('uri')=='model://iris_miss2_full':
                include.find('pose').text=('-4 -18.2 0.35 0 0 1.5708' if textured_approach
                                          else '-3.8 -10.2 0.35 0 0 1.5708')
            if include.findtext('uri')=='model://qr_markers': include.find('uri').text=str(marker_dir)
        world_path=output/'field_qr.sdf'
        world_tree.write(world_path,encoding='unicode',xml_declaration=True)
        if not textured_approach:
            (output/'qr_fixture.json').write_text(json.dumps([dict(name='qr_target_early',
                payload='REF-001',world_enu=[-3,target_n,.12],size_m=1.)],indent=2)+'\n')
    if a.scenario=='return-entry':
        a.return_mission=True
        env['MISSION_OWNED_RETURN_FIXTURE']='1'
        world_tree=ET.parse(world_path)
        for include in world_tree.getroot().findall('.//world/include'):
            if include.findtext('uri')=='model://iris_miss2_full':
                include.find('pose').text='4.1 -14 0.35 0 0 1.5708'
        world_path=output/'return_entry.sdf'
        world_tree.write(world_path,encoding='unicode',xml_declaration=True)
    if a.scenario=='missing-reference':
        marker_tree=ET.parse(ROOT/'world/models/models/qr_markers/model.sdf')
        container=marker_tree.getroot().find('model')
        container.remove(container.find("model[@name='qr_start']"))
        marker_dir=output/'models/qr_missing'; marker_dir.mkdir(parents=True)
        marker_tree.write(marker_dir/'model.sdf',encoding='unicode',xml_declaration=True)
        shutil.copyfile(ROOT/'world/models/models/qr_markers/model.config',marker_dir/'model.config')
        world_tree=ET.parse(world_path)
        for uri in world_tree.getroot().findall('.//include/uri'):
            if uri.text=='model://qr_markers': uri.text=str(marker_dir)
        world_path=output/'missing_reference.sdf'
        world_tree.write(world_path,encoding='unicode',xml_declaration=True)
    if a.instance:
        model_dir=output/'models/iris_owned'; model_dir.mkdir(parents=True)
        model_tree=ET.parse(ROOT/'world/models/models/iris_miss2_full/model.sdf')
        model_tree.getroot().find('.//fdm_port_in').text=str(9002+10*a.instance)
        model_tree.write(model_dir/'model.sdf',encoding='unicode',xml_declaration=True)
        shutil.copyfile(ROOT/'world/models/models/iris_miss2_full/model.config',model_dir/'model.config')
        world_tree=ET.parse(world_path)
        for uri in world_tree.getroot().findall('.//include/uri'):
            if uri.text=='model://iris_miss2_full': uri.text=str(model_dir)
        world_path=output/'owned_instance.sdf'
        world_tree.write(world_path,encoding='unicode',xml_declaration=True)
    def launch(name,cmd,cwd=ROOT):
        stream=(output/(name+'.log')).open('w'); streams.append(stream)
        proc=subprocess.Popen(cmd,cwd=cwd,env=env,stdin=subprocess.DEVNULL,
                              stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(proc); return proc
    try:
        gz=launch('gazebo',['gz','sim','-r','-s','-v2',str(world_path)])
        time.sleep(4)
        sitl_dir=output/'sitl'; sitl_dir.mkdir()
        sitl=launch('sitl',['/home/sid/ardupilot/build/sitl/bin/arducopter',
            '--model','JSON','--speedup','1','--home=-35.363261,149.165230,584,0',
            '--defaults','/home/sid/ardupilot/Tools/autotest/default_params/copter.parm,'
                         '/home/sid/ardupilot/Tools/autotest/default_params/gazebo-iris.parm',
            '--sim-address=127.0.0.1',f'-I{a.instance}'],cwd=sitl_dir)
        time.sleep(3)
        proxy=launch('mavproxy',['mavproxy.py',f'--master=tcp:127.0.0.1:{tcp_port}',
            f'--out=udp:127.0.0.1:{output_port}','--streamrate=20','--non-interactive'],cwd=output)
        truth=launch('truth',['python3','-m','coverage_mission.truth_monitor',
            '--config',str(ROOT/'config/full_mission_coverage.json'),
            '--model-name','iris_miss2_full','--pose-topic','/world/miss2_world/pose/info',
            '--zone','-9','-6','-4','-2','--zone','-2.25','7.75','-5.11','2.32',
            '--zone','8','11','6','8','--ground-z','.0731022',
            '--output',str(output/'truth.jsonl'),'--seconds',str(a.seconds+30)])
        time.sleep(6)
        trace=output/'truth.jsonl'
        if not trace.exists() or not trace.stat().st_size:
            raise RuntimeError('Independent pose oracle has no samples; flight start forbidden')
        command=['python3','-u',str(ROOT/'world/integration/experimental_corridor_manager.py'),
            '--start-mission','--takeoff-altitude','5','--mavlink',f'udpin:0.0.0.0:{output_port}',
            '--banner-loss-frames','5','--coverage-log',str(output/'coverage.jsonl'),
            '--debug-output',str(output/'preentry'),'--coverage-max-wall-seconds',str(a.seconds)]
        if not a.gui: command.append('--no-gui')
        if not a.return_mission: command.append('--qr-only')
        if a.scenario=='return-entry': command.append('--test-return-only')
        if field_fixture:
            command[command.index('--takeoff-altitude')+1]='10'
            command.append('--test-field-qr-only')
            if textured_approach: command.append('--test-field-qr-approach')
        manager=launch('manager',command)
        excluded_observed=False
        if a.scenario=='qr-too-close-red':
            deadline=time.monotonic()+a.seconds; excluded_since=None
            while manager.poll() is None and time.monotonic()<deadline:
                trace=output/'coverage.jsonl'
                if trace.exists():
                    # Bounded fixture, not a whole-field negative search.
                    for line in trace.read_text().splitlines()[-10:]:
                        try: record=json.loads(line)
                        except json.JSONDecodeError: continue
                        if any(c['status']=='EXCLUDED' for c in record.get('qr_candidates',[])):
                            if excluded_since is None: excluded_since=time.monotonic()
                    if excluded_since is not None and time.monotonic()-excluded_since>=4:
                        excluded_observed=True; break
                time.sleep(.25)
            code=manager.poll() if manager.poll() is not None else 124
        else:
            try: code=manager.wait(timeout=a.seconds)
            except subprocess.TimeoutExpired: code=124
        if manager.poll() is None: os.killpg(manager.pid,signal.SIGINT); manager.wait(timeout=10)
        os.killpg(truth.pid,signal.SIGINT); truth.wait(timeout=5)
        missing_pass=False
        if a.scenario=='missing-reference' and (output/'initial_qr.result.json').exists():
            failure=json.loads((output/'initial_qr.result.json').read_text())
            manager_log=(output/'manager.log').read_text()
            missing_pass=(code==1 and failure['state']=='ABORTED' and
                'Initial reference deadline' in failure['reason'] and
                'LAND confirmation: True' in manager_log and
                not (output/'coverage.result.json').exists())
        (output/'harness.json').write_text(json.dumps({'manager_exit':code,'scenario':a.scenario,
            'controlled_stop_after_exclusion':excluded_observed,
            'expected_failure_pass':missing_pass,
            'gazebo_exit':gz.poll(),'sitl_exit':sitl.poll(),'mavproxy_exit':proxy.poll()},indent=2)+'\n')
        print('Manager exit:',code,'Evidence:',output,flush=True)
        return 0 if missing_pass or excluded_observed else code
    finally:
        for proc in reversed(processes):
            if proc.poll() is None:
                try: os.killpg(proc.pid,signal.SIGINT); proc.wait(timeout=5)
                except subprocess.TimeoutExpired: os.killpg(proc.pid,signal.SIGTERM); proc.wait(timeout=5)
        for stream in streams: stream.close()


if __name__=='__main__': raise SystemExit(main())
