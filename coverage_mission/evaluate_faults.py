"""Offline fault-test assertions from issued commands and independent actual motion."""
import argparse
import json
from pathlib import Path
import numpy as np
from .evaluate_run import read_rows


def evaluate(folder):
    result=json.loads((folder/'mission.result.json').read_text())
    commands=read_rows(folder/'mission.supervision.jsonl')
    truth=read_rows(folder/'truth.jsonl')
    decisions=read_rows(folder/'mission.jsonl')
    if result.get('last_decision'): decisions.append(result['last_decision'])
    times=np.array([p['t'] for p in truth]); xy=np.array([[p['n'],p['e']] for p in truth])
    detected={}; checks=[]
    for c in commands:
        for kind in c['faults']:
            detected.setdefault(kind,[]).append(c)
    for kind,rows in detected.items():
        if 'push_north' in detected: continue  # Intentional actuator override, not a stop test.
        start=rows[0]['t']; end=rows[-1]['t']
        if kind not in ('camera_drop','camera_delay','black_frame','telemetry_drop','ekf_invalid','worker_delay'): continue
        # Recovery overrides missing imagery only for an existing known incursion;
        # command-stop tests therefore use the obstacle-free fixture.
        samples=[r for r in rows if r['t']>=start+.5]
        stopped=bool(samples and all(np.linalg.norm(r['command'][:2])<1e-6 for r in samples))
        mid=end-.3
        a=np.array([np.interp(mid-.1,times,xy[:,i]) for i in range(2)])
        b=np.array([np.interp(mid+.1,times,xy[:,i]) for i in range(2)])
        speed=float(np.linalg.norm(b-a)/.2)
        def at(t): return np.array([np.interp(t,times,xy[:,i]) for i in range(2)])
        v0=float(np.linalg.norm(at(start+.05)-at(start-.05))/.1)
        points=xy[(times>=start)&(times<=end)]
        excursion=float(np.max(np.linalg.norm(points-at(start),axis=1)))
        cfg=json.loads((folder/'truth.fixture.json').read_text())['config']
        bound=v0*cfg['reaction_time']+v0*v0/(2*cfg['braking'])+cfg['uncertainty']
        checks.append({'fault':kind,'start':start,'end':end,'commands_stopped':stopped,
                       'settling_sample_after_seconds':mid-start,'actual_speed_after_settling':speed,
                       'max_excursion':excursion,'stopping_bound':bound,
                       'passed':stopped and speed<.1 and excursion<=bound})
    events=[]
    for row in decisions:
        for e in row.get('events',[]):
            if e not in events: events.append(e)
    episodes=[]; entered=None
    first=commands[0]['t'] if commands else 0
    last=max((r.get('source_t',0) for r in decisions),default=0)
    for p in truth:
        if not first<=p['t']<=last: continue
        if p['inside'] and entered is None: entered=p['t']
        if not p['inside'] and entered is not None:
            episodes.append({'entered':entered,'exited':p['t'],'duration':p['t']-entered}); entered=None
    incursion_test='push_north' in detected
    if incursion_test:
        passed=bool(episodes and entered is None and max(e['duration'] for e in episodes)<=10 and
                    any(e['event']=='AUTONOMOUS_WARNING' for e in events) and
                    not any(e['event']=='DEADLINE_FAILURE' for e in events) and result['state']=='COMPLETE')
    else:
        passed=bool(checks and all(c['passed'] for c in checks) and result['state']=='COMPLETE')
    pause_report=None
    if (folder/'pause.json').exists():
        pause=json.loads((folder/'pause.json').read_text())
        paused=[c for c in commands if pause.get('wall_start',float('inf'))+1.5<=c.get('wall',0)<=pause.get('wall_end',0)]
        pause_report={'samples_after_watchdog':len(paused),
            'source_advance':max(c['t'] for c in paused)-min(c['t'] for c in paused) if paused else None,
            'commands_stopped':bool(paused and all(np.linalg.norm(c['command'])<1e-6 for c in paused))}
        pause_report['passed']=bool(paused and pause_report['source_advance']<=.05 and pause_report['commands_stopped']
            and 'true' in pause['pause_response'] and 'true' in pause['resume_response'])
        if not incursion_test and not checks: passed=result['state']=='COMPLETE'
        passed=passed and pause_report['passed']
    report={'state':result['state'],'stop_checks':checks,'residence_events':events,
            'physical_incursions':episodes,'physical_still_inside':entered is not None,
            'incursion_test':incursion_test,'pause_check':pause_report,'passed':passed}
    (folder/'fault_evaluation.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(); p.add_argument('folder',type=Path); a=p.parse_args()
    print(json.dumps(evaluate(a.folder),indent=2))
