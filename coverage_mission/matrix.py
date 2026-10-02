"""Serial commissioning matrix; all aircraft processes belong to each campaign.

This is not an unattended real-flight runner. Source/config manifests and failed
cases are retained. A failed case never becomes a passing result by being skipped.
"""
import argparse
import json
from pathlib import Path
from .campaign import run, ROOT
from .evaluate_faults import evaluate as evaluate_faults


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--group',choices=['nominal','faults','full'],required=True)
    parser.add_argument('--instance',type=int,default=0)
    parser.add_argument('--cases',nargs='+',help='Run only these named cases (fresh evidence directory still required)')
    args=parser.parse_args(); output=args.output.resolve(); output.mkdir(parents=True,exist_ok=False)
    smoke=ROOT/'config/smoke.json'; rows=[]
    if args.group=='nominal':
        cases=[(f'{scene}_{i}',scene,smoke,yaw,None,None,'COMPLETE')
               for scene in ('multiple','row_end','edge','partial','clear')
               for i,yaw in enumerate((0,.2,-.2))]
        cases += [('wide_'+str(i),'multiple',ROOT/'config/wide.json',yaw,None,None,'COMPLETE')
                  for i,yaw in enumerate((0,.2,-.2))]
        cases += [('noise','multiple',smoke,0,ROOT/'config/fault_noise.json',None,'COMPLETE')]
    elif args.group=='faults':
        cases=[(name,'clear',smoke,0,ROOT/f'config/fault_{name}.json',None,'ABORTED')
               for name in ('yaw','position','clock')]
        cases += [('deadline','incursion',smoke,0,ROOT/'config/fault_deadline.json',None,'ABORTED'),
                  ('incursion_pause','incursion',smoke,0,ROOT/'config/fault_incursion.json',5,'COMPLETE'),
                  ('pause','clear',smoke,0,None,10,'COMPLETE'),
                  ('recoverable','clear',smoke,0,ROOT/'config/fault_recoverable.json',None,'COMPLETE')]
    else:
        cases=[('full_multiple','multiple',ROOT/'config/coverage.json',0,None,None,'COMPLETE')]
    if args.cases:
        unknown=set(args.cases)-{c[0] for c in cases}
        if unknown: parser.error('Unknown cases: '+', '.join(sorted(unknown)))
        cases=[c for c in cases if c[0] in args.cases]
    for name,scene,cfg,yaw,faults,pause,expected in cases:
        folder=output/name
        print('START '+name,flush=True)
        try:
            entry=(.2,.2) if yaw>0 else ((-.2,.2) if yaw<0 else (0.,0.))
            result=run(cfg,scene,folder,2400 if args.group=='full' else 600,2,yaw,faults,pause,instance=args.instance,entry=entry)
            passed=result['controller_state']==expected and result['outside_samples']==0
            passed=passed and result['whole_interval_recorded'] and result['max_truth_gap'] is not None and result['max_truth_gap']<=.05
            if expected=='COMPLETE':
                passed=passed and result.get('sampled_geometry_coverage_passed',False) and result.get('original_025m_projection_target_passed',False)
                passed=passed and result.get('credited_route_tracking_passed',False)
                if result.get('straight_leg_samples',0): passed=passed and result['straight_leg_tracking_passed']
                passed=passed and result['whole_interval_recorded'] and result['max_truth_gap']<=.05
                if scene!='incursion': passed=passed and result['incursion_samples']==0
            if scene=='incursion' and expected=='COMPLETE': passed=passed and evaluate_faults(folder)['passed']
            if name in ('pause','recoverable'): passed=passed and evaluate_faults(folder)['passed']
            if name=='deadline':
                fault_result=evaluate_faults(folder)
                events={e['event']:e for e in fault_result['residence_events']}
                passed=passed and all(k in events for k in ('ENTER','AUTONOMOUS_WARNING','DEADLINE_FAILURE','EXIT'))
                if passed:
                    passed=(5<=events['AUTONOMOUS_WARNING']['t']-events['ENTER']['t']<=5.3 and
                            10<=events['DEADLINE_FAILURE']['t']-events['ENTER']['t']<=10.3 and
                            not fault_result['physical_still_inside'])
            rows.append({'case':name,'passed':bool(passed),'result':result})
        except Exception as exc:
            rows.append({'case':name,'passed':False,'error':str(exc)})
        (output/'summary.json').write_text(json.dumps(rows,indent=2)+'\n')
        print('END '+name+' '+str(rows[-1]['passed']),flush=True)
    return 0 if all(r['passed'] for r in rows) else 1


if __name__=='__main__': raise SystemExit(main())
