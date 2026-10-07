"""Read-only early-target oracle; incomplete coverage is expected after a match."""
import argparse
import json
import math
import statistics
from pathlib import Path


def truth_velocities(records, minimum_dt=.05):
    """Finite differences over a real >=50 ms baseline, not adjacent 60 Hz pairs."""
    result=[]
    if not records: return result
    a=records[0]
    for b in records[1:]:
        dt=b['t']-a['t']
        if dt>=minimum_dt:
            result.append((math.dist((a['n'],a['e']),(b['n'],b['e']))/dt,
                           abs(a['z']-b['z'])/dt))
            a=b
    return result


def evaluate(directory,markers):
    initial=json.loads((directory/'initial_qr.result.json').read_text())
    final=json.loads((directory/'coverage.result.json').read_text())
    first=initial['last_decision']; last=final['last_decision']
    reference=first['qr_reference']
    matches=[m for m in markers if m['name']!='qr_start' and m['payload']==reference]
    event=next(e for e in last['qr_events'] if e['state']=='TARGET_MATCH')
    selected=min(matches,key=lambda m:math.dist(event['xy'],m['world_enu'][1::-1]))
    truth=[json.loads(line) for line in (directory/'truth.jsonl').open()]
    source_end=last['source_t']
    terminal=[r for r in truth if source_end-2<=r['t']<=source_end+.15]
    field=[r for r in truth if r['t']>=next(
        json.loads(line)['source_t'] for line in (directory/'coverage.jsonl').open())]
    target=selected['world_enu'][1::-1]
    distances=[math.dist((r['n'],r['e']),target) for r in terminal]
    ground_samples=[r['z'] for r in truth if r['z']<.5 and r['t']<first['source_t']-10]
    home_model_z=statistics.median(ground_samples) if ground_samples else None
    truth_altitudes=[r['z']-home_model_z for r in terminal] if home_model_z is not None else []
    max_gap=max((b['t']-a['t'] for a,b in zip(field,field[1:])),default=None)
    independent_speed=truth_velocities(terminal)
    initial_reads=next(e['reads'] for e in first['qr_events'] if e['state']=='REFERENCE_READY')
    checks={
        'initial_reference':initial['state']=='REFERENCE_READY' and reference==next(
            m['payload'] for m in markers if m['name']=='qr_start'),
        'three_distinct_start_exposures':len({r['seq'] for r in initial_reads})>=3,
        'three_distinct_match_exposures':len({r['seq'] for r in event['reads']})>=3,
        'exact_identity':all(r['payload']==reference for r in event['reads']),
        'nonmatch_inspected':any(e['state']=='NONMATCH' for e in last['qr_events']),
        'terminal_kind':final['exit_code']==0 and final['state']=='TARGET_HOLD_5M',
        'coverage_cancelled_early':last['pending']>0,
        'no_sampled_field_red_incursions':bool(field) and not any(r['inside'] for r in field),
        'no_sampled_field_fence_violations':bool(field) and not any(r['outside'] for r in field),
        'truth_sampling':bool(field) and max_gap is not None and max_gap<=.1,
        'independent_target_alignment':bool(distances) and max(distances)<=.25,
        'measured_home_altitude':abs(last['altitude']-5)<=.15,
        'independent_altitude':bool(truth_altitudes) and max(abs(h-5) for h in truth_altitudes)<=.20,
        'measured_stationary':math.hypot(*last['measured_velocity'][:2])<=.10 and abs(last['measured_velocity'][2])<=.10,
        'independent_stationary':bool(independent_speed) and max(max(s) for s in independent_speed)<=.10,
    }
    decisions=[json.loads(line) for line in (directory/'coverage.jsonl').open()]
    after=[d for d in decisions if d['source_t']>=event['t']]
    checks['no_search_after_match']=bool(after) and not any(d['state'] in (
        'SWEEP','COVERAGE_REPAIR','QR_CENTER','QR_READ') for d in after)
    report={'pass':all(checks.values()),'checks':checks,'fixture_target':selected['name'],
        'max_truth_target_error_m':max(distances,default=None),'field_truth_max_gap_s':max_gap,
        'terminal_truth_samples':len(terminal),'source_end':source_end,
        'pending_coverage_points':last['pending'],'max_terminal_truth_speed_m_s':
            max((max(s) for s in independent_speed),default=None),
        'reference_hash':last['qr_reference_hash']}
    report['independent_home_model_z']=home_model_z
    report['terminal_truth_altitude_range']=([min(truth_altitudes),max(truth_altitudes)] if truth_altitudes else None)
    (directory/'qr_evaluation.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


def main():
    p=argparse.ArgumentParser(); p.add_argument('directory',type=Path)
    p.add_argument('--markers',type=Path,help='Independent fixture manifest; defaults to the retained run snapshot')
    args=p.parse_args(); markers=args.markers or args.directory/'qr_fixture.json'
    report=evaluate(args.directory,json.loads(markers.read_text()))
    print(json.dumps(report,indent=2))
    return 0 if report['pass'] else 1


if __name__=='__main__': raise SystemExit(main())
