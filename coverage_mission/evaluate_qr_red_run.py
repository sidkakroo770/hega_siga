"""Independent oracle for short, owned QR/red-boundary Gazebo fixtures."""
import argparse
import json
import math
import statistics
from pathlib import Path
import numpy as np
from .evaluate_qr_run import truth_velocities


def records(path):
    return [json.loads(line) for line in path.open() if line.strip()]


def evaluate(directory):
    harness=json.loads((directory/'harness.json').read_text())
    result=json.loads((directory/'coverage.result.json').read_text())
    decisions=records(directory/'coverage.jsonl')
    qr=records(directory/'coverage.qr.jsonl')
    truth=records(directory/'truth.jsonl')
    fixtures=json.loads((directory/'qr_fixture.json').read_text())
    fixture=next(m for m in fixtures if m['name']=='qr_target_early')
    target=fixture['world_enu'][1::-1]
    field=[r for r in truth if decisions[0]['source_t']<=r['t']<=decisions[-1]['source_t']+.15]
    gap=max((b['t']-a['t'] for a,b in zip(field,field[1:])),default=math.inf)
    shared={round(d['frame_t'],6) for d in decisions}&{round(r['t'],6) for r in qr}
    with np.load(directory/'coverage.map.npz') as saved:
        red_cells=int(saved['red'].sum())
        confirmed=int(saved['confirmed'].sum())
    checks={
        'independent_truth_sampling':bool(field) and gap<=.1,
        'no_body_red_incursions_at_any_field_altitude':bool(field) and not any(r['inside'] for r in field),
        'no_field_fence_violations':bool(field) and not any(r['outside'] for r in field),
        'same_exposure_used_by_qr_and_mapping':len(shared)>=3,
        'live_red_detected_and_confirmed':red_cells>0 and confirmed>0,
        'diagnostic_logs_no_loss':result['jsonl_dropped']==0 and result['jsonl_error'] is None,
    }
    metrics={'field_truth_samples':len(field),'truth_max_gap_s':gap,
             'minimum_body_red_clearance_m':min((r['red_clearance'] for r in field),default=None),
             'shared_exposure_timestamps':len(shared),'red_cells':red_cells,
             'confirmed_red_cells':confirmed,'qr_worker':result['qr_worker']}
    if harness['scenario'] in ('qr-near-red','qr-textured-approach'):
        match=next((e for d in decisions for e in d.get('qr_events',[])
                    if e['state']=='TARGET_MATCH'),None)
        terminal=[r for r in field if decisions[-1]['source_t']-2<=r['t']]
        home=statistics.median(r['z'] for r in truth if r['t']<decisions[0]['source_t'] and r['z']<.5)
        speeds=truth_velocities(terminal)
        descent=[d for d in decisions if d['state']=='TARGET_DESCEND']
        reads=match['reads'] if match else []
        read_sources={r['seq']:r for r in qr}
        checks.update(
            terminal_success=result['state']=='TARGET_HOLD_5M' and result['exit_code']==0 and harness['manager_exit']==0,
            three_distinct_matching_reads=len({r['seq'] for r in reads})>=3 and all(r['payload']==fixture['payload'] for r in reads),
            stationary_read_sources=bool(reads) and all(
                r['seq'] in read_sources and read_sources[r['seq']]['decode'] and
                math.hypot(read_sources[r['seq']]['pose']['vn'],read_sources[r['seq']]['pose']['ve'])<=.10 and
                abs(read_sources[r['seq']]['pose']['vd'])<=.10 for r in reads),
            centering_exercised=any(d['state']=='QR_CENTER' and math.hypot(d['vn'],d['ve'])>.01 for d in decisions),
            independent_target_alignment=bool(terminal) and max(math.dist([r['n'],r['e']],target) for r in terminal)<=.25,
            independent_five_metre_altitude=bool(terminal) and max(abs(r['z']-home-5) for r in terminal)<=.20,
            independent_stationary=bool(speeds) and max(max(s) for s in speeds)<=.10,
            red_mapping_continues_during_descent=bool(descent) and
                len({d['frame_t'] for d in descent})>=10 and
                any(d.get('observation_pose',{}).get('alt',10)<7 for d in descent),
            sweep_not_resumed_after_match=match is not None and not any(
                d['state'] in ('SWEEP','COVERAGE_REPAIR','QR_CENTER','QR_READ')
                for d in decisions if d['source_t']>match['t']),
        )
        metrics['max_terminal_target_error_m']=max((math.dist([r['n'],r['e']],target) for r in terminal),default=None)
        metrics['max_terminal_truth_speed_m_s']=max((max(s) for s in speeds),default=None)
        metrics['descent_distinct_mapped_exposures']=len({d['frame_t'] for d in descent})
        if harness['scenario']=='qr-textured-approach':
            events={ (e['t'],e['id']):e for d in decisions for e in d.get('qr_events',[])
                     if e['state']=='QR_SELECTED' }
            locations={c['id']:c['xy'] for d in decisions for c in d.get('qr_candidates',[])}
            checks['nonmatch_inspected_before_target']=any(e['state']=='NONMATCH'
                for d in decisions for e in d.get('qr_events',[]))
            checks['all_selected_candidates_are_real_markers']=bool(events) and all(
                min(math.dist(locations[e['id']],m['world_enu'][1::-1]) for m in fixtures)<=.30
                for e in events.values())
            failures={(e['t'],e['id']):e for d in decisions for e in d.get('qr_events',[])
                      if e['state'] in ('READ_ATTEMPT_EXPIRED','QR_CENTER_NO_PROGRESS','QR_LOST')}
            checks['no_false_candidate_inspection_churn']=all(
                min(math.dist(locations[e['id']],m['world_enu'][1::-1]) for m in fixtures)<=.30
                and e['attempt']<=2 for e in failures.values())
            metrics['selected_candidate_count']=len(events)
            metrics['real_marker_failed_attempts']=list(failures.values())
    else:
        excluded=[d for d in decisions if any(c['status']=='EXCLUDED' for c in d.get('qr_candidates',[]))]
        checks.update(
            controlled_short_negative_test=harness['controlled_stop_after_exclusion'],
            unsafe_target_excluded=bool(excluded),
            no_target_confirmation=not any(d.get('qr_match') for d in decisions),
            no_target_descent=not any(d['state'] in ('TARGET_DESCEND','TARGET_HOLD_5M') for d in decisions),
            no_inspection_of_excluded_target=not any(d['state'] in ('QR_CENTER','QR_SETTLE','QR_READ') for d in excluded),
            no_payload_decode_jobs=result['qr_worker']['decode_jobs']==0,
            stayed_at_coverage_altitude=bool(decisions) and min(d['altitude'] for d in decisions)>=9.5,
        )
        metrics['excluded_observation_samples']=len(excluded)
    metrics['mapping_processing_median_ms']=statistics.median(d['processing_ms'] for d in decisions)
    metrics['mapping_processing_p95_ms']=float(np.percentile([d['processing_ms'] for d in decisions],95))
    metrics['qr_processing_p95_ms']=float(np.percentile([r['processing_ms'] for r in qr],95)) if qr else None
    report={'pass':all(checks.values()),'scenario':harness['scenario'],'checks':checks,'metrics':metrics,
            'scope':'Targeted field fixture with injected reference; not initial QR, corridor, return or Pi proof.'}
    (directory/'qr_red_evaluation.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    report=evaluate(parser.parse_args().directory)
    print(json.dumps(report,indent=2))
    raise SystemExit(0 if report['pass'] else 1)
