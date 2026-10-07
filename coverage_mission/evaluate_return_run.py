"""Independent return-flight evidence evaluation; never imported by control."""
import argparse
import json
import math
from pathlib import Path
import statistics
from .evaluate_qr_run import truth_velocities
from .return_mission import ReturnConfig


def evaluate_entry(directory):
    """Targeted fixture only: deliberately no initial/target QR pass claim."""
    load=lambda name:json.loads((directory/name).read_text())
    truth=[json.loads(line) for line in (directory/'truth.jsonl').open()]
    final=load('coverage.result.json'); landing=load('coverage.landing.json')
    p=ReturnConfig(**load('coverage.manifest.json')['return_config'])
    home_z=statistics.median(r['z'] for r in truth if r['z']<.5 and r['t']<20)
    terminal=[r for r in truth if r['t']>=truth[-1]['t']-1]
    corridor=[r for r in truth if -30.1<=r['n']<=-20.1]
    gap=max((b['t']-a['t'] for a,b in zip(truth,truth[1:])),default=math.inf)
    log=(directory/'manager.log').read_text()
    checks={
        'manager_success':load('harness.json')['manager_exit']==0,
        'positive_orange_entry':final['state']=='RETURN_ENTRY_READY' and final['exit_code']==0,
        'reverse_native_exit':'[RETURN] Corridor exited;' in log,
        'no_sampled_red_incursion':bool(truth) and not any(r['inside'] for r in truth),
        'roof_origin_clearance':bool(corridor) and min(3.05-r['z'] for r in corridor)>=.25,
        'wall_body_clearance':bool(corridor) and all(abs(r['e']-4.101)<=(3.5/2-.4) for r in corridor),
        'independent_exterior_touchdown':bool(terminal) and all(
            r['n']<=-30.1-p.landing_clearance+.3 and abs(r['z']-home_z)<.10 for r in terminal),
        'fresh_fc_touchdown':landing['state']=='COMPLETE' and not landing['armed'] and landing['landed_state']==1,
        'truth_sampling':gap<=.1,
    }
    result={'kind':'targeted_return_entry','pass':all(checks.values()),'checks':checks,
        'truth_max_gap_s':gap,'landing':landing,'home_model_z':home_z,
        'minimum_roof_model_origin_gap_m':min((3.05-r['z'] for r in corridor),default=None)}
    (directory/'return_evaluation.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


def evaluate(directory):
    def load(name): return json.loads((directory/name).read_text())
    decisions=[json.loads(line) for line in (directory/'coverage.jsonl').open()]
    truth=[json.loads(line) for line in (directory/'truth.jsonl').open()]
    first=load('initial_qr.result.json'); final=load('coverage.result.json')
    landing=load('coverage.landing.json'); harness=load('harness.json')
    reference=first['last_decision']['qr_reference']
    event=next(e for e in final['last_decision']['qr_events'] if e['state']=='TARGET_MATCH')
    marker=next(m for m in load('qr_fixture.json') if m['name']!='qr_start' and m['payload']==reference)
    target=marker['world_enu'][1::-1]
    start=next(d for d in decisions if d['state']=='RETURN_START')
    hold_since=start['delivery_hold_since']
    held=[r for r in truth if hold_since<=r['t']<=start['source_t']]
    # Stable pre-takeoff model origin is independent of FC altitude reports.
    floor=[r['z'] for r in truth if r['t']<first['last_decision']['source_t']-10 and r['z']<.5]
    home_z=statistics.median(floor) if floor else None
    speeds=truth_velocities(held)
    field=[r for r in truth if decisions[0]['source_t']<=r['t']<=final['last_decision']['source_t']]
    transit=[r for r in field if r['z']>9.]
    after=[d for d in decisions if d['source_t']>=event['t']]
    terminal=[r for r in truth if r['t']>=truth[-1]['t']-1]
    corridor=[r for r in truth if r['t']>=final['last_decision']['source_t'] and -30.1<=r['n']<=-20.1]
    p=ReturnConfig(**load('coverage.manifest.json')['return_config'])
    def along(r):
        return ((r['n']-p.entrance_n)*math.cos(p.heading)+
                (r['e']-p.entrance_e)*math.sin(p.heading))
    log=(directory/'manager.log').read_text()
    gap=max((b['t']-a['t'] for a,b in zip(truth,truth[1:])),default=math.inf)
    checks={
        'manager_and_harness_success':harness['manager_exit']==0,
        'initial_reference_read':first['state']=='REFERENCE_READY',
        'exact_matching_identity':len({r['seq'] for r in event['reads']})>=3 and
            all(r['payload']==reference for r in event['reads']),
        'five_second_measured_dwell':start['source_t']-hold_since>=5.,
        'independent_hold_alignment':bool(held) and max(math.dist((r['n'],r['e']),target) for r in held)<=.25,
        'independent_hold_altitude':home_z is not None and bool(held) and max(abs(r['z']-home_z-5) for r in held)<=.20,
        'independent_hold_stationary':bool(speeds) and max(max(v) for v in speeds)<=.10,
        'coverage_cancelled':start['pending']>0 and not any(d['state'] in
            ('SWEEP','COVERAGE_REPAIR','QR_CENTER','QR_READ') for d in after),
        'return_ascent_and_route':any(d['state']=='RETURN_ASCEND' for d in decisions) and
            any(d['state']=='RETURN_ROUTE' and abs(d['altitude']-10)<=.25 for d in decisions),
        'no_sampled_field_red_incursion':bool(field) and not any(r['inside'] for r in field),
        'no_sampled_transit_fence_violation':bool(transit) and not any(r['outside'] for r in transit),
        'orange_geometry_handoff':final['state']=='RETURN_ENTRY_READY' and final['exit_code']==0,
        'two_corridor_exits':log.count('EXIT_DETECTION COMPLETE')>=1 and
            '[RETURN] Corridor exited;' in log,
        'return_roof_origin_clearance':bool(corridor) and min(3.05-r['z'] for r in corridor)>=.25,
        'return_wall_body_clearance':bool(corridor) and all(abs(r['e']-4.101)<=(3.5/2-.4) for r in corridor),
        'fc_landing_confirmed':landing['state']=='COMPLETE' and not landing['armed'] and landing['landed_state']==1,
        'independent_exterior_touchdown':home_z is not None and bool(terminal) and
            all(along(r)>=p.far_mouth_distance+p.landing_clearance-.3 and
                abs(r['z']-home_z)<.10 for r in terminal),
        'truth_sampling':len(truth)>100 and gap<=.10,
    }
    report={'pass':all(checks.values()),'checks':checks,'truth_max_gap_s':gap,
        'hold_seconds':start['source_t']-hold_since,'truth_hold_samples':len(held),
        'max_truth_hold_error_m':max((math.dist((r['n'],r['e']),target) for r in held),default=None),
        'max_truth_hold_speed_m_s':max((max(v) for v in speeds),default=None),
        'landing':landing,'home_model_z':home_z}
    report['minimum_return_roof_model_origin_gap_m']=min((3.05-r['z'] for r in corridor),default=None)
    (directory/'return_evaluation.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__); p.add_argument('directory',type=Path)
    directory=p.parse_args().directory
    scenario=json.loads((directory/'harness.json').read_text())['scenario']
    report=evaluate_entry(directory) if scenario=='return-entry' else evaluate(directory)
    print(json.dumps(report,indent=2)); return 0 if report['pass'] else 1


if __name__=='__main__': raise SystemExit(main())
