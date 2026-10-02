"""Explicit SITL-only fault injection; not imported unless the test flag is used."""
import json
import os


class Faults:
    allowed={'camera_drop','camera_delay','black_frame','telemetry_drop','ekf_invalid',
             'yaw_jump','position_jump','clock_reset','worker_delay','push_north','actuator_hold','pose_noise'}

    def __init__(self,path):
        if os.environ.get('COVERAGE_TEST_INJECTION')!='1':
            raise ValueError('Fault injection requires the owned SITL campaign harness')
        self.events=json.loads(path.read_text())
        for event in self.events:
            if event['kind'] not in self.allowed or event['start']<0 or event['duration']<=0:
                raise ValueError('Invalid fault schedule')

    def active(self,t):
        return {e['kind']:e.get('value',0) for e in self.events
                if e['start']<=t<e['start']+e['duration']}
