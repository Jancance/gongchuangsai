"""Explicit one-shot client for the separate 8-action combined candidate.

Default monitor is read-only. Run requires --action run-once --mode 3 --plan default.
Single-turn client and its policy remain in p1_serial_motion.py unchanged.
"""
import json
from pathlib import Path

import p1_serial_motion as serial_motion

ROWS=[{'kind': 'FORWARD', 'amount': 180, 'speed': 20, 'label': 'FORWARD'}, {'kind': 'RIGHT', 'amount': 180, 'speed': 20, 'label': 'RIGHT'}, {'kind': 'TURN', 'amount': -90, 'speed': 30, 'label': 'TURN'}, {'kind': 'TURN', 'amount': -90, 'speed': 30, 'label': 'TURN'}, {'kind': 'FORWARD', 'amount': 180, 'speed': 20, 'label': 'FORWARD'}, {'kind': 'TURN', 'amount': 90, 'speed': 30, 'label': 'TURN'}, {'kind': 'TURN', 'amount': 90, 'speed': 30, 'label': 'TURN'}, {'kind': 'LEFT', 'amount': 180, 'speed': 20, 'label': 'LEFT'}]
KINDS={'FORWARD':4,'LEFT':5,'RIGHT':6,'BACKWARD':7,'TURN':3}


class Perimeter8Controller(serial_motion.Controller):
    capture_drain_s=10.0
    def __init__(self,action='monitor',expected_mode=1,plan=None,route_trial=False,turn_trial=False):
        if route_trial or turn_trial:raise ValueError('Use the separate perimeter client without route/turn trial flags')
        if action=='run-once' and (expected_mode!=3 or plan!=0):
            raise ValueError('Perimeter start requires mode3 and explicit default plan')
        super().__init__(action,expected_mode,plan)

    def _speed_profile_healthy(self,r):
        return (r.get('debug_plan_id')==0 and r.get('sequence_count')==8 and
                r['feedback_limit_rpm']==80 and 3<=r['base_rpm']<=52 and not r.get('distance_mode'))

    def _healthy(self,r):
        return (r['phase']==0 and not r['trip'] and not r['backend_active'] and not r['backend_result'] and
                not r['vm_request'] and not r['pe2_active'] and r['source_valid']==1 and r['selftest']==1 and
                r['protocol_selftest']==1 and 0<=r['angle_age_ms']<100 and 0<=r['rate_age_ms']<100 and
                abs(r['rate_mdps'])<=3000 and r['motion_mode']==3 and self._speed_profile_healthy(r) and
                not any(r[k] for k in ['rtos_fault','safety_fault','can_fault']))

    def _running_shape(self,r):
        index=r.get('sequence_index',-1)
        if not 0<=index<8 or r.get('debug_plan_id')!=0 or r.get('sequence_count')!=8:return False
        row=ROWS[index];turn=row['kind']=='TURN';kind=KINDS[row['kind']]
        if r.get('action_kind')!=kind or r['feedback_limit_rpm']!=(40 if turn else 80):return False
        if turn and r['base_rpm']!=27:return False
        if not turn and not 0<=r['base_rpm']<=52:return False
        if turn and not 0 <= r['correction_rpm']*(1 if row['amount']>0 else -1) <=27:return False
        if r['backend_active']:
            if turn and r.get('distance_mode'):return False
            if not turn and (not r.get('distance_mode') or
                r.get('distance_target_milli_mm')!=round(row['amount']*10000)):return False
        return True

    def consume(self,r,now):
        if self.start_sent and self.status is None and self.action=='run-once':
            if r['phase'] in {4,5}:return super().consume(r,now)
            # The base class does the replay/clock check below. Completion here must
            # also require a genuinely newer frame, not a repeated cached terminal.
            fresh=self.last_seq is None or 0<((r['seq']-self.last_seq)&0xFFFFFFFF)<0x80000000
            if fresh and not self._running_shape(r):return self.abort('perimeter8_shape_changed')
            if fresh and (r['phase']==3 and not r['trip'] and r['backend_result']==1 and
                not r['backend_cause'] and r['stop_mask']==15 and not r['vm_request'] and not r['pe2_active'] and
                r.get('sequence_state')==3 and r.get('sequence_completed')==8 and r.get('distance_mode') and r.get('distance_reached') and
                not any(r[k] for k in ['rtos_fault','safety_fault','can_fault'])):
                self.last_seq=r['seq'];self.last_rx=now;self.status='software_complete';self.stage='finished';return []
            if fresh and r['phase']==3:return self.abort('perimeter8_incomplete_terminal')
        return super().consume(r,now)


def main():
    serial_motion.Controller=Perimeter8Controller
    return serial_motion.main()


if __name__=='__main__':raise SystemExit(main())
