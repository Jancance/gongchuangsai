"""Zero-motion probe: is LiftRuler_DebugService serviced in the full-vehicle profile?

Writing a STOP request to an IDLE controller must be consumed (request->0) if the
service runs. No motion, no VM, no power change.
"""
from pathlib import Path
import json, time, datetime
import source_stage2_readonly_sync_probe_20261105 as transport
from pyocd.core.helpers import ConnectHelper
R = Path('.').resolve()
transport.live.IMAGE = R / 'tmp/integration12_stage2_vision_20261107/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf'
names, _ = transport.live.load_image()
opt = {'target_override': 'cortex_m', 'connect_mode': 'attach', 'frequency': 1000000,
       'auto_unlock': False, 'resume_on_disconnect': False, 'no_config': True,
       'cmsis_dap.prefer_v1': False, 'cmsis_dap.limit_packets': True}
s = ConnectHelper.session_with_chosen_probe(unique_id='6afaae24', options=opt)
s.open(); t = s.target
def r(k): return t.read32(names[k])
out = {'tag': 'lift-service-probe-01', 'when': datetime.datetime.now().isoformat(timespec='seconds'),
       'note': 'STOP request on IDLE controller; consumed == service runs; no motion'}
out['before'] = {'g_lift_ruler_request': r('g_lift_ruler_request'), 'state': r('g_lift_ruler_state'),
                 'vm': r('g_vm_test_request'), 'fault': r('g_safety_fault')}
t.write32(names['g_lift_ruler_request'], 2)
out['trace'] = []
for i in range(20):
    time.sleep(0.05)
    out['trace'].append({'ms': int((i+1)*50), 'request': r('g_lift_ruler_request'),
                         'state': r('g_lift_ruler_state'), 'cause': r('g_lift_ruler_cause'),
                         'stop_pending': r('g_lift_ruler_stop_pending'),
                         'vm': r('g_vm_test_request'), 'fault': r('g_safety_fault')})
    if out['trace'][-1]['request'] == 0:
        break
last = out['trace'][-1]
out['consumed'] = last['request'] == 0
out['after'] = {'state': r('g_lift_ruler_state'), 'cause': r('g_lift_ruler_cause'),
                'vm': r('g_vm_test_request'), 'fault': r('g_safety_fault'),
                'safety_ready': r('g_safety_ready')}
print(json.dumps(out, ensure_ascii=False, indent=1))
ev = R / 'evidence/integration/2026-11-07/three-piece-i12/lift-service-probe-01.json'
ev.write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding='utf-8')
print('evidence=%s' % ev.relative_to(R))
