"""Integrated-profile gate test: one lift return-to-top (IT request 3) only.

Proves the full-vehicle profile actually services the LiftRuler controller and
that VM ownership does not fight. No descent, no arm, no gripper, no carrier.
"""
from pathlib import Path
import json, time, datetime, struct
import source_stage2_readonly_sync_probe_20261105 as transport
from pyocd.core.helpers import ConnectHelper
from pyocd.core.target import Target
R = Path('.').resolve()
IMAGE = R / 'tmp/integration12_stage2_vision_20261107/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf'
transport.live.IMAGE = IMAGE
symbols, _ = transport.live.load_image()
opt = {'target_override': 'cortex_m', 'connect_mode': 'attach', 'frequency': 1000000,
       'auto_unlock': False, 'resume_on_disconnect': False, 'no_config': True,
       'cmsis_dap.prefer_v1': False, 'cmsis_dap.limit_packets': True}
s = ConnectHelper.session_with_chosen_probe(unique_id='6afaae24', options=opt)
s.open(); t = s.target
def r(k):
    if k not in symbols: return None
    return t.read32(symbols[k])
def fl(k):
    if k not in symbols: return None
    return struct.unpack('<f', struct.pack('<I', t.read32(symbols[k])))[0]
KEYS = ('g_safety_ready','g_safety_fault','g_can_ctrl_fault_latched','g_vm_test_request',
        'g_it_state','g_it_cause','g_it_rejected','g_lift_ruler_state','g_lift_ruler_phase',
        'g_lift_ruler_cause','g_lift_ruler_vm_request','g_lift_ruler_rejected',
        'g_lift_compare_operation','g_lift_compare_top_valid','g_lift_compare_backoff_valid')
out = {'tag': 'integrated-lift-return-02', 'when': datetime.datetime.now().isoformat(timespec='seconds'),
       'image_sha256': __import__('hashlib').sha256(IMAGE.read_bytes()).hexdigest(),
       'action': 'g_it_request=4 (LiftRuler_StartReturn: home to top, builds top reference)', 'samples': []}
def snap(tag):
    d = {'tag': tag, 't': round(time.time()-t0, 3)}
    for k in KEYS: d[k] = r(k)
    d['lift_deg'] = fl('g_vehicle_lift_current_deg')
    d['target_pulses'] = r('g_lift_compare_target_pulses')
    return d
t0 = time.time()
out['before'] = snap('before')
print('BEFORE', json.dumps(out['before'], ensure_ascii=False))
t.write32(symbols['g_it_request'], 4)
faulted = False
for i in range(200):
    time.sleep(0.1)
    d = snap('run')
    out['samples'].append(d)
    if i % 5 == 0:
        print('t=%5.2f it=%s/%s/%s lift=%s/%s/%s vm=%s|%s f=%s deg=%s'
              % (d['t'], d['g_it_state'], d['g_it_cause'], d['g_it_rejected'],
                 d['g_lift_ruler_state'], d['g_lift_ruler_phase'], d['g_lift_ruler_cause'],
                 d['g_lift_ruler_vm_request'], d['g_vm_test_request'], d['g_safety_fault'], d['lift_deg']))
    if d['g_safety_fault'] or d['g_can_ctrl_fault_latched']:
        print('FAULT at t=%.2f' % d['t']); faulted = True; break
    if d['g_it_state'] == 2 and d['g_lift_ruler_state'] in (0, 4):
        print('DONE at t=%.2f' % d['t']); break
out['after'] = snap('after')
out['faulted'] = faulted
print('AFTER', json.dumps(out['after'], ensure_ascii=False))
ev = R / 'evidence/integration/2026-11-07/three-piece-i12/integrated-lift-return-02.json'
ev.write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding='utf-8')
print('evidence=%s' % ev.relative_to(R))
