"""VM-on CAN1 wheel-node readiness probe (no motion commands).

Turns VM on, samples per-node poll results at 100 ms, records guard/fault,
then leaves the board reset to a clean idle state. Saves evidence JSON.
"""
from pathlib import Path
import json
import time
import datetime
import source_stage2_readonly_sync_probe_20261105 as transport
from pyocd.core.helpers import ConnectHelper
from pyocd.core.target import Target

R = Path(__file__).resolve().parents[1]
IMAGE = R / 'tmp/integration12_stage2_vision_20261107/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf'
transport.live.IMAGE = IMAGE
names, _ = transport.live.load_image()
opt = {'target_override': 'cortex_m', 'connect_mode': 'attach', 'frequency': 1000000,
       'auto_unlock': False, 'resume_on_disconnect': False, 'no_config': True,
       'cmsis_dap.prefer_v1': False, 'cmsis_dap.limit_packets': True}


def open_session():
    s = ConnectHelper.session_with_chosen_probe(unique_id='6afaae24', options=opt)
    s.open()
    return s, s.target


s, t = open_session()


def r32(name):
    if name not in names:
        return None
    v = t.read32(names[name])
    if v >= (1 << 31):
        v -= (1 << 32)
    return v


def arr(name, n=4):
    if name not in names:
        return None
    return [r32_off(names[name] + 4 * i) for i in range(n)]


def r32_off(addr):
    v = t.read32(addr)
    if v >= (1 << 31):
        v -= (1 << 32)
    return v


out = {'tag': 'vm-on-wheel-can-probe', 'when': datetime.datetime.now().isoformat(timespec='seconds'),
       'image': str(IMAGE.relative_to(R)), 'note': 'VM on, no motion command sent',
       'before': {k: r32(k) for k in ('g_safety_ready', 'g_safety_fault', 'g_vm_test_request',
                                      'g_can_task_count', 'g_can_poll_valid_mask', 'g_can_guard_ready')},
       'samples': []}
c0 = r32('g_can_task_count')
t0 = time.time()
t.write32(names['g_vm_test_request'], 1)
for i in range(60):
    time.sleep(0.1)
    row = {
        'ms': int((time.time() - t0) * 1000),
        'vm': r32('g_vm_test_request'),
        'ready': r32('g_safety_ready'),
        'fault': r32('g_safety_fault'),
        'guard_ready': r32('g_can_guard_ready'),
        'guard_cause': r32('g_can_guard_cause'),
        'guard_bad': r32('g_can_guard_bad_nodes'),
        'valid_mask': r32('g_can_poll_valid_mask'),
        'result': arr('g_can_poll_result'),
        'count': arr('g_can_poll_count'),
        'age': arr('g_can_poll_age_ticks'),
        'task_count': r32('g_can_task_count'),
        'ctrl_fault': r32('g_can_ctrl_fault_latched'),
        'esr': r32('g_can_poll_esr'),
    }
    out['samples'].append(row)
    print('ms=%4d vm=%s ready=%s fault=%-3s guard=%s/%-2s bad=0x%X mask=0x%X res=%s cnt=%s task=%s'
          % (row['ms'], row['vm'], row['ready'], row['fault'], row['guard_ready'], row['guard_cause'],
             row['guard_bad'] or 0, row['valid_mask'] or 0, row['result'], row['count'], row['task_count']))
    if row['fault']:
        print('FAULT_LATCHED at ms=%d' % row['ms'])
        break
c1 = r32('g_can_task_count')
dur = time.time() - t0
out['task_hz'] = round((c1 - c0) / dur, 2) if (c0 is not None and c1 is not None) else None
print('can_task_hz=%.2f' % out['task_hz'] if out['task_hz'] else 'can_task_hz=?')
out['after'] = {k: r32(k) for k in ('g_safety_ready', 'g_safety_fault', 'g_vm_test_request',
                                    'g_can_poll_valid_mask', 'g_can_guard_ready',
                                    'g_can_guard_bad_nodes', 'g_can_ctrl_fault_latched')}

# leave the board clean: VM off, then reset only if a fault latched
t.write32(names['g_vm_test_request'], 0)
time.sleep(0.3)
t.reset(reset_type=Target.ResetType.SYSRESETREQ)
time.sleep(2.0)
s, t = open_session()
out['clean_after_reset'] = {k: r32(k) for k in ('g_safety_ready', 'g_safety_fault', 'g_vm_test_request',
                                               'g_can_ctrl_fault_latched')}
print('clean_after_reset=%s' % out['clean_after_reset'])

ev = R / 'evidence/integration/2026-11-07/three-piece-i12/i12-can-guard-probe-01.json'
ev.parent.mkdir(parents=True, exist_ok=True)
ev.write_text(json.dumps(out, indent=1, ensure_ascii=False), encoding='utf-8')
print('evidence=%s' % ev.relative_to(R))
