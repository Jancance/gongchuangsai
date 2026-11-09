"""Verify flashed image equals the built AXF (segment-by-segment), read-only."""
from pathlib import Path
import json, hashlib, datetime
import source_stage2_readonly_sync_probe_20261105 as transport
from pyocd.core.helpers import ConnectHelper
from pyocd.core.target import Target

R = Path('.').resolve()
IMAGE = R / 'tmp/integration12_stage2_vision_20261107/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf'
transport.live.IMAGE = IMAGE
symbols, segments = transport.live.load_image()
opt = {'target_override': 'cortex_m', 'connect_mode': 'attach', 'frequency': 1000000,
       'auto_unlock': False, 'resume_on_disconnect': False, 'no_config': True,
       'cmsis_dap.prefer_v1': False, 'cmsis_dap.limit_packets': True}
s = ConnectHelper.session_with_chosen_probe(unique_id='6afaae24', options=opt)
s.open(); t = s.target
out = {'tag': 'post-flash-verify', 'when': datetime.datetime.now().isoformat(timespec='seconds'),
       'image': str(IMAGE.relative_to(R)), 'image_sha256': hashlib.sha256(IMAGE.read_bytes()).hexdigest(),
       'core_state': t.get_state().name, 'mismatch': None}
compared = 0
for address, data in segments:
    for offset in range(0, len(data), 4096):
        expected = data[offset:offset + 4096]
        got = bytes(t.read_memory_block8(address + offset, len(expected)))
        if got != expected:
            out['mismatch'] = {'address': hex(address + offset), 'len': len(expected)}
            break
        compared += len(expected)
    if out['mismatch']:
        break
out['flash_matches'] = out['mismatch'] is None
out['flash_bytes_compared'] = compared
keys = ['g_safety_ready', 'g_safety_fault', 'g_can_ctrl_fault_latched', 'g_vm_test_request',
        'g_can_guard_ready', 'g_it_state', 'g_lift_ruler_state', 'g_lift_ruler_cause',
        'g_startup_axes_state', 'g_dm_can2_state', 'g_can_test_result']
out['state'] = {}
for k in keys:
    if k in symbols:
        v = t.read32(symbols[k])
        out['state'][k] = v
print(json.dumps(out, ensure_ascii=False, indent=1))
ev = R / 'evidence/integration/2026-11-07/three-piece-i12/post-flash-verify-canguard-20261107.json'
ev.parent.mkdir(parents=True, exist_ok=True)
ev.write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding='utf-8')
print('evidence=%s' % ev.relative_to(R))
