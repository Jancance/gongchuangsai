import time
from pathlib import Path
import source_stage2_readonly_sync_probe_20261105 as transport
from pyocd.core.helpers import ConnectHelper
from pyocd.core.target import Target
R = Path('.').resolve()
transport.live.IMAGE = R / 'tmp/integration12_stage2_vision_20261107/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf'
names, _ = transport.live.load_image()
opt = {'target_override': 'cortex_m', 'connect_mode': 'attach', 'frequency': 1000000,
       'auto_unlock': False, 'resume_on_disconnect': False, 'no_config': True,
       'cmsis_dap.prefer_v1': False, 'cmsis_dap.limit_packets': True}
s = ConnectHelper.session_with_chosen_probe(unique_id='6afaae24', options=opt)
s.open(); t = s.target
g = lambda k: t.read32(names[k])
t.write32(names['g_startup_axes_vm_request'], 1)
best = None
for i in range(24):
    time.sleep(0.15)
    res = [g('g_can_poll_result' + ('' if j == 0 else '') ) if False else None for j in range(4)]
    # arrays: read base address of g_can_poll_result
    base = names['g_can_poll_result']
    res = [t.read32(base + 4 * j) for j in range(4)]
    esr = g('g_can_poll_esr')
    vmask = g('g_can_poll_valid_mask')
    cnt = [t.read32(names['g_can_poll_count'] + 4 * j) for j in range(4)]
    print('t=%4.1f result=%s esr=0x%08X(LEC=%d TEC=%d) mask=%d cnt=%s RF0R=0x%08X' % (
        0.15 * (i + 1), res, esr, esr & 7, (esr >> 24) & 0xFF, vmask, cnt, t.read32(0x4000640C)))
    if any(c > 0 for c in cnt):
        print('SOME_NODE_OK')
        break
    if g('g_safety_fault'):
        print('FAULT')
        break
