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
print('target state:', t.get_state())
g = lambda k: t.read32(names[k])
print('boot: safety_ready=%d fault=%d guard=%d cantest=%d lift=%d PE2=%d' % (
    g('g_safety_ready'), g('g_safety_fault'), g('g_can_guard_ready'), g('g_can_test_result'),
    g('g_lift_ruler_state'), (t.read32(0x40021014) >> 2) & 1))
t.write32(names['g_startup_axes_vm_request'], 1)
for i in range(20):
    time.sleep(0.15)
    res = [t.read32(names['g_can_poll_result'] + 4 * j) for j in range(4)]
    cnt = [t.read32(names['g_can_poll_count'] + 4 * j) for j in range(4)]
    mask = g('g_can_poll_valid_mask'); gr = g('g_can_guard_ready'); fl = g('g_safety_fault')
    print('t=%4.1f res=%s cnt=%s mask=0x%X guard=%d fault=%d' % (0.15 * (i + 1), res, cnt, mask, gr, fl))
    if gr == 1:
        print('GUARD_READY_ALL4_OK')
        break
    if fl:
        print('FAULT')
        break
