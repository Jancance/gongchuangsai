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
print('before: safety_ready=%d fault=%d guard_ready=%d bad=%d' % (
    t.read32(names['g_safety_ready']), t.read32(names['g_safety_fault']),
    t.read32(names['g_can_guard_ready']), t.read32(names['g_can_guard_bad_nodes'])))
t.reset(reset_type=Target.ResetType.SYSRESETREQ)
print('reset issued')
time.sleep(2.0)
s.open()
for i in range(40):
    try:
        sr = t.read32(names['g_safety_ready']); sf = t.read32(names['g_safety_fault'])
        gr = t.read32(names['g_can_guard_ready']); bn = t.read32(names['g_can_guard_bad_nodes'])
        ct = t.read32(names['g_can_test_result']); ls = t.read32(names['g_lift_ruler_state'])
        print('t=%5.1fs ready=%d fault=%d guard=%d bad=%d cantest=%d lift=%d' % (2.0 + i * 0.5, sr, sf, gr, bn, ct, ls))
        if sr == 1:
            print('PLATFORM_READY')
            break
    except Exception as e:
        print('t=%5.1fs read err %r' % (2.0 + i * 0.5, e))
    time.sleep(0.5)
