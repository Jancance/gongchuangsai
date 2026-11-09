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
t.write32(names['g_vm_test_request'], 0)
time.sleep(1.0)
t.reset(reset_type=Target.ResetType.SYSRESETREQ)
time.sleep(2.5)
s.open()
print('after reset: ready=%d fault=%d guard=%d bad=%d' % (g('g_safety_ready'), g('g_safety_fault'), g('g_can_guard_ready'), g('g_can_guard_bad_nodes')))
t.write32(names['g_vm_test_request'], 1)
ok = False
for i in range(20):
    time.sleep(0.7)
    gr = g('g_can_guard_ready'); bn = g('g_can_guard_bad_nodes'); sf = g('g_safety_fault')
    pe2 = (t.read32(0x40021014) >> 2) & 1
    print('t=%4.1f PE2=%d guard=%d bad=%d fault=%d ready=%d' % (0.7 * (i + 1), pe2, gr, bn, sf, g('g_safety_ready')))
    if gr == 1:
        ok = True
        print('GUARD_READY')
        break
    if sf:
        print('FAULT_LATCHED bad=%d' % bn)
        break
print('RESULT', 'OK' if ok else 'BLOCKED')
