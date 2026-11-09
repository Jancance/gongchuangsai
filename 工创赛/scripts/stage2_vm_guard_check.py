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
pe2 = (t.read32(0x40021014) >> 2) & 1
print('PE2(VM)=%d vm_test=%d guard=%d bad=%d fault=%d ready=%d' % (
    pe2, g('g_vm_test_request'), g('g_can_guard_ready'), g('g_can_guard_bad_nodes'), g('g_safety_fault'), g('g_safety_ready')))
t.write32(names['g_vm_test_request'], 1)
for i in range(20):
    time.sleep(0.7)
    pe2 = (t.read32(0x40021014) >> 2) & 1
    print('t=%4.1f PE2=%d guard=%d bad=%d cause=%d fault=%d ready=%d cantest=%d' % (
        0.7 * (i + 1), pe2, g('g_can_guard_ready'), g('g_can_guard_bad_nodes'),
        g('g_can_guard_cause'), g('g_safety_fault'), g('g_safety_ready'), g('g_can_test_result')))
    if g('g_safety_fault'):
        print('FAULT_LATCHED')
        break
