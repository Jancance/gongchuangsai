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
t.write32(names['g_vm_test_request'], 1)
for i in range(15):
    time.sleep(0.2)
    vm = t.read32(names['g_vm_test_request'])
    pe2 = (t.read32(0x40021014) >> 2) & 1
    sr = t.read32(names['g_safety_ready'])
    sf = t.read32(names['g_safety_fault'])
    cut = t.read32(names['g_motion_auto_stop_cut_vm'])
    print('t=%4.1f vm_req=%d PE2=%d ready=%d fault=%d cut=%d' % (0.2 * (i + 1), vm, pe2, sr, sf, cut))
    if pe2 == 1:
        print('VM_ON')
        break
