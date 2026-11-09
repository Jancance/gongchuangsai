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
ks = [k for k in sorted(names) if 'motion_auto_stop' in k or 'cut_vm' in k or k in
      ('g_safety_ready','g_safety_fault','g_vm_test_request','g_can_guard_ready',
       'g_can_guard_cause','g_can_guard_bad_nodes','g_can_test_result','g_startup_axes_vm_request',
       'g_lift_ruler_vm_request','g_carrier_vm_request','g_extension_vm_request')]
for k in ks:
    print('%-34s %d' % (k, t.read32(names[k])))
print('PE2 =', (t.read32(0x40021014) >> 2) & 1)
