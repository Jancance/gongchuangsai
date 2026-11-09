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
print('before: poll_en=%d vm=%d axes_vm=%d ready=%d diag=%d' % (
    g('g_can_poll_enable'), g('g_vm_test_request'), g('g_startup_axes_vm_request'),
    g('g_safety_ready'), g('g_diag_scan_enable')))
t.write32(names['g_diag_scan_enable'], 1)
t.write32(names['g_startup_axes_vm_request'], 1)
for i in range(6):
    time.sleep(0.5)
    print('t=%3.1f poll_en=%d vm=%d axes_vm=%d ready=%d fault=%d diag=%d res0=%d res3=%d' % (
        0.5 * (i + 1), g('g_can_poll_enable'), g('g_vm_test_request'), g('g_startup_axes_vm_request'),
        g('g_safety_ready'), g('g_safety_fault'), g('g_diag_scan_enable'),
        t.read32(names['g_diag_scan_result']), t.read32(names['g_diag_scan_result'] + 12)))
t.write32(names['g_diag_scan_enable'], 0)
