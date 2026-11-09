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
t.write32(names['g_startup_axes_vm_request'], 0)
time.sleep(0.3)
t.reset(reset_type=Target.ResetType.SYSRESETREQ)
time.sleep(4.0)
s.open()
print('post-reset: poll_en=%d ready=%d fault=%d' % (g('g_can_poll_enable'), g('g_safety_ready'), g('g_safety_fault')))
t.write32(names['g_diag_scan_enable'], 1)
t.write32(names['g_startup_axes_vm_request'], 1)
for i in range(10):
    time.sleep(0.4)
    print('t=%3.1f res0=%d res3=%d res4=%d res5=%d fault=%d ready=%d axes_vm=%d diag=%d' % (
        0.4 * (i + 1), t.read32(names['g_diag_scan_result']), t.read32(names['g_diag_scan_result'] + 12),
        t.read32(names['g_diag_scan_result'] + 16), t.read32(names['g_diag_scan_result'] + 20),
        g('g_safety_fault'), g('g_safety_ready'), g('g_startup_axes_vm_request'), g('g_diag_scan_enable')))
    if g('g_safety_fault'):
        break
