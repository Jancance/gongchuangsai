import time
from pathlib import Path
import source_stage2_readonly_sync_probe_20261105 as transport
from pyocd.core.helpers import ConnectHelper
from pyocd.core.target import Target
R = Path('.').resolve()
transport.live.IMAGE = R / 'tmp/integration12_stage2_vision_20261107/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf'
names, _ = transport.live.load_image()
print('diag symbols present:', 'g_diag_scan_result' in names)
opt = {'target_override': 'cortex_m', 'connect_mode': 'attach', 'frequency': 1000000,
       'auto_unlock': False, 'resume_on_disconnect': False, 'no_config': True,
       'cmsis_dap.prefer_v1': False, 'cmsis_dap.limit_packets': True}
s = ConnectHelper.session_with_chosen_probe(unique_id='6afaae24', options=opt)
s.open(); t = s.target
g = lambda k: t.read32(names[k])
t.write32(names['g_diag_scan_enable'], 1)
t.write32(names['g_startup_axes_vm_request'], 1)
time.sleep(1.6)
print('fault=%d ready=%d guard=%d' % (g('g_safety_fault'), g('g_safety_ready'), g('g_can_guard_ready')))
base = {k: names[k] for k in ('g_diag_scan_result','g_diag_scan_ext','g_diag_scan_ide','g_diag_scan_dlc','g_diag_scan_data1')}
for d in range(8):
    r = t.read32(base['g_diag_scan_result'] + 4 * d)
    ext = t.read32(base['g_diag_scan_ext'] + 4 * d)
    ide = t.read32(base['g_diag_scan_ide'] + 4 * d)
    dlc = t.read32(base['g_diag_scan_dlc'] + 4 * d)
    d1 = t.read32(base['g_diag_scan_data1'] + 4 * d)
    print('addr=%d result=%d extid=0x%04X ide=%d dlc=%d data1=0x%02X' % (d + 1, r, ext, ide, dlc, d1))
t.write32(names['g_diag_scan_enable'], 0)
