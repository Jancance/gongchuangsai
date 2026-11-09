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
base = {k: names[k] for k in ('g_diag_scan_result','g_diag_scan_ext','g_diag_scan_ide','g_diag_scan_dlc','g_diag_scan_data1')}
print('%-5s %-7s %-8s %-4s %-4s %-6s' % ('addr', 'result', 'extid', 'ide', 'dlc', 'data1'))
for d in range(8):
    print('%-5d %-7d 0x%04X   %-4d %-4d 0x%02X' % (
        d + 1, t.read32(base['g_diag_scan_result'] + 4 * d), t.read32(base['g_diag_scan_ext'] + 4 * d),
        t.read32(base['g_diag_scan_ide'] + 4 * d), t.read32(base['g_diag_scan_dlc'] + 4 * d),
        t.read32(base['g_diag_scan_data1'] + 4 * d)))
