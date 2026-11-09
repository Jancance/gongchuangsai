from pathlib import Path
import json
import source_stage2_readonly_sync_probe_20261105 as transport
from pyocd.core.helpers import ConnectHelper
R = Path('.').resolve()
transport.live.IMAGE = R / 'tmp/integration12_stage2_vision_20261107/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf'
names, _ = transport.live.load_image()
opt = {'target_override': 'cortex_m', 'connect_mode': 'attach', 'frequency': 1000000,
       'auto_unlock': False, 'resume_on_disconnect': False, 'no_config': True,
       'cmsis_dap.prefer_v1': False, 'cmsis_dap.limit_packets': True}
s = ConnectHelper.session_with_chosen_probe(unique_id='6afaae24', options=opt)
s.open(); t = s.target
for k in ('g_it_request','g_it_state','g_it_rejected','g_it_action','g_it_cause',
          'g_lift_ruler_request','g_lift_ruler_state','g_lift_ruler_rejected','g_lift_ruler_cause',
          'g_lift_ruler_vm_request','g_vm_test_request','g_can_task_count','g_src_now_ms',
          'g_vis_rx_bytes','g_vis_command_sent'):
    if k in names:
        print('%-26s %d' % (k, t.read32(names[k])))
    else:
        print('%-26s MISSING' % k)
