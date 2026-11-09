from pathlib import Path
import time
import source_stage2_readonly_sync_probe_20261105 as transport
from pyocd.core.helpers import ConnectHelper
from pyocd.core.target import Target
R = Path(__file__).resolve().parents[1]
transport.live.IMAGE = R / 'tmp/integration12_stage2_vision_20261107/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf'
names, _ = transport.live.load_image()
opt = {'target_override': 'cortex_m', 'connect_mode': 'attach', 'frequency': 1000000,
       'auto_unlock': False, 'resume_on_disconnect': False, 'no_config': True,
       'cmsis_dap.prefer_v1': False, 'cmsis_dap.limit_packets': True}
s = ConnectHelper.session_with_chosen_probe(unique_id='6afaae24', options=opt)
s.open(); t = s.target
assert t.get_state() == Target.State.RUNNING
c0 = t.read32(names['g_vis_command_sent'])
t.write32(names['g_vis_command_request'], 1)
for i in range(3):
    time.sleep(0.3)
    t.write32(names['g_vis_command_request'], 1)
print('sent %d -> %d' % (c0, t.read32(names['g_vis_command_sent'])))
