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
e0 = t.read32(0x40006808)
t.write32(names['g_dm_can2_read_request'], 1)
for i in range(12):
    time.sleep(0.3)
    if g('g_dm_can2_feedback_valid') == 1:
        print('DM_ACK_OK pos_raw cycle=%d' % g('g_dm_can2_read_cycle'))
        break
else:
    print('DM_NO_ACK')
e1 = t.read32(0x40006808)
print('CAN2 ESR before=0x%08X after=0x%08X (LEC=%d TEC=%d)' % (e0, e1, e1 & 7, (e1 >> 24) & 0xFF))
print('can2 state=%d result=%d fb_valid=%d' % (g('g_dm_can2_state'), g('g_dm_can2_result'), g('g_dm_can2_feedback_valid')))
