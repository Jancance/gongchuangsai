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
for k in ['g_dm_can2_feedback_valid','g_dm_can2_motor_state','g_dm_can2_state','g_dm_can2_result',
          'g_dm_can2_read_cycle','g_dm_can2_feedback_tick','g_vehicle_arm_state']:
    if k in names: print('%-30s %d' % (k, t.read32(names[k])))
esr2 = t.read32(0x40006808); print('CAN2 ESR=0x%08X (LEC=%d TEC=%d REC=%d)' % (esr2, esr2 & 7, (esr2 >> 24) & 0xFF, (esr2 >> 16) & 0xFF))
