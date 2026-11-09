import struct
from pathlib import Path
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
ints = ['g_safety_ready','g_can_test_result','g_safety_fault','g_rtos_fault','g_can_ctrl_fault_latched',
        'g_chassis_auto_active','g_it_state','g_it_cause','g_it_rejected','g_it_action','g_it_elapsed_ms',
        'g_lift_ruler_state','g_lift_ruler_cause']
for k in ints:
    if k in names:
        print('%-26s %d' % (k, t.read32(names[k])))
for k in ['g_it_carrier_deg']:
    if k in names:
        print('%-26s %.3f' % (k, struct.unpack('<f', struct.pack('<I', t.read32(names[k])))[0]))
