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
ks = [k for k in sorted(names) if ('can_poll' in k or 'can_status' in k or 'can_feedback' in k) and k.startswith('g_')]
for k in ks:
    v = t.read32(names[k])
    if v or 'mask' in k or 'enable' in k or 'result' in k:
        print('%-32s %d' % (k, v))
esr = t.read32(0x40006408); msr = t.read32(0x40006400); tsr = t.read32(0x4000640C)
print('CAN1 MSR=0x%08X ESR=0x%08X TSR=0x%08X (TEC=%d REC=%d)' % (msr, esr, tsr, (esr >> 24) & 0xFF, (esr >> 16) & 0xFF))
