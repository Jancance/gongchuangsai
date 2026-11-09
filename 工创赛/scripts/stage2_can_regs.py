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
r = lambda a: t.read32(a)
print('CAN1 MCR =0x%08X  MSR=0x%08X  TSR=0x%08X' % (r(0x40006400), r(0x40006404), r(0x40006408)))
print('CAN1 RF0R=0x%08X RF1R=0x%08X IER=0x%08X' % (r(0x4000640C), r(0x40006410), r(0x40006414)))
print('CAN1 ESR=0x%08X (LEC=%d TEC=%d REC=%d BOFF=%d EPVF=%d EWGF=%d)' % (r(0x40006418), r(0x40006418)&7, (r(0x40006418)>>24)&0xFF, (r(0x40006418)>>16)&0xFF, (r(0x40006418)>>2)&1, (r(0x40006418)>>1)&1, r(0x40006418)&1))
btr = r(0x4000641C)
print('CAN1 BTR=0x%08X  SILM=%d LBKM=%d SJW=%d TS2=%d TS1=%d BRP=%d' % (btr, (btr>>31)&1, (btr>>30)&1, (btr>>24)&3, (btr>>20)&7, (btr>>16)&0xF, (btr&0x3FF)))
brp = (btr & 0x3FF) + 1
ts1 = ((btr >> 16) & 0xF) + 1
ts2 = ((btr >> 20) & 7) + 1
apb1 = 42000000
print('  -> prescaler=%d tq_total=%d  bitrate = %.0f bps' % (brp, 1 + ts1 + ts2, apb1 / (brp * (1 + ts1 + ts2))))
print('CAN2 BTR=0x%08X ESR=0x%08X TSR=0x%08X' % (r(0x4000681C), r(0x40006818), r(0x40006808)))
