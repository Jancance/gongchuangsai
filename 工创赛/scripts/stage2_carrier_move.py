import struct, sys, time
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
deg = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0
u = struct.unpack('<I', struct.pack('<f', deg))[0]
t.write32(names['g_it_carrier_deg'], u)
t.write32(names['g_it_request'], 10)
print('request 10 carrier=%.1fdeg issued' % deg)
for i in range(10):
    time.sleep(0.4)
    st = t.read32(names['g_it_state']); ca = t.read32(names['g_it_cause']); rj = t.read32(names['g_it_rejected'])
    print('t=%4.1fs state=%d cause=%d rejected=%d' % (0.4 * (i + 1), st, ca, rj))
    if st == 2:
        print('DONE')
        break
    if st == 9:
        print('FAILED')
        break
