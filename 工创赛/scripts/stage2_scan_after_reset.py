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
t.write32(names['g_startup_axes_vm_request'], 0)
t.write32(names['g_vm_test_request'], 0)
time.sleep(0.3)
t.reset(reset_type=Target.ResetType.SYSRESETREQ)
time.sleep(2.0)
s.open()
print('reset ok fault=%d' % g('g_safety_fault'))
for addr in (4, 5, 6, 3):
    hit = None
    for i in range(20):
        t.write32(names['g_vm_test_request'], 1)
        t.write32(names['g_can_version_address'], addr)
        t.write32(names['g_can_status_request'], 1)
        time.sleep(0.02)
        r = g('g_can_status_result')
        if r:
            hit = (i, r)
            if r == 1:
                b = names['g_can_status_reply']
                ext = t.read32(b + 4); dlc = t.read32(b + 8) & 0xF
                data = [t.read32(b + 16) & 0xFF, (t.read32(b + 16) >> 8) & 0xFF, (t.read32(b + 16) >> 16) & 0xFF]
                hit = (i, r, hex(ext), dlc, data)
                break
    print('addr=%d -> %s' % (addr, hit))
    if g('g_safety_fault'):
        print('  fault latched at addr', addr)
        break
