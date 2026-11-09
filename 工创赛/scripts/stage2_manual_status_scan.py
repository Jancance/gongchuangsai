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

def manual_query(addr, tries=12):
    got = []
    for i in range(tries):
        t.write32(names['g_vm_test_request'], 1)
        t.write32(names['g_can_version_address'], addr)
        t.write32(names['g_can_status_request'], 1)
        time.sleep(0.05)
        r = g('g_can_status_result')
        esr = g('g_can_status_esr')
        got.append((r, esr))
        if r == 1:
            b = names['g_can_status_reply']
            ext = t.read32(b + 4)
            dlc = t.read32(b + 8) & 0xF
            data = [t.read32(b + 16 + 4 * j) & 0xFF for j in range(3)]
            return True, i, r, hex(ext), dlc, data
        if g('g_safety_fault'):
            # 清故障需要复位，这里直接返回
            return False, i, r, 'FAULT', 0, []
    return False, tries, got[-1][0] if got else None, '', 0, []

for addr in (3, 4, 5, 6):
    ok, i, r, ext, dlc, data = manual_query(addr)
    print('addr=%d -> %s  tries=%d result=%s ext=%s dlc=%s data=%s' % (
        addr, 'OK' if ok else 'SILENT', i, r, ext, dlc, data))
    if g('g_safety_fault'):
        print('  (fault latched, stopping scan)')
        break
