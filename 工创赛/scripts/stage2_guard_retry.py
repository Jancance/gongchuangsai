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
# 清锁存：复位后马上拉 axes_vm，再在窗口内反复重启窗口
t.write32(names['g_startup_axes_vm_request'], 0)
time.sleep(0.5)
t.reset(reset_type=Target.ResetType.SYSRESETREQ)
time.sleep(2.2)
s.open()
print('after reset fault=%d ready=%d' % (g('g_safety_fault'), g('g_safety_ready')))
for attempt in range(8):
    t.write32(names['g_startup_axes_vm_request'], 1)
    for i in range(6):
        time.sleep(0.35)
        gr = g('g_can_guard_ready'); bn = g('g_can_guard_bad_nodes'); sf = g('g_safety_fault')
        print('attempt=%d t=%4.1f guard=%d bad=%d fault=%d PE2=%d' % (
            attempt, 0.35 * (i + 1), gr, bn, sf, (t.read32(0x40021014) >> 2) & 1))
        if gr == 1:
            print('GUARD_READY')
            raise SystemExit(0)
        if sf:
            break
    t.write32(names['g_startup_axes_vm_request'], 0)
    time.sleep(0.6)
    print('  -> reset window, restarting')
print('GUARD_NOT_READY')
