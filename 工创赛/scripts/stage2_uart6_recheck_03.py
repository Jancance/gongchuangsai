"""USART6 recheck: absolute counters + PC6/PC7 edge counting. Read-only."""
from pathlib import Path
import time, json
import source_stage2_readonly_sync_probe_20261105 as transport
from pyocd.core.helpers import ConnectHelper
from pyocd.core.target import Target

R = Path(__file__).resolve().parents[1]
IMAGE = R / 'tmp/integration12_stage2_vision_20261107/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf'
PROBE = '6afaae24'
GPIOC_IDR = 0x40020810
USART6_SR = 0x40011400

transport.live.IMAGE = IMAGE
names, _ = transport.live.load_image()
opt = {'target_override': 'cortex_m', 'connect_mode': 'attach', 'frequency': 1000000,
       'auto_unlock': False, 'resume_on_disconnect': False, 'no_config': True,
       'cmsis_dap.prefer_v1': False, 'cmsis_dap.limit_packets': True}
s = ConnectHelper.session_with_chosen_probe(unique_id=PROBE, options=opt)
s.open(); t = s.target
assert t.get_state() == Target.State.RUNNING
g = lambda k: t.read32(names[k])

keys = ['g_vis_rx_bytes', 'g_uart6_rx_errors', 'g_vis_rx_bad', 'g_vis_rx_armed', 'g_vis_command_sent']
print('symbol addr:', {k: hex(names[k]) for k in keys})
a = {k: g(k) for k in keys}
sr0 = t.read32(USART6_SR) & 0xFFFF
print('t0 %s SR=0x%04X' % (a, sr0))

edges6 = edges7 = 0
n = 0
prev = None
t_end = time.time() + 5.0
while time.time() < t_end:
    idr = t.read32(GPIOC_IDR)
    cur = ((idr >> 6) & 1, (idr >> 7) & 1)
    if prev is not None:
        if cur[0] != prev[0]:
            edges6 += 1
        if cur[1] != prev[1]:
            edges7 += 1
    prev = cur
    n += 1
    time.sleep(0.001)

b = {k: g(k) for k in keys}
sr1 = t.read32(USART6_SR) & 0xFFFF
print('t1 %s SR=0x%04X' % (b, sr1))
print('samples=%d  PC6 edges=%d  PC7 edges=%d' % (n, edges6, edges7))
for k in keys:
    print('%s: %d -> %d (%+d)' % (k, a[k], b[k], b[k] - a[k]))
json.dump({'t0': a, 't1': b, 'sr0': sr0, 'sr1': sr1, 'samples': n,
           'pc6_edges': edges6, 'pc7_edges': edges7,
           'axf_sha': '98c8fbe38783e8d1a026c53f1cc799b61b1a82cbdec94403b0e10c5b5dc19376'},
          open(str(R / 'evidence/integration/2026-11-07/stage2-mainline/uart6-recheck-03-ch340-2.json'), 'w', encoding='utf-8'), indent=2)

