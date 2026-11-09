"""USART6 link test 2: correct RX frames (0xCC) + verify board TX actually shifts out."""
from pathlib import Path
import time, json, serial
import source_stage2_readonly_sync_probe_20261105 as transport
from pyocd.core.helpers import ConnectHelper
from pyocd.core.target import Target

R = Path(__file__).resolve().parents[1]
IMAGE = R / 'tmp/integration12_stage2_vision_20261107/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf'
PROBE = '6afaae24'
PORT = 'COM19'
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

port = serial.Serial(PORT, 115200, bytesize=8, parity='N', stopbits=1, timeout=0.3)
time.sleep(0.3); port.reset_input_buffer()

keys = ['g_vis_rx_bytes', 'g_uart6_rx_errors', 'g_vis_rx_bad', 'g_vis_material_valid',
        'g_vis_mat_cycle', 'g_vis_mat_left', 'g_vis_mat_right', 'g_vis_mat_down',
        'g_vis_center_valid', 'g_vis_center_cycle', 'g_vis_center_x', 'g_vis_center_y',
        'g_vis_command_sent', 'g_vis_command_rejected']
base = {k: g(k) for k in keys}
print('BASE rx=%d err=%d bad=%d sent=%d' % (base['g_vis_rx_bytes'], base['g_uart6_rx_errors'], base['g_vis_rx_bad'], base['g_vis_command_sent']))

# --- A) trigger TX and watch PC6 for edges ---
c0 = g('g_vis_command_sent')
t.write32(names['g_vis_command_request'], 1)
edges = 0
prev = None
for i in range(1200):
    idr = t.read32(GPIOC_IDR)
    cur = (idr >> 6) & 1
    if prev is not None and cur != prev:
        edges += 1
    prev = cur
sr = t.read32(USART6_SR) & 0xFFFF
c1 = g('g_vis_command_sent')
print('TX trigger: sent %d -> %d, PC6 edges during burst = %d, SR=0x%04X (TC=%d TXE=%d)' % (c0, c1, edges, sr, (sr >> 6) & 1, (sr >> 7) & 1))
heard = port.read(32)
print('PC heard:', heard.hex() if heard else '(none)')

# --- B) canonical CC frames ---
mat = bytes([0xCC, 0x02, 0x01, 0x02, 0x03, 0xDD])
port.write(mat); port.flush(); time.sleep(0.5)
mid = {k: g(k) for k in keys}
ctr = bytes([0xCC, 0x05, 0x34, 0x12, 0x78, 0x56, 0xDD])
port.write(ctr); port.flush(); time.sleep(0.5)
end = {k: g(k) for k in keys}
print('sent CC material=%s center=%s' % (mat.hex(), ctr.hex()))
print('--- after CC frames ---')
for k in ['g_vis_rx_bytes', 'g_uart6_rx_errors', 'g_vis_rx_bad', 'g_vis_material_valid',
          'g_vis_mat_cycle', 'g_vis_mat_left', 'g_vis_mat_right', 'g_vis_mat_down',
          'g_vis_center_valid', 'g_vis_center_cycle', 'g_vis_center_x', 'g_vis_center_y']:
    print('%-22s %d -> %d' % (k, base[k], end[k]))

# --- C) second TX trigger (buffer must have drained) ---
c2 = g('g_vis_command_sent')
t.write32(names['g_vis_command_request'], 2)
time.sleep(0.4)
c3 = g('g_vis_command_sent')
heard2 = port.read(32)
print('2nd TX trigger: sent %d -> %d, PC heard: %s' % (c2, c3, heard2.hex() if heard2 else '(none)'))
port.close()
json.dump({'port': PORT, 'base': base, 'after_cc': end,
           'tx_edges_pc6': edges, 'usart6_sr_after_tx': sr,
           'tx_sent_1': [c0, c1], 'tx_sent_2': [c2, c3],
           'pc_heard_1': heard.hex(), 'pc_heard_2': heard2.hex(),
           'cc_material': mat.hex(), 'cc_center': ctr.hex()},
          open(str(R / 'evidence/integration/2026-11-07/stage2-mainline/uart6-link-test-02.json'), 'w', encoding='utf-8'), indent=2)
