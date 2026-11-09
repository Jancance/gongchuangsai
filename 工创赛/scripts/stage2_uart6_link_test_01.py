"""USART6 end-to-end link test: PC(CH340 COM19) <-> STM32 PC6/PC7.

Sends protocol frames from the PC and asks the board to send one 3-byte command.
Only writes g_vis_command_request (UART command trigger); no motion, no reset.
"""
from pathlib import Path
import time, json, serial
import source_stage2_readonly_sync_probe_20261105 as transport
from pyocd.core.helpers import ConnectHelper
from pyocd.core.target import Target

R = Path(__file__).resolve().parents[1]
IMAGE = R / 'tmp/integration12_stage2_vision_20261107/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf'
PROBE = '6afaae24'
PORT = 'COM19'

transport.live.IMAGE = IMAGE
names, _ = transport.live.load_image()
opt = {'target_override': 'cortex_m', 'connect_mode': 'attach', 'frequency': 1000000,
       'auto_unlock': False, 'resume_on_disconnect': False, 'no_config': True,
       'cmsis_dap.prefer_v1': False, 'cmsis_dap.limit_packets': True}
s = ConnectHelper.session_with_chosen_probe(unique_id=PROBE, options=opt)
s.open(); t = s.target
assert t.get_state() == Target.State.RUNNING
g = lambda k: t.read32(names[k])

port = serial.Serial(PORT, 115200, bytesize=8, parity='N', stopbits=1, timeout=0.2)
time.sleep(0.3)
port.reset_input_buffer()

keys = ['g_vis_rx_bytes', 'g_uart6_rx_errors', 'g_vis_rx_bad', 'g_vis_material_valid',
        'g_vis_mat_cycle', 'g_vis_mat_left', 'g_vis_mat_right', 'g_vis_mat_down',
        'g_vis_center_valid', 'g_vis_center_cycle', 'g_vis_center_x', 'g_vis_center_y',
        'g_vis_command_sent', 'g_vis_command_rejected']
base = {k: g(k) for k in keys}
print('BASE', base)

idle = port.read(32)
print('idle bytes from board:', idle.hex() if idle else '(none)')

# 1) board -> PC : trigger one command frame
t.write32(names['g_vis_command_request'], 1)
time.sleep(0.6)
heard = port.read(32)
print('board->PC heard:', heard.hex() if heard else '(none)')

# 2) PC -> board : material frame AA 02 L R D DD
mat = bytes([0xAA, 0x02, 0x01, 0x02, 0x03, 0xDD])
port.write(mat); port.flush()
time.sleep(0.5)
mid = {k: g(k) for k in keys}
print('AFTER MAT', mid)

# 3) PC -> board : center frame AA 05 xL xH yL yH DD  (x=0x1234, y=0x5678)
ctr = bytes([0xAA, 0x05, 0x34, 0x12, 0x78, 0x56, 0xDD])
port.write(ctr); port.flush()
time.sleep(0.5)
end = {k: g(k) for k in keys}
print('AFTER CTR', end)

print('--- delta base->end ---')
for k in keys:
    print('%-24s %d -> %d (%+d)' % (k, base[k], end[k], end[k] - base[k]))
print('sent mat=%s ctr=%s' % (mat.hex(), ctr.hex()))
print('board->PC raw = %s' % heard.hex())
port.close()
json.dump({'port': PORT, 'base': base, 'after_material': mid, 'after_center': end,
           'idle_from_board_hex': idle.hex(), 'board_to_pc_hex': heard.hex(),
           'material_frame': mat.hex(), 'center_frame': ctr.hex(),
           'axf_sha': '98c8fbe38783e8d1a026c53f1cc799b61b1a82cbdec94403b0e10c5b5dc19376'},
          open(str(R / 'evidence/integration/2026-11-07/stage2-mainline/uart6-link-test-01.json'), 'w', encoding='utf-8'), indent=2)
