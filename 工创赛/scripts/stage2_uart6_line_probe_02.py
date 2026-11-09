"""PC6/PC7 线电平 + USART6 字节速率探测（纯读，不发任何东西）。"""
from pathlib import Path
import time, json
import source_stage2_readonly_sync_probe_20261105 as transport
from pyocd.core.helpers import ConnectHelper
from pyocd.core.target import Target

R = Path(__file__).resolve().parents[1]
IMAGE = R / 'tmp/integration12_stage2_vision_20261107/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf'
PROBE = '6afaae24'
GPIOC_IDR = 0x40020810
GPIOC_MODER = 0x40020800
USART6_SR = 0x40011400
USART6_DR = 0x40011404
USART6_BRR = 0x40011408
USART6_CR1 = 0x4001140C

transport.live.IMAGE = IMAGE
names, _ = transport.live.load_image()
opt = {'target_override': 'cortex_m', 'connect_mode': 'attach', 'frequency': 1000000,
       'auto_unlock': False, 'resume_on_disconnect': False, 'no_config': True,
       'cmsis_dap.prefer_v1': False, 'cmsis_dap.limit_packets': True}
s = ConnectHelper.session_with_chosen_probe(unique_id=PROBE, options=opt)
s.open(); t = s.target
assert t.get_state() == Target.State.RUNNING
g = lambda k: t.read32(names[k])

print('MODER=0x%08X  (PC6=%d PC7=%d, 2=AF)' % (t.read32(GPIOC_MODER), (t.read32(GPIOC_MODER) >> 12) & 3, (t.read32(GPIOC_MODER) >> 14) & 3))
print('USART6 BRR=0x%04X SR=0x%04X CR1=0x%04X' % (t.read32(USART6_BRR) & 0xFFFF, t.read32(USART6_SR) & 0xFFFF, t.read32(USART6_CR1) & 0xFFFF))

print('--- PC6/PC7 电平采样 (20 次 / 1s) ---')
samples = []
for i in range(20):
    idr = t.read32(GPIOC_IDR)
    samples.append(((idr >> 6) & 1, (idr >> 7) & 1))
    time.sleep(0.05)
print('PC6(TX) 序列:', ''.join(str(a) for a, b in samples))
print('PC7(RX) 序列:', ''.join(str(b) for a, b in samples))
pc6_hi = sum(a for a, b in samples); pc7_hi = sum(b for a, b in samples)
print('PC6 高电平 %d/20, PC7 高电平 %d/20' % (pc6_hi, pc7_hi))

print('--- USART6 速率 10s ---')
b0, e0 = g('g_vis_rx_bytes'), g('g_uart6_rx_errors')
time.sleep(10.0)
b1, e1 = g('g_vis_rx_bytes'), g('g_uart6_rx_errors')
db, de = b1 - b0, e1 - e0
print('rx_bytes +%d (%.1f B/s) | errors +%d (%.1f /s)' % (db, db / 10.0, de, de / 10.0))
if db and abs(db - de) < max(3, db // 50):
    print('=> 收到字节数 ≈ 错误数：几乎每个字节都是帧错误 → 线上是"持续低电平/波特率不符"，不是正常串口数据')
elif pc7_hi == 0:
    print('=> PC7 一直低电平 → 对端没在驱动（CH340 没上电 / 线没接上 / 被拉低）')
elif pc7_hi == 20 and db == 0:
    print('=> PC7 空闲高、无字节 → 线接对了但对端没发数据')
json.dump({'pc6_hi': pc6_hi, 'pc7_hi': pc7_hi, 'delta_bytes': db, 'delta_err': de,
           'bytes_per_s': db / 10.0, 'err_per_s': de / 10.0,
           'pc6_seq': ''.join(str(a) for a, b in samples), 'pc7_seq': ''.join(str(b) for a, b in samples)},
          open(str(R / 'evidence/integration/2026-11-07/stage2-mainline/uart6-line-probe-02.json'), 'w', encoding='utf-8'),
          indent=2)

