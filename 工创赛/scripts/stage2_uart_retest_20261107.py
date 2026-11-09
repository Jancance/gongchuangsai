"""USART6 视觉链路实测：PC 侧发 CC 帧，板子侧读计数。
自动排除探针虚拟串口（SER=6AFAAE24 / VID FAED），只认真正的 CH340（1A86:7523）。
用法:
  python stage2_uart_retest_20261107.py              # 自动找 CH340
  python stage2_uart_retest_20261107.py --port COM7  # 指定端口
"""
from pathlib import Path
import sys, time, json, datetime
import serial
from serial.tools import list_ports

import source_stage2_readonly_sync_probe_20261105 as transport
from pyocd.core.helpers import ConnectHelper
from pyocd.core.target import Target

R = Path(__file__).resolve().parents[1]
IMAGE = R / 'tmp/integration12_stage2_vision_20261107/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf'
PROBE = '6afaae24'
PROBE_SER = '6AFAAE24'
OUT = R / 'evidence/integration/2026-11-07/stage2-mainline'

KEYS = ['g_vis_rx_armed', 'g_vis_rx_bytes', 'g_vis_rx_bad', 'g_vis_center_cycle',
        'g_vis_center_x', 'g_vis_center_y', 'g_vis_center_tick', 'g_vis_command_sent',
        'g_uart6_rx_errors', 'g_uart1_rx_errors', 'g_vis_mat_cycle',
        'g_serial_rx_count']


def enumerate_ports():
    out = []
    for p in list_ports.comports():
        out.append((p.device, p.description or '', (p.hwid or '').upper(), p.serial_number or ''))
    return out


def pick(force=None):
    ports = enumerate_ports()
    print('=== COM ports ===')
    for dev, desc, hwid, ser in ports:
        mark = ''
        if PROBE_SER in (ser or '').upper() or 'FAED:4873' in hwid:
            mark = '  <-- DAP probe VCP, NOT your CH340'
        elif '1A86:7523' in hwid:
            mark = '  <-- CH340'
        print('  %-6s %-38s %-58s%s' % (dev, desc, hwid[:58], mark))
    if force:
        return force.upper()
    for dev, desc, hwid, ser in ports:
        if '1A86:7523' in hwid:
            return dev
    return None


def main():
    force = None
    if '--port' in sys.argv:
        force = sys.argv[sys.argv.index('--port') + 1]
    port = pick(force)
    if not port:
        print()
        print('!! 没找到 CH340 (VID:PID=1A86:7523)。')
        print('   1) 把 CH340 的 USB 头从树莓派上拔下来，插到这台电脑；或')
        print('   2) 保持在树莓派上，用下面的脚本在树莓派那边发帧，我这边只读板子计数：')
        print("      python3 -c \"import serial,time;s=serial.Serial('/dev/ttyUSB0',115200,timeout=0.2);"
              "f=bytes([0xCC,0x05,0x40,0x01,0x58,0x02,0xDD]);[ (s.write(f),s.flush(),time.sleep(0.05)) "
              "for _ in range(20)];print('sent 20 frames')\"")
        print('   已把板子基线写进证据，等你接线/发帧。')
        port = None
    print('using port:', port)

    transport.live.IMAGE = IMAGE
    names, segs = transport.live.load_image()
    miss = [k for k in KEYS if k not in names]
    assert not miss, miss
    opt = {'target_override': 'cortex_m', 'connect_mode': 'attach', 'frequency': 1000000,
           'auto_unlock': False, 'resume_on_disconnect': False, 'no_config': True,
           'cmsis_dap.prefer_v1': False, 'cmsis_dap.limit_packets': True}
    s = ConnectHelper.session_with_chosen_probe(unique_id=PROBE, options=opt)
    s.open(); t = s.target
    assert t.get_state() == Target.State.RUNNING
    g = lambda k: t.read32(names[k])
    base = {k: g(k) for k in KEYS}
    print('baseline:', json.dumps(base))

    heard = b''
    after = dict(base)
    if port:
        ser = serial.Serial(port, 115200, timeout=0.1)
        ser.reset_input_buffer()
        print('--- listen 1.0s ---')
        t0 = time.time()
        while time.time() - t0 < 1.0:
            heard += ser.read(4096)
        print('heard %d bytes: %r' % (len(heard), heard[:80]))
        print('--- send 20 x CC 05 40 01 58 02 DD (x=320 y=600) ---')
        frame = bytes([0xCC, 0x05, 0x40, 0x01, 0x58, 0x02, 0xDD])
        for _ in range(20):
            ser.write(frame); ser.flush(); time.sleep(0.04)
        time.sleep(0.5)
        ser.close()
        after = {k: g(k) for k in KEYS}
        print('after   :', json.dumps(after))

    d = {k: after[k] - base[k] for k in KEYS}
    print('delta   :', json.dumps(d))
    print('rx_bytes +%d | rx_bad +%d | center_cycle +%d | uart6_err +%d | uart1_err +%d | serial_rx +%d'
          % (d['g_vis_rx_bytes'], d['g_vis_rx_bad'], d['g_vis_center_cycle'],
             d['g_uart6_rx_errors'], d['g_uart1_rx_errors'], d['g_serial_rx_count']))
    if after['g_vis_center_cycle'] > base['g_vis_center_cycle']:
        verdict = 'PASS_USART6_VISION_LINK'
    elif not port:
        verdict = 'NO_CH340_PENDING_OPERATOR'
    elif d['g_vis_rx_bytes'] > 0:
        verdict = 'BYTES_IN_BUT_NO_VALID_CC_FRAME'
    elif d['g_serial_rx_count'] > 0:
        verdict = 'FRAMES_ARRIVED_ON_USART1_RX_NOT_USART6'
    else:
        verdict = 'NO_BYTES_AT_BOARD'
    print('VERDICT:', verdict)

    OUT.mkdir(parents=True, exist_ok=True)
    ev = {'tag': 'uart-retest-02', 'port': port, 'base': base, 'after': after, 'delta': d,
          'heard_bytes': len(heard), 'heard_head': heard[:200].decode('ascii', 'replace'),
          'verdict': verdict, 'ts': datetime.datetime.now().isoformat(timespec='seconds')}
    p = OUT / ('uart-retest-%s.json' % datetime.datetime.now().strftime('%H%M%S'))
    p.write_text(json.dumps(ev, indent=2, ensure_ascii=False), encoding='utf-8')
    print('evidence:', p)


if __name__ == '__main__':
    main()
