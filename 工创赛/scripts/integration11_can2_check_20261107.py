"""只读检查 4310(CAN2) 通信：不写动作寄存、不发点位、不复位。只发一次只读状态读取请求。"""
from pathlib import Path
import time, json, struct

import source_stage2_readonly_sync_probe_20261105 as transport
from pyocd.core.helpers import ConnectHelper

R = Path(__file__).resolve().parents[1]
IMAGE = R / 'tmp/integration11_short_20261107/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf'
CAN2 = 0x40006800

WANT_W = ['g_dm_can2_state', 'g_dm_can2_result', 'g_dm_can2_read_cycle', 'g_dm_can2_read_request',
          'g_dm_can2_feedback_valid', 'g_dm_can2_feedback_tick', 'g_dm_can2_valid_mask',
          'g_dm_can2_range_mask', 'g_dm_can2_ctrl_mode', 'g_dm_can2_motor_state',
          'g_dm_can2_esc_id', 'g_dm_can2_master_id', 'g_dm_can2_rx_count', 'g_dm_can2_tx_count']
WANT_F = ['g_dm_can2_pmax', 'g_dm_can2_vmax', 'g_dm_can2_tmax', 'g_dm_can2_position_deg',
          'g_dm_can2_logical_position_deg', 'g_dm_can2_velocity_rad_s']


def main():
    transport.live.IMAGE = IMAGE
    n, _ = transport.live.load_image()
    opt = {'target_override': 'cortex_m', 'connect_mode': 'attach', 'frequency': 1000000,
           'auto_unlock': False, 'resume_on_disconnect': False, 'no_config': True,
           'cmsis_dap.prefer_v1': False, 'cmsis_dap.limit_packets': True}
    s = ConnectHelper.session_with_chosen_probe(unique_id='6afaae24', options=opt)
    out = {}
    with s:
        t = s.target

        def w(k):
            return t.read32(n[k]) if k in n else None

        def f(k):
            return round(struct.unpack('<f', struct.pack('<I', t.read32(n[k])))[0], 3) if k in n else None

        def esr():
            v = t.read32(CAN2 + 0x18)
            msr = t.read32(CAN2 + 0x04)
            return {'esr': hex(v), 'TEC': (v >> 16) & 0xFF, 'REC': (v >> 24) & 0xFF,
                    'BOFF': (v >> 2) & 1, 'EPV': (v >> 3) & 1, 'EWGF': (v >> 1) & 1,
                    'msr': hex(msr), 'INAK': (msr >> 2) & 1, 'SLAK': (msr >> 3) & 1,
                    'ERRI': (msr >> 4) & 1}

        out['before'] = {k: w(k) for k in WANT_W}
        out['before'].update({k: f(k) for k in WANT_F})
        out['before']['bus'] = esr()
        out['before']['now_tick'] = t.read32(n['g_dm_can2_feedback_tick']) if 'g_dm_can2_feedback_tick' in n else None

        cycle0 = w('g_dm_can2_read_cycle')
        t.write32(n['g_dm_can2_read_request'], 2)      # 只读状态读取请求（无动作）
        out['request_sent'] = True
        begin = time.monotonic()
        moved = False
        while time.monotonic() - begin < 3.0:
            if w('g_dm_can2_read_cycle') != cycle0:
                moved = True
                break
            time.sleep(.02)
        out['read_cycle_advanced'] = moved
        out['read_wait_ms'] = round((time.monotonic() - begin) * 1000)
        time.sleep(.2)
        out['after'] = {k: w(k) for k in WANT_W}
        out['after'].update({k: f(k) for k in WANT_F})
        out['after']['bus'] = esr()
        ok = (out['after'].get('g_dm_can2_state') == 4 and out['after'].get('g_dm_can2_result') == 1
              and out['after'].get('g_dm_can2_feedback_valid') == 1
              and (out['after'].get('g_dm_can2_pmax') or 0) > 0)
        out['comm_ok'] = bool(ok)
    print(json.dumps(out, indent=1, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
