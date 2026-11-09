"""复位主控以重新初始化 CAN2，然后只读检查 4310 是否应答。不动机械臂、不发点位。"""
from pathlib import Path
import time, json, struct

import source_stage2_readonly_sync_probe_20261105 as transport
from pyocd.core.helpers import ConnectHelper
from pyocd.core.target import Target

R = Path(__file__).resolve().parents[1]
IMAGE = R / 'tmp/integration11_short_20261107/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf'
CAN2 = 0x40006800


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

        def bus():
            v = t.read32(CAN2 + 0x18)
            msr = t.read32(CAN2 + 0x04)
            return {'ESR': hex(v), 'TEC': (v >> 16) & 0xFF, 'REC': (v >> 24) & 0xFF,
                    'BOFF': (v >> 2) & 1, 'MSR': hex(msr), 'SLEEP': msr & 1, 'INAK': (msr >> 2) & 1,
                    'SLAK': (msr >> 3) & 1, 'TXM': (msr >> 7) & 1, 'RXM': (msr >> 8) & 1}

        out['before_bus'] = bus()
        out['claw_before'] = t.read32(0x40000034)
        t.reset(reset_type=Target.ResetType.SYSRESETREQ)
        out['reset'] = True
        time.sleep(4)
        out['after_bus'] = bus()
        out['claw_after'] = t.read32(0x40000034)
        out['after_reset_regs'] = {k: w(k) for k in ['g_dm_can2_state', 'g_dm_can2_result',
                                                     'g_dm_can2_read_cycle', 'g_dm_can2_feedback_valid']}
        cycle0 = w('g_dm_can2_read_cycle')
        t.write32(n['g_dm_can2_read_request'], 2)
        begin = time.monotonic()
        while time.monotonic() - begin < 4:
            if w('g_dm_can2_read_cycle') != cycle0:
                break
            time.sleep(.02)
        out['read_ms'] = round((time.monotonic() - begin) * 1000)
        time.sleep(.3)
        out['final'] = {
            'state': w('g_dm_can2_state'), 'result': w('g_dm_can2_result'),
            'read_cycle': w('g_dm_can2_read_cycle'), 'feedback_valid': w('g_dm_can2_feedback_valid'),
            'valid_mask': w('g_dm_can2_valid_mask'), 'range_mask': w('g_dm_can2_range_mask'),
            'ctrl_mode': w('g_dm_can2_ctrl_mode'), 'motor_state': w('g_dm_can2_motor_state'),
            'esc_id': w('g_dm_can2_esc_id'), 'master_id': w('g_dm_can2_master_id'),
            'rx_count': w('g_dm_can2_rx_count'), 'tx_count': w('g_dm_can2_tx_count'),
            'pmax': f('g_dm_can2_pmax'), 'vmax': f('g_dm_can2_vmax'), 'tmax': f('g_dm_can2_tmax'),
            'position_deg': f('g_dm_can2_position_deg'),
            'logical_deg': f('g_dm_can2_logical_position_deg'),
            'velocity': f('g_dm_can2_velocity_rad_s')}
        out['bus_final'] = bus()
        out['comm_ok'] = bool(out['final']['state'] == 4 and out['final']['result'] == 1
                             and out['final']['feedback_valid'] == 1
                             and (out['final']['pmax'] or 0) > 0)
    print(json.dumps(out, indent=1, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
