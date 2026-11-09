"""只开电机电源(VM)不开任何动作，检查 4310(CAN2) 是否应答；查完断电。"""
from pathlib import Path
import time, json, struct

import source_stage2_readonly_sync_probe_20261105 as transport
from pyocd.core.helpers import ConnectHelper

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
                    'BOFF': (v >> 2) & 1, 'MSR': hex(msr), 'SLEEP_bit1': (msr >> 1) & 1,
                    'INAK': (msr >> 2) & 1, 'SLAK': (msr >> 3) & 1}

        out['before'] = {'vm': w('g_vm_test_request'), 'can_guard': w('g_can_guard_ready'),
                         'bus': bus(), 'claw': t.read32(0x40000034)}
        t.write32(n['g_lift_ruler_controller'] + 4, 1)     # 仅请求电机电源
        out['vm_requested'] = True
        begin = time.monotonic()
        while time.monotonic() - begin < 8:
            if w('g_vm_test_request') == 1 and (t.read32(0x40021014) & 4) and w('g_can_guard_ready') == 1:
                break
            time.sleep(.1)
        out['vm_up_ms'] = round((time.monotonic() - begin) * 1000)
        out['vm_state'] = {'vm': w('g_vm_test_request'), 'pe2': bool(t.read32(0x40021014) & 4),
                           'can_guard': w('g_can_guard_ready'), 'init_result': w('g_dm_can2_init_result')}
        cycle0 = w('g_dm_can2_read_cycle')
        t.write32(n['g_dm_can2_read_request'], 2)
        begin = time.monotonic()
        while time.monotonic() - begin < 5:
            if w('g_dm_can2_read_cycle') != cycle0:
                break
            time.sleep(.02)
        out['read_ms'] = round((time.monotonic() - begin) * 1000)
        time.sleep(.3)
        out['dm'] = {'state': w('g_dm_can2_state'), 'result': w('g_dm_can2_result'),
                     'read_cycle': w('g_dm_can2_read_cycle'), 'feedback_valid': w('g_dm_can2_feedback_valid'),
                     'valid_mask': w('g_dm_can2_valid_mask'), 'range_mask': w('g_dm_can2_range_mask'),
                     'ctrl_mode': w('g_dm_can2_ctrl_mode'), 'motor_state': w('g_dm_can2_motor_state'),
                     'esc_id': w('g_dm_can2_esc_id'), 'master_id': w('g_dm_can2_master_id'),
                     'rx': w('g_dm_can2_rx_count'), 'tx': w('g_dm_can2_tx_count'),
                     'pmax': f('g_dm_can2_pmax'), 'vmax': f('g_dm_can2_vmax'), 'tmax': f('g_dm_can2_tmax'),
                     'position_deg': f('g_dm_can2_position_deg'),
                     'logical_deg': f('g_dm_can2_logical_position_deg'),
                     'velocity': f('g_dm_can2_velocity_rad_s')}
        out['bus_up'] = bus()
        d = out['dm']
        out['comm_ok'] = bool(d['state'] == 4 and d['result'] == 1 and d['feedback_valid'] == 1
                             and (d['pmax'] or 0) > 0)
        # 检查完断电
        t.write32(n['g_lift_ruler_controller'] + 4, 0)
        t.write32(n['g_startup_axes_vm_request'], 0)
        for _ in range(30):
            time.sleep(.1)
            if w('g_vm_test_request') == 0 and not (t.read32(0x40021014) & 4):
                break
        out['vm_off_after'] = (w('g_vm_test_request') == 0 and not (t.read32(0x40021014) & 4))
        out['move_commands_sent'] = 0
    print(json.dumps(out, indent=1, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
