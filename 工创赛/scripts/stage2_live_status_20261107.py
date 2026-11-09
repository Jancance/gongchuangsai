"""现场只读状态观察：视觉串口收帧计数、观测有效性、目标事务、第二段状态、故障位。
用法：
  python stage2_live_status_20261107.py            # 读一次
  python stage2_live_status_20261107.py 20         # 观察 20 秒，每 0.5s 打印视觉计数变化
只读，不写任何变量，不发电机命令。
"""
from pathlib import Path
import sys, time, struct, hashlib

import source_stage2_readonly_sync_probe_20261105 as transport
from pyocd.core.helpers import ConnectHelper
from pyocd.core.target import Target

R = Path(__file__).resolve().parents[1]
IMAGE = R / 'tmp/integration12_stage2_vision_20261107/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf'
PROBE = '6afaae24'

W32 = ['g_vis_rx_armed', 'g_vis_rx_bytes', 'g_vis_rx_bad',
       'g_vis_mat_cycle', 'g_vis_mat_left', 'g_vis_mat_right', 'g_vis_mat_down', 'g_vis_mat_tick',
       'g_vis_center_cycle', 'g_vis_center_x', 'g_vis_center_y', 'g_vis_center_tick',
       'g_vis_material_valid', 'g_vis_metric_valid', 'g_vis_center_valid',
       'g_vis_loaded_mask', 'g_vis_pick_region', 'g_vis_taught_mask', 'g_vis_txn_result',
       'g_vis_task_valid', 'g_vis_task_batch', 'g_vis_task_item', 'g_vis_task_run_id',
       'g_vis_batch_complete', 'g_vis_pending_seq', 'g_vis_pending_slot', 'g_vis_pending_grasped',
       'g_vis_command_request', 'g_vis_command_sent', 'g_vis_command_rejected',
       'g_src_txn_revision', 'g_src_txn_consumed', 'g_src_txn_result', 'g_src_slot', 'g_src_now_ms',
       'g_stage2_ready', 'g_stage2_body_state', 'g_stage2_body_session', 'g_stage2_body_reason',
       'g_it_state', 'g_it_cause', 'g_it_rejected', 'g_it_action',
       'g_lift_ruler_state', 'g_lift_ruler_cause', 'g_startup_axes_state', 'g_startup_axes_cause',
       'g_dm_jog_state', 'g_dm_jog_result', 'g_dm_posvel_holding', 'g_safety_fault', 'g_rtos_fault',
       'g_vm_test_request', 'g_can_guard_ready']
F32 = ['g_vis_mm_per_px', 'g_vis_ref_x', 'g_vis_ref_y',
       'g_src_pick_deg', 'g_src_extension_mm', 'g_src_chassis_dx_mm', 'g_src_chassis_dy_mm',
       'g_dm_can2_logical_position_deg', 'g_vehicle_lift_current_deg']


def main():
    watch = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
    transport.live.IMAGE = IMAGE
    n, segs = transport.live.load_image()
    miss = [k for k in W32 + F32 if k not in n]
    assert not miss, miss
    opt = {'target_override': 'cortex_m', 'connect_mode': 'attach', 'frequency': 1000000,
           'auto_unlock': False, 'resume_on_disconnect': False, 'no_config': True,
           'cmsis_dap.prefer_v1': False, 'cmsis_dap.limit_packets': True}
    s = ConnectHelper.session_with_chosen_probe(unique_id=PROBE, options=opt)
    s.open()
    t = s.target
    assert t.get_state() == Target.State.RUNNING

    def g(k):
        return t.read32(n[k])

    def gf(k):
        return struct.unpack('<f', struct.pack('<I', t.read32(n[k])))[0]

    def dump(label):
        print('--- %s  (vis: armed=%d bytes=%d bad=%d | mat_cycle=%d L/R/D=%d/%d/%d | center_cycle=%d (%d,%d))'
              % (label, g('g_vis_rx_armed'), g('g_vis_rx_bytes'), g('g_vis_rx_bad'),
                 g('g_vis_mat_cycle'), g('g_vis_mat_left'), g('g_vis_mat_right'), g('g_vis_mat_down'),
                 g('g_vis_center_cycle'), g('g_vis_center_x'), g('g_vis_center_y')))
        print('    valid: mat=%d metric=%d center=%d | task: valid=%d batch=%d item=%d run=%d complete=%d'
              % (g('g_vis_material_valid'), g('g_vis_metric_valid'), g('g_vis_center_valid'),
                 g('g_vis_task_valid'), g('g_vis_task_batch'), g('g_vis_task_item'),
                 g('g_vis_task_run_id'), g('g_vis_batch_complete')))
        print('    calib: taught_mask=0x%X pick_region=%d mm_per_px=%.4f ref=(%.1f,%.1f) loaded_mask=0x%X'
              % (g('g_vis_taught_mask'), g('g_vis_pick_region'), gf('g_vis_mm_per_px'),
                 gf('g_vis_ref_x'), gf('g_vis_ref_y'), g('g_vis_loaded_mask')))
        print('    txn: result=%d rev=%d consumed=%d slot=%d | pick=%.2f ext=%.1f dx=%.2f dy=%.2f'
              % (g('g_vis_txn_result'), g('g_src_txn_revision'), g('g_src_txn_consumed'), g('g_src_slot'),
                 gf('g_src_pick_deg'), gf('g_src_extension_mm'),
                 gf('g_src_chassis_dx_mm'), gf('g_src_chassis_dy_mm')))
        print('    stage2: ready=%d body_state=%d session=%d reason=%d | pending seq=%d slot=%d grasped=%d'
              % (g('g_stage2_ready'), g('g_stage2_body_state'), g('g_stage2_body_session'),
                 g('g_stage2_body_reason'), g('g_vis_pending_seq'), g('g_vis_pending_slot'),
                 g('g_vis_pending_grasped')))
        print('    it=%d/%d cause=%d rej=%d act=%d | lift=%d cause=%d | ext=%d/%d | dm=%d/%d hold=%d'
              % (g('g_it_state'), g('g_it_action'), g('g_it_cause'), g('g_it_rejected'),
                 g('g_it_action'), g('g_lift_ruler_state'), g('g_lift_ruler_cause'),
                 g('g_startup_axes_state'), g('g_startup_axes_cause'),
                 g('g_dm_jog_state'), g('g_dm_jog_result'), g('g_dm_posvel_holding')))
        print('    plat: vm=%d can_guard=%d safety_fault=%d rtos_fault=%d | arm=%.2f lift=%.1f'
              % (g('g_vm_test_request'), g('g_can_guard_ready'), g('g_safety_fault'), g('g_rtos_fault'),
                 gf('g_dm_can2_logical_position_deg'), gf('g_vehicle_lift_current_deg')))

    dump('now')
    if watch > 0:
        b0 = g('g_vis_rx_bytes')
        m0 = g('g_vis_mat_cycle')
        c0 = g('g_vis_center_cycle')
        begin = time.monotonic()
        while time.monotonic() - begin < watch:
            time.sleep(0.5)
            print('  t=%4.1fs  rx_bytes=+%-6d mat_cycle=+%-3d center_cycle=+%-3d bad=%d'
                  % (time.monotonic() - begin, g('g_vis_rx_bytes') - b0,
                     g('g_vis_mat_cycle') - m0, g('g_vis_center_cycle') - c0, g('g_vis_rx_bad')),
                  flush=True)
        dump('after watch')
    s.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
