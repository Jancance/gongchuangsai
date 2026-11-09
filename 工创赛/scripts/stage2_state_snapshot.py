"""Read-only snapshot of the safety/CAN/lift globals on the board.

No writes, no reset, no motion. Saves an evidence JSON so the state is
documented instead of only printed to the console.
"""
from pathlib import Path
import json
import datetime
import source_stage2_readonly_sync_probe_20261105 as transport
from pyocd.core.helpers import ConnectHelper
from pyocd.core.target import Target

R = Path(__file__).resolve().parents[1]
IMAGE = R / 'tmp/integration12_stage2_vision_20261107/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf'
transport.live.IMAGE = IMAGE
names, _ = transport.live.load_image()
opt = {'target_override': 'cortex_m', 'connect_mode': 'attach', 'frequency': 1000000,
       'auto_unlock': False, 'resume_on_disconnect': False, 'no_config': True,
       'cmsis_dap.prefer_v1': False, 'cmsis_dap.limit_packets': True}
s = ConnectHelper.session_with_chosen_probe(unique_id='6afaae24', options=opt)
s.open()
t = s.target
state = t.get_state()
print('target_state=%s' % state)

KEYS = [
    'g_safety_ready', 'g_safety_fault', 'g_safety_count', 'g_rtos_fault',
    'g_can_ctrl_fault_latched', 'g_can_guard_ready', 'g_can_guard_cause',
    'g_can_guard_bad_nodes', 'g_can_test_result',
    'g_vm_test_request', 'g_startup_axes_vm_request', 'g_startup_axes_state',
    'g_startup_axes_cause',
    'g_can_poll_valid_mask', 'g_can_poll_result', 'g_can_poll_count',
    'g_can_status_result', 'g_can_status_esr',
    'g_it_state', 'g_it_cause', 'g_it_action', 'g_it_request', 'g_it_rejected',
    'g_lift_ruler_state', 'g_lift_ruler_cause', 'g_lift_ruler_phase',
    'g_lift_ruler_stop_pending', 'g_lift_compare_operation',
    'g_lift_compare_target_pulses', 'g_lift_compare_safe_raw',
    'g_lift_compare_top_valid', 'g_lift_compare_backoff_valid',
    'g_extension_home_reference_valid', 'g_extension_hold_confirmed',
    'g_dm_can2_state', 'g_dm_can2_result', 'g_dm_can2_feedback_valid',
    'g_dm_can2_motor_state', 'g_it_carrier_deg',
    'g_source_parallel_state', 'g_source_parallel_cause',
    'g_vehicle_arm_state', 'g_vis_material_valid', 'g_vis_center_valid',
]
out = {'tag': 'state-snapshot-readonly', 'when': datetime.datetime.now().isoformat(timespec='seconds'),
       'target_state': str(state), 'image': str(IMAGE.relative_to(R)), 'values': {}}
for k in KEYS:
    if k not in names:
        out['values'][k] = 'MISSING_SYMBOL'
        continue
    v = t.read32(names[k])
    if v >= (1 << 31):
        v -= (1 << 32)
    out['values'][k] = v
for k, v in out['values'].items():
    print('%-34s %s' % (k, v))

ev = R / 'evidence/integration/2026-11-07/stage2-mainline/state-snapshot-20261107-latest.json'
ev.parent.mkdir(parents=True, exist_ok=True)
ev.write_text(json.dumps(out, indent=2, ensure_ascii=False), encoding='utf-8')
print('evidence=%s' % ev.relative_to(R))
