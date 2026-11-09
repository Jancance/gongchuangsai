"""Post-flash verify + integrated-profile static snapshot. Read-only."""
from pathlib import Path
import json, hashlib, datetime
import source_stage2_readonly_sync_probe_20261105 as transport
from pyocd.core.helpers import ConnectHelper
from pyocd.core.target import Target
R = Path('.').resolve()
IMAGE = R / 'tmp/integration12_stage2_vision_20261107/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf'
transport.live.IMAGE = IMAGE
symbols, segments = transport.live.load_image()
opt = {'target_override': 'cortex_m', 'connect_mode': 'attach', 'frequency': 1000000,
       'auto_unlock': False, 'resume_on_disconnect': False, 'no_config': True,
       'cmsis_dap.prefer_v1': False, 'cmsis_dap.limit_packets': True}
s = ConnectHelper.session_with_chosen_probe(unique_id='6afaae24', options=opt)
s.open(); t = s.target
out = {'tag': 'integrated-profile-post-flash', 'when': datetime.datetime.now().isoformat(timespec='seconds'),
       'image': str(IMAGE.relative_to(R)), 'image_sha256': hashlib.sha256(IMAGE.read_bytes()).hexdigest(),
       'core_state': t.get_state().name, 'mismatch': None}
compared = 0
for address, data in segments:
    for offset in range(0, len(data), 4096):
        exp = data[offset:offset+4096]
        got = bytes(t.read_memory_block8(address+offset, len(exp)))
        if got != exp:
            out['mismatch'] = {'address': hex(address+offset)}; break
        compared += len(exp)
    if out['mismatch']: break
out['flash_matches'] = out['mismatch'] is None
out['flash_bytes_compared'] = compared
keys = ['g_safety_ready','g_safety_fault','g_rtos_fault','g_can_test_result','g_can_ctrl_fault_latched',
        'g_can_poll_enable','g_can_guard_ready','g_can_guard_cause','g_can_poll_valid_mask',
        'g_vm_test_request','g_startup_axes_vm_request','g_startup_axes_state','g_startup_axes_cause',
        'g_extension_home_reference_valid','g_extension_hold_confirmed',
        'g_lift_ruler_state','g_lift_ruler_cause','g_lift_ruler_vm_request',
        'g_vehicle_lift_reference_valid','g_vis_command_sent','g_vis_rx_bytes','g_uart6_rx_errors',
        'g_vis_material_valid','g_vis_center_valid','g_vis_mat_cycle','g_vis_center_cycle',
        'g_stage2_body_state','g_source_stage2_state','g_source_parallel_state',
        'g_it_state','g_it_request','g_it_lift_permit','g_dm_can2_state','g_vehicle_arm_state']
out['state'] = {}
for k in keys:
    if k in symbols:
        out['state'][k] = t.read32(symbols[k])
    else:
        out['state'][k] = 'MISSING'
print(json.dumps(out, ensure_ascii=False, indent=1))
ev = R / 'evidence/integration/2026-11-07/three-piece-i12/post-flash-integrated-20261107.json'
ev.parent.mkdir(parents=True, exist_ok=True)
ev.write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding='utf-8')
print('evidence=%s' % ev.relative_to(R))
