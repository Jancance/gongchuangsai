"""Match the accepted image before reading symbols. No reset/halt/flash/motion."""
from datetime import datetime
import hashlib
import json
from pathlib import Path
import struct
import time
import gripper210_attach_read_20261105 as live

ROOT = Path(__file__).resolve().parents[1]
E = ROOT / 'evidence/integration/2026-11-07/stage2-mainline'
live.IMAGE = ROOT / 'tmp/stage2_three_overlap_20261109/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf'
SHA = '4aa68a090da08bcebeb28e3a8b6b57ee897a85b48ee7f1fd0c73eebb158d18dd'
assert hashlib.sha256(live.IMAGE.read_bytes()).hexdigest() == SHA
symbols, segments = live.load_image()
dest = E / ('integration13-readonly-'+datetime.now().strftime('%Y%m%d-%H%M%S')+'.json')
assert not dest.exists()
out = {'image_sha256': SHA, 'flash_matches': False, 'flash_bytes': 0,
       'actuator_writes': 0, 'reset': False, 'halt': False, 'flashed': False, 'samples': []}
names = '''g_src_now_ms g_it_state g_it_action g_it_request g_it_rejected g_it_cause g_it_sequence
g_safety_ready g_safety_fault g_rtos_fault g_can_test_result g_can_ctrl_fault_latched
g_startup_axes_state g_startup_axes_cause g_extension_home_reference_valid g_extension_hold_confirmed
g_lift_ruler_state g_lift_ruler_cause g_lift_ruler_request g_lift_ruler_samples g_lift_ruler_after_raw
g_lift_compare_top_valid g_lift_compare_backoff_valid g_lift_compare_safe_raw g_lift_compare_top_raw
g_lift_compare_operation g_lift_compare_target_pulses g_lift_ruler_stop_pending
g_vehicle_arm_state g_vehicle_arm_cause g_vehicle_arm_stop_pending
g_source_parallel_state g_source_parallel_cause g_source_parallel_stop_mask g_source_stage2_state
g_chassis_auto_active g_vm_test_request g_dm_jog_request g_dm_jog_state g_dm_jog_first_fault
g_dm_posvel_holding g_dm_can2_feedback_valid g_dm_can2_feedback_tick g_dm_can2_motor_state
g_vis_loaded_mask g_vis_pending_seq g_vis_batch_complete g_can_guard_ready g_can_guard_cause
g_can_guard_bad_nodes g_can_poll_valid_mask g_can_poll_count'''.split()
floats = '''g_dm_can2_logical_position_deg g_startup_extension_current_deg
g_extension_home_reference_deg g_vehicle_lift_current_deg'''.split()
try:
    session = live.attach()
    if session is None:
        raise RuntimeError('Probe session unavailable; no action')
    with session:
        target = session.target
        out['core'] = target.get_state().name
        assert out['core'] == 'RUNNING', 'Core not running; no action'
        for addr, data in segments:
            for i in range(0, len(data), 4096):
                expected = data[i:i+4096]
                assert bytes(target.read_memory_block8(addr+i, len(expected))) == expected, \
                    f'Flash mismatch at {addr+i:#x}; no symbol access or writes'
                out['flash_bytes'] += len(expected)
        out['flash_matches'] = True
        for _ in range(2):
            sample = {k: target.read32(symbols[k]) for k in names if k in symbols}
            for k in floats:
                sample[k] = struct.unpack('<f', struct.pack('<I', target.read32(symbols[k])))[0]
            sample['pe2'] = (target.read32(0x40021010) >> 2) & 1
            sample['gripper_pwm_us'] = target.read32(0x40000034)
            sample['carrier_pwm_us'] = target.read32(0x40000440)
            out['samples'].append(sample)
            time.sleep(.25)
except Exception as exc:
    out['error'] = repr(exc)
    raise
finally:
    dest.write_text(json.dumps(out, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(out, ensure_ascii=False, indent=2))
    print(dest)
