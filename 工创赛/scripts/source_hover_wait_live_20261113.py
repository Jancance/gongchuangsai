"""One authorized empty-gripper hover move; no grasp, yaw, extension or body."""
import argparse
import hashlib
import json
import struct
import time
from datetime import datetime
from pathlib import Path
import gripper210_attach_read_20261105 as live
from source_body_probe_20261111 import attach

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'evidence/integration/2026-11-13/source-auto-index-observe-20261113-a'


def run(return_top=False, at_retracted=False, low_candidate=False, homeexit_candidate=False, observe_settle_candidate=False):
    model = json.loads((OUT / 'matlab-hover-wait.json').read_text(encoding='utf-8'))
    assert model['passed'] and model['user_confirmed_wait_clearance_mm'] == 20
    depth = int(model['wait_depth_virtual_pulses'])
    assert depth == 62400
    evidence = ROOT / ('evidence/integration/2026-11-14/low-pick' if low_candidate
                       else 'evidence/integration/2026-11-13/region-batch')
    if homeexit_candidate:
        evidence = ROOT / 'evidence/integration/2026-11-14/low-pick-homeexit'
    if observe_settle_candidate:
        evidence = ROOT / 'evidence/integration/2026-11-14/low-pick-observe-settle'
    candidate = json.loads((evidence / 'candidate.json').read_text(encoding='utf-8'))
    assert candidate['build_passed']
    image = candidate['images']['Vehicle_LiftRuler']
    live.IMAGE = ROOT / image['axf']
    assert hashlib.sha256(live.IMAGE.read_bytes()).hexdigest() == image['sha256']
    symbols, segments = live.load_image()
    path = OUT / ('hover-once-' + datetime.now().strftime('%Y%m%d-%H%M%S') + '.json')
    report = dict(complete=False, image_sha256=image['sha256'], flashed=False, reset=False,
                  requested_depth_virtual_pulses=depth, grasp_command=False,
                  yaw_command=False, extension_command=False, body_command=False,
                  actual_clearance_pending_observation=True, trace=[])
    report['return_to_original_safe_top'] = return_top
    report['expected_extension_mm'] = 0 if at_retracted else 65

    def save():
        path.write_text(json.dumps(report, indent=2), encoding='utf-8')

    save()
    wrote = False
    try:
        with attach() as session:
            target = session.target
            assert target.get_state().name == 'RUNNING'
            count = 0
            for address, data in segments:
                for offset in range(0, len(data), 4096):
                    part = data[offset:offset + 4096]
                    assert bytes(target.read_memory_block8(address + offset, len(part))) == part
                    count += len(part)
            report['verified_flash_bytes'] = count
            keys = ('g_it_state g_it_request g_it_action g_it_sequence g_it_cause g_it_rejected '
                    'g_region_state g_region_request g_it_auto_state g_pzq_near_state '
                    'g_safety_ready g_safety_fault g_rtos_fault g_can_test_result g_can_ctrl_fault_latched '
                    'g_chassis_auto_active g_source_parallel_cause g_source_parallel_stop_mask '
                    'g_vehicle_arm_cause g_vehicle_arm_stop_pending g_lift_ruler_cause '
                    'g_lift_ruler_stop_pending g_lift_compare_top_valid g_lift_compare_backoff_valid '
                    'g_lift_compare_top_raw g_lift_compare_safe_raw g_lift_ruler_options g_lift_ruler_samples '
                    'g_extension_home_reference_valid g_extension_hold_confirmed '
                    'g_dm_jog_first_fault g_startup_axes_cause g_it_place_depth_limit_pulses').split()
            floats = ['g_vehicle_lift_current_deg', 'g_dm_can2_logical_position_deg',
                      'g_startup_extension_current_deg', 'g_extension_home_reference_deg']
            if 'g_low_state' in symbols:
                keys.append('g_low_state')

            def snapshot():
                values = {key: target.read32(symbols[key]) for key in keys}
                for key in floats:
                    values[key] = struct.unpack('<f', struct.pack('<I', target.read32(symbols[key])))[0]
                values['gripper_pwm_us'] = target.read32(0x40000034)
                values['pe2'] = (target.read32(0x40021010) >> 2) & 1
                return values

            faults = ('g_it_cause g_it_rejected g_safety_fault g_rtos_fault g_can_ctrl_fault_latched '
                      'g_chassis_auto_active g_source_parallel_cause g_source_parallel_stop_mask '
                      'g_vehicle_arm_cause g_vehicle_arm_stop_pending g_lift_ruler_cause '
                      'g_lift_ruler_stop_pending g_dm_jog_first_fault g_startup_axes_cause').split()

            def healthy(values):
                return not any(values[key] for key in faults) and all(
                    values[key] == 1 for key in ('g_safety_ready', 'g_can_test_result', 'pe2'))

            before = snapshot()
            report['before'] = before
            save()
            assert healthy(before) and before['g_it_state'] == 2 and not before['g_it_request']
            assert before['g_region_state'] == 0 and not before['g_region_request']
            assert before['g_it_auto_state'] == 0 and before['g_pzq_near_state'] == 0
            assert before.get('g_low_state', 0) in (0, 2)
            assert all(before[key] == 1 for key in ('g_lift_compare_top_valid',
                                                  'g_extension_home_reference_valid', 'g_extension_hold_confirmed'))
            if not return_top:
                assert before['g_lift_compare_backoff_valid'] == 1
            assert before['gripper_pwm_us'] == 2056 and before['g_it_place_depth_limit_pulses'] == 0
            assert abs(before['g_dm_can2_logical_position_deg'] - 180) < 1
            extension = 0 if at_retracted else 65
            assert abs(before['g_startup_extension_current_deg'] - before['g_extension_home_reference_deg']
                       + extension * 315.5 * 360 / 3200) <= (2 if at_retracted else 45)
            if not return_top:
                target.write32(symbols['g_it_axes'], 4)
                target.write32(symbols['g_it_lift_pulses'], struct.unpack('<I', struct.pack('<f', float(depth)))[0])
            wrote = True
            request = 4 if return_top else 3
            target.write32(symbols['g_it_request'], request)
            begin = time.monotonic()
            try:
                while time.monotonic() - begin < 20:
                    values = snapshot()
                    report['trace'].append(values)
                    if not healthy(values) or values['g_it_state'] == 9:
                        raise RuntimeError('hover actual backend fault/rejection')
                    fresh = (values['g_lift_ruler_samples'] != before['g_lift_ruler_samples'] if return_top
                             else values['g_it_sequence'] != before['g_it_sequence'])
                    if (values['g_it_state'] == 2 and values['g_it_action'] == request and not values['g_it_request']
                            and fresh):
                        assert values['gripper_pwm_us'] == 2056
                        assert abs(values['g_dm_can2_logical_position_deg'] - 180) < 1
                        report.update(complete=True, after=values, elapsed_s=time.monotonic() - begin)
                        break
                    time.sleep(.12)
                else:
                    raise TimeoutError('one hover move')
            except BaseException:
                if wrote:
                    try:
                        target.write32(symbols['g_it_request'], 9)
                        report['stop_requested'] = True
                        for _ in range(12):
                            time.sleep(.15)
                            report['after_stop'] = snapshot()
                    except BaseException as stop_error:
                        report['stop_error'] = repr(stop_error)
                raise
    except BaseException as error:
        report['error'] = repr(error)
        raise
    finally:
        save()
        print(path, flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--once', action='store_true')
    parser.add_argument('--return-top', action='store_true')
    parser.add_argument('--at-retracted', action='store_true')
    parser.add_argument('--low-candidate', action='store_true')
    parser.add_argument('--homeexit-candidate', action='store_true')
    parser.add_argument('--observe-settle-candidate', action='store_true')
    args = parser.parse_args()
    if args.once or args.return_top:
        run(return_top=args.return_top, at_retracted=args.at_retracted, low_candidate=args.low_candidate,
            homeexit_candidate=args.homeexit_candidate, observe_settle_candidate=args.observe_settle_candidate)
    else:
        print('CHECK only. --once performs one empty-hover lift move; no pick or retry.')
