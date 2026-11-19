"""One explicit supervised region batch. Default checks local files only."""
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
EVIDENCE = ROOT / 'evidence/integration/2026-11-13/region-batch'


def manifest():
    initial_proof = json.loads((EVIDENCE / 'matlab-result.json').read_text(encoding='utf-8'))
    assert initial_proof['passed']
    plan_path = EVIDENCE / 'trial-plan.json'
    if not plan_path.is_file():
        plan_path = EVIDENCE / 'matlab-result.json'
    plan = json.loads(plan_path.read_text(encoding='utf-8'))
    candidate = json.loads((EVIDENCE / 'candidate.json').read_text(encoding='utf-8'))
    checks = json.loads((EVIDENCE / 'checks.json').read_text(encoding='utf-8'))
    assert plan['passed'] and checks['passed'] and candidate['build_passed']
    project = ROOT / candidate['project']
    for filename, digest in candidate['changes'].items():
        assert hashlib.sha256((project / filename).read_bytes()).hexdigest() == digest
    image = candidate['images']['Vehicle_LiftRuler']
    assert hashlib.sha256((ROOT / image['axf']).read_bytes()).hexdigest() == image['sha256']
    assert plan['task_colors'] == [2, 1, 5] and plan['near_180_zero_physically_confirmed_by_user']
    session = plan.get('source_session', 'align-after-back4-20261113-2136')
    assert Path(session).name == session
    raw = ROOT / 'evidence/integration/2026-11-13' / session / 'last-raw.png'
    assert hashlib.sha256(raw.read_bytes()).hexdigest() == plan['source_image_sha256']
    return plan, image


def run():
    raise RuntimeError('Superseded by the user: turntable auto-indexes with variable dwell. '
                       'Do not trigger the stationary three-piece owner on this scene.')


def _archived_stationary_run():
    plan, image = manifest()
    live.IMAGE = ROOT / image['axf']
    symbols, segments = live.load_image()
    path = EVIDENCE / ('region-live-' + datetime.now().strftime('%Y%m%d-%H%M%S') + '.json')
    report = dict(image_sha256=image['sha256'], one_batch_request=True, flashed=False,
                  reset=False, body_command=False, complete=False,
                  task_code='215+213+512+321', task_source='user supplied',
                  scene_must_remain_stopped_and_unchanged=True,
                  physical_grasp_and_placement_pending=True, trace=[])

    def save():
        path.write_text(json.dumps(report, indent=2), encoding='utf-8')

    save()
    wrote = accepted = rejected_unstarted = False
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
            keys = ('g_region_request g_region_state g_region_phase g_region_item '
                    'g_region_accepted_sequence g_region_rejected g_region_fault g_region_elapsed_ms '
                    'g_it_request g_it_state g_it_cause g_it_rejected g_it_auto_state '
                    'g_pzq_near_state g_safety_ready g_safety_fault g_rtos_fault '
                    'g_can_test_result g_can_ctrl_fault_latched g_chassis_auto_active '
                    'g_lift_ruler_cause g_lift_ruler_stop_pending g_vehicle_arm_cause '
                    'g_vehicle_arm_stop_pending g_source_parallel_cause g_source_parallel_stop_mask '
                    'g_startup_axes_cause g_dm_jog_first_fault g_lift_compare_top_valid '
                    'g_lift_compare_backoff_valid g_extension_home_reference_valid '
                    'g_extension_hold_confirmed g_vis_material_valid g_vis_mat_cycle '
                    'g_vis_mat_left g_vis_mat_right g_vis_mat_down g_vis_mat_tick g_src_now_ms').split()
            floats = ('g_dm_can2_logical_position_deg g_startup_extension_current_deg '
                      'g_extension_home_reference_deg').split()
            groups = []
            for address, name in sorted((symbols[key], key) for key in keys + floats):
                if not groups or address - groups[-1][-1][0] > 12 or address - groups[-1][0][0] > 128:
                    groups.append([])
                groups[-1].append((address, name))

            def snapshot():
                values = {}
                for group in groups:
                    start = group[0][0]
                    words = target.read_memory_block32(start, (group[-1][0] - start) // 4 + 1)
                    for address, name in group:
                        value = words[(address - start) // 4]
                        values[name] = struct.unpack('<f', struct.pack('<I', value))[0] if name in floats else value
                values['pe2'] = (target.read32(0x40021010) >> 2) & 1
                values['gripper_pwm_us'] = target.read32(0x40000034)
                values['release_ms'] = list(target.read_memory_block32(symbols['g_region_release_ms'], 3))
                return values

            faults = ('g_region_fault g_it_cause g_it_rejected g_safety_fault g_rtos_fault '
                      'g_can_ctrl_fault_latched g_chassis_auto_active g_lift_ruler_cause '
                      'g_lift_ruler_stop_pending g_vehicle_arm_cause g_vehicle_arm_stop_pending '
                      'g_source_parallel_cause g_source_parallel_stop_mask g_startup_axes_cause '
                      'g_dm_jog_first_fault').split()

            def healthy(values):
                return not any(values[key] for key in faults) and all(
                    values[key] == 1 for key in ('g_safety_ready', 'g_can_test_result', 'pe2'))

            before = snapshot()
            report['before'] = before
            save()
            assert healthy(before) and before['g_region_state'] in (0, 2)
            assert before['g_it_state'] == 2 and not before['g_it_request'] and not before['g_region_request']
            assert before['g_it_auto_state'] in (0, 2) and before['g_pzq_near_state'] in (0, 2)
            assert all(before[key] == 1 for key in ('g_lift_compare_top_valid', 'g_lift_compare_backoff_valid',
                                                  'g_extension_home_reference_valid', 'g_extension_hold_confirmed'))
            assert before['gripper_pwm_us'] == 2056
            assert abs(before['g_dm_can2_logical_position_deg'] - 180) < 1
            assert abs(before['g_startup_extension_current_deg'] - before['g_extension_home_reference_deg']
                       + 65 * 315.5 * 360 / 3200) <= 45
            observed = [before[key] for key in ('g_vis_mat_left', 'g_vis_mat_right', 'g_vis_mat_down')]
            assert observed == plan['colors_by_region'] and before['g_vis_material_valid'] == 1
            assert (before['g_src_now_ms'] - before['g_vis_mat_tick']) & 0xffffffff <= 1000
            sequence = (before['g_region_accepted_sequence'] + 1) & 0xffffffff or 1
            report['command_sequence'] = sequence
            for name, values in [('g_region_colors', plan['colors_by_region']),
                                 ('g_region_task_colors', plan['task_colors'])]:
                target.write_memory_block32(symbols[name], values)
            for name, values in [('g_region_yaw_deg', plan['region_yaw_deg']),
                                 ('g_region_extension_mm', plan['region_extension_mm'])]:
                target.write_memory_block32(symbols[name], [struct.unpack('<I', struct.pack('<f', v))[0] for v in values])
            target.write32(symbols['g_region_sequence'], sequence)
            target.write32(symbols['g_region_observation_cycle'], target.read32(symbols['g_vis_mat_cycle']))
            wrote = True
            try:
                target.write32(symbols['g_region_request'], 1)
            except BaseException:
                # A transport exception does not prove the target missed the write.
                try:
                    target.write32(symbols['g_region_request'], 9)
                    target.write32(symbols['g_it_request'], 9)
                    report['stop_requested_after_uncertain_start'] = True
                except BaseException as stop_error:
                    report['stop_error'] = repr(stop_error)
                raise
            begin = time.monotonic()
            last_phase = None
            try:
                while time.monotonic() - begin < 120:
                    values = snapshot()
                    report['trace'].append(values)
                    accepted = values['g_region_accepted_sequence'] == sequence
                    if not healthy(values) or values['g_region_state'] == 9:
                        raise RuntimeError('actual region/backend fault')
                    if values['g_region_rejected'] != before['g_region_rejected']:
                        rejected_unstarted = (not accepted and values['g_region_state'] in (0, 2)
                                              and not values['g_region_request'] and not values['g_it_request'])
                        raise RuntimeError('batch request rejected; no automatic retry')
                    phase = (values['g_region_item'], values['g_region_phase'])
                    if phase != last_phase:
                        print('item/phase', phase, 'release_ms', values['release_ms'], flush=True)
                        last_phase = phase
                    if accepted and values['g_region_state'] == 2:
                        assert all(values['release_ms']) and values['g_it_state'] == 2 and not values['g_it_request']
                        assert abs(values['g_dm_can2_logical_position_deg'] - 180) < 1
                        assert abs(values['g_startup_extension_current_deg'] - values['g_extension_home_reference_deg']
                                   + 65 * 315.5 * 360 / 3200) <= 45
                        assert values['gripper_pwm_us'] == 2056
                        report.update(complete=True, after=values, command_chain_elapsed_ms=values['g_region_elapsed_ms'])
                        break
                    time.sleep(.12)
                else:
                    raise TimeoutError('one region batch')
            except BaseException:
                if wrote and not rejected_unstarted:
                    try:
                        target.write32(symbols['g_region_request'], 9)
                        target.write32(symbols['g_it_request'], 9)
                        report['stop_requested'] = True
                        for _ in range(12):
                            time.sleep(.15)
                            values = snapshot()
                            report['after_stop'] = values
                            if (values['g_it_state'] == 9 and not values['g_it_request'] and
                                not any(values[key] for key in ('g_lift_ruler_stop_pending', 'g_vehicle_arm_stop_pending',
                                                               'g_source_parallel_stop_mask'))):
                                break
                    except BaseException as stop_error:
                        report['stop_error'] = repr(stop_error)
                raise
            finally:
                report['last'] = snapshot()
    except BaseException as error:
        report['error'] = repr(error)
        report['request_written'] = wrote
        raise
    finally:
        save()
        print(path, flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--once-batch', action='store_true')
    args = parser.parse_args()
    if args.once_batch:
        run()
    else:
        manifest()
        print('CHECK only: no devices. --once-batch starts one explicit board-owned three-piece trial.')
