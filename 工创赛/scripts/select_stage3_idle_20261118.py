"""Select area 3 in RAM only; default is local audit, --select permits one write.

No movement/request, reference initialization, reset/halt/resume or flashing.
The caller must leave PC13 released throughout; idle samples are not atomic.
"""
import argparse
from datetime import datetime
import json
import sys

import stage3_resume_status_20261118 as status

ZERO = '''g_source_cycle_state g_source_cycle_request g_source_cycle_fault
g_stage3_state g_stage3_request g_stage3_teach_request g_stage3_fault
g_it_request g_it_cause g_it_rejected g_low_request g_low_fault g_low_rejected
g_source_body_step_request g_source_body_step_reason g_source_align_chain_active
g_chassis_auto_active g_chassis_auto_cause g_safety_fault g_rtos_fault
g_can_ctrl_fault_latched g_dm_jog_first_fault g_startup_axes_cause g_lift_ruler_cause
g_lift_ruler_stop_pending g_vehicle_arm_stop_pending g_source_parallel_stop_mask
g_dm_integrated_owner g_vehicle_arm_axis_owned g_vehicle_arm_dm_owned
g_it_auto_fault g_motion_diag_fault_latched g_region_fault g_source_stage2_fault
g_stage2_fast_fault s3_pending s3_teach_pending vsp_owned'''.split()
EXTRA = '''g_region_dispatch_area g_start_key_count g_source_cycle_state
g_source_cycle_request g_source_cycle_fault g_stage3_state g_stage3_request
g_stage3_teach_request g_stage3_fault g_dm_integrated_owner g_vehicle_arm_axis_owned
g_vehicle_arm_dm_owned g_it_auto_fault g_motion_diag_fault_latched g_region_fault
g_source_stage2_fault g_stage2_fast_fault s3_pending s3_teach_pending vsp_owned'''.split()


def read_state(board, target, objects, types):
    value = board.snapshot()
    for name in EXTRA:
        address, size = objects[name]
        if size != 4 or types.get(name) != {'size': 4, 'encoding': 7}:
            raise ValueError('Expected existing uint32 ELF field: ' + name)
        value[name] = target.read32(address)
    value['core_state'] = target.get_state().name
    value['pc13_input_high'] = bool(target.read32(0x40020810) & (1 << 13))
    return value


def check_idle(value):
    failures = [name for name in ZERO if value[name] != 0]
    for name in ('g_it_state', 'g_low_state'):
        if value[name] not in (0, 2):
            failures.append(name)
    if value['g_source_body_step_state'] not in (0, 3):
        failures.append('g_source_body_step_state')
    if value['core_state'] != 'RUNNING' or not value['pc13_input_high']:
        failures.append('running core / released PC13')
    if value['g_region_dispatch_area'] not in (1, 3):
        failures.append('g_region_dispatch_area')
    if failures:
        raise RuntimeError('Existing idle conditions not met: ' + ', '.join(failures))


def select(axf, image, objects, types, report):
    import gripper210_attach_read_20261105 as live
    from source_body_probe_20261111 import attach
    from smart_carrier.vision_manipulation.committed_pick_board import CommittedPickBoard
    live.IMAGE = axf
    symbols, segments = live.load_image()
    if sum(len(data) for _, data in segments) != image['flash_bytes']:
        raise ValueError('Unexpected candidate Flash byte count')
    report['hardware_access_attempted'] = True
    session = attach()
    try:
        session.open()
        report['hardware_access'] = True
        target = status.ReadOnlyTarget(session.target)
        board = CommittedPickBoard(target, symbols, segments)
        report['verified_flash_bytes'] = board.verified_flash_bytes
        report['before'] = read_state(board, target, objects, types)
        check_idle(report['before'])
        if report['before']['g_region_dispatch_area'] == 1:
            report['writes_attempted'] = 1
            report['write_outcome_uncertain'] = True
            session.target.write32(symbols['g_region_dispatch_area'], 3)
            report['readback'] = target.read32(symbols['g_region_dispatch_area'])
            if report['readback'] != 3:
                raise RuntimeError('Area write not confirmed; no retry')
            report['writes_confirmed'] = 1
            report['write_outcome_uncertain'] = False
        else:
            report['readback'] = target.read32(symbols['g_region_dispatch_area'])
            if report['readback'] != 3:
                raise RuntimeError('Area changed during zero-write selection; no retry')
        report['after'] = read_state(board, target, objects, types)
        report['keycount_unchanged'] = report['before']['g_start_key_count'] == report['after']['g_start_key_count']
        check_idle(report['after'])
        if not report['keycount_unchanged'] or report['after']['g_region_dispatch_area'] != 3:
            raise RuntimeError('Post-selection state changed; recorded only, no rollback or retry')
    finally:
        previous_error = sys.exc_info()[0] is not None
        try:
            session.close()
        except Exception as error:
            report['disconnect_error'] = type(error).__name__ + ': ' + str(error)
            if not previous_error:
                raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--select', action='store_true', help='Explicit single area=3 RAM configuration write')
    args = parser.parse_args()
    report = dict(created_at=datetime.now().astimezone().isoformat(), select_requested=args.select,
                  hardware_access=False, hardware_access_attempted=False, writes_attempted=0,
                  writes_confirmed=0, write_outcome_uncertain=False, readback=None,
                  motion_requests=0, reset_requested=False, halt_requested=False,
                  resume_requested=False, flash_requested=False,
                  write_scope='Only g_region_dispatch_area=3; references may remain invalid; not motion acceptance')
    code = 0
    try:
        axf, image, audit = status.local_audit()
        report['local_audit'] = audit
        if not audit['source_manifest_matches']:
            raise ValueError('Candidate source manifest mismatch; no hardware access')
        if args.select:
            objects, types = status.objects_and_types(axf)
            select(axf, image, objects, types, report)
    except Exception as error:
        report['error'] = dict(type=type(error).__name__, message=str(error))
        code = 1
    status.OUT.mkdir(parents=True, exist_ok=True)
    path = status.OUT / ('select-stage3-' + datetime.now().strftime('%Y%m%d-%H%M%S-%f') + '.json')
    with path.open('x', encoding='utf-8') as stream:
        json.dump(status.json_safe(report), stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    summary = {key: report.get(key) for key in ('hardware_access', 'writes_attempted',
               'writes_confirmed', 'write_outcome_uncertain', 'readback', 'keycount_unchanged', 'error')}
    summary['evidence'] = str(path)
    print(json.dumps(summary, ensure_ascii=False))
    return code


if __name__ == '__main__':
    raise SystemExit(main())
