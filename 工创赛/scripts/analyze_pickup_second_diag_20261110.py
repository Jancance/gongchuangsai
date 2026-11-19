"""Analyze the saved second-gear failure; never connect to hardware."""
from pathlib import Path
import csv
import json
import math

R = Path(__file__).resolve().parents[1]
E = R / 'evidence/integration/2026-11-10/second-diag'
P = E / 'integration13-three-immediate-once-20261110-222451.json'


def signed(sign, raw):
    assert sign in (0, 1)
    return -raw if sign else raw


def main():
    data = json.loads(P.read_text(encoding='utf-8'))
    fault = data['first_fault']
    ring = data['extension_motion_ring']
    assert ring['stable_read'] and ring['count_before'] == ring['count_after']
    count = ring['count_after']
    rows = [ring['raw_rows'][i % 128] for i in range(max(0, count - 128), count)]
    ack_tick = fault['g_ext_move_ack_tick']
    seq = fault['g_ext_move_ack_sequence']
    assert fault['g_ext_move_ack_result'] == 1 and fault['g_ext_move_ack_code'] == 2
    assert fault['g_ext_move_target_sequence'] == seq and fault['g_ext_move_target_result'] == 1
    raw = fault['ext_motion_target_raw']
    assert raw[0] == 0x33 and raw[6] == 0x6B
    target = signed(raw[1], int.from_bytes(bytes(raw[2:6]), 'big'))
    axis = fault['vsp_axes'][0]
    factor = 315.5 * 360 / 3200
    samples = []
    for row in rows:
        assert row[1] == seq and row[10:12] == [1, 2]
        samples.append({
            'position_time_s_after_ack': (row[2] - ack_tick) / 1000,
            'speed_time_s_after_ack': (row[0] - ack_tick) / 1000,
            'position_raw': signed(row[3], row[4]),
            'position_taught_mm': -signed(row[3], row[4]) / 10 / factor,
            'speed_rpm': signed(row[5], row[6]) / 10,
            'motor_flags': row[7],
        })
    assert len(samples) == 22
    fault_mm = abs(axis['position_minus_goal_deg']) / factor
    after = data['after_stop']
    summary = {
        'source': P.relative_to(R).as_posix(),
        'firmware_sha256': data['sha256'],
        'complete': data['complete'],
        'fault_item_zero_based': fault['g_it_auto_item'],
        'fault_elapsed_s': fault['g_it_auto_elapsed_ms'] / 1000,
        'stop_request_sent': data['stop_request_sent'],
        'stop_masks_clear': all(after[k] == 0 for k in ('g_source_parallel_stop_mask', 'g_lift_ruler_stop_pending', 'g_vehicle_arm_stop_pending')),
        'ack_code': 2,
        'accepted_driver_target_raw': target,
        'software_target_in_raw_convention': axis['goal'] * 10,
        'driver_minus_software_target_taught_mm': (target / 10 - axis['goal']) / factor,
        'sample_count': len(samples),
        'observed_speed_peak_rpm': max(abs(s['speed_rpm']) for s in samples),
        'last_three_sampled_speeds_rpm': [abs(s['speed_rpm']) for s in samples[-3:]],
        'fault_overrun_virtual_deg': abs(axis['position_minus_goal_deg']),
        'fault_overrun_taught_mm': fault_mm,
        'first_release_to_next_request_ms': fault['next_yaw_request_times_ms'][0] - fault['release_command_times_ms'][0],
        'position_virtual_units_per_mm': factor,
        'position_is_taught_virtual_coordinate': True,
        'position_not_new_ruler_measurement': True,
        'speed_and_position_samples_not_simultaneous': True,
        'after_stop_target_not_used_as_original_target': True,
        'inferences': [
            'This run accepted the intended final target; a large relative-origin error is not supported.',
            'Feedback shows terminal speed reduction then reacceleration and target overrun before STOP.',
            'PID, supply, load and mechanical causes cannot be uniquely identified from this trace.',
            'Delaying extension until half descent changes sequencing, not the demonstrated tracking error.',
        ],
        'high_speed_overrun_resolved': False,
    }
    assert summary['stop_request_sent'] and summary['stop_masks_clear']
    assert not summary['complete'] and math.isclose(fault_mm, 2.698359, abs_tol=0.001)
    (E / 'second-diag-analysis.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    with (E / 'second-diag-motion.csv').open('w', encoding='utf-8', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(samples[0]))
        writer.writeheader()
        writer.writerows(samples)
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
