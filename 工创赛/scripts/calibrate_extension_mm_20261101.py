"""用尺量和本次完成状态生成伸缩轴毫米候选；故障/中止样本不计算比例。"""
from pathlib import Path
import argparse
import csv
import json
import math


def assess(row):
    reasons = []
    try:
        direction = row.get('direction', '')
        if direction not in ('retract', 'extend'):
            reasons.append('direction_unknown')
        pulses = int(row['command_pulses'])
        start, end = float(row['start_mm']), float(row['end_mm'])
        state, result = int(row['motion_state'], 0), int(row['motion_result'], 0)
        flags, speed = int(row['flags'], 0), int(row['speed_rpm'], 0)
        manual_stop = row.get('manual_stop', '').strip().lower()
        physical = row.get('actual_movement_confirmed', '').strip().lower()
        if pulses <= 0 or not math.isfinite(start) or not math.isfinite(end):
            reasons.append('invalid_measurement')
        if state != 7 or result != 1:
            reasons.append('not_successfully_completed')
        if (flags & 3) != 3 or (flags & 12) != 0 or speed != 0:
            reasons.append('drive_not_arrived_idle_without_stall')
        if manual_stop not in ('0', 'false', 'no'):
            reasons.append('manual_stop_or_unspecified')
        if physical not in ('1', 'true', 'yes'):
            reasons.append('physical_movement_not_confirmed')
        distance = abs(end - start)
        if not math.isfinite(distance) or distance <= 0:
            reasons.append('no_positive_measured_travel')
    except (KeyError, ValueError, TypeError):
        return {'sample_id': row.get('sample_id'), 'usable': False,
                'rejection_reasons': ['missing_or_invalid_fields'], 'input': row}
    record = {'sample_id': row.get('sample_id'), 'input': row,
              'usable': not reasons, 'rejection_reasons': reasons}
    if not reasons:
        record.update(direction=direction, command_pulses=pulses,
                      measured_travel_mm=distance, pulses_per_mm=pulses / distance)
    return record


def summarize(rows):
    samples = [assess(row) for row in rows]
    usable = [sample for sample in samples if sample['usable']]
    by_direction = {}
    for direction in ('retract', 'extend'):
        selected = [sample for sample in usable if sample['direction'] == direction]
        ratios = [sample['pulses_per_mm'] for sample in selected]
        by_direction[direction] = {
            'sample_count': len(selected),
            'candidate_pulses_per_mm': (sum(sample['command_pulses'] for sample in selected)
                                        / sum(sample['measured_travel_mm'] for sample in selected)) if selected else None,
            'minimum_sample_ratio': min(ratios) if ratios else None,
            'maximum_sample_ratio': max(ratios) if ratios else None,
            'repeatability_assessed': len(selected) >= 3,
        }
    return {
        'classification': 'E: provisional ruler-measured relative travel, not an automatic motion profile',
        'status': 'provisional_samples_only' if usable else 'awaiting_valid_measurements',
        'samples': samples, 'usable_sample_count': len(usable), 'by_direction': by_direction,
        'reported_remaining_retract_mm_before_test': 30,
        'remaining_distance_source': 'User report, not total travel or a calibrated zero.',
        'total_usable_travel_mm': None, 'backlash_mm': None, 'home_valid': False,
        'ruler_resolution_mm': None, 'absolute_position_verified': False,
        'metric_calibration_accepted': False, 'motion_allowed': False,
        'next': 'Measure accepted complete steps in both directions; inspect repeatability, reversal/backlash and usable endpoints independently. Never use commanded pulses for an interrupted step.',
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--samples', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    with args.samples.open(encoding='utf-8-sig', newline='') as stream:
        rows = list(csv.DictReader(stream))
    report = summarize(rows)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    print(json.dumps({'status': report['status'], 'usable_samples': report['usable_sample_count'],
                      'metric_calibration_accepted': False, 'output': str(args.output)}, ensure_ascii=False))


if __name__ == '__main__':
    main()
