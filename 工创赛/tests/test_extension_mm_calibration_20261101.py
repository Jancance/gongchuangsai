"""防止把堵转、中止、空白或非有限尺量误当毫米标定。"""
import importlib.util
from pathlib import Path
import unittest

SOURCE = Path(__file__).resolve().parents[1] / 'scripts/calibrate_extension_mm_20261101.py'
SPEC = importlib.util.spec_from_file_location('extension_mm_calibration', SOURCE)
CAL = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CAL)


class CalibrationEvidenceTests(unittest.TestCase):
    def sample(self, **changes):
        row = dict(sample_id='synthetic', direction='retract', command_pulses='1600',
                   start_mm='40', end_mm='30', motion_state='7', motion_result='1',
                   flags='0x03', speed_rpm='0', manual_stop='false',
                   actual_movement_confirmed='true', notes='synthetic only')
        row.update(changes)
        return row

    def test_known_distance_and_reverse_are_positive(self):
        for row in (self.sample(), self.sample(direction='extend', start_mm='30', end_mm='40')):
            self.assertEqual(CAL.assess(row)['pulses_per_mm'], 160)

    def test_captured_stall_pattern_cannot_produce_coefficient(self):
        checked = CAL.assess(self.sample(motion_state='9', motion_result='5', flags='0x05'))
        self.assertFalse(checked['usable'])
        self.assertNotIn('pulses_per_mm', checked)

    def test_manual_interrupt_cannot_use_commanded_pulses(self):
        self.assertFalse(CAL.assess(self.sample(manual_stop='true'))['usable'])

    def test_zero_nan_infinite_and_missing_distance_are_rejected(self):
        for value in ('40', 'nan', 'inf', ''):
            self.assertFalse(CAL.assess(self.sample(end_mm=value))['usable'])

    def test_motion_without_arrival_or_stop_is_rejected(self):
        for changes in (dict(flags='0x01'), dict(speed_rpm='5'), dict(actual_movement_confirmed='')):
            self.assertFalse(CAL.assess(self.sample(**changes))['usable'])

    def test_no_sample_does_not_invent_scale_or_total_travel(self):
        report = CAL.summarize([])
        self.assertEqual(report['usable_sample_count'], 0)
        self.assertIsNone(report['by_direction']['retract']['candidate_pulses_per_mm'])
        self.assertIsNone(report['total_usable_travel_mm'])

    def test_samples_never_enable_metric_motion_automatically(self):
        report = CAL.summarize([self.sample()] * 3)
        self.assertFalse(report['metric_calibration_accepted'])
        self.assertFalse(report['motion_allowed'])
        self.assertFalse(report['home_valid'])


if __name__ == '__main__':
    unittest.main()
