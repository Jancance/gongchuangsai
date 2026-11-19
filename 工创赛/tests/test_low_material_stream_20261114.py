"""Validate partial observations and pose heartbeat before enabling serial output."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('low_stream', ROOT / 'output/low_material_stream_20261114.py')
stream = importlib.util.module_from_spec(spec)
spec.loader.exec_module(stream)


def detection(colors, shift=0):
    return dict(calibrated=True, slot_colors=colors,
                regions=[dict(color_id=c, center_x_px=400 + i * 300 + shift,
                              center_y_px=200 + i * 200) for i, c in enumerate(colors) if c])


class LowStreamTests(unittest.TestCase):
    def test_actual_remaining_three_two_one(self):
        for colors in [(2, 5, 1), (0, 5, 1), (0, 5, 0)]:
            gate = stream.RemainingStopped()
            remaining = tuple(c for c in colors if c)
            results = [gate.update(detection(colors), t, remaining=remaining,
                                   pose_id='low65', pose_sequence=4) for t in (0, .06, .12)]
            self.assertEqual(results, [False, False, True])

    def test_missing_target_is_not_filled_from_task(self):
        gate = stream.RemainingStopped()
        d = detection((0, 5, 0))
        for t in (0, .06, .12):
            ready = gate.update(d, t, remaining=(1, 5), pose_id='target-pose', pose_sequence=2)
        self.assertTrue(ready)
        self.assertEqual(d['slot_colors'], (0, 5, 0))
        self.assertNotIn(1, d['slot_colors'])
        self.assertFalse(gate.update(detection((5, 5, 0)), .13,
                                     remaining=(1, 5), pose_id='target-pose', pose_sequence=2))

    def test_motion_pose_changes_and_invalid_clock_clear_readiness(self):
        gate = stream.RemainingStopped()
        for t, shift in [(0, 0), (.06, 10), (.12, 20)]:
            self.assertFalse(gate.update(detection((1, 0, 0), shift), t,
                                         remaining=(1,), pose_id='low', pose_sequence=2))
        for t in (.3, .36, .42):
            ready = gate.update(detection((1, 0, 0)), t, remaining=(1,), pose_id='low', pose_sequence=2)
        self.assertTrue(ready)
        self.assertFalse(gate.update(detection((1, 0, 0)), .43, remaining=(1,),
                                     pose_id='other', pose_sequence=3))
        self.assertFalse(gate.update(detection((1, 0, 0)), .42, remaining=(1,),
                                     pose_id='other', pose_sequence=3))

    def test_heartbeat_expires_and_moving_mode_disables(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'control.json'
            value = dict(session_id='trial', pose_id='low65', pose_sequence=4,
                         state='CONFIRM', remaining_colors=[2, 1, 5], written_monotonic_s=10)
            path.write_text(json.dumps(value), encoding='utf-8')
            self.assertIsNotNone(stream.read_control(path, 10.5))
            self.assertIsNone(stream.read_control(path, 10.7))
            self.assertIsNone(stream.read_control(path, 9.9))
            value['state'] = 'MOVING'
            path.write_text(json.dumps(value), encoding='utf-8')
            self.assertIsNone(stream.read_control(path, 10.1))
            path.write_text('{', encoding='utf-8')
            self.assertIsNone(stream.read_control(path, 10.1))


if __name__ == '__main__':
    unittest.main()
