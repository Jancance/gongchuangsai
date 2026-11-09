"""异常取证应区分过期和真实漏检，并保存对应原图且不改变检测结果。"""
import copy
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
FOLDER = ROOT / 'output/p2_generic135_rejection_audit_20261026'
sys.path.insert(0, str(FOLDER))
spec = importlib.util.spec_from_file_location('rejection_audit', FOLDER / 'p2_color_follow_audit.py')
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


class RejectionAuditTests(unittest.TestCase):
    def test_stale_does_not_consume_missing_event_and_first_real_failure_is_preserved(self):
        with tempfile.TemporaryDirectory() as temp:
            rec = audit.ColorRecorder(Path(temp) / 'run', 'blue')
            for index, state in enumerate(('STALE_RESULT', 'FOLLOWING_PIXEL', 'FOLLOWING_PIXEL')):
                stamp = 100. + index
                result = {'state': state, 'timestamp': stamp,
                          'targets': [], 'detector_regions': [{'label': 'lightblue'}],
                          'ready': False, 'motion_allowed': False}
                original = copy.deepcopy(result)
                rec.observe(np.full((16, 16, 3), index * 20, np.uint8), result, stamp, stamp + .01)
                self.assertEqual(original, result)
            rec.close(0)
            name = 'first-nonstale-blue-missing'
            data = json.loads((rec.folder / (name + '.json')).read_text(encoding='utf-8'))
            self.assertEqual(data['current']['frame_index'], 2)
            self.assertEqual(data['previous']['frame_index'], 1)
            self.assertTrue(np.all(cv2.imread(str(rec.folder / (name + '-raw.png'))) == 20))
            self.assertEqual(len(list(rec.folder.glob(name + '-raw.png'))), 1)

    def test_duplicate_candidates_captured_even_when_tracker_hides_color(self):
        with tempfile.TemporaryDirectory() as temp:
            rec = audit.ColorRecorder(Path(temp) / 'run', 'blue')
            for index, state in enumerate(('STALE_RESULT', 'FOLLOWING_PIXEL', 'FOLLOWING_PIXEL')):
                stamp = 100. + index
                result = {'state': state, 'timestamp': stamp,
                          'targets': [{'color': 'blue', 'id': 'T001'}] if index else [],
                          'detector_regions': [{'label': 'lightblue'}, {'label': 'lightblue'}, {'label': 'blue'}],
                          'ready': False, 'motion_allowed': False}
                original = copy.deepcopy(result)
                rec.observe(np.full((16, 16, 3), index * 30, np.uint8), result, stamp, stamp + .01)
                self.assertEqual(original, result)
            rec.close(0)
            data = json.loads((rec.folder / 'first-duplicate-lightblue.json').read_text(encoding='utf-8'))
            self.assertEqual(data['current']['frame_index'], 2)
            self.assertFalse((rec.folder / 'first-nonstale-blue-missing.json').exists())
            self.assertTrue(np.all(cv2.imread(str(rec.folder / 'first-duplicate-lightblue-raw.png')) == 30))


if __name__ == '__main__':
    unittest.main()
