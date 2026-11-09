"""蓝色边界修复必须保留颜色纯度、几何、边界和禁用回退。"""
from pathlib import Path
import copy
import importlib.util
import json
import sys
import unittest

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
APP = ROOT / 'output/p2_vision_generic135_blue2_20261026/vision_app'
BASE = ROOT / 'output/p2_vision_generic135_perf1_20261026/vision_app'
sys.path.insert(0, str(APP / 'code'))


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


new = load('blue2_test_new', APP / 'code/project_detector.py')
old = load('blue2_test_old', BASE / 'code/project_detector.py')


class Blue2Tests(unittest.TestCase):
    def test_actual_missing_frame_recovers_blue_and_flag_off_is_exact(self):
        cfg = json.loads((APP / 'code/config/material_color_thresholds.json').read_text(encoding='utf-8'))
        frame = cv2.imdecode(np.frombuffer((ROOT / 'evidence/p2-camera/2026-10-26/rejection-audit/first-nonstale-blue-missing-raw.png').read_bytes(), np.uint8), 1)
        before, _ = old.analyze(frame, cfg, draw_overlay=False)
        after, _ = new.analyze(frame, cfg, draw_overlay=False)
        self.assertEqual([r['label'] for r in before['regions'] if r['label'] != 'UNKNOWN'], ['lightblue'])
        self.assertEqual([r['label'] for r in after['regions'] if r['label'] != 'UNKNOWN'], ['lightblue', 'blue'])
        blue = next(r for r in after['regions'] if r['label'] == 'blue')
        self.assertTrue(.94 <= blue['solidity'] < .95)
        self.assertGreaterEqual(blue['color_fractions']['blue'], .90)
        off = copy.deepcopy(cfg)
        off['blue_projected_silhouette_candidate']['solidity94_candidate'] = False
        self.assertEqual(before, new.analyze(frame, off, draw_overlay=False)[0])

    def test_projected_branch_keeps_border_aspect_concavity_and_purity_rejections(self):
        parent = np.array([[[0, 0]], [[399, 0]], [[399, 259]], [[0, 259]]], np.int32)
        for case in ('valid', 'border', 'elongated', 'concave', 'impure'):
            masks = {i: np.zeros((260, 400), np.uint8) for i in range(1, 7)}
            center = (60, 130) if case == 'border' else (200, 130)
            axes = (110, 40) if case == 'elongated' else (85, 50)
            cv2.ellipse(masks[3], center, axes, 0, 0, 360, 255, -1)
            if case == 'concave':
                cv2.rectangle(masks[3], (180, 70), (220, 130), 0, -1)
            if case == 'impure':
                masks[3][105:155, 170:230] = 0
                masks[5][105:155, 170:230] = 255
            frame = np.full((260, 400, 3), (180, 10, 10), np.uint8)
            rows = new.chromatic_fallback(frame, masks, [parent], [], hsv=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV),
                                          blue_settings={'enabled': True, 'solidity94_candidate': True})
            with self.subTest(case=case):
                self.assertEqual(len(rows), 1 if case == 'valid' else 0)

    def test_original_valid_circle_center_is_preserved_with_history(self):
        cfg = json.loads((APP / 'code/config/material_color_thresholds.json').read_text(encoding='utf-8'))
        frame = cv2.imdecode(np.frombuffer((ROOT / 'evidence/p2-camera/2026-10-26/rejection-audit/first-nonstale-blue-missing-raw.png').read_bytes(), np.uint8), 1)
        hints = [{'color': 'blue', 'bbox_px': [348, 511, 265, 170], 'center_px': [480, 596],
                  'proposal_source': 'projected_blue_component'},
                 {'color': 'lightblue', 'bbox_px': [1274, 524, 237, 155], 'center_px': [1388, 604]}]
        before = old.analyze(frame, cfg, prior_regions=hints, draw_overlay=False)[0]
        self.assertTrue(any(r['label'] == 'blue' for r in before['regions']))
        self.assertEqual(before, new.analyze(frame, cfg, prior_regions=hints, draw_overlay=False)[0])


if __name__ == '__main__':
    unittest.main()
