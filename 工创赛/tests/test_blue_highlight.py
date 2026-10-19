"""Geometry may use highlights; colour decisions require original evidence."""
import copy
import importlib
import json
from pathlib import Path
import sys
import unittest
from zipfile import ZipFile
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'vision_app/code'))
detector=importlib.import_module('project_detector')
CFG=json.loads((ROOT/'vision_app/code/config/material_color_thresholds.json').read_text(encoding='utf-8'))


class BlueHighlightTests(unittest.TestCase):
    def test_actual_blue_glare_loss_recovers_with_unchanged_color_votes(self):
        with ZipFile(ROOT/'evidence/20261006-085047-455357.zip') as archive:
            cfg=json.loads(archive.read('material_color_thresholds.json'))
            meta=json.loads(archive.read('first-blue-loss.json'))
            hints=meta['previous']['targets']
            frame=cv2.imdecode(np.frombuffer(archive.read('first-blue-loss-raw.png'),np.uint8),1)
        result,_=detector.analyze(frame,cfg,hints)
        known=[r for r in result['regions'] if r['label']!='UNKNOWN']
        self.assertEqual([r['label'] for r in known],['blue'])
        self.assertGreaterEqual(known[0]['arc_fraction'],.75)
        self.assertGreaterEqual(known[0]['color_fractions']['blue'],.65)

    def test_highlight_only_and_real_lightblue_are_not_classified_blue(self):
        hint=[{'color':'blue','center_px':[300,240],'bbox_px':[220,160,161,161],
               'proposal_source':'chromatic_circle','radius_px':80}]
        for color in [(255,255,255),(210,160,100)]:
            frame=np.full((480,640,3),150,np.uint8)
            cv2.circle(frame,(300,240),80,color,-1)
            result,_=detector.analyze(frame,CFG,hint)
            self.assertFalse(any(r['label']=='blue' for r in result['regions']))

    def test_captured_red_labels_unchanged_against_frozen_v4(self):
        with ZipFile(ROOT/'output/follow_touch_patch_v4_2026-10-06.zip') as package:
            baseline=type(sys)('v4_detector')
            exec(compile(package.read('vision_app/code/project_detector.py'),'v4_detector','exec'),baseline.__dict__)
        with ZipFile(ROOT/'evidence/20261006-083236-468596.zip') as archive:
            cfg=json.loads(archive.read('material_color_thresholds.json'))
            hints=json.loads(archive.read('first-red-loss.json'))['previous']['targets']
            for name in archive.namelist():
                if not name.endswith('-raw.png'):continue
                frame=cv2.imdecode(np.frombuffer(archive.read(name),np.uint8),1)
                known=lambda r:sorted(x['label'] for x in r['regions'] if x['label']!='UNKNOWN')
                self.assertEqual(known(detector.analyze(frame,cfg,copy.deepcopy(hints))[0]),
                                 known(baseline.analyze(frame,cfg,copy.deepcopy(hints))[0]),name)


if __name__=='__main__':unittest.main()
