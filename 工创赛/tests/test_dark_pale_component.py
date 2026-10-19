"""Current observed face recovery without extending identity or freshness."""
import json
from pathlib import Path
import sys
import unittest
from zipfile import ZipFile
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'vision_app/code'));sys.path.insert(0,str(ROOT/'scripts'))
from project_detector import analyze
from analyze_dark_pale_v10 import candidate

class DarkPaleTests(unittest.TestCase):
    def test_actual_loss_images_recover_one_pale_face_and_one_blue_without_history(self):
        with ZipFile(ROOT/'evidence/20261006-202015-651610.zip') as z:
            cfg=candidate(json.loads(z.read('material_color_thresholds.json')))
            for name in ('first-lightblue-loss-raw.png','sample-07-raw.png','sample-08-raw.png',
                         'longest-lightblue-loss-before-raw.png'):
                frame=cv2.imdecode(np.frombuffer(z.read(name),np.uint8),1)
                result,_=analyze(frame,cfg,draw_overlay=False)
                known=[r for r in result['regions'] if r['label']!='UNKNOWN']
                self.assertEqual(sorted(r['label'] for r in known),['blue','lightblue'],name)
                pale=next(r for r in known if r['label']=='lightblue')
                self.assertGreaterEqual(pale['color_fractions']['lightblue'],.65)
                self.assertGreaterEqual(pale['arc_fraction'],.75)
                self.assertEqual(pale['search_reference'],'current_frame_component_bbox')

    def test_static_pair_and_recorded_blue_glare_do_not_gain_a_pale_false_positive(self):
        with ZipFile(ROOT/'evidence/20261006-202015-651610.zip') as z:
            cfg=candidate(json.loads(z.read('material_color_thresholds.json')))
        for path,expected in [('20261006-201311-289701.zip',['blue','lightblue']),
                              ('20261006-091047-121270.zip',['blue'])]:
            with ZipFile(ROOT/'evidence'/path) as z:
                names=['sample-12-raw.png'] if '201311' in path else ['first-blue-visible-raw.png']
                for name in names:
                    frame=cv2.imdecode(np.frombuffer(z.read(name),np.uint8),1)
                    result,_=analyze(frame,cfg,draw_overlay=False)
                    self.assertEqual(sorted(r['label'] for r in result['regions'] if r['label']!='UNKNOWN'),expected)

    def test_clipped_white_image_has_no_observed_material(self):
        with ZipFile(ROOT/'evidence/20261006-202015-651610.zip') as z:
            cfg=candidate(json.loads(z.read('material_color_thresholds.json')))
        result,_=analyze(np.full((960,1280,3),255,np.uint8),cfg,draw_overlay=False)
        self.assertFalse(any(r['label']!='UNKNOWN' for r in result['regions']))

if __name__=='__main__':unittest.main()
