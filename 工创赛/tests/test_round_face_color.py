"""Current top-face colour evidence must override an uncertain sidewall vote."""
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
tracker_module=importlib.import_module('project_tracker')


class RoundFaceColorTests(unittest.TestCase):
    def test_actual_lightblue_face_overrides_low_confidence_blue_body(self):
        with ZipFile(ROOT/'evidence/20261006-183128-046231.zip') as archive:
            cfg=json.loads(archive.read('material_color_thresholds.json'))
            frame=cv2.imdecode(np.frombuffer(archive.read('first-lightblue-with-blue-raw.png'),np.uint8),1)
        original=copy.deepcopy(cfg)
        result,_=detector.analyze(frame,cfg)
        known=[r for r in result['regions'] if r['label']!='UNKNOWN']
        self.assertEqual([r['label'] for r in known],['lightblue'])
        self.assertEqual(known[0]['proposal_source'],'bright_round_face')
        self.assertGreaterEqual(known[0]['color_fractions']['lightblue'],.65)
        self.assertEqual(cfg,original)

    def test_body_to_face_scale_uses_actual_diameter_and_preserves_id(self):
        tracker=tracker_module.TabletopTracker()
        parent={'label':'lightblue','bbox_px':[780,250,360,330],
                'area_px2':90000,'silhouette_center_px':[970,430],'proposal_source':'combined_silhouette'}
        face={'label':'lightblue','bbox_px':[910,345,220,220],
              'area_px2':35000,'silhouette_center_px':[1020,455],'proposal_source':'bright_round_face'}
        first=tracker.update([parent],1.)
        second=tracker.update([face],1.1)
        self.assertEqual(second['observed'][0]['id'],first['observed'][0]['id'])
        self.assertTrue(second['observed'][0]['geometry_reference_changed'])

    def test_reference_change_does_not_accept_an_unrelated_tiny_region(self):
        tracker=tracker_module.TabletopTracker()
        parent={'label':'lightblue','bbox_px':[80,80,240,280],
                'area_px2':55000,'silhouette_center_px':[200,200],'proposal_source':'combined_silhouette'}
        small={'label':'lightblue','bbox_px':[180,180,40,40],
               'area_px2':1200,'silhouette_center_px':[200,200],'proposal_source':'bright_round_face'}
        old=tracker.update([parent],1.)['observed'][0]['id']
        new=tracker.update([small],1.1)['observed'][0]['id']
        self.assertNotEqual(old,new)


if __name__=='__main__':unittest.main()
