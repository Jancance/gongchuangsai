"""Current edges restore black when supported; fresh boxes never become observations alone."""
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
from analyze_black_edge_v12 import candidate

class BlackEdgeTests(unittest.TestCase):
    def test_first_glare_shape_can_initialize_from_current_compact_region(self):
        with ZipFile(ROOT/'evidence/20261007-004517-208447.zip') as z:
            cfg=candidate(json.loads(z.read('material_color_thresholds.json')))
            frame=cv2.imdecode(np.frombuffer(z.read('first-frame-raw.png'),np.uint8),1)
        result,_=analyze(frame,cfg,draw_overlay=False)
        black=[r for r in result['regions'] if r['label']=='black']
        self.assertEqual(len(black),1)
        self.assertEqual(black[0]['search_reference'],'current_compact_dark_region')
        self.assertGreaterEqual(black[0]['edge_fraction'],.75)
        self.assertGreaterEqual(black[0]['color_fractions']['black'],.65)

    def test_touch_loss_validates_current_edges_inside_fresh_search_box(self):
        with ZipFile(ROOT/'evidence/20261007-004517-208447.zip') as z:
            cfg=candidate(json.loads(z.read('material_color_thresholds.json')))
            frame=cv2.imdecode(np.frombuffer(z.read('first-black-loss-raw.png'),np.uint8),1)
            current=json.loads(z.read('first-black-loss.json'))['current']
            rows=[json.loads(s) for s in z.read('observations.jsonl').decode().splitlines()]
        previous=rows[current['frame_index']-2]
        hints=[t for t in previous['internal_tracks_not_actionable'] if 0<=current['timestamp']-t['read_time']<=.5]
        result,_=analyze(frame,cfg,hints,draw_overlay=False)
        black=[r for r in result['regions'] if r['label']=='black']
        self.assertEqual(len(black),1)
        self.assertEqual(black[0]['search_reference'],'fresh_black_bbox')
        self.assertGreaterEqual(black[0]['edge_fraction'],.75)

    def test_empty_current_frame_does_not_reuse_old_black_position(self):
        with ZipFile(ROOT/'evidence/20261007-004517-208447.zip') as z:
            cfg=candidate(json.loads(z.read('material_color_thresholds.json')))
        blank=np.full((960,1280,3),(160,120,160),np.uint8)
        hints=[{'color':'black','bbox_px':[521,274,262,319],'center_px':[648,437],
                'proposal_source':'black_edge_ellipse','radius_px':None}]
        result,_=analyze(blank,cfg,hints,draw_overlay=False)
        self.assertFalse(any(r['label']=='black' for r in result['regions']))

    def test_known_colour_material_is_not_given_an_extra_black(self):
        with ZipFile(ROOT/'evidence/20261007-004517-208447.zip') as z:
            cfg=candidate(json.loads(z.read('material_color_thresholds.json')))
        for name in ('red-e50-g30.png','yellow-e50-g30.png','blue-day-e50-g30.png','green-day-e50-g30.png','lightblue-e50-g30.png'):
            path=ROOT/'evidence/six_color_crosscheck_2026-09-24/inputs'/name
            frame=cv2.imdecode(np.frombuffer(path.read_bytes(),np.uint8),1)
            result,_=analyze(frame,cfg,draw_overlay=False)
            self.assertFalse(any(r['label']=='black' for r in result['regions']),name)

    def test_two_supported_dark_objects_are_not_selected_as_one_by_score(self):
        with ZipFile(ROOT/'evidence/20261007-004517-208447.zip') as z:
            cfg=candidate(json.loads(z.read('material_color_thresholds.json')))
            image=cv2.imdecode(np.frombuffer(z.read('first-frame-raw.png'),np.uint8),1)
        result,_=analyze(np.hstack((image,image)),cfg,draw_overlay=False)
        self.assertFalse(any(r['label']=='black' for r in result['regions']))
        self.assertTrue(any(r.get('distinct_ellipse_count',0)>1 for r in result['circle_diagnostics']))

    def test_fully_clipped_white_does_not_become_dark_material(self):
        with ZipFile(ROOT/'evidence/20261007-004517-208447.zip') as z:
            cfg=candidate(json.loads(z.read('material_color_thresholds.json')))
        result,_=analyze(np.full((960,1280,3),255,np.uint8),cfg,draw_overlay=False)
        self.assertFalse(any(r['label']=='black' for r in result['regions']))

if __name__=='__main__':unittest.main()
