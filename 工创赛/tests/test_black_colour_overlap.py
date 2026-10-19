"""Overlapping dark pixels are not independent conflicting colour evidence."""
import json
from pathlib import Path
import sys
import types
import unittest
from zipfile import ZipFile
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'vision_app/code'));sys.path.insert(0,str(ROOT/'scripts'))
from project_detector import analyze,black_edge_recovery
from analyze_black_overlap_v13 import candidate

class BlackOverlapTests(unittest.TestCase):
    def test_actual_touch_has_valid_edges_but_overlapping_blue_votes_caused_rejection(self):
        with ZipFile(ROOT/'evidence/20261007-012619-825807.zip') as z:
            cfg=json.loads(z.read('material_color_thresholds.json'))
            frame=cv2.imdecode(np.frombuffer(z.read('first-black-loss-raw.png'),np.uint8),1)
            current=json.loads(z.read('first-black-loss.json'))['current']
            rows=[json.loads(s) for s in z.read('observations.jsonl').decode().splitlines()]
        with ZipFile(ROOT/'output/black_edge_patch_v12_2026-10-07.zip') as z:
            old=types.ModuleType('v12');exec(z.read('vision_app/code/project_detector.py'),old.__dict__)
        hints=[h for h in rows[current['frame_index']-2]['internal_tracks_not_actionable'] if 0<=current['timestamp']-h['read_time']<=.5]
        before,_=old.analyze(frame,cfg,hints,draw_overlay=False)
        self.assertFalse(any(r['label']=='black' for r in before['regions']))
        after,_=analyze(frame,candidate(cfg),hints,draw_overlay=False)
        black=[r for r in after['regions'] if r['label']=='black']
        self.assertEqual(len(black),1)
        self.assertGreater(black[0]['color_fractions']['blue'],.10)
        self.assertLessEqual(black[0]['chromatic_nonblack_fractions']['blue'],.10)
        self.assertGreaterEqual(black[0]['edge_fraction'],.75)

    def test_genuine_blue_highlight_does_not_gain_black(self):
        with ZipFile(ROOT/'evidence/20261007-012619-825807.zip') as z:
            cfg=candidate(json.loads(z.read('material_color_thresholds.json')))
        with ZipFile(ROOT/'evidence/20261007-000604-409723.zip') as z:
            for name in ('first-frame-raw.png','first-blue-loss-raw.png','first-blue-id-change-raw.png'):
                frame=cv2.imdecode(np.frombuffer(z.read(name),np.uint8),1)
                result,_=analyze(frame,cfg,draw_overlay=False)
                self.assertFalse(any(r['label']=='black' for r in result['regions']),name)

    def test_no_current_object_is_not_recovered_from_old_black_hint(self):
        with ZipFile(ROOT/'evidence/20261007-012619-825807.zip') as z:
            cfg=candidate(json.loads(z.read('material_color_thresholds.json')))
        blank=np.full((960,1280,3),(160,120,160),np.uint8)
        result,_=analyze(blank,cfg,[{'color':'black','bbox_px':[497,261,262,322]}],draw_overlay=False)
        self.assertFalse(any(r['label']=='black' for r in result['regions']))

    def test_colour_evidence_outside_black_range_still_rejects_recovery(self):
        with ZipFile(ROOT/'evidence/20261007-012619-825807.zip') as z:
            cfg=json.loads(z.read('material_color_thresholds.json'))
            frame=cv2.imdecode(np.frombuffer(z.read('first-black-loss-raw.png'),np.uint8),1)
            current=json.loads(z.read('first-black-loss.json'))['current']
            rows=[json.loads(s) for s in z.read('observations.jsonl').decode().splitlines()]
        from 颜色标定 import make_all_color_masks
        masks=make_all_color_masks(frame,cfg)
        union=np.zeros(frame.shape[:2],np.uint8)
        for mask in masks.values():union=cv2.bitwise_or(union,mask)
        union=cv2.morphologyEx(union,cv2.MORPH_CLOSE,np.ones((5,5),np.uint8))
        contours,_=cv2.findContours(union,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        rejected=[c for c in contours if cv2.contourArea(c)>=1000]
        # 模拟真正位于黑范围外的蓝证据；不能通过去重把它一起消掉。
        masks[3]=(masks[5]==0).astype(np.uint8)*255
        hints=[h for h in rows[current['frame_index']-2]['internal_tracks_not_actionable'] if h['color']=='black']
        diagnostics=[]
        result=black_edge_recovery(frame,masks,rejected,[],hints,cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)[:,:,2],
                                  diagnostics,{'disjoint_colour_evidence':True})
        self.assertEqual(result,[])
        self.assertGreater(diagnostics[-1]['colour_rejection_count'],0)

if __name__=='__main__':unittest.main()
