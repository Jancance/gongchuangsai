"""Actual paired cold-start and shaded pale-face regressions."""
import copy
import json
from pathlib import Path
import sys
import unittest
from zipfile import ZipFile
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'vision_app/code'))
sys.path.insert(0,str(ROOT/'scripts'))
from project_detector import analyze,pale_face_candidate
from analyze_paired_color_v8 import candidate

class PairedFaceTests(unittest.TestCase):
    def get(self,folder,name):
        with ZipFile(ROOT/'evidence'/folder) as z:
            cfg=json.loads(z.read('material_color_thresholds.json'))
            frame=cv2.imdecode(np.frombuffer(z.read(name),np.uint8),1)
        return cfg,frame

    def test_pair_is_recognized_without_prior_lightblue_identity_on_both_sides(self):
        for name in ('sample-03-raw.png','sample-09-raw.png'):
            cfg,frame=self.get('20261006-193600-046816.zip',name)
            previous=copy.deepcopy(cfg)
            result,_=analyze(frame,candidate(cfg))
            known=[r for r in result['regions'] if r['label']!='UNKNOWN']
            self.assertEqual(sorted(r['label'] for r in known),['blue','lightblue'])
            pale=next(r for r in known if r['label']=='lightblue')
            self.assertEqual(pale['silhouette_center_px'][0]<640,name=='sample-03-raw.png')
            self.assertEqual(cfg,previous)

    def test_shaded_pale_faces_with_saturation_above_original_limit_are_recovered(self):
        for name in ('first-lightblue-with-blue-raw.png','first-lightblue-with-red-raw.png'):
            cfg,frame=self.get('20261006-191939-893382.zip',name)
            result,_=analyze(frame,candidate(cfg))
            self.assertIn('lightblue',[r['label'] for r in result['regions']],name)

    def test_real_deep_blue_glare_keeps_current_fresh_supported_blue(self):
        with ZipFile(ROOT/'evidence/20261006-091047-121270.zip') as z:
            cfg=json.loads(z.read('material_color_thresholds.json'))
            frame=cv2.imdecode(np.frombuffer(z.read('longest-blue-loss-raw.png'),np.uint8),1)
            current=json.loads(z.read('longest-blue-loss.json'))['current']
            rows=[json.loads(s) for s in z.read('observations.jsonl').decode().splitlines()]
        previous={}
        for row in rows:
            if row['frame_index']>=current['frame_index']:break
            for t in row['targets']:previous[t['color']]=t
        hints=[t for t in previous.values() if 0<=current['timestamp']-t['read_time']<=.5]
        result,_=analyze(frame,candidate(cfg),hints)
        self.assertEqual([r['label'] for r in result['regions'] if r['label']!='UNKNOWN'],['blue'])

    def test_white_information_loss_is_not_labeled_pale(self):
        cfg,_=self.get('20261006-193600-046816.zip','sample-03-raw.png')
        frame=np.full((960,1280,3),255,np.uint8)
        result,_=analyze(frame,candidate(cfg))
        self.assertFalse(any(r['label']!='UNKNOWN' for r in result['regions']))

    def test_two_supported_pale_faces_do_not_become_one_face(self):
        cfg,_=self.get('20261006-193600-046816.zip','sample-03-raw.png')
        hsv=np.full((250,340,3),(120,255,80),np.uint8)
        cv2.circle(hsv,(95,120),65,(110,170,200),-1)
        cv2.circle(hsv,(235,120),65,(110,170,200),-1)
        frame=cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)
        from 颜色标定 import make_all_color_masks
        masks=make_all_color_masks(frame,cfg)
        parent=np.array([[[20,40]],[[310,40]],[[310,210]],[[20,210]]],np.int32)
        self.assertIsNone(pale_face_candidate(frame,masks,parent,hsv[:,:,2],candidate(cfg)['round_face_color_candidate']))

if __name__=='__main__':unittest.main()
