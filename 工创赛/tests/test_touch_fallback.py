"""Replay captured contact failures and enforce conservative rejection."""
import importlib
import json
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
from zipfile import ZipFile
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'vision_app/code'))
detector=importlib.import_module('project_detector')
follower_module=importlib.import_module('project_follow')
CFG=json.loads((ROOT/'vision_app/code/config/material_color_thresholds.json').read_text(encoding='utf-8'))


class TouchFallbackTests(unittest.TestCase):
    def test_saved_contact_and_glare_frames_have_one_red_without_black_duplicate(self):
        with ZipFile(ROOT/'evidence/20261006-051539-184251.zip') as archive:
            cfg=json.loads(archive.read('material_color_thresholds.json'))
            for name in archive.namelist():
                if not name.endswith('-raw.png'):
                    continue
                frame=cv2.imdecode(np.frombuffer(archive.read(name),np.uint8),cv2.IMREAD_COLOR)
                result,_=detector.analyze(frame,cfg)
                known=[r for r in result['regions'] if r['label']!='UNKNOWN']
                self.assertEqual([r['label'] for r in known],['red'],name)
                if name=='first-red-loss-raw.png':
                    self.assertEqual(known[0]['proposal_source'],'chromatic_component')
                    self.assertTrue(any(r['reason']=='TOUCHES_BORDER' for r in result['regions']))

    def test_border_hand_alone_and_clipped_disk_are_not_recovered(self):
        hand=np.full((480,640,3),150,np.uint8)
        cv2.rectangle(hand,(0,180),(260,270),(0,0,180),-1)
        cv2.circle(hand,(260,225),45,(0,0,180),-1)
        result,_=detector.analyze(hand,CFG)
        self.assertFalse(any(r['label']!='UNKNOWN' for r in result['regions']))
        clipped=np.full_like(hand,150)
        cv2.circle(clipped,(300,240),85,(255,255,255),-1)
        result,_=detector.analyze(clipped,CFG)
        self.assertFalse(any(r['label']!='UNKNOWN' for r in result['regions']))

    def test_existing_empty_background_remains_empty(self):
        paths=[ROOT/'evidence/six_color_crosscheck_2026-09-24/inputs/empty-day-e50-g30.png',
               ROOT/'evidence/empty_v1_audit_20260925_211456/first.png']
        for path in paths:
            frame=cv2.imdecode(np.fromfile(path,np.uint8),cv2.IMREAD_COLOR)
            result,_=detector.analyze(frame,CFG)
            self.assertFalse(any(r['label']!='UNKNOWN' for r in result['regions']),str(path))

    def test_proposal_reference_switch_does_not_create_velocity_jump(self):
        follower=follower_module.MaterialFollower(CFG)
        for i,source in enumerate(['combined_silhouette','chromatic_component','chromatic_component']):
            region={'label':'red','proposal_source':source,'silhouette_center_px':[100,100+i*20],
                    'bbox_px':[80,80+i*20,40,40],'area_px2':1600}
            with patch.object(follower_module,'analyze',return_value=({'regions':[region]},None)), \
                 patch.object(follower_module.time,'monotonic',return_value=1+i*.1+.01):
                value=follower.detect(np.zeros((480,640,3),np.uint8),timestamp=1+i*.1)
            self.assertEqual(value['targets'][0]['id'],'T001')
            self.assertFalse(value['motion_allowed'])
            if i==1:
                self.assertIsNone(value['targets'][0]['velocity_px_s'])
                self.assertIsNone(value['targets'][0]['predicted_px'])
            if i==2:self.assertIsNotNone(value['targets'][0]['predicted_px'])

    def test_same_color_hand_connected_to_border_needs_current_circle_evidence(self):
        with ZipFile(ROOT/'evidence/20261006-053817-238086.zip') as archive:
            cfg=json.loads(archive.read('material_color_thresholds.json'))
            hints=json.loads(archive.read('first-red-visible.json'))['current']['targets']
            for name in archive.namelist():
                if not name.endswith('-raw.png'):continue
                frame=cv2.imdecode(np.frombuffer(archive.read(name),np.uint8),cv2.IMREAD_COLOR)
                result,_=detector.analyze(frame,cfg,hints)
                known=[r for r in result['regions'] if r['label']!='UNKNOWN']
                self.assertEqual([r['label'] for r in known],['red'],name)
                if name=='first-red-loss-raw.png':
                    self.assertEqual(known[0]['reason'],'CURRENT_COLOR_ARC_SUPPORT')
                    self.assertGreaterEqual(known[0]['arc_fraction'],.75)
                    self.assertLess(np.linalg.norm(np.array(known[0]['silhouette_center_px'])-[745,515]),20)

    def test_search_hint_cannot_invent_a_disk_in_hand_or_blank_frame(self):
        hand=np.full((480,640,3),150,np.uint8)
        cv2.rectangle(hand,(0,180),(260,270),(0,0,180),-1)
        cv2.circle(hand,(260,225),45,(0,0,180),-1)
        hint=[{'color':'red','bbox_px':[180,140,160,160]}]
        for frame in (hand,np.full_like(hand,150)):
            result,_=detector.analyze(frame,CFG,hint)
            self.assertFalse(any(r['label']!='UNKNOWN' for r in result['regions']))

    def test_expired_internal_hint_is_not_passed_into_detector(self):
        follower=follower_module.MaterialFollower(CFG)
        follower.tracker.tracks={'red':{'read_time':1.,'color':'red','id':'T001','bbox_px':[10,10,40,40]}}
        with patch.object(follower_module,'analyze',return_value=({'regions':[]},None)) as analyze, \
             patch.object(follower_module.time,'monotonic',return_value=1.61), \
             patch.object(follower.tracker,'update',return_value={'observed':[],'retired_ids':[]}):
            value=follower.detect(np.zeros((480,640,3),np.uint8),timestamp=1.6)
        self.assertEqual(analyze.call_args.kwargs['prior_regions'],[])
        self.assertEqual(value['targets'],[])
        self.assertFalse(value['motion_allowed'])

    def latest_v3_loss(self):
        with ZipFile(ROOT/'evidence/20261006-055844-166424.zip') as archive:
            cfg=json.loads(archive.read('material_color_thresholds.json'))
            metadata=json.loads(archive.read('first-red-loss.json'))
            frame=cv2.imdecode(np.frombuffer(archive.read('first-red-loss-raw.png'),np.uint8),1)
        hint=metadata['previous']['targets'][0]
        hint['radius_px']=math.sqrt(hint['area_px2']/math.pi)
        return frame,cfg,hint

    def test_previous_round_measurement_is_revalidated_when_hough_misses(self):
        frame,cfg,hint=self.latest_v3_loss()
        with patch.object(detector.cv2,'HoughCircles',return_value=None):
            result,_=detector.analyze(frame,cfg,[hint])
        known=[r for r in result['regions'] if r['label']!='UNKNOWN']
        self.assertEqual([r['label'] for r in known],['red'])
        self.assertEqual(known[0]['circle_proposal'],'revalidated_previous_circle')
        self.assertGreaterEqual(known[0]['arc_fraction'],.75)

    def test_circle_center_in_highlight_gap_uses_area_support(self):
        frame,cfg,hint=self.latest_v3_loss()
        hint['proposal_source']='combined_silhouette'  # Exercise Hough validation only.
        x,y,w,h=hint['bbox_px'];margin=int(min(w,h)*.4)
        proposed=np.array([[[(668-(x-margin))/2,(547-(y-margin))/2,111.4/2]]],np.float32)
        with patch.object(detector.cv2,'HoughCircles',return_value=proposed):
            result,_=detector.analyze(frame,cfg,[hint])
        known=[r for r in result['regions'] if r['label']!='UNKNOWN']
        self.assertEqual([r['label'] for r in known],['red'])
        self.assertGreaterEqual(known[0]['parent_fraction'],.5)
        self.assertGreaterEqual(known[0]['arc_fraction'],.75)

    def test_previous_circle_cannot_continue_in_blank_or_hand_only_image(self):
        hand=np.full((480,640,3),150,np.uint8)
        cv2.rectangle(hand,(0,180),(260,270),(0,0,180),-1)
        cv2.circle(hand,(260,225),45,(0,0,180),-1)
        hint=[{'color':'red','center_px':[260,225],'bbox_px':[215,180,91,91],
               'proposal_source':'chromatic_circle','radius_px':45}]
        with patch.object(detector.cv2,'HoughCircles',return_value=None):
            for frame in (hand,np.full_like(hand,150)):
                result,_=detector.analyze(frame,CFG,hint)
                self.assertFalse(any(r['label']!='UNKNOWN' for r in result['regions']))


if __name__=='__main__':unittest.main()
