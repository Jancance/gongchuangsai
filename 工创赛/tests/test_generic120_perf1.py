"""性能修订保持检测结果，保留负样本拒绝并复核最终时效。"""
import copy
import importlib.util
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
from zipfile import ZipFile

import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'output/p2_vision_generic120_20261023/vision_app/code'
CODE=ROOT/'output/p2_vision_generic120_perf1_20261023/vision_app/code'
sys.path.insert(0,str(CODE))


def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec)
    sys.modules[name]=module
    spec.loader.exec_module(module)
    return module


old=load('perf1_old_detector',BASE/'project_detector.py')
new=load('perf1_new_detector',CODE/'project_detector.py')
old_color=load('perf1_old_color',BASE/'颜色标定.py')
new_color=load('perf1_new_color',CODE/'颜色标定.py')
follow=load('perf1_follow',CODE/'project_follow.py')


class Perf1Tests(unittest.TestCase):
    def setUp(self):
        self.cfg=json.loads((CODE/'config/material_color_thresholds.json').read_text(encoding='utf-8'))

    def compare(self,frame,hints=None,cfg=None):
        cfg=copy.deepcopy(self.cfg if cfg is None else cfg)
        original=copy.deepcopy(cfg)
        with patch.dict(sys.modules,{'颜色标定':old_color}):
            before,drawing=old.analyze(frame,cfg,hints,draw_overlay=False)
            self.assertIsNone(drawing)
        with patch.dict(sys.modules,{'颜色标定':new_color}):
            after,drawing=new.analyze(frame,cfg,hints,draw_overlay=False)
            self.assertIsNone(drawing)
        self.assertEqual(after,before)
        self.assertEqual(cfg,original)
        return after

    def archived(self,archive,name):
        with ZipFile(ROOT/'evidence'/archive) as z:
            return cv2.imdecode(np.frombuffer(z.read(name),np.uint8),1)

    def test_masks_exact_for_zero_one_and_multiple_ranges(self):
        rng=np.random.default_rng(123)
        hsv=rng.integers(0,256,(160,220,3),dtype=np.uint8)
        choices=[[],[{'lower':[0,0,0],'upper':[179,255,255]}]]
        choices += [c['ranges'] for c in self.cfg['colors']]
        for ranges in choices:
            with self.subTest(ranges=ranges):
                np.testing.assert_array_equal(old_color._mask_from_hsv(hsv,ranges),new_color._mask_from_hsv(hsv,ranges))

    def test_new_camera_two_startup_frames_exact(self):
        for name in ('startup1.png','startup2.png'):
            with self.subTest(name=name):
                frame=cv2.imdecode(np.fromfile(ROOT/'evidence/p2-camera/2026-10-23/startup-repeatability'/name,dtype=np.uint8),1)
                self.compare(frame)

    def test_historical_six_colors_exact(self):
        paths=sorted((ROOT/'evidence/six_color_crosscheck_2026-09-24/inputs').glob('*.png'))
        self.assertGreaterEqual(len(paths),6)
        for path in paths:
            with self.subTest(name=path.name):
                frame=cv2.imdecode(np.fromfile(path,dtype=np.uint8),1)
                self.compare(frame)

    def test_actual_black_recovery_and_hint_exact(self):
        archive='20261007-004517-208447.zip'
        first=self.archived(archive,'first-frame-raw.png')
        before=self.compare(first)
        self.assertTrue(any(r['label']=='black' and r['proposal_source']=='black_edge_ellipse' for r in before['regions']))
        with ZipFile(ROOT/'evidence'/archive) as z:
            current=json.loads(z.read('first-black-loss.json'))['current']
            rows=[json.loads(s) for s in z.read('observations.jsonl').decode().splitlines()]
            hints=[t for t in rows[current['frame_index']-2]['internal_tracks_not_actionable'] if 0<=current['timestamp']-t['read_time']<=.5]
        result=self.compare(self.archived(archive,'first-black-loss-raw.png'),hints)
        self.assertTrue(any(r['label']=='black' for r in result['regions']))

    def test_overlap_black_and_genuine_blue_glare_exact(self):
        archive='20261007-012619-825807.zip'
        with ZipFile(ROOT/'evidence'/archive) as z:
            current=json.loads(z.read('first-black-loss.json'))['current']
            rows=[json.loads(s) for s in z.read('observations.jsonl').decode().splitlines()]
        hints=[t for t in rows[current['frame_index']-2]['internal_tracks_not_actionable'] if 0<=current['timestamp']-t['read_time']<=.5]
        result=self.compare(self.archived(archive,'first-black-loss-raw.png'),hints)
        self.assertTrue(any(r['label']=='black' for r in result['regions']))
        for name in ('first-frame-raw.png','first-blue-loss-raw.png','first-blue-id-change-raw.png'):
            result=self.compare(self.archived('20261007-000604-409723.zip',name))
            self.assertFalse(any(r['label']=='black' for r in result['regions']))

    def test_black_two_objects_and_empty_old_hint_still_reject(self):
        image=self.archived('20261007-004517-208447.zip','first-frame-raw.png')
        result=self.compare(np.hstack((image,image)))
        self.assertFalse(any(r['label']=='black' for r in result['regions']))
        self.assertTrue(any(r.get('distinct_ellipse_count',0)>1 for r in result['circle_diagnostics']))
        hints=[{'color':'black','bbox_px':[497,261,262,322]}]
        for frame in (np.full((960,1280,3),(160,120,160),np.uint8),np.full((960,1280,3),255,np.uint8)):
            result=self.compare(frame,hints)
            self.assertFalse(any(r['label']=='black' for r in result['regions']))

    def test_disjoint_policy_on_and_off_exact(self):
        frame=self.archived('20261007-012619-825807.zip','first-black-loss-raw.png')
        for enabled in (False,True):
            cfg=copy.deepcopy(self.cfg)
            cfg['black_edge_recovery_candidate']['disjoint_colour_evidence']=enabled
            self.compare(frame,cfg=cfg)

    def test_tracker_time_crosses_deadline_clears_all_history(self):
        follower=follow.MaterialFollower(self.cfg)
        region={'label':'green','bbox_px':[100,100,120,120],'area_px2':12000.,
                'silhouette_center_px':[160.,160.],'proposal_source':'combined_silhouette'}
        now=[1.]
        original=follower.tracker.update
        def slow_update(*args):
            value=original(*args)
            now[0]=1.151
            return value
        with patch.object(follow.time,'monotonic',side_effect=lambda:now[0]), \
             patch.object(follow,'analyze',return_value=({'regions':[region]},None)), \
             patch.object(follower.tracker,'update',side_effect=slow_update):
            result=follower.detect(np.zeros((10,10,3),np.uint8),1.)
        self.assertEqual(result['state'],'STALE_RESULT')
        self.assertEqual(result['targets'],[])
        self.assertEqual(follower.history,{})
        self.assertEqual(follower.tracker.tracks,{})
        self.assertFalse(result['ready']);self.assertFalse(result['motion_allowed'])

    def test_fresh_result_remains_observed_and_late_analyze_is_rejected(self):
        frame=np.zeros((10,10,3),np.uint8)
        for times,state in [([1.,1.,1.01],'SEARCHING'),([1.,1.151],'STALE_RESULT')]:
            follower=follow.MaterialFollower(self.cfg)
            with patch.object(follow.time,'monotonic',side_effect=times), \
                 patch.object(follow,'analyze',return_value=({'regions':[]},None)):
                result=follower.detect(frame,1.)
            self.assertEqual(result['state'],state)
            self.assertFalse(result['motion_allowed'])


if __name__=='__main__':unittest.main()
