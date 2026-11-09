"""Printed-ring negative samples must not remove real black or six-color features."""
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import unittest
from zipfile import ZipFile

import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
CODE=ROOT/'output/p2_vision_generic120_ring1_20261024/vision_app/code'
BASE=ROOT/'output/p2_vision_generic120_perf1_20261023/vision_app/code'
sys.path.insert(0,str(CODE))


def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec);sys.modules[name]=module;spec.loader.exec_module(module)
    return module


old=load('ring1_before',BASE/'project_detector.py')
new=load('ring1_after',CODE/'project_detector.py')


class Ring1Tests(unittest.TestCase):
    def setUp(self):
        self.cfg=json.loads((CODE/'config/material_color_thresholds.json').read_text(encoding='utf-8'))

    def frame(self,archive,name):
        with ZipFile(ROOT/'evidence'/archive) as z:
            return cv2.imdecode(np.frombuffer(z.read(name),np.uint8),1)

    def detect(self,frame,detector=new,cfg=None,hints=None):
        original=copy.deepcopy(self.cfg if cfg is None else cfg)
        result,_=detector.analyze(frame,original,hints,draw_overlay=False)
        self.assertEqual(original,self.cfg if cfg is None else cfg)
        return result

    def test_reported_false_positive_is_rejected_with_reason_green_unchanged(self):
        frame=self.frame('20261024-104324-197600.zip','first-green-with-black-raw.png')
        before=self.detect(frame,old);after=self.detect(frame)
        self.assertEqual([r['label'] for r in before['regions']],['black','green','UNKNOWN'])
        self.assertFalse(any(r['label']=='black' for r in after['regions']))
        self.assertEqual(after['regions'][0]['reason'],'PRINTED_CONCENTRIC_RING_PATTERN')
        self.assertGreaterEqual(after['regions'][0]['printed_ring_evidence']['dark_band_count'],3)
        self.assertEqual(before['regions'][1],after['regions'][1])

    def test_all_unique_saved_raw_frames_keep_green_without_black(self):
        seen=set();tested=0
        with ZipFile(ROOT/'evidence/20261024-104324-197600.zip') as z:
            for name in z.namelist():
                if not name.endswith('-raw.png'):continue
                raw=z.read(name);digest=hashlib.sha256(raw).hexdigest()
                if digest in seen:continue
                seen.add(digest);tested+=1
                with self.subTest(name=name):
                    result=self.detect(cv2.imdecode(np.frombuffer(raw,np.uint8),1))
                    self.assertFalse(any(r['label']=='black' for r in result['regions']))
                    self.assertEqual(sum(r['label']=='green' for r in result['regions']),1)
        self.assertGreaterEqual(tested,12)

    def test_feature_switch_restores_exact_baseline(self):
        frame=self.frame('20261024-104324-197600.zip','first-green-with-black-raw.png')
        cfg=copy.deepcopy(self.cfg);cfg['printed_ring_rejection_candidate']['enabled']=False
        self.assertEqual(self.detect(frame,old,cfg),self.detect(frame,new,cfg))

    def test_scale_rotation_and_vertical_ellipse_are_not_fixed_roi(self):
        frame=self.frame('20261024-104324-197600.zip','first-green-with-black-raw.png')
        transform=cv2.getRotationMatrix2D((960,540),17.,1.)
        variants=[cv2.resize(frame,None,fx=.75,fy=.75),
                  cv2.resize(frame,None,fx=1.,fy=.8),
                  cv2.warpAffine(frame,transform,(1920,1080),borderValue=(255,255,255))]
        for index,image in enumerate(variants):
            with self.subTest(index=index):
                result=self.detect(image)
                self.assertFalse(any(r['label']=='black' for r in result['regions']))
                self.assertTrue(any(r['label']=='green' for r in result['regions']))

    def test_real_black_first_glare_and_touch_recovery_remain(self):
        archive='20261007-004517-208447.zip'
        with ZipFile(ROOT/'evidence'/archive) as z:
            current=json.loads(z.read('first-black-loss.json'))['current']
            rows=[json.loads(s) for s in z.read('observations.jsonl').decode().splitlines()]
        hints=[t for t in rows[current['frame_index']-2]['internal_tracks_not_actionable'] if 0<=current['timestamp']-t['read_time']<=.5]
        for name,prior in [('first-frame-raw.png',None),('first-black-loss-raw.png',hints)]:
            frame=self.frame(archive,name)
            before=self.detect(frame,old,hints=prior);after=self.detect(frame,hints=prior)
            self.assertEqual(after,before)
            self.assertTrue(any(r['label']=='black' for r in after['regions']))

    def test_six_color_history_unchanged(self):
        paths=sorted((ROOT/'evidence/six_color_crosscheck_2026-09-24/inputs').glob('*.png'))
        self.assertGreaterEqual(len(paths),6)
        for path in paths:
            with self.subTest(name=path.name):
                frame=cv2.imdecode(np.fromfile(path,np.uint8),1)
                self.assertEqual(self.detect(frame),self.detect(frame,old))

    def test_solid_black_center_on_printed_ring_is_preserved(self):
        frame=self.frame('20261024-104324-197600.zip','first-green-with-black-raw.png')
        # 合成黑色实心顶面覆盖印刷圆心，测试不能只见外露环线就否决物料。
        cv2.circle(frame,(414,542),108,(8,8,8),-1)
        result=self.detect(frame)
        self.assertTrue(any(r['label']=='black' for r in result['regions']))

    def test_blue_glare_and_empty_old_black_hint_still_reject(self):
        for name in ('first-frame-raw.png','first-blue-loss-raw.png','first-blue-id-change-raw.png'):
            result=self.detect(self.frame('20261007-000604-409723.zip',name))
            self.assertFalse(any(r['label']=='black' for r in result['regions']))
        frame=np.full((960,1280,3),(160,120,160),np.uint8)
        result=self.detect(frame,hints=[{'color':'black','bbox_px':[497,261,262,322]}])
        self.assertFalse(any(r['label']=='black' for r in result['regions']))


if __name__=='__main__':unittest.main()
