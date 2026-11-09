"""135°性能候选必须保持完整检测结果，含诊断与像素中心。"""
from pathlib import Path
from zipfile import ZipFile
import hashlib
import importlib.util
import json
import sys
import unittest

import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'output/p2_vision_generic135_blue1_20261026/vision_app'
APP=ROOT/'output/p2_vision_generic135_perf1_20261026/vision_app'
sys.path.insert(0,str(APP/'code'))

def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path);mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);return mod

old=load('generic135_perf1_before',BASE/'code/project_detector.py')
new=load('generic135_perf1_after',APP/'code/project_detector.py')

class ExactPerf1Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cv2.setNumThreads(8)
        cls.cfg=json.loads((BASE/'code/config/material_color_thresholds.json').read_text(encoding='utf-8'))

    def test_sparse_contours_exact_including_holes_and_frame_edges(self):
        rng=np.random.default_rng(135)
        for index in range(48):
            mask=np.zeros((240,320),np.uint8)
            if index:
                for _ in range(index%7+1):
                    x,y=map(int,rng.integers(-30,290,2));cv2.rectangle(mask,(x,y),(x+45,y+35),255,-1)
                cv2.circle(mask,(160,120),30,0,-1)
            expected,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            actual,_=new.sparse_external_contours(mask)
            self.assertEqual(len(expected),len(actual))
            for a,b in zip(expected,actual):np.testing.assert_array_equal(a,b)

    def test_current_and_historical_full_results_exact(self):
        frames=[];seen=set()
        for folder in ('blue-pair-diagnostic','blue1-board-diagnostic'):
            for p in (ROOT/'evidence/p2-camera/2026-10-26'/folder).glob('*raw.png'):
                frames.append((folder+'/'+p.name,p.read_bytes()))
        archives=['20261024-121400-134643','20261024-120512-675071','20261024-114659-072282',
                  '20261024-112215-162612','20261024-111235-577533','20261024-104324-197600',
                  '20261024-105943-475482','20261007-000604-409723']
        for name in archives:
            with ZipFile(ROOT/'evidence'/(name+'.zip')) as z:
                frames.extend((name+'/'+n,z.read(n)) for n in z.namelist() if n.endswith('-raw.png'))
        for p in (ROOT/'evidence/six_color_crosscheck_2026-09-24/inputs').glob('*.png'):
            frames.append((p.name,p.read_bytes()))
        hints=[{'color':'blue','bbox_px':[348,511,265,170],'center_px':[480,596],
                'proposal_source':'projected_blue_component'},
               {'color':'lightblue','bbox_px':[1274,524,237,155],'center_px':[1388,604]}]
        count=0
        for name,data in frames:
            digest=hashlib.sha256(data).digest()
            if digest in seen:continue
            seen.add(digest);frame=cv2.imdecode(np.frombuffer(data,np.uint8),1)
            for prior in (None,hints):
                with self.subTest(image=name,history=prior is not None):
                    a,_=old.analyze(frame,self.cfg,prior_regions=prior,draw_overlay=False)
                    b,_=new.analyze(frame,self.cfg,prior_regions=prior,draw_overlay=False)
                    self.assertEqual(a,b)
            count+=1
        self.assertGreaterEqual(count,175)
        print('135 PERF1 exact unique images:',count)

    def test_synthetic_six_color_overlays_and_unconfigured_mode_exact(self):
        for color in ((0,0,230),(0,220,220),(220,0,0),(0,180,0),(15,15,15),(230,190,120)):
            frame=np.full((480,640,3),240,np.uint8);cv2.circle(frame,(320,240),65,color,-1)
            for cfg in (self.cfg,None):
                a,oa=old.analyze(frame,cfg,draw_overlay=True)
                b,ob=new.analyze(frame,cfg,draw_overlay=True)
                self.assertEqual(a,b);np.testing.assert_array_equal(oa,ob)

    def test_only_detector_runtime_file_changed(self):
        for p in BASE.rglob('*'):
            if p.is_file() and '__pycache__' not in p.parts and p.suffix!='.pyc':
                name=p.relative_to(BASE)
                if name.as_posix()!='code/project_detector.py':self.assertEqual(p.read_bytes(),(APP/name).read_bytes(),str(name))

if __name__=='__main__':unittest.main()
