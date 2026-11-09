"""Black-face recovery must retain negative, ambiguity, and freshness guards."""
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

ROOT = Path(__file__).resolve().parents[1]
NEW = ROOT/'output/p2_vision_generic120_black1_20261024/vision_app/code'
OLD = ROOT/'output/p2_vision_generic120_ring1_20261024/vision_app/code'
sys.path.insert(0,str(NEW))


def load(name, path):
    spec = importlib.util.spec_from_file_location(name,path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


old = load('black1_old',OLD/'project_detector.py')
new = load('black1_new',NEW/'project_detector.py')


class Black1Tests(unittest.TestCase):
    def setUp(self):
        self.cfg = json.loads((NEW/'config/material_color_thresholds.json').read_text(encoding='utf-8'))

    def frame(self, archive, name):
        with ZipFile(ROOT/'evidence'/archive) as z:
            return cv2.imdecode(np.frombuffer(z.read(name),np.uint8),1)

    def detect(self, frame, mod=new, hints=None, config=None):
        cfg=copy.deepcopy(self.cfg if config is None else config)
        before=copy.deepcopy(cfg)
        result,_=mod.analyze(frame,cfg,hints,draw_overlay=False)
        self.assertEqual(before,cfg)
        return result

    def test_saved_static_and_dynamic_misses_recover_without_history(self):
        for archive,name in [('20261024-112215-162612.zip','sample-02-before-raw.png'),
                             ('20261024-111235-577533.zip','sample-02-raw.png')]:
            with self.subTest(archive=archive):
                im=self.frame(archive,name)
                before=self.detect(im,old);after=self.detect(im)
                self.assertFalse(any(r['label']=='black' for r in before['regions']))
                black=[r for r in after['regions'] if r['label']=='black']
                self.assertEqual(len(black),1)
                self.assertEqual(black[0]['reason'],'CURRENT_SOLID_DARK_FACE_SUPPORT')
                self.assertGreaterEqual(black[0]['face_evidence']['black_fraction'],.65)
                self.assertGreaterEqual(black[0]['face_evidence']['edge_fraction'],.75)
                self.assertEqual([r for r in before['regions'] if r['label']=='green'],
                                 [r for r in after['regions'] if r['label']=='green'])

    def test_all_saved_green_black_frames_keep_both(self):
        count=0
        for archive in ['20261024-112215-162612.zip','20261024-111235-577533.zip']:
            seen=set()
            with ZipFile(ROOT/'evidence'/archive) as z:
                for name in z.namelist():
                    if not name.endswith('-raw.png'):continue
                    data=z.read(name);h=hashlib.sha256(data).hexdigest()
                    if h in seen:continue
                    seen.add(h);count+=1
                    with self.subTest(archive=archive,name=name):
                        labels=[r['label'] for r in self.detect(cv2.imdecode(np.frombuffer(data,np.uint8),1))['regions']]
                        self.assertEqual(labels.count('black'),1)
                        self.assertEqual(labels.count('green'),1)
        self.assertGreaterEqual(count,30)

    def test_printed_rings_remain_negative(self):
        count=0
        for archive in ['20261024-104324-197600.zip','20261024-105943-475482.zip']:
            seen=set()
            with ZipFile(ROOT/'evidence'/archive) as z:
                for name in z.namelist():
                    if not name.endswith('-raw.png'):continue
                    data=z.read(name);h=hashlib.sha256(data).hexdigest()
                    if h in seen:continue
                    seen.add(h);count+=1
                    with self.subTest(archive=archive,name=name):
                        result=self.detect(cv2.imdecode(np.frombuffer(data,np.uint8),1))
                        self.assertFalse(any(r['label']=='black' for r in result['regions']))
        self.assertGreaterEqual(count,40)

    def test_switch_off_is_exact_ring1(self):
        cfg=copy.deepcopy(self.cfg);cfg['black_face_support_candidate']['enabled']=False
        frame=self.frame('20261024-112215-162612.zip','sample-02-before-raw.png')
        self.assertEqual(self.detect(frame,new,config=cfg),self.detect(frame,old,config=cfg))

    def test_recovered_hull_keeps_current_track_without_copying_old_coordinates(self):
        from project_tracker import TabletopTracker
        p=ROOT/'evidence/20261024-112215-162612'
        rows=[json.loads(s) for s in (p/'observations.jsonl').read_text(encoding='utf-8').splitlines()]
        tracker=TabletopTracker()
        before=tracker.update(rows[60]['detector_regions'],rows[60]['timestamp'])
        im=self.frame('20261024-112215-162612.zip','sample-02-before-raw.png')
        result=self.detect(im)
        after=tracker.update(result['regions'],rows[61]['timestamp'])
        first=next(t for t in before['observed'] if t['color']=='black')
        current=next(t for t in after['observed'] if t['color']=='black')
        self.assertEqual(current['id'],first['id'])
        self.assertTrue(current['geometry_reference_changed'])
        self.assertFalse(current['identity_continuity_unverified'])
        self.assertNotEqual(current['center_px'],first['center_px'])
        self.assertFalse(result.get('motion_allowed',False))

    def test_six_color_and_existing_black_recovery_unchanged(self):
        paths=sorted((ROOT/'evidence/six_color_crosscheck_2026-09-24/inputs').glob('*.png'))
        self.assertGreaterEqual(len(paths),6)
        for p in paths:
            with self.subTest(name=p.name):
                im=cv2.imdecode(np.frombuffer(p.read_bytes(),np.uint8),1)
                self.assertEqual(self.detect(im),self.detect(im,old))
        for name in ['first-frame-raw.png','first-black-loss-raw.png']:
            im=self.frame('20261007-004517-208447.zip',name)
            # Existing recovery can depend on current search hints.
            with ZipFile(ROOT/'evidence/20261007-004517-208447.zip') as z:
                rows=[json.loads(s) for s in z.read('observations.jsonl').decode().splitlines()]
                cur=json.loads(z.read('first-black-loss.json'))['current']
            hints=rows[cur['frame_index']-2]['internal_tracks_not_actionable'] if 'loss' in name else None
            self.assertEqual(self.detect(im,hints=hints),self.detect(im,old,hints=hints))

    def test_no_stale_hint_hallucination_or_blue_false_black(self):
        blank=np.full((960,1280,3),(160,120,160),np.uint8)
        self.assertFalse(any(r['label']=='black' for r in self.detect(blank,hints=[
            {'color':'black','bbox_px':[169,395,382,307]}])['regions']))
        for name in ['first-frame-raw.png','first-blue-loss-raw.png','first-blue-id-change-raw.png']:
            self.assertFalse(any(r['label']=='black' for r in self.detect(
                self.frame('20261007-000604-409723.zip',name))['regions']))

    def test_ambiguous_faces_border_and_low_dark_evidence_reject(self):
        image=np.full((500,700,3),255,np.uint8)
        parents=[]
        for cx in [200,500]:
            cv2.circle(image,(cx,250),60,(10,10,10),-1)
            parents.append(np.array([[[cx-90,160]],[[cx+90,160]],[[cx+90,340]],[[cx-90,340]]],np.int32))
        hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        masks={k:np.zeros((500,700),np.uint8) for k in range(1,7)}
        masks[5]=(hsv[:,:,2]<45).astype(np.uint8)*255
        diag=[]
        self.assertEqual(new.black_face_supported_hull(image,masks,parents,[],hsv[:,:,2],
                         {'enabled':True},diag),[])
        self.assertEqual(diag[-1]['supported_face_count'],2)
        masks[5][:]=0
        self.assertEqual(new.black_face_supported_hull(image,masks,parents,[],hsv[:,:,2],
                         {'enabled':True},[]),[])
        border=np.array([[[0,160]],[[280,160]],[[280,340]],[[0,340]]],np.int32)
        self.assertEqual(new.black_face_supported_hull(image,masks,[border],[],hsv[:,:,2],
                         {'enabled':True},[]),[])

    def test_rotation_and_ambiguous_scaled_view(self):
        im=self.frame('20261024-112215-162612.zip','sample-02-before-raw.png')
        rotate=cv2.getRotationMatrix2D((960,540),8,1)
        rotated=cv2.warpAffine(im,rotate,(1920,1080),borderValue=(255,255,255))
        self.assertEqual(sum(r['label']=='black' for r in self.detect(rotated)['regions']),1)
        # 缩小图中侧壁形成第二个有效暗圆；不能为追求增强集检出率强猜。
        scaled=self.detect(cv2.resize(im,None,fx=.8,fy=.8))
        self.assertEqual(sum(r['label']=='black' for r in scaled['regions']),0)
        self.assertTrue(any(r.get('supported_face_count',0)>1 for r in scaled['circle_diagnostics']))
        self.assertEqual(sum(r['label']=='green' for r in scaled['regions']),1)


if __name__=='__main__':unittest.main()
