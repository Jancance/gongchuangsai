"""Exact outputs, unchanged guards and standalone fallback for HSV reuse."""
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
from zipfile import ZipFile
import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT/'output/p2_vision_generic120_black1_20261024/vision_app'
APP = ROOT/'output/p2_vision_generic120_perf2_20261024/vision_app'
sys.path.insert(0, str(APP/'code'))


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


old = load('perf2_before', BASE/'code/project_detector.py')
new = load('perf2_after', APP/'code/project_detector.py')


class Perf2Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg = json.loads((APP/'code/config/material_color_thresholds.json').read_text(encoding='utf-8'))

    def test_saved_current_and_historical_images_exact_outputs(self):
        # Real material, old misses, printed rings, empty scene and blue highlights.
        archives = ['20261024-121400-134643','20261024-120512-675071',
                    '20261024-114659-072282','20261024-112215-162612',
                    '20261024-111235-577533','20261024-104324-197600',
                    '20261024-105943-475482','20261007-000604-409723']
        seen = set()
        count = 0
        for archive in archives:
            with ZipFile(ROOT/'evidence'/ (archive + '.zip')) as z:
                for member in z.namelist():
                    if not member.endswith('-raw.png'):
                        continue
                    data = z.read(member)
                    checksum = hashlib.sha256(data).digest()
                    if checksum in seen:
                        continue
                    seen.add(checksum)
                    frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
                    for hints in (None, [{'color': 'green', 'bbox_px': [830, 400, 240, 240]},
                                         {'color': 'black', 'bbox_px': [170, 380, 390, 320]}]):
                        with self.subTest(archive=archive, member=member, hints=hints):
                            cfg = copy.deepcopy(self.cfg)
                            before, _ = old.analyze(frame, cfg, hints, draw_overlay=False)
                            after, _ = new.analyze(frame, cfg, hints, draw_overlay=False)
                            self.assertEqual(before, after)
                            self.assertEqual(cfg, self.cfg)
                    count += 1
        self.assertGreaterEqual(count, 80)
        print('PERF2 exact replay unique images:', count)

    def test_six_colors_and_unconfigured_overlay_exact(self):
        for color in [(0,0,230),(0,220,220),(220,0,0),(0,180,0),(15,15,15),(230,190,120)]:
            frame = np.full((480,640,3),240,np.uint8)
            cv2.circle(frame,(320,240),65,color,-1)
            for cfg in (self.cfg,None):
                with self.subTest(color=color, configured=cfg is not None):
                    a,overlay_a=old.analyze(frame,cfg,draw_overlay=True)
                    b,overlay_b=new.analyze(frame,cfg,draw_overlay=True)
                    self.assertEqual(a,b)
                    np.testing.assert_array_equal(overlay_a,overlay_b)

    def test_historical_six_color_and_black_reflection_exact(self):
        paths=sorted((ROOT/'evidence/six_color_crosscheck_2026-09-24/inputs').glob('*.png'))
        self.assertGreaterEqual(len(paths),6)
        frames=[cv2.imdecode(np.frombuffer(p.read_bytes(),np.uint8),cv2.IMREAD_COLOR) for p in paths]
        with ZipFile(ROOT/'evidence/20261007-004517-208447.zip') as z:
            frames += [cv2.imdecode(np.frombuffer(z.read(n),np.uint8),cv2.IMREAD_COLOR)
                       for n in z.namelist() if n.endswith('-raw.png')]
        for index,frame in enumerate(frames):
            with self.subTest(index=index):
                self.assertEqual(old.analyze(frame,self.cfg,draw_overlay=False)[0],
                                 new.analyze(frame,self.cfg,draw_overlay=False)[0])

    def test_skips_one_whole_frame_conversion_and_standalone_still_works(self):
        p=ROOT/'evidence/20261024-121400-134643/first-frame-raw.png'
        frame=cv2.imdecode(np.frombuffer(p.read_bytes(),np.uint8),cv2.IMREAD_COLOR)
        original=cv2.cvtColor
        with patch.object(cv2,'cvtColor',wraps=original) as spy:
            old.analyze(frame,self.cfg,draw_overlay=False)
            before=sum(c.args[0].shape==frame.shape and c.args[1]==cv2.COLOR_BGR2HSV for c in spy.call_args_list)
        with patch.object(cv2,'cvtColor',wraps=original) as spy:
            new.analyze(frame,self.cfg,draw_overlay=False)
            after=sum(c.args[0].shape==frame.shape and c.args[1]==cv2.COLOR_BGR2HSV for c in spy.call_args_list)
        self.assertEqual(before-after,1)
        masks={k:np.zeros(frame.shape[:2],np.uint8) for k in range(1,7)}
        parent=np.array([[[700,300]],[[1150,300]],[[1150,750]],[[700,750]]],np.int32)
        hsv=original(frame,cv2.COLOR_BGR2HSV)
        self.assertEqual(new.chromatic_fallback(frame,masks,[parent],[]),
                         new.chromatic_fallback(frame,masks,[parent],[],hsv=hsv))

    def test_only_detector_changed_all_other_files_byte_equal(self):
        for p in BASE.rglob('*'):
            if not p.is_file() or '__pycache__' in p.parts or p.suffix=='.pyc':continue
            rel=p.relative_to(BASE)
            if rel.as_posix()=='code/project_detector.py':continue
            self.assertEqual(p.read_bytes(),(APP/rel).read_bytes(),str(rel))


if __name__=='__main__':unittest.main()
