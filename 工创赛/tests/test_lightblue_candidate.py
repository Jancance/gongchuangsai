"""Measured pale-blue correction with deep-blue and untouched-config checks."""
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
sys.path.insert(0,str(ROOT/'scripts'));sys.path.insert(0,str(ROOT/'vision_app/code'))
from make_lightblue_color_candidate import make_candidate
detector=importlib.import_module('project_detector')


class LightblueCandidateTests(unittest.TestCase):
    def test_only_two_saturation_limits_and_candidate_metadata_change(self):
        with ZipFile(ROOT/'evidence/20261006-092130-937051.zip') as archive:
            old=json.loads(archive.read('material_color_thresholds.json'))
        new=make_candidate(old);restored=copy.deepcopy(new)
        self.assertFalse(new['calibrated_for_this_camera'])
        restored.pop('_candidate_update')
        next(c for c in restored['colors'] if c['color_id']==3)['ranges'][0]['lower'][1]=140
        next(c for c in restored['colors'] if c['color_id']==6)['ranges'][0]['upper'][1]=139
        self.assertEqual(restored,old)
        self.assertEqual(next(c for c in old['colors'] if c['color_id']==3)['ranges'][0]['lower'][1],140)

    def test_actual_lightblue_static_images_are_single_lightblue(self):
        with ZipFile(ROOT/'evidence/20261006-092130-937051.zip') as archive:
            cfg=make_candidate(json.loads(archive.read('material_color_thresholds.json')))
            for name in archive.namelist():
                if not name.endswith('-raw.png'):continue
                frame=cv2.imdecode(np.frombuffer(archive.read(name),np.uint8),1)
                result,_=detector.analyze(frame,cfg)
                self.assertEqual([r['label'] for r in result['regions'] if r['label']!='UNKNOWN'],['lightblue'],name)

    def test_known_deepblue_glare_frame_keeps_strict_blue_support(self):
        with ZipFile(ROOT/'evidence/20261006-091047-121270.zip') as archive:
            cfg=make_candidate(json.loads(archive.read('material_color_thresholds.json')))
            frame=cv2.imdecode(np.frombuffer(archive.read('longest-blue-loss-raw.png'),np.uint8),1)
            rows=[json.loads(s) for s in archive.read('observations.jsonl').decode().splitlines()]
            now=json.loads(archive.read('longest-blue-loss.json'))['current']
        past={}
        for row in rows:
            if row['frame_index']>=now['frame_index']:break
            for t in row['targets']:past[t['color']]=t
        hints=[t for t in past.values() if 0<=now['timestamp']-t['read_time']<=.5]
        result,_=detector.analyze(frame,cfg,hints)
        blue=[r for r in result['regions'] if r['label']=='blue']
        self.assertEqual(len(blue),1)
        self.assertGreaterEqual(blue[0]['color_fractions']['blue'],.65)
        self.assertGreaterEqual(blue[0]['arc_fraction'],.75)


if __name__=='__main__':unittest.main()
