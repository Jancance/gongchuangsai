"""Optimization must preserve V8 evidence and never expose stale targets."""
import json
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch
from zipfile import ZipFile
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'vision_app/code'))
from project_detector import analyze
from project_follow import MaterialFollower

def old_detector():
    module=types.ModuleType('archived_v8')
    with ZipFile(ROOT/'output/paired_face_patch_v8_2026-10-06.zip') as z:
        exec(compile(z.read('vision_app/code/project_detector.py'),'archived_v8','exec'),module.__dict__)
    return module

class DetectorEquivalenceTests(unittest.TestCase):
    def test_all_saved_loss_and_periodic_raw_images_match_v8_with_same_past_hints(self):
        old=old_detector()
        with ZipFile(ROOT/'evidence/20261006-195642-102168.zip') as z:
            cfg=json.loads(z.read('material_color_thresholds.json'))
            rows=[json.loads(s) for s in z.read('observations.jsonl').decode().splitlines()]
            for name in z.namelist():
                if not name.endswith('-raw.png'):continue
                meta=name.replace('-before-raw.png','.json').replace('-raw.png','.json')
                packet=json.loads(z.read(meta));current=packet.get('previous' if '-before-' in name else 'current')
                hints=[]
                if current and current['frame_index']>1:
                    previous=rows[current['frame_index']-2]
                    hints=[t for t in previous.get('internal_tracks_not_actionable',[])
                           if 0<=current['timestamp']-t['read_time']<=.5]
                frame=cv2.imdecode(np.frombuffer(z.read(name),np.uint8),1)
                expected,_=old.analyze(frame,cfg,hints)
                actual,overlay=analyze(frame,cfg,hints,draw_overlay=False)
                expected.pop('version');actual.pop('version')
                self.assertEqual(actual,expected,name)
                self.assertIsNone(overlay)

    def test_default_overlay_and_unconfigured_probe_keep_results_without_mutating_input(self):
        old=old_detector()
        path=ROOT/'evidence/six_color_crosscheck_2026-09-24/inputs/blue-day-e50-g30.png'
        frame=cv2.imdecode(np.frombuffer(path.read_bytes(),np.uint8),1);original=frame.copy()
        expected,_=old.analyze(frame)
        actual,overlay=analyze(frame)
        expected.pop('version');actual.pop('version')
        self.assertEqual(actual,expected)
        self.assertEqual(overlay.shape,frame.shape)
        np.testing.assert_array_equal(frame,original)

    def test_follower_still_clears_targets_if_optimized_detection_finishes_late(self):
        import project_follow
        follower=MaterialFollower()
        region={'label':'blue','silhouette_center_px':[50.,50.],
                'bbox_px':[20,20,60,60],'area_px2':3000.}
        with patch.object(project_follow.time,'monotonic',side_effect=[1.01,1.151]), \
             patch.object(project_follow,'analyze',return_value=({'regions':[region]},None)) as detector:
            result=follower.detect(np.zeros((100,100,3),np.uint8),timestamp=1.)
        self.assertEqual(result['state'],'STALE_RESULT')
        self.assertEqual(result['targets'],[])
        self.assertFalse(result['motion_allowed'])
        self.assertFalse(detector.call_args.kwargs['draw_overlay'])

if __name__=='__main__':unittest.main()
