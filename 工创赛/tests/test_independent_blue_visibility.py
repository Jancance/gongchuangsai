"""Uncertain pale evidence must not remove an independently observed blue."""
import json
from pathlib import Path
import sys
import unittest
from zipfile import ZipFile
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'vision_app/code'))
from project_detector import analyze
from project_tracker import TabletopTracker

def config():
    with ZipFile(ROOT/'evidence/20261006-204150-736167.zip') as z:
        cfg=json.loads(z.read('material_color_thresholds.json'))
    cfg['round_face_color_candidate']['reject_unverified_blue_family']=True
    return cfg

class IndependentBlueTests(unittest.TestCase):
    def test_actual_dual_loss_keeps_existing_blue_id_and_does_not_invent_pale(self):
        with ZipFile(ROOT/'evidence/20261006-204150-736167.zip') as z:
            metadata=json.loads(z.read('first-lightblue-loss.json'))
            frame=cv2.imdecode(np.frombuffer(z.read('first-lightblue-loss-raw.png'),np.uint8),1)
        tracker=TabletopTracker();before=metadata['previous'];current=metadata['current']
        original=tracker.update(before['detector_regions'],before['timestamp'])
        old_blue=next(t['id'] for t in original['observed'] if t['color']=='blue')
        detection,_=analyze(frame,config(),draw_overlay=False)
        result=tracker.update(detection['regions'],current['timestamp'])
        self.assertEqual([(t['color'],t['id']) for t in result['observed']],[('blue',old_blue)])
        self.assertTrue(any(r['reason']=='PALE_COLOR_CONFLICT_WITHOUT_VALID_FACE' for r in detection['regions']))

    def test_two_genuine_blue_regions_remain_duplicate_and_are_rejected(self):
        base=ROOT/'evidence/six_color_crosscheck_2026-09-24/inputs/blue-day-e50-g30.png'
        image=cv2.imdecode(np.frombuffer(base.read_bytes(),np.uint8),1)
        frame=np.hstack((image,image))
        detection,_=analyze(frame,config(),draw_overlay=False)
        self.assertEqual(sum(r['label']=='blue' for r in detection['regions']),2)
        tracked=TabletopTracker().update(detection['regions'],1.)
        self.assertEqual(tracked['observed'],[])
        self.assertEqual(tracked['ambiguous_colors'],['blue'])

    def test_partly_outside_pale_does_not_remove_complete_blue(self):
        with ZipFile(ROOT/'evidence/20261006-201311-289701.zip') as z:
            frame=cv2.imdecode(np.frombuffer(z.read('first-frame-raw.png'),np.uint8),1)
        piece=frame[140:540,600:1000].copy()
        frame[140:540,600:1000]=(160,120,160)
        frame[140:540,1100:1280]=piece[:,:180]
        detection,_=analyze(frame,config(),draw_overlay=False)
        self.assertEqual([r['label'] for r in detection['regions'] if r['label']!='UNKNOWN'],['blue'])
        self.assertTrue(any(r['reason']=='TOUCHES_BORDER' for r in detection['regions']))

if __name__=='__main__':unittest.main()
