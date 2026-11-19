"""Released CYQM callback, actual images and protocol/freshness integration."""
from pathlib import Path
import json
import sys
import time
import tempfile
import unittest
from unittest.mock import patch
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
P=ROOT/'output/stage3_row_bridge_20261118'
BASE=ROOT/'output/stage23_desktop_20261116'
sys.path[:0]=[str(P),str(BASE),str(BASE/'stage3_candidate')]
import region_desktop as ui
from ring_row_alignment import RingRowPipeline
from stage3 import read_image
from protocol import frame,decode
E=ROOT/'evidence/integration/2026-11-18/stage3-resume'

class BridgeTests(unittest.TestCase):
    def test_restart_sequence_uses_crc_and_exact_run_pose(self):
        with tempfile.TemporaryDirectory() as directory:
            for i,(run,pose,seq) in enumerate(((2,16,6500),(2,18,9000),(2,16,20))):
                folder=Path(directory)/f'run-{i}';folder.mkdir()
                wire=frame(['O31',run,pose,seq,0,0,0,0,0,0,100]).decode().strip()
                (folder/'observations.jsonl').write_text(json.dumps({'observation_wire':wire})+'\n')
            self.assertEqual(ui.resume_wire_sequence(directory,2,16,0),6500)
            self.assertEqual(ui.resume_wire_sequence(directory,2,16,7000),7000)
            self.assertEqual(ui.resume_wire_sequence(directory,4,16,0),0)

    def make(self):
        return ui.RegionDesktop(P/'stage3-row-config.json')

    def receive(self,u,pose=16,state=2,stamp=10):
        self.assertTrue(u.receive(frame(['S31',2,state,pose,0,2,1,5,0,0]),stamp))

    def test_original_detector_and_physical_flag(self):
        p=RingRowPipeline(P/'stage3-row-config.json',(1920,1080))
        self.assertTrue(p.alignment_ready());self.assertFalse(p.verified())
        self.assertEqual(p.detector.config.stable_frame_count,3)
        self.assertEqual(p.detector.config.minimum_confidence,.9)
        self.assertEqual(p.detector.config.maximum_center_jitter_px,1.5)

    def test_real_final_pair_and_no_fabricated_stability(self):
        p=RingRowPipeline(P/'stage3-row-config.json',(1920,1080))
        images=[E/'repark-final-before-20261118-071414-426627.png',E/'repark-final-after-20261118-071414-426627.png']
        for path in images:
            _,result=p.detect(read_image(path));self.assertTrue(result.detected);self.assertFalse(result.stable)
        motion,errors=p.measured_motion(result)
        self.assertLess(errors[0],2);self.assertLess(abs(errors[1]),1)
        self.assertFalse(p.profile['calibration_verified'])
        p.detector.reset()
        _,lost=p.detect(np.zeros((1080,1920,3),np.uint8))
        self.assertFalse(lost.detected);self.assertFalse(lost.stable)

    def test_command_matches_matlab_yaw_then_planar(self):
        p=RingRowPipeline(P/'stage3-row-config.json',(1920,1080))
        from types import SimpleNamespace
        J=p.response
        r=SimpleNamespace(centers_px=((500,540),(970,520),(1400,540)),station_yaw_deg=3)
        m,e=p.measured_motion(r)
        self.assertEqual(m[:2],(0.,0.));self.assertAlmostEqual(m[2],-3/J[2,2])
        r.station_yaw_deg=.3;m,e=p.measured_motion(r)
        self.assertEqual(m[2],0);np.testing.assert_allclose(J[:2,:2]@m[:2],[-10,20])

    def test_skipped_sequential_frames_duplicate_and_generation(self):
        u=self.make()
        u.capture_boundary(dict(generation=1,sequence=10,completed=10.01));self.assertIsNone(u.acquired_after)
        u.capture_boundary(dict(generation=1,sequence=14,completed=10.04));self.assertEqual(u.acquired_after,10.01)
        u.capture_boundary(dict(generation=1,sequence=14,completed=10.04));self.assertIsNone(u.acquired_after)
        u.capture_boundary(dict(generation=2,sequence=15,completed=10.08));self.assertIsNone(u.acquired_after)

    def test_actual_callback_old_epoch_expiry_and_three_frames(self):
        u=self.make();self.receive(u)
        image=read_image(E/'repark-final-after-20261118-071414-426627.png')
        # Repeated image below is software callback control-flow only, never a
        # claim of independent physical frames/stability.
        def step(seq,stamp,now):
            meta=dict(generation=1,sequence=seq,completed=stamp)
            u.capture_boundary(meta)
            with patch.object(ui.time,'monotonic',return_value=now):return u.process(image,meta,False)[0]
        self.assertFalse(step(1,9.99,10.01)['metric_candidate_valid'])
        self.assertFalse(step(3,10.02,10.03)['metric_candidate_valid'])
        self.assertFalse(step(5,10.04,10.05)['metric_candidate_valid'])
        self.assertFalse(step(7,10.06,10.07)['metric_candidate_valid'])
        result=step(9,10.08,10.09)
        self.assertTrue(result['metric_candidate_valid']);self.assertEqual(decode(result['_wire'])[4],'1')
        self.assertFalse(result['calibration_verified']);self.assertTrue(result['alignment_response_ready'])
        self.assertFalse(step(9,10.08,10.10)['metric_candidate_valid'])
        self.assertFalse(step(11,10.25,10.26)['metric_candidate_valid'])
        self.receive(u,pose=17,stamp=11)
        self.assertFalse(step(13,11.02,11.03)['metric_candidate_valid'])
        self.receive(u,pose=18,state=9,stamp=12)
        result=step(15,12.02,12.03)
        self.assertFalse(result['metric_candidate_valid']);self.assertEqual(decode(result['_wire'])[4],'0')

if __name__=='__main__':unittest.main()
