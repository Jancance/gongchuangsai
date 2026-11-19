import json
from pathlib import Path
import unittest
import cv2
import numpy as np
from adaptive_source_rim_20261115 import AdaptiveSourceRim

R=Path(__file__).resolve().parents[1]
E=R/'evidence/integration/2026-11-15/source-rim-reacquire'

class AdaptiveRimTests(unittest.TestCase):
    def setUp(self):
        self.profile=json.loads((R/'evidence/integration/2026-11-15/integration18/window5-pause/integration19-field/source_body_registration_field.json').read_text())
        self.observer=AdaptiveSourceRim(self.profile,E/'actual-turntable-detector.json')
        self.image=cv2.imdecode(np.fromfile(E/'current-raw.png',dtype=np.uint8),cv2.IMREAD_COLOR)

    def test_actual_new_frame_matches_independent_matlab(self):
        proof=json.loads((E/'matlab-result.json').read_text());expected=proof['results'][0]
        value=self.observer.estimate(self.image)
        np.testing.assert_allclose(value['center_px'],expected['center_matlab_one_based'],atol=1e-5)
        self.assertLess(value['fit_rms_px'],3);self.assertFalse(value['temporal_filter_used'])

    def test_local_tracking_is_raw_and_reacquires_after_scene_translation(self):
        start=self.observer.estimate(self.image);centers=[]
        for _ in range(12):centers.append(self.observer.estimate(self.image)['center_px'])
        self.assertLess(np.ptp(np.asarray(centers),axis=0).max(),3)
        for dx,dy in ((120,0),(-80,-60),(60,80)):
            moved=cv2.warpAffine(self.image,np.float32([[1,0,dx],[0,1,dy]]),(1920,1080),borderValue=(200,200,200))
            value=self.observer.estimate(moved)
            fresh=AdaptiveSourceRim(self.profile,E/'actual-turntable-detector.json').estimate(moved)
            # Cropping changes the visible arc: validate immediate global
            # reacquisition, and use the existing5mm positioning budget.
            np.testing.assert_allclose(value['center_px'],fresh['center_px'],atol=1e-5)
            error=np.asarray(value['center_px'])-start['center_px']-[dx,dy]
            self.assertLess(np.linalg.norm(error)*self.profile['scale_mm_per_px'],self.profile['position_tolerance_mm'])
            self.assertLess(value['fit_rms_px'],3)

    def test_missing_rim_never_reuses_old_circle(self):
        self.observer.estimate(self.image)
        with self.assertRaises(ValueError):self.observer.estimate(np.full_like(self.image,200))
        self.assertIsNone(self.observer.detector.seed)
        value=self.observer.estimate(self.image);self.assertLess(value['fit_rms_px'],3)

if __name__=='__main__':unittest.main()
