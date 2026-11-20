import json
from argparse import Namespace
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
from types import SimpleNamespace
import math
import numpy as np
import cv2

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from stage3 import estimate_affine, fit_response, angle_error_in_plane, fit_calibration, read_json, write_image


class CalibrationTest(unittest.TestCase):
    def test_affine_60_maps_and_yaw(self):
        rng = np.random.default_rng(31)
        for _ in range(60):
            t = rng.uniform(-.3, .3)
            rotation = np.array(((math.cos(t), -math.sin(t)), (math.sin(t), math.cos(t))))
            projection = rotation @ np.array(((rng.uniform(1, 4), rng.uniform(-.5, .5)), (0, rng.uniform(.5, 2))))
            shape = projection @ projection.T * 47.5**2
            values, vectors = np.linalg.eigh(shape)
            ellipse = ((0., 0.), tuple(2*np.sqrt(values)), math.degrees(math.atan2(vectors[1,0], vectors[0,0])))
            centers = np.array((-150*projection[:,0], (0,0), 150*projection[:,0]))
            affine, residual = estimate_affine(centers, ellipse, 150., 95.)
            np.testing.assert_allclose(affine @ projection, np.eye(2), atol=1e-12)
            self.assertLess(residual, 1e-12)
            self.assertAlmostEqual(angle_error_in_plane(affine, 179., -1.), 0.)

    def test_response_cross_coupling_validation_and_degeneracy(self):
        response = np.array(((-1., .2, .1), (.3, -1.4, .4), (.01, -.02, -.8)))
        commands = np.array(((10,0,0), (0,10,0), (0,0,2), (-10,0,0), (0,-10,0), (0,0,-2)))
        changes = commands @ response.T
        fitted, correction = fit_response(commands, changes)
        np.testing.assert_allclose(fitted, response, atol=1e-12)
        error = np.array((3., -4., .7))
        np.testing.assert_allclose(error + response @ correction @ error, 0, atol=1e-12)
        noisy = changes[0] + np.array((1.2, -.8, .3))
        self.assertGreater(np.linalg.norm((noisy-fitted @ commands[0])[:2]), 1.)
        with self.assertRaises(ValueError): fit_response([[1,0,0]], [[1,0,0]])
        with self.assertRaises(ValueError): fit_response(np.eye(3), np.zeros((3,3)))
        with self.assertRaises(ValueError): estimate_affine(np.zeros((3,2)), ((0,0),(0,0),0),150.,95.)

    def test_fit_file_pipeline_report_is_unverified(self):
        # Controlled detections isolate orchestration, units, report and no config changes.
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory)
            cfg = base/'config.json'
            cfg.write_bytes((ROOT/'config/stage3.json').read_bytes())
            original = cfg.read_bytes()
            for name in ('reference.png','right.png','forward.png','yaw.png','validation.png'):
                write_image(base/name, np.full((480,640,3),255,np.uint8))
            pairs = [{'before':'reference.png','after':name,'command_right_forward_ccw':command}
                     for name,command in [('right.png',[10,0,0]),('forward.png',[0,10,0]),('yaw.png',[0,0,2])]]
            pairs.append({'before':'reference.png','after':'validation.png',
                          'command_right_forward_ccw':[10,0,0], 'validation_only':True})
            source = base/'pairs.json'
            source.write_text(json.dumps({'reference_image':'reference.png','pairs':pairs}))
            triple = SimpleNamespace(detected=True,centers_px=((20,240),(320,240),(620,240)))
            outputs = iter([{'middle':(320,240),'yaw':0}, {'middle':(300,240),'yaw':0},
                            {'middle':(320,220),'yaw':0}, {'middle':(320,240),'yaw':-2},
                            {'middle':(302,240),'yaw':.3}])
            with patch('stage3.RingStationDetector.detect',return_value=triple), patch('stage3.find_calibration_outer_ring',return_value=(((320,240),(190,190),0),1.)), patch('stage3.detect_scaled_reference_lines',side_effect=lambda *a,**k:next(outputs)):
                fit_calibration(Namespace(measurements=source, config=cfg, output=base/'fit'))
            report = read_json(base/'fit/measured.json')
            self.assertFalse(report['calibration_verified'])
            self.assertTrue(report['independent_validation_present'])
            self.assertAlmostEqual(report['independent_validation_residuals'][0]['plane_mm'],1.)
            self.assertAlmostEqual(report['independent_validation_residuals'][0]['yaw_deg'],.3)
            self.assertEqual(original,cfg.read_bytes())
            np.testing.assert_allclose(report['chassis_image_to_motion'], np.eye(3), atol=1e-12)


if __name__ == '__main__': unittest.main()
