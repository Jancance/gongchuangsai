"""Offline contracts, including existing image detector and real O31 framing."""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('stack_offline', ROOT / 'scripts/prepare_stage3_stack_vision_offline_20261119.py')
M = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = M
spec.loader.exec_module(M)


class StackOfflineTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(dir=ROOT / 'tmp')
        self.folder = Path(self.temp.name)
        self.config = json.loads((ROOT / 'config/stage3_stack_materials_offline_20261119.json').read_text())
        self.config.update(task_code='124+123+421+231', start_id=1)
        self.config['physical_image_order'] = {'axis': 'x', 'physical_ring_order': [1, 2, 3]}
        self.config['camera'].update(width=640, height=480)
        self.config['color_config'] = str(ROOT / 'vision_app/code/config/material_color_thresholds.json')
        self.path = self.folder / 'profile.json'
        self.save()

    def tearDown(self):
        self.temp.cleanup()

    def save(self):
        self.path.write_text(json.dumps(self.config))

    def pipeline(self):
        return M.StackMaterialPipeline(self.path, (640, 480))

    def regions(self, offset=0):
        return [dict(label=label, silhouette_center_px=[x + offset, 240])
                for label, x in [('red', 120), ('yellow', 320), ('green', 520)]]

    def enable_synthetic(self):
        # These values are a test fixture, never copied into shipping config.
        (self.folder / 'synthetic-evidence.txt').write_text('SYNTHETIC UNIT FIXTURE ONLY')
        self.config['material_plane'].update(response_measured_on_material_plane=True,
            target_confirmed_by_user=True, response_pixel_per_right_forward_ccw=[[2, 0, 0], [0, 3, 0], [0, 0, -1]],
            nominal_image_to_station_affine=[[.5, 0], [0, 1/3]], target_center_px=[310, 240],
            target_row_yaw_deg=0, calibration_evidence='synthetic-evidence.txt')
        self.save()

    def test_default_unready_without_task_or_material_plane(self):
        p = self.pipeline()
        self.assertFalse(p.alignment_ready())
        for _ in range(5):
            result = p.from_regions(self.regions())
        self.assertTrue(result.stable)
        with self.assertRaises(ValueError):
            p.measured_motion(result)
        self.assertFalse(p.verified())

    def test_missing_duplicate_unknown_reset_stability(self):
        p = self.pipeline()
        for _ in range(5):
            result = p.from_regions(self.regions())
        self.assertTrue(result.stable)
        for bad in [self.regions()[:2], self.regions() + [dict(label='UNKNOWN')],
                    [self.regions()[0]] * 3, []]:
            self.assertFalse(p.from_regions(bad).detected)
            self.assertFalse(p.from_regions(self.regions()).stable)

    def test_jitter_and_reset(self):
        p = self.pipeline()
        for offset in (0, 0, 0, 0, 2):
            result = p.from_regions(self.regions(offset))
        self.assertFalse(result.stable)
        p.reset()
        self.assertEqual(len(p.samples), 0)

    def test_swapped_bases_cannot_be_valid(self):
        self.enable_synthetic()
        p = self.pipeline()
        swapped = self.regions()
        swapped[0]['label'], swapped[1]['label'] = swapped[1]['label'], swapped[0]['label']
        for _ in range(5):
            result = p.from_regions(swapped)
        self.assertFalse(result.detected)
        self.assertEqual(result.detail, 'BASE_COLOR_AT_WRONG_PHYSICAL_RING')
        self.config['physical_image_order'] = None
        self.save()
        self.assertFalse(self.pipeline().alignment_ready())

    def test_material_response_and_hash_invalidation(self):
        self.enable_synthetic()
        p = self.pipeline()
        for _ in range(5):
            result = p.from_regions(self.regions())
        motion, errors = p.measured_motion(result)
        self.assertEqual(motion, (-5, 0, 0))
        self.assertEqual(errors, (5, 0))
        self.path.write_text(self.path.read_text() + '\n')
        self.assertFalse(p.alignment_ready())

    def test_existing_detector_to_session_full_chain(self):
        self.enable_synthetic()
        p = self.pipeline()
        bridge = M.OfflineStackBridge(p)
        frame = M.np.full((480, 640, 3), 255, M.np.uint8)
        for color, x in [((0, 0, 220), 120), ((0, 220, 220), 320), ((0, 180, 0), 520)]:
            M.cv2.circle(frame, (x, 240), 35, color, -1)
        status = M.PROTOCOL.frame(('S31', 1, 2, 7, 0, 1, 2, 4, 0, 1))
        for i in range(1, 6):
            at = i * .05
            bridge.accept_status(status, at)
            row = bridge.process(frame, i, at + .001, at + .005, at + .01)
            self.assertEqual(row['observation_valid'], i == 5, p.last_regions)
        self.assertEqual(M.PROTOCOL.decode(row['wire'].encode())[5:8], ['-5000', '0', '0'])
        # New observation pose clears all five-frame history.
        bridge.accept_status(M.PROTOCOL.frame(('S31', 1, 2, 8, 0, 1, 2, 4, 0, 1)), .3)
        self.assertEqual(len(p.samples), 0)
        stale = bridge.process(frame, 6, .301, .302, .6)
        self.assertFalse(stale['observation_valid'])
        # An empty scene can never supply an empty-ring fallback.
        blank = M.np.full_like(frame, 255)
        row = bridge.process(blank, 7, .601, .602, .603)
        self.assertFalse(row['detected'])
        self.assertFalse(row['observation_valid'])


if __name__ == '__main__':
    unittest.main()
