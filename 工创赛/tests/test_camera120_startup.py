"""120°启动控制顺序、错误拒绝；不代表板端相机验收。"""
import importlib.util
import json
from pathlib import Path
import subprocess
import unittest
from unittest.mock import MagicMock, patch

import numpy as np

CODE = Path(__file__).resolve().parents[1] / 'vision_app' / 'code'
spec = importlib.util.spec_from_file_location('camera120_startup', CODE / 'camera120_startup_check.py')
camera = importlib.util.module_from_spec(spec)
spec.loader.exec_module(camera)


class StartupTests(unittest.TestCase):
    def setUp(self):
        self.profile = json.loads(camera.PROFILE.read_text(encoding='utf-8'))
        self.controls = ('exposure_auto: 1 (Manual Mode)\nexposure_absolute: 166\n'
                         'gain: 80\nwhite_balance_temperature_auto: 0\n'
                         'white_balance_temperature: 4600\n')

    def test_new_session_adjusts_before_lock_and_never_writes_temperature(self):
        events = []
        cap = MagicMock()
        cap.isOpened.return_value = True
        with patch.object(camera, 'set_control', side_effect=lambda d, s: events.append(s)), \
             patch.object(camera, 'read_for', side_effect=lambda c, t, s: events.append(('read', t)) or 'frame'), \
             patch.object(camera, 'command', return_value=self.controls):
            for _ in range(2):
                frame, _ = camera.initialize_camera(cap, self.profile)
                self.assertEqual(frame, 'frame')
        expected = [('read', 0), 'exposure_auto=1', 'exposure_absolute=166',
                    'gain=80', 'white_balance_temperature_auto=1', ('read', 5),
                    'white_balance_temperature_auto=0', ('read', 2)]
        self.assertEqual(events, expected * 2)

    def test_wrong_readback_rejects_candidate(self):
        cap = MagicMock()
        with patch.object(camera, 'set_control'), patch.object(camera, 'read_for'), \
             patch.object(camera, 'command', return_value=self.controls.replace('gain: 80', 'gain: 30')):
            with self.assertRaisesRegex(RuntimeError, '控制读回'):
                camera.initialize_camera(cap, self.profile)

    def test_failed_control_aborts_without_locking_or_publishing(self):
        cap = MagicMock()
        with patch.object(camera, 'read_for'), patch.object(camera, 'command') as readback, \
             patch.object(camera, 'set_control', side_effect=subprocess.CalledProcessError(1, 'v4l2-ctl')) as control:
            with self.assertRaises(subprocess.CalledProcessError):
                camera.initialize_camera(cap, self.profile)
        self.assertEqual(control.call_count, 1)
        readback.assert_not_called()

    def test_wrong_size_and_read_failure_are_rejected(self):
        cap = MagicMock()
        for result in [(False, None), (True, np.zeros((960, 1280, 3), np.uint8))]:
            cap.read.return_value = result
            with self.assertRaises(RuntimeError):
                camera.read_for(cap, 0, (1080, 1920, 3))

    def test_other_camera_identity_is_rejected(self):
        with patch.object(Path, 'exists', return_value=True), \
             patch.object(camera, 'command', return_value='ID_VENDOR_ID=0000\n'):
            with self.assertRaisesRegex(RuntimeError, '身份不匹配'):
                camera.verify_identity(self.profile)


if __name__ == '__main__':
    unittest.main()
