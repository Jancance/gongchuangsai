"""验证135°换机的标定隔离和同会话启动控制，不操作硬件。"""
from pathlib import Path
import importlib.util
import json
import unittest
from unittest.mock import Mock, patch

import cv2

ROOT = Path(__file__).resolve().parents[1]
CODE = ROOT / 'output/p2_vision_generic135_20261026/vision_app/code'


def module(name):
    spec = importlib.util.spec_from_file_location(name, CODE / (name + '.py'))
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


class Generic135Tests(unittest.TestCase):
    def test_old_lens_calibration_rejected_even_at_same_resolution(self):
        guard = module('camera135_calibration')
        for profile in ('', 'generic_10bb_2b08_1920x1080_candidate_v1'):
            text = '%YAML:1.0\n---\nimage_width: 1920\nimage_height: 1080\ncalibrated: 1\n'
            if profile:
                text += 'camera_profile_id: ' + profile + '\n'
            fs = cv2.FileStorage(text, cv2.FILE_STORAGE_READ | cv2.FILE_STORAGE_MEMORY)
            try:
                with self.assertRaisesRegex(RuntimeError, '不属于当前135'):
                    guard.validate_calibration_storage(fs)
            finally:
                fs.release()

    def test_current_profile_requires_completion_and_correct_size(self):
        guard = module('camera135_calibration')
        for flag, width, accepted in ((0, 1920, False), (1, 1280, False), (1, 1920, True)):
            text = ('%YAML:1.0\n---\ncamera_profile_id: ' + guard.EXPECTED_PROFILE_ID +
                    f'\ncalibrated: {flag}\nimage_width: {width}\nimage_height: 1080\n')
            fs = cv2.FileStorage(text, cv2.FILE_STORAGE_READ | cv2.FILE_STORAGE_MEMORY)
            try:
                if accepted:
                    guard.validate_calibration_storage(fs)
                else:
                    with self.assertRaises(RuntimeError):
                        guard.validate_calibration_storage(fs)
            finally:
                fs.release()

    def startup(self, gain):
        startup = module('camera135_startup_check')
        profile = json.loads((CODE / 'config/generic_135_camera.json').read_text(encoding='utf-8'))
        cap = Mock()
        cap.isOpened.return_value = True
        readback = ('exposure_auto: 1\nexposure_absolute: 166\ngain: ' + str(gain) +
                    '\nwhite_balance_temperature_auto: 0\nwhite_balance_temperature: 4600\n')
        return startup, profile, cap, readback

    def test_startup_uses_same_capture_and_does_not_write_color_temperature(self):
        startup, profile, cap, readback = self.startup(64)
        with patch.object(startup, 'command', return_value=readback) as command, \
                patch.object(startup, 'read_for', return_value='frame') as read:
            frame, _ = startup.initialize_camera(cap, profile)
        self.assertEqual(frame, 'frame')
        self.assertEqual([c.args[1] for c in read.call_args_list], [0, 7, 2])
        self.assertTrue(all(c.args[0] is cap for c in read.call_args_list))
        settings = [c.args[0][-1] for c in command.call_args_list if c.args[0][-1].startswith('--set-ctrl=')]
        self.assertEqual(settings, ['--set-ctrl=exposure_auto=1', '--set-ctrl=exposure_absolute=166',
                                    '--set-ctrl=gain=64', '--set-ctrl=white_balance_temperature_auto=1',
                                    '--set-ctrl=white_balance_temperature_auto=0'])

    def test_bad_control_readback_rejected(self):
        startup, profile, cap, readback = self.startup(80)
        with patch.object(startup, 'command', return_value=readback), \
                patch.object(startup, 'read_for', return_value='frame'):
            with self.assertRaisesRegex(RuntimeError, '控制读回'):
                startup.initialize_camera(cap, profile)


if __name__ == '__main__':
    unittest.main()
