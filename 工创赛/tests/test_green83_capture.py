"""Capture wrapper verifies exposure without changing other controls or outputs."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock,patch

FILE=Path(__file__).resolve().parents[1]/'output/p2_green83_ring_capture.py'
spec=importlib.util.spec_from_file_location('green83_capture',FILE)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


class Green83Tests(unittest.TestCase):
    def controls(self,exposure=83):
        return f'exposure_auto: 1 (Manual Mode)\nexposure_absolute: {exposure}\ngain: 80\nwhite_balance_temperature_auto: 0\nwhite_balance_temperature: 4600\n'

    def test_only_exposure_is_written_and_readback_verified(self):
        cap=MagicMock();cap.read.return_value=(True,object())
        with patch.object(module.subprocess,'run',side_effect=[SimpleNamespace(stdout=''),SimpleNamespace(stdout=self.controls())]) as command, \
             patch.object(module.time,'monotonic',side_effect=[0.,.1,1.1,1.2]):
            report=module.lock_trial_exposure(cap,'camera')
        self.assertEqual(report['controls']['exposure_absolute'],83)
        self.assertEqual(command.call_args_list[0].args[0],['v4l2-ctl','-d','camera','--set-ctrl=exposure_absolute=83'])
        self.assertEqual(command.call_count,2)
        cap.read.assert_called_once();cap.release.assert_not_called()

    def test_wrong_readback_releases_without_collecting_frames(self):
        cap=MagicMock()
        with patch.object(module.subprocess,'run',side_effect=[SimpleNamespace(stdout=''),SimpleNamespace(stdout=self.controls(166))]):
            with self.assertRaisesRegex(RuntimeError,'mismatch'):module.lock_trial_exposure(cap,'camera')
        cap.read.assert_not_called();cap.release.assert_called_once()

    def test_setting_failure_stops_without_readback(self):
        cap=MagicMock()
        with patch.object(module.subprocess,'run',side_effect=RuntimeError('control failed')) as command:
            with self.assertRaisesRegex(RuntimeError,'control failed'):module.lock_trial_exposure(cap,'camera')
        self.assertEqual(command.call_count,1);cap.release.assert_called_once()

    def test_failed_settling_frame_releases(self):
        cap=MagicMock();cap.read.return_value=(False,None)
        with patch.object(module.subprocess,'run',side_effect=[SimpleNamespace(stdout=''),SimpleNamespace(stdout=self.controls())]), \
             patch.object(module.time,'monotonic',side_effect=[0.,.1]):
            with self.assertRaisesRegex(RuntimeError,'Read failed'):module.lock_trial_exposure(cap,'camera')
        cap.release.assert_called_once()


if __name__=='__main__':unittest.main()
