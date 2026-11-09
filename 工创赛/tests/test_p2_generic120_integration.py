"""隔离进程验证120度候选，避免与旧DECXIN模块缓存混用。"""
from pathlib import Path
import subprocess
import sys
import textwrap
import unittest

ROOT = Path(__file__).resolve().parents[1]
APP = ROOT / 'output/p2_vision_generic120_20261023/vision_app'


class Generic120Tests(unittest.TestCase):
    def run_code(self, code):
        prefix = 'import sys\nsys.path.insert(0, ' + repr(str(APP / 'code')) + ')\n'
        result = subprocess.run([sys.executable, '-X', 'utf8', '-c', prefix + textwrap.dedent(code)],
                                cwd=ROOT, capture_output=True, text=True, encoding='utf-8', timeout=40)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_check_never_opens_camera_and_modes_preserved(self):
        self.run_code('''
            import main as app
            from unittest.mock import patch
            with patch.object(sys, 'argv', ['main', '--check']): args=app.parse_args()
            assert (args.width,args.height)==(1920,1080)
            with patch.object(app.cv2,'VideoCapture',side_effect=AssertionError('camera opened')):
                app.initialize(args)
            assert app.serial_link.status.startswith('DISABLED')
            for key,mode in [('A',app.Mode.TURNTABLE),('D',app.Mode.MEASUREMENT),
                             ('M',app.Mode.MATERIAL_PICKUP),('R',app.Mode.RING_STATION),
                             ('S',app.Mode.RING_STACKING),('F',app.Mode.FOLLOW)]:
                app.handle_key(ord(key)); assert app.current_mode==mode
        ''')

    def test_reopen_initializes_each_session_and_failure_releases(self):
        self.run_code('''
            import project_camera as c
            from unittest.mock import MagicMock,patch
            with patch.object(c,'verify_identity') as identity, patch.object(c,'initialize_camera') as init:
                for _ in range(2): c.prepare_camera(MagicMock(),c.DEFAULT_DEVICE)
                assert identity.call_count==2 and init.call_count==2
                cap=MagicMock(); init.side_effect=RuntimeError('bad controls')
                try: c.prepare_camera(cap,c.DEFAULT_DEVICE)
                except RuntimeError: pass
                else: raise AssertionError('failed controls accepted')
                cap.release.assert_called_once()
        ''')

    def test_old_geometry_and_other_device_rejected(self):
        self.run_code('''
            import project_camera as c
            from unittest.mock import MagicMock,patch
            for device,w,h in [(c.DEFAULT_DEVICE,1280,960),('/dev/video0',1920,1080)]:
                with patch.object(c,'verify_identity') as identity:
                    try: c.prepare_camera(MagicMock(),device,w,h)
                    except RuntimeError: pass
                    else: raise AssertionError('invalid camera accepted')
                    identity.assert_not_called()
            for path in ['decxin_camera_unconfigured.yaml','generic_120_unconfigured.yaml','realtek_rgb_camera.yaml']:
                try: c.calibration_maps(c.DEFAULT_CALIBRATION.parent/path,True)
                except RuntimeError: pass
                else: raise AssertionError('old/unconfigured calibration accepted')
        ''')

    def test_shared_slot_full_resolution_and_no_ready_after_init_failure(self):
        self.run_code('''
            import multiprocessing as mp
            import numpy as np
            import project_capture as capture
            from unittest.mock import MagicMock,patch
            slot=capture.LatestSlot(mp.get_context('spawn'),(1080,1920,3))
            frame=np.zeros((1080,1920,3),np.uint8); slot.publish(frame,1.25)
            seq,stamp,actual=slot.take_after(0)
            assert seq==1 and stamp==1.25 and actual.shape==frame.shape
            ready=MagicMock(); status=MagicMock(); badslot=MagicMock()
            with patch('project_camera.prepare_camera',side_effect=RuntimeError('identity mismatch')), patch.object(capture.cv2,'VideoCapture'):
                capture.capture_worker('invalid',badslot,ready,MagicMock(),status,4)
            ready.set.assert_not_called(); badslot.publish.assert_not_called()
            assert status.put.call_args[0][0]['event']=='error'
        ''')

    def test_thresholds_tracking_and_source_preserved(self):
        self.run_code('''
            import json,hashlib
            from pathlib import Path
            code=Path(sys.path[0]); source=Path('tmp/p2_board_source_20261023/code')
            old=json.loads((source/'config/material_color_thresholds.json').read_text(encoding='utf-8'))
            new=json.loads((code/'config/material_color_thresholds.json').read_text(encoding='utf-8'))
            for key in ('camera_device','image_width','image_height','calibration_file'):
                old.pop(key,None); new.pop(key,None)
            assert old==new
            for name in ['project_follow.py','project_tracker.py','project_detector.py','turntable_serial.py']:
                assert (code/name).read_bytes()==(source/name).read_bytes()
            profile=json.loads((code/'config/project_profile.json').read_text(encoding='utf-8'))
            assert not any(profile[k] for k in ['camera_verified','geometry_verified','ring_geometry_verified','stack_geometry_verified','serial_enabled','follow_grasp_enabled'])
        ''')

    def test_all_four_tools_load_and_use_new_dimensions(self):
        for feature in ('color','turntable-settings','pickup-regions','stack-lines'):
            result = subprocess.run([sys.executable,'-X','utf8',str(APP/'run.py'),'--feature',feature,'--help'],
                                    capture_output=True,text=True,encoding='utf-8',timeout=30)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
        self.run_code('''
            import importlib
            color=importlib.import_module('颜色标定')
            assert (color.IMAGE_WIDTH,color.IMAGE_HEIGHT)==(1920,1080)
            assert color.load_color_config(color.DEFAULT_CONFIG)['image_width']==1920
            importlib.import_module('物料抓取区域标定')
            importlib.import_module('叠放位直线标定')
        ''')


if __name__=='__main__': unittest.main()
