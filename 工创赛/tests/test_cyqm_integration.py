"""Feature preservation and no-hardware regression for the adapted app."""
import ast
import copy
import importlib
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
CODE=ROOT/'vision_app/code'
sys.path.insert(0,str(CODE))
app=importlib.import_module('main')
camera=importlib.import_module('project_camera')
color=importlib.import_module('颜色标定')

class CyqmIntegrationTests(unittest.TestCase):
    def args(self,*options):
        with patch.object(sys,'argv',['main.py',*options]):return app.parse_args()

    def test_decxin_defaults_and_safe_initialization(self):
        args=self.args('--check')
        self.assertEqual((args.width,args.height),(1280,960))
        self.assertIn('DECXIN',args.device)
        with patch.object(app.cv2,'VideoCapture',side_effect=AssertionError('camera opened')):
            app.initialize(args)
        self.assertIsInstance(app.serial_link,camera.DisabledSerialLink)
        self.assertFalse(app.undistort_enabled)
        self.assertEqual(app.current_mode,app.Mode.FOLLOW)

    def test_all_original_modes_and_new_follow_are_reachable(self):
        app.initialize(self.args('--check'))
        cases={'A':app.Mode.TURNTABLE,'D':app.Mode.MEASUREMENT,'R':app.Mode.RING_STATION,
               'S':app.Mode.RING_STACKING,'M':app.Mode.MATERIAL_PICKUP,'F':app.Mode.FOLLOW,'I':app.Mode.IDLE}
        for key,mode in cases.items():
            self.assertTrue(app.handle_key(ord(key)));self.assertEqual(app.current_mode,mode)
        self.assertFalse(app.handle_key(ord('Q')))

    def test_four_tool_interaction_classes_still_exist(self):
        self.assertTrue(hasattr(color,'ColorCalibratorWindow'))
        self.assertTrue(hasattr(color,'HSVCalibrationSession'))
        params=importlib.import_module('转盘参数设置')
        self.assertTrue(hasattr(params,'ParameterPanel'))
        self.assertTrue(hasattr(params,'ChinesePanelRenderer'))
        self.assertTrue(hasattr(importlib.import_module('物料抓取区域标定'),'RegionSelector'))
        stack_tool=importlib.import_module('叠放位直线标定')
        self.assertTrue(callable(stack_tool.run))

    def test_config_camera_geometry_cannot_be_inherited(self):
        cfg=app.load_material_pickup_config(CODE/'config/material_pickup_detector.json')
        self.assertEqual(cfg['image_height'],960)
        self.assertFalse(cfg['calibrated']);self.assertTrue(all(v is None for v in cfg['regions'].values()))
        self.assertIsNone(camera.calibration_maps(CODE/'config/decxin_camera_unconfigured.yaml',False))
        with self.assertRaisesRegex(RuntimeError,'尚未标定'):camera.calibration_maps(CODE/'config/decxin_camera_unconfigured.yaml',True)
        with self.assertRaisesRegex(RuntimeError,'Realtek'):camera.calibration_maps(CODE/'config/realtek_rgb_camera.yaml',True,size=(1280,720))
        with self.assertRaisesRegex(app.CameraError,'Realtek'):app.load_calibration(CODE/'config/realtek_rgb_camera.yaml')
        maps=camera.calibration_maps(ROOT/'tests/fixtures/decxin_synthetic_camera.yaml',True)
        self.assertEqual(maps[0].shape[:2],(960,1280))

    def test_actual_legacy_serial_requires_unimplemented_gate(self):
        with self.assertRaises(app.CameraError):app.initialize(self.args('--legacy-serial'))
        protocol=importlib.import_module('turntable_serial')
        encoded=protocol.TurntableSerialLink.encode_center_frame(123,456)
        self.assertIsInstance(encoded,bytes);self.assertEqual(len(encoded),7)

    def test_color_config_and_hsv_tools_work_at_960(self):
        cfg=color.load_color_config(CODE/'config/material_color_thresholds.json')
        self.assertEqual((cfg['image_width'],cfg['image_height']),(1280,960))
        masks=color.make_all_color_masks(np.zeros((960,1280,3),np.uint8),cfg)
        self.assertEqual(set(masks),{1,2,3,4,5,6})
        self.assertTrue(all(m.shape==(960,1280) for m in masks.values()))
        session=color.HSVCalibrationSession(cfg)
        window=color.ColorCalibratorWindow(session,CODE/'config/material_color_thresholds.json')
        canvas=window.render(np.zeros((960,1280,3),np.uint8))
        self.assertEqual(canvas.shape[0],960)
        self.assertGreater(canvas.shape[1],1280)

    def test_follow_uses_new_observations_and_does_not_wait_stop(self):
        module=importlib.import_module('project_follow')
        follower=module.MaterialFollower()
        for i in range(3):
            region={'label':'red','silhouette_center_px':[100+i*3,100],
                    'bbox_px':[80+i*3,80,40,40],'area_px2':1600.}
            with patch.object(module,'analyze',return_value=({'regions':[region]},None)), \
                 patch.object(module.time,'monotonic',return_value=1+i*.1+.01):
                result=follower.detect(np.zeros((960,1280,3),np.uint8),timestamp=1+i*.1)
        self.assertEqual(result['state'],'FOLLOWING_PIXEL')
        self.assertGreater(result['targets'][0]['velocity_px_s'][0],0)
        self.assertGreater(result['targets'][0]['predicted_px'][0],106)
        self.assertFalse(result['ready']);self.assertFalse(result['motion_allowed'])
        with patch.object(module,'analyze',return_value=({'regions':[]},None)), \
             patch.object(module.time,'monotonic',return_value=1.31):
            result=follower.detect(np.zeros((960,1280,3),np.uint8),timestamp=1.3)
        self.assertEqual(result['targets'],[])

    def test_follow_stale_input_and_stale_result_clear_history(self):
        module=importlib.import_module('project_follow')
        follower=module.MaterialFollower()
        with patch.object(module.time,'monotonic',return_value=1.2):
            r=follower.detect(np.zeros((960,1280,3),np.uint8),timestamp=1)
        self.assertEqual(r['state'],'STALE_FRAME');self.assertEqual(r['targets'],[])
        with patch.object(module.time,'monotonic',side_effect=[1.01,1.2]), \
             patch.object(module,'analyze',return_value=({'regions':[]},None)):
            r=follower.detect(np.zeros((960,1280,3),np.uint8),timestamp=1)
        self.assertEqual(r['state'],'STALE_RESULT');self.assertFalse(r['motion_allowed'])

    def test_user_camera_size_mismatch_fails_before_any_device(self):
        with patch.object(app.cv2,'VideoCapture',side_effect=AssertionError('camera opened')):
            with self.assertRaises(app.CameraError):app.initialize(self.args('--height','720','--check'))

    def test_source_manifest_retains_all_original_files(self):
        manifest=json.loads((ROOT/'vision_app/upstream_manifest.json').read_text())
        self.assertEqual(len(manifest['files']),23)
        for item in manifest['files']:
            self.assertTrue((ROOT/'vision_app'/item['imported_path']).is_file(),item['imported_path'])

    def test_turntable_finds_and_tracks_synthetic_outer_circle(self):
        module=importlib.import_module('turntable_detector')
        detector=module.TurntableDetector(module.load_turntable_detector_config(CODE/'config/turntable_detector.json'))
        import cv2
        frame=np.full((960,1280,3),35,np.uint8)
        cv2.circle(frame,(640,480),300,(120,120,120),5)
        for _ in range(6):result=detector.detect(frame)
        self.assertTrue(result.detected);self.assertTrue(result.stable)
        self.assertLess(abs(result.center_x_px-640),3)
        self.assertLess(abs(result.center_y_px-480),3)

    def test_three_ring_pixel_detection_retained_without_metric_acceptance(self):
        module=importlib.import_module('ring_station_detector')
        detector=module.RingStationDetector(module.load_ring_station_config(CODE/'config/ring_station_detector.json'))
        import cv2
        frame=np.full((960,1280,3),220,np.uint8)
        for x in (360,640,920):
            for radius in (60,50,40,30,20):cv2.circle(frame,(x,480),radius,(10,10,10),2)
        for _ in range(4):result=detector.detect(frame)
        self.assertTrue(result.detected);self.assertEqual(len(result.centers_px),3)
        self.assertFalse(importlib.import_module('project_geometry').metric_verified('ring'))

    def test_original_material_stop_reference_function_is_preserved(self):
        module=importlib.import_module('material_pickup_detector')
        config=copy.deepcopy(app.load_material_pickup_config(CODE/'config/material_pickup_detector.json'))
        config['calibrated']=True
        config['regions']={'workspace':{'x':0,'y':0,'width':1280,'height':960},
            'left':{'x':100,'y':100,'width':300,'height':200},
            'right':{'x':700,'y':100,'width':300,'height':200},
            'down':{'x':450,'y':600,'width':300,'height':200}}
        colors=color.load_color_config(CODE/'config/material_color_thresholds.json')
        detector=module.MaterialPickupDetector(config,colors)
        import cv2
        states=[]
        for i,x in enumerate([180,190,200,200,200,200,200]):
            frame=np.full((960,1280,3),180,np.uint8)
            cv2.circle(frame,(x,180),25,(0,0,220),-1)
            result=detector.detect(frame,timestamp=1+i*.1)
            states.append(result.state)
        self.assertTrue(result.ready,states)
        self.assertEqual(result.slot_colors,(1,0,0))

    def test_original_reference_configs_preserved(self):
        reference=json.loads((ROOT/'vision_app/reference_config/material_pickup_detector.json').read_text())
        self.assertEqual(reference['image_height'],720)
        self.assertTrue(reference['calibrated'])
        current=json.loads((CODE/'config/stacked_ring_detector.json').read_text())
        self.assertIsNone(current['search_calibration'])

if __name__=='__main__':unittest.main()
