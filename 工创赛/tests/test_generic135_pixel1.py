"""普通R入口的稳定/候选边界、失效清空与实机旧证据回放。"""
import argparse
from contextlib import redirect_stdout
import importlib.util
import io
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
APP = ROOT / 'output/p2_vision_generic135_pixel1_20261027/vision_app'
spec = importlib.util.spec_from_file_location('pixel1_presenter', APP/'code/project_ring_pixel.py')
pixel = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pixel)


class PresenterTests(unittest.TestCase):
    def result(self, stable=True, confidence=.99, detected=True, shift=0.):
        return SimpleNamespace(stable=stable, confidence=confidence, detected=detected,
                               centers_px=((30.+shift, 60.), (80.+shift, 60.), (130.+shift, 60.)))

    def test_low_confidence_waiting_loss_and_recovery_never_share_stable_output(self):
        gate = pixel.RingPixelPresenter()
        self.assertEqual(gate.observe(self.result(), (100,200,3), 10., 10.02)['state'], 'STABLE')
        low = gate.observe(self.result(stable=True,confidence=.84,shift=13.), (100,200,3), 10.1,10.12)
        self.assertEqual(low['state'], 'CANDIDATE')
        self.assertTrue(low['candidate_centers_px'])
        self.assertEqual(low['stable_centers_px'], [])
        wait = gate.observe(self.result(stable=False), (100,200,3),10.2,10.22)
        self.assertEqual(wait['reason'], 'WINDOW_NOT_STABLE')
        lost = gate.observe(self.result(detected=False), (100,200,3),10.3,10.32)
        self.assertEqual(lost['state'],'LOST')
        self.assertEqual(lost['candidate_centers_px'],[])
        self.assertEqual(lost['stable_centers_px'],[])
        self.assertTrue(gate.observe(self.result(),(100,200,3),10.4,10.42)['pixel_ready'])

    def test_expiry_at_print_time_clears_a_previously_stable_coordinate(self):
        gate = pixel.RingPixelPresenter()
        gate.observe(self.result(),(100,200,3),10.,10.02)
        self.assertTrue(gate.current(10.1)['pixel_ready'])
        status = gate.current(10.3)
        self.assertEqual(status['state'],'STALE')
        self.assertEqual(status['stable_centers_px'],[])
        self.assertEqual(status['candidate_centers_px'],[])

    def test_duplicate_backward_missing_and_future_stamps_do_not_reuse_old_result(self):
        gate = pixel.RingPixelPresenter()
        gate.observe(self.result(),(100,200,3),10.,10.01)
        for stamp, now in ((10.,10.02),(9.9,10.03),(None,10.04),(11.,10.05),(10.2,10.21)):
            status=gate.observe(self.result(),(100,200,3),stamp,now)
            self.assertEqual(status['state'],'STALE')
            self.assertFalse(status['pixel_ready'])
        gate.reset()
        self.assertEqual(gate.current(20.)['state'],'WAITING')
        self.assertTrue(gate.observe(self.result(),(100,200,3),1.,1.01)['pixel_ready'])

    def test_invalid_geometry_and_nan_confidence_cannot_produce_stable_coordinates(self):
        for centers in (((float('nan'),50.),)*3, ((200.,50.),)*3, ((1.,2.),), None):
            gate=pixel.RingPixelPresenter();result=self.result();result.centers_px=centers
            status=gate.observe(result,(100,200,3),10.,10.01)
            self.assertEqual(status['stable_centers_px'],[])
        status=pixel.RingPixelPresenter().observe(self.result(confidence=float('nan')),(100,200,3),10.,10.01)
        self.assertFalse(status['pixel_ready'])
        json.dumps(status,allow_nan=False)

    def test_manual_labels_are_explicit_never_inferred_or_used_to_reorder_centers(self):
        self.assertEqual(pixel.parse_paper_labels('3,2,1'),(3,2,1))
        for text in ('1,1,2','3,2','1,2,7','abc','1,2,3,4'):
            with self.assertRaises(argparse.ArgumentTypeError):pixel.parse_paper_labels(text)
        plain=pixel.RingPixelPresenter().observe(self.result(),(100,200,3),10.,10.01)
        self.assertIsNone(plain['paper_labels'])
        gate=pixel.RingPixelPresenter(paper_labels=(3,2,1),ring1_side='right')
        tagged=gate.observe(self.result(),(100,200,3),10.,10.01)
        self.assertEqual(tagged['stable_centers_px'],plain['stable_centers_px'])
        self.assertEqual(tagged['label_source'],'MANUAL_FIXED_LAYOUT_NOT_OCR')
        self.assertEqual(tagged['order'],'RIGHT_TO_LEFT')

    def test_saved_v2_board_records_keep_the_validated_gate_results(self):
        cases=(('ring-rotation','20261027-193422-279783',680),
               ('ring-shadow','20261027-194314-063293',514),
               ('ring-empty','20261027-194859-020849',0))
        for label,run_name,expected_count in cases:
            folder=ROOT/'evidence/p2-camera/2026-10-27'/label/('run-'+run_name)
            rows=[json.loads(s) for s in (folder/'observations.jsonl').read_text(encoding='utf8').splitlines()]
            gate=pixel.RingPixelPresenter();count=0
            for row in rows:
                result=SimpleNamespace(detected=row['detected'],stable=row['detector_stable'],
                                       confidence=row['confidence'],centers_px=row['centers_px'])
                status=gate.observe(result,(1080,1920,3),row['host_read_time'],
                                    row['host_read_time']+row['result_host_age_ms']/1000)
                self.assertEqual(status['pixel_ready'],row['pixel_ready'])
                self.assertEqual(status['stable_centers_px'],row['stable_centers_px'])
                count+=status['pixel_ready']
            self.assertEqual(count,expected_count)

    def test_overlay_distinguishes_candidate_color_and_draws_no_cross_for_expired_result(self):
        gate=pixel.RingPixelPresenter(paper_labels=(3,2,1))
        frame=np.full((1080,1920,3),220,np.uint8)
        candidate=gate.observe(self.result(stable=False),(1080,1920,3),10.,10.01)
        with patch.object(pixel.cv2,'drawMarker') as marker:
            pixel.draw_ring_pixels(frame,candidate)
            self.assertEqual(marker.call_count,3)
            self.assertEqual(marker.call_args.args[2],(0,215,255))
        with patch.object(pixel.cv2,'drawMarker') as marker:
            pixel.draw_ring_pixels(frame,gate.current(10.3))
            marker.assert_not_called()


class MainIntegrationTests(unittest.TestCase):
    def run_code(self, code):
        prefix='import sys\nsys.path.insert(0,'+repr(str(APP/'code'))+')\n'
        result=subprocess.run([sys.executable,'-X','utf8','-c',prefix+code],cwd=ROOT,
                              capture_output=True,text=True,encoding='utf8',timeout=40)
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)

    def test_check_preserves_modes_four_tools_and_rejects_actual_serial(self):
        self.run_code('''
import main as app
from unittest.mock import patch
with patch.object(sys,'argv',['main','--check']):args=app.parse_args()
with patch.object(app.cv2,'VideoCapture',side_effect=AssertionError('camera opened')):
    app.initialize(args)
assert app.current_mode==app.Mode.FOLLOW
assert app.serial_link.status.startswith('DISABLED')
for key,mode in [('A',app.Mode.TURNTABLE),('D',app.Mode.MEASUREMENT),('M',app.Mode.MATERIAL_PICKUP),('R',app.Mode.RING_STATION),('S',app.Mode.RING_STACKING),('F',app.Mode.FOLLOW)]:
    app.handle_key(ord(key));assert app.current_mode==mode
with patch.object(sys,'argv',['main','--check','--legacy-serial']):args=app.parse_args()
try:app.initialize(args)
except app.CameraError:pass
else:raise AssertionError('legacy serial accepted')
''')
        import hashlib
        base=ROOT/'output/p2_vision_generic135_lightblue1_20261027/vision_app'
        manifest=json.loads((APP/'PIXEL1_MANIFEST.json').read_text(encoding='utf8'))
        self.assertEqual(manifest['changed_files'],['code/main.py'])
        for name,expected in manifest['source_files'].items():
            self.assertEqual(hashlib.sha256((base/name).read_bytes()).hexdigest(),expected)
            if name!='code/main.py':self.assertEqual((base/name).read_bytes(),(APP/name).read_bytes())

    def test_normal_r_process_outputs_stable_then_candidate_then_lost_and_expires(self):
        self.run_code('''
import contextlib,io,json,time
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import main as app
with patch.object(sys,'argv',['main','--start-ring','--ring-paper-labels','3,2,1']):args=app.parse_args()
app.initialize(args)
assert app.current_mode==app.Mode.RING_STATION
app.camera=SimpleNamespace(last_read_completed_time=10.)
def result(stable=True,confidence=.99,detected=True):
    return SimpleNamespace(detected=detected,stable=stable,confidence=confidence,centers_px=((400.,500.),(900.,500.),(1400.,500.)))
frame=np.zeros((1080,1920,3),np.uint8)
with patch.object(app.time,'monotonic',return_value=10.02),patch.object(app.ring_station_detector,'detect',return_value=result()):
    app.process_ring_station(frame)
    output=io.StringIO()
    with contextlib.redirect_stdout(output):app.print_runtime_status(None,25.,1)
    assert 'State=STABLE' in output.getvalue() and 'MANUAL_FIXED_LAYOUT_NOT_OCR' in output.getvalue()
with patch.object(app.time,'monotonic',return_value=10.3),patch.object(app.ring_station_detector,'reset') as reset:
    assert app.current_ring_pixel_status()['state']=='STALE'
    reset.assert_called_once()
for stamp,res,expected in [(11.,result(False,.84),'CANDIDATE'),(12.,result(False,0.,False),'LOST')]:
    app.camera.last_read_completed_time=stamp
    with patch.object(app.time,'monotonic',return_value=stamp+.02),patch.object(app.ring_station_detector,'detect',return_value=res):
        app.process_ring_station(frame)
        state=app.current_ring_pixel_status()
        assert state['state']==expected and state['stable_centers_px']==[]
app.reset_detection_history()
assert app.ring_pixel_presenter.current(13.)['state']=='WAITING'
assert app.serial_link.sent_ring_batches==0
assert app.ring_station_detector.config.ring1_side=='left'
''')


if __name__=='__main__':
    unittest.main()
