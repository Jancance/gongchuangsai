"""圆环像素记录必须拒绝过期/旧时间戳/失败坐标，且不改变原R检测器。"""
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from zipfile import ZipFile

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('ring_pixel_audit',ROOT/'scripts/p2_ring_pixel_audit.py')
audit=importlib.util.module_from_spec(spec);spec.loader.exec_module(audit)


class RingPixelAuditTests(unittest.TestCase):
    def setUp(self):
        (ROOT/'tmp').mkdir(exist_ok=True)
        self.temp=tempfile.TemporaryDirectory(dir=ROOT/'tmp')
        self.root=Path(self.temp.name).resolve()
        assert self.root.is_relative_to((ROOT/'tmp').resolve())
        self.addCleanup(self.temp.cleanup)
        self.frame=np.full((100,160,3),230,np.uint8)

    def result(self,detected=True,shift=0):
        return SimpleNamespace(detected=detected,centers_px=((20.+shift,50.),(70.+shift,50.),(120.+shift,50.)),
                               processing_ms=5.,confidence=.95,stable=True,detail='test')

    def close(self,rec):
        with contextlib.redirect_stdout(io.StringIO()):return rec.close(0)

    def test_fresh_statistics_and_complete_zip(self):
        r=audit.RingPixelRecorder(self.root/'run',3.)
        for stamp,shift in ((10.,0.),(10.1,.2),(10.2,-.2)):
            row=r.observe(self.frame,self.frame,self.result(shift=shift),stamp,stamp+.01)
            self.assertTrue(row['usable_for_pixel_statistics'])
        result=self.close(r)
        self.assertEqual(result['usable_frames'],3)
        self.assertAlmostEqual(result['ring_statistics'][0]['range_xy_px'][0],.4)
        self.assertEqual(result['ring_statistics'][0]['median_xy_px'],[20.,50.])
        self.assertFalse(result['metric_valid']);self.assertFalse(result['motion_allowed'])
        with ZipFile(r.folder.with_suffix('.zip')) as z:
            self.assertIsNone(z.testzip())
            for name in z.namelist():self.assertEqual(z.read(name),(r.folder/name).read_bytes())

    def test_stale_missing_and_nonincreasing_stamp_never_enter_statistics(self):
        r=audit.RingPixelRecorder(self.root/'timestamps')
        for stamp,finished in ((10.,10.01),(10.,10.02),(9.,10.03),(9.5,10.04),(None,10.05),(10.1,10.3)):
            row=r.observe(self.frame,self.frame,self.result(),stamp,finished)
            if stamp!=10. or finished!=10.01:self.assertEqual(row['usable_centers_px'],[])
        self.assertEqual(self.close(r)['usable_frames'],1)

    def test_failed_or_invalid_detection_does_not_reuse_centers(self):
        r=audit.RingPixelRecorder(self.root/'invalid')
        r.observe(self.frame,self.frame,self.result(),10.,10.01)
        cases=[self.result(detected=False),SimpleNamespace(detected=True,centers_px=((float('nan'),5.),)*3),
               SimpleNamespace(detected=True,centers_px=((500.,5.),)*3),SimpleNamespace(detected=True,centers_px=((1.,1.),))]
        for i,result in enumerate(cases):
            stamp=10.1+i*.1
            row=r.observe(self.frame,self.frame,result,stamp,stamp+.01)
            self.assertEqual(row['usable_centers_px'],[])
        self.assertEqual(self.close(r)['usable_frames'],1)

    def test_duration_and_bounded_snapshots(self):
        r=audit.RingPixelRecorder(self.root/'duration',3.)
        self.assertFalse(r.done(100.))
        for i in range(16):
            stamp=10.+i*.1
            r.observe(self.frame,self.frame,self.result(detected=i%2==0),stamp,stamp+.01)
        self.assertFalse(r.done(12.9));self.assertTrue(r.done(13.02))
        self.close(r)
        self.assertLessEqual(len(list(r.folder.glob('*.png'))),10)

    def test_low_confidence_or_waiting_candidates_are_not_stable_coordinates(self):
        r=audit.RingPixelRecorder(self.root/'quality')
        good=self.result()
        row=r.observe(self.frame,self.frame,good,10.,10.01)
        self.assertTrue(row['pixel_ready'])
        low=self.result(shift=13.)
        low.confidence=.84;low.stable=False
        row=r.observe(self.frame,self.frame,low,10.1,10.11)
        self.assertTrue(row['usable_for_pixel_statistics'])
        self.assertEqual(row['stable_centers_px'],[])
        self.assertFalse(row['pixel_ready'])
        waiting=self.result();waiting.stable=False
        row=r.observe(self.frame,self.frame,waiting,10.2,10.21)
        self.assertFalse(row['pixel_ready'])
        row=r.observe(self.frame,self.frame,self.result(),10.3,10.31)
        self.assertTrue(row['pixel_ready'])
        summary=self.close(r)
        self.assertEqual(summary['usable_frames'],4)
        self.assertEqual(summary['pixel_ready_frames'],2)
        self.assertEqual(summary['stable_ring_statistics'][0]['range_xy_px'],[0.,0.])
        self.assertTrue((r.folder/'first-stable-reappearance-raw.png').exists())

    def test_detector_stable_flag_cannot_override_low_confidence_or_stale_frame(self):
        r=audit.RingPixelRecorder(self.root/'quality-conflict')
        low=self.result();low.confidence=.84
        row=r.observe(self.frame,self.frame,low,10.,10.01)
        self.assertFalse(row['pixel_ready'])
        row=r.observe(self.frame,self.frame,self.result(),10.1,10.3)
        self.assertEqual(row['stable_centers_px'],[])
        self.assertEqual(self.close(r)['pixel_ready_frames'],0)

    def test_entry_switches_original_app_to_r_and_records_without_replacing_detector(self):
        fake=SimpleNamespace(Mode=SimpleNamespace(RING_STATION='R'),current_mode='FOLLOW',
                             camera=SimpleNamespace(last_read_completed_time=None))
        calls=[]
        fake.initialize=lambda args:calls.append('initialize')
        fake.set_mode=lambda mode:setattr(fake,'current_mode',mode)
        def process(frame):calls.append('original-ring-process');return self.result()
        fake.process_ring_station=process
        fake.print_runtime_status=lambda *args:None
        fake.handle_key=lambda key:key not in (ord('q'),ord('Q'),27)
        def run():
            fake.initialize(None)
            self.assertEqual(fake.current_mode,'R')
            for _ in range(2):
                fake.camera.last_read_completed_time=audit.time.monotonic()-.01
                result=fake.process_ring_station(self.frame.copy())
                fake.print_runtime_status(result,25.,1)
            self.assertFalse(fake.handle_key(ord('q')))
            return 0
        fake.main=run
        argv=['audit','--app',str(ROOT/'output/p2_vision_generic135_lightblue1_20261027/vision_app'),
              '--output-dir',str(self.root/'entry')]
        output=io.StringIO()
        with patch.object(sys,'argv',argv),patch.object(audit.importlib,'import_module',return_value=fake),contextlib.redirect_stdout(output):
            self.assertEqual(audit.main(),0)
        self.assertEqual(calls,['initialize','original-ring-process','original-ring-process'])
        self.assertIn('Centers=',output.getvalue())
        archive=next((self.root/'entry').glob('*.zip'))
        with ZipFile(archive) as z:
            summary=json.loads(z.read('run.json'))
            self.assertEqual(summary['frames'],2)
            self.assertFalse(summary['completed_duration'])


if __name__=='__main__':unittest.main()
