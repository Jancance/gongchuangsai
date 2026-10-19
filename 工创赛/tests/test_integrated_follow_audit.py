"""Evidence correctness through lost frames; no camera, serial or motion."""
import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import time
import threading
import unittest
from zipfile import ZipFile
import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('follow_audit', ROOT/'scripts/p2_integrated_follow_audit.py')
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


def result(identity=None, state='FOLLOWING_PIXEL', regions=None):
    return {'timestamp': time.monotonic(), 'state': state,
            'targets': [] if identity is None else [{'id': identity, 'color': 'red'}],
            'detector_regions': regions or [], 'ready': False, 'motion_allowed': False}


class IntegratedFollowAuditTests(unittest.TestCase):
    def observe(self, recorder, frame, value):
        now = time.monotonic()
        recorder.observe(frame, value, now-.01, now)

    def test_loss_preserves_raw_before_and_after_and_reject_reason(self):
        with tempfile.TemporaryDirectory() as temp:
            recorder = audit.FollowRecorder(Path(temp)/'run')
            before = np.full((32,32,3), 21, np.uint8)
            after = np.full((32,32,3), 42, np.uint8)
            self.observe(recorder, before, result('T001'))
            before[:] = 255  # Caller subsequently draws on the original frame.
            lost = result(state='SEARCHING', regions=[{'label':'UNKNOWN','reason':'SHAPE_UNCERTAIN'}])
            saved = copy.deepcopy(lost)
            self.observe(recorder, after, lost)
            self.assertEqual(lost, saved)
            recorder.close(0)
            self.assertTrue(np.all(cv2.imread(str(recorder.folder/'first-red-loss-before-raw.png')) == 21))
            self.assertTrue(np.all(cv2.imread(str(recorder.folder/'first-red-loss-raw.png')) == 42))
            meta = json.loads((recorder.folder/'first-red-loss.json').read_text(encoding='utf-8'))
            self.assertEqual(meta['previous']['targets'][0]['id'], 'T001')
            self.assertEqual(meta['current']['targets'], [])
            self.assertEqual(meta['current']['detector_regions'][0]['reason'], 'SHAPE_UNCERTAIN')
            with ZipFile(recorder.folder.with_suffix('.zip')) as bundle:
                self.assertIsNone(bundle.testzip())
                self.assertIn('observations.jsonl', bundle.namelist())
                self.assertEqual(bundle.read('first-red-loss-raw.png'),
                                 (recorder.folder/'first-red-loss-raw.png').read_bytes())

    def test_id_change_across_missing_gap_and_all_frames_logged(self):
        with tempfile.TemporaryDirectory() as temp:
            recorder = audit.FollowRecorder(Path(temp)/'run')
            frame = np.zeros((16,16,3), np.uint8)
            for value in (result(), result('T001'), result(state='SEARCHING'), result('T002')):
                self.observe(recorder, frame, value)
            recorder.close(0)
            self.assertIn('first-red-id-change', recorder.saved)
            self.assertIn('first-red-reappearance', recorder.saved)
            rows = [json.loads(s) for s in (recorder.folder/'observations.jsonl').read_text(encoding='utf-8').splitlines()]
            self.assertEqual(len(rows),4)
            self.assertEqual(rows[-1]['targets'][0]['id'],'T002')
            summary = json.loads((recorder.folder/'run.json').read_text(encoding='utf-8'))
            self.assertFalse(summary['motion_allowed'])

    def test_same_id_reappearance_is_separate_from_identity_change(self):
        with tempfile.TemporaryDirectory() as temp:
            recorder = audit.FollowRecorder(Path(temp)/'run')
            frame = np.zeros((16,16,3),np.uint8)
            for value in (result('T001'), result(state='STALE_RESULT'), result('T001'),
                          result(state='SEARCHING'), result('T001')):
                self.observe(recorder,frame,value)
            recorder.close(0)
            self.assertNotIn('first-red-id-change', recorder.saved)
            self.assertEqual(recorder.transitions['first-red-loss'],2)
            meta=json.loads((recorder.folder/'first-red-loss.json').read_text(encoding='utf-8'))
            self.assertEqual(meta['current']['state'],'STALE_RESULT')

    def test_startup_no_red_has_no_false_loss_and_existing_run_not_overwritten(self):
        with tempfile.TemporaryDirectory() as temp:
            folder=Path(temp)/'run'
            recorder=audit.FollowRecorder(folder)
            self.observe(recorder,np.zeros((16,16,3),np.uint8),result(state='SEARCHING'))
            recorder.close(0)
            recorder.close(0)
            self.assertEqual(recorder.saved,set())
            with self.assertRaises(FileExistsError):audit.FollowRecorder(folder)

    def test_image_io_does_not_block_detection_and_shutdown_waits_for_evidence(self):
        with tempfile.TemporaryDirectory() as temp:
            recorder=audit.FollowRecorder(Path(temp)/'run')
            entered=threading.Event();release=threading.Event()
            original=recorder.write_image
            def delayed_image(path,frame):
                entered.set()
                if not release.wait(2):raise RuntimeError('Test writer timeout')
                return original(path,frame)
            recorder.write_image=delayed_image
            try:
                self.observe(recorder,np.zeros((16,16,3),np.uint8),result('T001'))
                self.assertTrue(entered.wait(1))
                self.assertFalse(recorder.pending[0].done())
            finally:
                release.set()
                recorder.close(0)
            self.assertTrue((recorder.folder/'first-red-visible-raw.png').is_file())

    def test_background_save_failure_is_reported_as_failure(self):
        with tempfile.TemporaryDirectory() as temp:
            recorder=audit.FollowRecorder(Path(temp)/'run')
            def failed_image(path,frame):raise RuntimeError('disk unavailable')
            recorder.write_image=failed_image
            self.observe(recorder,np.zeros((16,16,3),np.uint8),result('T001'))
            with self.assertRaisesRegex(RuntimeError,'Evidence save failed'):
                recorder.close(0)
            summary=json.loads((recorder.folder/'run.json').read_text(encoding='utf-8'))
            self.assertEqual(summary['exit_code'],1)
            self.assertIn('disk unavailable',summary['error'])

    def test_longest_loss_saves_last_missing_frame_and_reappearance(self):
        with tempfile.TemporaryDirectory() as temp:
            recorder=audit.FollowRecorder(Path(temp)/'run')
            for stamp,identity,gray in [(1.,'T001',10),(1.1,None,20),(1.2,'T001',30),
                                       (2.,None,40),(2.8,None,50),(3.,'T002',60)]:
                value=result(identity,state='SEARCHING' if identity is None else 'FOLLOWING_PIXEL')
                value['timestamp']=stamp
                self.observe(recorder,np.full((16,16,3),gray,np.uint8),value)
            recorder.close(0)
            meta=json.loads((recorder.folder/'longest-red-loss.json').read_text(encoding='utf-8'))
            self.assertEqual(meta['loss_duration_s'],1.)
            self.assertEqual(meta['previous']['timestamp'],2.8)
            self.assertEqual(meta['current']['targets'][0]['id'],'T002')
            self.assertTrue(np.all(cv2.imread(str(recorder.folder/'longest-red-loss-before-raw.png'))==50))


if __name__ == '__main__':
    unittest.main()
