"""Watch selected colors without replacing actual target labels."""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import time
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
spec=importlib.util.spec_from_file_location('color_follow_audit',ROOT/'scripts/p2_color_follow_audit.py')
audit=importlib.util.module_from_spec(spec);spec.loader.exec_module(audit)


class ColorAuditTests(unittest.TestCase):
    def test_periodic_raw_capture_when_watched_color_never_appears_is_bounded(self):
        with tempfile.TemporaryDirectory() as temp:
            recorder=audit.ColorRecorder(Path(temp)/'run','lightblue',sample_every=2.,max_samples=2)
            for offset,gray in [(0.,10),(1.,20),(2.,30),(4.,40)]:
                stamp=100.+offset
                value={'state':'FOLLOWING_PIXEL','timestamp':stamp,
                       'targets':[{'id':'T001','color':'blue'}],
                       'detector_regions':[],'ready':False,'motion_allowed':False}
                recorder.observe(np.full((16,16,3),gray,np.uint8),value,stamp-.01,stamp)
                self.assertEqual(value['targets'][0]['color'],'blue')
            recorder.close(0)
            from cv2 import imread
            self.assertTrue(np.all(imread(str(recorder.folder/'sample-01-raw.png'))==10))
            self.assertTrue(np.all(imread(str(recorder.folder/'sample-02-raw.png'))==30))
            self.assertFalse((recorder.folder/'sample-03-raw.png').exists())
            summary=json.loads((recorder.folder/'run.json').read_text(encoding='utf-8'))
            self.assertEqual(summary['periodic_samples']['saved'],2)

    def test_blue_loss_and_reappearance_preserve_blue_and_other_labels(self):
        with tempfile.TemporaryDirectory() as temp:
            recorder=audit.ColorRecorder(Path(temp)/'run','blue')
            for stamp,targets in [(1.,[{'id':'T001','color':'blue'}]),
                                  (1.1,[{'id':'T002','color':'black'}]),
                                  (1.3,[{'id':'T003','color':'blue'}])]:
                result={'state':'FOLLOWING_PIXEL','timestamp':stamp,'targets':targets,
                        'detector_regions':[],'ready':False,'motion_allowed':False}
                now=time.monotonic();recorder.observe(np.zeros((16,16,3),np.uint8),result,now-.01,now)
            recorder.close(0)
            meta=json.loads((recorder.folder/'first-blue-loss.json').read_text(encoding='utf-8'))
            self.assertEqual(meta['previous']['targets'][0]['color'],'blue')
            self.assertEqual(meta['current']['targets'][0]['color'],'black')
            self.assertTrue((recorder.folder/'first-blue-id-change.json').is_file())
            self.assertTrue((recorder.folder/'longest-blue-loss.json').is_file())
            summary=json.loads((recorder.folder/'run.json').read_text(encoding='utf-8'))
            self.assertEqual(summary['watch_color'],'blue')
            self.assertAlmostEqual(summary['longest_watched_loss_s'],.2)
            self.assertNotIn('longest_red_loss_s',summary)
            self.assertFalse(summary['motion_allowed'])

    def test_no_watched_color_still_captures_first_frame(self):
        with tempfile.TemporaryDirectory() as temp:
            recorder=audit.ColorRecorder(Path(temp)/'run','lightblue')
            now=time.monotonic()
            value={'state':'FOLLOWING_PIXEL','timestamp':now,'targets':[{'id':'T001','color':'black'}],
                   'detector_regions':[],'ready':False,'motion_allowed':False}
            recorder.observe(np.zeros((16,16,3),np.uint8),value,now-.01,now)
            recorder.close(0)
            self.assertTrue((recorder.folder/'first-frame-raw.png').is_file())
            self.assertNotIn('first-lightblue-visible',recorder.saved)

    def test_red_candidate_does_not_consume_blue_confusion_image(self):
        with tempfile.TemporaryDirectory() as temp:
            recorder=audit.ColorRecorder(Path(temp)/'run','lightblue')
            for stamp,label,gray in [(1.,'lightblue',10),(1.1,'red',20),(1.2,'blue',30)]:
                now=time.monotonic()
                value={'state':'FOLLOWING_PIXEL','timestamp':stamp,
                       'targets':[{'id':'T001','color':label}],
                       'detector_regions':[],'ready':False,'motion_allowed':False}
                recorder.observe(np.full((16,16,3),gray,np.uint8),value,now-.01,now)
            recorder.close(0)
            generic=json.loads((recorder.folder/'first-other-color-candidate.json').read_text(encoding='utf-8'))
            specific=json.loads((recorder.folder/'first-lightblue-with-blue.json').read_text(encoding='utf-8'))
            self.assertEqual(generic['current']['targets'][0]['color'],'red')
            self.assertEqual(specific['current']['targets'][0]['color'],'blue')
            self.assertTrue((recorder.folder/'first-lightblue-with-red-raw.png').is_file())
            from cv2 import imread
            self.assertTrue(np.all(imread(str(recorder.folder/'first-lightblue-with-blue-raw.png'))==30))


if __name__=='__main__':unittest.main()
