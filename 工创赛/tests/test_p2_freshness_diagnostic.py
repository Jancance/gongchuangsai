"""Diagnostic buffer/freshness contracts, independent of camera hardware."""
import multiprocessing as mp
import unittest
from unittest.mock import patch, MagicMock
import threading
import queue
import numpy as np
from scripts.p2_freshness_audit import (LatestSlot, age_ms, result_fresh, stop_worker,
                                      capture_worker, require_recent_frame, NoFreshFrameError)

def blocked_worker():
    import time
    time.sleep(30)

class LatestFrameDiagnosticTests(unittest.TestCase):
    def test_overwrite_preserves_latest_pixels_and_timestamp(self):
        slot=LatestSlot(mp.get_context('spawn'),(4,4,3))
        self.assertIsNone(slot.take_after(0))
        slot.publish(np.zeros((4,4,3),np.uint8),10.0)
        slot.publish(np.full((4,4,3),7,np.uint8),10.1)
        seq,stamp,im=slot.take_after(0)
        self.assertEqual(seq,2);self.assertEqual(stamp,10.1)
        self.assertTrue(np.all(im==7))
        self.assertIsNone(slot.take_after(seq))
        im[:]=0
        self.assertTrue(np.all(slot.take_after(0)[2]==7))

    def test_size_mismatch_cannot_publish(self):
        slot=LatestSlot(mp.get_context('spawn'),(4,4,3))
        with self.assertRaises(ValueError):slot.publish(np.zeros((2,2,3),np.uint8),0)
        self.assertIsNone(slot.take_after(0))

    def test_processing_can_make_a_fresh_input_expire(self):
        self.assertTrue(result_fresh(10.02,10.,150))
        self.assertFalse(result_fresh(10.20,10.,150))
        self.assertAlmostEqual(age_ms(10.02,10),20)

    def test_no_new_frame_watchdog_trips_at_expiry(self):
        require_recent_frame(10.49,10.,500)
        with self.assertRaises(NoFreshFrameError):require_recent_frame(10.51,10.,500)

    def test_stuck_capture_process_is_terminated(self):
        ctx=mp.get_context('spawn');stop=ctx.Event()
        worker=ctx.Process(target=blocked_worker)
        worker.start()
        self.assertTrue(stop_worker(worker,stop))
        self.assertFalse(worker.is_alive());worker.close()

    def test_capture_uses_requested_buffers_and_reports_timing(self):
        import cv2
        cap=MagicMock();cap.isOpened.return_value=True
        cap.read.return_value=(True,np.zeros((960,1280,3),np.uint8))
        values={cv2.CAP_PROP_FRAME_WIDTH:1280,cv2.CAP_PROP_FRAME_HEIGHT:960,
                cv2.CAP_PROP_FPS:30,cv2.CAP_PROP_FOURCC:cv2.VideoWriter_fourcc(*'MJPG'),
                cv2.CAP_PROP_BUFFERSIZE:4}
        cap.get.side_effect=lambda key:values[key]
        stop=threading.Event();ready=threading.Event();status=queue.Queue()
        slot=MagicMock();slot.publish.side_effect=lambda frame,stamp:stop.set()
        with patch('scripts.p2_freshness_audit.cv2.VideoCapture',return_value=cap), \
             patch('scripts.p2_freshness_audit.subprocess.run') as command:
            command.return_value.stdout='test readback'
            capture_worker('fake-device',slot,ready,stop,status,4)
        cap.set.assert_any_call(cv2.CAP_PROP_BUFFERSIZE,4)
        self.assertTrue(ready.is_set());cap.release.assert_called_once()
        event=status.get_nowait();self.assertEqual(event['requested_buffers'],4)
        self.assertEqual(event['opencv_mode_readback']['fourcc'],'MJPG')
        timing=status.get_nowait();self.assertEqual(timing['event'],'capture_timing')
        self.assertEqual(timing['host_reads_including_shutdown'],1)

    def test_injected_pause_publishes_no_new_frame(self):
        import cv2
        cap=MagicMock();cap.isOpened.return_value=True
        cap.read.return_value=(True,np.zeros((960,1280,3),np.uint8))
        cap.get.return_value=0
        stop=MagicMock();stop.is_set.return_value=False
        ready=threading.Event();status=queue.Queue();slot=MagicMock()
        with patch('scripts.p2_freshness_audit.cv2.VideoCapture',return_value=cap), \
             patch('scripts.p2_freshness_audit.subprocess.run') as command:
            command.return_value.stdout='test readback'
            capture_worker('fake-device',slot,ready,stop,status,4,0)
        slot.publish.assert_not_called();stop.wait.assert_called_once()
        self.assertEqual(cap.read.call_count,91)
        self.assertEqual(status.get_nowait()['event'],'ready')
        event=status.get_nowait();self.assertEqual(event['event'],'simulated_capture_stall')
        self.assertEqual(event['published_before_pause'],0)
        cap.release.assert_called_once()

if __name__=='__main__':unittest.main()
