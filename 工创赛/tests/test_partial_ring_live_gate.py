"""验证实时状态门的关键失效/切换行为；不打开摄像头。"""
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from p2_partial_ring_live_gate import LivePixelGate


def sample(method='partial_arc_template_v2',delta=0):
    return dict(accepted=True,centers_px=[[100+delta,100],[500+delta,100],[900+delta,100]],method=method)


class TestGate(unittest.TestCase):
    def test_fresh_window_then_expiry_and_reacquire(self):
        gate=LivePixelGate()
        for stamp in (1.,1.03,1.06):status=gate.observe(sample(),(1080,1920,3),stamp,stamp+.08)
        self.assertTrue(status['pixel_ready'])
        expired=gate.current(1.211)
        self.assertEqual(expired['state'],'STALE')
        self.assertEqual(expired['stable_centers_px'],[])
        self.assertEqual(expired['candidate_centers_px'],[])
        self.assertFalse(gate.observe(sample(),(1080,1920,3),1.22,1.3)['pixel_ready'])

    def test_method_switch_does_not_inherit_window(self):
        gate=LivePixelGate()
        for stamp in (1.,1.03,1.06):gate.observe(sample(),(1080,1920,3),stamp,stamp+.05)
        status=gate.observe(sample('original_quality_candidate'),(1080,1920,3),1.09,1.14)
        self.assertEqual(status['window_frames'],1)
        self.assertFalse(status['pixel_ready'])

    def test_lost_and_fault_revoke_coordinates(self):
        gate=LivePixelGate()
        gate.observe(sample(),(1080,1920,3),1,1.05)
        status=gate.observe(dict(accepted=False,reason='UNKNOWN'),(1080,1920,3),1.03,1.08)
        self.assertEqual(status['state'],'LOST')
        self.assertFalse(status['candidate_centers_px'])
        gate.fault('camera')
        self.assertEqual(gate.current(1.1)['state'],'FAULT')
        self.assertFalse(gate.current(1.1)['stable_centers_px'])

    def test_repeated_future_stale_and_invalid_coordinates_rejected(self):
        for stamp,now in ((1,1.151),(2,1),(float('nan'),1),(True,1)):
            gate=LivePixelGate()
            self.assertEqual(gate.observe(sample(),(1080,1920,3),stamp,now)['state'],'STALE')
        gate=LivePixelGate()
        gate.observe(sample(),(1080,1920,3),1,1.05)
        self.assertEqual(gate.observe(sample(),(1080,1920,3),1,1.07)['state'],'STALE')
        bad=sample();bad['centers_px'][0][0]=-1
        self.assertEqual(gate.observe(bad,(1080,1920,3),1.1,1.15)['state'],'LOST')

    def test_jitter_or_observation_gap_cannot_sign_stable(self):
        gate=LivePixelGate()
        for stamp,delta in ((1,0),(1.03,5),(1.06,0)):
            state=gate.observe(sample(delta=delta),(1080,1920,3),stamp,stamp+.05)
        self.assertFalse(state['pixel_ready'])
        status=gate.observe(sample(),(1080,1920,3),1.4,1.45)
        self.assertEqual(status['window_frames'],1)


if __name__=='__main__':unittest.main()
