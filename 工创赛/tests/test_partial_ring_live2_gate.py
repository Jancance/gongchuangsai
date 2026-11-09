"""历史观测与显示TTL分离，同时不复活任何过期坐标。"""
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from p2_partial_ring_live_gate_v2 import LivePixelGate

SAMPLE=dict(accepted=True,centers_px=[[100,100],[500,100],[900,100]],method='partial_arc_template_v2')


class TestLive2Gate(unittest.TestCase):
    def test_display_expiry_never_publishes_old_coordinates_but_keeps_valid_observations(self):
        gate=LivePixelGate()
        gate.observe(SAMPLE,(1080,1920,3),1.,1.11)
        stale=gate.current(1.151)
        self.assertEqual(stale['candidate_centers_px'],[])
        self.assertEqual(stale['stable_centers_px'],[])
        self.assertFalse(stale['pixel_ready'])
        gate.observe(SAMPLE,(1080,1920,3),1.13,1.24)
        self.assertFalse(gate.current(1.281)['pixel_ready'])
        stable=gate.observe(SAMPLE,(1080,1920,3),1.26,1.37)
        self.assertTrue(stable['pixel_ready'])
        self.assertAlmostEqual(stable['window_span_ms'],260)
        self.assertEqual(stable['host_read_time'],1.26)
        self.assertEqual(gate.current(1.411)['stable_centers_px'],[])

    def test_real_stale_observation_or_loss_resets_history(self):
        for invalid in ('stale','lost','gap','method','fault'):
            gate=LivePixelGate()
            gate.observe(SAMPLE,(1080,1920,3),1.,1.1)
            gate.observe(SAMPLE,(1080,1920,3),1.1,1.2)
            if invalid=='stale':gate.observe(SAMPLE,(1080,1920,3),1.2,1.351)
            elif invalid=='lost':gate.observe(dict(accepted=False,reason='UNKNOWN'),(1080,1920,3),1.2,1.3)
            elif invalid=='fault':gate.fault('test')
            elif invalid=='gap':gate.observe(SAMPLE,(1080,1920,3),1.4,1.5)
            else:gate.observe(dict(SAMPLE,method='original_quality_candidate'),(1080,1920,3),1.2,1.3)
            stamp=1.5 if invalid=='gap' else 1.3
            status=gate.observe(SAMPLE,(1080,1920,3),stamp,stamp+.1)
            self.assertFalse(status['pixel_ready'],invalid)

    def test_invalid_display_clock_also_resets_history(self):
        gate=LivePixelGate()
        gate.observe(SAMPLE,(1080,1920,3),1.,1.1)
        gate.observe(SAMPLE,(1080,1920,3),1.1,1.2)
        self.assertEqual(gate.current(1.05)['stable_centers_px'],[])
        self.assertFalse(gate.observe(SAMPLE,(1080,1920,3),1.2,1.3)['pixel_ready'])


if __name__=='__main__':unittest.main()
