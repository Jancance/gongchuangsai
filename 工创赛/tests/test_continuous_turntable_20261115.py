import math
from pathlib import Path
import json
import unittest
import numpy as np
from smart_carrier.vision_manipulation.continuous_turntable import ContinuousTurntable,rim_observation_valid
from smart_carrier.vision_manipulation.mcu_cycle_protocol import frame,decode_status,continuous_observation

R=Path(__file__).resolve().parents[1]

class ContinuousTests(unittest.TestCase):
    def test_learn_real_turn_then_schedule_both_directions(self):
        cfg=json.loads((R/'evidence/integration/2026-11-15/source-continuous/prediction-config.json').read_text())
        for omega in (-1.1,1.1):
            model=ContinuousTurntable(cfg['target_angles_rad'],cfg['lead_ms_by_region'])
            def regions(t,colors=(2,1,5)):
                out=[];matrix=np.array([[430,18],[18,385]])
                for color,offset in ((2,.3),(1,.3+2*math.pi/3),(5,.3+4*math.pi/3)):
                    if color not in colors:continue
                    angle=omega*t+offset;p=np.array([970,390])+matrix@np.array([math.cos(angle),math.sin(angle)])
                    out.append(dict(color_id=color,center_x_px=p[0],center_y_px=p[1]))
                return out
            for t in np.arange(0,2*math.pi/abs(omega)-.05,.05):
                self.assertIsNone(model.update(regions(t),t,t,2,[2,1,5]))
            prediction=None
            for t in np.arange(2*math.pi/abs(omega),2*math.pi/abs(omega)+.3,.05):
                prediction=model.update(regions(t),t,t,2,[2,1,5])
                if prediction is not None:break
            self.assertIsNotNone(prediction);self.assertTrue(model.learned)
            for i,delay in enumerate(prediction['start_delay_ms']):
                phase=omega*(t+delay/1000+cfg['lead_ms_by_region'][i]/1000)+.3
                error=math.atan2(math.sin(phase-cfg['target_angles_rad'][i]),math.cos(phase-cfg['target_angles_rad'][i]))
                self.assertAlmostEqual(error,0,places=7)
            model.pause()
            self.assertIsNone(model.update(regions(20,(5,)),20,20,5,[5]))
            self.assertIsNotNone(model.update(regions(20.05,(5,)),20.05,20.05,5,[5]))
            model.reset();self.assertFalse(model.learned)

    def test_user_rim_gate_waits_and_has_no_actuator_api(self):
        for value,expected in ((.557,True),(3,True),(3.001,False),(50.3,False),(float('nan'),False)):
            self.assertEqual(rim_observation_valid({'fit_rms_px':value}),expected)
        self.assertFalse(rim_observation_valid({}))

    def test_new_wire_separates_measured_colours_and_future_start_times(self):
        wire=continuous_observation(1,4,2,True,[2,5,1],10,2,[300,100,400])
        self.assertTrue(wire.startswith(b'OC2,1,4,2,1,0,0,0,1,2,5,1,10,2,300,100,400,'))
        status=decode_status(frame('SC2',[1,7,5,6,0,2,1,5,0,0,0,1,100]))
        self.assertEqual(status['scheduled'],1);self.assertEqual(status['start_in_ms'],100)
        bad=bytearray(frame('SC2',[1,7,5,6,0,2,1,5,0,0,0,1,100]));bad[10]^=1
        with self.assertRaises(ValueError):decode_status(bytes(bad))

if __name__=='__main__':unittest.main()
