import copy,json,sys,unittest
from pathlib import Path
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'src'))
from smart_carrier.vision_manipulation.stopped_frame_trigger import StoppedFrameTrigger
CFG=dict(motion=dict(near_stop_speed_px_s=25.,minimum_frame_interval_s=.02,maximum_frame_interval_s=.25),
         detection=dict(slot_interior_margin_px=5),
         regions={name:dict(x=x,y=0,width=390,height=390) for name,x in [('left',0),('right',400),('down',800)]})
def detection(x=100,extra=False):
    r=[dict(color_id=2,center_x_px=x,center_y_px=100)]
    if extra:r.append(dict(color_id=1,center_x_px=500,center_y_px=100))
    return dict(calibrated=True,regions=r)
class Tests(unittest.TestCase):
    def setUp(self):self.g=StoppedFrameTrigger(copy.deepcopy(CFG))
    def feed(self,d,t,remaining=(2,1,5)):return self.g.update(d,t,2,remaining)
    def test_matlab_vectors(self):
        proof=json.loads((R/'evidence/integration/2026-11-15/stopped3-grip170-release200/matlab-result.json').read_text())
        for row in proof['vectors']:
            self.g.reset()
            actual=[self.feed(detection(x),t) is not None for t,x in zip(row['time'],row['x'])]
            self.assertEqual(actual,row['opened'])
    def test_no_commit_until_third_frame(self):
        self.assertIsNone(self.feed(detection(),0));self.assertIsNone(self.feed(detection(),.1))
        result=self.feed(detection(),.2)
        self.assertEqual(result['confirmation_frames'],3);self.assertTrue(result['stopped'])
        self.assertEqual(result['start_delay_ms'],[0,1,1])
    def test_motion_in_same_roi_is_rejected(self):
        for i in range(15):self.assertIsNone(self.feed(detection(100+6*i),i*.1))
    def test_other_visible_material_moving_is_rejected(self):
        for i in range(10):
            d=detection(extra=True);d['regions'][1]['center_x_px']+=6*i
            self.assertIsNone(self.feed(d,i*.1))
    def test_invalid_detection_restarts_three(self):
        for kind in ('missing','uncalibrated','duplicate','nan'):
            self.g.reset();self.feed(detection(),0);self.feed(detection(),.1)
            bad=detection()
            if kind=='missing':bad['regions']=[]
            elif kind=='uncalibrated':bad['calibrated']=False
            elif kind=='duplicate':bad['regions']*=2
            else:bad['regions'][0]['center_x_px']=float('nan')
            self.assertIsNone(self.feed(bad,.2))
            self.assertIsNone(self.feed(detection(),.3));self.assertIsNone(self.feed(detection(),.4))
            self.assertIsNotNone(self.feed(detection(),.5))
    def test_new_remaining_and_pose_reset(self):
        self.feed(detection(),0);self.feed(detection(),.1)
        self.assertIsNone(self.feed(detection(),.2,(2,5)))
        self.assertIsNone(self.feed(detection(),.3,(2,5)))
        self.g.pause();self.assertEqual(self.g.diagnostic()['stable_frames'],0)
        self.assertIsNone(self.feed(detection(),.4,(2,5)))
    def test_fast_and_duplicate_samples_do_not_supply_three_frames(self):
        for t in (0,.001,.002):self.assertIsNone(self.feed(detection(),t))
        self.assertEqual(self.g.diagnostic()['stable_frames'],1)
        self.assertIsNone(self.feed(detection(),0));self.assertIsNone(self.feed(detection(),.1))
        self.assertIsNotNone(self.feed(detection(),.2))
    def test_gap_discards_old_confirmations(self):
        self.feed(detection(),0);self.feed(detection(),.1)
        self.assertIsNone(self.feed(detection(),1));self.assertIsNone(self.feed(detection(),1.1))
        self.assertIsNotNone(self.feed(detection(),1.2))
    def test_black_only_remaining_uses_same_stop_requirement(self):
        d=detection();d['regions'][0]['color_id']=5
        self.assertIsNone(self.g.update(d,0,5,(5,)))
        self.assertIsNone(self.g.update(d,.1,5,(5,)))
        self.assertIsNotNone(self.g.update(d,.2,5,(5,)))
if __name__=='__main__':unittest.main()
