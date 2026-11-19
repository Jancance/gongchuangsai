import importlib.util
import json
import math
from pathlib import Path
import unittest
import numpy as np
from smart_carrier.vision_manipulation.initial_rotation import InitialRotation

R=Path(__file__).resolve().parents[1]
E=R/'evidence/integration/2026-11-15/initial-rotation120'
CFG=json.loads((R/'output/source_direct_frame_vehicle_20261115/orbit-config.json').read_text())


def detection(degrees,center=None):
    theta=math.radians(degrees)
    p=np.asarray(CFG['center_px'] if center is None else center)+np.linalg.solve(CFG['whitening'],[math.cos(theta),math.sin(theta)])
    return dict(calibrated=True,regions=[dict(color_id=2,center_x_px=p[0],center_y_px=p[1])])


class RotationTests(unittest.TestCase):
    def setUp(self):
        self.assertTrue(json.loads((E/'matlab-result.json').read_text())['passed'])

    def test_both_directions_wrap_and_no_early_release(self):
        for direction in (-1,1):
            g=InitialRotation(CFG)
            for i in range(120):
                self.assertFalse(g.update(detection(175+direction*i),i*.1,2))
            self.assertTrue(g.update(detection(175+direction*121),12.1,2))
            g.pause()
            self.assertTrue(g.update({},100,2))  # Open stays open for later items.
            g.reset()
            self.assertFalse(g.update(detection(300),200,2))

    def test_stationary_and_reversal_do_not_accumulate_distance(self):
        g=InitialRotation(CFG)
        for i in range(1000):
            self.assertFalse(g.update(detection(.2*math.sin(i)),i*.1,2))
        g.reset()
        for i,a in enumerate([*range(119),*range(118,-1,-1)]):
            self.assertFalse(g.update(detection(a),i*.1,2))

    def test_unknown_interval_cannot_hide_reversal(self):
        for missing in ('gap','missing','duplicate','uncalibrated'):
            g=InitialRotation(CFG)
            for i in range(61):self.assertFalse(g.update(detection(i),i*.1,2))
            if missing=='gap':self.assertFalse(g.update(detection(0),10,2))
            else:
                bad=detection(60)
                if missing=='missing':bad['regions']=[]
                elif missing=='duplicate':bad['regions']*=2
                else:bad['calibrated']=False
                self.assertFalse(g.update(bad,6.1,2))
            for i in range(61):self.assertFalse(g.update(detection(i),10.1+i*.1,2))

    def test_current_rim_translation_does_not_count_camera_motion(self):
        g=InitialRotation(CFG)
        for i in range(60):g.update(detection(i),i*.1,2)
        delta=np.asarray([100,-50])
        g.register_rim(np.asarray(CFG['calibration_rim_center_px'])+delta)
        center=np.asarray(CFG['center_px'])+delta
        for i in range(120):self.assertFalse(g.update(detection(i,center),10+i*.1,2))
        self.assertTrue(g.update(detection(121,center),22.1,2))

    def test_recorded_replay_does_not_invent_progress_across_gaps(self):
        rows=json.loads((R/'evidence/integration/2026-11-15/source-rim-reacquire/field-no-pick/observations-summary.json').read_text())
        g=InitialRotation(CFG);first=None;opened=None;resets=0
        for row in rows:
            if row['state']['state']==2 and row.get('body'):
                g.register_rim(row['body']['center_px'])
            if row['state']['state']!=6:continue
            stamp=row['observation_host_timestamp'];first=stamp if first is None else first
            before=g.progress
            ready=g.update(dict(calibrated=True,regions=row['regions']),stamp,2)
            if before and g.progress==0:resets+=1
            if ready and opened is None:opened=stamp
            if not ready:self.assertLess(abs(math.degrees(g.progress)),120)
        report=dict(frames=len(rows),opened_after_s=None if opened is None else opened-first,
                    invalid_or_gap_resets=resets,physical_angle_truth_available=False)
        (E/'recorded-replay.json').write_text(json.dumps(report,indent=2)+'\n')


if __name__=='__main__':unittest.main()
