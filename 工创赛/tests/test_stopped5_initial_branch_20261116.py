import copy
import ast
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import unittest

R = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(R / 'src'))
from smart_carrier.vision_manipulation.initial_observation_branch import InitialObservationRotation
from smart_carrier.vision_manipulation.stopped_frame_trigger import StoppedFrameTrigger

CFG = dict(motion=dict(near_stop_speed_px_s=25., moving_speed_px_s=30., required_moving_frames=2,
                      minimum_frame_interval_s=.02, maximum_frame_interval_s=.25),
           detection=dict(slot_interior_margin_px=5),
           regions={name:dict(x=x,y=0,width=390,height=390)
                    for name,x in [('left',0),('right',400),('down',800)]})
ORBIT = dict(center_px=[0,0],calibration_rim_center_px=[0,0],whitening=[[1,0],[0,1]],
             maximum_frame_interval_s=.25)


def detection(x=100,y=100,color=2):
    return dict(calibrated=True,regions=[dict(color_id=color,center_x_px=x,center_y_px=y)])


class Tests(unittest.TestCase):
    def setUp(self):
        self.branch = InitialObservationRotation(ORBIT, copy.deepcopy(CFG))
        self.stop = StoppedFrameTrigger(copy.deepcopy(CFG), required_frames=5)

    def feed(self,d,t):
        ready = self.branch.update(d,t,2,(2,1,5))
        result = self.stop.update(d,t,2,(2,1,5))
        return result if ready else None

    def moving(self):
        for t,x in ((0,100),(.1,110),(.2,120)):
            self.assertIsNone(self.feed(detection(x),t))
        self.assertEqual(self.branch.branch,'initial_moving')

    def test_initial_moving_then_stop_without120(self):
        self.moving()
        for t in (.3,.4,.5):self.assertIsNone(self.feed(detection(120),t))
        result = self.feed(detection(120),.6)
        self.assertEqual(result['confirmation_frames'],5)
        self.assertLess(abs(self.branch.rotation.progress),math.radians(120))

    def test_initial_static_must_wait120_then_five_stopped(self):
        for i in range(6):self.assertIsNone(self.feed(detection(),i*.1))
        self.assertEqual(self.branch.branch,'initial_stopped')
        for i,angle in enumerate((65,85,105,125,145,165,185),6):
            a=math.radians(angle)
            self.assertIsNone(self.feed(detection(140*math.cos(a),140*math.sin(a)),i*.1))
        self.assertTrue(self.branch.rotation.opened)
        self.assertEqual(self.branch.branch,'initial_stopped')
        # Bring the same phase into a legal ROI and establish a fresh five samples.
        for i in range(4):self.assertIsNone(self.feed(detection(100,100),1.3+i*.1))
        self.assertIsNotNone(self.feed(detection(),1.7))

    def test_ambiguous25to30_cannot_lock(self):
        for i in range(12):self.feed(detection(100+2.8*i),i*.1)
        self.assertIsNone(self.branch.branch)

    def test_other_visible_color_classifies_moving(self):
        for i in range(3):
            d=detection();d['regions'].append(dict(color_id=1,center_x_px=500+10*i,center_y_px=100))
            self.feed(d,i*.1)
        self.assertEqual(self.branch.branch,'initial_moving')

    def test_invalid_observations_never_classify_static(self):
        for kind in ('empty','duplicate','nan','uncalibrated','unknown'):
            self.setUp()
            for i in range(4):self.feed(detection(),i*.1)
            bad=detection()
            if kind=='empty':bad['regions']=[]
            if kind=='duplicate':bad['regions']*=2
            if kind=='nan':bad['regions'][0]['center_x_px']=float('nan')
            if kind=='uncalibrated':bad['calibrated']=False
            if kind=='unknown':bad['regions'][0]['color_id']=9
            self.assertIsNone(self.feed(bad,.4));self.assertIsNone(self.branch.branch)
            for i in range(4):self.assertIsNone(self.feed(detection(),.5+i*.1))
            self.assertIsNone(self.branch.branch)

    def test_gap_duplicate_fast_do_not_classify(self):
        for times in ((0,.1,.2,.3,1),(0,.1,.2,.3,.3),(0,.001,.002,.003,.004)):
            self.setUp()
            for t in times:self.feed(detection(),t)
            self.assertIsNone(self.branch.branch)

    def test_moving_confirmation_is_consecutive(self):
        for t,x in ((0,100),(.1,110),(.2,112.8),(.3,122.8)):
            self.feed(detection(x),t)
        self.assertIsNone(self.branch.branch)

    def test_classification_latches_once(self):
        self.moving()
        for i in range(12):self.feed(detection(120),.3+i*.1)
        self.assertEqual(self.branch.branch,'initial_moving')
        self.assertTrue(self.branch.update(dict(calibrated=False),2,2,(2,1,5)))
        self.assertIsNone(self.stop.update(dict(calibrated=False),2,2,(2,1,5)))

    def test_pose_run_reset_and_rim_registration(self):
        for action in ('pause','reset','register_rim'):
            self.setUp();self.moving()
            getattr(self.branch,action)(*[ [5,5] ] if action=='register_rim' else [])
            self.assertIsNone(self.branch.branch)
            self.assertIsNone(self.branch.previous)
            self.assertFalse(self.branch.rotation.opened)
        self.branch.rotation.opened=True
        self.branch.pause()
        self.assertFalse(self.branch.rotation.opened)

    def test_remaining_materials_all_require_five(self):
        for target,remaining in ((1,(1,5)),(5,(5,))):
            self.stop.reset()
            for i in range(4):self.assertIsNone(self.stop.update(detection(color=target),i*.1,target,remaining))
            self.assertEqual(self.stop.update(detection(color=target),.4,target,remaining)['confirmation_frames'],5)

    def test_short_circuit_cannot_prevent_phase_tracking(self):
        for i in range(5):self.branch.update(detection(),i*.1,2,(2,1,5))
        for i in range(1,9):
            a=math.radians(45+20*i)
            self.branch.update(detection(140*math.cos(a),140*math.sin(a)),.4+.1*i,2,(2,1,5))
        self.assertTrue(self.branch.rotation.opened)

    def test_visibility_changes_restart_classification(self):
        for i in range(4):self.feed(detection(),i*.1)
        d=detection();d['regions'].append(dict(color_id=1,center_x_px=500,center_y_px=100))
        self.feed(d,.4)
        self.assertIsNone(self.branch.branch)
        self.assertEqual(len(self.branch.stable),1)


class PackageTests(unittest.TestCase):
    def setUp(self):
        self.pi = R / 'output/source_stopped5_initial_branch_pi_20261116'

    def test_manifest_and_unchanged_baseline_files(self):
        manifest = json.loads((self.pi/'manifest.json').read_text())
        old = R/'output/source_stopped3_grip170_release200_pi_20261115'
        allowed = {'start.sh','start-trial.sh','source_fast_cycle_view_20261115.py',
                   'source_cycle_dashboard_20261115.py','stopped_frame_trigger.py'}
        for name,sha in manifest.items():
            self.assertEqual(hashlib.sha256((self.pi/name).read_bytes()).hexdigest(),sha)
            if (old/name).exists() and name not in allowed:
                self.assertEqual((self.pi/name).read_bytes(),(old/name).read_bytes())

    def test_actual_entry_branch_and_remaining_item_flow(self):
        tree = ast.parse((self.pi/'source_fast_cycle_view_20261115.py').read_text(encoding='utf-8'))
        node = next(n for n in ast.walk(tree) if isinstance(n,ast.If)
                    and ast.unparse(n.test)=="args.continuous_pick and status['state'] == 6 and valid")
        code = compile(ast.fix_missing_locations(ast.Module(body=node.body,type_ignores=[])),str(self.pi),'exec')
        env = dict(initial_rotation=self._new_branch(),direct=StoppedFrameTrigger(CFG,required_frames=5))
        env['status']=dict(item=0,colors=[2,1,5])
        for t,x in ((0,100),(.1,110),(.2,120),(.3,120),(.4,120),(.5,120),(.6,120)):
            env.update(value=detection(x),observed=t,prediction=None)
            exec(code,env)
            if t<.6:self.assertIsNone(env['prediction'])
        self.assertEqual(env['prediction']['confirmation_frames'],5)
        # Later materials bypass first-item classification, but never skip five stop frames.
        for item,target in ((1,1),(2,5)):
            env['initial_rotation'].pause();env['direct'].reset();env['status']['item']=item
            for i in range(5):
                env.update(value=detection(color=target),observed=1+i*.1,prediction=None)
                exec(code,env)
                if i<4:self.assertIsNone(env['prediction'])
            self.assertIsNotNone(env['prediction'])
            self.assertIsNone(env['initial_rotation'].branch)

    @staticmethod
    def _new_branch():return InitialObservationRotation(ORBIT,CFG)


if __name__=='__main__':unittest.main()
