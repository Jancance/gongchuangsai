import importlib.util
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
from smart_carrier.vision_manipulation.source_body_registration import estimate,load_profile,next_existing_bounded_step
from smart_carrier.vision_manipulation.committed_pick_board import CommittedPickBoard
import source_commit_trial_20261114 as trial

def module(name,path):
    spec=importlib.util.spec_from_file_location(name,ROOT/path)
    mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);return mod
stream=module('commit_stream','output/commit_material_stream_20261114.py')

class CommitTests(unittest.TestCase):
    def test_rim_matches_matlab_on_three_real_frames(self):
        profile=load_profile(ROOT/'output/source_body_registration_20261114.json')
        evidence=ROOT/'evidence/integration/2026-11-14'
        expected=json.loads((evidence/'overlap200-commit/body-registration-matlab.json').read_text())
        for i,name in enumerate(expected['sessions']):
            frame=cv2.imdecode(np.fromfile(evidence/'top-pick-pb9'/name/'last-raw.png',dtype=np.uint8),cv2.IMREAD_COLOR)
            value=estimate(frame,profile)
            np.testing.assert_allclose(value['center_px'],expected['rim_centers_px'][i],atol=1e-5)
            self.assertFalse(value['motion_allowed'])
        self.assertTrue(value['within_tolerance']);self.assertIsNone(next_existing_bounded_step(value,profile))

    def test_alignment_can_observe_empty_disc_without_sending_material(self):
        control=dict(state='ALIGN',session_id='s',pose_id='p',pose_sequence=1,
                     remaining_colors=[],written_monotonic_s=5)
        self.assertIsNotNone(stream.validated_control(control,5.1))
        control['state']='OBSERVE';self.assertIsNone(stream.validated_control(control,5.1))

    def test_single_stable_decision_starts_without_second_confirmation(self):
        from test_low_pick_board_20261114 import state,Symbols,Target
        from smart_carrier.vision_manipulation.low_pick_board import LowPickTarget
        target=Target();board=object.__new__(CommittedPickBoard);board.target=target;board.symbols=Symbols();board._prepared=None
        value=state();value.update(g_low_done_sequence=3,g_low_accepted_sequence=3,g_low_wheel_hold_ready=1,
          g_source_align_chain_active=0)
        board.snapshot=lambda:value.copy();board.pose_ready=lambda _:True
        goal=LowPickTarget(4,1,2,1,163,159,1,1,'observer-commit-top65')
        observation=dict(pose_active=True,stationary=True,pose_id='observer-commit-top65',
          pose_sequence=3,frame_after_pose_ack=True,age_s=.01,colors=[5,2,1])
        self.assertTrue(board.start_from_observation(goal,observation))
        self.assertIn(('g_low_confirm_pose_sequence',4),target.writes)
        self.assertEqual(target.writes[-1],('g_low_request',1))
        self.assertNotIn(('g_low_request',2),target.writes)
        target.writes.clear();observation['stationary']=False
        self.assertFalse(board.start_from_observation(goal,observation));self.assertEqual(target.writes,[])

    def test_one_batch_camera_disappears_after_commit_controller_still_finishes(self):
        clock=[100.];shared={};started=[];alignment=[]
        class Session:
            target=None
            def __enter__(self):return self
            def __exit__(self,*args):pass
        class Board:
            _healthy=staticmethod(CommittedPickBoard._healthy)
            def __init__(self,*args):
                from test_low_pick_board_20261114 import state
                self.value=state();self.value.update(g_chassis_auto_cause=0,g_source_body_step_reason=0,
                  g_low_return_overlap_axes=7,g_low_release_wait_ms=50)
                self.verified_flash_bytes=1;self.ticks=0;shared['board']=self
            def snapshot(self):
                if self.value['g_low_state']==1:
                    self.ticks+=1
                    if self.ticks>=4:self.value.update(g_low_state=2,g_low_phase=10,
                      g_low_done_sequence=self.value['g_low_accepted_sequence'],g_low_clear_valid=1,
                      g_low_actual_rise_mm=69.75,g_low_release_wait_ms=50)
                return self.value.copy()
            def start_from_observation(self,goal,value):
                started.append(goal);self.ticks=0
                self.value.update(g_low_state=1,g_low_phase=2,g_low_accepted_sequence=goal.sequence)
                return True
            def motion_evidence(self):return [{'axes':6},{'axes':7}]
            def cancel(self):raise AssertionError('camera loss must not cancel')
        class Vision:
            def __init__(self,*args):pass
            def exchange(self,mode,pose,seq,remaining):
                self.assert_not_active=shared['board'].value['g_low_state']!=1
                if not self.assert_not_active:raise AssertionError('second visual check after commit')
                return dict(pose_active=True,stationary=True,pose_id=pose,pose_sequence=seq,
                  frame_after_pose_ack=True,age_s=.01,colors=[c if c in remaining else 0 for c in [2,1,5]])
            def inactive(self,*args):pass # peer may have disconnected; MCU continues
            def close(self):pass
        def position(board,vision,remaining,*args):alignment.append(tuple(remaining))
        fake_live=SimpleNamespace(load_image=lambda:({},[(0x08000000,b'x')]))
        with tempfile.TemporaryDirectory() as temp,patch.dict(sys.modules,{
          'source_body_probe_20261111':SimpleNamespace(attach=lambda:Session()),
          'gripper210_attach_read_20261105':fake_live}),patch.object(trial,'E',Path(temp)), \
          patch.object(trial,'manifest',return_value=({'axf':'fixture','sha256':'fixture'},[(194,165),(163,164),(180,0)],{})), \
          patch.object(trial,'CommittedPickBoard',Board),patch.object(trial,'Vision',Vision), \
          patch.object(trial,'position_body',side_effect=position), \
          patch.object(trial.time,'monotonic',side_effect=lambda:clock[0]), \
          patch.object(trial.time,'sleep',side_effect=lambda dt:clock.__setitem__(0,clock[0]+dt)):
            trial.run(SimpleNamespace(align_only=False,task_code='215+213+512+321',port=18887))
            report=json.loads(next(Path(temp).glob('committed-*.json')).read_text())
            self.assertTrue(report['complete']);self.assertTrue(report['physical_placement_pending'])
            self.assertFalse(report['formal_inventory_updated'])
        self.assertEqual([(g.color,g.slot) for g in started],[(2,1),(1,2),(5,3)])
        self.assertEqual(alignment,[(2,1,5),(1,5),(5,),()])

    def test_body_active_is_expected_only_in_alignment(self):
        from test_low_pick_board_20261114 import state
        v=state();v.update(g_chassis_auto_active=1,g_chassis_auto_cause=0,g_source_body_step_reason=0)
        board=SimpleNamespace(snapshot=lambda:v,_healthy=CommittedPickBoard._healthy)
        self.assertEqual(trial.checked_state(board,allow_body=True),v)
        with self.assertRaises(trial.ControllerFault):trial.checked_state(board)

if __name__=='__main__':unittest.main()
