"""Regression for the real rejected-confirm deadlock; simulated IO only."""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
def load(name,path):
    spec=importlib.util.spec_from_file_location(name,ROOT/path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module
trial=load('top_trial','scripts/source_top_trial_20261114.py')
stream=load('top_stream','output/top_material_stream_20261114.py')

class TwoFrameTests(unittest.TestCase):
    def test_two_frames_then_pose_reset_and_motion_rejection(self):
        for colors in [(2,1,5),(0,1,5),(0,0,5)]:
            gate=stream.RemainingStopped();remaining=tuple(c for c in colors if c)
            def detected(shift=0):
                return dict(calibrated=True,slot_colors=colors,regions=[
                  dict(color_id=c,center_x_px=i*300+shift,center_y_px=200) for i,c in enumerate(colors) if c])
            self.assertFalse(gate.update(detected(),0,remaining=remaining,pose_id='top',pose_sequence=1))
            self.assertTrue(gate.update(detected(),.04,remaining=remaining,pose_id='top',pose_sequence=1))
            self.assertFalse(gate.update(detected(),.08,remaining=remaining,pose_id='other',pose_sequence=2))
            self.assertFalse(gate.update(detected(20),.12,remaining=remaining,pose_id='other',pose_sequence=2))

    def test_rejected_first_write_keeps_vision_until_actual_descent_three_items(self):
        from test_low_pick_board_20261114 import state
        clock=[100.0];shared={};goals=[];attempts=[]
        class Session:
            target=None
            def __enter__(self):return self
            def __exit__(self,*args):pass
        class Board:
            _healthy=staticmethod(trial.LowPickBoard._healthy)
            def __init__(self,*args):
                self.value=state();self.verified_flash_bytes=1;self.attempt=0;shared['board']=self
            def snapshot(self):return self.value.copy()
            def prepare(self,goal):
                self.goal=goal;goals.append(goal);self.attempt=0
                self.value.update(g_low_state=1,g_low_phase=3,g_low_accepted_sequence=goal.sequence)
            def confirm(self,**kwargs):
                self.attempt+=1;attempts.append((self.goal.sequence,self.attempt))
                if self.attempt==1:self.value['g_low_rejected']+=1
                else:self.value.update(g_low_phase=4)
                return True  # A successful write can still be rejected by the MCU.
            def cancel(self):raise AssertionError('successful model must not cancel')
        class Peer:
            def __init__(self,*args):pass
            def exchange(self,control):
                board=shared['board']
                if control['state']=='MOVING' and board.attempt:
                    if board.value['g_low_phase']==3:raise AssertionError('vision disabled before acceptance')
                    board.value.update(g_low_state=2,g_low_phase=10,g_low_done_sequence=board.goal.sequence,
                      g_low_clear_valid=1,g_low_clear_ms=700,g_low_actual_rise_mm=66,g_low_release_ms=1000)
                colors=[c if c in control['remaining_colors'] else 0 for c in [2,1,5]]
                return dict(pose_active=control['state']!='MOVING',stationary=control['state']=='CONFIRM',
                  pose_id=control['pose_id'],pose_sequence=control['pose_sequence'],frame_after_pose_ack=True,
                  age_s=.01,pause_age_s=None,stop_sequence=0,colors=colors)
            def close(self):pass
        args=SimpleNamespace(focus_profile_verified=True,task_code='215+213+512+321',
          profile='config/task_materials_preliminary_2027.json',port=18887,start_item=0,completed_prefix=None)
        with tempfile.TemporaryDirectory() as temp,patch.dict(sys.modules,{
          'source_body_probe_20261111':SimpleNamespace(attach=lambda:Session()),
          'gripper210_attach_read_20261105':SimpleNamespace(load_image=lambda:({},[(0x08000000,b'fixture')]))}), \
          patch.object(trial,'E',Path(temp)),patch.object(trial,'manifest',return_value=(
            {'axf':'fixture.axf','sha256':'fixture'},[(194,163),(163,159),(180,0)])), \
          patch.object(trial,'LowPickBoard',Board),patch.object(trial,'CameraPeer',Peer), \
          patch.object(trial.time,'monotonic',side_effect=lambda:clock[0]), \
          patch.object(trial.time,'sleep',side_effect=lambda dt:clock.__setitem__(0,clock[0]+dt)):
            trial.run(args)
            report=json.loads(next(Path(temp).glob('trial-*.json')).read_text())
            self.assertTrue(report['complete']);self.assertTrue(report['physical_placement_pending'])
            self.assertFalse(report['formal_inventory_updated'])
        self.assertEqual([(g.color,g.slot) for g in goals],[(2,1),(1,2),(5,3)])
        self.assertEqual(attempts,[(1,1),(1,2),(2,1),(2,2),(3,1),(3,2)])

if __name__=='__main__':unittest.main()
