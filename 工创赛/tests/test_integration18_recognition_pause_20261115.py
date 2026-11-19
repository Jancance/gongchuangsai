import importlib.util
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'src'),str(ROOT/'tests')]
import source_latched_trial_20261115 as trial

spec=importlib.util.spec_from_file_location('latched_stream',ROOT/'output/latched_material_stream_20261115.py')
stream=importlib.util.module_from_spec(spec);spec.loader.exec_module(stream)


class RecognitionPauseTests(unittest.TestCase):
    def test_detector_not_called_for_motion_alignment_or_missing_control(self):
        class Detector:
            def __init__(self):self.calls=0
            def detect(self,frame,timestamp):self.calls+=1;return {'slot_colors':[5,2,1]}
        detector=Detector()
        for control in [None,{'state':'MOVING'},{'state':'ALIGN'},{'state':'FAULT'}]:
            detection,value=stream.material_observation(detector,object(),control=control,now=1,to_json=lambda x:x)
            self.assertIsNone(detection);self.assertEqual(value['state'],'PAUSED');self.assertEqual(value['material_count'],0)
        self.assertEqual(detector.calls,0)
        for _ in range(2):
            detection,value=stream.material_observation(detector,object(),control={'state':'OBSERVE'},now=2,to_json=lambda x:x)
            self.assertEqual(value['slot_colors'],[5,2,1]);self.assertIsNotNone(detection)
        self.assertEqual(detector.calls,2)
        stream.material_observation(detector,object(),control=None,now=3,to_json=lambda x:x)
        self.assertEqual(detector.calls,2)

    def test_pause_sends_without_waiting_for_camera_ack(self):
        class Socket:
            def __init__(self):self.sent=[]
            def sendall(self,data):self.sent.append(json.loads(data))
            def recv(self,*args):raise AssertionError('no receive or acknowledgement wait allowed')
        report={'camera_link_events':[]};v=trial.Vision(18887,'test',report);socket=Socket()
        v.peer=SimpleNamespace(socket=socket)
        v.inactive(7,[2,1,5])
        self.assertEqual(socket.sent[0]['state'],'MOVING')
        self.assertEqual(socket.sent[0]['pose_sequence'],7)
        self.assertEqual(len(report['recognition_pause_requests']),1)

    def test_three_piece_order_is_observe_pause_commit_complete_then_observe(self):
        from test_low_pick_board_20261114 import state
        events=[];shared={};clock=[1.0]
        class Session:
            target=None
            def __enter__(self):return self
            def __exit__(self,*args):pass
        class Board:
            verified_flash_bytes=1
            _healthy=staticmethod(lambda v:True)
            def __init__(self,*args):
                shared['board']=self;self.value=state();self.running=False;self.polls=0
                self.value.update(g_chassis_auto_cause=0,g_source_body_step_reason=0,g_low_done_sequence=0,
                    g_low_accepted_sequence=0,g_low_release_wait_ms=50,g_low_return_overlap_axes=7)
            def snapshot(self):
                if self.running:
                    self.polls+=1
                    if self.polls>=2:
                        self.running=False;self.value.update(g_low_state=2,g_low_phase=10,
                            g_low_done_sequence=self.value['g_low_accepted_sequence'],g_low_clear_valid=1,
                            g_low_actual_rise_mm=66)
                        events.append(('complete',self.value['g_low_done_sequence']))
                return dict(self.value)
            def start_from_observation(self,goal,value):
                self.assert_pause=events[-1]==('pause',goal.sequence)
                if not self.assert_pause:raise AssertionError('pause must precede MCU start')
                events.append(('start',goal.sequence));self.running=True;self.polls=0
                self.value.update(g_low_state=1,g_low_phase=2,g_low_request=0,g_low_accepted_sequence=goal.sequence)
                return True
            def motion_evidence(self):return [{'axes':6},{'axes':7}]
            def cancel(self):raise AssertionError('no fault/cancel expected')
        class Vision:
            def __init__(self,*args):pass
            def exchange(self,mode,pose,seq,remaining):
                if shared['board'].running:raise AssertionError('no recognition while transaction active')
                events.append(('observe',seq+1))
                return dict(pose_active=True,stationary=True,pose_id=pose,pose_sequence=seq,
                    frame_after_pose_ack=True,age_s=.01,colors=[c if c in remaining else 0 for c in [2,1,5]])
            def inactive(self,seq,remaining):events.append(('pause',seq))
            def close(self):pass
        def position(board,*args):
            if board.running:raise AssertionError('must finish and return before observing')
        with tempfile.TemporaryDirectory() as temp,patch.dict(sys.modules,{
            'source_body_probe_20261111':SimpleNamespace(attach=lambda:Session()),
            'gripper210_attach_read_20261105':SimpleNamespace(load_image=lambda:({},[(0x8000000,b'x')]))}), \
            patch.object(trial,'E',Path(temp)),patch.object(trial,'manifest',return_value=({'axf':'fixture','sha256':'fixture'},[(195,162),(164,160),(180,0)],{})), \
            patch.object(trial,'CommittedPickBoard',Board),patch.object(trial,'Vision',Vision), \
            patch.object(trial,'position_body',side_effect=position),patch.object(trial.time,'monotonic',side_effect=lambda:clock[0]), \
            patch.object(trial.time,'sleep',side_effect=lambda x:clock.__setitem__(0,clock[0]+x)):
            trial.run(SimpleNamespace(align_only=False,task_code='215+213+512+321',port=18887))
            result=json.loads(next(Path(temp).glob('committed-*.json')).read_text())
            self.assertTrue(result['complete']);self.assertFalse(result['formal_inventory_updated'])
        self.assertEqual(events,[(kind,i) for i in [1,2,3] for kind in ['observe','pause','start','complete']])


if __name__=='__main__':unittest.main()
