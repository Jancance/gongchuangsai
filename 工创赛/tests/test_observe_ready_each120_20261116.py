import ast
import hashlib
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
import zipfile
import numpy as np

R=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(R/'src'))
from smart_carrier.vision_manipulation.observation_frame_epoch import ObservationFrameEpoch,camera_metadata
from smart_carrier.vision_manipulation.each_observation_rotation import EachObservationRotation
from smart_carrier.vision_manipulation.stopped_frame_trigger import StoppedFrameTrigger

CFG=dict(motion=dict(near_stop_speed_px_s=25.,moving_speed_px_s=30.,required_moving_frames=2,
                    minimum_frame_interval_s=.02,maximum_frame_interval_s=.25),
         detection=dict(slot_interior_margin_px=5),
         regions={n:dict(x=x,y=0,width=390,height=390) for n,x in [('left',0),('right',400),('down',800)]})
ORBIT=dict(center_px=[0,0],calibration_rim_center_px=[0,0],whitening=[[1,0],[0,1]],maximum_frame_interval_s=.25)
PI=R/'output/source_observe_ready_each120_pi_20261116'


def status(run=1,pose=3,item=0,state=6):
    return dict(run=run,pose=pose,item=item,state=state,colors=[2,1,5])


def detected(x=100,y=100,color=2):
    return dict(calibrated=True,regions=[dict(color_id=color,center_x_px=x,center_y_px=y)],slot_colors=[color,0,0])


def meta(seq,t,gen=1):return dict(sequence=seq,completed=t,generation=gen)


class EpochTests(unittest.TestCase):
    def setUp(self):self.e=ObservationFrameEpoch();self.e.receive(status(),10)
    def test_old_crossing_and_next_read(self):
        self.assertFalse(self.e.frame(meta(1,9.9),10))
        self.assertFalse(self.e.frame(meta(2,10.1),10.2))
        self.assertTrue(self.e.frame(meta(3,10.2),10.3))
    def test_heartbeat_never_resets_epoch(self):
        self.e.frame(meta(2,10.1),10.2);revision=self.e.revision
        self.e.receive(status(),10.25)
        self.assertEqual(self.e.revision,revision);self.assertEqual(self.e.epoch,10)
        self.assertTrue(self.e.frame(meta(3,10.3),10.4))
    def test_old_pose_run_do_not_revert_or_refresh(self):
        for s in (status(pose=2),status(run=0),status(state=5)):
            self.assertFalse(self.e.receive(s,20))
        self.assertEqual(self.e.latest,status());self.assertEqual(self.e.epoch,10)
    def test_each_ready_and_generation_invalidate_fence(self):
        self.e.frame(meta(1,10.1),10.2);self.assertTrue(self.e.frame(meta(2,10.2),10.3))
        old=self.e.revision;self.assertFalse(self.e.frame(meta(3,10.3,2),10.4))
        self.assertGreater(self.e.revision,old)
        self.assertTrue(self.e.frame(meta(4,10.4,2),10.5))
        self.e.receive(status(pose=5,item=1),11)
        self.assertFalse(self.e.frame(meta(5,11.1,2),11.2))
        self.assertTrue(self.e.frame(meta(6,11.2,2),11.3))
    def test_unknown_duplicate_and_future_metadata(self):
        for m in (None,meta(1,float('nan')),meta(1,12),meta(0,10.1)):
            self.assertFalse(self.e.frame(m,11))
        self.e.frame(meta(1,10.1),11)
        self.assertFalse(self.e.frame(meta(1,10.1),11))
        self.assertFalse(self.e.frame(meta(2,10.0),11))
    def test_camera_requires_worker_identity(self):
        self.assertIsNone(camera_metadata(SimpleNamespace(last_read_completed_time=10,_sequence=1)))
        c=SimpleNamespace(_worker=SimpleNamespace(pid=12),last_read_completed_time=10,_sequence=1)
        self.assertEqual(camera_metadata(c)['sequence'],1)
    def test_exit_ready_invalidates(self):
        self.e.receive(status(pose=4,state=7),11)
        self.assertFalse(self.e.frame(meta(3,11.1),11.2));self.assertIsNone(self.e.ready)
    def test_restart_requires_idle_then_new_preparation(self):
        idle=dict(run=0,pose=0,state=0,item=0,prep=0,colors=[2,1,5])
        self.assertTrue(self.e.receive(idle,10.1));self.assertIsNone(self.e.ready)
        self.assertFalse(self.e.receive(status(run=5,pose=30),10.2))
        self.assertFalse(self.e.receive(status(run=1,pose=1),10.3))
        prep=dict(run=1,pose=1,state=1,prep=0,item=0,colors=[2,1,5])
        self.assertTrue(self.e.receive(prep,10.4))
        self.assertTrue(self.e.receive(status(run=1,pose=2),10.5))
        self.assertEqual(self.e.epoch,10.5)


class EachRotationTests(unittest.TestCase):
    def test_three_visits_static_each_requires_later120(self):
        gate=EachObservationRotation(ORBIT,CFG)
        for color,remaining in ((2,[2,1,5]),(1,[1,5]),(5,[5])):
            gate.pause();stop=StoppedFrameTrigger(CFG,5)
            for i in range(5):
                self.assertFalse(gate.update(detected(color=color),i*.1,color,remaining))
            self.assertEqual(gate.branch,'initial_stopped');self.assertEqual(gate.rotation.progress,0)
            for i in range(1,8):
                angle=math.radians(45+i*20)
                gate.update(detected(140*math.cos(angle),140*math.sin(angle),color),.4+i*.1,color,remaining)
            self.assertTrue(gate.rotation.opened)
            for i in range(5):
                ready=gate.update(detected(color=color),1.2+i*.1,color,remaining)
                result=stop.update(detected(color=color),1.2+i*.1,color,remaining)
                self.assertEqual(ready and result is not None,i==4)
    def test_unknown_return_never_grants_moving_bypass(self):
        gate=EachObservationRotation(ORBIT,CFG)
        for i in range(3):gate.update(detected(100+10*i),i*.1,2,[2,1,5])
        self.assertEqual(gate.branch,'initial_moving')
        gate.pause()
        for i in range(5):self.assertFalse(gate.update(detected(color=1),i*.1,1,[1,5]))
        self.assertEqual(gate.branch,'initial_stopped')


class ProcessTests(unittest.TestCase):
    def setUp(self):
        tree=ast.parse((PI/'source_fast_cycle_view_20261115.py').read_text(encoding='utf-8'))
        proc=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='process')
        proc.body=[n for n in proc.body if not isinstance(n,ast.Nonlocal)]
        # Original closure variables become globals for actual process execution.
        assigned=['status','received','buffer','context','frame_count','sequence','last_sent','packets','window_ready','current_run','epoch_revision']
        proc.body.insert(0,ast.Global(names=assigned))
        self.now=10.;self.incoming=[];self.writes=[];self.detections=0
        self.camera=SimpleNamespace(_sequence=1,last_read_completed_time=9.9,_worker=SimpleNamespace(pid=1))
        def detect(frame,timestamp):self.detections+=1;return self.value
        detector=SimpleNamespace(detect=detect,config={})
        link=SimpleNamespace(in_waiting=0,read=lambda n:b''.join(self.incoming),write=lambda b:self.writes.append(b) or len(b))
        noop=lambda *a,**k:None
        cv=SimpleNamespace(putText=noop,imshow=noop,imwrite=noop,waitKey=lambda *a:0,FONT_HERSHEY_SIMPLEX=0)
        gate=SimpleNamespace(reset=noop,update=lambda *a,**k:False)
        self.env=dict(time=SimpleNamespace(monotonic=lambda:self.now),link=link,decode_status=lambda line:json.loads(line),
            json=json,app=SimpleNamespace(camera=self.camera,material_pickup_detector=detector,draw_material_pickup_detection=noop),
            frame_epoch=ObservationFrameEpoch(),camera_metadata=camera_metadata,epoch_revision=1,
            initial_rotation=EachObservationRotation(ORBIT,CFG),direct=StoppedFrameTrigger(CFG,5),gate=gate,
            predictor=None,rim=None,profile={},estimate=lambda *a:dict(center_px=[0,0],observation_valid=False),
            rim_observation_valid=lambda *a:False,capture=SimpleNamespace(json_value=lambda x:x),
            args=SimpleNamespace(study_stopped_pose=False,continuous_pick=True,observe_motion=False,preview_only=False,session_dir=R),
            cv2=cv,STATES={},compose=lambda *a,**k:None,window='test',status=None,received=0,buffer=b'',context=None,
            frame_count=0,sequence=0,last_sent=0,packets=0,current_run=None,window_ready=True,
            continuous_observation=lambda *a:b'OC2',observation=lambda *a:b'OC1')
        exec(compile(ast.fix_missing_locations(ast.Module(body=[proc],type_ignores=[])),'actual-process','exec'),self.env)
        self.value=detected()
    def feed(self,t,seq,stamp,s=None):
        self.now=t;self.camera._sequence=seq;self.camera.last_read_completed_time=stamp
        self.incoming[:]=[json.dumps(s).encode()+b'\n'] if s else []
        return self.env['process'](np.zeros((2,2,3),dtype=np.uint8),'M')[0]
    def test_actual_process_fence_and_every_item_static(self):
        seq=0
        for item in range(3):
            start=10+item*2;self.value=detected(color=[2,1,5][item]);s=status(pose=3+item*2,item=item)
            seq+=1;r=self.feed(start,seq,start-.01,s)
            self.assertFalse(r['recognition_enabled'])
            seq+=1;r=self.feed(start+.1,seq,start+.05,s)
            self.assertFalse(r['recognition_enabled'])
            for j in range(5):
                seq+=1;r=self.feed(start+.2+j*.1,seq,start+.15+j*.1,s)
            self.assertEqual(r['initial_rotation']['initial_branch'],'initial_stopped')
            self.assertEqual(r['stop_confirmation']['stable_frames'],5)
            self.assertEqual(r['measurement_packets_sent'],0)
        self.assertEqual(self.detections,15)
    def test_actual_process_worker_restart_unknown_old_status(self):
        s=status();self.feed(10,1,9.9,s);self.feed(10.1,2,10.05,s)
        self.feed(10.2,3,10.15,s);self.assertEqual(self.detections,1)
        self.camera._worker=SimpleNamespace(pid=2)
        r=self.feed(10.3,4,10.25,s);self.assertFalse(r['recognition_enabled'])
        self.feed(10.4,5,10.35,status(pose=2))
        self.assertEqual(self.env['status']['pose'],3)
        self.camera._worker=None
        r=self.feed(10.5,6,10.45,s);self.assertFalse(r['recognition_enabled'])
        self.assertEqual(r['prearrival_motion'],'UNKNOWN');self.assertFalse(r['premotion_exception_enabled'])
    def test_actual_process_no_metadata_time_fallback(self):
        self.camera._worker=None
        for j in range(7):r=self.feed(10+j*.1,j+1,None,status())
        self.assertEqual(self.detections,0);self.assertIsNone(r['observation_host_timestamp'])
    def test_actual_process_moving_then_five_new_stopped(self):
        s=status();self.feed(10,1,9.9,s);self.feed(10.1,2,10.05,s)
        for i,x in enumerate((100,110,120,120,120,120,120)):
            self.value=detected(x)
            r=self.feed(10.2+i*.1,3+i,10.15+i*.1,s)
            if i<6:self.assertEqual(r['measurement_packets_sent'],0)
        self.assertEqual(r['measurement_packets_sent'],1)
        self.assertEqual(r['stop_confirmation']['stable_frames'],5)
    def test_actual_process_quick_reboot_then_preparation(self):
        self.feed(10,1,9.9,status(run=5,pose=30))
        idle=dict(run=0,pose=0,state=0,item=0,prep=0,colors=[2,1,5])
        self.feed(10.1,2,10.05,idle);self.assertEqual(self.env['status']['run'],0)
        r=self.feed(10.2,3,10.15,status(run=5,pose=30));self.assertIsNone(r['frame_epoch']['ready'])
        prep=dict(run=1,pose=1,state=1,prep=0,item=0,colors=[2,1,5])
        self.feed(10.3,4,10.25,prep)
        r=self.feed(10.4,5,10.35,status(pose=2));self.assertEqual(r['frame_epoch']['ready'][1],2)
    def test_actual_process_120_boundary_discards_prior_stopped_five(self):
        s=status();self.feed(10,1,9.9,s);self.feed(10.1,2,10.05,s)
        seq=2;t=10.2
        def point(angle):
            a=math.radians(angle);return detected(200+100*math.cos(a),200+100*math.sin(a))
        # Orbit center translated to keep the entire circular path in one ROI.
        self.env['initial_rotation']=EachObservationRotation(dict(ORBIT,center_px=[200,200]),CFG)
        for angle in [0]*5+[20,40,60,80,100,119.9]+[119.9]*5:
            self.value=point(angle);seq+=1;self.feed(t,seq,t-.03,s);t+=.1
        self.assertEqual(self.env['direct'].diagnostic()['stable_frames'],5)
        self.assertFalse(self.env['initial_rotation'].rotation.opened)
        for i in range(5):
            self.value=point(120.1);seq+=1;r=self.feed(t,seq,t-.03,s);t+=.1
            self.assertEqual(r['measurement_packets_sent'],int(i==4))
            self.assertEqual(r['stop_confirmation']['stable_frames'],i+1)


class PackageTests(unittest.TestCase):
    def test_manifest_zip_and_original_intact(self):
        for package in (PI,R/'output/source_stopped5_initial_branch_pi_20261116'):
            manifest=json.loads((package/'manifest.json').read_text())
            for name,h in manifest.items():self.assertEqual(hashlib.sha256((package/name).read_bytes()).hexdigest(),h)
        with zipfile.ZipFile(PI.with_suffix('.zip')) as z:
            self.assertIsNone(z.testzip())
            for name in [*json.loads((PI/'manifest.json').read_text()),'manifest.json']:
                self.assertEqual(z.read(name),(PI/name).read_bytes())


if __name__=='__main__':unittest.main()
