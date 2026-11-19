"""Exercise actual window/trigger and actual camera callback with mocked I/O."""
import ast,copy,json,math,sys,types,unittest
from pathlib import Path
from prepare_three_frame_warm_20261118 import PI,PIBASE,E
sys.path[:0]=[str(PI),str(PIBASE)]
from source_pickup_window import SourcePickupWindow
from stopped_frame_trigger import StoppedFrameTrigger
from observation_frame_epoch import ObservationFrameEpoch,camera_metadata
from source_alignment_settle import SourceAlignmentSettle
from mcu_cycle_protocol import frame as wire_frame,decode_status,observation,continuous_observation

CONFIG=dict(detection=dict(slot_interior_margin_px=0),motion=dict(near_stop_speed_px_s=2,minimum_frame_interval_s=.001,maximum_frame_interval_s=.25),regions={n:dict(x=x,y=-200,width=120,height=400) for n,x in [('left',-60),('right',200),('down',400)]})
ORBIT=dict(center_px=[0,0],calibration_rim_center_px=[0,0],whitening=[[1,0],[0,1]],maximum_frame_interval_s=.25)


def status(state=2,pose=1,item=0,run=1):
    return dict(run=run,state=state,pose=pose,item=item,colors=[2,1,5],prep=5)


def detection(color=2,x=0,y=0):
    return dict(calibrated=True,regions=[dict(color_id=color,center_x_px=x,center_y_px=y)],slot_colors=[color,0,0])


class WindowTests(unittest.TestCase):
    def setUp(self):
        self.w=SourcePickupWindow(StoppedFrameTrigger(CONFIG,3),ORBIT);self.seq=0;self.t=1.
        self.w.receive(status(),self.t);self.w.bind_rim([0,0])

    def take(self,d=None,gen=1):
        self.seq+=1;self.t+=.04
        ok=self.w.frame(dict(generation=gen,sequence=self.seq,completed=self.t),self.t)
        return self.w.observe(d or detection(),self.t,2,[2,1,5]) if ok else None

    def witness(self):
        self.take(detection(x=100,y=0))
        for a in [0,30,60,90,125]:self.take(detection(x=100*math.cos(math.radians(a)),y=100*math.sin(math.radians(a))))

    def test_first_requires_real120_then_three_stopped(self):
        for _ in range(8):self.assertIsNone(self.take())
        self.assertFalse(self.w.rotation.opened)
        self.witness();self.assertTrue(self.w.rotation.opened)
        fixed=detection(x=100*math.cos(math.radians(125)),y=100*math.sin(math.radians(125)))
        self.assertIsNone(self.take(fixed));candidate=self.take(fixed)
        self.assertIsNotNone(candidate);self.assertEqual(candidate['confirmation_frames'],3)

    def test_alignment_to_beep_carries_history(self):
        self.witness();d=detection(x=-57,y=82)
        for _ in range(3):self.take(d)
        self.w.receive(status(5,2),self.t);n=len(self.w.trigger.history)
        self.w.receive(status(6,3),self.t)
        self.assertEqual(n,3);self.assertIsNotNone(self.take(d))

    def test_later_already_stopped_needs_no_rotation(self):
        self.w.receive(status(7,2),self.t);self.w.receive(status(6,3,1),self.t)
        # One pre-existing fence read is discarded; exactly3 accepted fresh reads.
        self.assertIsNone(self.take());self.assertIsNone(self.take());self.assertIsNone(self.take())
        self.assertIsNotNone(self.take());self.assertFalse(self.w.rotation.opened)

    def test_duplicate_cannot_make_three(self):
        self.w.receive(status(6,2,1),self.t);self.take();self.take()
        for _ in range(9):self.assertFalse(self.w.frame(dict(generation=1,sequence=self.seq,completed=self.t),self.t))
        self.assertEqual(len(self.w.trigger.history),1)

    def test_camera_and_chassis_move_reset_stable_only(self):
        self.witness();self.assertTrue(self.w.rotation.opened)
        self.w.receive(status(3,2),self.t);self.assertEqual(len(self.w.trigger.history),0)
        self.w.receive(status(2,3),self.t);self.w.bind_rim([0,0]);self.take();self.take()
        self.assertEqual(len(self.w.trigger.history),1)
        self.take(gen=2);self.assertEqual(len(self.w.trigger.history),0)

    def test_measured_segments_no_invisible_gap(self):
        self.take(detection(x=100,y=0))
        for a in (0,30,60):self.take(detection(x=100*math.cos(math.radians(a)),y=100*math.sin(math.radians(a))))
        before=self.w.rotation.progress
        self.w.receive(status(3,2),self.t);self.t+=.1
        self.w.receive(status(2,3),self.t);self.w.bind_rim([20,0])
        self.assertAlmostEqual(self.w.rotation.progress,before)
        self.take(detection(x=-80,y=0));self.take(detection(x=-80,y=0))
        self.assertAlmostEqual(self.w.rotation.progress,before)

    def test_new_run_and_expiry(self):
        self.witness();self.w.receive(status(2,5,run=2),self.t)
        self.assertFalse(self.w.rotation.opened);self.assertEqual(len(self.w.trigger.history),0)
        self.take();self.take();self.w.receive(status(2,5,run=2),self.t,True)
        self.assertEqual(len(self.w.trigger.history),0)


class CallbackTests(unittest.TestCase):
    def test_actual_callback_warms_but_only_sends_pick_in6(self):
        clock=types.SimpleNamespace(now=10.0);clock.monotonic=lambda:clock.now
        worker=types.SimpleNamespace(pid=123);camera=types.SimpleNamespace(_worker=worker,_sequence=0,last_read_completed_time=clock.now)
        actual_detection=detection(x=10,y=0)
        app=types.SimpleNamespace(camera=camera,material_pickup_detector=types.SimpleNamespace(config=CONFIG,detect=lambda *a,**kw:copy.deepcopy(actual_detection)),draw_material_pickup_detection=lambda *a:None)
        class Link:
            def __init__(self):self.pending=b'';self.sent=[]
            @property
            def in_waiting(self):return len(self.pending)
            def read(self,n):data=self.pending;self.pending=b'';return data
            def write(self,data):self.sent.append(data);return len(data)
        class Frame:
            def copy(self):return self
        class CV:
            FONT_HERSHEY_SIMPLEX=MARKER_CROSS=0
            def __getattr__(self,n):return (lambda *a,**kw:-1) if n=='waitKey' else (lambda *a,**kw:None)
        class Region:
            region='source'
            def receive(self,*a):return False
            def source_received(self):pass
            def capture_boundary(self,*a):pass
        class Rim:
            def reset(self):pass
            def estimate(self,f):return dict(center_px=[0,0],radius_px=100,error_norm_mm=0,right_forward_mm=[0,0],fit_rms_px=1,observation_valid=True)
        direct=StoppedFrameTrigger(CONFIG,3);window=SourcePickupWindow(direct,ORBIT);link=Link();f=Frame()
        env=dict(time=clock,json=json,link=link,status=None,received=0.,buffer=b'',context=None,frame_count=0,sequence=0,last_sent=0.,packets=0,window_ready=True,current_run=None,epoch_revision=0,last_observation_tx=None,
                 boot_handshake=types.SimpleNamespace(receive=lambda *a:False,diagnostic=lambda:{}),region_ui=Region(),frame_epoch=ObservationFrameEpoch(),pickup_window=window,direct=direct,alignment_settle=SourceAlignmentSettle(dict(settle_ms=300,required_frames=5,maximum_pair_distance_px=2,count_frames_after_dwell=True)),
                 camera_metadata=camera_metadata,decode_status=decode_status,args=types.SimpleNamespace(preview_only=False,observe_motion=False,study_stopped_pose=False,continuous_pick=True,session_dir=Path('.')),app=app,rim=Rim(),profile=dict(reference_center_px=[0,0]),
                 cv2=CV(),capture=types.SimpleNamespace(json_value=lambda x:x),rim_observation_valid=lambda *a:True,gate=types.SimpleNamespace(reset=lambda:None),predictor=None,window='test',STATES={},compose=lambda *a,**kw:f,observation=observation,continuous_observation=continuous_observation)
        tree=ast.parse((PI/'source_fast_cycle_view_20261115.py').read_text(encoding='utf-8'))
        main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
        process=copy.deepcopy(next(n for n in main.body if isinstance(n,ast.FunctionDef) and n.name=='process'))
        class Scope(ast.NodeTransformer):
            def visit_Nonlocal(self,node):return ast.copy_location(ast.Global(names=node.names),node)
        process=Scope().visit(process);module=ast.fix_missing_locations(ast.Module(body=[process],type_ignores=[]));exec(compile(module,'actual-camera-callback','exec'),env)
        def run(state=2,pose=2,item=0,announce=True):
            clock.now+=.04;camera._sequence+=1;camera.last_read_completed_time=clock.now-.001
            if announce:link.pending=wire_frame('SC2',[1,state,pose,5,item,2,1,5,0,0,0,0,0])
            return env['process'](f,'M')[0]
        for a in [0,30,60,90,125]:
            actual_detection.update(detection(x=50*math.cos(math.radians(a)),y=50*math.sin(math.radians(a))))
            run()
        # The first frame/fence carry no angle; give another measured full sweep.
        for a in [155,185,215,245,275]:
            actual_detection.update(detection(x=50*math.cos(math.radians(a)),y=50*math.sin(math.radians(a))))
            run()
        actual_detection.update(detection(x=10,y=0))
        for _ in range(8):record=run()
        self.assertTrue(window.rotation.opened);self.assertEqual(len(direct.history),3)
        self.assertTrue(all(not s.startswith(b'OC2') for s in link.sent))
        run(5,3);before=len(link.sent);record=run(6,4)
        self.assertEqual(len(link.sent),before+1);self.assertTrue(link.sent[-1].startswith(b'OC2,'))
        self.assertEqual(record['stop_confirmation']['required_frames'],3)
        env['args'].preview_only=True;before=len(link.sent);run(6,4)
        self.assertEqual(len(link.sent),before);env['args'].preview_only=False
        run(7,5);actual_detection.update(detection(color=1))
        before=len(link.sent)
        for k in range(5):run(6,6,1)
        self.assertTrue(any(s.startswith(b'OC2,') for s in link.sent[before:]))
        self.assertFalse(window.rotation_diagnostic()['rotation_gate_enabled'])


if __name__=='__main__':
    suite=unittest.defaultTestLoader.loadTestsFromModule(sys.modules[__name__]);result=unittest.TextTestRunner(verbosity=2).run(suite)
    (E/'python-checks.json').write_text(json.dumps(dict(passed=result.wasSuccessful(),tests=result.testsRun,actual_camera_callback=True,hardware_access=False),indent=2)+'\n')
    raise SystemExit(0 if result.wasSuccessful() else 1)
