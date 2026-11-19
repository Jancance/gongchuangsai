"""Minimal inherited actual-window/callback tests plus real dashboard rendering."""
from pathlib import Path
import sys,json,subprocess
from prepare_pickup60_ui_20261118 import R,P,DEPS,E
def main():
 s=(R/'scripts/check_three_frame_warm_python_20261118.py').read_text(encoding='utf-8')
 s=s.replace('from prepare_three_frame_warm_20261118 import PI,PIBASE,E',
  'from pathlib import Path\nPI=Path('+repr(str(P))+')\nPIBASE=Path('+repr(str(DEPS))+')\nE=Path('+repr(str(E))+')')
 s=s.replace('test_first_requires_real120_then_three_stopped','test_first_requires_real60_then_three_stopped')
 s=s.replace('cv2=CV(),capture=', 'draw_pickup_geometry=lambda *a:None,cv2=CV(),capture=')
 begin=s.index('        for a in [0,30,60,90,125]:',s.index('class CallbackTests'))
 end=s.index('        actual_detection.update(detection(x=10,y=0))',begin)
 s=s[:begin]+'''        for a in [0,0,0,15,30,45,61]:
            actual_detection.update(detection(x=50*math.cos(math.radians(a)),y=50*math.sin(math.radians(a))))
            run()
        self.assertTrue(window.rotation.opened)
        self.assertGreaterEqual(abs(math.degrees(window.rotation.progress)),60)
        self.assertLess(abs(math.degrees(window.rotation.progress)),120)
'''+s[end:]
 pos=s.index("if __name__=='__main__':")
 extra=r'''
class AngleCompatibilityTests(unittest.TestCase):
    def test_signed60_boundary_and_legacy120(self):
        from initial_rotation import InitialRotation
        for sign in (1,-1):
            current=InitialRotation(ORBIT,required_deg=60);legacy=InitialRotation(ORBIT)
            for k,a in enumerate((0,20,40,59.9)):
                d=detection(x=100*math.cos(math.radians(sign*a)),y=100*math.sin(math.radians(sign*a)))
                self.assertFalse(current.update(d,1+k*.04,2));self.assertFalse(legacy.update(d,1+k*.04,2))
            a=sign*60.1;d=detection(x=100*math.cos(math.radians(a)),y=100*math.sin(math.radians(a)))
            self.assertTrue(current.update(d,1.16,2));self.assertFalse(legacy.update(d,1.16,2))
            self.assertEqual(current.diagnostic()['required_deg'],60)
            self.assertEqual(legacy.diagnostic()['required_deg'],120)
            for k,a in enumerate((80,100,120.1)):
                d=detection(x=100*math.cos(math.radians(sign*a)),y=100*math.sin(math.radians(sign*a)))
                legacy.update(d,1.20+k*.04,2)
            self.assertTrue(legacy.opened)
    def test_window_single_adjustable_parameter(self):
        a=SourcePickupWindow(StoppedFrameTrigger(CONFIG,3),ORBIT)
        b=SourcePickupWindow(StoppedFrameTrigger(CONFIG,3),ORBIT,first_rotation_deg=75)
        a.receive(status(),1);b.receive(status(),1)
        self.assertEqual(a.rotation_diagnostic()['required_deg'],60)
        self.assertEqual(b.rotation_diagnostic()['required_deg'],75)

class ActualRendererTests(unittest.TestCase):
    def test_actual_renderer_geometry_and_actual_policy(self):
        import cv2,numpy as np
        from source_cycle_dashboard_20261115 import compose,draw_pickup_geometry,pickup_text
        frame=np.full((1080,1920,3),35,dtype=np.uint8)
        cfg=json.loads((Path(__file__).resolve().parents[4]/'output/material_pickup_top65_20261114.json').read_text(encoding='utf-8'))
        d=dict(regions=[dict(color_id=2,center_x_px=550,center_y_px=160,bounding_box=[500,110,100,100]),dict(color_id=1,center_x_px=1300,center_y_px=180,bounding_box=[1250,130,100,100])],state='WAITING_FOR_MOTION',required_low_speed_frames=2,ready=False,max_speed_px_s=999)
        draw_pickup_geometry(frame,d,cfg)
        for label,state,item,angle,stable,ready in [('first_wait60',6,0,-45,3,False),('first_ready',6,0,61,3,True),('later_stopped',6,1,0,2,False),('executing',7,1,0,0,False),('complete',8,3,0,0,False)]:
            st=status(state,2,item);w=dict(stable_frames=stable,first_rotation=dict(rotation_gate_enabled=item==0,observed_net_deg=angle,required_deg=60 if item==0 else 0,ready=abs(angle)>=60))
            trig=dict(required_frames=3,near_stop_speed_px_s=25);pred={'region':0} if ready else None
            texts=pickup_text(st,w,trig,pred);self.assertNotIn('999',str(texts));self.assertNotIn('WAITING_FOR_MOTION',str(texts))
            if state==6 and item==0 and not ready:self.assertIn('真实转动',texts[1])
            if item==1 and state==6:self.assertIn('无需转角',texts[0])
            if state==7:self.assertIn('执行中',texts[0]);self.assertNotIn('0/3',str(texts))
            image=compose(frame.copy(),frame.copy(),st,active=True,initial_rotation=w['first_rotation'],pickup_window=w,trigger_info=trig,candidate=pred)
            self.assertEqual(image.shape,(400,960,3));ok,encoded=cv2.imencode('.png',image);self.assertTrue(ok);(E/('ui-'+label+'.png')).write_bytes(encoded.tobytes())
        expired=compose(frame,frame,status(7,2,1),active=False)
        ok,encoded=cv2.imencode('.png',expired);self.assertTrue(ok);(E/'ui-expired.png').write_bytes(encoded.tobytes())
'''
 # The generated test is under E, so use an explicit local fixture path.
 extra=extra.replace("(Path(__file__).resolve().parents[4]/'output/material_pickup_top65_20261114.json')",'Path('+repr(str(R/'output/material_pickup_top65_20261114.json'))+')')
 s=s[:pos]+extra+'\n'+s[pos:]
 test=E/'actual-window-callback-ui-tests.py';test.write_text(s,encoding='utf-8')
 z=subprocess.run([sys.executable,str(test)],cwd=R,capture_output=True,text=True,encoding='utf-8',errors='replace');(E/'python-test.log').write_text(z.stdout+z.stderr,encoding='utf-8');print((z.stdout+z.stderr).encode('ascii','backslashreplace').decode());assert z.returncode==0
if __name__=='__main__':main()
