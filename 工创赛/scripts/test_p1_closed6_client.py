import unittest
from p1_closed6_serial import Closed6Controller
from p1_serial_motion import command
from test_p1_perimeter_client import idle

class Closed6Tests(unittest.TestCase):
 def terminal(self,**kw):
  r=idle(seq=2,phase=3,trip=0,backend_result=1,backend_cause=0,stop_mask=15,vm_request=0,pe2_active=0,sequence_state=3,sequence_index=5,sequence_completed=6,sequence_count=6,action_kind=7,base_rpm=51,feedback_limit_rpm=80,debug_plan_id=0,distance_mode=1,distance_reached=1,correction_rpm=0);r.update(kw);return r
 def ctl(self):
  c=Closed6Controller('run-once',3,0);c.start_sent=True;c.last_seq=1;return c
 def test_complete_turn(self):
  c=self.ctl();self.assertEqual(c.consume(self.terminal(),1),[]);self.assertEqual(c.status,'software_complete')
 def test_incomplete_terminal(self):
  c=self.ctl();self.assertEqual(c.consume(self.terminal(sequence_completed=2),1),[command('STOP')]);self.assertNotEqual(c.status,'software_complete')
 def test_wrong_count(self):
  c=self.ctl();self.assertEqual(c.consume(self.terminal(sequence_count=12),1),[command('STOP')])
 def test_wrong_direction(self):
  c=self.ctl();self.assertEqual(c.consume(self.terminal(phase=2,sequence_index=2,action_kind=3,base_rpm=27,feedback_limit_rpm=40,distance_mode=0,correction_rpm=5),1),[command('STOP')])
 def test_wrong_distance(self):
  c=self.ctl();r=self.terminal(phase=2,sequence_index=0,action_kind=4,base_rpm=51,feedback_limit_rpm=80,backend_active=1,distance_mode=1,distance_target_milli_mm=1800000);self.assertEqual(c.consume(r,1),[command('STOP')])
 def test_monitor(self):
  c=Closed6Controller();self.assertEqual(c.consume(self.terminal(),1),[]);self.assertFalse(c.start_sent)
 def test_ccw_sign(self):
  c=self.ctl();r=self.terminal(phase=2,sequence_index=3,action_kind=3,base_rpm=27,feedback_limit_rpm=40,distance_mode=0,correction_rpm=5);self.assertTrue(c._running_shape(r));r['correction_rpm']=-5;self.assertFalse(c._running_shape(r))
 def test_final_distance_required(self):
  c=self.ctl();self.assertEqual(c.consume(self.terminal(distance_reached=0),1),[command('STOP')]);self.assertNotEqual(c.status,'software_complete')
if __name__=='__main__':unittest.main()
