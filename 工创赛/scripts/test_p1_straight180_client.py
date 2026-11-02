import unittest
from test_p1_perimeter_client import idle
from p1_sync_straight180_serial import Straight180Controller as Sync
from p1_sequential_straight180_serial import Straight180Controller as Sequential
class StraightTests(unittest.TestCase):
 def record(self,**kw):
  r=idle(seq=2,phase=3,trip=0,backend_result=1,backend_cause=0,stop_mask=15,vm_request=0,pe2_active=0,
   sequence_state=3,sequence_index=0,sequence_completed=1,sequence_count=1,action_kind=4,base_rpm=51,
   feedback_limit_rpm=80,debug_plan_id=0,distance_mode=1,distance_reached=1,distance_target_milli_mm=1800000,correction_rpm=.1)
  r.update(kw);return r
 def test_completion_both_protocol_policies(self):
  for cls in [Sync,Sequential]:
   c=cls('run-once',3,0);c.start_sent=True;c.last_seq=1
   self.assertEqual(c.consume(self.record(),1),[]);self.assertEqual(c.status,'software_complete')
 def test_other_distance_direction_or_route_stops(self):
  for cls in [Sync,Sequential]:
   for bad in [dict(sequence_count=8),dict(sequence_index=1),dict(action_kind=7),dict(distance_target_milli_mm=2100000),dict(distance_reached=0),dict(distance_mode=0),dict(vm_request=1),dict(stop_mask=7),dict(correction_rpm=2.1),dict(feedback_limit_rpm=800)]:
    with self.subTest(cls=cls.__name__,bad=bad):
     c=cls('run-once',3,0);c.start_sent=True;c.last_seq=1
     self.assertTrue(c.consume(self.record(**bad),1));self.assertNotEqual(c.status,'software_complete')
 def test_monitor_and_early_start(self):
  for cls in [Sync,Sequential]:
   c=cls();self.assertEqual(c.consume(self.record(),1),[]);self.assertFalse(c.start_sent)
   self.assertTrue(c._running_shape(self.record(phase=2,backend_active=0,distance_mode=0)))
   with self.assertRaises(ValueError):cls('run-once',3,1)
if __name__=='__main__':unittest.main(verbosity=2)
