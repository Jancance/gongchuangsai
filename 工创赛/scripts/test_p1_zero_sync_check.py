import unittest
from p1_zero_sync_check import review,ZeroSyncController
from test_p1_straight180_client import StraightTests
class ZeroTests(unittest.TestCase):
 def summary(self):
  last=StraightTests().record(phase=5,trip=1,sequence_state=5,sequence_completed=0,backend_result=2,backend_cause=9,backend_active=0)
  return dict(status='software_complete',batch_records=[dict(outcome=1,sync_result=1,sync_cache_mask=15,ack_mask=15,base_rpm=0,correction_rpm=0)],last=last,max_abs_rpm=[0,0,0,0],fault_frames=1,last_fault=dict(trip=1,source_valid=1,backend_fault_reason=0))
 def test_requires_zero_and_complete_single_trigger(self):
  s=self.summary();self.assertTrue(review(s)['passed']);self.assertFalse(review(s)['travel_completed'])
  for key,value in [('base_rpm',1),('correction_rpm',.1),('sync_cache_mask',7),('sync_result',0)]:
   s=self.summary();s['batch_records'][0][key]=value;self.assertFalse(review(s)['passed'])
  s=self.summary();s['max_abs_rpm'][0]=1;self.assertFalse(review(s)['passed'])
  s=self.summary();s['batch_records']=[];self.assertFalse(review(s)['passed'])
  s=self.summary();s['batch_records']*=2;self.assertFalse(review(s)['passed'])
  s=self.summary();s['last_fault']['trip']=2;self.assertFalse(review(s)['passed'])
 def test_expected_cancel_is_distinct_from_distance_success(self):
  c=ZeroSyncController('run-once',3,0);c.start_sent=True;c.last_seq=1
  self.assertEqual(c.consume(self.summary()['last'],1),[]);self.assertEqual(c.status,'software_complete')
  s=self.summary();s['last']['sequence_completed']=1;self.assertFalse(review(s)['passed'])
if __name__=='__main__':unittest.main(verbosity=2)
