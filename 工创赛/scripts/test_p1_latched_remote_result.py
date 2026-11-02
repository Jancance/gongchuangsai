import unittest,json
from pathlib import Path
import p1_fine_serial_motion as fine
import p1_sync_serial_motion as sync
from p1_sequential_straight210_serial import Straight210Controller as Fine
from p1_sync_straight210_serial import Straight210Controller as Sync
from test_p1_perimeter_client import idle
class LatchedResultTests(unittest.TestCase):
 def plan(self,cls):
  c=cls('run-once',3,0)
  for i in range(1,4):out=c.consume(idle(seq=i,tick_ms=i*100,sequence_count=1,remote_result=4,remote_rejected=2),i*.1)
  self.assertEqual(len(out),1);self.assertIn(b'PLAN0',out[0]);return c
 def test_latched_result_plan_accepted_and_reobservation(self):
  for cls in [Fine,Sync]:
   c=self.plan(cls);self.assertEqual(c.consume(idle(seq=4,tick_ms=400,sequence_count=1,remote_result=4,remote_rejected=2,remote_accepted=1),.4),[]);self.assertIsNone(c.status);self.assertTrue(c.plan_selected);self.assertFalse(c.start_sent)
   for i in range(5,8):out=c.consume(idle(seq=i,tick_ms=i*100,sequence_count=1,remote_result=4,remote_rejected=2,remote_accepted=1),i*.1)
   self.assertIn(b'ARM',out[0]);self.assertFalse(c.start_sent)
 def test_new_rejection_even_if_result_overwritten_stops(self):
  for cls in [Fine,Sync]:
   for result in [1,4,5,6]:
    c=self.plan(cls);out=c.consume(idle(seq=4,tick_ms=400,sequence_count=1,remote_result=result,remote_rejected=3,remote_accepted=1),.4);self.assertIn(b'STOP',out[0]);self.assertEqual(c.status,'remote_rejected_or_expired');self.assertFalse(c.start_sent)
 def test_expiry_and_timeout_still_stop(self):
  for cls in [Fine,Sync]:
   for result in [7,8]:
    c=self.plan(cls);out=c.consume(idle(seq=4,tick_ms=400,sequence_count=1,remote_result=result,remote_rejected=2),.4);self.assertIn(b'STOP',out[0])
   c=self.plan(cls);self.assertIn(b'STOP',c.tick(1.0)[0]);self.assertEqual(c.status,'telemetry_timeout')
 def test_actual_third_attempt_log_plan_ack_no_false_stop(self):
  root=Path(__file__).resolve().parents[2];p=max((root/'工创赛/evidence/p1-foundation/2026-10-31/heading_p100/vehicle/third_once').glob('*.jsonl'))
  data=[json.loads(l) for l in p.read_text(encoding='utf8').splitlines()];c=Fine('run-once',3,0);commands=[]
  from datetime import datetime
  start=None
  for r in data:
   if r.get('direction')!='RX' or 'yaw_mdeg' not in r:continue
   t=datetime.fromisoformat(r['host_utc']).timestamp();start=t if start is None else start;commands+=c.consume(r,t-start)
   if r['remote_accepted']==1:break
  self.assertTrue(c.plan_selected);self.assertIsNone(c.status);self.assertEqual(len(commands),1);self.assertIn(b'PLAN0',commands[0]);self.assertFalse(c.start_sent)
if __name__=='__main__':unittest.main(verbosity=2)
