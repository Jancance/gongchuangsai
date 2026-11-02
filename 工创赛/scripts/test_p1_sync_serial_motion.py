import importlib,json,math,unittest
from pathlib import Path
import p1_sync_serial_motion as sync
import p1_fine_serial_motion as fine
from test_p1_fine_serial_motion import frame
ROOT=Path(__file__).resolve().parents[2]
class SyncProtocolTests(unittest.TestCase):
 def batch(self):
  p=next((ROOT/'工创赛/evidence/p1-foundation/2026-10-31/heading_reference_vehicle/ccw_run_once').glob('*.summary.json'))
  r=json.loads(p.read_text(encoding='utf8'))['batch_records'][-1]
  end=r['end_ms'];r.update(sync_generation=7,sync_cache_mask=15,sync_trigger_ms=end+1,sync_reply_ms=end+2,sync_result=1,end_ms=end+2,correction_rpm=-7,pending_rpm=4)
  return r
 def parse(self,r):return sync.parse_batch_telemetry(frame('BT3',[r[k] for k in sync.BATCH_FIELDS]))
 def test_success_requires_trigger(self):
  r=self.batch();p=self.parse(r);self.assertEqual(p['sync_cache_mask'],15);self.assertEqual(p['correction_rpm'],-.7)
  for key,value in [('sync_result',0),('sync_result',5),('sync_cache_mask',7),('sync_generation',0),('sync_trigger_ms',r['reply_ms4']-1),('sync_reply_ms',r['sync_trigger_ms']+50)]:
   x=dict(r);x[key]=value
   with self.subTest(key=key,value=value),self.assertRaises(ValueError):self.parse(x)
 def test_interrupted_cache_is_not_completion(self):
  r=self.batch();r.update(outcome=2,sync_cache_mask=7,sync_result=0,sync_trigger_ms=0,sync_reply_ms=0)
  self.assertEqual(self.parse(r)['outcome'],2)
 def test_protocol_isolation(self):
  r=self.batch();b=frame('BT3',[r[k] for k in sync.BATCH_FIELDS]);self.assertIsNone(fine.parse_batch_telemetry(b))
  b=frame('BT2',[r[k] for k in fine.BATCH_FIELDS]);self.assertIsNone(sync.parse_batch_telemetry(b))
  p=next((ROOT/'工创赛/evidence/p1-foundation/2026-10-31/fine_rpm_vehicle/idle').glob('*.summary.json'))
  r=json.loads(p.read_text(encoding='utf8'))['last']
  names=sync.FIELDS+sync.SEQUENCE_FIELDS+sync.CAN_DIAG_FIELDS+sync.ACTION_FIELDS+sync.DISPLAY_FIELDS+sync.DISTANCE_FIELDS+['distance_fault_reason','can_stack_words','debug_plan_id']
  r['correction_rpm']=7
  b=frame('TM3',[int(r[k]) for k in names]);self.assertEqual(sync.parse_telemetry(b)['correction_rpm'],.7);self.assertIsNone(fine.parse_telemetry(b))
  self.assertIsNone(sync.parse_telemetry(frame('TM2',[int(r[k]) for k in names])))
if __name__=='__main__':
 policy=importlib.import_module('test_p1_perimeter8_client')
 policy.Perimeter8Controller=importlib.import_module('p1_sync_perimeter8_serial').Perimeter8Controller
 policy.command=sync.command
 suite=unittest.TestSuite([unittest.defaultTestLoader.loadTestsFromTestCase(SyncProtocolTests),unittest.defaultTestLoader.loadTestsFromModule(policy)])
 result=unittest.TextTestRunner(verbosity=2).run(suite);raise SystemExit(not result.wasSuccessful())
