import unittest,sys,binascii,tempfile
from pathlib import Path
from unittest.mock import patch
import p1_x_serial_motion as x
import p1_fine_serial_motion as emm
from test_p1_fine_serial_motion import frame,FineUnitsTests
class XProtocolTests(unittest.TestCase):
 def test_fractional_feedback_and_identity(self):
  names=x.FIELDS+x.SEQUENCE_FIELDS+x.CAN_DIAG_FIELDS+x.ACTION_FIELDS+x.DISPLAY_FIELDS+x.DISTANCE_FIELDS+['distance_fault_reason','can_stack_words','debug_plan_id'];r=FineUnitsTests().record()
  for raw in [-801,-51,-1,0,1,51,801]:
   for n in ['rpm1','rpm2','rpm3','rpm4']:r[n]=raw
   r['correction_rpm']=3;r['peak_abs_rpm']=3;r['last_nonzero_rpm']=3
   wire=frame('TM6',[r[k] for k in names]);q=x.parse_telemetry(wire)
   self.assertAlmostEqual(q['rpm1'],raw/10);self.assertEqual(q['rpm1_decirpm'],raw);self.assertEqual(q['feedback_resolution_rpm'],.1);self.assertAlmostEqual(q['correction_rpm'],.3);self.assertIsNone(emm.parse_telemetry(wire))
   self.assertIsNone(x.parse_telemetry(frame('TM2',[r[k] for k in names])))
 def test_fractional_batch_and_identity(self):
  import json
  root=Path(__file__).resolve().parents[2]
  p=next((root/'工创赛/evidence/p1-foundation/2026-10-31/heading_reference_vehicle/ccw_run_once').glob('*.summary.json'))
  r=json.loads(p.read_text(encoding='utf8'))['batch_records'][-1]
  r['correction_rpm']=-7;r['pending_rpm']=4
  wire=frame('BT6',[r[k] for k in x.BATCH_FIELDS]);out=x.parse_batch_telemetry(wire)
  self.assertAlmostEqual(out['correction_rpm'],-.7);self.assertAlmostEqual(out['pending_rpm'],.4)
  self.assertIsNone(emm.parse_batch_telemetry(wire))
  self.assertIsNone(x.parse_batch_telemetry(frame('BT2',[r[k] for k in x.BATCH_FIELDS])))
 def test_crc_rejected(self):
  with self.assertRaises(ValueError):x.parse_telemetry(b'TM6,1,0000')
 def test_real_serial_run_forbidden(self):
  with tempfile.TemporaryDirectory() as d,patch.object(sys,'argv',['x','--action','run-once','--mode','3','--plan','default','--output',d]),patch.object(x,'WindowsSerial') as serial:
   with self.assertRaises(SystemExit) as exc:x.main()
   self.assertEqual(exc.exception.code,2);serial.assert_not_called()
 def test_positive_negative_command_crc(self):
  self.assertEqual(x.command('STOP'),emm.command('STOP'))
if __name__=='__main__':unittest.main(verbosity=2)
