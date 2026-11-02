import binascii,importlib,json,sys,unittest
from pathlib import Path
import p1_fine_serial_motion as fine
import p1_serial_motion as integer

ROOT=Path(__file__).resolve().parents[2]
def frame(tag,values):
    prefix=(tag+','+','.join(str(v) for v in values)+',').encode('ascii')
    return prefix+f'{binascii.crc_hqx(prefix,0xffff):04X}\n'.encode('ascii')
class FineUnitsTests(unittest.TestCase):
    def record(self):
        p=next((ROOT/'工创赛/evidence/p1-foundation/2026-10-31/heading_reference_vehicle/perimeter_idle').glob('*.summary.json'))
        return json.loads(p.read_text(encoding='utf8'))['last']
    def test_decimal_command_and_integer_feedback(self):
        names=fine.FIELDS+fine.SEQUENCE_FIELDS+fine.CAN_DIAG_FIELDS+fine.ACTION_FIELDS+fine.DISPLAY_FIELDS+fine.DISTANCE_FIELDS+['distance_fault_reason','can_stack_words','debug_plan_id']
        r=self.record();r['rpm1']=23
        for n in [-270,-53,-7,-1,0,1,7,53,270]:
            r['correction_rpm']=n;r['peak_abs_rpm']=abs(n);r['last_nonzero_rpm']=n
            b=frame('TM2',[r[k] for k in names]);parsed=fine.parse_telemetry(b)
            self.assertEqual(parsed['correction_rpm'],n/10)
            self.assertEqual(parsed['correction_rpm_decirpm'],n)
            self.assertEqual(parsed['rpm1'],23)
            self.assertEqual(parsed['feedback_resolution_rpm'],1)
            self.assertIsNone(integer.parse_telemetry(b))
        self.assertIsNone(fine.parse_telemetry(frame('TM1',[r[k] for k in names])))
        r['correction_rpm']=271
        with self.assertRaises(ValueError):fine.parse_telemetry(frame('TM2',[r[k] for k in names]))
    def test_crc_and_negative_fractional_batch(self):
        p=next((ROOT/'工创赛/evidence/p1-foundation/2026-10-31/heading_reference_vehicle/ccw_run_once').glob('*.summary.json'))
        source=json.loads(p.read_text(encoding='utf8'))['batch_records'][-1]
        source['correction_rpm']=-7;source['pending_rpm']=4
        b=frame('BT2',[source[k] for k in fine.BATCH_FIELDS]);r=fine.parse_batch_telemetry(b)
        self.assertEqual(r['correction_rpm'],-.7);self.assertEqual(r['pending_rpm'],.4)
        self.assertIsNone(integer.parse_batch_telemetry(b))
        with self.assertRaises(ValueError):fine.parse_batch_telemetry(b[:-5]+b'0000\n')

if __name__=='__main__':
    # Existing perimeter policy tests exercise the copied controller unchanged.
    policy=importlib.import_module('test_p1_perimeter8_client')
    policy.Perimeter8Controller=importlib.import_module('p1_fine_perimeter8_serial').Perimeter8Controller
    policy.command=fine.command
    for name,value in list(vars(policy).items()):
        if getattr(value,'__name__',None)=='p1_perimeter8_serial':
            setattr(policy,name,importlib.import_module('p1_fine_perimeter8_serial'))
    suite=unittest.TestSuite([unittest.defaultTestLoader.loadTestsFromTestCase(FineUnitsTests),unittest.defaultTestLoader.loadTestsFromModule(policy)])
    result=unittest.TextTestRunner(verbosity=2).run(suite)
    raise SystemExit(not result.wasSuccessful())
