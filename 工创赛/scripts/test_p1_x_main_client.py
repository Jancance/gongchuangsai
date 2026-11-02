import binascii
import unittest
import p1_x_main_transport as wire
from p1_x_main_perimeter import Perimeter8Controller, ROWS, KINDS
from test_p1_perimeter_client import idle

def frame(tag, values):
    prefix=(tag+','+','.join(str(v) for v in values)+',').encode('ascii')
    return prefix+f'{binascii.crc_hqx(prefix,0xFFFF):04X}\r\n'.encode('ascii')

class XMainTests(unittest.TestCase):
    def test_current_identity_only(self):
        values=[0]*66
        self.assertIsNone(wire.parse_telemetry(frame('TM9',values)))
        self.assertEqual(wire.parse_telemetry(frame('TMX',values))['telemetry_schema'],'TMX')

    def test_all_route_targets_and_wrong_distance(self):
        c=Perimeter8Controller()
        for i,row in enumerate(ROWS):
            turn=row['kind']=='TURN'
            r=idle(sequence_count=8,sequence_index=i,debug_plan_id=0,
                   action_kind=KINDS[row['kind']],feedback_limit_rpm=40 if turn else 80,
                   base_rpm=27 if turn else 49,backend_active=1,distance_mode=0 if turn else 1,
                   distance_target_milli_mm=round(row['amount']*10000),
                   correction_rpm=(5 if row['amount']>0 else -5) if turn else 0)
            self.assertTrue(c._running_shape(r))
            if not turn:
                r['distance_target_milli_mm']+=1000
                self.assertFalse(c._running_shape(r))

    def test_hf3_signed_target_and_bounds(self):
        v=[0]*43
        v[29]=11;v[30]=6;v[32]=1;v[34]=v[35]=15
        v[38]=0xFFFFFFF6;v[39]=3;v[40]=900;v[41]=950;v[42]=2
        r=wire.parse_fault_telemetry(frame('HF3',v))
        self.assertEqual(r['backend_target_decirpm'],-10)
        self.assertEqual(r['backend_measured_decirpm'],3)
        self.assertEqual(r['backend_reverse_samples'],2)
        v[29]=13
        with self.assertRaises(ValueError):wire.parse_fault_telemetry(frame('HF3',v))
        with self.assertRaises(ValueError):wire.parse_fault_telemetry(frame('HF3',v[:-1]))

    def test_crc_fault_is_rejected(self):
        data=bytearray(frame('HF3',[0]*43));data[7]^=1
        with self.assertRaises(ValueError):wire.parse_fault_telemetry(bytes(data))

    def test_replayed_terminal_cannot_complete(self):
        c=Perimeter8Controller('run-once',3,0);c.start_sent=True;c.last_seq=10
        r=idle(seq=10,phase=3,sequence_count=8,sequence_index=7,
               sequence_completed=8,sequence_state=3,backend_result=1,
               backend_cause=0,stop_mask=15,action_kind=5,base_rpm=1,
               feedback_limit_rpm=80,debug_plan_id=0,distance_mode=1,
               distance_reached=1,vm_request=0,pe2_active=0,correction_rpm=0)
        self.assertEqual(c.consume(r,1),[])
        self.assertIsNone(c.status)
        r['seq']=11
        self.assertEqual(c.consume(r,2),[])
        self.assertEqual(c.status,'software_complete')

if __name__=='__main__':unittest.main()
