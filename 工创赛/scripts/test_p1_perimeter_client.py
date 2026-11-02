import binascii
import json
from pathlib import Path
import tempfile
import unittest

from p1_perimeter_serial import PerimeterController,ROWS,KINDS
from p1_serial_motion import Controller,command,parse_perimeter_checkpoint,parse_position_endpoints,DIRECTIONAL_ENDPOINT_FIELDS
from p1_perimeter_records import export
from test_p1_serial_motion import record


def idle(**updates):
    r=record(motion_mode=3,sequence_count=12,sequence_index=0,sequence_state=0,sequence_completed=0,
             debug_plan_id=0,feedback_limit_rpm=80,base_rpm=15,distance_mode=0,action_kind=4)
    r.update(updates);return r


class PerimeterClientTests(unittest.TestCase):
    def test_explicit_identity_required(self):
        for args in [('run-once',1,0),('run-once',3,None)]:
            with self.assertRaises(ValueError):PerimeterController(*args)
        c=Controller('run-once',3,0)
        for i in range(1,6):self.assertEqual(c.consume(idle(seq=i,tick_ms=i*100),i*.1),[])

    def test_arm_start_once_then_all_action_shapes(self):
        c=PerimeterController('run-once',3,0)
        messages=[]
        for i in range(1,4):messages=c.consume(idle(seq=i,tick_ms=i*100),i*.1)
        self.assertEqual(messages,[command('PLAN0',(3,300))])
        c.consume(idle(seq=4,tick_ms=400,remote_accepted=1),.4)
        for i in range(5,8):messages=c.consume(idle(seq=i,tick_ms=i*100),i*.1)
        self.assertEqual(messages,[command('ARM',(7,700))])
        messages=c.consume(idle(seq=8,tick_ms=800,remote_state=1,token_seq=7,token_tick=700),.8)
        self.assertEqual(messages,[command('START',(7,700))])
        for i,row in enumerate(ROWS):
            turn=row['kind']=='TURN'
            r=idle(seq=9+i,tick_ms=900+i*100,phase=2,remote_state=2,token_seq=7,token_tick=700,
                   vm_request=1,pe2_active=1,backend_active=1,sequence_index=i,sequence_state=1,
                   action_kind=KINDS[row['kind']],feedback_limit_rpm=40 if turn else 80,
                   base_rpm=27 if turn else 15,distance_mode=0 if turn else 1,
                   distance_target_milli_mm=0 if turn else round(row['amount']*10000))
            self.assertEqual(c.consume(r,.9+i*.1),[])
        terminal=idle(seq=30,tick_ms=3000,phase=3,backend_result=1,stop_mask=15,sequence_state=3,
            sequence_index=11,sequence_completed=12,action_kind=5,distance_mode=1,distance_reached=1,backend_cause=0)
        self.assertEqual(c.consume(terminal,3),[]);self.assertEqual(c.status,'software_complete')

    def test_monitor_and_wrong_shape(self):
        c=PerimeterController()
        self.assertEqual(c.consume(idle(),0),[])
        c=PerimeterController('run-once',3,0);c.start_sent=True;c.last_seq=1
        self.assertEqual(c.consume(idle(seq=2,sequence_count=8),1),[command('STOP')])
        self.assertEqual(c.status,'perimeter_shape_changed')

    def test_checkpoint_crc_and_flags(self):
        fields=[12,1,1234,2250,2250,-90000,1,0,0,10,2250,2250,-90000]
        p=('OP1,'+','.join(map(str,fields))+',').encode();wire=p+f'{binascii.crc_hqx(p,0xFFFF):04X}'.encode()
        self.assertEqual(parse_perimeter_checkpoint(wire)['checkpoint'],12)
        with self.assertRaises(ValueError):parse_perimeter_checkpoint(wire[:-1]+b'X')

    def test_pe3_reverse_is_distinct_from_legacy_nontranslation(self):
        r=dict.fromkeys(DIRECTIONAL_ENDPOINT_FIELDS,0)
        r.update(outcome=1,start_mask=15,end_mask=15,translation_mode=3,distance_mode=1)
        for w in range(1,5):
            r[f'start_ms{w}']=1000+w*20;r[f'end_ms{w}']=2000+w*20
            r[f'end_raw{w}']=65536;r[f'end_sign{w}']=int(w in (1,3))
        def frame(prefix):
            p=(prefix+','+','.join(str(r[k]) for k in DIRECTIONAL_ENDPOINT_FIELDS)+',').encode()
            return p+f'{binascii.crc_hqx(p,0xFFFF):04X}'.encode()
        result=parse_position_endpoints(frame('PE3'))
        self.assertAlmostEqual(result['translation_travel_mm'],241.902634,places=4)
        with self.assertRaises(ValueError):parse_position_endpoints(frame('PE2'))

    def test_export_preserves_return_error_and_missing_samples(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);source=root/'capture.jsonl';rows=[]
            for i in range(13):rows.append(dict(direction='RX',frame_type='OP1',checkpoint=i,captured=1,
                tick_ms=i*100,x_mm=2250+i,y_mm=2250,heading_mdeg=-90000,valid=1,reasons=0,
                continuity_lost=0,age_ms=10,target_x_mm=2250,target_y_mm=2250,target_heading_mdeg=-90000))
            source.write_text('\n'.join(map(json.dumps,rows)),encoding='utf8')
            report=export(source,root/'full');self.assertEqual(report['estimated_return_error']['dx_mm'],12)
            self.assertFalse(report['vehicle_return_proven'])
            source.write_text(json.dumps(rows[0]),encoding='utf8')
            report=export(source,root/'missing');self.assertFalse(report['all_checkpoints_present_and_valid'])
            self.assertNotIn('estimated_return_error',report)


if __name__=='__main__':unittest.main()
