import unittest
from pathlib import Path
import p1_runtime_route as rt
import p1_x_main_transport as wire
from test_p1_perimeter_client import idle

CFG=Path(__file__).resolve().parents[1]/'config/p1_runtime_perimeter.json'
def sample(seq,**kw):
    r=idle(seq=seq,tick_ms=1000+seq*100,sequence_count=8,base_rpm=49,telemetry_schema='TMQ')
    r.update(kw);return r

class RuntimeRouteTests(unittest.TestCase):
    def setUp(self):self.rows=rt.load_route(CFG)
    def test_units_and_digest(self):
        self.assertEqual(self.rows[0],(4,18144,200,35000))
        self.assertEqual(self.rows[2],(3,-90000,30000,35000))
        self.assertNotEqual(rt.digest(1,self.rows),rt.digest(2,self.rows))
        self.assertNotEqual(rt.digest(1,self.rows),rt.digest(1,self.rows[:-1]))
        with self.assertRaises(ValueError):rt.integer(1.234,100)

    def test_current_schema_isolated_from_old_client(self):
        frame=rt.packet('TMQ',[0]*66)
        self.assertIsNone(wire.parse_telemetry(frame))
        self.assertEqual(wire.parse_telemetry(frame,b'TMQ')['telemetry_schema'],'TMQ')

    def test_upload_needs_paced_live_telemetry(self):
        u=rt.RouteUpload(1,self.rows)
        ack=rt.parse_route(rt.packet('RA1',[1,1,0,0,0,0,0,0,0]))
        u.consume_ack(ack);self.assertFalse(u.complete);self.assertEqual(u.tick(0),[])
        for n in range(3):u.observe(sample(n+1),n*.1)
        self.assertIn(b',BEGIN,8,',u.tick(.21)[0])
        self.assertEqual(u.tick(.3),[])

    def test_duplicate_ack_cannot_skip_missing_row(self):
        u=rt.RouteUpload(5,self.rows)
        for n in range(3):u.observe(sample(n+1),n*.1)
        u.tick(.21)
        u.consume_ack(dict(route_id=5,op=1,result=0,mask=0))
        self.assertEqual(u.index,1)
        u.tick(.22)
        u.consume_ack(dict(route_id=5,op=2,result=1,mask=0))
        self.assertEqual(u.index,1)
        u.consume_ack(dict(route_id=5,op=2,result=1,mask=1))
        self.assertEqual(u.index,2)

    def test_consumed_route_cannot_be_started_again(self):
        u=rt.RouteUpload(5,self.rows);u.index=len(u.commands)-1;u.sent_at=1
        u.consume_ack(dict(route_id=5,op=3,result=1,mask=255,committed_id=5,
                           digest=u.digest,count=8,ready=0,consumed=1))
        self.assertFalse(u.complete);self.assertEqual(u.error,'route_already_consumed')

    def test_status_is_not_health_or_start_authority(self):
        c=rt.RouteController(1,self.rows)
        status=dict(route_id=1,digest=c.digest,count=8,state=1)
        self.assertEqual(c.route_status(status,0),[])
        self.assertEqual(c.tick(.3),[]);self.assertFalse(c.start_sent)
        for n in range(3):c.consume(sample(n+1),n*.1)
        self.assertEqual(c.stage,'arm_wait')
        r=sample(4,remote_state=1,token_seq=c.token[0],token_tick=c.token[1])
        messages=c.consume(r,.3)
        self.assertEqual(len(messages),1);self.assertIn(b',GO,',messages[0])
        self.assertNotIn(b',START,',messages[0])
        self.assertIn(str(c.digest).encode(),messages[0])

    def test_all_action_shapes_and_wrong_target(self):
        c=rt.RouteController(1,self.rows)
        for i,row in enumerate(self.rows):
            turn=row[0]==3
            r=sample(i+1,sequence_index=i,action_kind=row[0],backend_active=1,
                     distance_mode=0 if turn else 1,feedback_limit_rpm=40 if turn else 80,
                     base_rpm=27 if turn else 49,correction_rpm=(5 if row[1]>0 else -5) if turn else 0,
                     distance_target_milli_mm=row[1]*100)
            self.assertTrue(c.shape(r))
            r['action_kind']=99;self.assertFalse(c.shape(r))

    def test_wrong_route_status_after_start_stops(self):
        c=rt.RouteController(1,self.rows);c.start_sent=True
        self.assertTrue(c.route_status(dict(route_id=2,digest=c.digest,count=8,state=2),1))
        self.assertEqual(c.status,'route_identity_changed')

    def test_corrupt_auxiliary_frames_rejected(self):
        frame=rt.packet('RS1',[1,2,1,0,0,8,0,0,0,1,255,0])
        self.assertEqual(rt.parse_route(frame)['count'],8)
        bad=bytearray(frame);bad[5]^=1
        with self.assertRaises(ValueError):rt.parse_route(bytes(bad))
        with self.assertRaises(ValueError):rt.parse_route(rt.packet('RA1',[1]*8))

if __name__=='__main__':unittest.main()
