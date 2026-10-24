import binascii
from pathlib import Path
import unittest
from p1_serial_motion import FIELDS, Controller, command, parse_telemetry


def record(**updates):
    r = dict.fromkeys(FIELDS, 0)
    r.update(seq=1,tick_ms=1000,source_valid=1,selftest=1,protocol_selftest=1,
             motion_mode=1,angle_age_ms=10,rate_age_ms=10,base_rpm=15,feedback_limit_rpm=30)
    r.update(updates)
    return r


def frame(r):
    prefix=("TM1,"+",".join(str(r[name]) for name in FIELDS)+",").encode()
    return prefix+f"{binascii.crc_hqx(prefix,0xFFFF):04X}\r\n".encode()


def armed_client():
    c=Controller("run-once")
    for n in range(1,4):
        messages=c.consume(record(seq=n,tick_ms=n*100),n*.1)
    return c,messages


class SerialMotionTests(unittest.TestCase):
    def test_crc_reference_and_signed_feedback(self):
        self.assertEqual(binascii.crc_hqx(b"123456789",0xFFFF),0x29B1)
        r=parse_telemetry(frame(record(rpm2=-24,yaw_mdeg=-90000,seq=-1)))
        self.assertEqual(r["rpm2"],-24)
        self.assertEqual(r["seq"],0xFFFFFFFF)

    def test_corrupted_frame_and_short_frame_rejected(self):
        r=frame(record(rpm1=21))
        for bad in [r.replace(b",21,",b",22,"),r[:-7],b"TM1,0,0000"]:
            with self.assertRaises(ValueError):parse_telemetry(bad)

    def test_wrong_protocol_does_not_identify(self):
        self.assertIsNone(parse_telemetry(b"LINK,HB,1,1000,VM0"))

    def test_frame_and_command_size_bounds(self):
        self.assertLess(len(frame(dict.fromkeys(FIELDS,-(2**31)))),768)
        self.assertLess(len(command("START",(0xFFFFFFFF,0xFFFFFFFF))),64)

    def test_unknown_command_rejected(self):
        with self.assertRaises(ValueError):command("SET_VM")

    def test_monitor_never_sends(self):
        c=Controller()
        for n in range(1,8):
            self.assertEqual(c.consume(record(seq=n),n*.1),[])
            self.assertEqual(c.tick(10+n),[])
        self.assertEqual(c.abort("done"),[])

    def test_unhealthy_or_wrong_mode_never_arms(self):
        for field,value in [("source_valid",0),("selftest",0),("can_fault",1),
                            ("angle_age_ms",100),("rate_mdps",3001),("vm_request",1),
                            ("motion_mode",0),("feedback_limit_rpm",17)]:
            c=Controller("run-once")
            for n in range(1,6):self.assertEqual(c.consume(record(seq=n,**{field:value}),n*.1),[])

    def test_three_fresh_frames_arm_then_matching_ack_starts_once(self):
        c,m=armed_client()
        self.assertEqual(m,[command("ARM",(3,300))])
        self.assertEqual(c.consume(record(seq=4,remote_state=1,token_seq=2,token_tick=200),.4),[])
        m=c.consume(record(seq=5,remote_state=1,token_seq=3,token_tick=300),.5)
        self.assertEqual(m,[command("START",(3,300))])
        self.assertEqual(c.consume(record(seq=6,remote_state=1,token_seq=3,token_tick=300),.6),[])

    def test_cached_frames_do_not_arm(self):
        c=Controller("run-once")
        for n in range(8):self.assertEqual(c.consume(record(seq=1),n*.1),[])
        self.assertEqual(c.healthy_frames,1)

    def test_health_change_before_start_aborts(self):
        c,_=armed_client()
        m=c.consume(record(seq=4,remote_state=1,token_seq=3,token_tick=300,source_valid=0),.4)
        self.assertEqual(m,[command("STOP",(3,300))])
        self.assertFalse(c.start_sent)

    def test_keepalive_then_telemetry_loss_stops_once(self):
        c,_=armed_client()
        c.consume(record(seq=4,remote_state=1,token_seq=3,token_tick=300),.4)
        c.consume(record(seq=5,remote_state=2,token_seq=3,token_tick=300,phase=2),.5)
        self.assertEqual(c.tick(.7),[command("KEEP",(3,300))])
        self.assertEqual(c.tick(1.2),[command("STOP",(3,300))])
        self.assertEqual(c.tick(1.3),[])

    def test_deadline_does_not_retry_start(self):
        c,_=armed_client()
        self.assertEqual(c.tick(3),[command("STOP",(3,300))])
        self.assertFalse(c.start_sent)

    def test_completion_requires_stop_mask_and_vm_off(self):
        c,_=armed_client()
        c.consume(record(seq=4,remote_state=1,token_seq=3,token_tick=300),.4)
        c.consume(record(seq=5,phase=3,backend_result=1,stop_mask=0,vm_request=1),.5)
        self.assertIsNone(c.status)
        c.consume(record(seq=6,phase=3,backend_result=1,stop_mask=15),.6)
        self.assertEqual(c.status,"software_complete")
        self.assertEqual(c.tick(20),[])

    def test_fault_aborts_and_cannot_restart(self):
        c,_=armed_client()
        c.consume(record(seq=4,remote_state=1,token_seq=3,token_tick=300),.4)
        self.assertEqual(c.consume(record(seq=5,phase=4,trip=5),.5),[command("STOP",(3,300))])
        self.assertEqual(c.consume(record(seq=6),.6),[])

    def test_counter_restart_during_arm_aborts(self):
        c,_=armed_client()
        self.assertEqual(c.consume(record(seq=0),.4),[command("STOP",(3,300))])

    def test_left_and_right_auto_start_prepare_run_and_finish(self):
        for mode in (1, 2):
            with self.subTest(mode=mode):
                c = Controller("run-once", expected_mode=mode)
                for seq in range(1, 4):
                    sent = c.consume(record(seq=seq, tick_ms=seq*100, motion_mode=mode), seq*.1)
                self.assertEqual(sent, [command("ARM", (3, 300))])
                sent = c.consume(record(seq=4, motion_mode=mode, remote_state=1,
                                        token_seq=3, token_tick=300), .4)
                self.assertEqual(sent, [command("START", (3, 300))])
                self.assertEqual(c.consume(record(seq=5, motion_mode=mode, phase=1,
                                        remote_state=2, token_seq=3, token_tick=300,
                                        vm_request=1, pe2_active=1), .5), [])
                self.assertEqual(c.stage, "running")
                self.assertEqual(c.tick(.7), [command("KEEP", (3, 300))])
                c.consume(record(seq=6, motion_mode=mode, phase=2,
                                 vm_request=1, pe2_active=1), .8)
                self.assertIsNone(c.status)
                c.consume(record(seq=7, motion_mode=mode, phase=3,
                                 backend_result=1, stop_mask=15), .9)
                self.assertEqual(c.status, "software_complete")
                self.assertEqual(c.tick(10), [])

    def test_link_test_firmware_cannot_trigger_auto_start(self):
        c = Controller("run-once")
        for line in (b"LINK,HB,1,1000,VM0", b"LINK,YAW,123,3,10", b"PONG,abc"):
            self.assertIsNone(parse_telemetry(line))
        self.assertEqual(c.stage, "observe")
        self.assertEqual(c.tick(10), [])

    def test_buffered_burst_cannot_arm(self):
        c = Controller("run-once")
        for seq in range(1, 15):
            self.assertEqual(c.consume(record(seq=seq, tick_ms=seq*100), 1.0), [])
        self.assertFalse(c.start_sent)
        self.assertEqual(c.stage, "observe")

    def test_backlog_to_live_jump_restarts_health_window(self):
        c = Controller("run-once")
        for seq in (1, 2):
            self.assertEqual(c.consume(record(seq=seq, tick_ms=seq*100), 1.0), [])
        self.assertEqual(c.consume(record(seq=300, tick_ms=30000), 1.1), [])
        self.assertEqual(c.consume(record(seq=301, tick_ms=30100), 1.2), [])
        self.assertEqual(c.consume(record(seq=302, tick_ms=30200), 1.3),
                         [command("ARM", (302, 30200))])

    def test_sequence_can_readiness_diagnostics_decode(self):
        r = record(motion_mode=3,safety_fault=32)
        prefix = ("TM1," + ",".join(str(r[name]) for name in FIELDS) + ",4,1,1,2,8,3,7,").encode()
        wire = prefix + f"{binascii.crc_hqx(prefix,0xFFFF):04X}\n".encode()
        parsed = parse_telemetry(wire)
        self.assertEqual(parsed["can_guard_cause"],2)
        self.assertEqual(parsed["can_guard_bad_nodes"],8)
        self.assertEqual(parsed["post_settle_mask"],7)

    def test_extended_sequence_telemetry_roundtrip(self):
        r = record(motion_mode=3)
        prefix = ("TM1," + ",".join(str(r[name]) for name in FIELDS) + ",2,1,1,").encode()
        wire = prefix + f"{binascii.crc_hqx(prefix,0xFFFF):04X}\n".encode()
        parsed = parse_telemetry(wire)
        self.assertEqual(parsed["sequence_index"], 1)
        self.assertEqual(parsed["sequence_completed"], 1)

    def test_combined_client_does_not_finish_at_first_segment(self):
        c = Controller("run-once", expected_mode=3)
        for n in range(1,4):
            c.consume(record(seq=n,tick_ms=n*100,motion_mode=3),n*.1)
        self.assertEqual(c.consume(record(seq=4,motion_mode=3,remote_state=1,
                                          token_seq=3,token_tick=300),.4),
                         [command("START",(3,300))])
        for n, completed in enumerate((1,2,3),5):
            c.consume(record(seq=n,motion_mode=3,phase=3,backend_result=1,
                             stop_mask=15,sequence_state=2,
                             sequence_completed=completed),n*.1)
            self.assertIsNone(c.status)
        c.consume(record(seq=8,motion_mode=3,phase=3,backend_result=1,
                         stop_mask=15,sequence_state=3,sequence_completed=4),.8)
        self.assertEqual(c.status,"software_complete")

    def test_combined_link_loss_during_gap_cancels(self):
        c = Controller("run-once", expected_mode=3)
        for n in range(1,4):
            c.consume(record(seq=n,tick_ms=n*100,motion_mode=3),n*.1)
        c.consume(record(seq=4,motion_mode=3,remote_state=1,
                         token_seq=3,token_tick=300),.4)
        c.consume(record(seq=5,motion_mode=3,phase=2,sequence_state=2,
                         remote_state=2,token_seq=3,token_tick=300),.5)
        self.assertEqual(c.tick(1.2),[command("STOP",(3,300))])
        self.assertEqual(c.tick(2),[])

    def test_combined_powered_gaps_keep_alive_until_final_vm_off(self):
        c = Controller("run-once", expected_mode=3)
        for n in range(1,4):
            c.consume(record(seq=n,tick_ms=n*100,motion_mode=3),n*.1)
        c.consume(record(seq=4,motion_mode=3,remote_state=1,
                         token_seq=3,token_tick=300),.4)
        for n in range(5,8):
            c.consume(record(seq=n,motion_mode=3,phase=2,sequence_state=2,
                             sequence_completed=n-4,sequence_index=n-5,
                             remote_state=2,token_seq=3,token_tick=300,
                             backend_result=1,stop_mask=15,vm_request=1,pe2_active=1),n*.3)
            self.assertIsNone(c.status)
            self.assertEqual(c.tick(n*.3+.21),[command("KEEP",(3,300))])
        c.consume(record(seq=8,motion_mode=3,phase=3,sequence_state=3,
                         sequence_completed=4,backend_result=1,stop_mask=15,
                         vm_request=1,pe2_active=1),2.7)
        self.assertIsNone(c.status)
        c.consume(record(seq=9,motion_mode=3,phase=3,sequence_state=3,
                         sequence_completed=4,backend_result=1,stop_mask=15),2.8)
        self.assertEqual(c.status,"software_complete")

    def test_explicit_stop_never_arms_and_waits_for_vm_off(self):
        c=Controller("stop")
        self.assertEqual(c.consume(record(),0),[command("STOP")])
        c.consume(record(seq=2,remote_state=3,remote_result=3,vm_request=1),.1)
        self.assertIsNone(c.status)
        c.consume(record(seq=3,remote_state=3,remote_result=3),.2)
        self.assertEqual(c.status,"stop_acknowledged")


if __name__=="__main__":unittest.main()
