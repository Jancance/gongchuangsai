import binascii
from pathlib import Path
import unittest
from p1_serial_motion import FIELDS, Controller, command, parse_telemetry, target_theoretical_cm_s, POSITION_FIELDS, parse_position_telemetry, ENDPOINT_FIELDS, DIRECTIONAL_ENDPOINT_FIELDS, parse_position_endpoints


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

    def test_action_count_telemetry_and_variable_completion(self):
        r = record(motion_mode=3,base_rpm=5)
        prefix = ("TM1," + ",".join(str(r[name]) for name in FIELDS) + ",0,0,0,0,0,0,0,5,").encode()
        wire = prefix + f"{binascii.crc_hqx(prefix,0xFFFF):04X}\n".encode()
        self.assertEqual(parse_telemetry(wire)["sequence_count"],5)
        for count in (1, 3, 5, 8):
            c = Controller("run-once", expected_mode=3)
            for n in range(1,4):
                c.consume(record(seq=n,tick_ms=n*100,motion_mode=3,
                                 base_rpm=5,sequence_count=count),n*.1)
            self.assertEqual(c.stage,"arm_wait")
            c.consume(record(seq=4,motion_mode=3,base_rpm=5,sequence_count=count,
                             remote_state=1,token_seq=3,token_tick=300),.4)
            if count > 1:
                c.consume(record(seq=5,motion_mode=3,phase=3,backend_result=1,
                                 stop_mask=15,sequence_state=3,sequence_count=count,
                                 sequence_completed=count-1),.5)
                self.assertIsNone(c.status)
            c.consume(record(seq=6,motion_mode=3,phase=3,backend_result=1,
                             stop_mask=15,sequence_state=3,sequence_count=count,
                             sequence_completed=count),.6)
            self.assertEqual(c.status,"software_complete")

    def test_invalid_action_count_cannot_arm(self):
        for count in (0, 9):
            c = Controller("run-once", expected_mode=3)
            for n in range(1,6):
                self.assertEqual(c.consume(record(seq=n,tick_ms=n*100,
                    motion_mode=3,sequence_count=count),n*.1),[])
            self.assertEqual(c.stage,"observe")

    def test_speed_display_units_and_turn_sentinel(self):
        self.assertAlmostEqual(target_theoretical_cm_s(record()),6.048,places=3)
        self.assertEqual(target_theoretical_cm_s(record(motion_mode=3,
            target_theoretical_milli_cm_s=6047)),6.047)
        self.assertIsNone(target_theoretical_cm_s(record(motion_mode=3,
            action_kind=3,target_theoretical_milli_cm_s=-(2**31))))
        self.assertIsNone(target_theoretical_cm_s(record(motion_mode=3)))

    def test_new_action_and_speed_frame_keeps_control_data(self):
        r = record(motion_mode=3)
        prefix = ("TM1," + ",".join(str(r[name]) for name in FIELDS) +
                  ",0,0,0,0,0,0,0,4,1,6047,").encode()
        wire = prefix + f"{binascii.crc_hqx(prefix,0xFFFF):04X}\n".encode()
        parsed = parse_telemetry(wire)
        self.assertEqual(parsed["action_kind"],1)
        self.assertEqual(parsed["sequence_count"],4)
        self.assertEqual(target_theoretical_cm_s(parsed),6.047)
        self.assertEqual(parsed["base_rpm"],15)

    def test_explicit_stop_never_arms_and_waits_for_vm_off(self):
        c=Controller("stop")
        self.assertEqual(c.consume(record(),0),[command("STOP")])
        c.consume(record(seq=2,remote_state=3,remote_result=3,vm_request=1),.1)
        self.assertIsNone(c.status)
        c.consume(record(seq=3,remote_state=3,remote_result=3),.2)
        self.assertEqual(c.status,"stop_acknowledged")


class PositionTelemetryTests(unittest.TestCase):
    def packet(self, **updates):
        r = dict.fromkeys(POSITION_FIELDS, 0)
        r.update(tick_ms=100, session=2, sample_sequence=3, valid_mask=15, vm_active=1)
        for w in range(1, 5):
            r.update({f"result{w}": 1, f"sample_ms{w}": 90, f"age_ms{w}": 10, f"raw{w}": 65536})
        r.update(updates)
        prefix = ("PM1," + ",".join(str(r[k]) for k in POSITION_FIELDS) + ",").encode()
        return prefix + f"{binascii.crc_hqx(prefix,0xFFFF):04X}\r\n".encode()

    def test_full_uint32_magnitude_and_sign(self):
        p = parse_position_telemetry(self.packet(raw2=0xFFFFFFFF, sign2=1))
        self.assertEqual(p["signed_counts"], [65536, -0xFFFFFFFF, 65536, 65536])

    def test_off_and_expired_samples_never_become_counts(self):
        p = parse_position_telemetry(self.packet(valid_mask=0, vm_active=0, age_ms1=0xFFFFFFFF))
        self.assertEqual(p["signed_counts"], [None]*4)

    def test_validity_rejects_stale_failed_or_power_off(self):
        for values in [dict(age_ms1=1000), dict(result2=4), dict(vm_active=0)]:
            with self.subTest(values=values), self.assertRaises(ValueError):
                parse_position_telemetry(self.packet(**values))

    def test_unsigned_tick_wrap_is_preserved(self):
        p = parse_position_telemetry(self.packet(tick_ms=0xFFFFFFFF, sample_ms1=0xFFFFFFF5))
        self.assertEqual(p["tick_ms"], 0xFFFFFFFF)
        self.assertEqual(p["sample_ms1"], 0xFFFFFFF5)

    def test_invalid_sign_mask_and_range(self):
        for values in [dict(sign1=2), dict(valid_mask=16), dict(raw4=-1), dict(raw1=2**32)]:
            with self.subTest(values=values), self.assertRaises(ValueError):
                parse_position_telemetry(self.packet(**values))

    def test_crc_corruption(self):
        b = self.packet().replace(b"65536", b"65537", 1)
        with self.assertRaises(ValueError): parse_position_telemetry(b)

    def test_no_control_token_from_position(self):
        self.assertIsNone(parse_telemetry(self.packet()))
        self.assertIsNone(parse_position_telemetry(frame(record())))
        self.assertNotIn("token_seq", parse_position_telemetry(self.packet()))

    def test_missing_and_excess_fields(self):
        for p in [b"PM1,1,", b"PM1," + b"1,"*26]:
            b = p + f"{binascii.crc_hqx(p,0xFFFF):04X}".encode()
            with self.assertRaises(ValueError): parse_position_telemetry(b)


class EndpointTelemetryTests(unittest.TestCase):
    def packet(self, **updates):
        r = dict.fromkeys(ENDPOINT_FIELDS, 0)
        r.update(tick_ms=5000, session=1, outcome=1, start_mask=15, end_mask=15)
        for w in range(1,5):
            r.update({f"start_sign{w}": int(w in (2,4)), f"end_sign{w}": int(w in (2,4)),
                      f"start_raw{w}": 100, f"end_raw{w}": 65636,
                      f"start_ms{w}": 1000+w*10, f"end_ms{w}": 4000+w*10})
        r.update(updates)
        p = ("PE1,"+",".join(str(r[k]) for k in ENDPOINT_FIELDS)+",").encode()
        return p+f"{binascii.crc_hqx(p,0xFFFF):04X}\n".encode()

    def test_one_turn_all_logical_forward(self):
        r = parse_position_endpoints(self.packet())
        self.assertEqual(r["delta_counts"], [65536,-65536,65536,-65536])
        for v in r["logical_wheel_mm"]: self.assertAlmostEqual(v,241.90263432641407)
        self.assertTrue(r["historical"])

    def test_sign_crossing_uses_wide_integer(self):
        r = parse_position_endpoints(self.packet(start_sign1=1,start_raw1=0xFFFFFFFF,end_raw1=0xFFFFFFFF))
        self.assertEqual(r["delta_counts"][0],2*0xFFFFFFFF)

    def test_unsigned_time_wrap(self):
        updates = {f"start_ms{w}":0xFFFFFF00+w*10 for w in range(1,5)}
        updates.update({f"end_ms{w}":100+w*10 for w in range(1,5)})
        self.assertEqual(parse_position_endpoints(self.packet(**updates))["session"],1)

    def test_partial_failed_and_invalid_sign(self):
        for v in [dict(start_mask=7),dict(end_mask=0),dict(outcome=2),dict(end_sign2=2)]:
            with self.subTest(v=v),self.assertRaises(ValueError):parse_position_endpoints(self.packet(**v))

    def test_reversed_time_and_stale_batch(self):
        for v in [dict(end_ms2=1),dict(start_ms4=3000),dict(end_ms1=200000)]:
            with self.subTest(v=v),self.assertRaises(ValueError):parse_position_endpoints(self.packet(**v))

    def test_crc_failure(self):
        with self.assertRaises(ValueError):parse_position_endpoints(self.packet().replace(b"65636",b"65637",1))

    def test_other_protocols_ignore_endpoint(self):
        p=self.packet()
        self.assertIsNone(parse_telemetry(p));self.assertIsNone(parse_position_telemetry(p))
        self.assertIsNone(parse_position_endpoints(frame(record())))

    def test_client_drains_final_endpoint_without_sending_commands(self):
        import contextlib, io, itertools, json, tempfile
        from unittest.mock import patch
        import p1_serial_motion as module
        packet = self.packet()
        class Serial:
            def __init__(self, port): self.blocks = iter([frame(record(phase=3)), packet])
            def read(self): return next(self.blocks, b"")
            def write(self, data): raise AssertionError("No commands after completion")
            def close(self): pass
        class CompletedController:
            def __init__(self, *args): self.status = None
            def consume(self, record, now): self.status = "software_complete"; return []
            def tick(self, now): raise AssertionError("No control tick after completion")
        ticks = itertools.count(0, .05)
        with tempfile.TemporaryDirectory() as directory:
            argv = ["p1_serial_motion.py", "--action", "run-once", "--seconds", "2", "--output", directory]
            with patch.object(module, "WindowsSerial", Serial), patch.object(module, "Controller", CompletedController), \
                 patch("sys.argv", argv), patch.object(module.time, "monotonic", side_effect=lambda: next(ticks)), \
                 contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(module.main(), 0)
            result = json.loads(next(Path(directory).glob("*.summary.json")).read_text(encoding="utf-8"))
            self.assertEqual(result["endpoint_frames"], 1)
            self.assertEqual(result["sent_commands"], 0)
            self.assertEqual(result["endpoints"][0]["delta_counts"], [65536,-65536,65536,-65536])

    def test_field_count_and_unsigned_overflow(self):
        with self.assertRaises(ValueError):parse_position_endpoints(self.packet(start_raw1=2**32))
        p=b"PE1,1,"
        with self.assertRaises(ValueError):parse_position_endpoints(p+f"{binascii.crc_hqx(p,0xFFFF):04X}".encode())


class DistanceTelemetryTests(unittest.TestCase):
    def test_extended_distance_frame(self):
        r=record(motion_mode=3)
        p=("TM1,"+",".join(str(r[k]) for k in FIELDS)+",1,0,0,0,0,4,15,1,4,6047,1,870000,725000,15,0,").encode()
        parsed=parse_telemetry(p+f"{binascii.crc_hqx(p,0xFFFF):04X}\n".encode())
        self.assertEqual(parsed["distance_target_milli_mm"],870000)
        self.assertEqual(parsed["distance_progress_milli_mm"],725000)
        self.assertEqual(parsed["distance_reached"],0)

    def test_distance_fault_reason_frame(self):
        r=record(motion_mode=3,phase=4,trip=5)
        p=("TM1,"+",".join(str(r[k]) for k in FIELDS)+",4,0,0,0,0,4,15,1,4,6047,1,870000,361180,15,0,3,").encode()
        parsed=parse_telemetry(p+f"{binascii.crc_hqx(p,0xFFFF):04X}\n".encode())
        self.assertEqual(parsed["distance_fault_reason"],3)
        self.assertEqual(parsed["distance_reached"],0)

    def test_done_without_reached_cannot_pass(self):
        c=Controller("run-once",3);c.stage="running";c.start_sent=True
        r=record(motion_mode=3,phase=3,backend_result=1,stop_mask=15,
                 sequence_state=3,sequence_count=1,sequence_completed=1,distance_mode=1,distance_reached=0)
        self.assertTrue(c.consume(r,1))
        self.assertNotEqual(c.status,"software_complete")

    def test_done_after_reached_passes(self):
        c=Controller("run-once",3);c.stage="running";c.start_sent=True
        r=record(motion_mode=3,phase=3,backend_result=1,stop_mask=15,
                 sequence_state=3,sequence_count=1,sequence_completed=1,distance_mode=1,distance_reached=1)
        self.assertEqual(c.consume(r,1),[])
        self.assertEqual(c.status,"software_complete")


class LateralEndpointTests(unittest.TestCase):
    def packet(self, mode=1, distance=1, **updates):
        r = dict.fromkeys(DIRECTIONAL_ENDPOINT_FIELDS,0)
        r.update(tick_ms=5000,session=1,outcome=1,start_mask=15,end_mask=15,
                 translation_mode=mode,distance_mode=distance)
        signs = {0:(0,1,0,1),1:(0,0,1,1),2:(1,1,0,0),3:(0,0,0,0)}[mode]
        for w,sign in enumerate(signs,1):
            r.update({f"start_ms{w}":1000+w*10,f"end_ms{w}":4000+w*10,
                      f"end_raw{w}":65536,f"end_sign{w}":sign})
        r.update(updates)
        p=("PE2,"+",".join(str(r[k]) for k in DIRECTIONAL_ENDPOINT_FIELDS)+",").encode()
        return p+f"{binascii.crc_hqx(p,0xFFFF):04X}\n".encode()

    def test_left_projects_positive_without_forward_average(self):
        r=parse_position_endpoints(self.packet(1))
        self.assertAlmostEqual(sum(r["logical_wheel_mm"]),0)
        self.assertAlmostEqual(r["translation_travel_mm"],241.90263432641407)
        for x in r["requested_wheel_mm"]:self.assertGreater(x,0)

    def test_right_projects_positive(self):
        r=parse_position_endpoints(self.packet(2))
        self.assertAlmostEqual(r["translation_travel_mm"],241.90263432641407)
        self.assertLess(r["logical_wheel_mm"][0],0)
        self.assertGreater(r["logical_wheel_mm"][1],0)

    def test_forward_remains_compatible(self):
        r=parse_position_endpoints(self.packet(0))
        self.assertAlmostEqual(r["translation_travel_mm"],241.90263432641407)

    def test_turn_does_not_invent_translation(self):
        r=parse_position_endpoints(self.packet(3,0))
        self.assertIsNone(r["translation_travel_mm"])
        with self.assertRaises(ValueError):parse_position_endpoints(self.packet(3,1))

    def test_timed_lateral_is_labeled_without_distance_control_claim(self):
        r=parse_position_endpoints(self.packet(1,0))
        self.assertEqual(r["distance_mode"],0)
        self.assertGreater(r["translation_travel_mm"],0)

    def test_bad_flags_and_incomplete_records(self):
        for values in [dict(translation_mode=4),dict(distance_mode=2),dict(start_mask=0),dict(end_mask=7)]:
            with self.subTest(values=values),self.assertRaises(ValueError):
                parse_position_endpoints(self.packet(**values))

    def test_crc_and_other_parser_isolation(self):
        b=self.packet()
        self.assertIsNone(parse_telemetry(b));self.assertIsNone(parse_position_telemetry(b))
        with self.assertRaises(ValueError):parse_position_endpoints(b.replace(b"65536",b"65537",1))

    def test_legacy_endpoint_direction_stays_unknown(self):
        legacy=EndpointTelemetryTests().packet()
        self.assertIsNone(parse_position_endpoints(legacy)["translation_travel_mm"])


class FaultTelemetryTests(unittest.TestCase):
    def packet(self, **updates):
        from p1_serial_motion import FAULT_FIELDS
        values = dict.fromkeys(FAULT_FIELDS, 0)
        values.update(fault_tick_ms=28600, arm_sequence=1, trip=5, angle_age_ms=0xFFFFFFFF,
                      event_reason=1, event_sr=0x28, uart_ore=1, hwt_uart_errors=1)
        values.update(updates)
        prefix = ("HF1,"+",".join(str(values[k]) for k in FAULT_FIELDS)+",").encode()
        return prefix+f"{binascii.crc_hqx(prefix,0xFFFF):04X}\n".encode()

    def test_fault_uint32_and_uart_reason(self):
        from p1_serial_motion import parse_fault_telemetry
        r=parse_fault_telemetry(self.packet())
        self.assertEqual(r["angle_age_ms"],0xFFFFFFFF)
        self.assertEqual((r["event_reason"],r["event_sr"],r["uart_ore"]),(1,0x28,1))

    def test_fault_checksum_and_overflow_reasons(self):
        from p1_serial_motion import parse_fault_telemetry
        for reason in (0,2,4):
            self.assertEqual(parse_fault_telemetry(self.packet(event_reason=reason))["event_reason"],reason)

    def test_fault_rejects_crc_range_and_flags(self):
        from p1_serial_motion import parse_fault_telemetry
        with self.assertRaises(ValueError): parse_fault_telemetry(self.packet().replace(b"28600",b"28601"))
        for changes in (dict(event_reason=8),dict(source_valid=2),dict(event_seen=4),dict(uart_ore=-1)):
            with self.assertRaises(ValueError):parse_fault_telemetry(self.packet(**changes))

    def test_fault_rejects_short_frame(self):
        from p1_serial_motion import parse_fault_telemetry
        prefix=b"HF1,1,2,"
        with self.assertRaises(ValueError):parse_fault_telemetry(prefix+f"{binascii.crc_hqx(prefix,0xFFFF):04X}".encode())

    def test_failed_run_drains_fault_snapshot_without_restarting(self):
        import contextlib
        import io
        import json
        import tempfile
        from unittest.mock import patch
        import p1_serial_motion as motion
        packet = self.packet()
        class Serial:
            clock = 0.0
            sent = []
            count = 0
            def __init__(self, port): pass
            def read(self):
                self.clock += .1
                Serial.clock = self.clock
                self.count += 1
                if self.count <= 3:
                    return frame(record(seq=self.count,tick_ms=self.count*100,motion_mode=3))
                if self.count == 4:
                    return frame(record(seq=4,tick_ms=400,motion_mode=3,remote_state=1,
                                        token_seq=3,token_tick=300))
                if self.count == 5:
                    return frame(record(seq=5,tick_ms=500,motion_mode=3,phase=4,trip=5,
                                        remote_state=3,backend_cause=9))
                if self.count == 6: return packet
                return b""
            def write(self, data): self.sent.append(data)
            def close(self): pass
        with tempfile.TemporaryDirectory() as output:
            with patch.object(motion,"WindowsSerial",Serial), patch.object(motion.time,"monotonic",side_effect=lambda:Serial.clock), \
                 patch("sys.argv",["p1_serial_motion","--mode","3","--action","run-once","--seconds","5","--output",output]), \
                 contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(motion.main(),2)
            report=json.loads(next(Path(output).glob("*.summary.json")).read_text(encoding="utf8"))
        self.assertEqual(report["status"],"motion_fault_or_cancelled")
        self.assertEqual(report["fault_frames"],1)
        self.assertEqual(report["last_fault"]["event_reason"],1)
        verbs=[b.rsplit(b",",2)[1] for b in Serial.sent]
        self.assertEqual(verbs,[b"ARM",b"START",b"STOP"])

    def test_fault_not_motion_or_position_token(self):
        b=self.packet()
        self.assertIsNone(parse_telemetry(b));self.assertIsNone(parse_position_telemetry(b))
        self.assertIsNone(parse_position_endpoints(b))


class BackendFaultTelemetryTests(unittest.TestCase):
    def packet(self, **updates):
        from p1_serial_motion import BACKEND_FAULT_FIELDS
        base=FaultTelemetryTests().packet().rstrip().rsplit(b",",1)[0].split(b",",1)[1]
        extra=dict.fromkeys(BACKEND_FAULT_FIELDS,0)
        extra.update(backend_fault_reason=6,backend_fault_state=4,backend_io_result=4,backend_address=2)
        extra.update(updates)
        prefix=b"HF2,"+base+b","+(",".join(str(extra[k]) for k in BACKEND_FAULT_FIELDS)+",").encode()
        return prefix+f"{binascii.crc_hqx(prefix,0xFFFF):04X}\n".encode()

    def test_backend_branch_io_and_first_state(self):
        from p1_serial_motion import parse_fault_telemetry
        r=parse_fault_telemetry(self.packet())
        self.assertEqual((r["backend_fault_reason"],r["backend_io_result"],r["backend_address"]),(6,4,2))
        self.assertEqual(r["backend_fault_state"],4)
        self.assertEqual(r["angle_age_ms"],0xFFFFFFFF)

    def test_backend_invalid_flags_rejected(self):
        from p1_serial_motion import parse_fault_telemetry
        for key,value in [("backend_fault_reason",9),("backend_fault_state",12),("backend_address",5),
                          ("backend_status_mask",16),("backend_speed_mask",16)]:
            with self.assertRaises(ValueError):parse_fault_telemetry(self.packet(**{key:value}))

    def test_backend_bad_crc_and_protocol_isolation(self):
        from p1_serial_motion import parse_fault_telemetry
        b=self.packet()
        with self.assertRaises(ValueError):parse_fault_telemetry(b.replace(b"28600",b"28601"))
        self.assertIsNone(parse_telemetry(b));self.assertIsNone(parse_position_telemetry(b))
        self.assertIsNone(parse_position_endpoints(b))

    def test_backend_command_timeout_and_missing_samples(self):
        from p1_serial_motion import parse_fault_telemetry
        r=parse_fault_telemetry(self.packet(backend_fault_reason=1,backend_command_age_ms=100,
                                           backend_status_mask=0,backend_feedback_age_ms=0xFFFFFFFF))
        self.assertEqual(r["backend_command_age_ms"],100)
        self.assertEqual(r["backend_feedback_age_ms"],0xFFFFFFFF)

    def test_failed_run_drains_hf2_without_restarting(self):
        FaultTelemetryTests.test_failed_run_drains_fault_snapshot_without_restarting(self)

    def test_legacy_has_no_invented_backend_cause(self):
        from p1_serial_motion import parse_fault_telemetry
        self.assertNotIn("backend_fault_reason",parse_fault_telemetry(FaultTelemetryTests().packet()))


class TimingTelemetryTests(unittest.TestCase):
    def packet(self, **updates):
        from p1_serial_motion import TIMING_FIELDS
        r=dict.fromkeys(TIMING_FIELDS,0)
        r.update(fault_tick_ms=37868,cycle_hz=168000000,timer_enabled=1,event_cycle=18000,
                 hold_critical_count=3,hold_critical_max_cycles=16800,hold_critical_max_start=1000,
                 hold_critical_max_end=17800,hold_critical_max_arm=2)
        r.update(updates)
        prefix=("CT1,"+",".join(str(r[k]) for k in TIMING_FIELDS)+",").encode()
        return prefix+f"{binascii.crc_hqx(prefix,0xFFFF):04X}\n".encode()

    def test_timing_cycles_convert_to_microseconds(self):
        from p1_serial_motion import parse_timing_telemetry
        r=parse_timing_telemetry(self.packet())
        self.assertEqual(r["hold_critical_max_us"],100)
        self.assertIsNone(r["backend_begin_max_us"])

    def test_timing_interval_counter_wrap(self):
        from p1_serial_motion import parse_timing_telemetry
        r=parse_timing_telemetry(self.packet(hold_critical_max_start=0xFFFFFFF0,
            hold_critical_max_end=0x90,hold_critical_max_cycles=160))
        self.assertAlmostEqual(r["hold_critical_max_us"],160/168)

    def test_disabled_timer_never_claims_zero_cost(self):
        from p1_serial_motion import parse_timing_telemetry
        r=parse_timing_telemetry(self.packet(timer_enabled=0,hold_critical_max_cycles=0))
        self.assertIsNone(r["hold_critical_max_us"])
        with self.assertRaises(ValueError): parse_timing_telemetry(self.packet(timer_enabled=0))

    def test_timing_rejects_inconsistent_interval_and_frequency(self):
        from p1_serial_motion import parse_timing_telemetry
        for changes in (dict(cycle_hz=0),dict(timer_enabled=2),dict(hold_critical_max_end=17801)):
            with self.assertRaises(ValueError):parse_timing_telemetry(self.packet(**changes))

    def test_timing_crc_and_protocol_isolation(self):
        from p1_serial_motion import parse_timing_telemetry,parse_fault_telemetry
        b=self.packet()
        with self.assertRaises(ValueError):parse_timing_telemetry(b.replace(b"37868",b"37869"))
        self.assertIsNone(parse_fault_telemetry(b));self.assertIsNone(parse_telemetry(b))
        self.assertIsNone(parse_position_telemetry(b));self.assertIsNone(parse_position_endpoints(b))

    def test_timing_after_fault_does_not_restart_or_renew_control(self):
        from unittest.mock import patch
        combined=FaultTelemetryTests().packet()+self.packet()
        with patch.object(self,"packet",return_value=combined):
            FaultTelemetryTests.test_failed_run_drains_fault_snapshot_without_restarting(self)

    def test_timing_short_or_out_of_range_frame_rejected(self):
        from p1_serial_motion import parse_timing_telemetry
        with self.assertRaises(ValueError):parse_timing_telemetry(self.packet(event_cycle=-1))
        prefix=b"CT1,1,2,"
        with self.assertRaises(ValueError):parse_timing_telemetry(prefix+f"{binascii.crc_hqx(prefix,0xFFFF):04X}".encode())


class CompletedTimingTelemetryTests(unittest.TestCase):
    def packet(self, index=0, session=1, outcome=1):
        old=TimingTelemetryTests().packet().strip().rsplit(b",",1)[0].split(b",",1)[1]
        prefix=f"CT2,{outcome},{index},{session},".encode()+old+b","
        return prefix+f"{binascii.crc_hqx(prefix,0xFFFF):04X}\n".encode()

    def test_completed_timing_has_explicit_stage_context(self):
        from p1_serial_motion import parse_timing_telemetry
        r=parse_timing_telemetry(self.packet(2,3))
        self.assertEqual((r["outcome"],r["sequence_index"],r["backend_session"]),(1,2,3))
        self.assertEqual(r["snapshot_tick_ms"],37868)
        self.assertNotIn("fault_tick_ms",r)
        self.assertEqual(r["hold_critical_max_us"],100)

    def test_completed_timing_rejects_wrong_context_or_crc(self):
        from p1_serial_motion import parse_timing_telemetry
        for b in (self.packet(index=8),self.packet(outcome=0),self.packet().replace(b"37868",b"37869")):
            with self.assertRaises(ValueError):parse_timing_telemetry(b)

    def test_completed_timing_does_not_parse_as_motion_or_fault(self):
        from p1_serial_motion import parse_fault_telemetry
        b=self.packet()
        self.assertIsNone(parse_telemetry(b));self.assertIsNone(parse_fault_telemetry(b))
        self.assertIsNone(parse_position_telemetry(b));self.assertIsNone(parse_position_endpoints(b))

    def test_success_drains_all_stages_deduplicates_replay_and_sends_no_restart(self):
        import contextlib,io,json,tempfile
        from unittest.mock import patch
        import p1_serial_motion as motion
        packets=[self.packet(0,1),self.packet(1,2),self.packet(2,3),self.packet(1,2)]
        names=(motion.FIELDS+motion.SEQUENCE_FIELDS+motion.CAN_DIAG_FIELDS+motion.ACTION_FIELDS+
               motion.DISPLAY_FIELDS+motion.DISTANCE_FIELDS+["distance_fault_reason"])
        def full_frame(**changes):
            r=dict.fromkeys(names,0)
            r.update(record(motion_mode=3),sequence_count=3)
            r.update(changes)
            prefix=("TM1,"+",".join(str(r[k]) for k in names)+",").encode()
            return prefix+f"{binascii.crc_hqx(prefix,0xFFFF):04X}\n".encode()
        class Serial:
            clock=0.0
            count=0
            sent=[]
            def __init__(self,port):pass
            def read(self):
                Serial.clock+=.1;self.count+=1
                if self.count<=3:return full_frame(seq=self.count,tick_ms=self.count*100)
                if self.count==4:return full_frame(seq=4,tick_ms=400,remote_state=1,token_seq=3,token_tick=300)
                if self.count==5:return full_frame(seq=5,tick_ms=500,phase=2,remote_state=2,token_seq=3,token_tick=300,vm_request=1,pe2_active=1)
                if self.count==6:return full_frame(seq=6,tick_ms=600,phase=3,remote_state=3,backend_result=1,
                    stop_mask=15,sequence_state=3,sequence_completed=3,distance_mode=1,distance_reached=1)
                if 7<=self.count<=10:return packets[self.count-7]
                return b""
            def write(self,data):self.sent.append(data)
            def close(self):pass
        with tempfile.TemporaryDirectory() as output:
            with patch.object(motion,"WindowsSerial",Serial),patch.object(motion.time,"monotonic",side_effect=lambda:Serial.clock), \
                 patch("sys.argv",["p1_serial_motion","--mode","3","--action","run-once","--seconds","5","--output",output]), \
                 contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(motion.main(),0)
            r=json.loads(next(Path(output).glob("*.summary.json")).read_text(encoding="utf8"))
        self.assertEqual(r["status"],"software_complete")
        self.assertEqual(r["timing_frames"],4)
        self.assertEqual(len(r["timing_records"]),3)
        self.assertEqual([v["sequence_index"] for v in r["timing_records"]],[0,1,2])
        self.assertEqual([v.rsplit(b",",2)[1] for v in Serial.sent],[b"ARM",b"START"])




class CanStackTelemetryTests(unittest.TestCase):
    def test_new_single_and_combination_tail_while_accepting_old_lengths(self):
        import p1_serial_motion as motion
        combo = (motion.FIELDS + motion.SEQUENCE_FIELDS + motion.CAN_DIAG_FIELDS +
                 motion.ACTION_FIELDS + motion.DISPLAY_FIELDS + motion.DISTANCE_FIELDS +
                 ["distance_fault_reason"])
        for names in [motion.FIELDS, combo]:
            values = [0] * len(names)
            for stack in [None, 0, 96, 384]:
                body = values if stack is None else values + [stack]
                prefix = ("TM1," + ",".join(map(str, body)) + ",").encode()
                line = prefix + f"{binascii.crc_hqx(prefix, 0xFFFF):04X}\r\n".encode()
                parsed = parse_telemetry(line)
                if stack is None:
                    self.assertNotIn("can_stack_words", parsed)
                else:
                    self.assertEqual(parsed["can_stack_words"], stack)
                self.assertEqual(parsed["rx_stack_words"], 0)

    def test_extra_stack_field_does_not_arm_unhealthy_controller(self):
        c = Controller("run-once", expected_mode=3)
        for n in range(1, 6):
            self.assertEqual(c.consume(record(seq=n, tick_ms=n*100, motion_mode=3,
                source_valid=0, can_stack_words=200), n*.1), [])
            self.assertEqual(c.tick(n*.1), [])



class EntryPresetTests(unittest.TestCase):
    def ready(self, n, **extra):
        return record(seq=n, tick_ms=n*100, motion_mode=3, debug_plan_id=0, **extra)

    def test_select_ack_then_fresh_arm_start_only_once(self):
        for plan in [0,1,2,3,4,6]:
            c=Controller("run-once",3,plan)
            for n in range(1,4): out=c.consume(self.ready(n),n*.1)
            self.assertEqual(out,[command("PLAN"+str(plan),(3,300))])
            self.assertFalse(c.start_sent)
            # Same echo without an acceptance increment must not authorize ARM.
            r=self.ready(4);r["debug_plan_id"]=plan
            self.assertEqual(c.consume(r,.4),[])
            self.assertEqual(c.stage,"plan_wait")
            r=self.ready(5,remote_accepted=1);r["debug_plan_id"]=plan
            self.assertEqual(c.consume(r,.5),[])
            for n in range(6,9):
                r=self.ready(n,remote_accepted=1);r["debug_plan_id"]=plan
                out=c.consume(r,n*.1)
            self.assertEqual(out,[command("ARM",(8,800))])
            r=self.ready(9,remote_accepted=2,remote_state=1,token_seq=8,token_tick=800)
            r["debug_plan_id"]=plan
            self.assertEqual(c.consume(r,.9),[command("START",(8,800))])
            self.assertEqual(c.consume(r,1.0),[])

    def test_missing_plan_support_does_not_start(self):
        c=Controller("run-once",3,4)
        for n in range(1,4):
            r=self.ready(n);r.pop("debug_plan_id")
            c.consume(r,n*.1)
        self.assertEqual(c.status,"firmware_plan_unsupported")
        self.assertFalse(c.start_sent)

    def test_plan_ack_timeout_or_rejection_does_not_start(self):
        for reject in [False,True]:
            c=Controller("run-once",3,4)
            for n in range(1,4):c.consume(self.ready(n),n*.1)
            if reject:c.consume(self.ready(4,remote_result=6),.4)
            else:
                for n in range(4,24):c.consume(self.ready(n),n*.1)
                c.tick(2.4)
            self.assertIsNotNone(c.status)
            self.assertFalse(c.start_sent)

    def test_plan_only_allowed_for_debug_run_once(self):
        for action,mode,plan in [("monitor",3,1),("stop",3,1),("run-once",2,1),("run-once",3,5)]:
            with self.assertRaises(ValueError):Controller(action,mode,plan)

    def test_new_66_field_frame_and_old_65_still_parse(self):
        import p1_serial_motion as motion
        names=(motion.FIELDS+motion.SEQUENCE_FIELDS+motion.CAN_DIAG_FIELDS+
            motion.ACTION_FIELDS+motion.DISPLAY_FIELDS+motion.DISTANCE_FIELDS+
            ["distance_fault_reason","can_stack_words"])
        for plan in [None,0,4]:
            vals=[0]*len(names);vals[-1]=163
            if plan is not None:vals.append(plan)
            prefix=("TM1,"+",".join(map(str,vals))+",").encode()
            parsed=parse_telemetry(prefix+f"{binascii.crc_hqx(prefix,0xffff):04X}\r\n".encode())
            self.assertEqual(parsed["can_stack_words"],163)
            if plan is None:self.assertNotIn("debug_plan_id",parsed)
            else:self.assertEqual(parsed["debug_plan_id"],plan)

class RetiredLongPresetTests(unittest.TestCase):
    def test_wrong_237_request_is_not_remapped_to_210(self):
        import p1_serial_motion as motion
        self.assertEqual(motion.DEBUG_PLAN_IDS["forward210"],6)
        self.assertNotIn("forward237",motion.DEBUG_PLAN_IDS)
        with self.assertRaises(ValueError):command("PLAN5")
        self.assertTrue(command("PLAN6").startswith(b"RC1,"))


class BatchDiagnosticTests(unittest.TestCase):
    def make_record(self, **updates):
        from p1_serial_motion import BATCH_FIELDS
        r=dict.fromkeys(BATCH_FIELDS,0)
        r.update(session=1,batch_id=1,segment=2,generation_source=1,generated_ms=90,start_ms=100,end_ms=140,
                 outcome=1,sent_mask=15,ack_mask=15,base_rpm=15,correction_rpm=-1,pending_rpm=1,total_success=1)
        for wheel in range(1,5):
            r[f"send_ms{wheel}"]=100+(wheel-1)*10
            r[f"reply_ms{wheel}"]=105+(wheel-1)*10
            r[f"result{wheel}"]=1
        r.update(updates)
        return r

    def encode(self,r):
        from p1_serial_motion import BATCH_FIELDS
        prefix=("BT1,"+",".join(str(r[key]) for key in BATCH_FIELDS)+",").encode()
        return prefix+f"{binascii.crc_hqx(prefix,0xFFFF):04X}\r\n".encode()

    def test_completed_batch_and_signed_correction(self):
        from p1_serial_motion import parse_batch_telemetry
        r=parse_batch_telemetry(self.encode(self.make_record()))
        self.assertEqual((r["queue_ms"],r["batch_ms"],r["call_ms"]),(10,40,[5]*4))
        self.assertFalse(r["mechanical_response_measured"])
        self.assertIsNone(parse_batch_telemetry(b"TM1,0"))

    def test_zero_failure_and_unreturned_calls(self):
        from p1_serial_motion import parse_batch_telemetry
        r=parse_batch_telemetry(self.encode(self.make_record(outcome=2,sent_mask=3,ack_mask=1,
                    failed_address=2,io_result=0,result2=0)))
        self.assertEqual(r["call_ms"],[5,5,None,None])
        r=parse_batch_telemetry(self.encode(self.make_record(outcome=3,sent_mask=1,ack_mask=0)))
        self.assertEqual(r["call_ms"],[None]*4)

    def test_tick_wraparound(self):
        from p1_serial_motion import parse_batch_telemetry
        r=self.make_record(generated_ms=0xfffffff0,start_ms=0xfffffff8,end_ms=20)
        for wheel in range(1,5):
            r[f"send_ms{wheel}"]=(0xfffffff8+(wheel-1)*6)&0xffffffff
            r[f"reply_ms{wheel}"]=(r[f"send_ms{wheel}"]+2)&0xffffffff
        r=parse_batch_telemetry(self.encode(r))
        self.assertEqual((r["queue_ms"],r["batch_ms"],r["call_ms"]),(8,28,[2]*4))

    def test_invalid_crc_fields_flags_and_intervals(self):
        from p1_serial_motion import parse_batch_telemetry
        for updates in [dict(ack_mask=16),dict(outcome=1,ack_mask=7),dict(dry_run=2),
                        dict(generation_source=4),dict(correction_rpm=6),dict(reply_ms4=141),
                        dict(result1=0),dict(start_ms=80),dict(outcome=0)]:
            with self.subTest(updates=updates):
                with self.assertRaises(ValueError):parse_batch_telemetry(self.encode(self.make_record(**updates)))
        good=self.encode(self.make_record())
        for bad in [good[:-5]+b"0000\n",good.replace(b"BT1,1,",b"BT1,2,"),b"BT1,0,0000"]:
            with self.assertRaises(ValueError):parse_batch_telemetry(bad)

    def test_history_cannot_be_control_telemetry(self):
        from p1_serial_motion import parse_batch_telemetry
        payload=self.encode(self.make_record())
        self.assertIsNone(parse_telemetry(payload))
        self.assertIsNotNone(parse_batch_telemetry(payload))
        self.assertLess(len(self.encode(dict.fromkeys(__import__('p1_serial_motion').BATCH_FIELDS,0xffffffff))),448)


class ScurveClientCompatibilityTests(unittest.TestCase):
    def test_zero_idle_speed_does_not_arm(self):
        c=Controller("run-once",expected_mode=3)
        for n in range(1,5):
            self.assertEqual(c.consume(record(seq=n,tick_ms=n*100,motion_mode=3,base_rpm=0),n*.1),[])
        self.assertFalse(c.start_sent)

    def test_running_zero_profile_speed_remains_a_live_session(self):
        c=Controller("run-once",expected_mode=3)
        c.stage="running";c.start_sent=True;c.last_keep=1.0
        r=record(seq=10,tick_ms=1000,motion_mode=3,phase=2,base_rpm=0,backend_active=1,
                 remote_state=2,distance_mode=1,distance_base_rpm=0,distance_reached=0)
        self.assertEqual(c.consume(r,1.0),[])
        self.assertIsNone(c.status)
        self.assertEqual(c.tick(1.21),[command("KEEP",c.token)])


class Speed20PresetTests(unittest.TestCase):
    def low(self,n,**extra):
        return record(seq=n,tick_ms=n*100,motion_mode=3,debug_plan_id=0,**extra)

    def high(self,n,**extra):
        r=record(seq=n,tick_ms=n*100,motion_mode=3,debug_plan_id=7,base_rpm=52,
                 feedback_limit_rpm=80,sequence_count=1,remote_accepted=1,token_seq=7,token_tick=700)
        r.update(extra);return r

    def started(self):
        c=Controller('run-once',3,7)
        for n in range(1,4):out=c.consume(self.low(n),n*.1)
        self.assertEqual(out,[command('PLAN7',(3,300))])
        self.assertEqual(c.consume(self.high(4),.4),[])
        for n in range(5,8):out=c.consume(self.high(n),n*.1)
        self.assertEqual(out,[command('ARM',(7,700))])
        out=c.consume(self.high(8,remote_state=1,token_seq=7,token_tick=700),.8)
        self.assertEqual(out,[command('START',(7,700))])
        return c

    def test_plan7_explicit_selection_arm_and_single_start(self):
        import p1_serial_motion as m
        self.assertEqual(m.DEBUG_PLAN_IDS['forward210-20'],7)
        c=self.started();self.assertTrue(c.start_sent)
        self.assertEqual(c.consume(self.high(9,remote_state=1,token_seq=7,token_tick=700),.9),[])

    def test_default_or_unspecified_run_cannot_arm_speed20(self):
        for plan in [None,0]:
            c=Controller('run-once',3,plan)
            for n in range(1,6):
                out=c.consume(self.high(n),n*.1)
                self.assertFalse(any(b',ARM,' in msg or b',START,' in msg for msg in out))
            self.assertFalse(c.start_sent)
            self.assertNotEqual(c.stage,'arm_wait')

    def test_wrong_speed_feedback_or_action_count_cannot_arm(self):
        for bad in [dict(base_rpm=49),dict(feedback_limit_rpm=30),dict(feedback_limit_rpm=81),dict(sequence_count=2)]:
            c=Controller('run-once',3,7);c.plan_selected=True
            for n in range(1,6):self.assertEqual(c.consume(self.high(n,**bad),n*.1),[])
            self.assertFalse(c.start_sent)

    def test_speed20_legacy_echo_after_selection_cannot_arm(self):
        c=Controller('run-once',3,7);c.plan_selected=True
        for n in range(1,6):self.assertEqual(c.consume(self.low(n),n*.1),[])
        self.assertFalse(c.start_sent)

    def test_runtime_profile_mutation_stops_once(self):
        for bad in [dict(debug_plan_id=6),dict(base_rpm=53),dict(feedback_limit_rpm=30),
                    dict(sequence_count=3),dict(distance_base_rpm=53),
                    dict(distance_mode=1,action_kind=5,distance_target_milli_mm=2100000),
                    dict(distance_mode=1,action_kind=4,distance_target_milli_mm=300000)]:
            c=self.started();r=self.high(9,phase=2,remote_state=2);r.update(bad)
            self.assertEqual(c.consume(r,.9),[command('STOP',c.token)])
            self.assertEqual(c.status,'speed20_profile_changed')
            self.assertEqual(c.tick(1.5),[])

    def test_zero_at_start_and_final_approach_are_live_not_done(self):
        c=self.started()
        for n,base in [(9,0),(10,1),(11,49),(12,1)]:
            r=self.high(n,phase=2,remote_state=2,base_rpm=base,distance_mode=1,
                        distance_base_rpm=base,action_kind=4,distance_target_milli_mm=2100000,
                        sequence_state=1,sequence_completed=0,distance_reached=0)
            self.assertEqual(c.consume(r,n*.1),[]);self.assertIsNone(c.status)
        r=self.high(13,phase=3,remote_state=3,base_rpm=1,backend_result=1,stop_mask=15,
                    sequence_state=3,sequence_completed=1,distance_mode=1,distance_reached=1,
                    action_kind=4,distance_target_milli_mm=2100000)
        self.assertEqual(c.consume(r,1.3),[]);self.assertEqual(c.status,'software_complete')

    def test_speed20_still_stops_on_loss_after_600ms(self):
        c=self.started();self.assertEqual(c.tick(1.41),[command('STOP',c.token)])
        self.assertEqual(c.status,'telemetry_timeout')

    def test_fault_not_reclassified_as_profile_change(self):
        c=self.started();r=self.high(9,phase=4,feedback_limit_rpm=30)
        c.consume(r,.9);self.assertEqual(c.status,'motion_fault_or_cancelled')

    def test_monitor_high_speed_is_always_read_only(self):
        c=Controller('monitor',3)
        for n in range(1,5):self.assertEqual(c.consume(self.high(n),n*.1),[])
        self.assertEqual(c.tick(10),[])

class RouteTrialTests(unittest.TestCase):
    def ready(self, n, **updates):
        data = record(seq=n, tick_ms=n*100, motion_mode=3, debug_plan_id=0,
                      feedback_limit_rpm=80, base_rpm=51, sequence_count=3,
                      token_seq=7, token_tick=700)
        data.update(updates)
        return data

    def started(self):
        client = Controller('run-once', 3, 0, route_trial=True)
        for n in range(1, 4):
            out = client.consume(self.ready(n), n*.1)
        self.assertEqual(out, [command('PLAN0', (3, 300))])
        client.consume(self.ready(4, remote_accepted=1), .4)
        for n in range(5, 8):
            out = client.consume(self.ready(n, remote_accepted=1), n*.1)
        self.assertEqual(out, [command('ARM', (7, 700))])
        out = client.consume(self.ready(8, remote_state=1, remote_accepted=2,
                                        token_seq=7, token_tick=700), .8)
        self.assertEqual(out, [command('START', (7, 700))])
        return client

    def test_candidate_requires_explicit_opt_in(self):
        for plan in [None, 0]:
            client = Controller('run-once', 3, plan)
            for n in range(1, 8):
                self.assertEqual(client.consume(self.ready(n), n*.1), [])
            self.assertFalse(client.start_sent)

    def test_explicit_low_preset_can_leave_candidate_before_arming(self):
        client = Controller('run-once', 3, 1)
        for n in range(1, 4):
            out = client.consume(self.ready(n), n*.1)
        self.assertEqual(out, [command('PLAN1', (3, 300))])
        self.assertFalse(client.start_sent)
        client.consume(self.ready(4, debug_plan_id=1, feedback_limit_rpm=30,
                                  base_rpm=15, sequence_count=1, remote_accepted=1), .4)
        for n in range(5, 8):
            out = client.consume(self.ready(n, debug_plan_id=1, feedback_limit_rpm=30,
                                            base_rpm=15, sequence_count=1), n*.1)
        self.assertEqual(out, [command('ARM', (7, 700))])
        self.assertFalse(client.start_sent)

    def test_opt_in_requires_explicit_plan_and_mode(self):
        for action, mode, plan in [('monitor', 3, 0), ('run-once', 2, 0),
                                   ('run-once', 3, None), ('run-once', 3, 7)]:
            with self.assertRaises(ValueError):
                Controller(action, mode, plan, route_trial=True)

    def test_all_directions_and_zero_speed_tail_are_accepted(self):
        client = self.started()
        for n, kind in enumerate([4, 5, 6], 9):
            frame = self.ready(n, phase=2, remote_state=2, distance_mode=1,
                               distance_base_rpm=0, base_rpm=0, action_kind=kind,
                               distance_target_milli_mm=2100000)
            self.assertEqual(client.consume(frame, n*.1), [])
            self.assertIsNone(client.status)

    def test_profile_changes_stop_instead_of_continuing(self):
        bad_updates = [dict(debug_plan_id=7), dict(feedback_limit_rpm=30),
                       dict(sequence_count=4), dict(base_rpm=53), dict(distance_base_rpm=53),
                       dict(distance_mode=1, action_kind=0, distance_target_milli_mm=300000),
                       dict(distance_mode=1, action_kind=5, distance_target_milli_mm=2100001)]
        for updates in bad_updates:
            client = self.started()
            frame = self.ready(9, phase=2, remote_state=2)
            frame.update(updates)
            self.assertEqual(client.consume(frame, .9), [command('STOP', client.token)])
            self.assertEqual(client.status, 'route_trial_profile_changed')

    def test_low_profile_echo_cannot_arm_candidate(self):
        client = Controller('run-once', 3, 0, route_trial=True)
        for n in range(1, 4):
            client.consume(self.ready(n), n*.1)
        client.consume(self.ready(4, remote_accepted=1), .4)
        for n in range(5, 10):
            self.assertEqual(client.consume(self.ready(n, feedback_limit_rpm=30, base_rpm=15), n*.1), [])
        self.assertFalse(client.start_sent)

    def test_changed_count_before_arm_does_not_start(self):
        client = Controller('run-once', 3, 0, route_trial=True)
        for n in range(1, 4):
            client.consume(self.ready(n), n*.1)
        client.consume(self.ready(4, remote_accepted=1), .4)
        for n in range(5, 10):
            self.assertEqual(client.consume(self.ready(n, sequence_count=4), n*.1), [])
        self.assertFalse(client.start_sent)

    def test_link_loss_and_fault_keep_existing_stop_behavior(self):
        client = self.started()
        self.assertEqual(client.tick(1.41), [command('STOP', client.token)])
        self.assertEqual(client.status, 'telemetry_timeout')
        client = self.started()
        client.consume(self.ready(9, phase=4, trip=5), .9)
        self.assertEqual(client.status, 'motion_fault_or_cancelled')

    def test_early_normal_backend_stop_does_not_mean_distance_completed(self):
        client = self.started()
        client.consume(self.ready(9, phase=3, backend_result=1, stop_mask=15, base_rpm=0,
                                  sequence_state=3, sequence_completed=3, distance_mode=1,
                                  action_kind=6, distance_target_milli_mm=300000, distance_reached=0), .9)
        self.assertEqual(client.status, 'distance_not_reached')


if __name__=="__main__":unittest.main()
