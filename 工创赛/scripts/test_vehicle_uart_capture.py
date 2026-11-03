"""离线核对日志不会掩盖坏帧、时钟变化或采集末尾截断。"""
import binascii
import csv
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import p1_serial_motion as protocol
import vehicle_uart_capture as capture


def frame(seq=1, tick=100):
    values = {name: 0 for name in protocol.FIELDS}
    values.update(seq=seq, tick_ms=tick)
    body = ("TM1," + ",".join(str(values[name]) for name in protocol.FIELDS) + ",").encode()
    return body + f"{binascii.crc_hqx(body, 0xffff):04X}\r\n".encode()


class CaptureChecks(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.out = Path(self.tmp.name) / "capture"
        self.log = capture.Capture(self.out, 3)

    def tearDown(self):
        if not self.log.raw.closed:
            self.log.close("test_cleanup")
        self.tmp.cleanup()

    def records(self):
        return [json.loads(line) for line in (self.out / "frames.jsonl").read_text(encoding="utf8").splitlines()]

    def test_integrated_stage_and_reference_reads_are_not_operator_guesses(self):
        values = dict.fromkeys(capture.MISSION_FIELDS, 0)
        values.update(seq=1, tick_ms=123, run_id=1, stage=3, startup_state=8,
                      reference_feedback_mask=3, extension_motor_millideg=-12345,
                      lift_motor_millideg=67890)
        body = ("VM1," + ",".join(str(values[k]) for k in capture.MISSION_FIELDS) + ",").encode()
        wire = body + f"{binascii.crc_hqx(body, 0xffff):04X}\r\n".encode()
        self.log.feed(wire)
        self.log.close("test")
        record = self.records()[0]
        self.assertTrue(record["firmware_stage_confirmed"])
        self.assertEqual(record["firmware_stage_id"], 3)
        parsed = capture.parse_mission(wire)
        self.assertEqual(parsed["extension_motor_millideg"], -12345)
        self.assertEqual(parsed["reference_feedback_mask"], 3)
        with self.assertRaises(ValueError):
            capture.parse_mission(wire.replace(b"VM1,1,", b"VM1,2,", 1))

    def test_whole_run_label_uses_each_firmware_stage(self):
        self.log.close("unused")
        self.log = capture.Capture(Path(self.tmp.name) / "whole", 0)
        for stage in (1, 2, 8):
            values = dict.fromkeys(capture.MISSION_FIELDS, 0)
            values.update(stage=stage)
            body = ("VM1," + ",".join(str(values[k]) for k in capture.MISSION_FIELDS) + ",").encode()
            self.log.feed(body + f"{binascii.crc_hqx(body, 0xffff):04X}\r\n".encode())
        summary = self.log.close("test")
        self.assertTrue(summary["firmware_stage_confirmed"])

    def test_fragmented_raw_is_exact_and_no_firmware_stage_invented(self):
        raw = frame() + b"F407 READY\r\n"
        for start in range(0, len(raw), 7):
            self.log.feed(raw[start:start + 7])
        result = self.log.close("test")
        self.assertEqual((self.out / "uart1.raw").read_bytes(), raw)
        records = self.records()
        self.assertEqual([r["tag"] for r in records], ["TM1", "TEXT"])
        self.assertFalse(records[0]["firmware_stage_confirmed"])
        self.assertEqual(result["invalid_records"], 0)
        self.assertEqual(records[1]["raw_offset"], len(frame()))

    def test_bad_crc_and_truncated_tail_are_not_valid_rows(self):
        raw = frame().replace(b"TM1,1,", b"TM1,2,", 1) + frame()[:-3]
        self.log.feed(raw)
        result = self.log.close("test")
        self.assertEqual(result["invalid_records"], 2)
        self.assertIn("CRC", self.records()[0]["error"])
        self.assertEqual(self.records()[1]["error"], "truncated_at_capture_end")
        self.assertEqual(list(self.out.glob("TM1_*.csv")), [])

    def test_gap_duplicate_out_of_order_and_wrap_are_marked(self):
        for seq, tick in [(0xfffffffe, 0xfffffffe), (0, 0), (0, 0), (0xffffffff, 0), (1, 1)]:
            # Existing TM1 wire representation uses signed int32 text.
            self.log.feed(frame(seq if seq < 2**31 else seq - 2**32,
                                tick if tick < 2**31 else tick - 2**32))
        result = self.log.close("test")
        self.assertEqual(result["tm1_sequence_missing"], 1)
        self.assertEqual(result["tm1_duplicates"], 1)
        self.assertEqual(result["tm1_out_of_order"], 1)
        self.assertEqual(result["tm1_clock_rewinds"], 0)
        with next(self.out.glob("TM1_*.csv")).open(encoding="utf-8-sig", newline="") as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual(rows[1]["sequence_gap"], "1")
        self.assertEqual(rows[2]["duplicate"], "True")

    def test_clock_rewind_is_distinct_from_massive_packet_loss(self):
        self.log.feed(frame(500, 1000) + frame(1, 1))
        result = self.log.close("test")
        self.assertEqual(result["tm1_clock_rewinds"], 1)
        self.assertEqual(result["tm1_sequence_missing"], 0)

    def test_oversize_line_is_bounded_and_next_frame_survives(self):
        raw = b"x" * (capture.MAX_LINE * 3 + 5) + b"\n" + frame()
        self.log.feed(raw)
        result = self.log.close("test")
        self.assertLess(len(self.log.pending), capture.MAX_LINE)
        self.assertEqual(result["invalid_records"], 4)
        self.assertEqual(self.records()[-1]["tag"], "TM1")
        self.assertEqual(sum(r["raw_length"] for r in self.records()), len(raw))

    def test_live_capture_has_no_transmit_method(self):
        # 假串口不提供write：若采集器尝试发送任何命令，测试立即失败。
        class ReadOnlySerial:
            in_waiting = 0
            def __init__(self, **kwargs): self.calls = 0
            def __enter__(self): return self
            def __exit__(self, *args): pass
            def open(self): pass
            def read(self, size):
                self.calls += 1
                if self.calls == 1: return frame()
                raise KeyboardInterrupt
        module = type("SerialModule", (), {"Serial": ReadOnlySerial})
        out = Path(self.tmp.name) / "live"
        with mock.patch.dict("sys.modules", {"serial": module}), mock.patch.object(capture, "source_manifest", return_value={}), mock.patch("sys.argv", ["capture", "--port", "FAKE", "--stage", "1", "--out", str(out)]):
            capture.main()
        self.assertEqual(json.loads((out / "summary.json").read_text(encoding="utf8"))["serial_commands_sent"], 0)

    def test_wheel_commands_feedback_and_sample_time_are_separate(self):
        values = dict.fromkeys(capture.WHEEL_FIELDS, 0)
        values.update(seq=4, tick_ms=1000, session=2, active=1, speed_valid_mask=15, ack_valid_mask=15)
        for wheel in range(1, 5):
            values.update({f"wheel{wheel}_acknowledged_decirpm": -520,
                           f"wheel{wheel}_measured_decirpm": -500,
                           f"wheel{wheel}_ack_ms": 900,
                           f"wheel{wheel}_sample_ms": 990 - wheel,
                           f"wheel{wheel}_age_ms": 10 + wheel})
        def encode():
            body = ("WF1," + ",".join(str(values[name]) for name in capture.WHEEL_FIELDS) + ",").encode()
            return body + f"{binascii.crc_hqx(body, 0xffff):04X}\r\n".encode()
        record = capture.parse_wheel(encode())
        self.assertEqual(record["wheel1_acknowledged_decirpm"], -520)
        self.assertEqual(record["wheel1_measured_decirpm"], -500)
        values["wheel1_age_ms"] = 0
        with self.assertRaisesRegex(ValueError, "age mismatch"):
            capture.parse_wheel(encode())
        values["wheel1_age_ms"] = 11
        values["dry_run"] = 1
        with self.assertRaisesRegex(ValueError, "simulated"):
            capture.parse_wheel(encode())

    def test_untrusted_protocol_tag_cannot_create_path(self):
        body = b"../../outside1,1,"
        self.log.feed(body + f"{binascii.crc_hqx(body, 0xffff):04X}\n".encode())
        self.log.close("test")
        self.assertEqual(self.records()[0]["tag"], "TEXT")
        self.assertEqual(list(Path(self.tmp.name).glob("outside*")), [])

    def test_stage_identity_and_task_code_require_valid_crc_schema(self):
        def encode(body):
            return body + f"{binascii.crc_hqx(body, 0xffff):04X}\r\n".encode()
        good = encode(b"SC1,1,1000,1,1,12345,156+123+516+231,")
        self.assertEqual(capture.parse_stage(good)["task_code"], "156+123+516+231")
        with self.assertRaisesRegex(ValueError, "task code"):
            capture.parse_stage(encode(b"SC1,1,1000,1,1,12345,156+123+516+236,"))
        self.log.feed(good)
        self.log.close("test")
        # 本测试采集标签为3，但固件说阶段1，必须显式暴露不匹配。
        self.assertFalse(self.records()[0]["firmware_stage_confirmed"])
        self.assertTrue(self.records()[0]["stage_label_mismatch"])


if __name__ == "__main__":
    unittest.main()
