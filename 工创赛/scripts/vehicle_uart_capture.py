"""USART1只读采集/离线重放：不发送启动、心跳、调参或停止命令。

原始字节是证据，解析表是派生结果。现场阶段标签不能替代固件阶段确认。
"""
from __future__ import annotations

import argparse
import base64
import binascii
import csv
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import time

import p1_serial_motion as protocol

ROOT = Path(__file__).resolve().parents[1]
ACTUAL_FIRMWARE = Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
PARSERS = [getattr(protocol, name) for name in (
    "parse_telemetry", "parse_fault_telemetry", "parse_batch_telemetry",
    "parse_timing_telemetry", "parse_perimeter_checkpoint",
    "parse_odometry_telemetry", "parse_position_telemetry", "parse_position_endpoints",
)]
MAX_LINE = 4096
WHEEL_FIELDS = ["seq", "tick_ms", "session", "active", "state", "dry_run",
                "speed_valid_mask", "ack_valid_mask", "telemetry_dropped"] + [
    f"wheel{wheel}_{field}" for wheel in range(1, 5) for field in
    ("acknowledged_decirpm", "measured_decirpm", "ack_ms", "sample_ms", "age_ms")
]
STAGE_FIELDS = ["seq", "tick_ms", "stage_id", "run_id", "stage_state", "zone",
                "action_index", "actions_completed", "action_count", "start_x_mm", "start_y_mm",
                "stop_x_mm", "stop_y_mm", "config_id", "scan_valid", "cause", "elapsed_ms",
                "backend_session", "denied", "telemetry_dropped", "nominal_scan_gap_deci_mm"]
MISSION_FIELDS = ["seq", "tick_ms", "run_id", "stage", "state", "cause", "startup_state",
                  "startup_cause", "arm_row", "item", "arm_state", "arm_cause",
                  "reference_feedback_mask", "extension_motor_millideg", "lift_motor_millideg"]


def parse_mission(line: bytes):
    if not line.startswith(b"VM1,"):
        return None
    body, separator, checksum = line.rstrip(b"\r\n").rpartition(b",")
    if not separator or len(checksum) != 4 or binascii.crc_hqx(body + b",", 0xffff) != int(checksum, 16):
        raise ValueError("VM1 CRC mismatch")
    values = body.decode("ascii").split(",")[1:]
    if len(values) != len(MISSION_FIELDS):
        raise ValueError("VM1 field count")
    result = dict(zip(MISSION_FIELDS, map(int, values)))
    for field, value in result.items():
        if field.endswith("millideg"):
            if not -(2**31) <= value < 2**31:
                raise ValueError("VM1 angle range")
        elif not 0 <= value <= 0xffffffff:
            raise ValueError("VM1 uint32 range")
    if (not 1 <= result["stage"] <= 8 or result["state"] > 7 or
            result["startup_state"] > 8 or result["arm_row"] >= 32 or result["item"] > 3 or
            result["arm_state"] > 3 or result["reference_feedback_mask"] > 3):
        raise ValueError("VM1 state range")
    return result


def parse_stage(line: bytes):
    code_frame = line.startswith(b"SC1,")
    if not code_frame and not line.startswith(b"ST1,"):
        return None
    body, separator, checksum = line.rstrip(b"\r\n").rpartition(b",")
    if not separator or len(checksum) != 4 or binascii.crc_hqx(body + b",", 0xffff) != int(checksum, 16):
        raise ValueError("stage CRC mismatch")
    values = body.decode("ascii").split(",")[1:]
    fields = ["seq", "tick_ms", "stage_id", "run_id", "config_id", "task_code"] if code_frame else STAGE_FIELDS
    if len(values) != len(fields):
        raise ValueError("stage field count")
    result = {key: value if key == "task_code" else int(value) for key, value in zip(fields, values)}
    if any(not 0 <= value <= 0xffffffff for key, value in result.items() if key != "task_code"):
        raise ValueError("stage uint32 range")
    if result["stage_id"] != 1:
        raise ValueError("unsupported stage schema")
    if code_frame:
        groups = result["task_code"].split("+")
        if (len(groups) != 4 or any(len(g) != 3 for g in groups) or
                any(set(g) - set("123456") or len(set(g)) != 3 for g in groups) or
                set(groups[1]) != set("123") or set(groups[3]) != set("123") or
                set(groups[0]) != set(groups[2]) or not result["run_id"]):
            raise ValueError("invalid stage task code")
    elif (result["stage_state"] > 7 or result["zone"] != 1 or result["scan_valid"] > 1 or
          result["action_count"] > 8 or result["actions_completed"] > result["action_count"] or
          any(result[name] >= 2400 for name in ("start_x_mm", "start_y_mm", "stop_x_mm", "stop_y_mm"))):
        raise ValueError("stage state/geometry")
    return result


def parse_wheel(line: bytes):
    if not line.startswith(b"WF1,"):
        return None
    body, separator, checksum = line.rstrip(b"\r\n").rpartition(b",")
    if not separator or len(checksum) != 4 or binascii.crc_hqx(body + b",", 0xffff) != int(checksum, 16):
        raise ValueError("WF1 CRC mismatch")
    values = [int(value) for value in body.split(b",")[1:]]
    if len(values) != len(WHEEL_FIELDS):
        raise ValueError("WF1 field count")
    result = dict(zip(WHEEL_FIELDS, values))
    for key, value in result.items():
        if key.endswith("decirpm"):
            if not -(2**31) <= value < 2**31:
                raise ValueError("WF1 RPM range")
        elif not 0 <= value <= 0xffffffff:
            raise ValueError("WF1 uint32 range")
    if result["active"] > 1 or result["dry_run"] > 1 or result["speed_valid_mask"] > 15 or result["ack_valid_mask"] > 15:
        raise ValueError("WF1 flags")
    if result["dry_run"] and (result["speed_valid_mask"] or result["ack_valid_mask"]):
        raise ValueError("WF1 simulated values cannot be marked real")
    for wheel in range(1, 5):
        age = (result["tick_ms"] - result[f"wheel{wheel}_sample_ms"]) & 0xffffffff
        if age != result[f"wheel{wheel}_age_ms"]:
            raise ValueError("WF1 sample time/age mismatch")
    return result


def decode(line: bytes) -> tuple[str, dict]:
    """CRC正确与物理动作成功是不同概念；不推测缺失的反馈。"""
    for parser in [parse_mission, parse_stage, parse_wheel, *PARSERS]:
        result = parser(line)
        if result is not None:
            return line.split(b",", 1)[0].decode("ascii"), result
    body = line.rstrip(b"\r\n")
    prefix, separator, checksum = body.rpartition(b",")
    # 保留新增协议，先做CRC核验；未知字段只编号，不猜测单位/意义。
    tag_bytes = body.split(b",", 1)[0]
    if (separator and len(checksum) == 4 and re.fullmatch(rb"[A-Z][A-Z0-9]{1,7}", tag_bytes)
            and tag_bytes.endswith((b"1", b"2"))):
        if binascii.crc_hqx(prefix + b",", 0xFFFF) != int(checksum, 16):
            raise ValueError("unknown_protocol_crc_mismatch")
        tag, *values = prefix.decode("ascii").split(",")
        return tag, {f"field_{i}": value for i, value in enumerate(values)}
    return "TEXT", {"text": body.decode("ascii", errors="replace")}


class Capture:
    """有界分帧；CRC错误、截断、重复、时钟倒退、序号缺口全部留记录。"""
    def __init__(self, out: Path, stage: int):
        out.mkdir(parents=True, exist_ok=False)
        self.out, self.stage = out, stage
        self.raw = (out / "uart1.raw").open("wb")
        self.events = (out / "frames.jsonl").open("w", encoding="utf8", newline="\n")
        self.pending = bytearray()
        self.discarding = False
        self.offset = 0
        self.total_bytes = 0
        self.rows = self.errors = self.unknown = 0
        self.gaps = self.duplicates = self.reorders = self.clock_rewinds = 0
        self.stage_records = self.stage_mismatches = 0
        self.last_seq = self.last_tick = None
        self.tables = {}
        self.started_ns = time.monotonic_ns()
        self.last_flush = time.monotonic()

    def emit(self, line: bytes, now_ns: int, error: str | None = None):
        tag, data = "ERROR", {}
        try:
            if error:
                raise ValueError(error)
            tag, data = decode(line)
        except (ValueError, UnicodeError, OverflowError) as exc:
            error = str(exc)
            self.errors += 1
        row = {
            "host_utc": datetime.now(timezone.utc).isoformat(),
            "host_monotonic_ns": now_ns,
            "host_elapsed_ns": now_ns - self.started_ns,
            "raw_offset": self.offset,
            "raw_length": len(line),
            "operator_stage_label": self.stage,
            "firmware_stage_confirmed": False,
            "tag": tag, "error": error,
        }
        if not error and tag in ("ST1", "SC1", "VM1"):
            firmware_stage = data["stage"] if tag == "VM1" else data["stage_id"]
            row["firmware_stage_id"] = firmware_stage
            row["firmware_stage_confirmed"] = self.stage in (0, firmware_stage)
            row["stage_label_mismatch"] = self.stage not in (0, firmware_stage)
            self.stage_records += 1
            self.stage_mismatches += int(row["stage_label_mismatch"])
        # 只对TM1使用其明确规定的seq/tick；其它帧不能拿批次号当全局序号。
        if tag == "TM1":
            seq, tick = data["seq"], data["tick_ms"]
            if self.last_tick is not None and ((tick - self.last_tick) & 0xFFFFFFFF) >= 0x80000000:
                row["clock_rewind"] = True
                self.clock_rewinds += 1
                # 这里只能确认时钟倒退，不能单凭它断言主控发生复位。
                self.last_seq = self.last_tick = None
            if self.last_seq is not None:
                delta = (seq - self.last_seq) & 0xFFFFFFFF
                if delta == 0:
                    row["duplicate"] = True
                    self.duplicates += 1
                elif delta >= 0x80000000:
                    row["out_of_order"] = True
                    self.reorders += 1
                else:
                    row["sequence_gap"] = delta - 1
                    self.gaps += delta - 1
            if not row.get("out_of_order") and not row.get("duplicate"):
                self.last_seq, self.last_tick = seq, tick
        if tag != "TEXT" and tag != "ERROR" and any(k.startswith("field_") for k in data):
            self.unknown += 1
        row["data"] = data
        if error:
            row["raw_base64"] = base64.b64encode(line).decode("ascii")
        self.events.write(json.dumps(row, ensure_ascii=False) + "\n")
        self.offset += len(line)
        self.rows += 1
        if not error:
            # 同一协议不同版本字段分表，防止旧/新列错位。
            fields = list(data)
            schema = hashlib.sha256(",".join(fields).encode()).hexdigest()[:8]
            key = tag, schema
            if key not in self.tables:
                stream = (self.out / f"{tag}_{schema}.csv").open("w", encoding="utf-8-sig", newline="")
                writer = csv.DictWriter(stream, fieldnames=[
                    "host_utc", "host_monotonic_ns", "raw_offset", "operator_stage_label",
                    "sequence_gap", "duplicate", "out_of_order", "clock_rewind", *fields,
                ])
                writer.writeheader()
                self.tables[key] = stream, writer
            self.tables[key][1].writerow({
                "host_utc": row["host_utc"], "host_monotonic_ns": now_ns,
                "raw_offset": row["raw_offset"], "operator_stage_label": self.stage,
                "sequence_gap": row.get("sequence_gap", 0), "duplicate": row.get("duplicate", False),
                "out_of_order": row.get("out_of_order", False), "clock_rewind": row.get("clock_rewind", False), **data,
            })

    def feed(self, chunk: bytes):
        self.raw.write(chunk)
        self.total_bytes += len(chunk)
        now_ns = time.monotonic_ns()
        # 原始文件保留全部字节；过长行按块报告，内存不会无限增长。
        for byte in chunk:
            self.pending.append(byte)
            if byte == 10:
                self.emit(bytes(self.pending), now_ns, "oversize_continuation" if self.discarding else None)
                self.pending.clear()
                self.discarding = False
            elif len(self.pending) >= MAX_LINE:
                self.emit(bytes(self.pending), now_ns, "oversize_frame")
                self.pending.clear()
                self.discarding = True
        if time.monotonic() - self.last_flush >= 1:
            self.flush()

    def flush(self):
        self.raw.flush()
        os.fsync(self.raw.fileno())
        self.events.flush()
        for stream, _ in self.tables.values():
            stream.flush()
        self.last_flush = time.monotonic()

    def close(self, reason: str):
        if self.pending:
            self.emit(bytes(self.pending), time.monotonic_ns(), "truncated_at_capture_end")
            self.pending.clear()
        self.flush()
        self.raw.close()
        self.events.close()
        for stream, _ in self.tables.values():
            stream.close()
        with (self.out / "uart1.raw").open("rb") as stream:
            raw_hash = hashlib.file_digest(stream, "sha256").hexdigest()
        summary = {
            "reason": reason, "raw_bytes": self.total_bytes, "records": self.rows,
            "invalid_records": self.errors, "unknown_crc_valid_frames": self.unknown,
            "tm1_sequence_missing": self.gaps, "tm1_duplicates": self.duplicates,
            "tm1_out_of_order": self.reorders, "tm1_clock_rewinds": self.clock_rewinds,
            "serial_commands_sent": 0, "operator_stage_label": self.stage,
            "firmware_stage_confirmed": self.stage_records > 0 and self.stage_mismatches == 0,
            "firmware_stage_records": self.stage_records,
            "stage_label_mismatches": self.stage_mismatches,
            "raw_sha256": raw_hash,
        }
        (self.out / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf8")
        return summary


def source_manifest():
    """记录本地构建来源；不把本地源码哈希冒充板上固件身份。"""
    fw = ACTUAL_FIRMWARE
    files = sorted(p for folder in ("USER", "HARDWARE", "SYSTEM", "CHASSIS")
                   for p in (fw / folder).rglob("*") if p.suffix.lower() in (".c", ".h", ".inc"))
    files += sorted(fw.glob("*.uvprojx"))
    hashes = {str(p.relative_to(fw)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    try:
        commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    except (OSError, subprocess.CalledProcessError):
        commit = None
    return {"workspace_git_head": commit, "firmware_source_directory": str(fw), "local_files_sha256": hashes,
            "source_set_sha256": hashlib.sha256(json.dumps(hashes, sort_keys=True).encode()).hexdigest(),
            "matches_running_firmware": "unverified"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--port", help="无线接收模块的COM口；只读，不发任何命令")
    source.add_argument("--replay", type=Path, help="离线重放已保存的uart1.raw")
    parser.add_argument("--stage", type=int, choices=range(0, 9), required=True,
                        help="1~8为单段；0记录整场并采用固件VM1实际阶段标签")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--seconds", type=float, default=0, help="0表示一直记录，Ctrl+C结束")
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    if args.seconds < 0 or args.baud <= 0:
        parser.error("seconds必须非负，baud必须为正")
    out = args.out or ROOT / "evidence/vehicle-stage" / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    capture = Capture(out, args.stage)
    reason = "finished"
    try:
        manifest = {"stage_label_source": "operator_argument", "stage": args.stage,
                    "port": args.port, "baud": args.baud, "replay": str(args.replay) if args.replay else None,
                    "read_only": True, **source_manifest()}
        (out / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf8")
        if args.replay:
            with args.replay.open("rb") as stream:
                while chunk := stream.read(8192):
                    capture.feed(chunk)
        else:
            import serial  # 离线重放不需要安装pyserial。
            with serial.Serial(port=None, baudrate=args.baud, timeout=0.2) as stream:
                stream.dtr = False
                stream.rts = False
                stream.port = args.port
                stream.open()
                started = time.monotonic()
                while not args.seconds or time.monotonic() - started < args.seconds:
                    capture.feed(stream.read(max(1, min(8192, stream.in_waiting))))
    except KeyboardInterrupt:
        reason = "operator_stopped_capture"
    except Exception as exc:
        reason = f"capture_error:{type(exc).__name__}:{exc}"
        raise
    finally:
        print(json.dumps({"output": str(out), **capture.close(reason)}, ensure_ascii=False))


if __name__ == "__main__":
    main()
