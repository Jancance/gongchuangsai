"""CRC telemetry and bounded one-shot serial motion client. Default action only monitors."""
import argparse
import binascii
import ctypes
import json
import os
from p1_x_load_telemetry import parse_load_telemetry, parse_integral_telemetry, LoadAccumulator, analyze_load
from pathlib import Path
import time
from datetime import datetime, timezone

FIELDS = ['seq', 'tick_ms', 'phase', 'trip', 'elapsed_ms', 'source_valid', 'yaw_mdeg', 'target_mdeg', 'error_mdeg', 'rate_mdps', 'angle_age_ms', 'rate_age_ms', 'correction_rpm', 'rpm1', 'rpm2', 'rpm3', 'rpm4', 'vm_request', 'pe2_active', 'backend_state', 'backend_active', 'backend_result', 'backend_cause', 'stop_mask', 'can_fault', 'rtos_fault', 'safety_fault', 'motion_mode', 'nonzero_samples', 'peak_abs_rpm', 'last_nonzero_rpm', 'remote_state', 'remote_result', 'token_seq', 'token_tick', 'telemetry_dropped', 'remote_rejected', 'selftest', 'base_rpm', 'feedback_limit_rpm', 'peak_error_mdeg', 'max_sample_gap_ms', 'uart_rx_errors', 'rx_stack_words', 'heap_min_bytes', 'link_lost', 'protocol_selftest', 'remote_accepted']
SEQUENCE_FIELDS = ["sequence_state", "sequence_index", "sequence_completed"]
DISPLAY_FIELDS = ["action_kind", "target_theoretical_milli_cm_s"]
DISTANCE_FIELDS = ["distance_mode", "distance_target_milli_mm", "distance_progress_milli_mm", "distance_base_rpm", "distance_reached"]
ACTION_FIELDS = ["sequence_count"]
CAN_DIAG_FIELDS = ["can_guard_cause", "can_guard_bad_nodes", "power_wait_state", "post_settle_mask"]
DEBUG_PLAN_IDS = {"default": 0, "left30": 1, "forward30": 2, "right30": 3, "route-a": 4, "forward210": 6, "forward210-20": 7}
UINT_FIELDS = {"seq", "tick_ms", "token_seq", "token_tick"}


def command(verb, token=(0, 0)):
    if verb not in {"ARM", "START", "KEEP", "STOP", "PLAN0", "PLAN1", "PLAN2", "PLAN3", "PLAN4", "PLAN6", "PLAN7"}:
        raise ValueError("unsupported command")
    seq, tick = token
    if not all(0 <= n <= 0xFFFFFFFF for n in token):
        raise ValueError("token outside uint32")
    prefix = f"RC1,{seq},{tick},{verb},".encode("ascii")
    return prefix + f"{binascii.crc_hqx(prefix, 0xFFFF):04X}\n".encode("ascii")


def parse_telemetry(line):
    line = line.rstrip(b"\r\n")
    if not line.startswith(b"TM8,"):
        return None
    if len(line) >= 768:
        raise ValueError("frame too long")
    prefix, sep, checksum = line.rpartition(b",")
    if not sep or len(checksum) != 4:
        raise ValueError("missing CRC")
    try:
        received_crc = int(checksum, 16)
    except ValueError as exc:
        raise ValueError("invalid CRC") from exc
    if binascii.crc_hqx(prefix + b",", 0xFFFF) != received_crc:
        raise ValueError("CRC mismatch")
    values = prefix.split(b",")[1:]
    names = (FIELDS + SEQUENCE_FIELDS + CAN_DIAG_FIELDS + ACTION_FIELDS + DISPLAY_FIELDS + DISTANCE_FIELDS + ["distance_fault_reason", "can_stack_words", "debug_plan_id"] if len(values) == len(FIELDS) + 18 else
             FIELDS + SEQUENCE_FIELDS + CAN_DIAG_FIELDS + ACTION_FIELDS + DISPLAY_FIELDS + DISTANCE_FIELDS + ["distance_fault_reason", "can_stack_words"] if len(values) == len(FIELDS) + 17 else
             FIELDS + ["can_stack_words"] if len(values) == len(FIELDS) + 1 else
             FIELDS + SEQUENCE_FIELDS + CAN_DIAG_FIELDS + ACTION_FIELDS + DISPLAY_FIELDS + DISTANCE_FIELDS + ["distance_fault_reason"] if len(values) == len(FIELDS) + 16 else
             FIELDS + SEQUENCE_FIELDS + CAN_DIAG_FIELDS + ACTION_FIELDS + DISPLAY_FIELDS + DISTANCE_FIELDS if len(values) == len(FIELDS) + 15 else
             FIELDS + SEQUENCE_FIELDS + CAN_DIAG_FIELDS + ACTION_FIELDS + DISPLAY_FIELDS if len(values) == len(FIELDS) + 10 else
             FIELDS + SEQUENCE_FIELDS + CAN_DIAG_FIELDS + ACTION_FIELDS if len(values) == len(FIELDS) + 8 else
             FIELDS + SEQUENCE_FIELDS + CAN_DIAG_FIELDS if len(values) == len(FIELDS) + 7 else
             FIELDS + SEQUENCE_FIELDS if len(values) == len(FIELDS) + 3 else FIELDS)
    if len(values) != len(names):
        raise ValueError("field count")
    numbers = [int(value) for value in values]
    if any(not -(2**31) <= n < 2**31 for n in numbers):
        raise ValueError("int32 field overflow")
    record = dict(zip(names, numbers))
    for name in UINT_FIELDS:
        record[name] &= 0xFFFFFFFF
    for name in ("correction_rpm", "peak_abs_rpm", "last_nonzero_rpm"):
        record[name + "_decirpm"] = record[name]
        record[name] /= 10.0
    if abs(record["correction_rpm"]) > 27:
        raise ValueError("fine command exceeds physical RPM limit")
    record["telemetry_schema"] = "TM8"
    record["command_resolution_rpm"] = 0.1
    for name in ("rpm1","rpm2","rpm3","rpm4"):
        record[name+"_decirpm"] = record[name]
        record[name] /= 10.0
    record["feedback_resolution_rpm"] = 0.1
    record["motor_firmware"] = "X"
    return record


FAULT_FIELDS = [
    "fault_tick_ms", "arm_sequence", "trip", "phase", "backend_state", "backend_cause", "backend_session",
    "source_valid", "angle_age_ms", "rate_age_ms", "hwt_uart_errors", "hwt_overflows", "hwt_checksum_errors",
    "base_uart_errors", "base_overflows", "base_checksum_errors", "uart_ore", "uart_fe", "uart_ne", "uart_pe",
    "last_uart_sr", "event_tick_ms", "event_reason", "event_sr", "event_uart_errors", "event_overflows",
    "event_checksum_errors", "event_seen", "source_sample_age_ms"]


BACKEND_FAULT_FIELDS = ["backend_fault_reason", "backend_fault_state", "backend_io_result",
    "backend_address", "backend_command_age_ms", "backend_status_mask", "backend_speed_mask",
    "backend_feedback_age_ms", "backend_fault_tick_ms"]


def parse_fault_telemetry(line):
    """HF1/HF2 is an immutable first-trip snapshot; never a command or freshness token."""
    line = line.rstrip(b"\r\n")
    extended = line.startswith(b"HF2,")
    if not extended and not line.startswith(b"HF1,"):
        return None
    if len(line) >= (512 if extended else 384):
        raise ValueError("fault frame too long")
    prefix, sep, checksum = line.rpartition(b",")
    if not sep or len(checksum) != 4 or binascii.crc_hqx(prefix+b",", 0xFFFF) != int(checksum,16):
        raise ValueError("fault CRC mismatch")
    values = [int(value) for value in prefix.split(b",")[1:]]
    fields = FAULT_FIELDS + BACKEND_FAULT_FIELDS if extended else FAULT_FIELDS
    if len(values) != len(fields) or any(not 0 <= value <= 0xFFFFFFFF for value in values):
        raise ValueError("fault field count or uint32 range")
    result = dict(zip(fields, values))
    if result["source_valid"] > 1 or result["event_reason"] not in (0,1,2,4) or result["event_seen"] > 3:
        raise ValueError("fault flags")
    if extended and (result["backend_fault_reason"] > 8 or result["backend_fault_state"] > 11 or
                     result["backend_address"] > 4 or result["backend_status_mask"] > 15 or
                     result["backend_speed_mask"] > 15):
        raise ValueError("backend fault flags")
    return result


TIMING_SCOPES = ["hold_critical", "backend_begin", "action_load"]
TIMING_FIELDS = ["fault_tick_ms", "cycle_hz", "timer_enabled", "event_cycle"] + [
    f"{scope}_{field}" for scope in TIMING_SCOPES for field in
    ("count", "last_cycles", "max_cycles", "max_start", "max_end", "max_tick_ms", "max_arm", "last_end")]


BATCH_FIELDS = [
    "session", "batch_id", "segment", "dry_run", "generation_source",
    "generated_ms", "start_ms", "end_ms", "outcome", "sent_mask", "ack_mask",
    "failed_address", "io_result", "correction_rpm", "base_rpm", "pending_changes", "pending_rpm"
] + [f"{name}{wheel}" for name in ("send_ms", "reply_ms", "result") for wheel in range(1,5)] + [
    "overwritten", "max_queue_ms", "max_batch_ms", "max_call_ms", "total_success", "total_interrupted"
]

def parse_batch_telemetry(line):
    """BT8 is retained communication history, never a command or fresh health sample."""
    line=line.rstrip(b"\r\n")
    if not line.startswith(b"BT8,"): return None
    if len(line)>=448: raise ValueError("batch frame too long")
    prefix,sep,checksum=line.rpartition(b",")
    if not sep or len(checksum)!=4 or binascii.crc_hqx(prefix+b",",0xFFFF)!=int(checksum,16):
        raise ValueError("batch CRC mismatch")
    values=[int(value) for value in prefix.split(b",")[1:]]
    if len(values)!=len(BATCH_FIELDS): raise ValueError("batch field count")
    for key,value in zip(BATCH_FIELDS,values):
        if key in ("correction_rpm","pending_rpm"):
            if not -270<=value<=270: raise ValueError("batch correction range")
        elif not 0<=value<=0xFFFFFFFF: raise ValueError("batch uint32 range")
    r=dict(zip(BATCH_FIELDS,values))
    if (r["dry_run"]>1 or r["generation_source"]>3 or r["segment"]>2 or
        r["outcome"] not in (1,2,3) or r["sent_mask"]>15 or r["ack_mask"]>15 or
        r["ack_mask"] & ~r["sent_mask"] or r["failed_address"]>4 or
        (r["outcome"]==1 and (r["sent_mask"]!=15 or r["ack_mask"]!=15 or r["failed_address"]))):
        raise ValueError("batch flags inconsistent")
    r["queue_ms"]=(r["start_ms"]-r["generated_ms"])&0xFFFFFFFF
    r["batch_ms"]=(r["end_ms"]-r["start_ms"])&0xFFFFFFFF
    if r["queue_ms"]>=0x80000000 or r["batch_ms"]>=0x80000000:
        raise ValueError("batch time order")
    r["call_ms"]=[]
    for wheel in range(1,5):
        bit=1<<(wheel-1)
        returned=bool(r["ack_mask"]&bit or r["failed_address"]==wheel)
        if returned:
            if not r["sent_mask"]&bit: raise ValueError("reply without send")
            if r["ack_mask"]&bit and r[f"result{wheel}"]!=1:
                raise ValueError("ack without success result")
            duration=(r[f"reply_ms{wheel}"]-r[f"send_ms{wheel}"])&0xFFFFFFFF
            start_offset=(r[f"send_ms{wheel}"]-r["start_ms"])&0xFFFFFFFF
            if start_offset+duration>r["batch_ms"]: raise ValueError("reply outside batch")
            r["call_ms"].append(duration)
        else: r["call_ms"].append(None)
    r["mechanical_response_measured"]=False
    for name in ("correction_rpm", "pending_rpm"):
        r[name + "_decirpm"] = r[name]
        r[name] /= 10.0
    r["telemetry_schema"] = "BT8"
    return r


def parse_timing_telemetry(line):
    """CT1 fault or CT2 completed-stage interval diagnostics; never a control token."""
    line=line.rstrip(b"\r\n")
    normal = line.startswith(b"CT2,")
    if not normal and not line.startswith(b"CT1,"): return None
    if len(line)>=384: raise ValueError("timing frame too long")
    prefix,sep,checksum=line.rpartition(b",")
    if not sep or len(checksum)!=4 or binascii.crc_hqx(prefix+b",",0xFFFF)!=int(checksum,16):
        raise ValueError("timing CRC mismatch")
    values=[int(value) for value in prefix.split(b",")[1:]]
    fields = (["outcome", "sequence_index", "backend_session", "snapshot_tick_ms"] + TIMING_FIELDS[1:]) if normal else TIMING_FIELDS
    if len(values)!=len(fields) or any(not 0<=v<=0xFFFFFFFF for v in values):
        raise ValueError("timing count or uint32 range")
    r=dict(zip(fields,values))
    if normal and (r["outcome"] != 1 or r["sequence_index"] >= 8):
        raise ValueError("normal timing flags")
    if r["timer_enabled"] not in (0,1) or (r["timer_enabled"] and not r["cycle_hz"]):
        raise ValueError("timing flags or frequency")
    for scope in TIMING_SCOPES:
        cycles=r[f"{scope}_max_cycles"]
        if cycles and (not r["timer_enabled"] or cycles != (r[f"{scope}_max_end"]-r[f"{scope}_max_start"])&0xFFFFFFFF):
            raise ValueError("timing interval inconsistent")
        r[f"{scope}_max_us"] = cycles*1e6/r["cycle_hz"] if r["timer_enabled"] and cycles else None
    return r


POSITION_FIELDS = ["tick_ms", "session", "sample_sequence", "valid_mask", "vm_active"] + [
    f"{name}{wheel}" for wheel in range(1, 5)
    for name in ("result", "sign", "raw", "sample_ms", "age_ms")]


def parse_perimeter_checkpoint(line):
    """OP1 retained checkpoints are observation only, not movement permission."""
    line=line.strip()
    if not line.startswith(b"OP1,"):return None
    prefix,sep,crc=line.rpartition(b",")
    if not sep or len(crc)!=4 or binascii.crc_hqx(prefix+b",",0xFFFF)!=int(crc,16):
        raise ValueError("OP1 CRC mismatch")
    values=[int(v) for v in prefix.split(b",")[1:]]
    fields=["checkpoint","captured","tick_ms","x_mm","y_mm","heading_mdeg","valid","reasons","continuity_lost","age_ms","target_x_mm","target_y_mm","target_heading_mdeg"]
    if len(values)!=len(fields):raise ValueError("OP1 field count")
    r=dict(zip(fields,values))
    if not 0<=r["checkpoint"]<=12 or any(r[k] not in (0,1) for k in ["captured","valid","continuity_lost"]):
        raise ValueError("OP1 invalid identity")
    if not 0<=r["tick_ms"]<=0xFFFFFFFF or not 0<=r["age_ms"]<=0xFFFFFFFF or not 0<=r["reasons"]<=15:
        raise ValueError("OP1 invalid status")
    if r["valid"] and (not r["captured"] or r["reasons"] or r["continuity_lost"]):
        raise ValueError("OP1 contradictory validity")
    if any(abs(r[k])>180000 for k in ["heading_mdeg","target_heading_mdeg"]):raise ValueError("OP1 invalid angle")
    return r


def parse_odometry_telemetry(line):
    """OD1 is observation only; never renew motion health or generate commands."""
    line = line.strip()
    if not line.startswith(b"OD1,"):
        return None
    prefix, sep, checksum = line.rpartition(b",")
    if not sep or len(checksum) != 4 or binascii.crc_hqx(prefix+b",", 0xFFFF) != int(checksum, 16):
        raise ValueError("invalid OD1 CRC")
    values = [int(v) for v in prefix.split(b",")[1:]]
    fields = ["tick_ms", "sample_ms", "updates", "valid", "reasons", "continuity_lost",
              "local_origin", "x_mm", "y_mm", "heading_mdeg", "age_ms"]
    if len(values) != len(fields):
        raise ValueError("invalid OD1 length")
    result = dict(zip(fields, values))
    for key in ["tick_ms", "sample_ms", "updates", "age_ms"]:
        if not 0 <= result[key] <= 0xFFFFFFFF:
            raise ValueError("invalid OD1 unsigned field")
    if any(result[key] not in (0, 1) for key in ["valid", "continuity_lost", "local_origin"]):
        raise ValueError("invalid OD1 flag")
    if not 0 <= result["reasons"] <= 15 or abs(result["heading_mdeg"]) > 180000:
        raise ValueError("invalid OD1 status")
    if result["valid"] and (result["continuity_lost"] or result["reasons"]):
        raise ValueError("contradictory OD1 validity")
    return result


def parse_position_telemetry(line):
    """PM1 is read-only diagnostic data; never use it as a control token."""
    line = line.rstrip(b"\r\n")
    if not line.startswith(b"PM1,"):
        return None
    if len(line) >= 512:
        raise ValueError("position frame too long")
    prefix, sep, checksum = line.rpartition(b",")
    if not sep or len(checksum) != 4:
        raise ValueError("position CRC missing")
    if binascii.crc_hqx(prefix + b",", 0xFFFF) != int(checksum, 16):
        raise ValueError("position CRC mismatch")
    values = [int(value) for value in prefix.split(b",")[1:]]
    if len(values) != len(POSITION_FIELDS) or any(not 0 <= n <= 0xFFFFFFFF for n in values):
        raise ValueError("position field count or uint32 range")
    record = dict(zip(POSITION_FIELDS, values))
    if record["valid_mask"] > 15 or record["vm_active"] > 1:
        raise ValueError("position flags")
    counts = []
    for wheel in range(1, 5):
        if record[f"sign{wheel}"] > 1:
            raise ValueError("position sign")
        valid = bool(record["valid_mask"] & (1 << (wheel - 1)))
        if valid and (record[f"result{wheel}"] != 1 or record[f"age_ms{wheel}"] >= 1000 or not record["vm_active"]):
            raise ValueError("position validity inconsistent")
        raw = record[f"raw{wheel}"]
        counts.append((-raw if record[f"sign{wheel}"] else raw) if valid else None)
    record["signed_counts"] = counts
    return record


ENDPOINT_FIELDS = ["tick_ms", "session", "outcome", "start_mask", "end_mask"] + [
    f"{phase}_{name}{wheel}" for phase in ("start", "end") for wheel in range(1, 5)
    for name in ("sign", "raw", "ms")]


DIRECTIONAL_ENDPOINT_FIELDS = ENDPOINT_FIELDS[:5] + ["translation_mode", "distance_mode"] + ENDPOINT_FIELDS[5:]


def parse_position_endpoints(line):
    """PE1 legacy or PE2 direction-tagged historical wheel-motion interval."""
    line = line.rstrip(b"\r\n")
    perimeter = line.startswith(b"PE3,")
    directional = line.startswith(b"PE2,") or perimeter
    if not directional and not line.startswith(b"PE1,"):
        return None
    if len(line) >= 512:
        raise ValueError("endpoint frame too long")
    prefix, sep, checksum = line.rpartition(b",")
    if not sep or len(checksum) != 4 or binascii.crc_hqx(prefix+b",",0xFFFF) != int(checksum,16):
        raise ValueError("endpoint CRC")
    values = [int(v) for v in prefix.split(b",")[1:]]
    names = DIRECTIONAL_ENDPOINT_FIELDS if directional else ENDPOINT_FIELDS
    if len(values) != len(names) or any(not 0 <= v <= 0xFFFFFFFF for v in values):
        raise ValueError("endpoint field count/range")
    r = dict(zip(names, values))
    if directional and (r["translation_mode"] > (4 if perimeter else 3) or r["distance_mode"] > 1 or
                        (r["distance_mode"] and r["translation_mode"] == (4 if perimeter else 3))):
        raise ValueError("endpoint direction flags")
    if r["outcome"] != 1 or r["start_mask"] != 15 or r["end_mask"] != 15:
        raise ValueError("incomplete endpoint record")
    delta = []
    for w in range(1, 5):
        if r[f"start_sign{w}"] > 1 or r[f"end_sign{w}"] > 1:
            raise ValueError("endpoint sign")
        duration = (r[f"end_ms{w}"] - r[f"start_ms{w}"]) & 0xFFFFFFFF
        if not 0 < duration <= 120000:
            raise ValueError("endpoint time ordering/duration")
        start = r[f"start_raw{w}"] * (-1 if r[f"start_sign{w}"] else 1)
        end = r[f"end_raw{w}"] * (-1 if r[f"end_sign{w}"] else 1)
        delta.append(end - start)  # Python wide int preserves uint32 sign crossing.
    for phase in ("start", "end"):
        first = r[f"{phase}_ms1"]
        if any(((r[f"{phase}_ms{w}"]-first)&0xFFFFFFFF) >= 1000 for w in range(1,5)):
            raise ValueError("endpoint batch age/order")
    r["delta_counts"] = delta
    r["logical_wheel_mm"] = [d * sign * 3.141592653589793 * 77 / 65536
                               for d, sign in zip(delta,(1,-1,1,-1))]
    r["requested_wheel_mm"] = None
    r["translation_travel_mm"] = None
    if directional and (r["translation_mode"] in (0,1,2) or (perimeter and r["translation_mode"]==3)):
        projection = ((1,1,1,1),(1,-1,-1,1),(-1,1,1,-1),(-1,-1,-1,-1))[r["translation_mode"]]
        r["requested_wheel_mm"] = [v*sign for v,sign in zip(r["logical_wheel_mm"],projection)]
        r["translation_travel_mm"] = sum(r["requested_wheel_mm"])/4
    r["historical"] = True
    return r


def target_theoretical_cm_s(record):
    """Display only: configured translation target, never actual chassis speed."""
    value = record.get("target_theoretical_milli_cm_s")
    if value is not None:
        return None if value == -(2**31) else value / 1000
    # Legacy translation frames used the verified 77 mm direct-drive wheel.
    if record.get("motion_mode") in {0, 1, 2}:
        return round(record["base_rpm"] * 3.141592653589793 * 7.7 / 60, 3)
    return None  # Historical combo frames do not identify a turn reliably.


class Controller:
    def __init__(self, action="monitor", expected_mode=1, plan=None, route_trial=False, turn_trial=False):
        if action not in {"monitor", "run-once", "stop"}:
            raise ValueError("invalid action")
        self.action = action
        self.mode = expected_mode
        if plan is not None and (plan not in DEBUG_PLAN_IDS.values() or expected_mode != 3 or action != "run-once"):
            raise ValueError("plan requires run-once mode3 and a supported preset id (5 is retired)")
        self.plan = plan
        if route_trial and (action != "run-once" or expected_mode != 3 or plan != 0):
            raise ValueError("route trial requires explicit run-once mode3 plan default")
        if turn_trial and (route_trial or action!="run-once" or expected_mode!=3 or plan!=0):
            raise ValueError("turn trial requires explicit run-once mode3 plan default without route trial")
        self.turn_trial=turn_trial
        self.route_trial = route_trial
        self.route_count = None
        self.plan_selected = False
        self.plan_before_accepted = None
        self.rejected_before_command = None
        self.stage = "observe"
        self.status = None
        self.token = (0, 0)
        self.last_rx = None
        self.stage_tick = None
        self.last_keep = None
        self.healthy_frames = 0
        self.healthy_since = None
        self.healthy_tick = None
        self.start_sent = False
        self.last_seq = None

    def _speed_profile_healthy(self, r):
        plan = r.get("debug_plan_id", 0)
        if self.turn_trial:
            return (plan==0 and r["base_rpm"]==27 and r["feedback_limit_rpm"]==40 and
                    r.get("sequence_count")==1 and not r.get("distance_mode"))
        if plan == 0 and r["feedback_limit_rpm"] == 80:
            # An explicit different preset may leave the candidate while idle;
            # these observations authorize PLAN only, never ARM on this profile.
            if (not self.route_trial and self.mode == 3 and self.plan not in {None, 0} and
                    not self.plan_selected):
                return 3 <= r["base_rpm"] <= 52 and 1 <= r.get("sequence_count", 0) <= 8
            return (self.route_trial and self.plan == 0 and self.mode == 3 and
                    3 <= r["base_rpm"] <= 52 and 1 <= r.get("sequence_count", 0) <= 8 and
                    (not self.plan_selected or r.get("sequence_count") == self.route_count))
        if self.route_trial and self.plan_selected:
            return False
        if plan == 7:
            # A high-speed idle echo never authorizes an unspecified/default run.
            return (self.mode == 3 and self.plan is not None and
                    (not self.plan_selected or self.plan == 7) and
                    r["base_rpm"] == 52 and r["feedback_limit_rpm"] == 80 and
                    r.get("sequence_count") == 1)
        if self.plan_selected and self.plan == 7:
            return False
        if self.mode == 3 and plan not in DEBUG_PLAN_IDS.values():
            return False
        return (((3 <= r["base_rpm"] <= 15) if self.mode == 3 else r["base_rpm"] == 15)
                and r["feedback_limit_rpm"] == 30)

    def _healthy(self, r):
        return (r["phase"] == 0 and r["trip"] == 0 and r["backend_active"] == 0 and
                r["backend_result"] == 0 and r["vm_request"] == 0 and r["pe2_active"] == 0 and
                r["source_valid"] == 1 and r["selftest"] == 1 and r["protocol_selftest"] == 1 and
                0 <= r["angle_age_ms"] < 100 and 0 <= r["rate_age_ms"] < 100 and
                abs(r["rate_mdps"]) <= 3000 and r["motion_mode"] == self.mode and
                self._speed_profile_healthy(r) and
                (self.mode != 3 or 1 <= r.get("sequence_count",4) <= 8) and
                not any(r[k] for k in ["rtos_fault", "safety_fault", "can_fault"]))

    def abort(self, reason):
        if self.status is not None:
            return []
        self.status = reason
        self.stage = "finished"
        return [command("STOP", self.token)] if self.action != "monitor" else []

    def consume(self, r, now):
        if self.last_seq is not None:
            delta = (r["seq"] - self.last_seq) & 0xFFFFFFFF
            if delta == 0:
                return []  # Cached/repeated frames cannot renew health or link freshness.
            if delta >= 0x80000000:
                self.healthy_frames = 0
                if self.stage != "observe":
                    return self.abort("telemetry_session_changed")
        self.last_seq = r["seq"]
        self.last_rx = now
        if self.status is not None or self.action == "monitor":
            return []
        if self.action == "stop":
            if self.stage == "observe":
                self.stage = "stop_wait"
                self.stage_tick = now
                return [command("STOP")]
            if r["remote_state"] == 3 and r["remote_result"] == 3 and r["vm_request"] == 0 and r["pe2_active"] == 0:
                self.status = "stop_acknowledged"
            return []
        if self.start_sent:
            if r["phase"] in {4, 5}:
                return self.abort("motion_fault_or_cancelled")
            if self.turn_trial and (r.get("debug_plan_id")!=0 or
                    r["base_rpm"]!=27 or r["feedback_limit_rpm"]!=40 or
                    r.get("sequence_count")!=1 or r.get("distance_mode") or
                    r.get("action_kind")!=3 or abs(r["correction_rpm"])>27):
                return self.abort("turn_trial_profile_changed")
            if self.route_trial and (r.get("debug_plan_id") != 0 or r["feedback_limit_rpm"] != 80 or
                    r.get("sequence_count") != self.route_count or
                    not 0 <= r["base_rpm"] <= 52 or not 0 <= r.get("distance_base_rpm", 0) <= 52 or
                    (r.get("distance_mode") and (r.get("action_kind") not in {4, 5, 6} or
                     not 100000 <= r.get("distance_target_milli_mm", -1) <= 2100000))):
                return self.abort("route_trial_profile_changed")
            if self.plan == 7 and (r.get("debug_plan_id") != 7 or
                    r["feedback_limit_rpm"] != 80 or r.get("sequence_count") != 1 or
                    not 0 <= r["base_rpm"] <= 52 or
                    not 0 <= r.get("distance_base_rpm", 0) <= 52 or
                    (r.get("distance_mode") and (r.get("action_kind") != 4 or
                        r.get("distance_target_milli_mm") != 2100000))):
                return self.abort("speed20_profile_changed")
            if (r["phase"] == 3 and r["trip"] == 0 and r["backend_result"] == 1 and
                    r["stop_mask"] == 15 and r["vm_request"] == 0 and r["pe2_active"] == 0 and
                    (self.mode != 3 or (r.get("sequence_state") == 3 and
                                       1 <= r.get("sequence_count",4) <= 8 and
                                       r.get("sequence_completed") == r.get("sequence_count",4)))):
                if r.get("distance_mode") and not r.get("distance_reached"):
                    return self.abort("distance_not_reached")
                self.status = "software_complete"
                self.stage = "finished"
                return []
        if self.stage != "observe":
            # PLAN success increments accepted but leaves the last result unchanged.
            # Distinguish a latched old error from a new rejection; never clear it.
            new_rejection = (self.rejected_before_command is not None and
                             r["remote_rejected"] != self.rejected_before_command)
            unknown_rejection = (self.rejected_before_command is None and
                                 r["remote_result"] in {4, 5, 6})
            if new_rejection or unknown_rejection or r["remote_result"] in {7, 8}:
                return self.abort("remote_rejected_or_expired")
        if self.stage == "plan_wait":
            if (r.get("debug_plan_id") == self.plan and r["remote_state"] == 0 and
                    r["remote_accepted"] > self.plan_before_accepted):
                self.plan_selected = True
                if self.route_trial:
                    self.route_count = r.get("sequence_count")
                self.stage = "observe"
                self.healthy_frames = 0
                self.healthy_since = self.healthy_tick = None
            return []
        if self.stage == "observe":
            healthy = self._healthy(r) and r["remote_state"] == 0
            if not healthy:
                self.healthy_frames = 0
                self.healthy_since = self.healthy_tick = None
            else:
                if self.healthy_since is None:
                    self.healthy_since, self.healthy_tick = now, r["tick_ms"]
                device_span = (r["tick_ms"] - self.healthy_tick) & 0xFFFFFFFF
                host_span = now - self.healthy_since
                # Buffered frames can be internally fresh yet arrive as an old burst.
                # Restart the observation window when device and host pacing disagree.
                if abs(device_span - host_span * 1000) > 150:
                    self.healthy_frames = 0
                    self.healthy_since, self.healthy_tick = now, r["tick_ms"]
                self.healthy_frames += 1
            if self.healthy_frames >= 3 and now - self.healthy_since >= 0.15:
                if self.plan is not None and not self.plan_selected:
                    if "debug_plan_id" not in r:
                        return self.abort("firmware_plan_unsupported")
                    self.stage = "plan_wait"
                    self.stage_tick = now
                    self.plan_before_accepted = r["remote_accepted"]
                    self.rejected_before_command = r["remote_rejected"]
                    return [command("PLAN" + str(self.plan), (r["seq"], r["tick_ms"]))]
                self.rejected_before_command = r["remote_rejected"]
                self.token = (r["seq"], r["tick_ms"])
                self.stage = "arm_wait"
                self.stage_tick = now
                return [command("ARM", self.token)]
        elif self.stage == "arm_wait":
            if r["remote_state"] == 1 and (r["token_seq"], r["token_tick"]) == self.token:
                if not self._healthy(r):
                    return self.abort("health_changed_before_start")
                self.stage = "start_wait"
                self.stage_tick = self.last_keep = now
                self.start_sent = True
                return [command("START", self.token)]
        elif self.stage == "start_wait" and r["remote_state"] == 2:
            if (r["token_seq"], r["token_tick"]) != self.token:
                return self.abort("session_mismatch")
            self.stage = "running"
        return []

    def tick(self, now):
        if self.status is not None or self.action == "monitor" or self.stage == "observe":
            return []
        if self.last_rx is not None and now - self.last_rx >= 0.6:
            return self.abort("telemetry_timeout")
        if self.stage in {"plan_wait", "arm_wait", "start_wait", "stop_wait"} and now - self.stage_tick >= 2.0:
            return self.abort("ack_timeout")
        if self.start_sent and now - self.last_keep >= 0.2:
            self.last_keep = now
            return [command("KEEP", self.token)]
        return []


class WindowsSerial:
    """Synchronous Win32 COM access; no DTR/RTS, no external Python packages."""
    def __init__(self, port):
        if os.name != "nt":
            raise OSError("Windows COM client requires Windows")
        from ctypes import wintypes as w
        class DCB(ctypes.Structure):
            _fields_ = [("length", w.DWORD), ("baud", w.DWORD), ("flags", w.DWORD),
                        ("reserved", w.WORD), ("xon_limit", w.WORD), ("xoff_limit", w.WORD),
                        ("byte_size", w.BYTE), ("parity", w.BYTE), ("stop_bits", w.BYTE),
                        ("xon", ctypes.c_char), ("xoff", ctypes.c_char), ("error", ctypes.c_char),
                        ("eof", ctypes.c_char), ("event", ctypes.c_char), ("reserved1", w.WORD)]
        class Timeouts(ctypes.Structure):
            _fields_ = [(name, w.DWORD) for name in ["interval", "read_mult", "read_const", "write_mult", "write_const"]]
        api = self.api = ctypes.WinDLL("kernel32", use_last_error=True)
        signatures = {
            "CreateFileW": ([w.LPCWSTR,w.DWORD,w.DWORD,ctypes.c_void_p,w.DWORD,w.DWORD,w.HANDLE],w.HANDLE),
            "GetCommState": ([w.HANDLE,ctypes.POINTER(DCB)],w.BOOL),
            "SetCommState": ([w.HANDLE,ctypes.POINTER(DCB)],w.BOOL),
            "PurgeComm": ([w.HANDLE,w.DWORD],w.BOOL),
            "SetCommTimeouts": ([w.HANDLE,ctypes.POINTER(Timeouts)],w.BOOL),
            "ReadFile": ([w.HANDLE,ctypes.c_void_p,w.DWORD,ctypes.POINTER(w.DWORD),ctypes.c_void_p],w.BOOL),
            "WriteFile": ([w.HANDLE,ctypes.c_void_p,w.DWORD,ctypes.POINTER(w.DWORD),ctypes.c_void_p],w.BOOL),
            "CloseHandle": ([w.HANDLE],w.BOOL)}
        for name,(args,result) in signatures.items():
            fn=getattr(api,name); fn.argtypes=args; fn.restype=result
        self.handle = api.CreateFileW("\\\\.\\"+port,0xC0000000,0,None,3,0x80,None)
        if self.handle == ctypes.c_void_p(-1).value:
            self.handle = None
            raise ctypes.WinError(ctypes.get_last_error())
        self.DWORD = w.DWORD
        try:
            dcb=DCB(); dcb.length=ctypes.sizeof(DCB)
            self._check(api.GetCommState(self.handle,ctypes.byref(dcb)))
            dcb.baud=115200; dcb.flags=0x81; dcb.byte_size=8; dcb.parity=0; dcb.stop_bits=0
            self._check(api.SetCommState(self.handle,ctypes.byref(dcb)))
            self._check(api.SetCommTimeouts(self.handle,ctypes.byref(Timeouts(0xFFFFFFFF,0,50,0,500))))
            # Discard RX backlog only; no reset, control command or TX data.
            self._check(api.PurgeComm(self.handle,0x000A))
        except Exception:
            self.close(); raise

    def _check(self, ok):
        if not ok:
            raise ctypes.WinError(ctypes.get_last_error())

    def read(self):
        buffer=ctypes.create_string_buffer(4096); count=self.DWORD()
        self._check(self.api.ReadFile(self.handle,buffer,len(buffer),ctypes.byref(count),None))
        return buffer.raw[:count.value]

    def write(self, data):
        count=self.DWORD(); buffer=ctypes.create_string_buffer(data)
        self._check(self.api.WriteFile(self.handle,buffer,len(data),ctypes.byref(count),None))
        if count.value != len(data):
            raise OSError("partial write; command not retried")

    def close(self):
        if self.handle is not None:
            self.api.CloseHandle(self.handle); self.handle=None


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM17")
    parser.add_argument("--action", choices=["monitor","run-once","stop"],default="monitor")
    parser.add_argument("--mode", type=int,choices=[0,1,2,3],default=1,help="expected firmware mode, does not change it")
    parser.add_argument("--seconds", type=float,default=150)
    parser.add_argument("--output",type=Path,default=Path(__file__).resolve().parent.parent/"tmp"/"serial_motion_captures")
    parser.add_argument("--plan", choices=list(DEBUG_PLAN_IDS),
                        help="select idle debug preset before ARM/START; mode3 only")
    parser.add_argument("--route-trial", action="store_true",
                        help="explicit opt-in to new unvalidated speed-route candidate; run-once mode3 plan default only")
    parser.add_argument("--turn-trial",action="store_true",help="explicit opt-in to single90deg30dps trial; mode3 plan default only")
    args=parser.parse_args()
    if args.plan is not None and (args.action != "run-once" or args.mode != 3):
        parser.error("--plan requires --action run-once --mode 3")
    if args.action == "run-once" and not (args.mode == 3 and args.plan == "default" and args.route_trial and not args.turn_trial):
        parser.error("X vehicle trial requires mode3 plan default --route-trial (210cm only)")
    plan_id = None if args.plan is None else DEBUG_PLAN_IDS[args.plan]
    if not 1 <= args.seconds <= 300:
        parser.error("seconds must be 1..300")
    args.output.mkdir(parents=True,exist_ok=True)
    capture=datetime.now().strftime("%Y%m%d_%H%M%S")+f"_{time.time_ns()%1000000:06d}"
    if args.route_trial and (args.action != "run-once" or args.mode != 3 or plan_id != 0):
        parser.error("--route-trial requires --action run-once --mode 3 --plan default")
    if args.turn_trial and (args.route_trial or args.action!="run-once" or args.mode!=3 or plan_id!=0):
        parser.error("--turn-trial requires run-once mode3 plan default without --route-trial")
    if args.turn_trial:
        ctl=Controller(args.action,args.mode,plan_id,turn_trial=True)
    elif args.route_trial:
        ctl=Controller(args.action,args.mode,plan_id,route_trial=True)
    else:
        ctl=Controller(args.action,args.mode,plan_id)
    load_history=LoadAccumulator()
    live_load_rows=[]
    if args.action=="run-once": ctl.capture_drain_s=90.0
    report=dict(action=args.action,expected_mode=args.mode,port=args.port,opened=False,valid_frames=0,invalid_frames=0,
                sent_commands=0,batch_frames=0,batch_records=[],timing_frames=0,timing_records=[],last_timing=None,fault_frames=0,last_fault=None,position_frames=0,last_position=None,endpoint_frames=0,endpoints=[],heading_observation=None,nonzero_frames=0,max_abs_rpm=[0,0,0,0],max_abs_error_deg=0,
                sequence_gaps=0,last=None,error=None,baseline="5f132ad")
    serial=None; pending=b""; previous=None; last_progress=0.0
    with (args.output/(capture+".raw")).open("wb") as raw, (args.output/(capture+".jsonl")).open("w",encoding="utf8") as records:
        def transmit(messages):
            for message in messages:
                serial.write(message)
                report["sent_commands"]+=1
                records.write(json.dumps(dict(host_utc=datetime.now(timezone.utc).isoformat(),direction="TX",text=message.decode().strip()))+"\n")
                records.flush()
        try:
            serial=WindowsSerial(args.port); report["opened"]=True
            start=time.monotonic()
            drain_until=None
            while time.monotonic()-start < args.seconds and (ctl.status is None or
                    (drain_until is not None and time.monotonic() < drain_until)):
                incoming=serial.read(); now=time.monotonic()
                raw.write(incoming); pending+=incoming
                if len(pending)>8192:
                    raise ValueError("receive buffer overflow or invalid stream")
                while b"\n" in pending:
                    line,pending=pending.split(b"\n",1)
                    try:
                        integral_packet=parse_integral_telemetry(line)
                        if integral_packet is not None:
                            report["integral_diagnostic"]=integral_packet
                            if load_history.complete and ctl.status is not None and report.get("integral_diagnostic"):
                                drain_until=min(drain_until or (now+1.0),now+1.0)
                            records.write(json.dumps(dict(host_utc=datetime.now(timezone.utc).isoformat(),direction="RX",frame_type="IG1",**integral_packet))+"\n")
                            records.flush()
                            continue
                        load_packet=parse_load_telemetry(line)
                        if load_packet is not None:
                            load_history.add(load_packet)
                            records.write(json.dumps(dict(host_utc=datetime.now(timezone.utc).isoformat(),direction="RX",frame_type="XD1",**load_packet))+"\n")
                            records.flush()
                            if load_history.complete and ctl.status is not None and report.get("integral_diagnostic"):
                                drain_until=min(drain_until or (now+1.0),now+1.0)
                            continue  # Historical records cannot arm/start/keep alive.
                        batch = parse_batch_telemetry(line)
                        if batch is not None:
                            report["batch_frames"] += 1
                            if batch not in report["batch_records"]: report["batch_records"].append(batch)
                            records.write(json.dumps(dict(host_utc=datetime.now(timezone.utc).isoformat(),
                                direction="RX",frame_type="BT8",**batch))+"\n")
                            records.flush()
                            continue  # History cannot renew health, arm, start, or keep a motion alive.
                        timing = parse_timing_telemetry(line)
                        if timing is not None:
                            report["timing_frames"] += 1; report["last_timing"] = timing
                            frame_type = "CT2" if line.startswith(b"CT2,") else "CT1"
                            saved = dict(frame_type=frame_type, **timing)
                            if saved not in report["timing_records"]: report["timing_records"].append(saved)
                            records.write(json.dumps(dict(host_utc=datetime.now(timezone.utc).isoformat(),
                                direction="RX",frame_type=frame_type,**timing))+"\n")
                            records.flush(); print(json.dumps(dict(event="critical_timing",**timing)),flush=True)
                            continue  # Timing data cannot renew link health or produce commands.
                        fault = parse_fault_telemetry(line)
                        if fault is not None:
                            report["fault_frames"] += 1
                            report["last_fault"] = fault
                            records.write(json.dumps(dict(host_utc=datetime.now(timezone.utc).isoformat(),
                                direction="RX", frame_type="HF2" if line.startswith(b"HF2,") else "HF1", **fault))+"\n")
                            records.flush()
                            print(json.dumps(dict(event="first_fault", **fault)), flush=True)
                            continue  # Diagnostics never produce or renew commands.
                        endpoint = parse_position_endpoints(line)
                        if endpoint is not None:
                            report["endpoint_frames"] += 1
                            report["endpoints"].append(endpoint)
                            records.write(json.dumps(dict(host_utc=datetime.now(timezone.utc).isoformat(),
                                direction="RX", frame_type="PE3" if line.startswith(b"PE3,") else ("PE2" if line.startswith(b"PE2,") else "PE1"), **endpoint))+"\n")
                            records.flush()
                            print(json.dumps(dict(event="wheel_interval", **endpoint)), flush=True)
                            continue  # Historical diagnostics never produce commands.
                        checkpoint = parse_perimeter_checkpoint(line)
                        if checkpoint is not None:
                            report.setdefault("perimeter_checkpoints", {})[str(checkpoint["checkpoint"])]=checkpoint
                            records.write(json.dumps(dict(host_utc=datetime.now(timezone.utc).isoformat(),
                                direction="RX", frame_type="OP1", **checkpoint))+"\n")
                            records.flush()
                            continue  # Checkpoints never renew motion health or generate commands.
                        odometry = parse_odometry_telemetry(line)
                        if odometry is not None:
                            report["odometry_frames"] = report.get("odometry_frames", 0) + 1
                            report["last_odometry"] = odometry
                            records.write(json.dumps(dict(host_utc=datetime.now(timezone.utc).isoformat(),
                                direction="RX", frame_type="OD1", **odometry))+"\n")
                            records.flush()
                            continue  # No ARM/START/KEEP decisions from OD1.
                        position = parse_position_telemetry(line)
                        if position is not None:
                            report["position_frames"] += 1
                            report["last_position"] = position
                            records.write(json.dumps(dict(host_utc=datetime.now(timezone.utc).isoformat(),
                                direction="RX", frame_type="PM1", **position))+"\n")
                            records.flush()
                            continue  # No ARM/START/KEEP decisions from PM1.
                        record=parse_telemetry(line)
                    except (ValueError,UnicodeError):
                        report["invalid_frames"]+=1; continue
                    if record is None: continue
                    if previous is not None:
                        delta=(record["seq"]-previous)&0xFFFFFFFF
                        if 0<delta<0x80000000: report["sequence_gaps"]+=delta-1
                    previous=record["seq"]
                    record["target_theoretical_cm_s"] = target_theoretical_cm_s(record)
                    live_load_rows.append(record.copy())
                    kind = record.get("action_kind")
                    record["distance_direction"] = kind-4 if record.get("distance_mode") and kind in (4,5,6) else None
                    if record.get("distance_mode") and record["source_valid"]:
                        yaw = record["yaw_mdeg"] / 1000
                        observation = report["heading_observation"]
                        if observation is None and record["backend_active"]:
                            observation = report["heading_observation"] = dict(
                                reference_yaw_deg=record["target_mdeg"]/1000,
                                initial_observed_yaw_deg=yaw, peak_departure_deg=0.0,
                                vm_off_yaw_deg=None, post_stop_yaw_deg=None)
                        if observation is not None:
                            departure = (yaw-observation["reference_yaw_deg"]+180)%360-180
                            if record["backend_active"]:
                                observation["peak_departure_deg"] = max(observation["peak_departure_deg"],abs(departure))
                            if record["phase"] == 3 and not record["vm_request"] and not record["pe2_active"]:
                                if observation["vm_off_yaw_deg"] is None: observation["vm_off_yaw_deg"] = yaw
                                observation["post_stop_yaw_deg"] = yaw
                                observation["post_stop_vs_reference_deg"] = departure
                                observation["post_stop_vs_vm_off_deg"] = (yaw-observation["vm_off_yaw_deg"]+180)%360-180

                    report["valid_frames"]+=1;report["last"]=record
                    report["nonzero_frames"]+=int(record["correction_rpm"]!=0)
                    if record["error_mdeg"]!=-(2**31): report["max_abs_error_deg"]=max(report["max_abs_error_deg"],abs(record["error_mdeg"])/1000)
                    report["max_abs_rpm"]=[max(old,abs(record[f"rpm{i+1}"])) for i,old in enumerate(report["max_abs_rpm"])]
                    records.write(json.dumps(dict(host_utc=datetime.now(timezone.utc).isoformat(),direction="RX",**record))+"\n");records.flush()
                    if now - last_progress >= 1.0:
                        progress = dict(event="telemetry", tick_ms=record["tick_ms"], phase=record["phase"],
                                        trip=record["trip"], error_deg=record["error_mdeg"]/1000,
                                        correction_rpm=record["correction_rpm"],
                                        rpm=[record[f"rpm{i+1}"] for i in range(4)],
                                        vm_request=record["vm_request"], remote_state=record["remote_state"])
                        progress.update({k: record[k] for k in SEQUENCE_FIELDS + ACTION_FIELDS + DISTANCE_FIELDS + ["distance_fault_reason", "distance_direction", "action_kind", "target_theoretical_cm_s"] if k in record})
                        if report["last_position"] is not None:
                            latest = report["last_position"]
                            progress["last_position"] = {k: latest[k] for k in
                                ("tick_ms", "session", "sample_sequence", "valid_mask", "signed_counts")}
                        print(json.dumps(progress), flush=True)
                        last_progress = now
                    if ctl.status is None:
                        transmit(ctl.consume(record,now))
                        if ctl.status == "motion_fault_or_cancelled":
                            drain_until = time.monotonic() + getattr(ctl,"capture_drain_s",2.5)  # No retry START.
                        elif ctl.status == "software_complete":
                            drain_until = time.monotonic() + getattr(ctl,"capture_drain_s",2.5)  # No further commands.
                        elif ctl.status is not None and hasattr(ctl,"capture_drain_s"):
                            drain_until = time.monotonic() + ctl.capture_drain_s  # Perimeter failure snapshots; no retry.
                if ctl.status is None:
                    transmit(ctl.tick(time.monotonic()))
            if ctl.status is None and args.action!="monitor":
                transmit(ctl.abort("duration_expired"))
        except (Exception,KeyboardInterrupt) as exc:
            report["error"]=str(exc) or "interrupted"
            if serial is not None:
                try: transmit(ctl.abort("client_error_or_interrupted"))
                except Exception: pass
        finally:
            if serial is not None: serial.close()
    report["status"]=ctl.status or ("monitor_received" if report["valid_frames"] else "no_telemetry")
    report["capture_prefix"]=str(args.output/capture)
    report["limits"]="software observations only; physical stop and actual trajectory require on-site confirmation"
    report["load_diagnostic"]=load_history.summary()
    meta=report.get("integral_diagnostic",{})
    report["integral_metadata_valid"]=(meta.get("session")==load_history.session and meta.get("enabled") is True and meta.get("ki")==0.03 and meta.get("limit_rpm")==0.3 and meta.get("controller_samples",0)>0)
    if load_history.complete:
        report["load_analysis"]=analyze_load(load_history,live_load_rows)
    elif args.action=="run-once":
        report["load_analysis_limit"]="No wheel-load conclusion: history missing, incomplete or overflowed."
    text=json.dumps(report,ensure_ascii=False,indent=2)
    (args.output/(capture+".summary.json")).write_text(text+"\n",encoding="utf8")
    print(text)
    if args.action=="run-once" and report["status"]=="software_complete" and (not load_history.complete or not report["integral_metadata_valid"]): return 3
    return 0 if report["status"] in {"software_complete","stop_acknowledged","monitor_received"} and report["error"] is None else 2


if __name__=="__main__":
    raise SystemExit(main())
