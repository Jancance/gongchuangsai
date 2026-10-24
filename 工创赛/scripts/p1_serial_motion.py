"""CRC telemetry and bounded one-shot serial motion client. Default action only monitors."""
import argparse
import binascii
import ctypes
import json
import os
from pathlib import Path
import time
from datetime import datetime, timezone

FIELDS = ['seq', 'tick_ms', 'phase', 'trip', 'elapsed_ms', 'source_valid', 'yaw_mdeg', 'target_mdeg', 'error_mdeg', 'rate_mdps', 'angle_age_ms', 'rate_age_ms', 'correction_rpm', 'rpm1', 'rpm2', 'rpm3', 'rpm4', 'vm_request', 'pe2_active', 'backend_state', 'backend_active', 'backend_result', 'backend_cause', 'stop_mask', 'can_fault', 'rtos_fault', 'safety_fault', 'motion_mode', 'nonzero_samples', 'peak_abs_rpm', 'last_nonzero_rpm', 'remote_state', 'remote_result', 'token_seq', 'token_tick', 'telemetry_dropped', 'remote_rejected', 'selftest', 'base_rpm', 'feedback_limit_rpm', 'peak_error_mdeg', 'max_sample_gap_ms', 'uart_rx_errors', 'rx_stack_words', 'heap_min_bytes', 'link_lost', 'protocol_selftest', 'remote_accepted']
SEQUENCE_FIELDS = ["sequence_state", "sequence_index", "sequence_completed"]
CAN_DIAG_FIELDS = ["can_guard_cause", "can_guard_bad_nodes", "power_wait_state", "post_settle_mask"]
UINT_FIELDS = {"seq", "tick_ms", "token_seq", "token_tick"}


def command(verb, token=(0, 0)):
    if verb not in {"ARM", "START", "KEEP", "STOP"}:
        raise ValueError("unsupported command")
    seq, tick = token
    if not all(0 <= n <= 0xFFFFFFFF for n in token):
        raise ValueError("token outside uint32")
    prefix = f"RC1,{seq},{tick},{verb},".encode("ascii")
    return prefix + f"{binascii.crc_hqx(prefix, 0xFFFF):04X}\n".encode("ascii")


def parse_telemetry(line):
    line = line.rstrip(b"\r\n")
    if not line.startswith(b"TM1,"):
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
    names = (FIELDS + SEQUENCE_FIELDS + CAN_DIAG_FIELDS if len(values) == len(FIELDS) + 7 else
             FIELDS + SEQUENCE_FIELDS if len(values) == len(FIELDS) + 3 else FIELDS)
    if len(values) != len(names):
        raise ValueError("field count")
    numbers = [int(value) for value in values]
    if any(not -(2**31) <= n < 2**31 for n in numbers):
        raise ValueError("int32 field overflow")
    record = dict(zip(names, numbers))
    for name in UINT_FIELDS:
        record[name] &= 0xFFFFFFFF
    return record


class Controller:
    def __init__(self, action="monitor", expected_mode=1):
        if action not in {"monitor", "run-once", "stop"}:
            raise ValueError("invalid action")
        self.action = action
        self.mode = expected_mode
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

    def _healthy(self, r):
        return (r["phase"] == 0 and r["trip"] == 0 and r["backend_active"] == 0 and
                r["backend_result"] == 0 and r["vm_request"] == 0 and r["pe2_active"] == 0 and
                r["source_valid"] == 1 and r["selftest"] == 1 and r["protocol_selftest"] == 1 and
                0 <= r["angle_age_ms"] < 100 and 0 <= r["rate_age_ms"] < 100 and
                abs(r["rate_mdps"]) <= 3000 and r["motion_mode"] == self.mode and
                r["base_rpm"] == 15 and r["feedback_limit_rpm"] == 30 and
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
            if (r["phase"] == 3 and r["trip"] == 0 and r["backend_result"] == 1 and
                    r["stop_mask"] == 15 and r["vm_request"] == 0 and r["pe2_active"] == 0 and
                    (self.mode != 3 or (r.get("sequence_state") == 3 and r.get("sequence_completed") == 4))):
                self.status = "software_complete"
                self.stage = "finished"
                return []
        if self.stage != "observe" and r["remote_result"] in {4, 5, 6, 7, 8}:
            return self.abort("remote_rejected_or_expired")
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
        if self.stage in {"arm_wait", "start_wait", "stop_wait"} and now - self.stage_tick >= 2.0:
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
    parser.add_argument("--seconds", type=float,default=25)
    parser.add_argument("--output",type=Path,default=Path(__file__).resolve().parent.parent/"tmp"/"serial_motion_captures")
    args=parser.parse_args()
    if not 1 <= args.seconds <= 300:
        parser.error("seconds must be 1..300")
    args.output.mkdir(parents=True,exist_ok=True)
    capture=datetime.now().strftime("%Y%m%d_%H%M%S")+f"_{time.time_ns()%1000000:06d}"
    ctl=Controller(args.action,args.mode)
    report=dict(action=args.action,expected_mode=args.mode,port=args.port,opened=False,valid_frames=0,invalid_frames=0,
                sent_commands=0,nonzero_frames=0,max_abs_rpm=[0,0,0,0],max_abs_error_deg=0,
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
            while time.monotonic()-start < args.seconds and ctl.status is None:
                incoming=serial.read(); now=time.monotonic()
                raw.write(incoming); pending+=incoming
                if len(pending)>8192:
                    raise ValueError("receive buffer overflow or invalid stream")
                while b"\n" in pending:
                    line,pending=pending.split(b"\n",1)
                    try:
                        record=parse_telemetry(line)
                    except (ValueError,UnicodeError):
                        report["invalid_frames"]+=1; continue
                    if record is None: continue
                    if previous is not None:
                        delta=(record["seq"]-previous)&0xFFFFFFFF
                        if 0<delta<0x80000000: report["sequence_gaps"]+=delta-1
                    previous=record["seq"]
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
                        progress.update({k: record[k] for k in SEQUENCE_FIELDS if k in record})
                        print(json.dumps(progress), flush=True)
                        last_progress = now
                    transmit(ctl.consume(record,now))
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
    text=json.dumps(report,ensure_ascii=False,indent=2)
    (args.output/(capture+".summary.json")).write_text(text+"\n",encoding="utf8")
    print(text)
    return 0 if report["status"] in {"software_complete","stop_acknowledged","monitor_received"} and report["error"] is None else 2


if __name__=="__main__":
    raise SystemExit(main())
