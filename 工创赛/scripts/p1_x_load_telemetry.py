"""Decode VM-off XD1 history; historical data never authorize motion."""
import binascii
import bisect
import struct

RECORD = struct.Struct("<IIIhhHHBBBB")
KINDS = {1: "target_tx", 2: "target_ack", 3: "sample", 4: "stop_ack", 5: "staged_ack"}
WHEELS = {1: "left_rear", 2: "right_rear", 3: "left_front", 4: "right_front"}
FORWARD_SIGN = {1: 1, 2: -1, 3: 1, 4: -1}


def parse_integral_telemetry(line):
    line=line.rstrip(b"\r\n")
    if not line.startswith(b"IG1,"):
        return None
    if len(line)>=192:
        raise ValueError("IG1 too long")
    prefix,crc=line.rsplit(b",",1)
    if len(crc)!=4 or binascii.crc_hqx(prefix+b",",0xFFFF)!=int(crc,16):
        raise ValueError("IG1 CRC mismatch")
    fields=prefix.split(b",")
    if len(fields)!=10:
        raise ValueError("IG1 fields")
    session,tick,enabled,ki,limit,peak,last,samples,nonzero=[int(x) for x in fields[1:]]
    if not(0<session<=0xFFFFFFFF and 0<=tick<=0xFFFFFFFF and enabled in (0,1) and 0<=ki<=100 and
           0<=peak<=limit<=300 and abs(last)<=limit and 0<=nonzero<=samples<=0xFFFFFFFF):
        raise ValueError("IG1 bounds")
    return dict(session=session,tick_ms=tick,enabled=bool(enabled),ki=ki/1000,limit_rpm=limit/1000,
                peak_integral_rpm=peak/1000,last_integral_rpm=last/1000,
                controller_samples=samples,nonzero_integral_samples=nonzero)


def parse_load_telemetry(line):
    line = line.rstrip(b"\r\n")
    if not line.startswith(b"XD1,"):
        return None
    if len(line) >= 768:
        raise ValueError("XD1 too long")
    prefix, crc = line.rsplit(b",", 1)
    if len(crc) != 4 or binascii.crc_hqx(prefix + b",", 0xFFFF) != int(crc, 16):
        raise ValueError("XD1 CRC mismatch")
    parts = prefix.split(b",")
    if len(parts) != 7:
        raise ValueError("XD1 field count")
    session, first, count, total, dropped = [int(x) for x in parts[1:6]]
    if not (0 < session <= 0xFFFFFFFF and 1 <= count <= 8 and
            0 <= first < total <= 3072 and first + count <= total and
            0 <= dropped <= 0xFFFFFFFF):
        raise ValueError("XD1 bounds")
    payload = parts[6]
    if len(payload) != count * RECORD.size * 2 or any(c not in b"0123456789abcdefABCDEF" for c in payload):
        raise ValueError("XD1 payload")
    decoded = []
    for i, values in enumerate(RECORD.iter_unpack(bytes.fromhex(payload.decode("ascii")))):
        tick, ack, pos, target, actual, current, voltage, address_kind, result, flags, faults = values
        address, kind = address_kind & 15, address_kind >> 4
        if address not in WHEELS or kind not in KINDS or flags & 0x80 or faults & 0xC0:
            raise ValueError("XD1 record identity")
        decoded.append(dict(index=first+i, tick_ms=tick, target_ack_ms=ack,
                            target_ack_age_ms=(tick-ack) & 0xFFFFFFFF,
                            position_deg=(-1 if flags & 1 else 1)*pos/10,
                            target_rpm=target/10, actual_rpm=actual/10,
                            phase_current_ma=current, bus_voltage_v=voltage/1000,
                            address=address, wheel=WHEELS[address], kind=KINDS[kind],
                            result=result, target_valid=bool(flags & 2),
                            speed_clipped=bool(flags & 4), backend_state=(flags >> 3) & 15,
                            encoder_ready=bool(faults & 1), calibration_ready=bool(faults & 2),
                            overtemperature=bool(faults & 4), overcurrent=bool(faults & 8),
                            stall=bool(faults & 16), stall_protection=bool(faults & 32)))
    return dict(session=session, first_index=first, count=count, total=total,
                dropped=dropped, records=decoded)


class LoadAccumulator:
    def __init__(self):
        self.session = self.total = self.dropped = None
        self.records = {}

    def add(self, packet):
        identity = tuple(packet[k] for k in ("session", "total", "dropped"))
        if self.session is None:
            self.session, self.total, self.dropped = identity
        elif identity != (self.session, self.total, self.dropped):
            raise ValueError("XD1 history changed during export")
        for record in packet["records"]:
            index = record["index"]
            if index in self.records and self.records[index] != record:
                raise ValueError("conflicting XD1 duplicate")
            self.records[index] = record

    @property
    def complete(self):
        return (self.total is not None and self.dropped == 0 and
                len(self.records) == self.total and
                set(self.records) == set(range(self.total)))

    def summary(self):
        return dict(session=self.session, expected_records=self.total,
                    received_records=len(self.records), dropped=self.dropped,
                    complete=self.complete)


def analyze_load(accumulator, live_rows, current_limit_ma=3000):
    """Stage statistics only. Phase current is not a measured wheel torque."""
    if not accumulator.complete:
        raise ValueError("incomplete or overflowed history; do not diagnose wheel loading")
    live = sorted((r for r in live_rows if r.get("distance_mode") == 1 and
                   r.get("backend_active") and r.get("source_valid")), key=lambda r: r["tick_ms"])
    ticks = [r["tick_ms"] for r in live]
    peak = max((r.get("distance_base_rpm", 0) for r in live), default=0)
    peak_tick = next((r["tick_ms"] for r in live if r.get("distance_base_rpm", 0) == peak), 0)
    groups = {}
    excluded = dict(invalid_result=0, no_confirmed_target=0, clipped=0)
    for index in sorted(accumulator.records):
        row = accumulator.records[index]
        if row["kind"] != "sample":
            continue
        if row["result"] != 1:
            excluded["invalid_result"] += 1
            continue
        if not row["target_valid"]:
            excluded["no_confirmed_target"] += 1
            continue
        if row["speed_clipped"]:
            excluded["clipped"] += 1
            continue
        i = bisect.bisect_right(ticks, row["tick_ms"])-1
        stage = "unknown"
        if row["backend_state"] in (7, 8, 9) or row["target_rpm"] == 0:
            stage = "stopping"
        elif i >= 0 and row["tick_ms"]-ticks[i] <= 300:
            base = live[i].get("distance_base_rpm", 0)
            stage = ("near_peak" if peak and base >= .9*peak else
                     "accelerating" if row["tick_ms"] <= peak_tick else "decelerating")
        sign = FORWARD_SIGN[row["address"]]
        target, actual = sign*row["target_rpm"], sign*row["actual_rpm"]
        item = dict(tick_ms=row["tick_ms"], target=target, actual=actual,
                    shortfall=target-actual, current=row["phase_current_ma"])
        groups.setdefault((row["wheel"], stage), []).append(item)
    output = []
    for (wheel, stage), samples in sorted(groups.items()):
        samples.sort(key=lambda r: r["tick_ms"])
        weights = [max(0, min(500, b["tick_ms"]-a["tick_ms"])) for a, b in zip(samples, samples[1:])]
        usable = [(a, w) for a, w in zip(samples, weights) if w and w < 500]
        duration = sum(w for _, w in usable)
        def mean(key):
            return sum(a[key]*w for a, w in usable)/duration if duration else None
        output.append(dict(wheel=wheel, stage=stage, samples=len(samples), covered_ms=duration,
                           mean_target_rpm=mean("target"), mean_actual_rpm=mean("actual"),
                           mean_shortfall_rpm=mean("shortfall"), mean_phase_current_ma=mean("current"),
                           peak_phase_current_ma=max(a["current"] for a in samples),
                           shortfall_over_1rpm_fraction=(sum(w for a, w in usable if a["shortfall"] > 1)/duration if duration else None),
                           near_current_limit_fraction=(sum(w for a, w in usable if a["current"] >= .9*current_limit_ma)/duration if duration else None)))
    return dict(history=accumulator.summary(), phase_statistics=output, excluded=excluded,
                limits="Time-weighted stage statistics; current is not wheel torque, and differently timed wheel samples are not an instantaneous left/right comparison.")
