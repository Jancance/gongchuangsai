"""Offline decoder for capture schema 1. Never imports a hardware driver.

RAW timestamps are service/DRDY proxies. Wheel source_ms is successful response
receipt time, not physical sample time. HWT20Hz/oldOD8Hz are sampled snapshots.
No interpolation, B fusion, zero-velocity assumption, or ground truth is added.
"""
from pathlib import Path
import argparse
import csv
import json
import struct

HEADER_NAMES = """magic version size_a size_b state reason session trigger_ms stop_ms
freeze_ms pre_count pre_head post_count raw_count wheel_count hwt_count od_count
wheel_dropped hwt_dropped od_dropped quality first_sample_sequence last_sample_sequence
first_edge_sequence last_edge_sequence trigger_skipped final_skipped trigger_races final_races
trigger_errors final_errors accel_config gyro_config whoami max_read_lag_cycles
max_read_duration_cycles max_step_cycles reference_set_ms origin_x_mm origin_y_mm
path_heading_deg heading_offset_deg first_ms last_ms last_raw_state last_raw_error
system_core_clock""".split()
SIZE_A, SIZE_B = 34020, 33600
RAW_A, WHEEL_A, HWT_B, OD_B = 256, 25456, 25200, 30240


def require(ok, message):
    if not ok:
        raise ValueError(message)


def delta(a, b):
    return ((a - b + 0x80000000) & 0xffffffff) - 0x80000000


def inspect_header(data):
    require(len(data) >= 256, "Short capture header")
    words = struct.unpack_from("<64I", data)
    h = dict(zip(HEADER_NAMES, words))
    for index in range(38, 42):
        h[HEADER_NAMES[index]] = struct.unpack_from("<f", data, index * 4)[0]
    require(h["magic"] == 0x31434246 and h["version"] == 1, "Unknown capture schema")
    require((h["size_a"], h["size_b"]) == (SIZE_A, SIZE_B), "Capture layout size mismatch")
    require(h["state"] in (0, 1, 2) and h["reason"] in (0, 1, 2, 3), "Unknown capture lifecycle")
    require(0 <= h["pre_count"] <= 100 and 0 <= h["pre_head"] < 100,
            "Invalid prewindow ring")
    require(h["post_count"] <= 2000 and h["raw_count"] == h["pre_count"] + h["post_count"],
            "Invalid raw record count")
    require(h["wheel_count"] <= 512 and h["hwt_count"] <= 210 and h["od_count"] <= 84,
            "Invalid source record count")
    h.update(frozen=h["state"] == 2, truncated=h["reason"] in (2, 3),
             tail_completed=h["state"] == 2 and h["reason"] == 1,
             anchor_valid=bool(words[47]), reference_changed=bool(h["quality"] & 8))
    h.update(terminal_seen=bool(words[51]), first_terminal_receiver_ms=words[52],
             terminal_tail_completed=h["state"] == 2 and h["reason"] == 1 and
                 bool(words[51]) and delta(h["freeze_ms"], words[52]) >= 500,
             stop_time_semantics="first STOP/FAULT/CANCEL/DONE receiver time; not physical standstill",
             terminal_time_semantics="first DONE/FAULT/CANCEL receiver time; not physical standstill")
    return h


def decode(a, b):
    h = inspect_header(a)
    require(len(a) == h["size_a"] and len(b) == h["size_b"], "Short or oversized bank")
    require(h["frozen"], "Only immutable frozen captures can be decoded")
    indices = [(h["pre_head"] - h["pre_count"] + i) % 100 for i in range(h["pre_count"])]
    indices += list(range(100, 100 + h["post_count"]))
    raw = []
    cycles = 0
    time_valid = bool(h["system_core_clock"])
    for ordinal, index in enumerate(indices):
        blob, offset = (a, RAW_A + 24 * index) if index < 1050 else (b, 24 * (index - 1050))
        values = struct.unpack_from("<II7hH", blob, offset)
        item = dict(zip(("service_ms", "edge_cycle", "ax", "ay", "az", "gx", "gy", "gz",
                         "temperature", "edge_delta"), values))
        item["ordinal"] = ordinal
        item["relative_service_ms"] = delta(item["service_ms"], h["trigger_ms"])
        if raw:
            service_dt = delta(item["service_ms"], raw[-1]["service_ms"])
            cycle_dt = (item["edge_cycle"] - raw[-1]["edge_cycle"]) & 0xffffffff
            time_valid = time_valid and 1 <= service_dt <= 20 and 0 < cycle_dt < 0x80000000
            cycles += cycle_dt
        item["edge_time_valid"] = time_valid
        item["relative_edge_cycles"] = cycles if time_valid else None
        item["relative_edge_s"] = cycles / h["system_core_clock"] if time_valid else None
        raw.append(item)
    edge = h["last_edge_sequence"]
    for index in range(len(raw) - 1, -1, -1):
        raw[index]["edge_sequence"] = edge
        edge = (edge - raw[index]["edge_delta"]) & 0xffffffff
    seq_usable = not (h["quality"] & 2) and (
        (h["last_sample_sequence"] - h["first_sample_sequence"]) & 0xffffffff) == max(0, len(raw) - 1)
    if seq_usable:
        for index, row in enumerate(raw):
            row["sample_sequence"] = (h["first_sample_sequence"] + index) & 0xffffffff
    if h["quality"] & 4:
        for row in raw:
            row["edge_sequence"] = None
    wheel = []
    for index in range(h["wheel_count"]):
        now, ms, value, meta = struct.unpack_from("<4I", a, WHEEL_A + index * 16)
        source = (meta >> 8) & 255
        require(source in (1, 2) and (meta & 255) < 4, "Unknown wheel source")
        wheel.append(dict(observed_ms=now, source_ms=ms, source_age_ms=delta(now, ms),
                          wheel=meta & 255, source="position" if source == 1 else "speed",
                          valid=bool(meta & (1 << 17)), sign=(meta >> 16) & 1,
                          value=value if source == 1 else struct.unpack("<i", struct.pack("<I", value))[0]))
    hwt = []
    for index in range(h["hwt_count"]):
        now, angle, rate, yaw, dps, flags = struct.unpack_from("<IIIffI", b, HWT_B + index * 24)
        hwt.append(dict(observed_ms=now, angle_ms=angle, rate_ms=rate, yaw_deg=yaw,
                        rate_dps=dps, healthy=bool(flags & 1), valid_mask=(flags >> 8) & 3,
                        angle_age_ms=delta(now, angle), rate_age_ms=delta(now, rate)))
    od = []
    for index in range(h["od_count"]):
        now, ms, x, y, yaw, updates, reasons, continuity, motion, faults = struct.unpack_from(
            "<IIfff5I", b, OD_B + index * 40)
        od.append(dict(observed_ms=now, sample_ms=ms, sample_age_ms=delta(now, ms),
                       x_mm=x, y_mm=y, heading_deg=yaw, updates=updates, reasons=reasons,
                       continuity_lost=continuity, state=motion & 255, result=(motion >> 8) & 255,
                       cause=(motion >> 16) & 255, active=bool(motion & (1 << 24)),
                       valid=bool(motion & (1 << 25)), vm_request=bool(motion & (1 << 26)), faults=faults))
    gaps = [i for i in range(1, len(raw)) if raw[i]["edge_delta"] != 1 or not (
        1 <= delta(raw[i]["service_ms"], raw[i-1]["service_ms"]) <= 20)]
    return dict(header=h, raw=raw, wheel=wheel, hwt=hwt, od=od,
                validation=dict(retained_gap_indices=gaps, sample_sequences_reconstructible=seq_usable,
                                source_events_complete=not any(h[n] for n in ("wheel_dropped", "hwt_dropped", "od_dropped")),
                                edge_time_valid=all(row["edge_time_valid"] for row in raw),
                                hardware_provenance_established=False, ground_accuracy_established=False),
                notes=["quality flags include boot/pre lifetime diagnostics; gap_indices checks retained records",
                       "source_ms are receiver times; wheel position and speed are distinct asynchronous sources",
                       "20Hz HWT and 8Hz old OD are sampled snapshots; PRE contains raw IMU only",
                       "STOP and inactive do not establish physical standstill or true zero velocity",
                       "edge cycle time requires stored core clock; DWT wrap uses modular differences"])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bank_a", type=Path)
    parser.add_argument("bank_b", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    result = decode(args.bank_a.read_bytes(), args.bank_b.read_bytes())
    args.out.mkdir(parents=True, exist_ok=False)
    for name in ("raw", "wheel", "hwt", "od"):
        rows = result[name]
        if rows:
            with (args.out / (name + ".csv")).open("w", newline="", encoding="utf8") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    (args.out / "decoded.json").write_text(json.dumps(result, indent=2, allow_nan=False), "utf8")
    print(json.dumps(dict(header=result["header"], validation=result["validation"]), indent=2))


if __name__ == "__main__":
    main()
