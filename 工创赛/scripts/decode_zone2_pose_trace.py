"""Locate a Zone 2 trace in a Keil map and decode a frozen RAM export to CSV."""

import argparse
import csv
import hashlib
import json
from pathlib import Path
import re
import struct


HEADER = struct.Struct("<5I")
ROW = struct.Struct("<IIhhhBB")
CAPACITY = 2048
TRACE_SIZE = HEADER.size + CAPACITY * ROW.size
MAGIC = 0x3154325A
PHASES = (
    "IDLE", "WAIT_READY", "SCAN", "PREPARE", "ZERO", "STOP",
    "SHADOW_DONE", "FAILED", "TRACK", "SECTION_ZERO", "DONE",
    "QR_WAIT", "STATION_WAIT", "MISSION_DONE",
)
TERMINAL = {7, 10, 13}
COLUMNS = (
    "elapsed_ms", "source_ms", "phase", "completed_legs",
    "valid", "x_mm", "y_mm", "heading_deg", "terminal",
)


def locate(map_path):
    return locate_trace(map_path, "g_zone2_pose_trace", "app_zone2", TRACE_SIZE)


def locate_trace(map_path, symbol, module, expected_size, required=True):
    matches = re.findall(
        rf"^\s*{re.escape(symbol)}\s+(0x[0-9a-fA-F]+)\s+Data\s+(\d+)\s+{re.escape(module)}\.o\(",
        map_path.read_text(encoding="ascii", errors="replace"),
        flags=re.MULTILINE,
    )
    if not matches and not required:
        return None
    if len(matches) != 1:
        raise ValueError(f"Expected exactly one {symbol} in the Keil map")
    address, size = (int(matches[0][0], 16), int(matches[0][1]))
    if size != expected_size or not any(
        start <= address and address + size <= end
        for start, end in ((0x10000000, 0x10010000), (0x20000000, 0x20020000))
    ):
        raise ValueError("Zone 2 trace symbol size or address does not match the firmware ABI")
    return {
        "symbol": symbol, "address": f"0x{address:08x}",
        "size_bytes": size, "end_exclusive": f"0x{address + size:08x}",
    }


def decode(blob):
    if len(blob) < HEADER.size:
        raise ValueError("Trace header is incomplete")
    magic, count, state, fault, flags = HEADER.unpack_from(blob)
    if magic != MAGIC or state != 2 or not 1 <= count <= CAPACITY:
        raise ValueError("Trace is not a frozen Z2T1 recording")
    if flags & ~1 or flags == 1 and count != CAPACITY:
        raise ValueError("Trace overflow metadata is inconsistent")
    used = HEADER.size + ROW.size * count
    if len(blob) not in (used, TRACE_SIZE):
        raise ValueError("Trace byte count does not match the header")

    rows = []
    for index, (elapsed, source, x, y, yaw, phase, meta) in enumerate(
        ROW.iter_unpack(blob[HEADER.size:used])
    ):
        final = index == count - 1
        valid = bool(meta & 16)
        legs = meta & 15
        if (
            phase >= len(PHASES) or legs > 8 or meta & ~63
            or bool(meta & 32) != final or (phase in TERMINAL) != final
        ):
            raise ValueError(f"Invalid phase, station count, or flags at row {index}")
        if (
            (not index and elapsed != 0) or
            (index and (elapsed < rows[-1][0] or not final and elapsed - rows[-1][0] < 100))
            or (index and legs < rows[-1][3])
        ):
            raise ValueError(f"Inconsistent time or station count at row {index}")
        if (not valid and (x or y or yaw)) or (valid and abs(yaw) > 18000):
            raise ValueError(f"Invalid pose payload at row {index}")
        pose = (x / 10, y / 10, yaw / 100) if valid else ("", "", "")
        rows.append((elapsed, source, PHASES[phase], legs, int(valid), *pose, int(final)))

    final = rows[-1]
    if (final[2] == "DONE" and (fault or final[3] != 8)) or (final[2] == "FAILED" and not fault):
        raise ValueError("Trace result is inconsistent with its terminal record")
    summary = {
        "records": count, "valid_pose_records": sum(row[4] for row in rows),
        "periodic_overflow": bool(flags & 1), "terminal_phase": final[2],
        "completed_legs": final[3], "fault": fault, "elapsed_ms": final[0],
        "terminal_source_ms": final[1],
        "trace_sha256": hashlib.sha256(blob).hexdigest(),
    }
    return rows, summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    locate_parser = sub.add_parser("locate")
    locate_parser.add_argument("map_file", type=Path)
    decode_parser = sub.add_parser("decode")
    decode_parser.add_argument("dump", type=Path)
    decode_parser.add_argument("csv_file", type=Path)
    decode_parser.add_argument("--compare", type=Path, help="Second read of the same frozen range")
    args = parser.parse_args()
    try:
        if args.command == "locate":
            result = locate(args.map_file)
        else:
            raw = args.dump.read_bytes()
            if args.compare and raw != args.compare.read_bytes():
                raise ValueError("The two RAM exports differ")
            rows, result = decode(raw)
            with args.csv_file.open("x", newline="", encoding="utf-8") as stream:
                writer = csv.writer(stream)
                writer.writerow(COLUMNS)
                writer.writerows(rows)
            result["matching_second_read"] = args.compare is not None
    except (OSError, ValueError) as error:
        parser.error(str(error))
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
