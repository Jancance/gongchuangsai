"""Decode a frozen Zone 2 map, eight segment records and terminal result."""

import argparse
import hashlib
import json
from pathlib import Path
import struct

from decode_zone2_pose_trace import PHASES, decode as decode_pose, locate_trace


SYMBOL = "g_zone2_route_trace"
HEADER = struct.Struct("<14I4B")
LEG = struct.Struct("<5IH6B24B")
STATIONS = (0, 2, 14, 10, 22, 14, 10, 22, 0)
SOURCES = (1, 2, 3, 4, 5, 3, 4, 8)
SIZE = HEADER.size + LEG.size * len(SOURCES)


def locate(map_path, required=True):
    return locate_trace(map_path, SYMBOL, "zone2_nav_schedule", SIZE, required)


def decode(blob, pose_summary=None):
    if len(blob) != SIZE:
        raise ValueError("Route trace has an unexpected size")
    (magic, state, fault, path_error, start_ms, elapsed, grid_valid, occupied,
     run, generation, layout, link, scan, completed, start_id, node, zone, phase) = HEADER.unpack_from(blob)
    if magic != 0x3152325A or state != 2 or phase not in (7, 10):
        raise ValueError("Route trace is not a frozen Z2R1 recording")
    if (phase == 10 and (fault or path_error or completed != 8)) or (phase == 7 and not fault):
        raise ValueError("Route terminal phase disagrees with its result")
    summary = dict(terminal_phase=PHASES[phase], fault=fault, path_error=path_error,
                   elapsed_ms=elapsed, completed_legs=completed)
    if pose_summary is not None and any(
        summary[key] != pose_summary[key]
        for key in ("terminal_phase", "fault", "elapsed_ms", "completed_legs")
    ):
        raise ValueError("Route and pose terminal records disagree")

    grid = None
    if grid_valid == 1:
        if occupied >> 25 or not all((run, generation, layout, link)) or (start_id, node, zone) != (2, 1, 2):
            raise ValueError("Route trace has an invalid map identity or occupancy mask")
        grid = dict(run_id=run, map_generation=generation, layout_id=layout, link_generation=link,
                    scan_sequence=scan, start_id=start_id, node_id=node, zone=zone,
                    occupied_mask=occupied, occupied_ids=[i for i in range(25) if occupied & (1 << i)])
    elif grid_valid or any((occupied, run, generation, layout, link, scan, start_id, node, zone)):
        raise ValueError("Absent map contains data")

    records = []
    prior_stop = 0
    arrived = 0
    for index, wire in enumerate(LEG.iter_unpack(blob[HEADER.size:])):
        seen, begun, stopped, stop_source, session, seq, status, start, goal, count, source, flags, *ids = wire
        if not flags:
            if any(wire):
                raise ValueError("Absent leg contains data")
            continue
        leg = index + 1
        label = f"Leg {leg}"
        if not grid or index != arrived or flags not in (1, 3, 7) or source != SOURCES[index]:
            raise ValueError(f"{label} has inconsistent event ordering or provenance")
        if (start, goal) != STATIONS[index:index + 2] or status or not 1 <= count <= 24:
            raise ValueError(f"{label} has inconsistent PATH metadata")
        waypoints = ids[:count]
        if any(cell >= 25 for cell in waypoints) or waypoints[-1] != goal or any(ids[count:]):
            raise ValueError(f"{label} has inconsistent PATH waypoints")
        if not prior_stop <= seen <= elapsed:
            raise ValueError(f"{label} path observation time is out of order")
        path = dict(sequence=seq, status=status, start=start, goal=goal, waypoints=waypoints)
        if source != leg and path != records[source - 1]["path"]:
            raise ValueError(f"{label} disagrees with its source leg's cached PATH")
        entry = dict(leg=leg, source_leg=source, path=path, observed_elapsed_ms=seen)
        if flags & 2:
            if not session or not seen <= begun <= elapsed:
                raise ValueError(f"{label} has inconsistent motion session timing")
            entry.update(begin_elapsed_ms=begun, session=session)
        elif begun or session:
            raise ValueError(f"{label} has data for motion that did not begin")
        if flags & 4:
            if not begun <= stopped <= elapsed:
                raise ValueError(f"{label} stop time is out of order")
            entry.update(arrive_elapsed_ms=stopped, stop_source_ms=stop_source)
            prior_stop = stopped
            arrived += 1
        elif stopped or stop_source:
            raise ValueError(f"{label} has data for a stop that did not complete")
        records.append(entry)
    if arrived != completed:
        raise ValueError("Route stop records disagree with the terminal station count")
    if phase == 10 and pose_summary is not None and records[-1]["stop_source_ms"] != pose_summary["terminal_source_ms"]:
        raise ValueError("Route and pose final stop source timestamps disagree")
    summary.update(run_start_ms=start_ms, grid=grid, legs=records, trace_sha256=hashlib.sha256(blob).hexdigest())
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dump", type=Path)
    parser.add_argument("--pose", type=Path, help="Matching frozen Z2T1 pose export")
    args = parser.parse_args()
    try:
        pose_summary = decode_pose(args.pose.read_bytes())[1] if args.pose else None
        result = decode(args.dump.read_bytes(), pose_summary)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
