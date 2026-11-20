"""After a supervised run has physically stopped, read its frozen Zone 2 trace over SWD."""

import argparse
import hashlib
from io import BytesIO
import json
from pathlib import Path
import sys

from decode_zone2_pose_trace import CAPACITY, HEADER, MAGIC, ROW, decode, locate
import decode_zone2_route_trace as route_trace


def image_flash(image, regions):
    from elftools.elf.elffile import ELFFile

    elf = ELFFile(BytesIO(image))
    table = elf.get_section_by_name(".symtab")
    expected = sorted((region["symbol"], int(region["address"], 16), region["size_bytes"])
                      for region in regions)
    symbols = sorted((symbol.name, symbol["st_value"], symbol["st_size"])
                     for symbol in table.iter_symbols()
                     if symbol.name in ("g_zone2_pose_trace", route_trace.SYMBOL)) if table is not None else []
    if symbols != expected:
        raise ValueError("AXF and MAP disagree on the trace objects")
    segments = [(segment["p_paddr"], segment.data()) for segment in elf.iter_segments()
                if segment["p_type"] == "PT_LOAD" and segment["p_filesz"]]
    if not segments or any(
        not 0x08000000 <= start <= 0x08080000 - len(data) for start, data in segments
    ):
        raise ValueError("AXF load data does not fit the configured internal Flash")
    return segments


def verify_flash(target, segments):
    compared = 0
    for start, data in segments:
        for offset in range(0, len(data), 4096):
            expected = data[offset:offset + 4096]
            if bytes(target.read_memory_block8(start + offset, len(expected))) != expected:
                raise ValueError(f"Installed Flash differs from AXF at {start + offset:#010x}")
            compared += len(expected)
    return compared


def read_pair(target, address, size, running_state, output, stem):
    first = bytes(target.read_memory_block8(address, size))
    (output / f"{stem}-read1.bin").write_bytes(first)
    second = bytes(target.read_memory_block8(address, size))
    (output / f"{stem}-read2.bin").write_bytes(second)
    if len(first) != size or first != second or target.get_state() != running_state:
        raise ValueError(f"Frozen {stem} or target state changed during read")
    return first


def read_frozen(target, address, running_state, output):
    if target.get_state() != running_state:
        raise ValueError("Target core is not running")
    header = bytes(target.read_memory_block8(address, HEADER.size))
    magic, count, state, _, _ = HEADER.unpack(header)
    if magic != MAGIC or state != 2 or not 1 <= count <= CAPACITY:
        raise ValueError("No frozen Z2T1 trace at the address in this build's map")
    size = HEADER.size + ROW.size * count
    blob = read_pair(target, address, size, running_state, output, "pose-trace")
    if blob[:HEADER.size] != header:
        raise ValueError("Pose trace header changed during read")
    return decode(blob)[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--axf", type=Path, required=True, help="Exact installed AXF, with its Keil map beside it")
    parser.add_argument("--probe", required=True, help="Exact UID of the connected CMSIS-DAPv2 probe")
    parser.add_argument("--out", type=Path, required=True, help="New directory for this run's raw evidence")
    args = parser.parse_args()

    map_path = args.axf.with_suffix(".map")
    region = locate(map_path)
    route_region = route_trace.locate(map_path, required=False)
    address = int(region["address"], 16)
    image = args.axf.read_bytes()
    candidate_sha256 = hashlib.sha256(image).hexdigest()
    dependencies = Path(__file__).resolve().parents[1] / "tmp/turn_probe_deps"
    sys.path.insert(0, str(dependencies))
    from pyocd.core.helpers import ConnectHelper
    from pyocd.core.session import Session
    from pyocd.core.target import Target
    from pyocd.target.pack.pack_target import PackTargets
    from source_stage2_readonly_sync_probe_20261105 import synchronous_interface

    regions = [region, route_region] if route_region else [region]
    segments = image_flash(image, regions)
    args.out.mkdir()
    PackTargets.populate_targets_from_pack("D:/keil/Arm/Packs/Keil/STM32F4xx_DFP/1.0.8")
    probes = [probe for probe in ConnectHelper.get_all_connected_probes(blocking=False)
              if probe.unique_id.casefold() == args.probe.casefold()]
    if len(probes) != 1:
        raise ValueError("Expected one CMSIS-DAPv2 probe with the selected UID")
    probe = probes[0]
    interface = probe._link._interface
    if not getattr(interface, "is_bulk", False):
        raise ValueError("The selected probe does not expose a CMSIS-DAPv2 bulk interface")
    synchronous_interface(interface)
    with Session(probe, options={
        "target_override": "stm32f407ve", "connect_mode": "attach", "frequency": 1000000,
        "auto_unlock": False, "resume_on_disconnect": False, "no_config": True,
        "dap_protocol": "swd", "cmsis_dap.prefer_v1": False, "cmsis_dap.limit_packets": True,
    }) as session:
        target = session.target
        if target.get_state() != Target.State.RUNNING:
            raise ValueError("Target core is not running")
        flash_bytes_verified = verify_flash(target, segments)
        summary = read_frozen(target, address, Target.State.RUNNING, args.out)
        if route_region:
            route_blob = read_pair(target, int(route_region["address"], 16), route_region["size_bytes"],
                                   Target.State.RUNNING, args.out, "route-trace")
            route = route_trace.decode(route_blob, summary)
            (args.out / "route-trace.json").write_text(json.dumps(route, indent=2) + "\n", encoding="utf-8")
            summary["route_trace"] = dict(**route_region, trace_sha256=route["trace_sha256"])
    receipt = dict(summary, **region, candidate_sha256=candidate_sha256,
                   flash_bytes_verified=flash_bytes_verified,
                   probe_uid=args.probe, matching_second_read=True)
    text = json.dumps(receipt, indent=2) + "\n"
    (args.out / "pose-trace-read.json").write_text(text, encoding="utf-8")
    print(text, end="")


if __name__ == "__main__":
    main()
