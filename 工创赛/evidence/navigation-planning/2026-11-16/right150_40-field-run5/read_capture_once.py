"""Inspect ELF offline by default; explicit --read-hardware performs one attach read.

Never flash, reset, halt, resume, write target memory, or retry. Importing this
module and running --help do not enumerate probes. Read after the existing tail
log and only when the vehicle is physically stopped; software inactive is not
proof of physical standstill. Use a new output directory for each attempt.
"""
from pathlib import Path
import argparse
import datetime
import hashlib
import json
import struct
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[4]
SYMBOLS = ("g_fusion_capture_a", "g_fusion_capture_b",
           "g_chassis_auto_session", "g_chassis_auto_active")
TAIL = "g_chassis_auto_tail_snapshots"


def require(ok, message):
    if not ok:
        raise ValueError(message)


def sha(data):
    return hashlib.sha256(data).hexdigest()


def stamp():
    return datetime.datetime.now().astimezone().isoformat()


def inspect_elf(path):
    sys.path.insert(0, str(ROOT / "工创赛/tmp/turn_probe_deps"))
    from elftools.elf.elffile import ELFFile
    symbols, flash = {}, []
    with Path(path).open("rb") as stream:
        elf = ELFFile(stream)
        require(elf.little_endian and elf.elfclass == 32
                and elf["e_machine"] == "EM_ARM", "Expected little-endian ARM ELF32")
        table = elf.get_section_by_name(".symtab")
        require(table is not None, "ELF symbol table absent")
        for symbol in table.iter_symbols():
            if symbol.name in SYMBOLS + (TAIL,):
                require(symbol.name not in symbols, "Duplicate ELF symbol")
                symbols[symbol.name] = (int(symbol["st_value"]), int(symbol["st_size"]))
        for index, segment in enumerate(elf.iter_segments()):
            if segment["p_type"] != "PT_LOAD" or not int(segment["p_filesz"]):
                continue
            address, size = int(segment["p_paddr"]), int(segment["p_filesz"])
            require(0x08000000 <= address < address + size <= 0x08080000,
                    "Unknown ELF load layout: all initialized bytes must load from F407VE flash")
            data = segment.data()
            require(len(data) == size, "Incomplete ELF load segment")
            flash.append(("PT_LOAD_" + str(index), address, data))
    require(set(symbols) == set(SYMBOLS + (TAIL,)), "Capture/state/tail symbols missing")
    require(symbols[TAIL][1] == 176, "Expected current 176-byte tail schema")
    for name, (address, size) in symbols.items():
        require(size > 0 and (
            0x10000000 <= address < address + size <= 0x10010000 or
            0x20000000 <= address < address + size <= 0x20020000),
            "Symbol outside F407 RAM: " + name)
        if name in SYMBOLS[2:]:
            require(size == 4, "State symbol must be uint32: " + name)
    a, b = (symbols[n] for n in SYMBOLS[:2])
    require(a[0] + a[1] <= b[0] or b[0] + b[1] <= a[0], "Capture banks overlap")
    require(flash, "No allocated flash bytes to bind to running firmware")
    return symbols, flash


def acquire(target, symbols, flash, expected_session, output, header_reader, decoder):
    """Only target read APIs; injectable target supports offline verification."""
    def state():
        require(target.get_state().name == "RUNNING", "Core not running; will not resume")
        session = target.read32(symbols["g_chassis_auto_session"][0])
        active = target.read32(symbols["g_chassis_auto_active"][0])
        require(session == expected_session and active == 0,
                "Session mismatch or motion backend active; read rejected")
        return session, active

    before = state()
    flash_checks = []
    # Same frozen candidate: current source/image SHA and predecessor full Flash evidence
    # are checked before hardware access. This repeat deliberately does not reread Flash.
    state()
    tail = None
    if TAIL in symbols:
        address, size = symbols[TAIL]
        require(size == 176, "Expected current 176-byte tail object")
        t1 = bytes(target.read_memory_block8(address, size))
        (output / "tail_read1.bin").write_bytes(t1)
        t2 = bytes(target.read_memory_block8(address, size))
        (output / "tail_read2.bin").write_bytes(t2)
        require(len(t1) == size and t1 == t2, "Tail incomplete or changed; no retry")
        version, session, mask = struct.unpack_from("<3I", t1)
        schema_valid = version == 1
        session_match = schema_valid and session == expected_session
        tail = dict(schema_valid=schema_valid, session_match=session_match,
                    version=version if schema_valid else None,
                    session=session if schema_valid else None,
                    valid_mask=mask if schema_valid else None,
                    complete=schema_valid and session_match and mask == 7,
                    sha256=sha(t1), stable_double_read=True,
                    note="Independent forward-compensation tail diagnostic; non-forward plans may leave it uncollected/all-zero. Complete RAW validation is separate. Not ground stopping distance")
    a_addr, a_size = symbols[SYMBOLS[0]]
    b_addr, b_size = symbols[SYMBOLS[1]]
    # Inspect a complete bank rather than guessing header size/offset from an old build.
    a1 = bytes(target.read_memory_block8(a_addr, a_size))
    require(len(a1) == a_size, "Incomplete first A bank")
    (output / "bank_a_read1.bin").write_bytes(a1)
    h = header_reader(a1)
    require(h["frozen"] and h["session"] == expected_session,
            "Capture not frozen or header session mismatch")
    require(h["size_a"] == a_size and h["size_b"] == b_size,
            "Header/ELF size mismatch")
    b1 = bytes(target.read_memory_block8(b_addr, b_size))
    (output / "bank_b_read1.bin").write_bytes(b1)
    a2 = bytes(target.read_memory_block8(a_addr, a_size))
    (output / "bank_a_read2.bin").write_bytes(a2)
    b2 = bytes(target.read_memory_block8(b_addr, b_size))
    (output / "bank_b_read2.bin").write_bytes(b2)
    require(len(b1) == b_size and len(a2) == a_size and len(b2) == b_size,
            "Incomplete capture read")
    require(a1 == a2 and b1 == b2, "Capture changed during read; saved but not retried")
    require(before == state(), "Backend/session changed during read")
    decoded = decoder(a1, b1)
    return dict(header=h, decoded=decoded, tail=tail, flash_checks=flash_checks,
                raw_sha256={"bank_a": sha(a1), "bank_b": sha(b1)},
                stable_double_read=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--axf", type=Path, required=True)
    parser.add_argument("--read-hardware", action="store_true",
                        help="Explicit opt-in; default only inspects ELF offline")
    parser.add_argument("--expected-sha256")
    parser.add_argument("--expected-session", type=int)
    parser.add_argument("--probe-uid")
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    digest = sha(args.axf.read_bytes())
    if args.expected_sha256 is not None:
        require(digest == args.expected_sha256.lower(), "AXF SHA256 mismatch")
    symbols, flash = inspect_elf(args.axf)
    if not args.read_hardware:
        print(json.dumps(dict(mode="offline_elf_only", hardware_accessed=False,
            axf=str(args.axf.resolve()), sha256=digest, symbols=symbols,
            flash_sections=[dict(name=n, address=a, size=len(d), sha256=sha(d))
                            for n, a, d in flash]), ensure_ascii=False, indent=2))
        return
    require(args.expected_sha256 and args.expected_session is not None
            and 0 < args.expected_session <= 0xFFFFFFFF and args.probe_uid
            and args.output_dir, "Hardware read requires SHA, session, probe UID and new output dir")
    # Imports and probe enumeration occur only after explicit hardware opt-in.
    from decode_capture import inspect_header, decode
    sys.path.insert(0, str(ROOT / "工创赛/scripts"))
    from source_stage2_readonly_sync_probe_20261105 import synchronous_interface
    from pyocd.target.pack.pack_target import PackTargets
    from pyocd.core.helpers import ConnectHelper
    from pyocd.core.session import Session
    install=json.loads((HERE/'execution_binding.json').read_text('utf8'))
    require(install['candidate_sha256']==digest, "Current install candidate mismatch")
    require(sha(Path(install['download_receipt']).read_bytes())==install['download_receipt_sha256'], "Install receipt changed")
    prior=json.loads(Path(install['download_receipt']).read_text('utf8'))
    require((prior.get('success') is True or prior.get('user_reported_completed') is True) and prior['candidate_sha256']==digest, "Verified install required")
    args.output_dir.mkdir(parents=True, exist_ok=False)
    receipt = dict(started_at=stamp(), axf=str(args.axf.resolve()), axf_sha256=digest,
        expected_session=args.expected_session, probe_uid=args.probe_uid,
        symbols=symbols, accepted=False, hardware_accessed=False,
        reset_requested=False, halt_requested=False, resume_requested=False,
        target_memory_writes=0, flash_programmed=False, automatic_retry=False)
    session = None
    try:
        PackTargets.populate_targets_from_pack("D:/keil/Arm/Packs/Keil/STM32F4xx_DFP/1.0.8")
        probes = [p for p in ConnectHelper.get_all_connected_probes(blocking=False)
                  if p.unique_id.lower() == args.probe_uid.lower()]
        require(len(probes) == 1, "Exactly the requested probe required")
        probe = probes[0]
        synchronous_interface(probe._link._interface)
        session = Session(probe, options={"target_override": "stm32f407ve",
            "connect_mode": "attach", "frequency": 1000000, "auto_unlock": False,
            "resume_on_disconnect": False, "no_config": True, "dap_protocol": "swd",
            "cmsis_dap.prefer_v1": False, "cmsis_dap.limit_packets": True})
        receipt["hardware_accessed"] = True
        session.open()
        result = acquire(session.target, symbols, flash, args.expected_session,
                         args.output_dir, inspect_header, decode)
        (args.output_dir / "decoded.json").write_text(
            json.dumps(result.pop("decoded"), ensure_ascii=False, indent=2), encoding="utf-8")
        receipt.update(result, accepted=True, flash_read_performed=False, flash_identity_policy="bound successful Keil Programming/Verify/Run install receipt for frozen all-directions-short candidate; no independent fullFlash reread")
    except Exception as exc:
        receipt["error"] = repr(exc)
        raise
    finally:
        if session is not None:
            try:
                session.close()
            except Exception as exc:
                receipt["disconnect_error"] = repr(exc)
        receipt["finished_at"] = stamp()
        (args.output_dir / "read_receipt.json").write_text(
            json.dumps(receipt, ensure_ascii=False, indent=2), encoding="utf-8")
        print(json.dumps(receipt, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
