"""Local image audit; --snapshot explicitly attaches once for read-only MCU state.

Never sends RAM requests, reset/halt/resume, flash, STOP/RUN, UART or camera
commands. No Pi access is implemented. Live state is sampled, not atomic.
"""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import re
import struct
import sys

ROOT = Path(__file__).resolve().parents[1]
CANDIDATE = ROOT / 'evidence/integration/2026-11-18/three-frame-grip-lead20/candidate.json'
OUT = ROOT / 'evidence/integration/2026-11-18/stage3-resume'
sys.path.insert(0, str(ROOT / 'tmp/gripper_debug_deps'))
sys.path.insert(0, str(ROOT / 'src'))
PATTERN = re.compile(
    r'^g_(stage3_|region_dispatch_|.*(?:pc13|key|boot|init|owner|fault|reference)|'
    r'it_(?:action|sequence|state|request|cause|rejected)$|startup_|'
    r'source_parallel_|vehicle_(?:stage|state|cause|run_id|alignment|arm)|'
    r'(?:extension_hold|vm_test|safety_|chassis_auto_))', re.I)
EXTRA = {'vsp_owned', 's3_teach_pending', 'g_src_now_ms', 'g_lift_compare_top_raw',
         'g_source_cycle_state', 'g_source_cycle_request', 'g_source_cycle_fault', 's3_pending',
         'g_lift_ruler_options', 'g_vehicle_lift_current_deg',
         'g_startup_extension_current_deg', 'g_dm_can2_logical_position_deg'}


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def local_audit():
    candidate = json.loads(CANDIDATE.read_text(encoding='utf-8'))
    image = candidate['images']['Vehicle_LiftRuler']
    axf = Path(image['axf'])
    actual = sha256(axf)
    if actual != image['sha256']:
        raise ValueError('Candidate AXF hash mismatch; no hardware access')
    project = Path(candidate['project']).resolve()
    manifest_path = CANDIDATE.with_name('source-manifest.json')
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    mismatches = []
    for name, expected in manifest.items():
        path = (project / name).resolve()
        if not path.is_relative_to(project):
            raise ValueError('Source manifest path escapes candidate project')
        if not path.is_file() or sha256(path) != expected:
            mismatches.append(name)
    return axf, image, {
        'candidate': str(CANDIDATE), 'axf': str(axf), 'axf_sha256': actual,
        'source_manifest': str(manifest_path), 'source_manifest_sha256': sha256(manifest_path),
        'source_files_checked': len(manifest), 'source_mismatches': mismatches,
        'source_manifest_matches': not mismatches,
    }


def scalar_type(die):
    """Resolve actual DWARF scalar encoding; leave unknown/aggregate bytes raw."""
    seen = set()
    while die is not None and die.offset not in seen:
        seen.add(die.offset)
        if die.tag in ('DW_TAG_base_type', 'DW_TAG_enumeration_type'):
            size = die.attributes.get('DW_AT_byte_size')
            enc = die.attributes.get('DW_AT_encoding')
            if size is None:
                return None
            if enc is None and die.tag == 'DW_TAG_enumeration_type':
                return None  # Do not guess whether an enum is signed.
            return {'size': size.value, 'encoding': enc.value}
        if die.tag not in ('DW_TAG_typedef', 'DW_TAG_volatile_type',
                           'DW_TAG_const_type', 'DW_TAG_restrict_type'):
            return None
        die = die.get_DIE_from_attribute('DW_AT_type') if 'DW_AT_type' in die.attributes else None
    return None


def objects_and_types(axf):
    from elftools.elf.elffile import ELFFile
    with axf.open('rb') as stream:
        elf = ELFFile(stream)
        objects = {symbol.name: (int(symbol['st_value']), int(symbol['st_size']))
                   for symbol in elf.get_section_by_name('.symtab').iter_symbols()
                   if symbol['st_info']['type'] == 'STT_OBJECT'
                   and (PATTERN.match(symbol.name) or symbol.name in EXTRA)}
        types = {}
        if elf.has_dwarf_info():
            for cu in elf.get_dwarf_info().iter_CUs():
                for die in cu.iter_DIEs():
                    if die.tag != 'DW_TAG_variable':
                        continue
                    name = die.attributes.get('DW_AT_name')
                    if name is None:
                        continue
                    name = name.value.decode('utf-8', errors='replace')
                    if name in objects and 'DW_AT_type' in die.attributes:
                        typ = scalar_type(die.get_DIE_from_attribute('DW_AT_type'))
                        if typ is not None:
                            types[name] = typ
    return objects, types


def decode(raw, typ):
    if typ and len(raw) == typ['size']:
        enc = typ['encoding']
        if enc == 4 and len(raw) in (4, 8):  # DW_ATE_float
            value = struct.unpack('<f' if len(raw) == 4 else '<d', raw)[0]
            # Strict JSON: preserve nonfinite values as readable strings.
            return value if value == value and abs(value) != float('inf') else str(value)
        if enc in (2, 5, 6, 7, 8):  # boolean/signed/char/unsigned/unsigned-char
            return int.from_bytes(raw, 'little', signed=enc in (5, 6))
    return {'raw_hex': raw.hex(), 'interpretation': 'unknown or aggregate; not guessed'}


class ReadOnlyTarget:
    """Limit the reused adapter to the exact read calls required by snapshot."""
    def __init__(self, target):
        self._target = target

    def get_state(self):
        return self._target.get_state()

    def read32(self, address):
        return self._target.read32(address)

    def read_memory_block8(self, address, count):
        return self._target.read_memory_block8(address, count)

    def read_memory_block32(self, address, count):
        return self._target.read_memory_block32(address, count)


def snapshot(axf, image, objects, types, report):
    # Lazy imports: the default path does not enumerate or attach any device.
    import gripper210_attach_read_20261105 as live
    from source_body_probe_20261111 import attach
    from smart_carrier.vision_manipulation.committed_pick_board import CommittedPickBoard
    live.IMAGE = axf  # Python module variable only; never target RAM.
    symbols, segments = live.load_image()
    if sum(len(data) for _, data in segments) != image['flash_bytes']:
        raise ValueError('Flash segment byte count differs from candidate')
    report['hardware_access_attempted'] = True
    session = attach()
    try:
        session.open()
        report['hardware_access'] = True
        target = ReadOnlyTarget(session.target)
        board = CommittedPickBoard(target, symbols, segments)
        report['verified_flash_bytes'] = board.verified_flash_bytes
        report['flash_matches'] = True
        report['sample_started_at'] = datetime.now().astimezone().isoformat()
        report['board_snapshot'] = board.snapshot()
        values = {}
        report['globals'] = values
        for name, (address, size) in sorted(objects.items()):
            in_ram = any(start <= address < address + size <= end for start, end in
                         ((0x10000000, 0x10010000), (0x20000000, 0x20020000)))
            if not 0 < size <= 65536 or not in_ram:
                values[name] = {'address': hex(address), 'size': size, 'skipped': 'not bounded SRAM object'}
                continue
            raw = bytes(target.read_memory_block8(address, size))
            values[name] = {'address': hex(address), 'size': size,
                            'dwarf_type': types.get(name), 'value': decode(raw, types.get(name))}
            if name in report['board_snapshot'] and types.get(name) and not isinstance(values[name]['value'], dict):
                report['board_snapshot'][name] = values[name]['value']
        report['gpio_c_idr'] = target.read32(0x40020810)
        report['pc13_input_high'] = bool(report['gpio_c_idr'] & (1 << 13))
        report['core_state'] = target.get_state().name
        report['sample_finished_at'] = datetime.now().astimezone().isoformat()
    finally:
        previous_error = sys.exc_info()[0] is not None
        try:
            session.close()
        except Exception as error:
            report['disconnect_error'] = {'type': type(error).__name__, 'message': str(error)}
            if not previous_error:
                raise


def json_safe(value):
    if isinstance(value, float) and (value != value or abs(value) == float('inf')):
        return str(value)
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshot', action='store_true', help='Explicit one-shot read-only probe attachment')
    args = parser.parse_args()
    report = {'created_at': datetime.now().astimezone().isoformat(), 'hardware_access': False,
              'hardware_access_attempted': False,
              'snapshot_requested': args.snapshot, 'ram_writes': 0, 'motion_requests': 0,
              'reset_requested': False, 'halt_requested': False, 'resume_requested': False,
              'flash_requested': False, 'pi_access': False,
              'rules_R': ['RUN-01', 'RUN-02', 'PWR-01', 'MECH-01', 'MECH-02'],
              'engineering_E': ['RUN-06', 'CAL-02', 'CAL-05', 'VIS-15'],
              'official_Q': 'Pending technical questions unchanged',
              'scope': 'Read-only debug evidence; not motion or physical acceptance; samples are non-atomic'}
    exit_code = 0
    try:
        axf, image, audit = local_audit()
        report['local_audit'] = audit
        if not audit['source_manifest_matches']:
            raise ValueError('Candidate sources differ from manifest; no hardware access')
        objects, types = objects_and_types(axf)
        report['selected_symbols'] = len(objects)
        report['typed_scalars'] = len(types)
        if args.snapshot:
            snapshot(axf, image, objects, types, report)
    except Exception as error:
        # Transport exceptions may include host diagnostics: keep only type/message here.
        report['error'] = {'type': type(error).__name__, 'message': str(error)}
        exit_code = 1
    OUT.mkdir(parents=True, exist_ok=True)
    path = OUT / ('readonly-status-' + datetime.now().strftime('%Y%m%d-%H%M%S-%f') + '.json')
    with path.open('x', encoding='utf-8') as stream:
        json.dump(json_safe(report), stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    summary = {key: report.get(key) for key in ('hardware_access', 'flash_matches',
               'verified_flash_bytes', 'selected_symbols', 'typed_scalars', 'error')}
    summary['evidence'] = str(path)
    summary['state'] = {name: row.get('value') for name, row in report.get('globals', {}).items()
                        if name in ('g_region_dispatch_area', 'g_stage3_state', 'g_stage3_fault',
                                    'g_source_cycle_state', 'g_source_cycle_request',
                                    'g_it_state', 'g_it_request', 'g_it_action', 'g_it_sequence')}
    print(json.dumps(summary, ensure_ascii=False, allow_nan=False))
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
