"""Read-only encoder/profile capture. Default only inspects local ELF; never sends actions."""
from pathlib import Path
from datetime import datetime
import argparse
import hashlib
import json
import struct
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tmp/gripper_debug_deps'))

FLOATS = ('s_distance_target_mm', 's_distance_progress_mm', 's_profile_follow_rpm',
          'g_source_align_distance_mm', 'g_hwt_yaw_deg')
UINTS = ('g_src_now_ms', 'g_chassis_auto_session', 'g_chassis_auto_active',
         'g_chassis_auto_result', 'g_chassis_auto_cause', 'g_source_align_command_seq',
         'g_source_body_step_state', 'g_source_body_step_reason',
         's_translation_mode', 's_translation_base_rpm', 's_distance_mode',
         's_distance_reached', 's_profile_done', 's_endpoint_count', 's_position_valid')
BLOCKS = ('s_position', 's_endpoints', 's_distance_reference',
          'g_chassis_auto_stop_mask', 's_position_tick')


def load_elf(path):
    from elftools.elf.elffile import ELFFile
    with path.open('rb') as stream:
        elf = ELFFile(stream)
        needed = set(FLOATS + UINTS + BLOCKS)
        found = {}
        for symbol in elf.get_section_by_name('.symtab').iter_symbols():
            if symbol.name in needed and symbol['st_info']['type'] == 'STT_OBJECT':
                if symbol.name in found:
                    raise ValueError('Ambiguous ELF symbol: ' + symbol.name)
                found[symbol.name] = (int(symbol['st_value']), int(symbol['st_size']))
        assert needed == set(found), ('Missing actual ELF symbols', sorted(needed-set(found)))
        assert all(found[n][1] == 4 for n in FLOATS+UINTS)
        assert found['s_position'][1] == 100
        assert found['s_endpoints'][1] == 960
        assert found['s_distance_reference'][1] == 16
        assert found['s_position_tick'][1] == 16
        segments = [(int(s['p_paddr']), s.data()) for s in elf.iter_segments()
                    if s['p_type'] == 'PT_LOAD' and s['p_filesz'] and
                    0x08000000 <= int(s['p_paddr']) < 0x08100000]
    assert segments, 'No flash segments'
    return found, segments


def snapshot(target, symbols):
    def words(name):
        address, size = symbols[name]
        return list(target.read_memory_block32(address, size//4))
    sample = dict(host_time=datetime.now().isoformat(), tick_before=words('g_src_now_ms')[0],
                  session_before=words('g_chassis_auto_session')[0])
    for name in UINTS:
        sample[name] = words(name)[0]
    for name in FLOATS:
        sample[name] = struct.unpack('<f', struct.pack('<I', words(name)[0]))[0]
    sample['raw_words'] = {name: words(name) for name in BLOCKS}
    position = sample['raw_words']['s_position']
    sample['position'] = dict(zip(('stored_enabled', 'session', 'sequence', 'stored_vm_active', 'stored_valid_mask'), position[:5]))
    for index, key in enumerate(('result', 'sign', 'raw', 'stored_sample_ms', 'stored_age_ms')):
        sample['position'][key] = position[5+4*index:9+4*index]
    sample['position']['actual_poll_tick_raw'] = sample['raw_words']['s_position_tick']
    sample['position']['actual_poll_valid_mask'] = sample['s_position_valid']
    reference = sample['raw_words']['s_distance_reference']
    sample['profile_reference'] = dict(zip(('position_mm', 'speed_mm_s', 'acceleration_mm_s2', 'jerk_mm_s3'),
        struct.unpack('<4f', struct.pack('<4I', *reference))))
    sample['tick_after'] = words('g_src_now_ms')[0]
    sample['session_after'] = words('g_chassis_auto_session')[0]
    sample['same_session_during_read'] = sample['session_before'] == sample['session_after']
    # Separate SWD reads are not an atomic snapshot. Keep all raw data/ticks;
    # a matching session does not prove simultaneous four-wheel observations.
    return sample


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--image', type=Path)
    source.add_argument('--candidate', type=Path)
    parser.add_argument('--sha256', help='Required with explicit --image')
    parser.add_argument('--watch', type=float, metavar='SECONDS')
    parser.add_argument('--interval', type=float, default=.15)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.candidate:
        candidate = json.loads(args.candidate.read_text(encoding='utf-8'))
        image = candidate['images']['Vehicle_LiftRuler']
        path, digest = ROOT / image['axf'], image['sha256']
    else:
        assert args.sha256, '--image requires --sha256 from reviewed build evidence'
        path, digest = args.image, args.sha256
    path = path.resolve()
    assert hashlib.sha256(path.read_bytes()).hexdigest() == digest, 'Local image SHA mismatch'
    symbols, segments = load_elf(path)
    report = dict(image=str(path), sha256=digest, symbols=symbols,
                  hardware_access=False, samples=[],
                  meaning='Encoder/profile evidence only; not ruler-measured body travel. Non-atomic reads.')
    if args.watch is None:
        print(json.dumps(report, ensure_ascii=False))
        return
    assert 0 < args.watch <= 600 and args.interval >= .05
    assert args.output, '--watch requires --output to preserve evidence'
    from source_body_probe_20261111 import attach
    args.output.parent.mkdir(parents=True, exist_ok=True)
    try:
        with attach() as session:
            target = session.target
            report['hardware_access'] = True
            verified = 0
            for address, expected in segments:
                actual = bytes(target.read_memory_block8(address, len(expected)))
                assert actual == expected, f'Live flash mismatch at {address:#x}; no capture'
                verified += len(expected)
            report['verified_flash_bytes'] = verified
            started = time.monotonic()
            while True:
                report['samples'].append(snapshot(target, symbols))
                if time.monotonic()-started >= args.watch:
                    break
                time.sleep(args.interval)
    except BaseException as error:
        report['error'] = f'{type(error).__name__}: {error}'
        raise
    finally:
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(dict(output=str(args.output.resolve()), samples=len(report['samples']),
                          verified_flash_bytes=report['verified_flash_bytes'])))


if __name__ == '__main__':
    main()
