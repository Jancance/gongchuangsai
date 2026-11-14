"""One authorized, running-core, read-only RAM acquisition. Never retries.

No reset/halt/resume, VM/actuator writes, getter calls, or IRQ masking.
Run only after this run's result_summary.json confirms powered completion.
"""
from pathlib import Path
import datetime
import hashlib
import importlib.util
import json
import sys

E = Path(__file__).resolve().parent
R = Path(__file__).resolve().parents[6]
EXPECTED = '5e803d0def9e33155752d4ceec71b9ca93ab903311407185d5d7af3c4880a185'
IMAGE = R / '工创赛/tmp/tail-snapshot-build-20261115/baseline40_integrated/Vehicle_Integrated.axf'
OBJECTS = {
    'timing': ('g_chassis_auto_lateral_timing', 104, 0x20012768),
    'tail': ('g_chassis_auto_tail_snapshots', 176, 0x2001306c),
}
STATE_NAMES = ['g_chassis_auto_session', 'g_chassis_auto_state',
    'g_chassis_auto_result', 'g_chassis_auto_active', 'g_vm_test_request',
    'g_safety_fault', 'g_rtos_fault', 'g_can_ctrl_fault_latched', 'g_debug_plan_id']


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def stamp():
    return datetime.datetime.now().astimezone().isoformat()


def sha(blob):
    return hashlib.sha256(blob).hexdigest()


def dump_json(name, value):
    (E / name).write_text(json.dumps(value, indent=2, ensure_ascii=False), 'utf8')


def load_parser(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.parse


def main():
    require(sha(IMAGE.read_bytes()) == EXPECTED, 'Matching final ELF SHA required')
    auth = json.loads((E / 'authorization.json').read_text('utf8'))
    candidate = auth.get('candidate_sha256', auth.get('candidate_sha'))
    require(candidate == EXPECTED, 'Authorization candidate SHA mismatch')
    require('candidate_sha' not in auth or auth['candidate_sha'] == EXPECTED, 'Conflicting candidate SHA')
    require(auth.get('diagnostic_reads_authorized') == 1 and auth.get('site_ready') is True,
            'One diagnostic read authorization and site_ready required')
    summary = json.loads((E / 'result_summary.json').read_text('utf8'))
    require(summary.get('status') == 'software_complete' and summary.get('powered_tail_valid') is True,
            'This run must have software_complete and powered_tail_valid')
    last = summary.get('last', {})
    require(last.get('backend_state') == 9 and last.get('backend_result') == 1
            and last.get('backend_active') == 0 and last.get('vm_request') == 1
            and last.get('pe2_active') == 1, 'Summary backend/VM state not eligible')

    sys.path.insert(0, str(R / '工创赛/tmp/turn_probe_deps'))
    sys.path.insert(0, str(R / '工创赛/scripts'))
    from elftools.elf.elffile import ELFFile
    from source_stage2_readonly_sync_probe_20261105 import synchronous_interface
    from pyocd.target.pack.pack_target import PackTargets
    from pyocd.core.helpers import ConnectHelper
    from pyocd.core.session import Session
    from pyocd.core.target import Target

    wanted = set(STATE_NAMES) | {v[0] for v in OBJECTS.values()}
    with IMAGE.open('rb') as stream:
        elf = ELFFile(stream)
        table = elf.get_section_by_name('.symtab')
        require(table is not None, 'ELF symbol table absent')
        symbols = {s.name: (int(s['st_value']), int(s['st_size']))
                   for s in table.iter_symbols() if s.name in wanted}
    require(set(symbols) == wanted, 'Required symbols absent')
    for name, (address, size) in symbols.items():
        require(size > 0 and ((0x10000000 <= address and address + size <= 0x10010000)
            or (0x20000000 <= address and address + size <= 0x20020000)), 'Symbol outside RAM: ' + name)
        if name in STATE_NAMES:
            require(size == 4, 'State must be a uint32: ' + name)
    for name, size, address in OBJECTS.values():
        require(symbols[name] == (address, size), 'Final ELF object layout mismatch: ' + name)

    parsers = {
        'timing': load_parser(R / '工创赛/evidence/navigation-planning/2026-11-15/float-timing-20261115/parse_snapshot.py', 'timing_offline_parser'),
        'tail': load_parser(R / '工创赛/evidence/navigation-planning/2026-11-15/stop-distance-study/snapshot-candidate/parse_snapshot.py', 'tail_offline_parser'),
    }
    receipt = dict(started_at=stamp(), candidate_sha256=EXPECTED, elf=str(IMAGE),
        symbols=symbols, accepted=False, acquisition_status='started',
        hardware_accessed=False, chip_cycles_measured=False, irq_included=True,
        reset=False, halted=False, resumed=False, actuator_requests_written=0,
        vm_writes=0, irq_mask_writes=0, getter_calls=0, automatic_retry=False)
    # Exclusive marker is created before probe enumeration or connection. Never remove/retry it.
    with (E / 'diagnostic_read_started.json').open('x', encoding='utf8') as stream:
        json.dump(receipt, stream, indent=2)
    session = None
    try:
        PackTargets.populate_targets_from_pack('D:/keil/Arm/Packs/Keil/STM32F4xx_DFP/1.0.8')
        probes = [p for p in ConnectHelper.get_all_connected_probes(blocking=False)
                  if p.unique_id.lower() == '6afaae24']
        require(len(probes) == 1, 'Exactly the authorized probe required')
        probe = probes[0]
        synchronous_interface(probe._link._interface)
        session = Session(probe, options={'target_override': 'stm32f407ve', 'connect_mode': 'attach',
            'frequency': 1000000, 'auto_unlock': False, 'resume_on_disconnect': False,
            'no_config': True, 'dap_protocol': 'swd', 'cmsis_dap.prefer_v1': False,
            'cmsis_dap.limit_packets': True})
        receipt['hardware_accessed'] = True
        session.open()
        target = session.target

        def read_state():
            core = target.get_state()
            values = {n: target.read32(symbols[n][0]) for n in STATE_NAMES}
            return dict(core=core.name, values=values,
                        pe2=(target.read32(0x40021010) >> 2) & 1)

        def eligible(state):
            v = state['values']
            return (state['core'] == Target.State.RUNNING.name
                and v['g_chassis_auto_state'] == 9 and v['g_chassis_auto_result'] == 1
                and v['g_chassis_auto_active'] == 0 and v['g_vm_test_request'] == 1
                and state['pe2'] == 1 and v['g_debug_plan_id'] == 8
                and all(v[n] == 0 for n in ['g_safety_fault', 'g_rtos_fault', 'g_can_ctrl_fault_latched']))

        before = read_state()
        receipt['before'] = before
        require(eligible(before), 'Core/backend/power/fault gate rejected; no object reads')
        blobs = {}
        # Exactly four full-object reads; no polling or retry on inconsistency.
        for read_index in (1, 2):
            for key, (name, size, _) in OBJECTS.items():
                blob = bytes(target.read_memory_block8(symbols[name][0], size))
                require(len(blob) == size, 'Incomplete RAM object: ' + key)
                filename = f'{key}_read{read_index}.bin'
                (E / filename).write_bytes(blob)
                blobs[(key, read_index)] = blob
                receipt.setdefault('raw', {})[filename] = dict(size=len(blob), sha256=sha(blob))
        after = read_state()
        receipt['after'] = after
        stable = eligible(after) and before == after and all(
            blobs[(key, 1)] == blobs[(key, 2)] for key in OBJECTS)
        receipt['acquisition_status'] = 'accepted_stable_double_read' if stable else 'unstable'
        if not stable:
            receipt['error'] = 'State/session or object bytes changed; saved once, not retried or trusted'
            return
        decoded = {key: parsers[key](blobs[(key, 1)]) for key in OBJECTS}
        require(decoded['tail']['session'] == before['values']['g_chassis_auto_session'],
                'Tail object session differs from stable backend session')
        measured = bool(decoded['timing']['enabled'] and decoded['timing']['SystemCoreClock']
                        and any(s['valid'] and s['count'] for s in decoded['timing']['scopes']))
        receipt.update(accepted=True, chip_cycles_measured=measured,
            SystemCoreClock=decoded['timing']['SystemCoreClock'],
            counts_and_windows=[{k: scope[k] for k in ['name', 'valid', 'count', 'window_ms']}
                               for scope in decoded['timing']['scopes']],
            hardware_provenance='matching ELF + authorized running-core SWD RAM stable double-read')
        for key, raw_decode in decoded.items():
            dump_json(key + '_parsed.json', dict(raw_parser_output=raw_decode,
                trusted_acquisition=dict(accepted=True, candidate_sha256=EXPECTED,
                    same_session=before['values']['g_chassis_auto_session'],
                    stable_double_read=True, core_running=True, no_halt_reset_vm_write=True,
                    chip_cycles_measured=measured if key == 'timing' else False,
                    irq_included=True if key == 'timing' else None)))
    except Exception as exc:
        receipt['error'] = repr(exc)
        receipt['acquisition_status'] = 'rejected_or_incomplete'
        raise
    finally:
        if session:
            try:
                session.close()
            except Exception as exc:
                receipt['disconnect_error'] = repr(exc)
        receipt['finished_at'] = stamp()
        dump_json('diagnostic_read_receipt.json', receipt)
        print(json.dumps(receipt, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
