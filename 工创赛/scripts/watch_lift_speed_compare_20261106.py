"""Read-only per-operation capture of lift comparison and VM request/PE2."""
from pathlib import Path
import argparse
import hashlib
import json
import struct
import time
import source_stage2_readonly_sync_probe_20261105 as transport
from pyocd.core.helpers import ConnectHelper
from pyocd.core.session import Session
from pyocd.core.target import Target

ROOT = Path(__file__).resolve().parents[1]
IMAGE = ROOT / 'tmp/lift_speed_compare_20261106/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf'
OUT = ROOT / 'evidence/integration/2026-11-06/lift-speed-compare'
NAMES = ['g_lift_ruler_state', 'g_lift_ruler_request', 'g_lift_ruler_cause', 'g_lift_ruler_phase',
         'g_lift_ruler_samples', 'g_lift_ruler_stop_pending', 'g_lift_ruler_stop_unconfirmed',
         'g_lift_ruler_before_raw', 'g_lift_ruler_after_raw', 'g_lift_ruler_target_raw',
         'g_lift_compare_operation', 'g_lift_compare_top_valid', 'g_lift_compare_top_raw',
         'g_lift_compare_home_flags', 'g_lift_compare_home_seen', 'g_lift_compare_home_reported',
         'g_lift_compare_home_exit', 'g_lift_compare_speed_raw', 'g_lift_compare_move_elapsed_ms',
         'g_vm_test_request', 'g_safety_fault', 'g_rtos_fault', 'g_can_ctrl_fault_latched', 'g_chassis_auto_active']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--operation', type=int, required=True, choices=[1, 2, 3])
    args = parser.parse_args()
    out = OUT / f'operation-{args.operation}'
    out.mkdir(parents=True, exist_ok=True)
    assert not (out / 'samples.jsonl').exists(), 'Preserve earlier capture'
    ready = json.loads((OUT / 'ready-manifest.json').read_text(encoding='utf-8'))
    assert hashlib.sha256(IMAGE.read_bytes()).hexdigest() == ready['builds']['online']['sha256']
    transport.live.IMAGE = IMAGE
    symbols, segments = transport.live.load_image()
    assert all(n in symbols for n in NAMES)
    options = {'target_override': 'cortex_m', 'connect_mode': 'attach', 'frequency': 1000000,
               'auto_unlock': False, 'resume_on_disconnect': False, 'dap_protocol': 'swd',
               'no_config': True, 'cmsis_dap.prefer_v1': False, 'cmsis_dap.limit_packets': True}
    Session(None, options=options)
    probes = ConnectHelper.get_all_connected_probes(blocking=False)
    assert len(probes) == 1 and probes[0].unique_id == '6afaae24'
    transport.synchronous_interface(probes[0]._link._interface)
    session = Session(probes[0], options=options)
    report = {'operation_expected': args.operation, 'actuator_writes': 0, 'reset_requested': False,
              'flash_requested': False, 'halt_requested': False, 'VM_PE2_drop_observed_after_first_high': False,
              'VM_pin_readback_is_electrical_voltage_measurement': False}
    try:
        session.open()
        target = session.target
        compared = 0
        for address, data in segments:
            for offset in range(0, len(data), 4096):
                expected = data[offset:offset+4096]
                assert bytes(target.read_memory_block8(address+offset, len(expected))) == expected
                compared += len(expected)
        report['flash_bytes_compared'] = compared
        start = time.monotonic()
        first = True
        vm_seen = False
        terminal_at = None
        previous = None
        with (out / 'samples.jsonl').open('w', encoding='utf-8') as log:
            while time.monotonic() - start < 240:
                assert target.get_state() == Target.State.RUNNING
                snap = {'elapsed_s': round(time.monotonic() - start, 3)}
                for n in NAMES:
                    raw = target.read32(symbols[n])
                    snap[n] = struct.unpack('<i', struct.pack('<I', raw))[0] if n.endswith('_raw') else raw
                snap['VM_PE2_high'] = bool(target.read32(0x40021014) & 4)
                if snap['VM_PE2_high']:
                    vm_seen = True
                elif vm_seen:
                    report['VM_PE2_drop_observed_after_first_high'] = True
                log.write(json.dumps(snap) + '\n')
                log.flush()
                report['last'] = snap
                if first:
                    # A normally completed full return may explicitly start a
                    # further down test without resetting its live reference.
                    assert (snap['g_lift_compare_operation'] == args.operation - 1 or
                            (args.operation == 2 and snap['g_lift_compare_operation'] == 3))
                    assert snap['g_lift_ruler_state'] in [0, 4] and not snap['g_lift_ruler_request']
                    print(f'WATCH_READY operation={args.operation}, no motion or request writes', flush=True)
                    first = False
                signature = (snap['g_lift_ruler_state'], snap['g_lift_ruler_phase'], snap['g_lift_ruler_cause'], snap['g_lift_compare_operation'])
                if signature != previous:
                    print(json.dumps({k: snap[k] for k in ['elapsed_s', 'g_lift_ruler_state', 'g_lift_ruler_phase', 'g_lift_ruler_cause', 'g_lift_compare_operation', 'VM_PE2_high']}), flush=True)
                previous = signature
                finished = snap['g_lift_compare_operation'] == args.operation and snap['g_lift_ruler_state'] == 4
                failed = snap['g_lift_ruler_state'] >= 5 and not snap['g_lift_ruler_stop_pending']
                if finished or failed:
                    if terminal_at is None:
                        terminal_at = time.monotonic()
                    if time.monotonic() - terminal_at >= 1:
                        report['normal_done_observed'] = finished
                        break
                else:
                    terminal_at = None
                time.sleep(.05)
        report['VM_high_observed'] = vm_seen
    except Exception as exc:
        report['error'] = repr(exc)
    finally:
        session.close()
        (out / 'summary.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
        print(json.dumps(report), flush=True)
    return 0 if 'error' not in report else 1


if __name__ == '__main__':
    raise SystemExit(main())
