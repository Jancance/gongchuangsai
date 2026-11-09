"""Read-only watcher for the isolated lift calibration. Never starts motion."""
from pathlib import Path
import json
import struct
import time

import source_stage2_readonly_sync_probe_20261105 as transport
from pyocd.core.helpers import ConnectHelper
from pyocd.core.session import Session
from pyocd.core.target import Target

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'evidence/integration/2026-11-06/lift-new-calibration/first-step'
IMAGE = ROOT / 'tmp/lift_ruler_20261105/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf'
NAMES = [
    'g_lift_ruler_request', 'g_lift_ruler_rejected', 'g_lift_ruler_state',
    'g_lift_ruler_phase', 'g_lift_ruler_cause', 'g_lift_ruler_samples',
    'g_lift_ruler_options', 'g_lift_ruler_encoder_before', 'g_lift_ruler_encoder_after',
    'g_lift_ruler_stop_pending', 'g_lift_ruler_stop_unconfirmed', 'g_lift_ruler_vm_request',
    'g_lift_ruler_before_raw', 'g_lift_ruler_after_raw', 'g_lift_ruler_target_raw',
    'g_vm_test_request', 'g_safety_ready', 'g_safety_fault', 'g_rtos_fault',
    'g_can_guard_ready', 'g_can_ctrl_fault_latched', 'g_chassis_auto_active',
]


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    assert not (OUT / 'samples.jsonl').exists(), 'Preserve earlier trace; no repeated session'
    transport.live.IMAGE = IMAGE
    symbols, _ = transport.live.load_image()
    assert all(n in symbols for n in NAMES)
    options = {'target_override': 'cortex_m', 'connect_mode': 'attach', 'frequency': 1000000,
               'auto_unlock': False, 'resume_on_disconnect': False, 'dap_protocol': 'swd',
               'no_config': True, 'cmsis_dap.prefer_v1': False, 'cmsis_dap.limit_packets': True}
    Session(None, options=options)
    probes = ConnectHelper.get_all_connected_probes(blocking=False)
    assert len(probes) == 1 and probes[0].unique_id == '6afaae24'
    transport.synchronous_interface(probes[0]._link._interface)
    session = Session(probes[0], options=options)
    report = {'actuator_writes': 0, 'reset_requested': False, 'halt_requested': False,
              'motion_started_by_watcher': False, 'firmware_downloaded_by_watcher': False}
    try:
        session.open()
        target = session.target
        started = time.monotonic()
        previous = None
        terminal_at = None
        with (OUT / 'samples.jsonl').open('w', encoding='utf-8') as log:
            while time.monotonic() - started < 180:
                if target.get_state() != Target.State.RUNNING:
                    raise RuntimeError('Core stopped; do not resume or reset')
                snap = {'elapsed_s': round(time.monotonic() - started, 3)}
                for name in NAMES:
                    value = target.read32(symbols[name])
                    snap[name] = struct.unpack('<i', struct.pack('<I', value))[0] if name.endswith('_raw') else value
                log.write(json.dumps(snap) + '\n')
                log.flush()
                report['last'] = snap
                state = snap['g_lift_ruler_state']
                signature = (state, snap['g_lift_ruler_phase'], snap['g_lift_ruler_cause'],
                             snap['g_lift_ruler_stop_pending'], snap['g_lift_ruler_stop_unconfirmed'])
                if previous is None:
                    assert state == 0 and not snap['g_lift_ruler_request'], 'Already active; do not instruct another press'
                    print('WATCH_READY: readonly, idle, waiting for one physical PC13 press', flush=True)
                elif signature != previous:
                    print(json.dumps(snap), flush=True)
                previous = signature
                if state >= 4 and not snap['g_lift_ruler_stop_pending']:
                    if terminal_at is None:
                        terminal_at = time.monotonic()
                    if time.monotonic() - terminal_at >= 1:
                        report['terminal_state_observed'] = True
                        break
                else:
                    terminal_at = None
                time.sleep(.1)
        report['watch_elapsed_s'] = round(time.monotonic() - started, 3)
    except Exception as exc:
        report['error'] = repr(exc)
    finally:
        session.close()
        (OUT / 'summary.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
        print(json.dumps(report), flush=True)
    return 0 if 'error' not in report else 1


if __name__ == '__main__':
    raise SystemExit(main())
