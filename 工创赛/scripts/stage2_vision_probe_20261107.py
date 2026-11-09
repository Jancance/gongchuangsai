"""Frozen Stage2 USART6 observation tool. No reset, flash, motor or calibration writes."""
from pathlib import Path
import argparse
import hashlib
import json
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
RELEASE = ROOT / 'evidence/integration/2026-11-07/stage2-mainline/candidate.json'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='Local files only; no USB access')
    parser.add_argument('--mode', choices=['A', 'M', 'idle', 'continuous', 'once'])
    parser.add_argument('--duration', type=float, default=5)
    parser.add_argument('--tag', default='wire-observe-01')
    args = parser.parse_args()
    assert 0 <= args.duration <= 30
    assert args.tag and all(c.isalnum() or c in '-_' for c in args.tag)
    manifest = json.loads(RELEASE.read_text(encoding='utf-8'))
    image = ROOT / manifest['online_image']
    assert hashlib.sha256(image.read_bytes()).hexdigest() == manifest['online_sha256'], 'candidate image changed'
    for name, digest in manifest['source_sha256'].items():
        assert hashlib.sha256((ROOT / manifest['project'] / name).read_bytes()).hexdigest() == digest, name
    if args.check:
        print(json.dumps({'passed': True, 'port': 'USART6 PC6/PC7', 'baud': 115200,
                          'image_sha256': manifest['online_sha256'], 'USB_opened': False,
                          'motor_commands': 0, 'flash_or_reset': False}))
        return 0

    # Project-bundled debug dependencies; imported only for explicit live use.
    import source_stage2_readonly_sync_probe_20261105 as transport
    from pyocd.core.helpers import ConnectHelper
    from pyocd.core.session import Session
    from pyocd.core.target import Target
    transport.live.IMAGE = image
    symbols, segments = transport.live.load_image()
    keys = ['g_vis_rx_bytes', 'g_vis_rx_bad', 'g_uart6_rx_errors', 'g_vis_mat_cycle',
            'g_vis_mat_left', 'g_vis_mat_right', 'g_vis_mat_down', 'g_vis_mat_tick',
            'g_vis_center_cycle', 'g_vis_center_x', 'g_vis_center_y', 'g_vis_center_tick',
            'g_vis_material_valid', 'g_vis_center_valid', 'g_vis_command_request',
            'g_vis_command_sent', 'g_vis_command_rejected', 'g_vis_task_valid',
            'g_vis_task_item', 'g_vis_loaded_mask', 'g_vis_batch_complete',
            'g_vis_taught_mask', 'g_vm_test_request', 'g_lift_ruler_request', 'g_it_request']
    assert all(k in symbols for k in keys)
    output = RELEASE.parent / (args.tag + '.json')
    assert not output.exists(), 'choose a new tag to preserve existing evidence'
    options = {'target_override': 'cortex_m', 'connect_mode': 'attach', 'frequency': 1000000,
               'auto_unlock': False, 'resume_on_disconnect': False, 'no_config': True,
               'cmsis_dap.prefer_v1': False, 'cmsis_dap.limit_packets': True}
    Session(None, options=options)
    probes = ConnectHelper.get_all_connected_probes(blocking=False)
    assert len(probes) == 1 and probes[0].unique_id == '6afaae24'
    transport.synchronous_interface(probes[0]._link._interface)
    session = Session(probes[0], options=options)
    result = {'image_sha256': manifest['online_sha256'], 'writes': [], 'reset': False,
              'flash': False, 'motor_commands': 0, 'calibration_written': False, 'samples': []}
    try:
        session.open()
        target = session.target
        assert target.get_state() == Target.State.RUNNING
        for address, data in segments:
            for offset in range(0, len(data), 4096):
                block = data[offset:offset + 4096]
                assert bytes(target.read_memory_block8(address + offset, len(block))) == block, 'board image mismatch'

        def read():
            snapshot = {k: target.read32(symbols[k]) for k in keys}
            snapshot['PE2_high'] = bool(target.read32(0x40021014) & 4)
            return snapshot

        result['before'] = before = read()
        if args.mode:
            # This utility is a pure communication trial, not a production motor gate.
            assert before['g_vm_test_request'] == 0 and not before['PE2_high'], 'communication trial requires VM off'
            assert before['g_it_request'] == before['g_lift_ruler_request'] == before['g_vis_command_request'] == 0
            command = {'idle': 1, 'M': 2, 'A': 5, 'continuous': 0x14, 'once': 0x15}[args.mode]
            target.write32(symbols['g_vis_command_request'], command)
            result['writes'].append(['vision UART command only', command])
        start = time.monotonic()
        while True:
            sample = read()
            sample['host_elapsed_s'] = round(time.monotonic() - start, 3)
            result['samples'].append(sample)
            if time.monotonic() - start >= args.duration:
                break
            time.sleep(.1)
        result['after'] = sample
        result['actual_UART_bytes_increase'] = sample['g_vis_rx_bytes'] - before['g_vis_rx_bytes']
        result['real_camera_or_calibration_accepted'] = False
    except Exception as error:
        result['error'] = repr(error)
    finally:
        session.close()
        output.write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps({k: v for k, v in result.items() if k != 'samples'}))
    return int('error' in result)


if __name__ == '__main__':
    raise SystemExit(main())
