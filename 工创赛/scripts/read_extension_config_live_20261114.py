"""One explicit read-only diagnostic transaction; no enable/move/clear/reference writes."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import time
import gripper210_attach_read_20261105 as live
from source_body_probe_20261111 import attach

R = Path(__file__).resolve().parents[1]
E = R / 'evidence/integration/2026-11-14/low-readonly-diag'


def run():
    candidate = json.loads((E / 'candidate.json').read_text(encoding='utf-8'))
    assert candidate['build_passed'] and (candidate.get('motion_and_fault_logic_unchanged') or candidate.get('read_only_diag_supported'))
    image = candidate['images']['Vehicle_LiftRuler']
    live.IMAGE = R / image['axf']
    assert hashlib.sha256(live.IMAGE.read_bytes()).hexdigest() == image['sha256']
    symbols, segments = live.load_image()
    result = dict(complete=False, flashed=False, reset=False, driver_motion_or_clear_commands=False,
                  image_sha256=image['sha256'], power_request_written=False)
    path = E / ('read-' + datetime.now().strftime('%Y%m%d-%H%M%S') + '.json')
    try:
        with attach() as session:
            target = session.target
            assert target.get_state().name == 'RUNNING'
            count = 0
            for address, data in segments:
                for offset in range(0, len(data), 4096):
                    chunk = data[offset:offset+4096]
                    assert bytes(target.read_memory_block8(address+offset, len(chunk))) == chunk
                    count += len(chunk)
            result['flash_bytes_verified'] = count
            faults = ['g_safety_fault', 'g_rtos_fault', 'g_can_ctrl_fault_latched', 'g_chassis_auto_active',
                      'g_low_request', 'g_it_request', 'g_source_parallel_stop_mask', 'g_vehicle_arm_stop_pending',
                      'g_lift_ruler_stop_pending']
            assert not any(target.read32(symbols[k]) for k in faults)
            assert target.read32(symbols['g_low_state']) in (0, 2, 9)
            assert target.read32(symbols['g_it_state']) != 1
            result['vm_before'] = target.read32(symbols['g_vm_test_request'])
            if result['vm_before'] == 0:
                # Only actuator-bus supply; no motor enable or axis command.
                target.write32(symbols['g_vm_test_request'], 1)
                result['power_request_written'] = True
                time.sleep(.8)
            assert (target.read32(0x40021010) >> 2) & 1
            assert not any(target.read32(symbols[k]) for k in faults)
            cycle = target.read32(symbols['g_recovery_diag_cycle'])
            target.write32(symbols['g_recovery_diag_request'], 1)
            begin = time.monotonic()
            while time.monotonic() - begin < 4:
                if target.read32(symbols['g_recovery_diag_cycle']) != cycle:
                    break
                time.sleep(.1)
            result['cycle'] = target.read32(symbols['g_recovery_diag_cycle'])
            result['done'] = target.read32(symbols['g_recovery_diag_done'])
            result['results'] = list(target.read_memory_block32(symbols['g_recovery_diag_result'], 5))
            raw = bytes(target.read_memory_block8(symbols['g_recovery_diag_raw'], 32))
            result['raw_queries'] = [list(raw[i:i+8]) for i in range(0, 32, 8)]
            data = bytes(target.read_memory_block8(symbols['g_recovery_diag_config'], 36))
            result['configuration_hex'] = data.hex()
            assert result['cycle'] != cycle and result['done'] == 1 and result['results'] == [1]*5
            assert data[:3] == bytes([0x42, 0x25, 0x18]) and data[-1] == 0x6b
            be = lambda i: int.from_bytes(data[i:i+2], 'big')
            result['configuration'] = dict(clog_pro=data[26], clog_rpm=be(27), clog_ma=be(29), clog_ms=be(31),
                max_closed_loop_ma=be(16), position_window_raw=be(33), scale_position=data[25],
                control_mode=data[4], motor_enable_mode=data[7])
            q = result['raw_queries']
            scale = 100 if q[0][2] & 0x80 else 10
            result['actual'] = dict(options=q[0][2], flags=q[1][1],
                speed_raw=q[2][2]*256+q[2][3], speed_sign=q[2][1],
                position_raw=int.from_bytes(bytes(q[3][2:6]), 'big') * (-1 if q[3][1] else 1),
                position_scale=scale)
            result['complete'] = True
    except BaseException as error:
        result['error'] = repr(error)
        raise
    finally:
        path.write_text(json.dumps(result, indent=2), encoding='utf-8')
        print(json.dumps(result, indent=2), flush=True)
        print(path, flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--once', action='store_true')
    args = parser.parse_args()
    if args.once:
        run()
    else:
        print('CHECK only. --once requests actuator supply if needed and read-only drive queries; no axis command.')
