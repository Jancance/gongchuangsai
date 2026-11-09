"""高度复核第2步：先回顶部(operation3)，再把下降目标改成固件允许的最大 110800p，下降一次后保持。

只走升降：不开合夹爪、不转臂、不动伸缩。
写 g_lift_compare_target_pulses 属于固件自己声明的配置量，SetDownTarget 会按 100000..110800 复核。
失败（已请求动作未到位）-> 请求 STOP 后断 VM。成功 -> 保持深度、VM 保持。
"""
from pathlib import Path
import sys, json, time, struct, hashlib

import source_stage2_readonly_sync_probe_20261105 as transport
from pyocd.core.helpers import ConnectHelper
from pyocd.core.session import Session
from pyocd.core.target import Target

R = Path(__file__).resolve().parents[1]
O = R / 'evidence/integration/2026-11-07/height-probe'
IMAGE = R / 'tmp/integration11_auto_release_20261107/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf'
EXPECTED = '0b2ddaec1216fe0572158e805240761fbc171426530b64cf132bb482a8065d1e'
PROBE = '6afaae24'

RAW_PER_PULSE = 11.25
PULSES_PER_MM = 1600.0
NEW_TARGET = 110800
OLD_TARGET = 108400
TOP_11_6_MM = 119.0

NAMES = ['g_lift_compare_backoff_valid', 'g_lift_compare_safe_raw', 'g_it_state', 'g_it_cause',
         'g_it_request', 'g_it_lift_permit',
         'g_lift_ruler_state', 'g_lift_ruler_request', 'g_lift_ruler_rejected', 'g_lift_ruler_phase',
         'g_lift_ruler_cause', 'g_lift_ruler_samples', 'g_lift_ruler_stop_pending',
         'g_lift_ruler_stop_unconfirmed', 'g_lift_ruler_before_raw', 'g_lift_ruler_after_raw',
         'g_lift_ruler_target_raw', 'g_lift_compare_operation', 'g_lift_compare_top_valid',
         'g_lift_compare_top_raw', 'g_lift_compare_home_seen', 'g_lift_compare_home_reported',
         'g_lift_compare_target_pulses', 'g_startup_axes_state', 'g_startup_axes_cause',
         'g_extension_home_reference_valid', 'g_extension_hold_confirmed', 'g_vehicle_arm_state',
         'g_vehicle_arm_stop_pending', 'g_source_parallel_stop_mask', 'g_dm_can2_read_cycle',
         'g_dm_can2_state', 'g_dm_can2_result', 'g_dm_can2_ctrl_mode', 'g_dm_can2_feedback_valid',
         'g_dm_can2_valid_mask', 'g_dm_can2_range_mask', 'g_dm_can2_motor_state',
         'g_dm_jog_state', 'g_dm_jog_request', 'g_dm_jog_first_fault', 'g_dm_jog_result',
         'g_dm_posvel_holding', 'g_vm_test_request', 'g_can_guard_ready', 'g_safety_ready',
         'g_safety_fault', 'g_rtos_fault', 'g_can_ctrl_fault_latched', 'g_chassis_auto_active']
FLOATS = ['g_dm_can2_logical_position_deg', 'g_dm_can2_velocity_rad_s',
          'g_extension_home_reference_deg', 'g_startup_extension_current_deg']
FAULT_KEYS = ['g_lift_ruler_cause', 'g_it_cause', 'g_safety_fault', 'g_rtos_fault',
              'g_can_ctrl_fault_latched', 'g_startup_axes_cause']


def main():
    tag = sys.argv[1] if len(sys.argv) > 1 else 'height110800-02'
    assert tag and all(c.isalnum() or c in '-_' for c in tag)
    assert hashlib.sha256(IMAGE.read_bytes()).hexdigest() == EXPECTED
    transport.live.IMAGE = IMAGE
    n, segs = transport.live.load_image()

    opt = {'target_override': 'cortex_m', 'connect_mode': 'attach', 'frequency': 1000000,
           'auto_unlock': False, 'resume_on_disconnect': False, 'no_config': True,
           'cmsis_dap.prefer_v1': False, 'cmsis_dap.limit_packets': True}
    Session(None, options=opt)
    probes = ConnectHelper.get_all_connected_probes(blocking=False)
    assert len(probes) == 1 and probes[0].unique_id == PROBE
    transport.synchronous_interface(probes[0]._link._interface)
    session = Session(probes[0], options=opt)

    report = {'image_sha256': EXPECTED, 'tag': tag, 'flash': False, 'writes': [],
              'gripper_commands': 0, 'horizontal_commands': 0, 'reset': False,
              'target_from': OLD_TARGET, 'target_to': NEW_TARGET,
              'target_from_mm': OLD_TARGET / PULSES_PER_MM, 'target_to_mm': NEW_TARGET / PULSES_PER_MM,
              'top_11_6_mm': TOP_11_6_MM,
              'reference_point_11_6': '110800p measured gripper bottom 50mm on 2026-11-06'}
    target = None
    moved = False
    done = False

    def get(name):
        return target.read32(n[name])

    def put(name, value):
        target.write32(n[name], value)
        report['writes'].append([name, value])

    def snap():
        assert target.get_state() == Target.State.RUNNING
        d = {k: get(k) for k in NAMES}
        for k in FLOATS:
            d[k] = struct.unpack('<f', struct.pack('<I', get(k)))[0]
        d['claw_pwm_us'] = target.read32(0x40000034)
        d['VM_PE2_high'] = bool(target.read32(0x40021014) & 4)
        return d

    def raise_on_fault(d):
        for k in FAULT_KEYS:
            if d[k]:
                raise RuntimeError('latched fault ' + k + '=' + str(d[k]))

    def run_op(request, expected_op, timeout, phase, log):
        nonlocal moved
        old = get('g_lift_ruler_samples')
        put('g_lift_ruler_request', request)
        moved = True
        begin = time.monotonic()
        while time.monotonic() - begin < timeout:
            d = snap()
            d['host_phase'] = phase
            d['elapsed_s'] = round(time.monotonic() - begin, 3)
            log.write(json.dumps(d) + '\n')
            log.flush()
            report['last'] = d
            raise_on_fault(d)
            if d['g_lift_ruler_state'] == 5 or d['g_lift_ruler_rejected']:
                raise RuntimeError(phase + ' fault/rejected: rejected=' + str(d['g_lift_ruler_rejected']))
            if (d['g_lift_ruler_state'] == 4 and d['g_lift_compare_operation'] == expected_op
                    and d['g_lift_ruler_samples'] != old):
                report[phase] = d
                print(phase + ' DONE', flush=True)
                return d
            time.sleep(.05)
        raise RuntimeError(phase + ' timeout; not retried')

    O.mkdir(parents=True, exist_ok=True)
    with (O / (tag + '-samples.jsonl')).open('x', encoding='utf-8') as log:
        try:
            session.open()
            target = session.target
            assert target.get_state() == Target.State.RUNNING
            for a, b in segs:
                for k in range(0, len(b), 4096):
                    assert bytes(target.read_memory_block8(a + k, len(b[k:k + 4096]))) == b[k:k + 4096]
            report['flash'] = True

            report['before'] = d = snap()
            raise_on_fault(d)
            assert d['g_vm_test_request'] == 1 and d['VM_PE2_high'] and d['g_safety_ready'] == 1
            assert d['g_lift_ruler_state'] == 4 and d['g_lift_compare_top_valid'] == 1
            assert d['g_lift_compare_backoff_valid'] == 1
            assert d['g_it_lift_permit'] == 0 and d['g_lift_ruler_request'] == 0
            assert d['g_lift_ruler_stop_pending'] == 0 and d['g_lift_compare_operation'] == 2
            assert d['claw_pwm_us'] == 2056

            put('g_it_lift_permit', 1)
            report['return_to_top'] = run_op(3, 3, 70, 'return_to_top', log)
            assert get('g_lift_compare_top_valid') == 1 and get('g_lift_compare_backoff_valid') == 1

            put('g_lift_compare_target_pulses', NEW_TARGET)
            d = snap()
            assert d['g_lift_compare_target_pulses'] == NEW_TARGET
            print('WARNING: descending to ' + str(NEW_TARGET) + ' now', flush=True)
            report['descent'] = run_op(1, 2, 35, 'down_110800', log)
            put('g_it_lift_permit', 0)
            done = True

            d = snap()
            after = d['g_lift_ruler_after_raw']
            report['measurement'] = {
                'before_raw': d['g_lift_ruler_before_raw'], 'after_raw': after,
                'target_raw': d['g_lift_ruler_target_raw'],
                'travel_pulses': round((after - d['g_lift_ruler_before_raw']) / RAW_PER_PULSE, 1),
                'travel_mm': round((after - d['g_lift_ruler_before_raw']) / RAW_PER_PULSE / PULSES_PER_MM, 2),
                'absolute_from_top_pulses': round(after / RAW_PER_PULSE, 1),
                'absolute_from_top_mm': round(after / RAW_PER_PULSE / PULSES_PER_MM, 2),
                'expected_bottom_if_11_6_geometry': round(TOP_11_6_MM - after / RAW_PER_PULSE / PULSES_PER_MM, 2),
            }
            report['holding_at_depth'] = True
            print('HOLDING_AT_DEPTH ' + json.dumps(report['measurement']), flush=True)
        except Exception as e:
            report['error'] = repr(e)
        finally:
            if target is not None:
                try:
                    if not done:
                        try:
                            if moved:
                                put('g_lift_ruler_request', 2)
                        except Exception:
                            pass
                        target.write32(n['g_lift_ruler_controller'] + 4, 0)
                        target.write32(n['g_startup_axes_vm_request'], 0)
                        target.write32(n['g_it_lift_permit'], 0)
                        for _ in range(30):
                            time.sleep(.1)
                            if get('g_vm_test_request') == 0 and not (target.read32(0x40021014) & 4):
                                break
                        report['failure_VM_off'] = (get('g_vm_test_request') == 0
                                                    and not (target.read32(0x40021014) & 4))
                    report['final'] = snap()
                except Exception as e:
                    report['final_read_error'] = repr(e)
            session.close()
            (O / (tag + '-result.json')).write_text(json.dumps(report, indent=2), encoding='utf-8')
            print(json.dumps({k: v for k, v in report.items()
                              if k not in ['before', 'return_to_top', 'descent', 'last', 'final']}), flush=True)
    return int('error' in report)


if __name__ == '__main__':
    raise SystemExit(main())
