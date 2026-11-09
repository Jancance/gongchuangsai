"""单件高度复核：只做一次升降下降到 108400，不开合夹爪、不回顶、不转臂、不动伸缩。

前置：板端 0b2ddaec 逐段核 Flash 一致。
恢复：仅当参考/退离失效或存在故障锁存时，执行一次真实 SYSRESETREQ + 原生回零请求(1)，
      回零后由固件自身完成 9C 退出与 3200p 退离，失败不重试、不伪造目标。
动作：g_it_lift_permit=1 后发 g_lift_ruler_request=1(operation2) 到绝对目标 108400p，
      到位即保持（VM 保持），等现场尺量，不自动回顶。
失败：已请求下降但未到位 -> 请求 STOP，随后断 VM（空爪、机械自锁已确认）。
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

# 2026-11-07 02:32 实测闭环：108400p -> target_raw 1219500，退离 3200p -> raw 36000
RAW_PER_PULSE = 11.25
PULSES_PER_MM = 1600.0
TARGET_PULSES = 108400
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
    tag = sys.argv[1] if len(sys.argv) > 1 else 'height108400-01'
    assert tag and all(c.isalnum() or c in '-_' for c in tag)
    assert hashlib.sha256(IMAGE.read_bytes()).hexdigest() == EXPECTED
    transport.live.IMAGE = IMAGE
    n, segs = transport.live.load_image()
    for key in ['g_it_lift_permit', 'g_lift_ruler_request', 'g_it_request', 'g_lift_ruler_controller',
                'g_startup_axes_vm_request', 'g_vm_test_request', 'g_safety_ready', 'g_can_test_result']:
        assert key in n, key

    opt = {'target_override': 'cortex_m', 'connect_mode': 'attach', 'frequency': 1000000,
           'auto_unlock': False, 'resume_on_disconnect': False, 'no_config': True,
           'cmsis_dap.prefer_v1': False, 'cmsis_dap.limit_packets': True}
    Session(None, options=opt)
    probes = ConnectHelper.get_all_connected_probes(blocking=False)
    assert len(probes) == 1 and probes[0].unique_id == PROBE
    transport.synchronous_interface(probes[0]._link._interface)
    session = Session(probes[0], options=opt)

    report = {'image_sha256': EXPECTED, 'tag': tag, 'reset': False, 'flash': False, 'writes': [],
              'gripper_commands': 0, 'horizontal_commands': 0, 'returned_to_top': False,
              'math': {'raw_per_pulse': RAW_PER_PULSE, 'pulses_per_mm': PULSES_PER_MM,
                       'target_pulses': TARGET_PULSES,
                       'nominal_from_top_mm': TARGET_PULSES / PULSES_PER_MM,
                       'top_11_6_mm': TOP_11_6_MM,
                       'expected_bottom_if_top_unchanged_mm': TOP_11_6_MM - TARGET_PULSES / PULSES_PER_MM}}
    target = None
    descent_requested = False
    descent_done = False

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

            report['before_recovery'] = d0 = snap()
            need = (d0['g_lift_ruler_state'] != 4 or d0['g_lift_ruler_cause'] != 0
                    or d0['g_lift_compare_top_valid'] != 1 or d0['g_lift_compare_backoff_valid'] != 1)
            report['recovery_needed'] = need
            if need:
                assert d0['g_lift_ruler_stop_pending'] == 0 and d0['g_dm_jog_request'] == 0, 'pending stop/jog'
                assert d0['g_chassis_auto_active'] == 0 and d0['g_vehicle_arm_stop_pending'] == 0
                target.reset(reset_type=Target.ResetType.SYSRESETREQ)
                report['reset'] = True
                time.sleep(3)
                assert target.get_state() == Target.State.RUNNING
                report['after_reset'] = d = snap()
                raise_on_fault(d)
                assert d['g_it_request'] == 0 and d['g_lift_ruler_request'] == 0
                assert target.read32(n['g_safety_ready']) == 1 and target.read32(n['g_can_test_result']) == 1
                put('g_it_request', 1)
                begin = time.monotonic()
                while time.monotonic() - begin < 100:
                    d = snap()
                    d['host_phase'] = 'native_home'
                    d['elapsed_s'] = round(time.monotonic() - begin, 3)
                    log.write(json.dumps(d) + '\n')
                    log.flush()
                    report['last'] = d
                    raise_on_fault(d)
                    if (d['g_it_state'] == 2 and d['g_lift_ruler_state'] == 4
                            and d['g_lift_compare_top_valid'] == 1 and d['g_lift_compare_backoff_valid'] == 1
                            and d['g_lift_compare_operation'] == 1):
                        report['native_home'] = d
                        print('native_home DONE', flush=True)
                        break
                    time.sleep(.05)
                else:
                    raise RuntimeError('native_home timeout 100s; not retried')

            d = snap()
            report['before_descent'] = d
            raise_on_fault(d)
            assert d['g_vm_test_request'] == 1 and d['VM_PE2_high'] and d['g_safety_ready'] == 1
            assert d['g_lift_ruler_state'] == 4 and d['g_lift_compare_operation'] in (1, 3)
            assert d['g_lift_compare_top_valid'] == 1 and d['g_lift_compare_backoff_valid'] == 1
            assert d['g_it_state'] in (0, 2) and d['g_it_request'] == 0
            for k in ['g_lift_ruler_request', 'g_lift_ruler_stop_pending', 'g_dm_jog_request',
                      'g_vehicle_arm_stop_pending', 'g_source_parallel_stop_mask', 'g_chassis_auto_active']:
                assert d[k] == 0, k
            assert d['g_lift_compare_target_pulses'] == TARGET_PULSES
            assert d['claw_pwm_us'] == 2056, 'gripper not at 210 open: %s' % d['claw_pwm_us']

            print('WARNING: descending to 108400 now', flush=True)
            old_samples = get('g_lift_ruler_samples')
            put('g_it_lift_permit', 1)
            put('g_lift_ruler_request', 1)
            descent_requested = True
            begin = time.monotonic()
            while time.monotonic() - begin < 30:
                d = snap()
                d['host_phase'] = 'down_108400'
                d['elapsed_s'] = round(time.monotonic() - begin, 3)
                log.write(json.dumps(d) + '\n')
                log.flush()
                report['last'] = d
                raise_on_fault(d)
                if d['g_lift_ruler_state'] == 5 or d['g_lift_ruler_rejected']:
                    raise RuntimeError('lift fault/rejected during descent')
                if (d['g_lift_ruler_state'] == 4 and d['g_lift_compare_operation'] == 2
                        and d['g_lift_ruler_samples'] != old_samples):
                    descent_done = True
                    report['descent_done'] = d
                    break
                time.sleep(.05)
            if not descent_done:
                raise RuntimeError('descent timeout 30s; not retried')
            put('g_it_lift_permit', 0)

            d = snap()
            travel_raw = d['g_lift_ruler_after_raw'] - d['g_lift_ruler_before_raw']
            travel_pulses = travel_raw / RAW_PER_PULSE
            report['measurement'] = {
                'before_raw': d['g_lift_ruler_before_raw'],
                'after_raw': d['g_lift_ruler_after_raw'],
                'target_raw': d['g_lift_ruler_target_raw'],
                'travel_raw': travel_raw,
                'travel_pulses': round(travel_pulses, 1),
                'travel_mm': round(travel_pulses / PULSES_PER_MM, 2),
                'absolute_from_top_pulses': round(d['g_lift_ruler_after_raw'] / RAW_PER_PULSE, 1),
                'absolute_from_top_mm': round(d['g_lift_ruler_after_raw'] / RAW_PER_PULSE / PULSES_PER_MM, 2),
                'expected_bottom_mm_if_top_still_119': round(
                    TOP_11_6_MM - d['g_lift_ruler_after_raw'] / RAW_PER_PULSE / PULSES_PER_MM, 2),
            }
            report['holding_at_depth'] = True
            report['note'] = ('VM kept on, lift holding at depth, gripper untouched at 2056, '
                              'no return-to-top, no rotation, no extension motion')
            print('HOLDING_AT_DEPTH ' + json.dumps(report['measurement']), flush=True)
        except Exception as e:
            report['error'] = repr(e)
            if target is not None:
                try:
                    if descent_requested:
                        put('g_lift_ruler_request', 2)
                        report['stop_requested'] = True
                        time.sleep(.2)
                except Exception as stop_error:
                    report['stop_request_error'] = repr(stop_error)
        finally:
            if target is not None:
                try:
                    if not descent_done:
                        try:
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
                              if k not in ['before_recovery', 'after_reset', 'native_home',
                                           'before_descent', 'descent_done', 'last', 'final']}), flush=True)
    return int('error' in report)


if __name__ == '__main__':
    raise SystemExit(main())
