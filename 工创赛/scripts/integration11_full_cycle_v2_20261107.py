"""单件全流程 v2：无参考/故障时复位+回零(最多3次，空爪安全) -> 伸缩回零 -> 下降118800 -> 闭爪140
-> 带料回顶 -> 转1号槽35.758 -> 松爪210 -> 空爪回180。全部走固件原生任务入口。
持料阶段任何失败只 STOP/保持，绝不复位(复位会把夹爪初始化成210度)。"""
from pathlib import Path
import sys, json, time, struct, hashlib

import source_stage2_readonly_sync_probe_20261105 as transport
from pyocd.core.helpers import ConnectHelper
from pyocd.core.session import Session
from pyocd.core.target import Target

R = Path(__file__).resolve().parents[1]
O = R / 'evidence/integration/2026-11-07/full-cycle'
IMAGE = R / 'tmp/integration11_backoff_retry_20261107/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf'
EXPECTED = 'b90779f76c5f4d5e261e51c5bbabf9551e035d5f63a5e0cd20ad5997f4b48ad5'
PROBE = '6afaae24'

RAW_PER_PULSE = 11.25
TARGET_PULSES = 118800
SLOT1_DEG = 35.758
SOURCE_DEG = 180.0

W32 = ['g_it_request', 'g_it_state', 'g_it_cause', 'g_it_rejected', 'g_it_action', 'g_it_sequence',
       'g_it_axes', 'g_it_elapsed_ms', 'g_it_lift_permit', 'g_it_place_depth_limit_pulses',
       'g_lift_compare_backoff_valid', 'g_lift_compare_safe_raw', 'g_lift_compare_top_valid',
       'g_lift_compare_top_raw', 'g_lift_compare_operation', 'g_lift_compare_target_pulses',
       'g_lift_ruler_state', 'g_lift_ruler_request', 'g_lift_ruler_rejected', 'g_lift_ruler_phase',
       'g_lift_ruler_cause', 'g_lift_ruler_samples', 'g_lift_ruler_stop_pending',
       'g_lift_ruler_stop_unconfirmed', 'g_lift_ruler_before_raw', 'g_lift_ruler_after_raw',
       'g_lift_ruler_target_raw',
       'g_startup_axes_state', 'g_startup_axes_cause', 'g_extension_home_reference_valid',
       'g_extension_hold_confirmed', 'g_extension_home_state', 'g_extension_home_result',
       'g_dm_can2_read_cycle', 'g_dm_can2_state', 'g_dm_can2_result', 'g_dm_can2_ctrl_mode',
       'g_dm_can2_feedback_valid', 'g_dm_can2_valid_mask', 'g_dm_can2_range_mask',
       'g_dm_can2_motor_state', 'g_dm_jog_state', 'g_dm_jog_request', 'g_dm_jog_result',
       'g_dm_jog_first_fault', 'g_dm_posvel_holding',
       'g_source_parallel_state', 'g_source_parallel_cause', 'g_source_parallel_stop_mask',
       'g_vehicle_arm_state', 'g_vehicle_arm_stop_pending', 'g_vehicle_lift_feedback_valid',
       'g_vm_test_request', 'g_can_guard_ready', 'g_safety_ready', 'g_safety_fault',
       'g_rtos_fault', 'g_can_ctrl_fault_latched', 'g_chassis_auto_active']
F32 = ['g_it_dm_deg', 'g_it_extension_mm', 'g_it_lift_pulses', 'g_dm_can2_logical_position_deg',
       'g_dm_can2_velocity_rad_s', 'g_extension_home_reference_deg',
       'g_startup_extension_current_deg', 'g_vehicle_lift_current_deg']
FAULT_KEYS = ['g_lift_ruler_cause', 'g_it_cause', 'g_safety_fault', 'g_rtos_fault',
              'g_can_ctrl_fault_latched', 'g_startup_axes_cause']


def main():
    tag = sys.argv[1] if len(sys.argv) > 1 else 'cycle-v2-01'
    assert tag and all(c.isalnum() or c in '-_' for c in tag)
    assert hashlib.sha256(IMAGE.read_bytes()).hexdigest() == EXPECTED
    transport.live.IMAGE = IMAGE
    n, segs = transport.live.load_image()
    missing = [k for k in W32 + F32 if k not in n]
    missing += [k for k in ['g_lift_ruler_controller', 'g_startup_axes_vm_request'] if k not in n]
    assert not missing, missing

    opt = {'target_override': 'cortex_m', 'connect_mode': 'attach', 'frequency': 1000000,
           'auto_unlock': False, 'resume_on_disconnect': False, 'no_config': True,
           'cmsis_dap.prefer_v1': False, 'cmsis_dap.limit_packets': True}
    Session(None, options=opt)
    probes = ConnectHelper.get_all_connected_probes(blocking=False)
    assert len(probes) == 1 and probes[0].unique_id == PROBE
    transport.synchronous_interface(probes[0]._link._interface)
    session = Session(probes[0], options=opt)

    report = {'image_sha256': EXPECTED, 'tag': tag, 'flash': False, 'steps': [], 'writes': [],
              'reset_count': 0, 'material_held': False, 'gripper_commands': [],
              'slot1_deg': SLOT1_DEG, 'source_deg': SOURCE_DEG, 'target_pulses': TARGET_PULSES,
              'empty_attempts': []}
    target = None
    holding = False

    def get(k):
        return target.read32(n[k])

    def getf(k):
        return struct.unpack('<f', struct.pack('<I', target.read32(n[k])))[0]

    def put(k, v):
        target.write32(n[k], v)
        report['writes'].append([k, v])

    def putf(k, v):
        raw = struct.unpack('<I', struct.pack('<f', v))[0]
        target.write32(n[k], raw)
        report['writes'].append([k, v])

    def snap():
        assert target.get_state() == Target.State.RUNNING
        d = {k: get(k) for k in W32}
        for k in F32:
            d[k] = getf(k)
        d['claw_pwm_us'] = target.read32(0x40000034)
        d['carrier_pwm_us'] = target.read32(0x40000440)
        d['VM_PE2_high'] = bool(target.read32(0x40021014) & 4)
        return d

    def faults(d):
        return {k: d[k] for k in FAULT_KEYS if d[k]}

    def at_top(d):
        return (d['g_lift_compare_top_valid'] == 1 and d['g_lift_compare_backoff_valid'] == 1
                and d['g_vehicle_lift_feedback_valid'] == 1
                and abs(d['g_vehicle_lift_current_deg'] - d['g_lift_compare_safe_raw'] / 100.0) <= 1.0)

    def record(name, d):
        report['steps'].append({'step': name, 'snapshot': d})
        print('STEP ' + name + ' OK', flush=True)

    def wait_it(action, timeout, log, phase):
        begin = time.monotonic()
        while time.monotonic() - begin < timeout:
            d = snap()
            d['host_phase'] = phase
            d['elapsed_s'] = round(time.monotonic() - begin, 3)
            log.write(json.dumps(d) + '\n')
            log.flush()
            report['last'] = d
            if d['g_lift_ruler_rejected']:
                raise RuntimeError(phase + ': lift rejected=' + str(d['g_lift_ruler_rejected']))
            f = faults(d)
            if f:
                raise RuntimeError(phase + ': fault ' + json.dumps(f))
            if d['g_it_request'] == 0 and d['g_it_action'] == action:
                if d['g_it_rejected']:
                    raise RuntimeError(phase + ': it_rejected=' + str(d['g_it_rejected'])
                                       + ' state=' + str(d['g_it_state']))
                if d['g_it_state'] == 2:
                    return d
                if d['g_it_state'] == 9:
                    raise RuntimeError(phase + ': it_fail cause=' + str(d['g_it_cause']))
            time.sleep(.05)
        raise RuntimeError(phase + ' timeout ' + str(timeout) + 's')

    def wait_lift_op(expected_op, timeout, log, phase):
        old = get('g_lift_ruler_samples')
        begin = time.monotonic()
        while time.monotonic() - begin < timeout:
            d = snap()
            d['host_phase'] = phase
            d['elapsed_s'] = round(time.monotonic() - begin, 3)
            log.write(json.dumps(d) + '\n')
            log.flush()
            report['last'] = d
            f = faults(d)
            if f:
                raise RuntimeError(phase + ': fault ' + json.dumps(f))
            if d['g_lift_ruler_state'] == 4 and d['g_lift_compare_operation'] == expected_op \
                    and d['g_lift_ruler_samples'] != old:
                return d
            time.sleep(.05)
        raise RuntimeError(phase + ' timeout ' + str(timeout) + 's')

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
            d = snap()
            report['before'] = d
            assert d['claw_pwm_us'] == 2056, 'gripper not open at start: %s' % d['claw_pwm_us']

            # ---- STEP 1 空爪到顶：故障/无参考 -> 复位+回零，最多3次 ----
            for attempt in range(3):
                d = snap()
                if faults(d) or d['g_lift_ruler_state'] in (5, 6) or d['g_lift_ruler_state'] == 0:
                    if faults(d) or d['g_lift_ruler_state'] in (5, 6) or d['g_lift_compare_top_valid'] != 1:
                        try:
                            target.reset(reset_type=Target.ResetType.SYSRESETREQ)
                            report['reset_count'] += 1
                            time.sleep(3)
                            assert target.get_state() == Target.State.RUNNING
                            d = snap()
                            assert not faults(d), faults(d)
                        except Exception as e:
                            report['empty_attempts'].append('reset:' + repr(e))
                            continue
                try:
                    if d['g_lift_ruler_state'] == 0:
                        put('g_it_request', 1)
                        d = wait_it(1, 100, log, 'lift_home')
                    elif not at_top(d):
                        put('g_it_request', 4)
                        d = wait_it(4, 75, log, 'return_top_empty')
                    if at_top(snap()):
                        break
                except Exception as e:
                    report['empty_attempts'].append(repr(e))
                    print('empty attempt %d failed: %r' % (attempt, e), flush=True)
            d = snap()
            assert at_top(d), 'not at top after empty phase: ' + json.dumps(faults(d))
            record('empty_at_top', d)

            # ---- STEP 2 伸缩回零 ----
            if not (d['g_startup_axes_state'] == 4 and d['g_extension_home_reference_valid'] == 1
                    and d['g_extension_hold_confirmed'] == 1):
                put('g_it_request', 2)
                d = wait_it(2, 30, log, 'extension_home')
            d = snap()
            assert d['g_startup_axes_state'] == 4 and d['g_extension_home_reference_valid'] == 1
            assert d['g_extension_hold_confirmed'] == 1
            record('extension_home', d)

            # ---- STEP 3 下降 118800 ----
            if get('g_lift_compare_target_pulses') != TARGET_PULSES:
                put('g_lift_compare_target_pulses', TARGET_PULSES)
            assert get('g_lift_compare_target_pulses') == TARGET_PULSES
            put('g_it_lift_permit', 1)
            put('g_lift_ruler_request', 1)
            d = wait_lift_op(2, 35, log, 'down_to_pick')
            put('g_it_lift_permit', 0)
            raw = d['g_lift_ruler_after_raw']
            assert abs(raw - TARGET_PULSES * RAW_PER_PULSE) <= 3, raw
            report['down_raw'] = raw
            record('down_to_pick', d)

            # ---- STEP 4 闭爪 140 ----
            put('g_it_request', 5)
            d = wait_it(5, 10, log, 'close_gripper')
            assert target.read32(0x40000034) == 1537
            holding = True
            report['material_held'] = True
            report['gripper_commands'].append('close140')
            record('close_gripper', d)

            # ---- STEP 5 带料回顶 ----
            put('g_it_request', 4)
            d = wait_it(4, 90, log, 'return_top_loaded')
            assert at_top(d), 'loaded return did not reach verified top'
            report['gripper_commands'].append('return_top_loaded_ok')
            record('return_top_loaded', d)

            # ---- STEP 6 转1号槽 ----
            put('g_it_axes', 1)
            putf('g_it_dm_deg', SLOT1_DEG)
            putf('g_it_extension_mm', 0.0)
            putf('g_it_lift_pulses', 0.0)
            put('g_it_request', 3)
            d = wait_it(3, 30, log, 'rotate_slot1')
            assert abs(d['g_dm_can2_logical_position_deg'] - SLOT1_DEG) <= 3.0, d['g_dm_can2_logical_position_deg']
            report['slot1_actual_deg'] = d['g_dm_can2_logical_position_deg']
            record('rotate_slot1', d)

            # ---- STEP 7 松爪 ----
            put('g_it_request', 6)
            d = wait_it(6, 10, log, 'release')
            assert target.read32(0x40000034) == 2056
            holding = False
            report['gripper_commands'].append('release210')
            record('release', d)

            # ---- STEP 8 空爪回180 ----
            put('g_it_axes', 1)
            putf('g_it_dm_deg', SOURCE_DEG)
            putf('g_it_extension_mm', 0.0)
            putf('g_it_lift_pulses', 0.0)
            put('g_it_request', 3)
            d = wait_it(3, 30, log, 'rotate_source180')
            assert abs(d['g_dm_can2_logical_position_deg'] - SOURCE_DEG) <= 3.0, d['g_dm_can2_logical_position_deg']
            report['source_actual_deg'] = d['g_dm_can2_logical_position_deg']
            record('rotate_source180', d)

            report['cycle_complete'] = True
            print('CYCLE_COMPLETE', flush=True)
        except Exception as e:
            report['error'] = repr(e)
            print('CYCLE_ERROR ' + repr(e), flush=True)
        finally:
            if target is not None:
                try:
                    if holding:
                        try:
                            put('g_lift_ruler_request', 2)
                        except Exception:
                            pass
                        report['held_material_stop_requested'] = True
                    report['final'] = d = snap()
                    report['final_at_top'] = at_top(d)
                except Exception as e:
                    report['final_read_error'] = repr(e)
            session.close()
            (O / (tag + '-result.json')).write_text(json.dumps(report, indent=2), encoding='utf-8')
            print(json.dumps({k: v for k, v in report.items()
                              if k not in ['before', 'steps', 'last', 'final']}), flush=True)
    return int(not report.get('cycle_complete'))


if __name__ == '__main__':
    raise SystemExit(main())
