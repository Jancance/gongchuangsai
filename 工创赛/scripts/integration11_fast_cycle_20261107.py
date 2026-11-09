"""丝滑版单件循环：闭爪后立刻上升，升到剩1/3时并行转动4310到1号槽，到顶立刻松爪并马上转回180。
升降走固件原生 op3，4310走 dm_can2 点位接口(g_dm_jog_request 4)，两者在唯一CAN任务里并行。
持料阶段绝不复位。"""
from pathlib import Path
import sys, json, time, struct, hashlib

import source_stage2_readonly_sync_probe_20261105 as transport
from pyocd.core.helpers import ConnectHelper
from pyocd.core.session import Session
from pyocd.core.target import Target

R = Path(__file__).resolve().parents[1]
O = R / 'evidence/integration/2026-11-07/fast-cycle'
IMAGE = R / 'tmp/integration11_backoff_retry_20261107/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf'
EXPECTED = 'b90779f76c5f4d5e261e51c5bbabf9551e035d5f63a5e0cd20ad5997f4b48ad5'
PROBE = '6afaae24'

RAW_PER_PULSE = 11.25
TARGET_PULSES = 118800
TOTAL_RAW = int(TARGET_PULSES * RAW_PER_PULSE)
ROTATE_TRIGGER_RAW = TOTAL_RAW // 3          # 只剩 1/3 行程时开始转臂
SLOT1_DEG = 35.758
SOURCE_DEG = 180.0
DM_SPEED = 300.0

W32 = ['g_it_request', 'g_it_state', 'g_it_cause', 'g_it_rejected', 'g_it_action',
       'g_it_axes', 'g_it_lift_permit',
       'g_lift_compare_backoff_valid', 'g_lift_compare_safe_raw', 'g_lift_compare_top_valid',
       'g_lift_compare_top_raw', 'g_lift_compare_operation', 'g_lift_compare_target_pulses',
       'g_lift_ruler_state', 'g_lift_ruler_request', 'g_lift_ruler_rejected', 'g_lift_ruler_phase',
       'g_lift_ruler_cause', 'g_lift_ruler_samples', 'g_lift_ruler_stop_pending',
       'g_lift_ruler_before_raw', 'g_lift_ruler_after_raw', 'g_lift_ruler_target_raw',
       'g_startup_axes_state', 'g_startup_axes_cause', 'g_extension_home_reference_valid',
       'g_extension_hold_confirmed',
       'g_dm_can2_state', 'g_dm_can2_result', 'g_dm_can2_ctrl_mode', 'g_dm_can2_feedback_valid',
       'g_dm_can2_valid_mask', 'g_dm_can2_range_mask', 'g_dm_can2_motor_state',
       'g_dm_jog_state', 'g_dm_jog_request', 'g_dm_jog_result', 'g_dm_jog_first_fault',
       'g_dm_posvel_holding', 'g_dm_point_enable', 'g_dm_integrated_owner',
       'g_vehicle_arm_state', 'g_vehicle_lift_feedback_valid',
       'g_vm_test_request', 'g_can_guard_ready', 'g_safety_ready', 'g_safety_fault',
       'g_rtos_fault', 'g_can_ctrl_fault_latched']
F32 = ['g_dm_can2_logical_position_deg', 'g_dm_can2_velocity_rad_s', 'g_dm_posvel_speed_deg_s',
       'g_dm_pick_target_deg', 'g_dm_place_target_deg', 'g_vehicle_lift_current_deg',
       'g_extension_home_reference_deg', 'g_startup_extension_current_deg']
FAULT_KEYS = ['g_lift_ruler_cause', 'g_it_cause', 'g_safety_fault', 'g_rtos_fault',
              'g_can_ctrl_fault_latched', 'g_startup_axes_cause']


def main():
    tag = sys.argv[1] if len(sys.argv) > 1 else 'fast-01'
    assert tag and all(c.isalnum() or c in '-_' for c in tag)
    assert hashlib.sha256(IMAGE.read_bytes()).hexdigest() == EXPECTED
    transport.live.IMAGE = IMAGE
    n, segs = transport.live.load_image()
    assert not [k for k in W32 + F32 if k not in n], [k for k in W32 + F32 if k not in n]

    opt = {'target_override': 'cortex_m', 'connect_mode': 'attach', 'frequency': 1000000,
           'auto_unlock': False, 'resume_on_disconnect': False, 'no_config': True,
           'cmsis_dap.prefer_v1': False, 'cmsis_dap.limit_packets': True}
    Session(None, options=opt)
    probes = ConnectHelper.get_all_connected_probes(blocking=False)
    assert len(probes) == 1 and probes[0].unique_id == PROBE
    transport.synchronous_interface(probes[0]._link._interface)
    session = Session(probes[0], options=opt)

    rep = {'tag': tag, 'image_sha256': EXPECTED, 'timing': {}, 'steps': [], 'writes': [],
           'rotate_trigger_raw': ROTATE_TRIGGER_RAW, 'target_pulses': TARGET_PULSES,
           'material_held': False}
    target = None
    holding = False

    def get(k):
        return target.read32(n[k])

    def getf(k):
        return struct.unpack('<f', struct.pack('<I', target.read32(n[k])))[0]

    def put(k, v):
        target.write32(n[k], v)
        rep['writes'].append([k, v])

    def putf(k, v):
        raw = struct.unpack('<I', struct.pack('<f', v))[0]
        target.write32(n[k], raw)
        rep['writes'].append([k, v])

    def pwm(v):
        target.write32(0x40000034, int(v))

    def ramp_pwm(to, ms, steps):
        start = target.read32(0x40000034)
        for i in range(1, steps + 1):
            pwm(start + (to - start) * i / steps)
            time.sleep(ms / 1000.0 / steps)
        pwm(to)

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

    def record(name, d, t0):
        rep['steps'].append({'step': name, 'ms': round((time.monotonic() - t0) * 1000)})
        print('STEP %-22s %6.0f ms' % (name, (time.monotonic() - t0) * 1000), flush=True)

    def dm_done(d, goal):
        return (d['g_dm_jog_state'] == 7 and d['g_dm_jog_result'] == 1 and d['g_dm_posvel_holding'] == 1
                and d['g_dm_can2_feedback_valid'] == 1
                and abs(d['g_dm_can2_logical_position_deg'] - goal) <= 1.0)

    def dm_move(goal):
        put('g_dm_integrated_owner', 1)
        put('g_dm_point_enable', 1)
        putf('g_dm_posvel_speed_deg_s', DM_SPEED)
        putf('g_dm_pick_target_deg', goal)
        putf('g_dm_place_target_deg', goal)
        put('g_dm_jog_request', 4)

    O.mkdir(parents=True, exist_ok=True)
    with (O / (tag + '-samples.jsonl')).open('x', encoding='utf-8') as log:
        try:
            session.open()
            target = session.target
            assert target.get_state() == Target.State.RUNNING
            for a, b in segs:
                for k in range(0, len(b), 4096):
                    assert bytes(target.read_memory_block8(a + k, len(b[k:k + 4096]))) == b[k:k + 4096]
            d = snap()
            rep['before'] = d
            assert d['claw_pwm_us'] == 2056, d['claw_pwm_us']
            assert not faults(d), faults(d)

            # ---- 0 前置：到顶 + 伸缩就绪 ----
            t0 = time.monotonic()
            if d['g_lift_ruler_state'] == 0 or faults(d) or d['g_lift_ruler_state'] in (5, 6):
                target.reset(reset_type=Target.ResetType.SYSRESETREQ)
                rep['reset'] = True
                time.sleep(3)
                d = snap()
                assert not faults(d), faults(d)
                put('g_it_request', 1)
                while True:
                    d = snap()
                    assert not faults(d), faults(d)
                    if d['g_it_state'] == 2 and at_top(d):
                        break
                    time.sleep(.05)
            elif not at_top(d):
                put('g_it_request', 4)
                while True:
                    d = snap()
                    assert not faults(d), faults(d)
                    if d['g_it_state'] == 2 and at_top(d):
                        break
                    time.sleep(.05)
            if not (d['g_startup_axes_state'] == 4 and d['g_extension_home_reference_valid'] == 1):
                put('g_it_request', 2)
                while True:
                    d = snap()
                    if d['g_it_state'] == 2:
                        break
                    assert not faults(d), faults(d)
                    time.sleep(.05)
            record('prep_at_top', d, t0)

            # ---- 1 下降 118800 ----
            t0 = time.monotonic()
            if get('g_lift_compare_target_pulses') != TARGET_PULSES:
                put('g_lift_compare_target_pulses', TARGET_PULSES)
            old = get('g_lift_ruler_samples')
            put('g_it_lift_permit', 1)
            put('g_lift_ruler_request', 1)
            while True:
                d = snap()
                log.write(json.dumps(dict(d, host_phase='down')) + '\n')
                log.flush()
                assert not faults(d), faults(d)
                if d['g_lift_ruler_state'] == 4 and d['g_lift_compare_operation'] == 2 \
                        and d['g_lift_ruler_samples'] != old:
                    break
                time.sleep(.02)
            put('g_it_lift_permit', 0)
            assert abs(d['g_lift_ruler_after_raw'] - TOTAL_RAW) <= 3, d['g_lift_ruler_after_raw']
            rep['down_raw'] = d['g_lift_ruler_after_raw']
            record('down_to_pick', d, t0)

            # ---- 2 闭爪(150ms斜坡) 后立刻上升 ----
            t0 = time.monotonic()
            ramp_pwm(1537, 150, 10)
            holding = True
            rep['material_held'] = True
            old = get('g_lift_ruler_samples')
            put('g_it_lift_permit', 1)
            put('g_lift_ruler_request', 3)
            t_close = time.monotonic()
            record('close_and_rise_start', d, t0)

            # ---- 3 升到剩1/3时并行转臂 ----
            rotated = False
            rot_fire_raw = None
            while True:
                d = snap()
                log.write(json.dumps(dict(d, host_phase='rise_overlap')) + '\n')
                log.flush()
                assert not faults(d), faults(d)
                raw = d['g_lift_ruler_after_raw']
                if (not rotated) and 0 < raw <= ROTATE_TRIGGER_RAW:
                    dm_move(SLOT1_DEG)
                    rotated = True
                    rot_fire_raw = raw
                    rep['rotate_fired_at_raw'] = raw
                    rep['rotate_fired_remaining_mm'] = round(raw / RAW_PER_PULSE / 1600.0, 2)
                    print('ROTATE_FIRED raw=%d remaining=%.2fmm' % (raw, raw / RAW_PER_PULSE / 1600.0), flush=True)
                if d['g_lift_ruler_state'] == 4 and d['g_lift_compare_operation'] == 3 \
                        and d['g_lift_ruler_samples'] != old:
                    break
                time.sleep(.02)
            rep['rise_ms'] = round((time.monotonic() - t_close) * 1000)
            rep['lift_top'] = d
            assert at_top(d), 'not at verified top: ' + json.dumps(faults(d))
            record('rise_to_top', d, t0)
            if not rotated:
                dm_move(SLOT1_DEG)
                rep['rotate_fired_at_raw'] = 'after_top'
            while not dm_done(snap(), SLOT1_DEG):
                d = snap()
                log.write(json.dumps(dict(d, host_phase='wait_slot')) + '\n')
                log.flush()
                assert not faults(d), faults(d)
                time.sleep(.02)
            d = snap()
            rep['slot1_deg_actual'] = d['g_dm_can2_logical_position_deg']
            record('rotate_slot1', d, t0)

            # ---- 4 立刻松爪，紧接着转回180 ----
            t0 = time.monotonic()
            ramp_pwm(2056, 120, 8)
            holding = False
            dm_move(SOURCE_DEG)
            record('release_and_return_start', d, t0)
            while not dm_done(snap(), SOURCE_DEG):
                d = snap()
                log.write(json.dumps(dict(d, host_phase='return180')) + '\n')
                log.flush()
                assert not faults(d), faults(d)
                time.sleep(.02)
            d = snap()
            rep['source_deg_actual'] = d['g_dm_can2_logical_position_deg']
            record('rotate_source180', d, t0)

            rep['cycle_complete'] = True
            print('FAST_CYCLE_COMPLETE', flush=True)
        except Exception as e:
            rep['error'] = repr(e)
            print('FAST_CYCLE_ERROR ' + repr(e), flush=True)
        finally:
            if target is not None:
                try:
                    if holding:
                        try:
                            put('g_lift_ruler_request', 2)
                        except Exception:
                            pass
                        rep['held_material_stop_requested'] = True
                    rep['final'] = d = snap()
                    rep['final_at_top'] = at_top(d)
                except Exception as e:
                    rep['final_read_error'] = repr(e)
            session.close()
            (O / (tag + '-result.json')).write_text(json.dumps(rep, indent=2), encoding='utf-8')
            print(json.dumps({k: v for k, v in rep.items()
                              if k not in ['before', 'steps', 'lift_top', 'final']}), flush=True)
    return int(not rep.get('cycle_complete'))


if __name__ == '__main__':
    raise SystemExit(main())
