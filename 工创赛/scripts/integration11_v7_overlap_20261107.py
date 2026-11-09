"""常使能直改目标版单件循环（stream 固件）：
点位之间不再失能/读回/预置/使能，直接按最新反馈改目标。
闭爪150ms -> 立刻上升(op3) -> 到顶立刻用固件任务action3转1号槽 -> 松爪120ms -> 立刻action3转回180。
含开头自动恢复(有故障时先缓开夹爪1.2s再复位+回零)。持料阶段失败只STOP，绝不复位。
注意：主站调试会话已改为普通pyOCD attach（先前的同步USB补丁现在会超时）。
"""
from pathlib import Path
import sys, json, time, struct, hashlib

import source_stage2_readonly_sync_probe_20261105 as transport
from pyocd.core.helpers import ConnectHelper
from pyocd.core.session import Session
from pyocd.core.target import Target

R = Path(__file__).resolve().parents[1]
O = R / 'evidence/integration/2026-11-07/fast-cycle'
IMAGE = R / 'tmp/integration11_stream_20261107/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf'
EXPECTED = 'b876af8c0a61160e69f6424c4a89c34afe1ad27aa1269673f7ab58ee16c97343'
PROBE = '6afaae24'

RAW_PER_PULSE = 11.25
TARGET_PULSES = 118800
TOTAL_RAW = int(TARGET_PULSES * RAW_PER_PULSE)
SLOT1_DEG = 35.758
ROTATE_AT_MS = 1400   # 定版：距顶约 18mm(≈2cm)。回升全程实测 1.85~2.05s(74.25mm)
#   对比真值：2.05s=已到顶；1.35s 那档出现过 1/4 概率回升被拉到 4.4s，1.4s 档 3 轮全在 5.87~5.96s，更稳。
SOURCE_DEG = 180.0

W32 = ['g_it_request', 'g_it_state', 'g_it_cause', 'g_it_rejected', 'g_it_action', 'g_it_axes',
       'g_it_lift_permit', 'g_it_elapsed_ms',
       'g_lift_compare_backoff_valid', 'g_lift_compare_safe_raw', 'g_lift_compare_top_valid',
       'g_lift_compare_operation', 'g_lift_compare_target_pulses',
       'g_lift_ruler_state', 'g_lift_ruler_request', 'g_lift_ruler_rejected', 'g_lift_ruler_phase',
       'g_lift_ruler_cause', 'g_lift_ruler_samples', 'g_lift_ruler_stop_pending',
       'g_lift_ruler_before_raw', 'g_lift_ruler_after_raw', 'g_lift_ruler_target_raw',
       'g_startup_axes_state', 'g_startup_axes_cause', 'g_extension_home_reference_valid',
       'g_extension_hold_confirmed', 'g_dm_can2_state', 'g_dm_can2_result', 'g_dm_can2_feedback_valid',
       'g_dm_can2_motor_state', 'g_dm_jog_state', 'g_dm_jog_result', 'g_dm_jog_first_fault',
       'g_dm_fast_retarget_count', 'g_dm_fast_retarget_refused', 'g_dm_jog_tx_count',
       'g_dm_jog_elapsed_ms', 'g_it_elapsed_ms',
       'g_dm_posvel_holding', 'g_dm_parallel_group_active', 'g_dm_point_enable', 'g_vehicle_arm_state', 'g_vehicle_lift_feedback_valid',
       'g_vm_test_request', 'g_safety_ready', 'g_safety_fault', 'g_rtos_fault',
       'g_can_ctrl_fault_latched', 'g_source_parallel_state', 'g_source_parallel_cause']
F32 = ['g_dm_can2_logical_position_deg', 'g_dm_can2_velocity_rad_s', 'g_vehicle_lift_current_deg',
       'g_dm_pick_target_deg', 'g_dm_place_target_deg', 'g_dm_posvel_speed_deg_s',
       'g_it_dm_deg', 'g_it_extension_mm', 'g_it_lift_pulses']
FAULT_KEYS = ['g_lift_ruler_cause', 'g_it_cause', 'g_safety_fault', 'g_rtos_fault',
              'g_can_ctrl_fault_latched', 'g_startup_axes_cause']


def main():
    tag = sys.argv[1] if len(sys.argv) > 1 else 'ovl-01'
    assert tag and all(c.isalnum() or c in '-_' for c in tag)
    assert hashlib.sha256(IMAGE.read_bytes()).hexdigest() == EXPECTED
    transport.live.IMAGE = IMAGE
    n, segs = transport.live.load_image()
    miss = [k for k in W32 + F32 if k not in n]
    assert not miss, miss

    opt = {'target_override': 'cortex_m', 'connect_mode': 'attach', 'frequency': 1000000,
           'auto_unlock': False, 'resume_on_disconnect': False, 'no_config': True,
           'cmsis_dap.prefer_v1': False, 'cmsis_dap.limit_packets': True}
    session = ConnectHelper.session_with_chosen_probe(unique_id=PROBE, options=opt)

    rep = {'tag': tag, 'image_sha256': EXPECTED, 'timing_ms': {}, 'writes': [],
           'target_pulses': TARGET_PULSES, 'material_held': False}
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
            pwm(start + (to - start) * (1.0 * i / steps))
            time.sleep(ms / 1000.0 / steps)
        pwm(to)

    def snap():
        assert target.get_state() == Target.State.RUNNING
        d = {k: get(k) for k in W32}
        for k in F32:
            d[k] = getf(k)
        d['claw_pwm_us'] = target.read32(0x40000034)
        d['VM_PE2_high'] = bool(target.read32(0x40021014) & 4)
        return d

    def faults(d):
        return {k: d[k] for k in FAULT_KEYS if d[k]}

    def at_top(d):
        return (d['g_lift_compare_top_valid'] == 1 and d['g_lift_compare_backoff_valid'] == 1
                and d['g_vehicle_lift_feedback_valid'] == 1
                and abs(d['g_vehicle_lift_current_deg'] - d['g_lift_compare_safe_raw'] / 100.0) <= 1.0)

    def mark(name, t0):
        ms = (time.monotonic() - t0) * 1000
        rep['timing_ms'][name] = round(ms)
        print('STEP %-28s %7.0f ms' % (name, ms), flush=True)

    def wait_lift(log, phase, expect_op, timeout):
        begin = time.monotonic()
        old = get('g_lift_ruler_samples')
        while time.monotonic() - begin < timeout:
            d = snap()
            d['host_phase'] = phase
            log.write(json.dumps(d) + '\n')
            log.flush()
            assert not faults(d), faults(d)
            if d['g_lift_ruler_state'] == 4 and d['g_lift_compare_operation'] == expect_op \
                    and d['g_lift_ruler_samples'] != old:
                return d
            time.sleep(.02)
        raise RuntimeError(phase + ' timeout')

    def wait_it(log, phase, action, timeout=100):
        begin = time.monotonic()
        seen = False
        while time.monotonic() - begin < timeout:
            d = snap()
            d['host_phase'] = phase
            log.write(json.dumps(d) + '\n')
            log.flush()
            assert not faults(d), faults(d)
            if d['g_it_state'] == 1 and d['g_it_action'] == action:
                seen = True
            if seen and d['g_it_state'] == 2 and d['g_it_action'] == action:
                return d
            if seen and d['g_it_state'] == 9:
                raise RuntimeError(phase + ' it_fail cause=' + str(d['g_it_cause'])
                                   + ' rejected=' + str(d['g_it_rejected']))
            time.sleep(.02)
        raise RuntimeError(phase + ' timeout')

    def rotate(log, phase, deg):
        """经 IT 任务转位（IT 空闲时使用）。"""
        d0 = snap()
        if not (d0['g_dm_jog_state'] == 7 and d0['g_dm_posvel_holding'] == 1):
            t0 = time.monotonic()
            stable = 0
            while time.monotonic() - t0 < 3.0:
                if (d0['g_dm_can2_feedback_valid'] == 1
                        and abs(d0['g_dm_can2_velocity_rad_s']) <= 0.012
                        and d0['g_dm_jog_state'] in (0, 7)):
                    stable += 1
                    if stable >= 3:
                        break
                else:
                    stable = 0
                time.sleep(.03)
                d0 = snap()
        put('g_it_axes', 1)
        putf('g_it_dm_deg', deg)
        putf('g_it_extension_mm', 0.0)
        putf('g_it_lift_pulses', 0.0)
        put('g_it_request', 3)
        d = wait_it(log, phase, 3, 30)
        assert abs(d['g_dm_can2_logical_position_deg'] - deg) <= 3.0, d['g_dm_can2_logical_position_deg']
        rep.setdefault('retarget', {})[phase] = {'count': d['g_dm_fast_retarget_count'],
            'refused': d['g_dm_fast_retarget_refused'], 'dm_tx': d['g_dm_jog_tx_count'],
            'dev_jog_ms': d['g_dm_jog_elapsed_ms'], 'dev_it_ms': d['g_it_elapsed_ms']}
        return d

    def fire_dm(log, phase, deg):
        """IT 正忙(升降在动)时，直接给 DM 点位任务下目标，与升降并行。
        请求号必须按方向选：4=角度增大，5=角度减小，与 SPF 的判定一致；
        选错会被固件方向校验直接拒绝(result=14)。"""
        cur = getf('g_dm_can2_logical_position_deg')
        req = 4 if deg >= cur else 5
        rep.setdefault('fires', []).append({'phase': phase, 'deg': deg, 'from': round(cur, 3), 'req': req,
            'rise_ms_at_fire': round((time.monotonic() - RISE_T0[0]) * 1000) if RISE_T0[0] else None,
            'lift_state_at_fire': get('g_lift_ruler_state')})
        putf('g_dm_pick_target_deg', deg)
        putf('g_dm_place_target_deg', deg)
        putf('g_dm_posvel_speed_deg_s', 300.0)
        put('g_dm_point_enable', 1)
        put('g_dm_parallel_group_active', 1)
        put('g_dm_parallel_group_go', 1)
        put('g_dm_jog_request', req)

    def wait_dm(log, phase, deg, timeout=12):
        begin = time.monotonic()
        while time.monotonic() - begin < timeout:
            d = snap()
            d['host_phase'] = phase
            log.write(json.dumps(d) + '\n')
            log.flush()
            assert not faults(d), faults(d)
            if (d['g_dm_jog_state'] == 7 and d['g_dm_posvel_holding'] == 1
                    and d['g_dm_jog_result'] == 1
                    and abs(d['g_dm_can2_logical_position_deg'] - deg) <= 3.0):
                rep.setdefault('retarget', {})[phase] = {'count': d['g_dm_fast_retarget_count'],
                    'refused': d['g_dm_fast_retarget_refused'], 'dm_tx': d['g_dm_jog_tx_count'],
                    'dev_jog_ms': d['g_dm_jog_elapsed_ms'], 'dev_it_ms': d['g_it_elapsed_ms']}
                return d
            if d['g_dm_jog_state'] == 8:
                raise RuntimeError('%s dm_jog failed result=%s first_fault=%s'
                                   % (phase, d['g_dm_jog_result'], d['g_dm_jog_first_fault']))
            time.sleep(.02)
        raise RuntimeError(phase + ' timeout')

    def wait_rise_time(log, ms, timeout=20):
        """回升途中 g_vehicle_lift_current_deg 不刷新，无法按行程比例判断，改用时间基准。
        注意：上一次下降结束后 state 仍是 LRC_DONE(4)，必须先看到它离开 4 才算回升真正开始，
        否则会立刻放行 -> 在最低点就转臂。"""
        begin = time.monotonic()
        started = False
        while time.monotonic() - begin < timeout:
            d = snap()
            d['host_phase'] = 'rise_wait'
            d['rise_started'] = started
            log.write(json.dumps(d) + '\n')
            log.flush()
            assert not faults(d), faults(d)
            st = d['g_lift_ruler_state']
            if not started:
                if st != 4:
                    started = True
                    begin = time.monotonic()
                elif (time.monotonic() - begin) > 1.0:
                    raise RuntimeError('rise never started')
                time.sleep(.02)
                continue
            if st == 4:
                print('NOTE lift reached top before rotate start', flush=True)
                return d
            if (time.monotonic() - begin) * 1000.0 >= ms:
                return d
            time.sleep(.02)
        raise RuntimeError('rise_wait timeout')

    def wait_it_done(log, phase, action, timeout=90):
        begin = time.monotonic()
        while time.monotonic() - begin < timeout:
            d = snap()
            d['host_phase'] = phase
            log.write(json.dumps(d) + '\n')
            log.flush()
            assert not faults(d), faults(d)
            if d['g_it_state'] == 2 and d['g_it_action'] == action:
                return d
            if d['g_it_state'] == 9 and d['g_it_action'] == action:
                raise RuntimeError(phase + ' it_fail cause=' + str(d['g_it_cause']))
            time.sleep(.02)
        raise RuntimeError(phase + ' timeout')

    RISE_T0 = [None]
    PWM_OPEN = 2056     # 210度，识别姿态
    PWM_PREGRIP = 1685  # 160度，下降途中预收
    PWM_GRIP = 1537     # 140度，夹紧

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

            if (faults(d) or d['g_lift_ruler_state'] in (5, 6)
                    or d['g_lift_ruler_state'] == 0 or d['g_it_state'] == 9
                    or d['g_it_cause'] != 0):
                t0 = time.monotonic()
                if target.read32(0x40000034) != PWM_OPEN:
                    ramp_pwm(PWM_OPEN, 1200, 24)
                    rep['gentle_release'] = True
                target.reset(reset_type=Target.ResetType.SYSRESETREQ)
                rep['reset'] = True
                time.sleep(3)
                d = snap()
                assert not faults(d), faults(d)
                put('g_it_request', 1)
                d = wait_it(log, 'lift_home', 1, 100)
                assert at_top(snap()), 'home did not reach top'
                mark('recover_gentle_release_reset_home', t0)

            d = snap()
            if not at_top(d):
                t0 = time.monotonic()
                put('g_it_request', 4)
                wait_it(log, 'return_top', 4, 90)
                mark('return_top', t0)
            d = snap()
            assert at_top(d), 'not at top'
            t0 = time.monotonic()
            begin = time.monotonic()
            while time.monotonic() - begin < 30:
                d = snap()
                assert not faults(d), faults(d)
                if (d['g_startup_axes_state'] == 4 and d['g_extension_home_reference_valid'] == 1
                        and d['g_extension_hold_confirmed'] == 1):
                    break
                if d['g_startup_axes_state'] == 0 and d['g_it_state'] != 1:
                    put('g_it_request', 2)
                time.sleep(.05)
            else:
                raise RuntimeError('extension not ready state=%s' % d['g_startup_axes_state'])
            mark('extension_home', t0)
            d = snap()
            if abs(d['g_dm_can2_logical_position_deg'] - SOURCE_DEG) > 3.0:
                t0 = time.monotonic()
                rotate(log, 'ensure_source180', SOURCE_DEG)
                mark('ensure_source180', t0)

            # 识别姿态：210度
            if target.read32(0x40000034) != PWM_OPEN:
                ramp_pwm(PWM_OPEN, 300, 10)

            # ---- 1 下降，途中把 210 度预收到 170 度 ----
            t0 = time.monotonic()
            if get('g_lift_compare_target_pulses') != TARGET_PULSES:
                put('g_lift_compare_target_pulses', TARGET_PULSES)
            old_samples = get('g_lift_ruler_samples')
            started = False
            begin = time.monotonic()
            while time.monotonic() - begin < 10 and not started:
                put('g_it_lift_permit', 1)
                put('g_lift_ruler_request', 1)
                t1 = time.monotonic()
                while time.monotonic() - t1 < 1.0:
                    dd = snap()
                    assert not faults(dd), faults(dd)
                    if dd['g_lift_ruler_state'] != 4 or dd['g_lift_ruler_samples'] != old_samples:
                        started = True
                        break
                    time.sleep(.03)
            assert started, 'descent request kept being rejected'
            # 下降 2.0s，这段时间把夹爪从 210 度收到 170 度，不额外占用节拍
            ramp_pwm(PWM_PREGRIP, 420, 14)
            d = wait_lift(log, 'down', 2, 35)
            put('g_it_lift_permit', 0)
            assert abs(d['g_lift_ruler_after_raw'] - TOTAL_RAW) <= 40, d['g_lift_ruler_after_raw']
            rep['down_raw'] = d['g_lift_ruler_after_raw']
            mark('down_118800_with_pregrip', t0)

            # ---- 2 只剩 30 度行程要夹紧 ----
            t0 = time.monotonic()
            ramp_pwm(PWM_GRIP, 80, 8)
            holding = True
            rep['material_held'] = True
            time.sleep(0.30)   # 30度行程的夹紧稳定时间（原70度用450ms）
            put('g_it_request', 4)
            mark('grip_from_170_then_rise_req', t0)
            rise_t0 = time.monotonic()
            RISE_T0[0] = rise_t0

            # ---- 3 升到安全高度后立刻并行转臂（不等升到顶） ----
            d = wait_rise_time(log, ROTATE_AT_MS)
            t0 = time.monotonic()
            fire_dm(log, 'rotate_slot1_parallel', SLOT1_DEG)
            d = wait_dm(log, 'rotate_slot1_parallel', SLOT1_DEG)
            rep['slot1_deg_actual'] = d['g_dm_can2_logical_position_deg']
            mark('rotate_slot1_parallel', t0)

            d = wait_it_done(log, 'rise', 4, 90)
            rep['lift_top'] = d
            assert at_top(d), 'rise did not reach verified top'
            mark('rise_to_top', rise_t0)

            # ---- 4 松爪120ms + 转回180 ----
            t0 = time.monotonic()
            ramp_pwm(PWM_OPEN, 120, 8)
            holding = False
            put('g_dm_parallel_group_active', 0)
            put('g_dm_parallel_group_go', 0)
            d = rotate(log, 'rotate_back', SOURCE_DEG)
            rep['source_deg_actual'] = d['g_dm_can2_logical_position_deg']
            mark('release120_and_rotate_back', t0)

            rep['cycle_complete'] = True
            print('FAST_SERIAL_COMPLETE total=%.1fs' % (sum(rep['timing_ms'].values()) / 1000.0), flush=True)
        except Exception as e:
            rep['error'] = repr(e)
            print('CYCLE_ERROR ' + repr(e), flush=True)
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
                              if k not in ['before', 'lift_top', 'final']}), flush=True)
    return int(not rep.get('cycle_complete'))


if __name__ == '__main__':
    raise SystemExit(main())
