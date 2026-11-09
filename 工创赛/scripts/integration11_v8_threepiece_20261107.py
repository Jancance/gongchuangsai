"""三件循环（第二段骨架）：原料侧同点取 3 件，逐件放入三槽载盘 1/2/3 号槽。
要点：
  1) 每件只做一次下降->夹紧->回升(途中并行转臂)->松爪->回 180，不重复回零/回参考；
  2) 三槽载盘是伺服转盘(0/120/240 度 = 1/2/3 号槽)，转位走固件 IT 请求10，藏在下一件的下降里；
  3) 视觉/底盘纠偏尚未接入：本脚本假设车已停在原料侧固定点，取件角度固定 SOURCE_DEG。
"""
from pathlib import Path
import sys, json, time, struct, hashlib

import source_stage2_readonly_sync_probe_20261105 as transport
from pyocd.core.helpers import ConnectHelper
from pyocd.core.target import Target

R = Path(__file__).resolve().parents[1]
O = R / 'evidence/integration/2026-11-07/three-piece'
IMAGE = R / 'tmp/integration11_stream_20261107/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf'
EXPECTED = '5576cc9e259ba60141dcc7e4997b788360f440fb29ea77c40219e7ce762b4890'
PROBE = '6afaae24'

RAW_PER_PULSE = 11.25
TARGET_PULSES = 118800
TOTAL_RAW = int(TARGET_PULSES * RAW_PER_PULSE)
SLOT_DEG = 35.758
SOURCE_DEG = 180.0
ROTATE_AT_MS = 1400
CARRIER_DEG = {'slot1': 0.0, 'slot2': 120.0, 'slot3': 240.0}
# 离线模拟"视觉输出"：真机由串口写 g_src_pick_deg/... 这组字段，脚本只扮演视觉侧
PICK_DEG = {1: 180.0, 2: 179.3, 3: 180.7}
CH_DX = {1: 0.0, 2: 1.5, 3: -2.0}   # 底盘纠偏 mm（离线只记录，不驱动底盘）
CH_DY = {1: 0.0, 2: -0.8, 3: 1.2}
PWM_OPEN = 2056      # 210度 识别姿态
PWM_PREGRIP = 1685   # 160度 下降途中预收
PWM_GRIP = 1537      # 140度 夹紧

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
       'g_dm_jog_elapsed_ms', 'g_dm_posvel_holding', 'g_dm_parallel_group_active', 'g_dm_point_enable',
       'g_vehicle_arm_state', 'g_vehicle_lift_feedback_valid',
       'g_vm_test_request', 'g_safety_ready', 'g_safety_fault', 'g_rtos_fault',
       'g_can_ctrl_fault_latched', 'g_source_parallel_state', 'g_source_parallel_cause',
       'g_src_txn_revision', 'g_src_txn_consumed', 'g_src_txn_seq', 'g_src_txn_tick',
       'g_src_txn_result', 'g_src_txn_age_ms', 'g_src_slot', 'g_src_now_ms']
F32 = ['g_dm_can2_logical_position_deg', 'g_dm_can2_velocity_rad_s', 'g_vehicle_lift_current_deg',
       'g_dm_pick_target_deg', 'g_dm_place_target_deg', 'g_dm_posvel_speed_deg_s',
       'g_it_carrier_deg', 'g_it_dm_deg', 'g_it_extension_mm', 'g_it_lift_pulses',
       'g_src_pick_deg', 'g_src_chassis_dx_mm', 'g_src_chassis_dy_mm']
FAULT_KEYS = ['g_lift_ruler_cause', 'g_it_cause', 'g_safety_fault', 'g_rtos_fault',
              'g_can_ctrl_fault_latched', 'g_startup_axes_cause']


def main():
    tag = sys.argv[1] if len(sys.argv) > 1 else 'three-01'
    assert tag and all(c.isalnum() or c in '-_' for c in tag)
    assert hashlib.sha256(IMAGE.read_bytes()).hexdigest() == EXPECTED, 'image sha mismatch'
    transport.live.IMAGE = IMAGE
    n, segs = transport.live.load_image()
    miss = [k for k in W32 + F32 if k not in n]
    assert not miss, miss

    opt = {'target_override': 'cortex_m', 'connect_mode': 'attach', 'frequency': 1000000,
           'auto_unlock': False, 'resume_on_disconnect': False, 'no_config': True,
           'cmsis_dap.prefer_v1': False, 'cmsis_dap.limit_packets': True}
    session = ConnectHelper.session_with_chosen_probe(unique_id=PROBE, options=opt)

    rep = {'tag': tag, 'image_sha256': EXPECTED, 'pieces': [], 'target_pulses': TARGET_PULSES}
    target = None

    def get(k):
        return target.read32(n[k])

    def getf(k):
        return struct.unpack('<f', struct.pack('<I', target.read32(n[k])))[0]

    def put(k, v):
        target.write32(n[k], int(v))

    def putf(k, v):
        target.write32(n[k], struct.unpack('<I', struct.pack('<f', float(v)))[0])

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
        d['t'] = time.monotonic()
        d['claw_pwm_us'] = target.read32(0x40000034)
        return d

    def faults(d):
        return {k: d[k] for k in FAULT_KEYS if d[k]}

    def at_top(d):
        return (d['g_lift_compare_top_valid'] == 1 and d['g_lift_compare_backoff_valid'] == 1
                and d['g_vehicle_lift_feedback_valid'] == 1
                and abs(d['g_vehicle_lift_current_deg'] - d['g_lift_compare_safe_raw'] / 100.0) <= 1.0)

    def mark(name, t0):
        ms = (time.monotonic() - t0) * 1000
        rep.setdefault('timing_ms', {})[name] = round(ms)
        print('STEP %-30s %7.0f ms' % (name, ms), flush=True)

    def wait_lift(log, phase, expect_op, timeout):
        begin = time.monotonic()
        old = get('g_lift_ruler_samples')
        while time.monotonic() - begin < timeout:
            d = snap()
            d['host_phase'] = phase
            log.write(json.dumps(d) + '\n')
            log.flush()
            assert not faults(d), faults(d)
            if (d['g_lift_ruler_state'] == 4 and d['g_lift_compare_operation'] == expect_op
                    and d['g_lift_ruler_samples'] != old):
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
            if d['g_it_state'] == 9 and d['g_it_action'] == action:
                raise RuntimeError(phase + ' it_fail cause=' + str(d['g_it_cause'])
                                   + ' rejected=' + str(d['g_it_rejected']))
            time.sleep(.02)
        raise RuntimeError(phase + ' timeout')

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

    def rotate(log, phase, deg):
        """经 IT 的纯转臂（IT 空闲时用）。"""
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
        return d

    def carrier_slot(log, slot, label):
        """三槽载盘转位：固件 IT 请求10，700ms 行程窗口，无位置反馈。"""
        putf('g_it_carrier_deg', CARRIER_DEG[slot])
        put('g_it_request', 10)
        d = wait_it(log, 'carrier_' + label, 10, 6)
        rep.setdefault('carrier', []).append({'label': label, 'slot': slot,
                                              'deg': CARRIER_DEG[slot], 'elapsed_ms': d['g_it_elapsed_ms']})
        return d

    def fire_dm(log, phase, deg):
        """IT 忙(升降在动)时直接给 DM 点位任务下目标，与升降并行。请求号按方向选，选错会被固件拒(result=14)。"""
        cur = getf('g_dm_can2_logical_position_deg')
        req = 4 if deg >= cur else 5
        rep.setdefault('fires', []).append({'phase': phase, 'deg': deg, 'from': round(cur, 3), 'req': req})
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
                return d
            if d['g_dm_jog_state'] == 8:
                raise RuntimeError('%s dm_jog failed result=%s first_fault=%s'
                                   % (phase, d['g_dm_jog_result'], d['g_dm_jog_first_fault']))
            time.sleep(.02)
        raise RuntimeError(phase + ' timeout')

    def wait_rise_time(log, ms, timeout=20):
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
            if st == 4 or (time.monotonic() - begin) * 1000.0 >= ms:
                return d
            time.sleep(.02)
        raise RuntimeError('rise_wait timeout')

    def grab(log, piece, after_start=None):
        """下降(含 210->160 预收) -> 夹紧。after_start 在下降被接受后回调，用来把载盘转位藏进这 2 秒。"""
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
        if after_start is not None:
            after_start()
        ramp_pwm(PWM_PREGRIP, 420, 14)
        d = wait_lift(log, 'p%d_down' % piece, 2, 35)
        put('g_it_lift_permit', 0)
        assert abs(d['g_lift_ruler_after_raw'] - TOTAL_RAW) <= 40, d['g_lift_ruler_after_raw']
        down_ms = (time.monotonic() - t0) * 1000.0
        t1 = time.monotonic()
        ramp_pwm(PWM_GRIP, 80, 8)
        time.sleep(0.30)
        return down_ms, (time.monotonic() - t1) * 1000.0

    def vision_commit(piece, deg, slot, dx, dy, tick_offset_ms=0):
        """扮演视觉侧：写字段后 revision+1 提交一次事务。"""
        now = get('g_src_now_ms')
        putf('g_src_pick_deg', deg)
        putf('g_src_chassis_dx_mm', dx)
        putf('g_src_chassis_dy_mm', dy)
        put('g_src_slot', slot)
        put('g_src_txn_seq', 100 + piece)
        put('g_src_txn_tick', now + tick_offset_ms)
        put('g_src_txn_revision', get('g_src_txn_revision') + 1)
        return now

    def txn_take(log, label, expect_ok=True):
        """经 IT 请求11 让固件消费事务；expect_ok=False 用于验证被拒绝。"""
        put('g_it_request', 11)
        d = wait_it_done(log, 'txn_' + label, 11, 6)
        rec = {'label': label, 'result': d['g_src_txn_result'], 'age_ms': d['g_src_txn_age_ms'],
               'rejected': d['g_it_rejected'], 'slot': d['g_src_slot'],
               'pick_deg': round(d['g_src_pick_deg'], 3), 'consumed': d['g_src_txn_consumed']}
        rep.setdefault('txn', []).append(rec)
        print('TXN %s %s' % (label, json.dumps(rec)), flush=True)
        if expect_ok:
            assert d['g_it_rejected'] == 0, ('txn rejected', rec)
            assert d['g_src_txn_result'] == 1, ('txn not accepted', rec)
        else:
            assert d['g_src_txn_result'] != 1, ('txn should have been refused', rec)
        return d

    def place(log, piece):
        """回升(途中并行转臂) -> 松爪。"""
        rise_t0 = time.monotonic()
        put('g_it_request', 4)
        wait_rise_time(log, ROTATE_AT_MS)
        t_r = time.monotonic()
        fire_dm(log, 'p%d_rotate' % piece, SLOT_DEG)
        wait_dm(log, 'p%d_rotate' % piece, SLOT_DEG)
        rot_ms = (time.monotonic() - t_r) * 1000.0
        d = wait_it_done(log, 'p%d_rise' % piece, 4, 90)
        assert at_top(d), 'rise did not reach verified top'
        rise_ms = (time.monotonic() - rise_t0) * 1000.0
        t0 = time.monotonic()
        ramp_pwm(PWM_OPEN, 120, 8)
        put('g_dm_parallel_group_active', 0)
        put('g_dm_parallel_group_go', 0)
        return rot_ms, rise_ms, (time.monotonic() - t0) * 1000.0

    O.mkdir(parents=True, exist_ok=True)
    with (O / (tag + '-samples.jsonl')).open('x', encoding='utf-8') as log:
        try:
            session.open()
            target = session.target
            assert target.get_state() == Target.State.RUNNING
            for a, b in segs:
                for k in range(0, len(b), 4096):
                    assert bytes(target.read_memory_block8(a + k, len(b[k:k + 4096]))) == b[k:k + 4096]
            rep['before'] = d = snap()

            if (faults(d) or d['g_lift_ruler_state'] in (5, 6) or d['g_lift_ruler_state'] == 0
                    or d['g_it_state'] == 9 or d['g_it_cause'] != 0):
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
                wait_it(log, 'lift_home', 1, 100)
                assert at_top(snap()), 'home did not reach top'
                mark('recover', t0)

            if not at_top(snap()):
                t0 = time.monotonic()
                put('g_it_request', 4)
                wait_it(log, 'return_top', 4, 90)
                mark('return_top', t0)
            assert at_top(snap()), 'not at top'

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
            mark('extension_ready', t0)

            if abs(snap()['g_dm_can2_logical_position_deg'] - SOURCE_DEG) > 3.0:
                t0 = time.monotonic()
                rotate(log, 'ensure_source180', SOURCE_DEG)
                mark('ensure_source180', t0)
            if target.read32(0x40000034) != PWM_OPEN:
                ramp_pwm(PWM_OPEN, 300, 10)

            t_cycle = time.monotonic()
            piece_t0 = time.monotonic()
            pending = 'slot1'
            for piece in (1, 2, 3):
                # ---- 每件开始：视觉提交取件目标 -> 固件校验消费 ----
                t0 = time.monotonic()
                vision_commit(piece, PICK_DEG[piece], piece, CH_DX[piece], CH_DY[piece])
                txn_take(log, 'p%d' % piece)
                rep.setdefault('chassis', []).append({'piece': piece, 'dx_mm': CH_DX[piece],
                                                     'dy_mm': CH_DY[piece], 'simulated': True})
                cur = getf('g_dm_can2_logical_position_deg')
                rec_goto = None
                if abs(cur - PICK_DEG[piece]) > 0.3:
                    rotate(log, 'p%d_go_pick' % piece, PICK_DEG[piece])
                    rec_goto = round(snap()['g_dm_can2_logical_position_deg'], 4)
                mark('p%d_vision_target' % piece, t0)

                hook = None
                lbl = pending

                def hook(lbl=lbl):
                    carrier_slot(log, lbl, lbl)
                down_ms, grip_ms = grab(log, piece, hook)
                rot_ms, rise_ms, rel_ms = place(log, piece)
                rec = {'piece': piece, 'down_ms': round(down_ms), 'grip_ms': round(grip_ms),
                       'rotate_ms': round(rot_ms), 'rise_ms': round(rise_ms), 'release_ms': round(rel_ms),
                       'claw_pwm_after_release': target.read32(0x40000034),
                       'pick_deg': PICK_DEG[piece], 'pick_deg_reached': rec_goto,
                       'deg': round(snap()['g_dm_can2_logical_position_deg'], 4)}
                nxt = PICK_DEG[piece + 1] if piece < 3 else SOURCE_DEG
                t0 = time.monotonic()
                rotate(log, ('p%d_back' % piece) if piece < 3 else 'handoff_back180', nxt)
                rec['back_ms'] = round((time.monotonic() - t0) * 1000.0)
                rec['back_deg_target'] = nxt
                rec['back_deg_actual'] = round(snap()['g_dm_can2_logical_position_deg'], 4)
                if piece < 3:
                    pending = 'slot%d' % (piece + 1)
                rec['piece_ms'] = round((time.monotonic() - piece_t0) * 1000.0)
                rep['pieces'].append(rec)
                piece_t0 = time.monotonic()
                print('PIECE %d %s' % (piece, json.dumps(rec)), flush=True)

            rep['cycle_complete'] = True
            rep['total_ms'] = round((time.monotonic() - t_cycle) * 1000.0)
            print('THREE_PIECE_COMPLETE total=%.1fs' % (rep['total_ms'] / 1000.0), flush=True)
        except Exception as e:
            rep['error'] = repr(e)
            print('CYCLE_ERROR ' + repr(e), flush=True)
        finally:
            if target is not None:
                try:
                    rep['final'] = d = snap()
                    rep['final_at_top'] = at_top(d)
                except Exception as e:
                    rep['final_read_error'] = repr(e)
            session.close()
            (O / (tag + '-result.json')).write_text(json.dumps(rep, indent=2), encoding='utf-8')
            print(json.dumps({k: v for k, v in rep.items()
                              if k not in ['before', 'final']}), flush=True)
    return int(not rep.get('cycle_complete'))


if __name__ == '__main__':
    raise SystemExit(main())
