"""抢救本轮：夹爪仍夹着物料、升降停在顶部(raw~36008)、4310仍在180。
把4310转到1号槽35.758，缓慢松爪让物料落到1号槽，再把4310转回180。不复位、不伪造状态。"""
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
SLOT1 = 35.758
BACK = 180.0
W32 = ['g_dm_jog_state', 'g_dm_jog_result', 'g_dm_jog_request', 'g_dm_posvel_holding',
       'g_dm_can2_logical_position_deg', 'g_dm_parallel_group_active', 'g_dm_parallel_group_go',
       'g_dm_can2_feedback_valid', 'g_dm_can2_motor_state', 'g_dm_can2_state', 'g_dm_can2_result',
       'g_source_parallel_stop_mask', 'g_source_parallel_state',
       'g_lift_ruler_state', 'g_lift_ruler_cause', 'g_lift_ruler_after_raw',
       'g_vm_test_request', 'g_safety_ready', 'g_safety_fault', 'g_rtos_fault']
F32 = ['g_dm_posvel_speed_deg_s', 'g_dm_pick_target_deg', 'g_dm_place_target_deg',
       'g_dm_can2_velocity_rad_s']


def main():
    tag = sys.argv[1] if len(sys.argv) > 1 else 'salvage-01'
    assert hashlib.sha256(IMAGE.read_bytes()).hexdigest() == EXPECTED
    transport.live.IMAGE = IMAGE
    n, segs = transport.live.load_image()
    opt = {'target_override': 'cortex_m', 'connect_mode': 'attach', 'frequency': 1000000,
           'auto_unlock': False, 'resume_on_disconnect': False, 'no_config': True,
           'cmsis_dap.prefer_v1': False, 'cmsis_dap.limit_packets': True}
    pr = ConnectHelper.get_all_connected_probes(blocking=False)
    assert len(pr) == 1 and pr[0].unique_id == PROBE
    s = Session(pr[0], options=opt)
    rep = {'tag': tag, 'writes': [], 'timing_ms': {}}
    t = None

    def g(k):
        return t.read32(n[k])

    def gf(k):
        return struct.unpack('<f', struct.pack('<I', t.read32(n[k])))[0]

    def put(k, v):
        t.write32(n[k], v)
        rep['writes'].append([k, v])

    def putf(k, v):
        t.write32(n[k], struct.unpack('<I', struct.pack('<f', v))[0])
        rep['writes'].append([k, v])

    def pwm(v):
        t.write32(0x40000034, int(v))

    def ramp_pwm(to, ms, steps):
        start = t.read32(0x40000034)
        for i in range(1, steps + 1):
            pwm(start + (to - start) * (1.0 * i / steps))
            time.sleep(ms / 1000.0 / steps)
        pwm(to)

    def snap():
        d = {k: g(k) for k in W32}
        for k in F32:
            d[k] = gf(k)
        d['claw'] = t.read32(0x40000034)
        return d

    def jog_to(goal, timeout, phase):
        put('g_dm_parallel_group_active', 0)
        put('g_dm_parallel_group_go', 0)
        put('g_dm_integrated_owner', 1)
        put('g_dm_point_enable', 1)
        putf('g_dm_posvel_speed_deg_s', 300.0)
        putf('g_dm_pick_target_deg', goal)
        putf('g_dm_place_target_deg', goal)
        put('g_dm_jog_request', 4)
        begin = time.monotonic()
        while time.monotonic() - begin < timeout:
            d = snap()
            if d['g_dm_jog_state'] == 7 and d['g_dm_jog_result'] == 1 and d['g_dm_posvel_holding'] == 1 \
                    and d['g_dm_can2_feedback_valid'] == 1 \
                    and abs(d['g_dm_can2_logical_position_deg'] - goal) <= 1.0:
                rep[phase] = d
                return d
            if d['g_dm_jog_state'] == 8 and d['g_dm_jog_result'] not in (2,):
                rep[phase + '_abort'] = d
                raise RuntimeError(phase + ' jog abort result=' + str(d['g_dm_jog_result']))
            time.sleep(.02)
        raise RuntimeError(phase + ' timeout')

    O.mkdir(parents=True, exist_ok=True)
    with (O / (tag + '-samples.jsonl')).open('x', encoding='utf-8') as log:
        try:
            s.open()
            t = s.target
            for a, b in segs:
                for k in range(0, len(b), 4096):
                    assert bytes(t.read_memory_block8(a + k, len(b[k:k + 4096]))) == b[k:k + 4096]
            rep['before'] = d = snap()
            assert d['claw'] == 1537, 'gripper not holding: %s' % d['claw']
            assert abs(d['g_lift_ruler_after_raw'] - 36008) <= 400, d['g_lift_ruler_after_raw']
            # 等SPF停止掩码清空，最多8秒
            begin = time.monotonic()
            while time.monotonic() - begin < 8:
                d = snap()
                if d['g_source_parallel_stop_mask'] == 0 and d['g_dm_jog_state'] in (0, 7):
                    break
                time.sleep(.1)
            rep['after_settle'] = snap()

            t0 = time.monotonic()
            d = jog_to(SLOT1, 20, 'at_slot1')
            rep['timing_ms']['rotate_slot1'] = round((time.monotonic() - t0) * 1000)
            print('AT_SLOT1 %.3f deg' % d['g_dm_can2_logical_position_deg'], flush=True)

            t0 = time.monotonic()
            ramp_pwm(2056, 600, 20)
            rep['released'] = True
            rep['timing_ms']['release_ramp'] = round((time.monotonic() - t0) * 1000)
            print('RELEASED at slot1', flush=True)
            time.sleep(.4)

            t0 = time.monotonic()
            d = jog_to(BACK, 20, 'at_180')
            rep['timing_ms']['rotate_back'] = round((time.monotonic() - t0) * 1000)
            rep['final'] = snap()
            rep['salvage_complete'] = True
            print('SALVAGE_COMPLETE %.3f deg claw=%s' % (d['g_dm_can2_logical_position_deg'], d['claw']), flush=True)
        except Exception as e:
            rep['error'] = repr(e)
            print('SALVAGE_ERROR ' + repr(e), flush=True)
        finally:
            s.close()
            (O / (tag + '-result.json')).write_text(json.dumps(rep, indent=2), encoding='utf-8')
            print(json.dumps({k: v for k, v in rep.items()
                              if k not in ['before', 'after_settle', 'at_slot1', 'at_180', 'final']}), flush=True)
    return int(not rep.get('salvage_complete'))


if __name__ == '__main__':
    raise SystemExit(main())
