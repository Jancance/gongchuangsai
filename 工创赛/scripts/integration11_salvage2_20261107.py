"""抢救：把4310转到1号槽并缓慢松爪，再转回180。用dm_can2点位接口，带重试。"""
from pathlib import Path
import sys, json, time, struct

import source_stage2_readonly_sync_probe_20261105 as transport
from pyocd.core.helpers import ConnectHelper

R = Path(__file__).resolve().parents[1]
O = R / 'evidence/integration/2026-11-07/fast-cycle'
IMAGE = R / 'tmp/integration11_short_20261107/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf'
PROBE = '6afaae24'


def main():
    tag = sys.argv[1] if len(sys.argv) > 1 else 'salvage2-01'
    transport.live.IMAGE = IMAGE
    n, _ = transport.live.load_image()
    opt = {'target_override': 'cortex_m', 'connect_mode': 'attach', 'frequency': 1000000,
           'auto_unlock': False, 'resume_on_disconnect': False, 'no_config': True,
           'cmsis_dap.prefer_v1': False, 'cmsis_dap.limit_packets': True}
    s = ConnectHelper.session_with_chosen_probe(unique_id=PROBE, options=opt)
    rep = {'tag': tag, 'writes': [], 'attempts': []}
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

    def pwm(v):
        t.write32(0x40000034, int(v))

    def ramp(to, ms, steps):
        st = t.read32(0x40000034)
        for i in range(1, steps + 1):
            pwm(st + (to - st) * (1.0 * i / steps))
            time.sleep(ms / 1000.0 / steps)
        pwm(to)

    def jog(goal, timeout, name):
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
            st = g('g_dm_jog_state')
            res = g('g_dm_jog_result')
            pos = gf('g_dm_can2_logical_position_deg')
            vel = gf('g_dm_can2_velocity_rad_s')
            rep['attempts'].append({'n': name, 't': round(time.monotonic() - begin, 2),
                                    'state': st, 'res': res, 'deg': round(pos, 2), 'vel': round(vel, 3)})
            if st == 7 and res == 1 and g('g_dm_posvel_holding') == 1 and abs(pos - goal) <= 1.0:
                return True
            if st == 8 and res not in (2,):
                return False
            time.sleep(.03)
        return False

    O.mkdir(parents=True, exist_ok=True)
    s.open()
    t = s.target
    try:
        rep['before'] = {'claw': t.read32(0x40000034), 'deg': round(gf('g_dm_can2_logical_position_deg'), 3),
                         'lift_raw': g('g_lift_ruler_after_raw'), 'lift_state': g('g_lift_ruler_state')}
        back = False
        for k in range(8):
            if jog(180.0, 15, 'back-%d' % k):
                back = True
                break
            time.sleep(1.2)
        rep['back_at_180'] = back
        rep['final'] = {'claw': t.read32(0x40000034),
                        'deg': round(gf('g_dm_can2_logical_position_deg'), 3),
                        'vel': round(gf('g_dm_can2_velocity_rad_s'), 3)}
        print(json.dumps({k: v for k, v in rep.items() if k != 'attempts'}), flush=True)
    except Exception as e:
        rep['error'] = repr(e)
        print('ERR', repr(e), flush=True)
    finally:
        s.close()
        (O / (tag + '-result.json')).write_text(json.dumps(rep, indent=2), encoding='utf-8')
    return 0 if rep.get('back_at_180') else 1


if __name__ == '__main__':
    raise SystemExit(main())
