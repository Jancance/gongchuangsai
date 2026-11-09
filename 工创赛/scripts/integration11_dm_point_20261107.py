"""4310 点位移动到目标角：带重发(处理 queued-stop-restart)与逐帧记录。"""
from pathlib import Path
import sys, json, time, struct

import source_stage2_readonly_sync_probe_20261105 as transport
from pyocd.core.helpers import ConnectHelper
from pyocd.core.session import Session

R = Path(__file__).resolve().parents[1]
O = R / 'evidence/integration/2026-11-07/fast-cycle'
IMAGE = R / 'tmp/integration11_backoff_retry_20261107/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf'


def main():
    goal = float(sys.argv[1])
    tag = sys.argv[2]
    transport.live.IMAGE = IMAGE
    n, _ = transport.live.load_image()
    opt = {'target_override': 'cortex_m', 'connect_mode': 'attach', 'frequency': 1000000,
           'auto_unlock': False, 'resume_on_disconnect': False, 'no_config': True,
           'cmsis_dap.prefer_v1': False, 'cmsis_dap.limit_packets': True}
    pr = ConnectHelper.get_all_connected_probes(blocking=False)
    s = Session(pr[0], options=opt)
    rep = {'goal': goal, 'writes': [], 'attempts': []}

    def g(k):
        return t.read32(n[k])

    def gf(k):
        return struct.unpack('<f', struct.pack('<I', t.read32(n[k])))[0]

    def put(k, v):
        t.write32(n[k], v)
        rep['writes'].append([k, v])

    def putf(k, v):
        t.write32(n[k], struct.unpack('<I', struct.pack('<f', v))[0])

    O.mkdir(parents=True, exist_ok=True)
    with (O / (tag + '-samples.jsonl')).open('x', encoding='utf-8') as log:
        s.open()
        t = s.target
        try:
            put('g_dm_parallel_group_active', 0)
            put('g_dm_parallel_group_go', 0)
            put('g_dm_integrated_owner', 1)
            put('g_dm_point_enable', 1)
            putf('g_dm_posvel_speed_deg_s', 300.0)
            putf('g_dm_pick_target_deg', goal)
            putf('g_dm_place_target_deg', goal)
            ok = False
            for attempt in range(4):
                put('g_dm_jog_request', 4)
                begin = time.monotonic()
                while time.monotonic() - begin < 12:
                    st = g('g_dm_jog_state')
                    res = g('g_dm_jog_result')
                    pos = gf('g_dm_can2_logical_position_deg')
                    row = {'t': round(time.monotonic() - begin, 3), 'state': st, 'result': res,
                           'deg': round(pos, 3), 'vel': round(gf('g_dm_can2_velocity_rad_s'), 4),
                           'hold': g('g_dm_posvel_holding')}
                    log.write(json.dumps(row) + '\n')
                    log.flush()
                    rep['attempts'].append(row)
                    if st == 7 and res == 1 and g('g_dm_posvel_holding') == 1 and abs(pos - goal) <= 1.0:
                        ok = True
                        break
                    if st == 8 and res != 2:
                        break
                    time.sleep(.03)
                if ok:
                    break
            rep['reached'] = ok
            rep['final_deg'] = round(gf('g_dm_can2_logical_position_deg'), 3)
            rep['final_state'] = g('g_dm_jog_state')
            rep['final_result'] = g('g_dm_jog_result')
            print(json.dumps({k: v for k, v in rep.items() if k != 'attempts'}), flush=True)
        except Exception as e:
            rep['error'] = repr(e)
            print(json.dumps(rep), flush=True)
        finally:
            s.close()
            (O / (tag + '-result.json')).write_text(json.dumps(rep, indent=2), encoding='utf-8')
    return 0 if rep.get('reached') else 1


if __name__ == '__main__':
    raise SystemExit(main())
