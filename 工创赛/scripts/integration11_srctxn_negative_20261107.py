"""视觉取件目标事务的否定用例：验证固件会拒绝过期/越界/重复事务，且被拒后不产生任何动作。
只读位置做对比，不发下降/转臂请求；不走恢复流程（要求板子当前无故障）。
"""
from pathlib import Path
import sys, json, time, struct, hashlib

import source_stage2_readonly_sync_probe_20261105 as transport
from pyocd.core.helpers import ConnectHelper
from pyocd.core.target import Target

R = Path(__file__).resolve().parents[1]
O = R / 'evidence/integration/2026-11-07/source-txn'
IMAGE = R / 'tmp/integration11_stream_20261107/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf'
PROBE = '6afaae24'

W32 = ['g_it_request', 'g_it_state', 'g_it_cause', 'g_it_rejected', 'g_it_action',
       'g_src_txn_revision', 'g_src_txn_consumed', 'g_src_txn_seq', 'g_src_txn_tick',
       'g_src_txn_result', 'g_src_txn_age_ms', 'g_src_slot', 'g_src_now_ms', 'g_src_txn_takes',
       'g_dm_jog_state', 'g_lift_ruler_state', 'g_lift_ruler_cause', 'g_startup_axes_cause']
F32 = ['g_src_pick_deg', 'g_src_chassis_dx_mm', 'g_src_chassis_dy_mm', 'g_dm_can2_logical_position_deg']


def main():
    tag = sys.argv[1] if len(sys.argv) > 1 else 'neg-01'
    sha = hashlib.sha256(IMAGE.read_bytes()).hexdigest()
    transport.live.IMAGE = IMAGE
    n, segs = transport.live.load_image()
    miss = [k for k in W32 + F32 if k not in n]
    assert not miss, miss

    opt = {'target_override': 'cortex_m', 'connect_mode': 'attach', 'frequency': 1000000,
           'auto_unlock': False, 'resume_on_disconnect': False, 'no_config': True,
           'cmsis_dap.prefer_v1': False, 'cmsis_dap.limit_packets': True}
    session = ConnectHelper.session_with_chosen_probe(unique_id=PROBE, options=opt)
    rep = {'tag': tag, 'image_sha256': sha, 'cases': []}
    target = None

    def g(k):
        return target.read32(n[k])

    def gf(k):
        return struct.unpack('<f', struct.pack('<I', target.read32(n[k])))[0]

    def p(k, v):
        target.write32(n[k], int(v))

    def pf(k, v):
        target.write32(n[k], struct.unpack('<I', struct.pack('<f', float(v)))[0])

    def snap():
        d = {k: g(k) for k in W32}
        for k in F32:
            d[k] = gf(k)
        return d

    def commit(deg, slot, dx, dy, seq, tick_offset):
        now = g('g_src_now_ms')
        pf('g_src_pick_deg', deg); pf('g_src_chassis_dx_mm', dx); pf('g_src_chassis_dy_mm', dy)
        p('g_src_slot', slot); p('g_src_txn_seq', seq)
        p('g_src_txn_tick', now + tick_offset)
        p('g_src_txn_revision', g('g_src_txn_revision') + 1)

    def take():
        before = snap()
        takes0 = g('g_src_txn_takes')
        p('g_it_request', 11)
        begin = time.monotonic()
        while time.monotonic() - begin < 5.0:
            d = snap()
            # 必须等到固件真的又消费了一次，不能只看 state==2（上一轮就是这个状态）
            # 被拒时固件不会进入完成态，所以只靠 takes 计数判断"这一笔已被评估过"
            if g('g_src_txn_takes') > takes0:
                break
            time.sleep(.02)
        else:
            raise RuntimeError('txn take timeout')
        # takes++ 发生在写入 result/rejected 之前，必须重新读一次才是本次评估的最终结果
        time.sleep(0.08)
        d = snap()
        time.sleep(0.30)   # 观察窗，确认没有动作被触发
        after = snap()
        moved = abs(after['g_dm_can2_logical_position_deg'] - before['g_dm_can2_logical_position_deg'])
        return d, before, after, moved

    session.open()
    target = session.target
    try:
        for a, b in segs:
            for k in range(0, len(b), 4096):
                assert bytes(target.read_memory_block8(a + k, len(b[k:k + 4096]))) == b[k:k + 4096]
        base = snap()
        rep['before'] = base
        if base['g_startup_axes_cause'] or base['g_lift_ruler_cause'] or base['g_it_cause']:
            raise RuntimeError('board has latched fault, reset first')

        cases = [
            ('stale_5s', dict(deg=180.0, slot=1, dx=0.0, dy=0.0, seq=201, tick_offset=-5000), 3,
             '过期(5s前的时间戳)'),
            ('deg_out_of_range', dict(deg=300.0, slot=1, dx=0.0, dy=0.0, seq=202, tick_offset=0), 4,
             '取件角 300 越界'),
            ('chassis_dx_overflow', dict(deg=180.0, slot=1, dx=25.0, dy=0.0, seq=203, tick_offset=0), 4,
             '底盘纠偏 25mm 越界'),
            ('slot_out_of_range', dict(deg=180.0, slot=4, dx=0.0, dy=0.0, seq=204, tick_offset=0), 4,
             '载盘槽位 4 越界'),
            ('valid_first', dict(deg=180.0, slot=1, dx=0.0, dy=0.0, seq=204, tick_offset=0), 1,
             '先来一笔合法事务(seq=204)'),
            ('dup_seq', dict(deg=180.0, slot=1, dx=0.0, dy=0.0, seq=204, tick_offset=0), 5,
             '重复序号(seq=204 再来一次)'),
        ]
        ok = True
        for name, kw, expect, desc in cases:
            commit(**kw)
            d, before, after, moved = take()
            # 接受(expect=1)时 g_it_rejected 必须为 0；拒绝时必须非 0；两种情况都不许产生动作
            want_rej = 0 if expect == 1 else 1
            good = (d['g_src_txn_result'] == expect and (d['g_it_rejected'] != 0) == bool(want_rej)
                    and moved < 0.05)
            ok = ok and good
            rec = {'case': name, 'desc': desc, 'expect_result': expect,
                   'result': d['g_src_txn_result'], 'it_rejected': d['g_it_rejected'],
                   'arm_moved_deg': round(moved, 4),
                   'tick': d['g_src_txn_tick'], 'now_ms': d['g_src_now_ms'],
                   'age': d['g_src_txn_age_ms'], 'pick_deg_read': round(d['g_src_pick_deg'], 3),
                   'pass': good}
            rep['cases'].append(rec)
            print('CASE %-20s result=%d (expect %d) rejected=%d moved=%.3f tick=%d now=%d age=%d deg=%.1f -> %s'
                  % (name, d['g_src_txn_result'], expect, d['g_it_rejected'], moved,
                     d['g_src_txn_tick'], d['g_src_now_ms'], d['g_src_txn_age_ms'],
                     d['g_src_pick_deg'], 'PASS' if good else 'FAIL'), flush=True)

        # 一个正常事务必须被接受，证明拒绝是"针对性"的而不是一律拒绝
        commit(deg=179.0, slot=2, dx=1.0, dy=-1.0, seq=205, tick_offset=0)
        d, before, after, moved = take()
        good = d['g_src_txn_result'] == 1 and d['g_it_rejected'] == 0 and moved < 0.05
        ok = ok and good
        rep['cases'].append({'case': 'valid_accepted', 'desc': '正常事务应接受且不自动动作',
                             'expect_result': 1, 'result': d['g_src_txn_result'],
                             'it_rejected': d['g_it_rejected'], 'arm_moved_deg': round(moved, 4),
                             'pass': good})
        print('CASE %-20s result=%d rejected=%d moved=%.3f -> %s'
              % ('valid_accepted', d['g_src_txn_result'], d['g_it_rejected'], moved,
                 'PASS' if good else 'FAIL'), flush=True)
        rep['all_pass'] = ok
    except Exception as e:
        rep['error'] = repr(e)
        print('NEG_ERROR ' + repr(e), flush=True)
    finally:
        rep['final'] = snap()
        session.close()
        O.mkdir(parents=True, exist_ok=True)
        (O / (tag + '-result.json')).write_text(json.dumps(rep, indent=2), encoding='utf-8')
        print(json.dumps({k: v for k, v in rep.items() if k != 'before'}), flush=True)
    return 0 if rep.get('all_pass') else 1


if __name__ == '__main__':
    raise SystemExit(main())
