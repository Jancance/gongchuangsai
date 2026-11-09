"""视觉串口解析 + 取件事务组装的台面验证。
不接香橙派也能验：固件提供 g_vis_test_byte/g_vis_test_push 注入通道，字节走的是和 UART 完全相同的
帧状态机（只证明解析逻辑，不证明电气层）。
"""
from pathlib import Path
import sys, json, time, struct, hashlib

import source_stage2_readonly_sync_probe_20261105 as transport
from pyocd.core.helpers import ConnectHelper
from pyocd.core.target import Target

R = Path(__file__).resolve().parents[1]
O = R / 'evidence/integration/2026-11-07/vision-link'
IMAGE = R / 'tmp/integration11_stream_20261107/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf'
PROBE = '6afaae24'

W32 = ['g_vis_rx_bytes', 'g_vis_rx_bad', 'g_vis_test_byte', 'g_vis_test_push',
       'g_vis_mat_cycle', 'g_vis_mat_left', 'g_vis_mat_right', 'g_vis_mat_down', 'g_vis_mat_tick',
       'g_vis_center_cycle', 'g_vis_center_x', 'g_vis_center_y', 'g_vis_center_tick',
       'g_vis_taught_mask', 'g_vis_pick_region', 'g_vis_txn_result',
       'g_src_txn_revision', 'g_src_txn_consumed', 'g_src_txn_result', 'g_src_txn_age_ms', 'g_src_slot',
       'g_it_state', 'g_it_cause', 'g_it_rejected', 'g_it_action', 'g_src_now_ms',
       'g_dm_jog_state', 'g_lift_ruler_state']
F32 = ['g_vis_mm_per_px', 'g_vis_ref_x', 'g_vis_ref_y', 'g_src_pick_deg', 'g_src_extension_mm',
       'g_src_chassis_dx_mm', 'g_src_chassis_dy_mm', 'g_dm_can2_logical_position_deg']
ARRS = {'g_vis_region_deg': 3, 'g_vis_region_ext_mm': 3}


def main():
    tag = sys.argv[1] if len(sys.argv) > 1 else 'vs-01'
    sha = hashlib.sha256(IMAGE.read_bytes()).hexdigest()
    transport.live.IMAGE = IMAGE
    n, segs = transport.live.load_image()
    miss = [k for k in W32 + F32 if k not in n]
    assert not miss, miss
    miss_a = [k for k in ARRS if k not in n]
    assert not miss_a, miss_a

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
        target.write32(n[k], int(v) & 0xFFFFFFFF)

    def pf(k, v):
        target.write32(n[k], struct.unpack('<I', struct.pack('<f', float(v)))[0])

    def ga(k, i):
        return struct.unpack('<f', struct.pack('<I', target.read32(n[k] + 4 * i)))[0]

    def inject(frame, gap_ms=4):
        for b in frame:
            p('g_vis_test_byte', b)
            p('g_vis_test_push', 1)
            time.sleep(gap_ms / 1000.0)

    def take_txn(timeout=6.0):
        takes0 = g('g_vis_center_cycle')  # 只用来占位，真正判据是下面轮询
        before = g('g_src_txn_revision')
        r0 = g('g_vis_txn_result')
        p('g_it_request', 12)
        begin = time.monotonic()
        while time.monotonic() - begin < timeout:
            time.sleep(.03)
            st = g('g_it_state')
            rev = g('g_src_txn_revision')
            if rev != before or g('g_vis_txn_result') != r0:
                break
            if st == 2 and g('g_it_state') == 2:
                # 被拒时不会进完成态，用 revision 变化或 result 变化都已经覆盖；
                # 这里再给一次机会（下一个 tick 结果就稳定了）
                pass
        time.sleep(0.12)
        return {'result': g('g_vis_txn_result'), 'rejected': g('g_it_rejected'),
                'src_result': g('g_src_txn_result'), 'consumed': g('g_src_txn_consumed'),
                'revision': g('g_src_txn_revision'), 'age_ms': g('g_src_txn_age_ms'),
                'pick_deg': round(gf('g_src_pick_deg'), 3),
                'ext_mm': round(gf('g_src_extension_mm'), 3),
                'dx_mm': round(gf('g_src_chassis_dx_mm'), 3),
                'dy_mm': round(gf('g_src_chassis_dy_mm'), 3)}

    session.open()
    target = session.target
    try:
        for a, b in segs:
            for k in range(0, len(b), 4096):
                assert bytes(target.read_memory_block8(a + k, len(b[k:k + 4096]))) == b[k:k + 4096]
        rep['before'] = {'mat_cycle': g('g_vis_mat_cycle'), 'center_cycle': g('g_vis_center_cycle'),
                         'rx_bytes': g('g_vis_rx_bytes'), 'rx_bad': g('g_vis_rx_bad')}

        # ---------- A. 帧解析 ----------
        c0 = g('g_vis_mat_cycle')
        inject([0xCC, 0x02, 0x01, 0x02, 0x03, 0xDD])
        time.sleep(0.1)
        ok_a1 = (g('g_vis_mat_cycle') == c0 + 1 and g('g_vis_mat_left') == 1
                 and g('g_vis_mat_right') == 2 and g('g_vis_mat_down') == 3)
        rep['cases'].append({'case': 'frame_CC02', 'expect': 'left/right/down=1/2/3',
                             'actual': [g('g_vis_mat_left'), g('g_vis_mat_right'), g('g_vis_mat_down')],
                             'pass': ok_a1})

        c0 = g('g_vis_center_cycle')
        inject([0xCC, 0x05, 0x40, 0x01, 0x58, 0x02, 0xDD])
        time.sleep(0.1)
        ok_a2 = (g('g_vis_center_cycle') == c0 + 1 and g('g_vis_center_x') == 320
                 and g('g_vis_center_y') == 600)
        rep['cases'].append({'case': 'frame_CC05', 'expect': 'center=(320,600)',
                             'actual': [g('g_vis_center_x'), g('g_vis_center_y')], 'pass': ok_a2})

        bad0 = g('g_vis_rx_bad')
        inject([0xCC, 0x09, 0x01, 0xDD])          # 未知功能码
        time.sleep(0.1)
        ok_a3 = g('g_vis_rx_bad') == bad0 + 1
        rep['cases'].append({'case': 'frame_bad_func', 'expect': 'bad+1',
                             'actual': g('g_vis_rx_bad') - bad0, 'pass': ok_a3})

        # 载荷里出现 0xCC/0xDD 也必须能正确解析（长度驱动，不靠重同步）
        c0 = g('g_vis_center_cycle')
        inject([0xCC, 0x05, 0xCC, 0xDD, 0xCC, 0xDD, 0xDD])
        time.sleep(0.1)
        ok_a4 = (g('g_vis_center_cycle') == c0 + 1 and g('g_vis_center_x') == 0xDDCC
                 and g('g_vis_center_y') == 0xDDCC)
        rep['cases'].append({'case': 'frame_payload_CCDD', 'expect': 'x=y=0xDDCC',
                             'actual': [hex(g('g_vis_center_x')), hex(g('g_vis_center_y'))], 'pass': ok_a4})

        # ---------- B. 事务组装 ----------
        p('g_vis_pick_region', 0)
        p('g_vis_taught_mask', 0)
        r = take_txn()
        ok_b1 = (r['result'] == 6 and r['rejected'] != 0)
        rep['cases'].append({'case': 'txn_untaught', 'expect': 'result=6 拒绝', 'actual': r, 'pass': ok_b1})

        # 示教区域0：193度/48mm伸缩；像素换算 0.4mm/px，参考中心 (310,590)
        pf('g_vis_region_deg', 193.0)
        target.write32(n['g_vis_region_ext_mm'], struct.unpack('<I', struct.pack('<f', 48.0))[0])
        p('g_vis_taught_mask', 1)
        pf('g_vis_mm_per_px', 0.4)
        pf('g_vis_ref_x', 310.0)
        pf('g_vis_ref_y', 590.0)
        inject([0xCC, 0x02, 0x01, 0x00, 0x00, 0xDD])
        inject([0xCC, 0x05, 0x40, 0x01, 0x58, 0x02, 0xDD])   # (320,600)
        r = take_txn()
        ok_b2 = (r['result'] == 0 and r['src_result'] == 1 and r['rejected'] == 0
                 and abs(r['pick_deg'] - 193.0) < 0.01 and abs(r['ext_mm'] - 48.0) < 0.01
                 and abs(r['dx_mm'] - 4.0) < 0.05 and abs(r['dy_mm'] - 4.0) < 0.05)
        rep['cases'].append({'case': 'txn_from_vision', 'expect': 'deg=193 ext=48 dx=4 dy=4 接受',
                             'actual': r, 'pass': ok_b2})

        # mm_per_px 未标定 -> 纠偏必须是 0，不许猜
        pf('g_vis_mm_per_px', 0.0)
        inject([0xCC, 0x02, 0x01, 0x00, 0x00, 0xDD])
        inject([0xCC, 0x05, 0x40, 0x01, 0x58, 0x02, 0xDD])
        r = take_txn()
        ok_b3 = (r['src_result'] == 1 and abs(r['dx_mm']) < 0.001 and abs(r['dy_mm']) < 0.001)
        rep['cases'].append({'case': 'txn_no_mm_scale', 'expect': '接受但 dx=dy=0',
                             'actual': r, 'pass': ok_b3})

        # 视觉结果过期 -> 拒绝
        time.sleep(3.3)
        r = take_txn()
        ok_b4 = (r['result'] == 7 or r['src_result'] == 3)
        rep['cases'].append({'case': 'txn_stale_vision', 'expect': 'result=7 或 src=3 拒绝',
                             'actual': r, 'pass': ok_b4})

        rep['all_pass'] = all(c['pass'] for c in rep['cases'])
    except Exception as e:
        rep['error'] = repr(e)
        print('VIS_ERROR ' + repr(e), flush=True)
    finally:
        session.close()
        O.mkdir(parents=True, exist_ok=True)
        (O / (tag + '-result.json')).write_text(json.dumps(rep, indent=2), encoding='utf-8')
        for c in rep['cases']:
            print('CASE %-22s %s  %s' % (c['case'], 'PASS' if c['pass'] else 'FAIL', json.dumps(c, ensure_ascii=False)), flush=True)
        print(json.dumps({k: v for k, v in rep.items() if k not in ('before', 'cases')}, ensure_ascii=False), flush=True)
    return 0 if rep.get('all_pass') else 1


if __name__ == '__main__':
    raise SystemExit(main())
