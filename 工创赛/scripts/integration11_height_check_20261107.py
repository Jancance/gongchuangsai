"""只做一次下降 118800 并保持，供现场尺量。不改其他轴。"""
from pathlib import Path
import sys, json, time, struct, hashlib

import source_stage2_readonly_sync_probe_20261105 as transport
from pyocd.core.helpers import ConnectHelper
from pyocd.core.session import Session
from pyocd.core.target import Target

R = Path(__file__).resolve().parents[1]
O = R / 'evidence/integration/2026-11-07/height-probe'
IMAGE = R / 'tmp/integration11_backoff_retry_20261107/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf'
EXPECTED = 'b90779f76c5f4d5e261e51c5bbabf9551e035d5f63a5e0cd20ad5997f4b48ad5'
TARGET = 118800
RAWPP = 11.25
W = ['g_lift_ruler_state', 'g_lift_ruler_cause', 'g_lift_ruler_rejected', 'g_lift_ruler_samples',
     'g_lift_ruler_phase', 'g_lift_ruler_after_raw', 'g_lift_ruler_before_raw',
     'g_lift_ruler_target_raw', 'g_lift_compare_operation', 'g_lift_compare_top_valid',
     'g_lift_compare_backoff_valid', 'g_lift_compare_safe_raw', 'g_lift_compare_target_pulses',
     'g_it_state', 'g_it_cause', 'g_it_request', 'g_it_lift_permit', 'g_it_action',
     'g_startup_axes_state', 'g_extension_home_reference_valid', 'g_extension_hold_confirmed',
     'g_dm_jog_state', 'g_dm_jog_result', 'g_dm_posvel_holding', 'g_vm_test_request',
     'g_safety_ready', 'g_safety_fault', 'g_rtos_fault', 'g_can_ctrl_fault_latched']
F = ['g_vehicle_lift_current_deg']


def main():
    tag = sys.argv[1] if len(sys.argv) > 1 else 'height-check-01'
    assert hashlib.sha256(IMAGE.read_bytes()).hexdigest() == EXPECTED
    transport.live.IMAGE = IMAGE
    n, segs = transport.live.load_image()
    opt = {'target_override': 'cortex_m', 'connect_mode': 'attach', 'frequency': 1000000,
           'auto_unlock': False, 'resume_on_disconnect': False, 'no_config': True,
           'cmsis_dap.prefer_v1': False, 'cmsis_dap.limit_packets': True}
    s = ConnectHelper.session_with_chosen_probe(unique_id='6afaae24', options=opt)
    rep = {'tag': tag, 'writes': [], 'image_sha256': EXPECTED}

    def g(k):
        return t.read32(n[k])

    def put(k, v):
        t.write32(n[k], v)
        rep['writes'].append([k, v])

    def snap():
        d = {k: g(k) for k in W}
        for k in F:
            d[k] = struct.unpack('<f', struct.pack('<I', g(k)))[0]
        d['claw'] = t.read32(0x40000034)
        d['PE2'] = bool(t.read32(0x40021014) & 4)
        return d

    O.mkdir(parents=True, exist_ok=True)
    with (O / (tag + '-samples.jsonl')).open('x', encoding='utf-8') as log:
        s.open()
        t = s.target
        try:
            for a, b in segs:
                for k in range(0, len(b), 4096):
                    assert bytes(t.read_memory_block8(a + k, len(b[k:k + 4096]))) == b[k:k + 4096]
            d = snap()
            rep['before'] = d
            if d['g_lift_ruler_state'] not in (4,) or d['g_lift_ruler_cause'] or d['g_lift_compare_top_valid'] != 1:
                t.reset(reset_type=Target.ResetType.SYSRESETREQ)
                rep['reset'] = True
                time.sleep(3)
                put('g_it_request', 1)
                b0 = time.monotonic()
                while time.monotonic() - b0 < 100:
                    d = snap()
                    if d['g_it_state'] == 2 and d['g_it_action'] == 1:
                        break
                    assert not d['g_lift_ruler_cause'] and not d['g_safety_fault'], d
                    time.sleep(.02)
                rep['home_ms'] = round((time.monotonic() - b0) * 1000)
            b0 = time.monotonic()
            while time.monotonic() - b0 < 30:
                d = snap()
                if d['g_startup_axes_state'] == 4 and d['g_extension_home_reference_valid'] == 1 \
                        and d['g_extension_hold_confirmed'] == 1:
                    break
                if d['g_startup_axes_state'] == 0 and d['g_it_state'] != 1:
                    put('g_it_request', 2)
                time.sleep(.05)
            if g('g_lift_compare_target_pulses') != TARGET:
                put('g_lift_compare_target_pulses', TARGET)
            old = g('g_lift_ruler_samples')
            started = False
            b0 = time.monotonic()
            while time.monotonic() - b0 < 10 and not started:
                put('g_it_lift_permit', 1)
                put('g_lift_ruler_request', 1)
                t1 = time.monotonic()
                while time.monotonic() - t1 < 1.0:
                    dd = snap()
                    assert not dd['g_lift_ruler_cause'] and not dd['g_safety_fault'], dd
                    if dd['g_lift_ruler_state'] != 4 or dd['g_lift_ruler_samples'] != old:
                        started = True
                        break
                    time.sleep(.03)
            assert started, 'descent rejected'
            t0 = time.monotonic()
            while time.monotonic() - t0 < 35:
                d = snap()
                d['host_phase'] = 'down'
                log.write(json.dumps(d) + '\n')
                log.flush()
                assert not d['g_lift_ruler_cause'], d['g_lift_ruler_cause']
                if d['g_lift_ruler_state'] == 4 and d['g_lift_compare_operation'] == 2 \
                        and d['g_lift_ruler_samples'] != old:
                    break
                time.sleep(.02)
            put('g_it_lift_permit', 0)
            rep['down_ms'] = round((time.monotonic() - t0) * 1000)
            rep['after'] = d = snap()
            raw = d['g_lift_ruler_after_raw']
            rep['result'] = {'after_raw': raw, 'pulses': round(raw / RAWPP, 1),
                             'target_raw': d['g_lift_ruler_target_raw'],
                             'before_raw': d['g_lift_ruler_before_raw'],
                             'travel_mm': round((raw - d['g_lift_ruler_before_raw']) / RAWPP / 1600.0, 2),
                             'abs_from_top_mm': round(raw / RAWPP / 1600.0, 2)}
            rep['holding_at_depth'] = True
            print('HOLD ' + json.dumps(rep['result']), flush=True)
        except Exception as e:
            rep['error'] = repr(e)
            print('ERR ' + repr(e), flush=True)
        finally:
            s.close()
            (O / (tag + '-result.json')).write_text(json.dumps(rep, indent=2), encoding='utf-8')
            print(json.dumps({k: v for k, v in rep.items() if k not in ['before', 'after']}), flush=True)
    return 0 if rep.get('holding_at_depth') else 1


if __name__ == '__main__':
    raise SystemExit(main())
