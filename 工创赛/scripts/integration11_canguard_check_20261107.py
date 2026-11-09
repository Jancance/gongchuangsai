"""只读：CAN 节点健康/轮询掩码。不动任何轴、不开VM。"""
from pathlib import Path
import json, time

import source_stage2_readonly_sync_probe_20261105 as transport
from pyocd.core.helpers import ConnectHelper

R = Path(__file__).resolve().parents[1]
IMAGE = R / 'tmp/integration11_short_20261107/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf'
KEYS = ['g_can_guard_ready', 'g_can_guard_bad_nodes', 'g_can_guard_cause',
        'g_can_poll_enable', 'g_can_poll_count', 'g_can_poll_valid_mask', 'g_can_poll_protect_mask',
        'g_can_poll_result', 'g_can_poll_reply', 'g_can_poll_flags', 'g_can_poll_esr',
        'g_can_poll_age_ticks', 'g_can_poll_tick', 'g_can_test_result', 'g_safety_ready',
        'g_safety_fault', 'g_vm_test_request']


def dump(t, n):
    return {k: (t.read32(n[k]) if k in n else None) for k in KEYS}


def main():
    transport.live.IMAGE = IMAGE
    n, _ = transport.live.load_image()
    opt = {'target_override': 'cortex_m', 'connect_mode': 'attach', 'frequency': 1000000,
           'auto_unlock': False, 'resume_on_disconnect': False, 'no_config': True,
           'cmsis_dap.prefer_v1': False, 'cmsis_dap.limit_packets': True}
    s = ConnectHelper.session_with_chosen_probe(unique_id='6afaae24', options=opt)
    out = {}
    with s:
        t = s.target
        out['vm_off'] = dump(t, n)
        t.write32(n['g_lift_ruler_controller'] + 4, 1)
        begin = time.monotonic()
        samples = []
        while time.monotonic() - begin < 12:
            d = dump(t, n)
            d['t_ms'] = round((time.monotonic() - begin) * 1000)
            samples.append(d)
            if d['g_can_guard_ready'] == 1:
                break
            time.sleep(.25)
        out['with_vm'] = samples[-1]
        out['vm_up_ms'] = samples[-1]['t_ms']
        out['samples'] = [{k: s2[k] for k in ['t_ms', 'g_can_guard_ready', 'g_can_guard_bad_nodes',
                                              'g_can_guard_cause', 'g_can_poll_valid_mask',
                                              'g_can_poll_result', 'g_can_poll_reply', 'g_can_poll_age_ticks']}
                          for s2 in samples]
        t.write32(n['g_lift_ruler_controller'] + 4, 0)
        t.write32(n['g_startup_axes_vm_request'], 0)
        time.sleep(1)
        out['vm_off_after'] = dump(t, n)
    print(json.dumps(out, indent=1, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
