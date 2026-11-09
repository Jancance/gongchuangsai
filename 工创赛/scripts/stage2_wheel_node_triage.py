"""Wheel-node communication triage, read-only by default.

用途：一次上板就把“四轮节点到底有没有通信响应”定性，区分三种情况：
  A. 从未轮询（count 全 0）        -> VM/门禁没放行，不是硬件问题
  B. 轮询了但超时（result=5）      -> 该节点真不回帧
  C. 轮询成功（result=1, mask=0xF）-> 通信正常

默认只读，不写任何变量、不发帧、不动作。
用法：
  python scripts/stage2_wheel_node_triage.py --check   # 只校验本地绑定，不开 USB
  python scripts/stage2_wheel_node_triage.py           # 只读快照
"""
import sys, json, datetime, argparse
from pathlib import Path

R = Path(r"D:\Codex\工创赛")
IMAGE = R / "tmp/integration12_stage2_vision_20261107/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf"
EVID = R / "evidence/integration/2026-11-07/three-piece-i12"

RESULT_MEANING = {0: "not polled yet", 1: "ok", 2: "?", 3: "?", 4: "tx mailbox full",
                  5: "CAN reply timeout", 6: "decode error", 7: "address rejected",
                  8: "ack not 02", 9: "refused by guard/profile"}


def check_only():
    ok = True
    print("IMAGE exists:", IMAGE.exists(), IMAGE)
    ok &= IMAGE.exists()
    for n in ("source_stage2_readonly_sync_probe_20261105.py",):
        p = R / "scripts" / n
        print("transport  exists:", p.exists(), p)
        ok &= p.exists()
    src = R / "tmp/integration12_stage2_vision_20261107/project/CHASSIS/USER/stage2_site_activate.inc"
    print("site glue  exists:", src.exists(), src)
    ok &= src.exists()
    print("RESULT:", "OK" if ok else "MISSING")
    return 0 if ok else 1


def read_only():
    sys.path.insert(0, str(R / "scripts"))
    import source_stage2_readonly_sync_probe_20261105 as transport
    from pyocd.core.helpers import ConnectHelper
    transport.live.IMAGE = IMAGE
    names, _ = transport.live.load_image()
    opt = {"target_override": "cortex_m", "connect_mode": "attach", "frequency": 1000000,
           "auto_unlock": False, "resume_on_disconnect": False, "no_config": True,
           "cmsis_dap.prefer_v1": False, "cmsis_dap.limit_packets": True}
    s = ConnectHelper.session_with_chosen_probe(unique_id="6afaae24", options=opt)
    s.open()
    t = s.target

    def r32(n):
        if n not in names:
            return None
        v = t.read32(names[n])
        return v - (1 << 32) if v >= (1 << 31) else v

    def arr(n, k=4):
        if n not in names:
            return None
        base = names[n]
        out = []
        for i in range(k):
            v = t.read32(base + 4 * i)
            out.append(v - (1 << 32) if v >= (1 << 31) else v)
        return out

    out = {"when": datetime.datetime.now().isoformat(timespec="seconds"),
           "kind": "wheel-node-triage-readonly", "image": str(IMAGE.name)}
    scalars = ("g_safety_ready", "g_safety_fault", "g_vm_test_request", "g_can_test_result",
               "g_can_poll_enable", "g_can_poll_valid_mask", "g_can_poll_protect_mask",
               "g_can_guard_ready", "g_can_guard_bad_nodes", "g_can_guard_cause",
               "g_can_ctrl_fault_latched", "g_can_task_count",
               "g_stage2_site_state", "g_stage2_site_reason")
    for k in scalars:
        out[k] = r32(k)
    out["poll_result"] = arr("g_can_poll_result")
    out["poll_count"] = arr("g_can_poll_count")
    out["poll_flags"] = arr("g_can_poll_flags")
    out["poll_age"] = arr("g_can_poll_age_ticks")
    s.close()

    print(json.dumps(out, indent=1, ensure_ascii=False))
    res = out["poll_result"] or [0, 0, 0, 0]
    cnt = out["poll_count"] or [0, 0, 0, 0]
    print("\n--- verdict per node (1..4) ---")
    never = all((c or 0) == 0 for c in cnt)
    for i in range(4):
        r = res[i] if i < len(res) else None
        print("node%d result=%s (%s) count=%s" % (i + 1, r, RESULT_MEANING.get(r, "?"), cnt[i]))
    if never:
        print("VERDICT: NEVER POLLED -> gate/VM release problem, not a motor fault")
    elif all(r == 1 for r in res):
        print("VERDICT: ALL NODES OK (communication healthy)")
    else:
        bad = [i + 1 for i, r in enumerate(res) if r != 1]
        print("VERDICT: nodes %s not OK -> see result codes above" % bad)
    EVID.mkdir(parents=True, exist_ok=True)
    ev = EVID / "wheel-node-triage.json"
    ev.write_text(json.dumps(out, indent=1, ensure_ascii=False), encoding="utf-8")
    print("evidence=%s" % ev)
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true", help="local binding check only, no USB")
    a = ap.parse_args()
    sys.exit(check_only() if a.check else read_only())
