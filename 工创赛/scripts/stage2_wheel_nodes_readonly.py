import sys
from pathlib import Path
import json, datetime
sys.path.insert(0, r"D:\Codex\工创赛\scripts")
import source_stage2_readonly_sync_probe_20261105 as transport
from pyocd.core.helpers import ConnectHelper
R = Path(r"D:\Codex\工创赛")
transport.live.IMAGE = R / "tmp/integration12_stage2_vision_20261107/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf"
names, _ = transport.live.load_image()
opt = {"target_override":"cortex_m","connect_mode":"attach","frequency":1000000,
       "auto_unlock":False,"resume_on_disconnect":False,"no_config":True,
       "cmsis_dap.prefer_v1":False,"cmsis_dap.limit_packets":True}
s = ConnectHelper.session_with_chosen_probe(unique_id="6afaae24", options=opt)
s.open(); t = s.target
def r32(n):
    if n not in names: return None
    v = t.read32(names[n])
    return v - (1<<32) if v >= (1<<31) else v
def arr(n, k=4):
    if n not in names: return None
    base = names[n]; out=[]
    for i in range(k):
        v = t.read32(base + 4*i)
        out.append(v - (1<<32) if v >= (1<<31) else v)
    return out
out = {"when": datetime.datetime.now().isoformat(timespec="seconds"), "kind":"readonly-wheel-nodes"}
for k in ("g_safety_ready","g_safety_fault","g_vm_test_request","g_can_test_result",
          "g_can_poll_enable","g_can_poll_valid_mask","g_can_poll_protect_mask",
          "g_can_guard_ready","g_can_guard_bad_nodes","g_can_guard_cause",
          "g_can_ctrl_fault_latched","g_can_task_count","g_src_now_ms"):
    out[k] = r32(k)
out["poll_result"] = arr("g_can_poll_result")
out["poll_count"] = arr("g_can_poll_count")
out["poll_flags"] = arr("g_can_poll_flags")
out["poll_age"] = arr("g_can_poll_age_ticks")
out["poll_tick"] = arr("g_can_poll_tick")
print(json.dumps(out, indent=1, ensure_ascii=False))
ev = R / "evidence/integration/2026-11-07/three-piece-i12/wheel-nodes-readonly.json"
ev.write_text(json.dumps(out, indent=1, ensure_ascii=False), encoding="utf-8")
print("evidence=" + str(ev))
s.close()
