"""单件/三件抓取序列 (第2项离线实现).

示教表由用户现场给出: 每个区域一组 (4310角, 伸缩mm)。
其余参数全部来自已实测记录, 不改。

安全顺序 (源码约束 integration10_complete.inc:323):
  IT3 不允许同时转臂和伸缩 -> 先转臂(伸缩在零位), 再伸缩伸出。
  反过来做会让伸出的臂画大圆弧扫过料盘。

用法:
  python scripts/pick_sequence_20261108.py --check
  python scripts/pick_sequence_20261108.py --plan --down 180,0 --left 159.4,150 --right 200.6,150
  python scripts/pick_sequence_20261108.py --run --tag seq-01 --order down,left,right \
      --down 180,0 --left 159.4,150 --right 200.6,150
"""
import sys, os, json, time, struct, argparse, datetime
from pathlib import Path

R = Path(r"D:\Codex\工创赛")
IMAGE = R / "tmp/integration12_stage2_vision_20261107/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf"
EVID = R / "evidence/integration/2026-11-07/stage2-mainline"
PROBE = "6afaae24"

TARGET_PULSES = 118800
RAW_PER_PULSE = 11.25
TOTAL_RAW = int(TARGET_PULSES * RAW_PER_PULSE)
SLOT_ARM_DEG = 35.758
CARRIER_DEG = {"slot1": 0.0, "slot2": 120.0, "slot3": 240.0}
PWM_OPEN, PWM_PREGRIp, PWM_GRIP = 2056, 1685, 1537
ROTATE_AT_MS = 1400
GRIP_SETTLE_S = 0.30
DEG_PER_MM = 3.6

FAULT_KEYS = ("g_safety_fault", "g_rtos_fault", "g_can_ctrl_fault_latched",
              "g_it_cause")


def parse_teach(text, name):
    try:
        a, e = text.split(",")
        return {"arm": float(a), "ext": float(e)}
    except Exception:
        raise SystemExit("--%s 需要 '角度,伸缩mm'" % name)


def validate(table):
    errs = []
    for k, v in table.items():
        if not (0.0 <= v["arm"] <= 270.0):
            errs.append("%s arm %.2f out of 0..270" % (k, v["arm"]))
        if not (0.0 <= v["ext"] <= 170.0):
            errs.append("%s ext %.2f out of 0..170" % (k, v["ext"]))
    if table["left"]["ext"] > 0.01 and table["right"]["ext"] > 0.01:
        if abs(table["left"]["ext"] - table["right"]["ext"]) > 5.0:
            errs.append("left/right ext differ by >5mm (should be symmetric)")
    return errs


def plan_print(table, order):
    print("=== 执行计划 (安全串行版) ===")
    total = 0.0
    for i, region in enumerate(order, 1):
        t = table[region]
        print("第%d件 region=%-5s 4310=%.2fdeg ext=%.1fmm -> slot%d" % (i, region, t["arm"], t["ext"], i))
        print("   1) 转臂 %.2f -> %.2f  (伸缩在零位, 约 %.2fs)"
              % (SLOT_ARM_DEG if i > 1 else table[order[i-2] if i > 1 else region]["arm"],
                 t["arm"], abs(t["arm"] - 180.0) / 300.0 + 0.9))
        print("   2) 伸缩 0 -> %.1fmm (速度未实测, 不计入估算)" % t["ext"])
        print("   3) 下降 118800 (2.05s, 藏载盘转位 + 夹爪预收)")
        print("   4) 夹紧 140 (0.40s)")
        if t["ext"] <= 0.05:
            print("   5) 回升 (3.35s), 1400ms 后并行转臂到 %.3f  [parallel]" % SLOT_ARM_DEG)
        else:
            print("   5) 回升到顶 (3.35s) -> 收伸缩 %.1f->0 -> 转臂到 %.3f  [serial, 慢]"
                  % (t["ext"], SLOT_ARM_DEG))
        print("   6) 松爪 210 (0.13s)")
    print()
    print("固定部分合计/件 = %.2fs (不含转臂/伸缩), 三件 %.1fs"
          % (2.05 + 0.40 + 3.35 + 0.13, 3 * (2.05 + 0.40 + 3.35 + 0.13)))
    print("转臂/伸缩时间取决于实测速度, 本脚本不估算、执行时按实际记录")
    return 0


def run(table, order, tag):
    sys.path.insert(0, str(R / "scripts"))
    import source_stage2_readonly_sync_probe_20261105 as transport
    from pyocd.core.helpers import ConnectHelper
    transport.live.IMAGE = IMAGE
    n, segs = transport.live.load_image()
    opt = {"target_override": "cortex_m", "connect_mode": "attach", "frequency": 1000000,
           "auto_unlock": False, "resume_on_disconnect": False, "no_config": True,
           "cmsis_dap.prefer_v1": False, "cmsis_dap.limit_packets": True}
    session = ConnectHelper.session_with_chosen_probe(unique_id=PROBE, options=opt)
    session.open()
    t = session.target
    assert t.get_state() == t.State.RUNNING

    W32 = ["g_it_request", "g_it_state", "g_it_cause", "g_it_rejected", "g_it_action",
           "g_it_axes", "g_it_lift_permit", "g_it_elapsed_ms",
           "g_lift_compare_target_pulses", "g_lift_compare_top_valid",
           "g_lift_compare_backoff_valid", "g_lift_compare_safe_raw",
           "g_lift_ruler_state", "g_lift_ruler_request", "g_lift_ruler_after_raw",
           "g_lift_ruler_samples", "g_lift_ruler_cause",
           "g_vm_test_request", "g_safety_ready", "g_safety_fault", "g_rtos_fault",
           "g_can_guard_ready", "g_can_ctrl_fault_latched",
           "g_dm_jog_state", "g_dm_jog_result", "g_dm_posvel_holding", "g_dm_point_enable",
           "g_vehicle_lift_feedback_valid"]
    F32 = ["g_it_dm_deg", "g_it_extension_mm", "g_it_lift_pulses", "g_it_carrier_deg",
           "g_dm_can2_logical_position_deg", "g_vehicle_lift_current_deg",
           "g_startup_extension_current_deg", "g_extension_home_reference_deg"]
    miss = [k for k in W32 + F32 if k not in n]
    assert not miss, miss

    def get(k): return t.read32(n[k])
    def getf(k): return struct.unpack("<f", struct.pack("<I", t.read32(n[k])))[0]
    def put(k, v): t.write32(n[k], int(v))
    def putf(k, v): t.write32(n[k], struct.unpack("<I", struct.pack("<f", float(v)))[0])
    def pwm(v): t.write32(0x40000034, int(v))
    def ramp(to, ms, steps):
        s0 = t.read32(0x40000034)
        for i in range(1, steps + 1):
            pwm(s0 + (to - s0) * (1.0 * i / steps)); time.sleep(ms / 1000.0 / steps)
        pwm(to)
    def faults():
        return {k: get(k) for k in FAULT_KEYS if get(k)}

    rep = {"tag": tag, "when": datetime.datetime.now().isoformat(timespec="seconds"),
           "image": str(IMAGE.name), "table": table, "order": order, "pieces": []}
    EVID.mkdir(parents=True, exist_ok=True)
    log = (EVID / (tag + "-samples.jsonl")).open("w", encoding="utf-8")

    def snap():
        d = {k: get(k) for k in W32}
        d.update({k: getf(k) for k in F32})
        d["claw_pwm_us"] = t.read32(0x40000034)
        return d

    def wait_it(action, phase, timeout=100):
        begin = time.monotonic(); seen = False
        while time.monotonic() - begin < timeout:
            d = snap(); d["host_phase"] = phase; log.write(json.dumps(d) + "\n")
            f = faults()
            if f: raise RuntimeError("%s fault %s" % (phase, f))
            if d["g_it_state"] == 1 and d["g_it_action"] == action: seen = True
            if seen and d["g_it_state"] == 2 and d["g_it_action"] == action: return d
            if d["g_it_state"] == 9:
                raise RuntimeError("%s it_fail cause=%s rejected=%s"
                                   % (phase, d["g_it_cause"], d["g_it_rejected"]))
            time.sleep(.02)
        raise RuntimeError(phase + " timeout")

    def wait_lift(phase, timeout=40):
        begin = time.monotonic(); old = get("g_lift_ruler_samples")
        while time.monotonic() - begin < timeout:
            d = snap(); d["host_phase"] = phase; log.write(json.dumps(d) + "\n")
            f = faults()
            if f: raise RuntimeError("%s fault %s" % (phase, f))
            if d["g_lift_ruler_state"] == 4 and d["g_lift_ruler_samples"] != old:
                return d
            time.sleep(.02)
        raise RuntimeError(phase + " timeout")

    def wait_dm(phase, deg, timeout=12):
        begin = time.monotonic()
        while time.monotonic() - begin < timeout:
            d = snap(); d["host_phase"] = phase; log.write(json.dumps(d) + "\n")
            f = faults()
            if f: raise RuntimeError("%s fault %s" % (phase, f))
            if (d["g_dm_jog_state"] == 7 and d["g_dm_posvel_holding"] == 1
                    and abs(d["g_dm_can2_logical_position_deg"] - deg) <= 3.0):
                return d
            if d["g_dm_jog_state"] == 8:
                raise RuntimeError("%s dm fail result=%s" % (phase, d["g_dm_jog_result"]))
            time.sleep(.02)
        raise RuntimeError(phase + " timeout")

    def ext_mm():
        return (getf("g_startup_extension_current_deg")
                - getf("g_extension_home_reference_deg")) / DEG_PER_MM

    def it_arm(deg, phase):
        """IT3 axes=1: 只转臂, 要求伸缩在零位."""
        put("g_it_axes", 1); putf("g_it_dm_deg", deg)
        putf("g_it_extension_mm", 0.0); putf("g_it_lift_pulses", 0.0)
        put("g_it_request", 3)
        return wait_it(3, phase, 30)

    def it_ext(mm, phase):
        """IT3 axes=2: 只动伸缩."""
        put("g_it_axes", 2); putf("g_it_extension_mm", mm)
        putf("g_it_dm_deg", getf("g_dm_can2_logical_position_deg")); putf("g_it_lift_pulses", 0.0)
        put("g_it_request", 3)
        return wait_it(3, phase, 40)

    def fire_dm(deg):
        cur = getf("g_dm_can2_logical_position_deg")
        req = 4 if deg >= cur else 5
        putf("g_dm_pick_target_deg", deg); putf("g_dm_place_target_deg", deg)
        putf("g_dm_posvel_speed_deg_s", 300.0); put("g_dm_point_enable", 1)
        put("g_dm_parallel_group_active", 1); put("g_dm_parallel_group_go", 1)
        put("g_dm_jog_request", req)
        return req

    d0 = snap()
    f = faults()
    if f:
        print("REFUSE: faults present", f); return 2
    if d0["g_safety_ready"] != 1:
        print("WARN: safety_ready != 1, guard may not be established")

    for idx, region in enumerate(order, 1):
        tp = table[region]
        slot = "slot%d" % idx
        rec = {"piece": idx, "region": region, "arm_deg": tp["arm"], "ext_mm": tp["ext"], "slot": slot}
        t_piece = time.monotonic()

        # 1) 转臂 (伸缩在零位)
        t0 = time.monotonic()
        it_arm(tp["arm"], "p%d_arm" % idx)
        rec["arm_ms"] = round((time.monotonic() - t0) * 1000)
        rec["arm_reached"] = round(getf("g_dm_can2_logical_position_deg"), 3)

        # 2) 伸缩伸出
        if tp["ext"] > 0.05:
            t0 = time.monotonic()
            it_ext(tp["ext"], "p%d_ext" % idx)
            rec["ext_ms"] = round((time.monotonic() - t0) * 1000)
            rec["ext_reached"] = round(ext_mm(), 2)

        # 3) 下降 (载盘转位 + 夹爪预收 藏在这里)
        t0 = time.monotonic()
        put("g_lift_compare_target_pulses", TARGET_PULSES)
        put("g_it_lift_permit", 1)
        put("g_lift_ruler_request", 1)
        time.sleep(0.10)
        putf("g_it_carrier_deg", CARRIER_DEG[slot]); put("g_it_request", 10)
        ramp(PWM_PREGRIp, 420, 14)
        d = wait_lift("p%d_down" % idx, 40)
        put("g_it_lift_permit", 0)
        rec["down_ms"] = round((time.monotonic() - t0) * 1000)
        rec["down_after_raw"] = d["g_lift_ruler_after_raw"]
        assert abs(d["g_lift_ruler_after_raw"] - TOTAL_RAW) <= 40, d["g_lift_ruler_after_raw"]

        # 4) 夹紧
        t0 = time.monotonic()
        ramp(PWM_GRIP, 80, 8); time.sleep(GRIP_SETTLE_S)
        rec["grip_ms"] = round((time.monotonic() - t0) * 1000)

        # 5) 回升. 伸缩已伸出时必须先收再转臂, 否则带料扫盘;
        #    伸缩在零位时才能把转臂藏进回升 (5.9s 节拍的前提).
        t0 = time.monotonic()
        put("g_it_request", 4)
        if tp["ext"] <= 0.05:
            time.sleep(ROTATE_AT_MS / 1000.0)
            fire_dm(SLOT_ARM_DEG)
            wait_dm("p%d_slot" % idx, SLOT_ARM_DEG)
            wait_it(4, "p%d_rise" % idx, 90)
            rec["path"] = "parallel_rotate"
        else:
            wait_it(4, "p%d_rise" % idx, 90)          # 先回升到顶
            it_ext(0.0, "p%d_retract" % idx)          # 收伸缩回零
            it_arm(SLOT_ARM_DEG, "p%d_slot" % idx)    # 再转臂到车载槽
            rec["path"] = "retract_then_rotate"
        rec["rise_ms"] = round((time.monotonic() - t0) * 1000)

        # 7) 松爪
        t0 = time.monotonic()
        ramp(PWM_OPEN, 120, 8); put("g_dm_parallel_group_active", 0); put("g_dm_parallel_group_go", 0)
        rec["release_ms"] = round((time.monotonic() - t0) * 1000)
        rec["piece_ms"] = round((time.monotonic() - t_piece) * 1000)
        rep["pieces"].append(rec)
        print("piece %d (%s) done in %d ms" % (idx, region, rec["piece_ms"]), flush=True)

    rep["final"] = snap()
    log.close()
    ev = EVID / (tag + "-result.json")
    ev.write_text(json.dumps(rep, indent=1, ensure_ascii=False), encoding="utf-8")
    print("evidence=%s" % ev)
    session.close()
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--plan", action="store_true")
    ap.add_argument("--run", action="store_true")
    ap.add_argument("--tag", default="seq-01")
    ap.add_argument("--order", default="down,left,right")
    ap.add_argument("--down"); ap.add_argument("--left"); ap.add_argument("--right")
    a = ap.parse_args()

    if a.check:
        ok = IMAGE.exists()
        print("IMAGE exists:", ok, IMAGE)
        print("PWM open/pregrip/grip =", PWM_OPEN, PWM_PREGRIp, PWM_GRIP)
        print("depth pulses =", TARGET_PULSES, "raw =", TOTAL_RAW)
        print("slot arm deg =", SLOT_ARM_DEG, "carrier =", CARRIER_DEG)
        return 0 if ok else 1

    if not (a.down and a.left and a.right):
        print("need --down/--left/--right '角度,伸缩mm'")
        return 2
    table = {"down": parse_teach(a.down, "down"),
             "left": parse_teach(a.left, "left"),
             "right": parse_teach(a.right, "right")}
    errs = validate(table)
    if errs:
        print("TABLE INVALID:")
        for e in errs:
            print("  -", e)
        return 2
    order = [x.strip() for x in a.order.split(",") if x.strip()]
    if len(set(order)) != 3 or any(x not in table for x in order):
        print("bad --order, need a permutation of down/left/right")
        return 2
    if a.plan or not a.run:
        return plan_print(table, order)
    return run(table, order, a.tag)


if __name__ == "__main__":
    raise SystemExit(main())
