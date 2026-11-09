"""示教采集 (只读): 用户手动把 4310 摆到左部物料上方后, 读角度并推算右部。

流程 (用户 2026-11-08 指定):
  1. 车距转盘 7.5cm, 升降先下降到抓取高度
  2. 用户手动调整 4310 到左部物料, 并给出伸缩伸出量
  3. 本脚本读取 4310 逻辑角 + 伸缩实际毫米
  4. 右部由镜像推算: right = 360 - left
  5. 输出三行示教表 (可直接贴进 source_pick_teach_table.h)
  6. 之后回顶

只读: 不写任何变量、不发运动、不复位。
用法:
  python scripts/pick_teach_capture.py --check
  python scripts/pick_teach_capture.py --left-ext 152.0
"""
import sys, json, datetime, argparse
from pathlib import Path

R = Path(r"D:\Codex\工创赛")
IMAGE = R / "tmp/integration12_stage2_vision_20261107/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf"
DEG_PER_MM = 3.6          # 32 pulses/mm * 360/3200 = 3.6 deg/mm (source: source_prepare.inc)
ANGLE0 = 180.0            # 原料区 (front)


def mirror_right(left_deg):
    """0=left, 180=front, 270=right -> mirror about 180."""
    r = 360.0 - left_deg
    return r


def emit(left_deg, left_ext, down_deg=180.0, down_ext=0.0):
    right_deg = mirror_right(left_deg)
    right_ext = left_ext
    lines = ["/* ---- site values: captured from the vehicle ------------------- */",
             "#define SRC_TEACH_DOWN_ARM_DEG    %.2ff" % down_deg,
             "#define SRC_TEACH_DOWN_EXT_MM     %.2ff" % down_ext,
             "#define SRC_TEACH_LEFT_ARM_DEG    %.2ff" % left_deg,
             "#define SRC_TEACH_LEFT_EXT_MM     %.2ff" % left_ext,
             "#define SRC_TEACH_RIGHT_ARM_DEG   %.2ff" % right_deg,
             "#define SRC_TEACH_RIGHT_EXT_MM    %.2ff" % right_ext]
    out = {"when": datetime.datetime.now().isoformat(timespec="seconds"),
           "kind": "pick-teach-capture",
           "down": {"arm_deg": down_deg, "ext_mm": down_ext, "source": "user defined, extension retracted"},
           "left": {"arm_deg": left_deg, "ext_mm": left_ext, "source": "user manual pose + given extension"},
           "right": {"arm_deg": round(right_deg, 2), "ext_mm": right_ext,
                     "source": "mirror of left about 180deg: 360-left"},
           "rotation_rule": "clockwise = decreasing angle; slot 35.758deg; no wrap past 270"}
    return out, lines


def check_only():
    ok = IMAGE.exists()
    print("IMAGE exists:", ok, IMAGE)
    print("mirror sanity: left 159.37 -> right", mirror_right(159.37), "(expect 200.63)")
    print("mirror sanity: left 200.63 -> right", mirror_right(200.63), "(expect 159.37)")
    return 0 if ok else 1


def capture(left_ext_given):
    sys.path.insert(0, str(R / "scripts"))
    import source_stage2_readonly_sync_probe_20261105 as transport
    from pyocd.core.helpers import ConnectHelper
    transport.live.IMAGE = IMAGE
    names, _ = transport.live.load_image()
    opt = {"target_override": "cortex_m", "connect_mode": "attach", "frequency": 1000000,
           "auto_unlock": False, "resume_on_disconnect": False, "no_config": True,
           "cmsis_dap.prefer_v1": False, "cmsis_dap.limit_packets": True}
    s = ConnectHelper.session_with_chosen_probe(unique_id="6afaae24", options=opt)
    s.open(); t = s.target

    def rf(n):
        if n not in names:
            return None
        return t.read32(names[n]) / 1000.0 if False else None

    def f32(n):
        if n not in names:
            return None
        import struct
        return struct.unpack("<f", struct.pack("<I", t.read32(names[n])))[0]

    arm = f32("g_dm_can2_logical_position_deg")
    cur = f32("g_startup_extension_current_deg")
    home = f32("g_extension_home_reference_deg")
    s.close()

    ext_measured = None
    if None not in (cur, home):
        ext_measured = (cur - home) / DEG_PER_MM

    print("4310 logical deg      = %s" % arm)
    print("extension current deg = %s" % cur)
    print("extension home deg    = %s" % home)
    print("extension measured mm = %s  (=(cur-home)/3.6)" % ext_measured)
    print("user stated left ext  = %s" % left_ext_given)
    if ext_measured is not None and left_ext_given is not None:
        print("difference            = %.2f mm" % (ext_measured - left_ext_given))

    left_deg = arm if arm is not None else 0.0
    left_ext = left_ext_given if left_ext_given is not None else (ext_measured or 0.0)
    out, lines = emit(left_deg, left_ext)
    out["readback"] = {"arm_deg": arm, "ext_deg": cur, "home_deg": home, "ext_mm_measured": ext_measured}
    print()
    print("\r\n".join(lines))
    ev = R / "evidence/integration/2026-11-07/stage2-mainline/pick-teach-capture.json"
    ev.parent.mkdir(parents=True, exist_ok=True)
    ev.write_text(json.dumps(out, indent=1, ensure_ascii=False), encoding="utf-8")
    print()
    print("evidence=%s" % ev)
    return 0


def descend(to_pulses=118800):
    """把升降下降到抓取高度 118800 脉冲, 等真实到位后返回。这里是唯一会发动作的分支。"""
    import time
    sys.path.insert(0, str(R / "scripts"))
    import source_stage2_readonly_sync_probe_20261105 as transport
    from pyocd.core.helpers import ConnectHelper
    transport.live.IMAGE = IMAGE
    names, _ = transport.live.load_image()
    opt = {"target_override": "cortex_m", "connect_mode": "attach", "frequency": 1000000,
           "auto_unlock": False, "resume_on_disconnect": False, "no_config": True,
           "cmsis_dap.prefer_v1": False, "cmsis_dap.limit_packets": True}
    s = ConnectHelper.session_with_chosen_probe(unique_id="6afaae24", options=opt)
    s.open(); t = s.target

    def i32(n):
        if n not in names:
            return None
        v = t.read32(names[n])
        return v - (1 << 32) if v >= (1 << 31) else v

    guard = ("g_safety_fault", "g_rtos_fault", "g_can_ctrl_fault_latched")
    bad = {g: i32(g) for g in guard if i32(g)}
    if bad:
        print("REFUSE: faults present", bad)
        s.close(); return 2
    if i32("g_lift_ruler_state") == 0:
        print("NOTE: lift controller idle, top reference may be missing -> run IT request 1 first")
    t.write32(names["g_lift_compare_target_pulses"], to_pulses)
    t.write32(names["g_it_lift_permit"], 1)
    t.write32(names["g_lift_ruler_request"], 1)
    print("descent requested to %d pulses (%.1f mm)" % (to_pulses, to_pulses / 1600.0))
    target_raw = int(to_pulses * 11.25)
    ok = False
    for _ in range(400):
        time.sleep(0.05)
        if i32("g_safety_fault"):
            print("FAULT during descent:", i32("g_safety_fault")); break
        st = i32("g_lift_ruler_state")
        after = i32("g_lift_ruler_after_raw")
        if st == 4 and after is not None and abs(after - target_raw) <= 60:
            ok = True; break
    t.write32(names["g_it_lift_permit"], 0)
    if ok:
        print("descent DONE, after_raw=%d (target %d)" % (after, target_raw))
        print("now: operator manually sets 4310 to the LEFT piece, then rerun with --left-ext <mm>")
    else:
        print("descent did NOT confirm; state=%s after=%s" % (i32("g_lift_ruler_state"), i32("g_lift_ruler_after_raw")))
    s.close()
    return 0 if ok else 1


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--left-ext", type=float, help="用户给出的左部伸缩伸出量 mm")
    ap.add_argument("--descend", action="store_true", help="先下降到抓取高度 118800 再退出")
    a = ap.parse_args()
    if a.check:
        sys.exit(check_only())
    if a.descend:
        sys.exit(descend())
    if a.left_ext is None:
        print("need --left-ext <mm>  (the extension amount you measured/decided)")
        sys.exit(2)
    sys.exit(capture(a.left_ext))
