# -*- coding: utf-8 -*-
"""把实测的 6 个数直接变成抓取表 (不做任何几何推算)。

输入全部来自卷尺实测, 不使用 R=100/120 度 假设, 不依赖 D/L0 候选值。

用法:
  python scripts/pick_measure_input.py --check
  python scripts/pick_measure_input.py --l0 161 --ext-full 331 \
      --near 180,0 --left 341,159.4 --right 341,200.6
  每个区域格式: 距离mm,相对正前方角度deg (左正右负 或 按实际方向, 正数=逆时针)
"""
import argparse, json, datetime, math
from pathlib import Path

STROKE_RECORDED = 170.0   # 有记录的总行程, 只作对照, 以 --ext-full 实测为准
CLEAR_MIN = 50.0          # MECH-06
ARM_LIMIT = 270.0         # integration10_complete.inc:314
ANGLE0 = 180.0            # 正前方对应 4310 逻辑角


def parse_region(text, name):
    try:
        d, a = text.split(",")
        return {"name": name, "dist_mm": float(d), "offset_deg": float(a)}
    except Exception:
        raise SystemExit("bad --%s format, need 'dist,deg'" % name)


def build(l0, ext_full, regions):
    stroke_measured = ext_full - l0
    rows = []
    for r in regions:
        ext = r["dist_mm"] - l0
        arm = ANGLE0 + r["offset_deg"]
        rows.append({
            "region": r["name"],
            "dist_measured_mm": round(r["dist_mm"], 2),
            "offset_deg": round(r["offset_deg"], 2),
            "extension_mm": round(ext, 2),
            "arm_deg": round(arm, 2),
            "ext_ok": -0.5 <= ext <= stroke_measured + 1e-6,
            "arm_ok": -1e-6 <= arm <= ARM_LIMIT + 1e-6,
        })
    return {
        "l0_mm": l0, "ext_full_mm": ext_full,
        "stroke_measured_mm": round(stroke_measured, 2),
        "stroke_recorded_mm": STROKE_RECORDED,
        "stroke_matches_record": abs(stroke_measured - STROKE_RECORDED) <= 5.0,
        "rows": rows,
        "all_ok": all(x["ext_ok"] and x["arm_ok"] for x in rows),
    }


def emit_c(out):
    lines = ["/* generated from measured values, do not hand-edit numbers */",
             "#define SRC_PICK_L0_MM          %.1ff" % out["l0_mm"],
             "#define SRC_PICK_STROKE_MM      %.1ff" % out["stroke_measured_mm"]]
    for r in out["rows"]:
        tag = "NEAR" if r["region"] == "near" else ("LEFT" if r["region"] == "left" else "RIGHT")
        lines.append("#define SRC_PICK_%s_ARM_DEG   %.2ff" % (tag, r["arm_deg"]))
        lines.append("#define SRC_PICK_%s_EXT_MM    %.2ff" % (tag, max(0.0, r["extension_mm"])))
        lines.append("#define SRC_PICK_%s_DIST_MM   %.2ff" % (tag, r["dist_measured_mm"]))
    return "\r\n".join(lines)


def self_check():
    print("=== 自检: 用旧候选值模拟 (只验证脚本算术, 不验证机械) ===")
    out = build(161.0, 331.0, [
        {"name": "near", "dist_mm": 180.0, "offset_deg": 0.0},
        {"name": "left", "dist_mm": 341.17, "offset_deg": -20.63},
        {"name": "right", "dist_mm": 341.17, "offset_deg": 20.63},
    ])
    print(json.dumps(out, indent=1, ensure_ascii=False))
    print()
    print("--- 生成 C 表 ---")
    print(emit_c(out))
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--l0", type=float, help="伸缩=0 时转轴到夹爪中心 mm")
    ap.add_argument("--ext-full", type=float, help="伸缩全伸时转轴到夹爪中心 mm")
    ap.add_argument("--near", help="下部物料: 距离,角度")
    ap.add_argument("--left", help="左部物料: 距离,角度")
    ap.add_argument("--right", help="右部物料: 距离,角度")
    a = ap.parse_args()
    if a.check:
        return self_check()
    need = {"--l0": a.l0, "--ext-full": a.ext_full, "--near": a.near, "--left": a.left, "--right": a.right}
    miss = [k for k, v in need.items() if v is None]
    if miss:
        print("missing:", " ".join(miss))
        print("usage: --l0 161 --ext-full 331 --near 180,0 --left 341,-20.6 --right 341,20.6")
        return 2
    regions = [parse_region(a.near, "near"), parse_region(a.left, "left"), parse_region(a.right, "right")]
    out = build(a.l0, a.ext_full, regions)
    out["when"] = datetime.datetime.now().isoformat(timespec="seconds")
    print(json.dumps(out, indent=1, ensure_ascii=False))
    print()
    print("--- 生成 C 表 ---")
    print(emit_c(out))
    print()
    bad = [r["region"] for r in out["rows"] if not (r["ext_ok"] and r["arm_ok"])]
    if bad:
        print("VERDICT: 不可达 ->", bad, "(看 extension_mm 是否超出 stroke_measured_mm)")
    else:
        print("VERDICT: 三个区域全部可达, 可直接按表执行")
    if not out["stroke_matches_record"]:
        print("注意: 实测行程 %.1fmm 与记录 170mm 差超过 5mm, 请复核测量" % out["stroke_measured_mm"])
    ev = Path(r"D:\Codex\工创赛\evidence\integration\2026-11-07\stage2-mainline\pick-measured-table.json")
    ev.parent.mkdir(parents=True, exist_ok=True)
    ev.write_text(json.dumps(out, indent=1, ensure_ascii=False), encoding="utf-8")
    print("evidence=%s" % ev)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
