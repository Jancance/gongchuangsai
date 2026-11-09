# -*- coding: utf-8 -*-
"""三方案判定: 纯伸缩 / 换停位 / 等盘转。全部离线。

约束来源:
  R  SRC-01 盘径300(半径150)  SRC-02 物料中心轨道半径100, 120度等分
  R  原料盘匀速 6-10s/圈, 停位允许误差
  E  MECH-06 车体外缘到原料盘外缘水平最短净距 >= 50mm
  E  SRC-03 相邻中心距 100*sqrt(3)=173.205mm

用法:
  python scripts/pick_three_plans_20261108.py --check
  python scripts/pick_three_plans_20261108.py --L0 180 --stroke 70 --clear 50
"""
import argparse, math, json, datetime
from pathlib import Path

R_M = 100.0
GAP = R_M * math.sqrt(3.0)      # 173.205
DISC_R = 150.0
MIN_CLEAR = 50.0                # MECH-06
HALF_STROKE_SIDE = 35.0         # 侧向用半行程


def plan_A(D, L0, stroke):
    """单停位, 靠伸缩够三件."""
    near = D - R_M
    side = math.sqrt(D * D + R_M * R_M + D * R_M)
    return {
        "plan": "A_纯伸缩_单停位",
        "near_extension_mm": round(near - L0, 2),
        "side_extension_mm": round(side - L0, 2),
        "stroke_max_mm": stroke,
        "feasible": (side - L0) <= stroke + 1e-6 and (near - L0) >= -1e-6,
    }


def plan_B(clear_base, stroke):
    """换停位: 三件同一 180 度姿态, 车横移+前进."""
    # 侧向停位: 车前进 115 -> 车头到盘心 y 分量 = (clear+150) - 115 = clear+35
    side_center = math.hypot(GAP / 2.0, clear_base + 35.0)
    side_clear = side_center - DISC_R
    return {
        "plan": "B_换停位_同一姿态",
        "move_side_mm": [round(GAP / 2.0, 2), round(150.0 - HALF_STROKE_SIDE, 2)],
        "base_clear_mm": round(clear_base, 2),
        "side_clear_mm": round(side_clear, 2),
        "side_extension_mm": HALF_STROKE_SIDE,
        "feasible": side_clear >= MIN_CLEAR,
        "needed_base_clear_mm": round(math.sqrt(max(0.0, (DISC_R + MIN_CLEAR) ** 2 - (GAP / 2.0) ** 2)) - 35.0, 2),
    }


def plan_C(turn_speed_deg_s_low=36.0, turn_speed_deg_s_high=60.0, dwell_s=3.0):
    """等盘把物料转到 down 区, 三件同一动作."""
    def t120(v):
        return 120.0 / v
    lo, hi = t120(turn_speed_deg_s_high), t120(turn_speed_deg_s_low)
    return {
        "plan": "C_等盘转_单一动作",
        "one_step_120deg_s": [round(lo, 2), round(hi, 2)],
        "dwell_s": dwell_s,
        "worst_per_piece_s": round(hi + dwell_s, 2),
        "three_pieces_extra_s": [round(2 * (lo + dwell_s), 2), round(2 * (hi + dwell_s), 2)],
        "note": "盘转向随机且停位有误差, 必须每次重新识别区域; 不能靠几何推算",
        "feasible": True,
    }


def self_check():
    L0, stroke, clear = 180.0, 70.0, 50.0
    print("=== 假设 L0=%.0fmm, 行程=%.0fmm, 基准净距=%.0fmm ===" % (L0, stroke, clear))
    print()
    for D in (250.0, 300.0, 350.0):
        a = plan_A(D, L0, stroke)
        print("方案A D=%.0f: 下部需伸 %.2f, 左右需伸 %.2f -> %s"
              % (D, a["near_extension_mm"], a["side_extension_mm"],
                 "可行" if a["feasible"] else "不可行(超行程)"))
    print()
    b = plan_B(clear, stroke)
    print("方案B: 车移动 %s mm, 侧向净距 %.2fmm -> %s"
          % (b["move_side_mm"], b["side_clear_mm"], "可行" if b["feasible"] else "撞盘"))
    print("        基准位至少需要净距 %.2fmm 才安全" % b["needed_base_clear_mm"])
    for p in (50.0, 100.0, 145.0, 160.0):
        bb = plan_B(p, stroke)
        print("        基准净距 %.0f -> 侧向净距 %.1f %s"
              % (p, bb["side_clear_mm"], "OK" if bb["feasible"] else "撞盘"))
    print()
    c = plan_C()
    print("方案C: 每转 120 度需 %s s, 停 %s s -> 单件最坏 %.2fs"
          % (c["one_step_120deg_s"], c["dwell_s"], c["worst_per_piece_s"]))
    print("        三件额外等待 %s s" % (c["three_pieces_extra_s"],))
    print()
    print("注: 方案C 已有视觉 left/right/down 三区域识别, 不需要几何推算.")
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--L0", type=float, default=180.0)
    ap.add_argument("--stroke", type=float, default=70.0)
    ap.add_argument("--clear", type=float, default=50.0)
    ap.add_argument("--D", type=float, default=300.0)
    a = ap.parse_args()
    if a.check:
        return self_check()
    out = {"when": datetime.datetime.now().isoformat(timespec="seconds"),
           "kind": "pick-three-plans",
           "A": plan_A(a.D, a.L0, a.stroke),
           "B": plan_B(a.clear, a.stroke),
           "C": plan_C()}
    print(json.dumps(out, indent=1, ensure_ascii=False))
    ev = Path(r"D:\Codex\工创赛\evidence\integration\2026-11-07\stage2-mainline\pick-three-plans.json")
    ev.parent.mkdir(parents=True, exist_ok=True)
    ev.write_text(json.dumps(out, indent=1, ensure_ascii=False), encoding="utf-8")
    print("evidence=%s" % ev)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
