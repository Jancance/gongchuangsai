# -*- coding: utf-8 -*-
"""Pick table calibrator (offline).

把“4310转轴到盘心距离 D”和“伸缩=0 时转轴到夹爪长度 L0”两个实测值，
换算成三行抓取表，并检查伸缩是否在行程内。

规则几何（R）: SRC-01 盘径300 -> 半径150; SRC-02 物料中心圆直径200 -> R=100, 120度等分
推导(E)      : 相邻中心距 100*sqrt(3)=173.205mm

用法:
  python scripts/pick_table_calibrator_20261108.py --check      # 自检
  python scripts/pick_table_calibrator_20261108.py --D 200 --L0 180 --stroke 70 --angle0 180
"""
import argparse, math, json, datetime
from pathlib import Path

R_MATERIAL = 100.0      # SRC-02: 物料中心轨道半径 mm
DEPTH_MM = 50.0         # 已实测: 118800 脉冲处夹爪底面 50mm


def arm_angle_offset(D):
    """左右物料相对正前方的偏角(度)。偏移方向: 物料在盘心 +-120 度,
    投影到转轴坐标系后 atan(86.6/(D+50))。"""
    return math.degrees(math.atan2(R_MATERIAL * math.sin(math.radians(120.0)),
                                   D + R_MATERIAL * math.cos(math.radians(120.0)) * -1.0 * -1.0))


def compute(D, L0, stroke_max, angle0=180.0):
    """返回三行表 + 诊断。角度定义: angle0 为正前方(下部物料)。"""
    # 以转轴为原点, 正前方为 +y; 盘心 (0, D)
    px, py = 0.0, D
    # 下部物料: 从盘心朝车方向 R
    near = (px, py - R_MATERIAL)
    # 左右: 相对下部方向 +-120 度
    offs = []
    for s in (+1.0, -1.0):
        a = math.radians(-90.0 + s * 120.0)
        offs.append((px + R_MATERIAL * math.cos(a), py + R_MATERIAL * math.sin(a)))

    def dist(p):
        return math.hypot(p[0], p[1])

    def ang(p):
        # +y 为 angle0, 顺时针为正 -> theta = angle0 - atan2(x, y)
        return angle0 - math.degrees(math.atan2(p[0], p[1]))

    rows = []
    for name, p in (("near", near), ("sideA", offs[0]), ("sideB", offs[1])):
        d = dist(p)
        rows.append({
            "region": name,
            "dist_from_axis_mm": round(d, 2),
            "arm_deg": round(ang(p), 2),
            "extension_mm": round(d - L0, 2),
            "reachable": (d - L0) <= stroke_max + 1e-6,
        })
    diag = {
        "D_mm": D, "L0_mm": L0, "stroke_max_mm": stroke_max, "angle0_deg": angle0,
        "near_gap_mm": round(dist(near) - L0, 2),
        "side_extra_extension_mm": round(dist(offs[0]) - dist(near), 2),
        "side_angle_offset_deg": round(abs(ang(offs[0]) - angle0), 2),
    }
    return rows, diag


def self_check():
    print("=== 自检: D=200, L0=180, stroke=70 ===")
    rows, diag = compute(200.0, 180.0, 70.0)
    for r in rows:
        print("  %-6s dist=%7.2f  arm=%7.2f  ext=%7.2f  reachable=%s"
              % (r["region"], r["dist_from_axis_mm"], r["arm_deg"], r["extension_mm"], r["reachable"]))
    print("  diag:", json.dumps(diag, ensure_ascii=False))
    print()
    print("=== 关键判定表: 左右相对下部要多伸多少 (与 L0/D 无关的部分) ===")
    print("  D(mm) | 下部距轴 | 左右距轴 | 左右需多伸 | 左右偏角")
    for D in (150, 200, 250, 300, 350):
        _, d = compute(float(D), 0.0, 1e9)
        near = d["near_gap_mm"]
        print("  %5d | %8.2f | %8.2f | %10.2f | %6.2f deg"
              % (D, near, near + d["side_extra_extension_mm"], d["side_extra_extension_mm"],
                 d["side_angle_offset_deg"]))
    print()
    print("结论: 左右比下部多伸的量随 D 增大而收敛到约 150mm; D 越小反而越大.")
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--D", type=float, help="4310 转轴到盘心水平距离 mm (实测)")
    ap.add_argument("--L0", type=float, help="伸缩=0 时转轴到夹爪长度 mm (实测)")
    ap.add_argument("--stroke", type=float, default=70.0, help="伸缩可用行程 mm")
    ap.add_argument("--angle0", type=float, default=180.0, help="正前方对应的 4310 角")
    a = ap.parse_args()
    if a.check or a.D is None or a.L0 is None:
        return self_check()
    rows, diag = compute(a.D, a.L0, a.stroke, a.angle0)
    out = {"when": datetime.datetime.now().isoformat(timespec="seconds"),
           "kind": "pick-table-calibration", "diag": diag, "rows": rows}
    print(json.dumps(out, indent=1, ensure_ascii=False))
    bad = [r["region"] for r in rows if not r["reachable"]]
    print()
    if bad:
        print("VERDICT: 不可达 -> %s 超出伸缩行程 %.0fmm" % (bad, a.stroke))
        print("  含义: 左/右物料不能靠当前伸缩直接够到, 需要底盘横移或多停位.")
    else:
        print("VERDICT: 三行全部可达, 可以按固定动作跑.")
    ev = Path(r"D:\Codex\工创赛\evidence\integration\2026-11-07\stage2-mainline\pick-table-calibration.json")
    ev.parent.mkdir(parents=True, exist_ok=True)
    ev.write_text(json.dumps(out, indent=1, ensure_ascii=False), encoding="utf-8")
    print("evidence=%s" % ev)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
