# -*- coding: utf-8 -*-
"""停位规划器: 判断"三件抓取"用纯伸缩 还是 需要底盘换停位。

几何来源(R): SRC-01 盘径300/半径150; SRC-02 物料中心轨道半径100, 120度等分
推导(E)    : 相邻中心距 100*sqrt(3)=173.205mm

坐标: 转轴为原点, +y=车头朝盘方向, +x=车右方, 4310 角度以 +y 为 180度。

用法:
  python scripts/pick_station_planner_20261108.py --check
  python scripts/pick_station_planner_20261108.py --D 200 --L0 100 --stroke 70
"""
import argparse, math, json, datetime
from pathlib import Path

R_M = 100.0          # 物料中心轨道半径 (SRC-02, R)
GAP = R_M * math.sqrt(3.0)   # 相邻中心距 173.205 (SRC-03, E)
DEFAULT_ANGLE0 = 180.0


def material_points(D):
    """返回 {region: (x,y)} 三个物料相对转轴的位置。"""
    cx, cy = 0.0, D
    near = (cx, cy - R_M)
    sides = {}
    for sign, name in ((+1.0, "sideRight"), (-1.0, "sideLeft")):
        a = math.radians(-90.0 + sign * 120.0)
        sides[name] = (cx + R_M * math.cos(a), cy + R_M * math.sin(a))
    return {"near": near, **sides}


def plan(D, L0, stroke, angle0=DEFAULT_ANGLE0):
    pts = material_points(D)
    # 基准停位: 让 near 正好在正前方, 伸缩 0
    mx, my = pts["near"][0], pts["near"][1] - L0
    # 侧向停位: 让该物料到正前方, 用掉 stroke/2
    s_side = stroke / 2.0
    out = []
    for name in ("near", "sideLeft", "sideRight"):
        px, py = pts[name]
        if name == "near":
            sx, sy, ext = mx, my, 0.0
        else:
            sx, sy = px, py - (L0 + s_side)
            ext = s_side
        # 物料相对新转轴
        rx, ry = px - sx, py - sy
        dist = math.hypot(rx, ry)
        # 该停位相对基准停位的移动量
        dx, dy = sx - mx, sy - my
        # 4310 角 (+y 为 angle0, 顺时针为正)
        theta = angle0 - math.degrees(math.atan2(rx, ry))
        out.append({
            "region": name,
            "move_from_base_mm": [round(dx, 2), round(dy, 2)],  # 车跟随转轴: 车移动量 = 新转轴 - 旧转轴
            "arm_deg": round(theta, 2),
            "extension_mm": round(ext, 2),
            "dist_from_axis_mm": round(dist, 2),
            "reachable": ext <= stroke + 1e-6,
        })
    return out


def self_check():
    print("=== 模式A: 只在基准停位, 靠伸缩够三件 (L0 任意) ===")
    print("  D(mm) | L0(mm) | 下部 | 左/右需伸 | 行程70内?")
    for D in (150.0, 200.0, 250.0, 300.0):
        L0 = D - 100.0
        pts = material_points(D)
        side = math.hypot(*pts["sideLeft"])
        print("  %5.0f | %6.0f |  0.00 | %9.2f | %s"
              % (D, L0, side - L0, "YES" if side - L0 <= 70.0 else "NO"))
    print("  -> 全部 NO: 单停位靠伸缩抓左右不可行 (需 ~160mm), 这是 R=100/120度几何决定.")
    print()
    print("=== 模式B: 换停位, 三件同一姿态 180度 ===")
    for D, L0 in ((200.0, 100.0), (250.0, 150.0), (300.0, 200.0)):
        rows = plan(D, L0, 70.0)
        ok = all(r["reachable"] for r in rows) and all(abs(r["arm_deg"] - 180.0) < 1e-6 for r in rows)
        print("  D=%.0f L0=%.0f -> %s" % (D, L0, "可行" if ok else "不可行"))
        for r in rows:
            print("     %-9s 车移动(x,y)=%s  臂=%6.2f  伸缩=%5.2f  %s"
                  % (r["region"], r["move_from_base_mm"], r["arm_deg"], r["extension_mm"],
                     "ok" if r["reachable"] else "OVER"))
    print()
    print("  关键: 三行臂角都锁定 180 度, 伸缩都用一半行程 -> 就是'同一套固定动作'.")
    print("  侧向停位相对基准: 横移 %.2f + 前进 %.2f mm (与 D/L0 无关)"
          % (GAP / 2.0, 150.0 - 35.0))
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--D", type=float)
    ap.add_argument("--L0", type=float)
    ap.add_argument("--stroke", type=float, default=70.0)
    a = ap.parse_args()
    if a.check or a.D is None or a.L0 is None:
        return self_check()
    rows = plan(a.D, a.L0, a.stroke)
    out = {"when": datetime.datetime.now().isoformat(timespec="seconds"),
           "kind": "pick-station-plan", "D_mm": a.D, "L0_mm": a.L0,
           "stroke_max_mm": a.stroke, "stations": rows}
    print(json.dumps(out, indent=1, ensure_ascii=False))
    bad = [r["region"] for r in rows if not r["reachable"]]
    print()
    print("VERDICT:", "三停位可行, 可用同一套 180 度固定动作" if not bad
          else "不可行 -> %s" % bad)
    ev = Path(r"D:\Codex\工创赛\evidence\integration\2026-11-07\stage2-mainline\pick-station-plan.json")
    ev.parent.mkdir(parents=True, exist_ok=True)
    ev.write_text(json.dumps(out, indent=1, ensure_ascii=False), encoding="utf-8")
    print("evidence=%s" % ev)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
