"""Offline geometry only: no serial/probe, motor commands, firmware export or flashing."""
import argparse
import copy
import hashlib
import html
import json
import math
from pathlib import Path
import re

from p1_route_preview import USER, candidate_profile, read_geometry

ROOT = Path(__file__).resolve().parents[2]
DEFAULT = ROOT / "工创赛/config/p1_perimeter_validation.json"
KINDS = {"FORWARD", "BACKWARD", "LEFT", "RIGHT", "TURN"}
COLORS = {"FORWARD": "#1763c0", "BACKWARD": "#8b40af", "LEFT": "#00866c", "RIGHT": "#c97400", "TURN": "#a44723"}


def wrap(deg):
    return (deg + 180) % 360 - 180


def finite(value, name):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f"{name}必须是有限数字")
    return float(value)


def corners(pose, length, width):
    a = math.radians(pose[2])
    return [(pose[0]+x*math.cos(a)-y*math.sin(a), pose[1]+x*math.sin(a)+y*math.cos(a))
            for x, y in [(length/2, width/2), (length/2, -width/2), (-length/2, -width/2), (-length/2, width/2)]]


def critical_turn_angles(start, delta, length, width):
    """Exact extrema of rectangle X/Y support over the whole angular interval."""
    lo, hi = sorted((start, start+delta))
    values = {lo, hi}
    for base in [math.degrees(math.atan2(width, length)), math.degrees(math.atan2(length, width))]:
        for sign in [-1, 1]:
            for k in range(math.floor(lo/180)-2, math.ceil(hi/180)+3):
                angle = sign*base+180*k
                if lo <= angle <= hi:
                    values.add(angle)
    return sorted(values)


def boundary_clearance(poses, length, width, field):
    points = [p for pose in poses for p in corners(pose, length, width)]
    return dict(left_mm=min(p[0] for p in points), right_mm=field[0]-max(p[0] for p in points),
                bottom_mm=min(p[1] for p in points), top_mm=field[1]-max(p[1] for p in points))


def evaluate(config):
    c = copy.deepcopy(config)
    if c.get("schema") != "p1-perimeter-preview-v1":
        raise ValueError("配置版本不匹配")
    field = c["field"]
    # Inner area permission is explicit and local to this test. No implicit bypass.
    if field.get("inner_regions_allowed_for_trial") is not True:
        raise ValueError("本预览只检查外边界；未明确允许经过内区时不得宣称路线合规")
    fw, fh = [finite(field[k], k) for k in ["width_mm", "height_mm"]]
    length, width = [finite(c["footprint"][k], k) for k in ["length_mm", "width_mm"]]
    margin = finite(c["required_boundary_margin_mm"], "margin")
    if min(fw, fh, length, width) <= 0 or min(fw, fh) > 100000 or margin < 0:
        raise ValueError("场地、外廓或余量无效")
    if max(length, width) > min(fw, fh):
        raise ValueError("外廓大于场地")
    pose = [finite(c["start_pose"][k], k) for k in ["x_mm", "y_mm", "heading_deg"]]
    if abs(pose[2]) > 180:
        raise ValueError("起始航向必须在±180度内")
    start = pose[:]
    rows = []
    geometry = read_geometry(USER / "chassis_kinematics.c")
    if not c.get("actions") or len(c["actions"]) > 64:
        raise ValueError("预览动作数必须为1～64，非固件容量许可")
    for index, action in enumerate(c["actions"], 1):
        kind = action["kind"]
        if kind not in KINDS:
            raise ValueError("未知动作")
        amount, speed = finite(action["amount"], "amount"), finite(action["speed"], "speed")
        before = pose[:]
        row = dict(index=index, **action, start_pose=before)
        if kind == "TURN":
            if not 0 < abs(amount) <= 90 or not 0 < speed <= 30:
                raise ValueError("单次转角必须≤90度，参考转速≤30度/s")
            swept = [[pose[0], pose[1], h] for h in critical_turn_angles(pose[2], amount, length, width)]
            pose[2] = wrap(pose[2]+amount)
            row.update(nominal_lower_time_s=abs(amount)/speed, profile=None)
        else:
            if not 10 <= amount <= 210 or not 1.21 <= speed <= 20:
                raise ValueError("预览平移范围10～210cm，参考速度1.21～20cm/s")
            distance = amount*10
            bx, by = {"FORWARD": (distance, 0), "BACKWARD": (-distance, 0), "LEFT": (0, distance), "RIGHT": (0, -distance)}[kind]
            a = math.radians(pose[2])
            pose[0] += bx*math.cos(a)-by*math.sin(a)
            pose[1] += bx*math.sin(a)+by*math.cos(a)
            # Constant heading + straight translation: support extrema are at endpoints.
            swept = [before, pose[:]]
            profile_kind = "FORWARD" if kind == "BACKWARD" else kind
            profile = candidate_profile(profile_kind, amount, speed, geometry)
            if profile is None:
                raise ValueError("当前标量S曲线模型不能规划该平移")
            row.update(profile=profile, backward_model_is_unvalidated=(kind == "BACKWARD"),
                       nominal_lower_time_s=amount/speed)
        clearance = boundary_clearance(swept, length, width, (fw, fh))
        minimum = min(clearance.values())
        row.update(end_pose=pose[:], boundary_clearance_mm=clearance,
                   minimum_boundary_clearance_mm=minimum,
                   boundary_pass=minimum+1e-7 >= margin)
        rows.append(row)
    delta = [pose[0]-start[0], pose[1]-start[1], wrap(pose[2]-start[2])]
    zone = [finite(v, "start_zone") for v in field["start_zone_mm"]]
    if len(zone) != 4 or not 0 <= zone[0] < zone[2] <= fw or not 0 <= zone[1] < zone[3] <= fh:
        raise ValueError("启停区范围无效")
    zone_clearance = boundary_clearance([[pose[0]-zone[0], pose[1]-zone[1], pose[2]]], length, width, (zone[2]-zone[0], zone[3]-zone[1]))
    initial_zone_clearance = boundary_clearance([[start[0]-zone[0], start[1]-zone[1], start[2]]], length, width, (zone[2]-zone[0], zone[3]-zone[1]))
    closed = math.hypot(*delta[:2]) < 1e-5 and abs(delta[2]) < 1e-6
    border_ok = all(r["boundary_pass"] for r in rows)
    capacity = re.search(r"#define\s+MOTION_ACTION_CAPACITY\s+(\d+)U", (USER/"motion_action_plan.h").read_bytes().decode("gbk"))
    if not capacity:
        raise ValueError("无法读取当前固件动作容量")
    capacity = int(capacity[1])
    budget = 5000+sum(33500 if r["kind"] == "TURN" else r["profile"]["run_ms"]+6000 for r in rows)
    return dict(schema=c["schema"], hardware_accessed=False, configuration=c, rows=rows,
                start_pose=start, end_pose=pose, closure_error_mm_deg=delta, closed=closed,
                initial_footprint_inside_start_zone=min(initial_zone_clearance.values()) >= 0,
                boundary_geometry_pass=border_ok, returned_footprint_inside_start_zone=min(zone_clearance.values()) >= 0,
                minimum_boundary_clearance_mm=min(r["minimum_boundary_clearance_mm"] for r in rows),
                travel_cm=sum(r["amount"] for r in rows if r["kind"] != "TURN"),
                turn_absolute_deg=sum(abs(r["amount"]) for r in rows if r["kind"] == "TURN"),
                translation_profile_reference_s=sum(r["profile"]["plan_duration_s"] for r in rows if r["profile"]),
                turn_time_lower_bound_s=sum(r["nominal_lower_time_s"] for r in rows if r["kind"] == "TURN"),
                current_style_conservative_budget_ms=budget, current_firmware_capacity=capacity,
                firmware_executable=False, vehicle_approved=False,
                blockers=[f"本路线{len(rows)}动作，当前固件容量{capacity}", "尚无后退距离执行接口",
                          "20cm/s平移与30度/s单段转向候选尚未统一为组合路线",
                          "现有120秒整组预算不支持该路线，需要分段调度与逐段截止设计",
                          "轮组估算不是整车最大外廓实测，初始与回区余量很小",
                          "30档转向及整场位置反馈覆盖尚未实车验证"],
                limits="外边界几何只对配置中的矩形外廓成立；内区本轮允许经过，不代表正式比赛路线合规。时间为标量参考与下限，非实车预计完成时间。")


def render_svg(report):
    c = report["configuration"]
    scale, ox, oy = .30, 55, 115
    height = c["field"]["height_mm"]
    def xy(x, y):
        return ox+x*scale, oy+(height-y)*scale
    def rect(x, y, w, h, fill, extra=""):
        sx, sy = xy(x, y+h)
        return f'<rect x="{sx}" y="{sy}" width="{w*scale}" height="{h*scale}" fill="{fill}" {extra}/>'
    def label(x, y, text, size=16, fill="#263746", extra=""):
        return f'<text x="{x}" y="{y}" font-size="{size}" fill="{fill}" {extra}>{html.escape(text)}</text>'
    result = ['<svg xmlns="http://www.w3.org/2000/svg" width="1380" height="940" viewBox="0 0 1380 940">',
              '<defs><marker id="arrow" markerWidth="8" markerHeight="8" refX="7" refY="3" orient="auto"><path d="M0,0 L7,3 L0,6" fill="#243e5b"/></marker></defs>',
              '<rect width="1380" height="940" fill="#f7f9fc"/>',
              '<g font-family="Microsoft YaHei,Arial,sans-serif">',
              label(55, 40, "外围闭合验证：两段前进 + 长距离左右侧移", 25),
              label(55, 73, "仅离线预览 · 内区本轮允许经过 · 外边界保留 · 未准备发车", 17, "#a04d17"),
              rect(0, 0, 2400, 2400, "#e3e6eb", 'stroke="#374b60" stroke-width="2"')]
    for x in [550, 1400]:
        for y in [550, 1400]:
            result.append(rect(x, y, 450, 450, "#fcf5c8"))
    result += [rect(0, 910, 150, 580, "white"), rect(910, 0, 580, 150, "white"),
               rect(2100, 2100, 300, 300, "#9ac2f7"), rect(2100, 0, 300, 300, "#c6daf4")]
    for x, y, text in [(2180, 2340, "启停1"), (2180, 160, "启停2"), (35, 1240, "暂存"), (1040, 75, "粗加工")]:
        px, py = xy(x, y); result.append(label(px, py, text, 13))
    result.append(label(55, 872, "背景功能区为示意；本轮不据此做内区碰撞判定。坐标：左下角为原点，单位mm。", 15))
    for row in report["rows"]:
        start, end = row["start_pose"], row["end_pose"]
        x1, y1 = xy(*start[:2]); x2, y2 = xy(*end[:2])
        color = COLORS[row["kind"]]
        if row["kind"] != "TURN":
            result.append(f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{color}" stroke-width="3" marker-end="url(#arrow)"/>')
        else:
            radius = 17+(row["index"]%2)*8
            result.append(f'<circle cx="{x1}" cy="{y1}" r="{radius}" fill="none" stroke="{color}" stroke-width="2" stroke-dasharray="5 4"/>')
        if row["kind"] != "TURN" and row["amount"] > 100:
            result.append(label((x1+x2)/2+8, (y1+y2)/2-8, f'{row["index"]} · {row["amount"]:g}cm', 15, color))
    for text, pose, dx, dy in [("起点 / 终点，车头向下", report["start_pose"], -210, -17),
                               ("5、6：顺时针90°×2", report["rows"][4]["start_pose"], 20, 38),
                               ("8、9：逆时针90°×2", report["rows"][7]["start_pose"], 20, -24)]:
        x, y = xy(*pose[:2]); result.append(label(x+dx, y+dy, text, 14))
    vertices = " ".join(f"{xy(x,y)[0]:.2f},{xy(x,y)[1]:.2f}" for x,y in corners(report["start_pose"], c["footprint"]["length_mm"], c["footprint"]["width_mm"]))
    result.append(f'<polygon id="car-body" points="{vertices}" fill="#e8f4ff" fill-opacity="0.8" stroke="#152b45" stroke-width="2"/>')
    def heading_line(pose):
        x,y=xy(*pose[:2]);a=math.radians(pose[2]);xe,ye=xy(pose[0]+100*math.cos(a),pose[1]+100*math.sin(a));return x,y,xe,ye
    x,y,xe,ye=heading_line(report["start_pose"])
    result.append(f'<line id="car-heading" x1="{x}" y1="{y}" x2="{xe}" y2="{ye}" stroke="#152b45" stroke-width="3" marker-end="url(#arrow)"/>')
    result.append(label(825, 123, f'{len(report["rows"])}个动作，逐段停稳后继续', 20))
    for row in report["rows"]:
        result.append(label(825, 160+row["index"]*31, f'{row["index"]:02d}  {row["label"]}', 16, COLORS[row["kind"]]))
    for j, text in enumerate([f'总平移 {report["travel_cm"]/100:g}m；累计转角 {report["turn_absolute_deg"]:g}°',
                              "几何闭合：位置及车头朝向相同", "估算轮组包络：283×254mm",
                              f'外边界最小理论余量：{report["minimum_boundary_clearance_mm"]:.1f}mm',
                              "完整整车外廓未确认；余量不是定位误差预算", "后退/组合高速/整圈调度尚未接入固件"]):
        result.append(label(825, 610+j*32, text, 16, "#374b60" if j < 2 else "#a04d17"))
    result.append('</g></svg>')
    svg = "\n".join(result)
    poses = [report["start_pose"]]+[r["end_pose"] for r in report["rows"]]
    frames = []
    for pose in poses:
        frames.append(dict(pose=pose, points=" ".join(f"{xy(x,y)[0]:.2f},{xy(x,y)[1]:.2f}" for x,y in corners(pose,c["footprint"]["length_mm"],c["footprint"]["width_mm"])), arrow=heading_line(pose)))
    return svg, frames


def write_preview(config_path, out):
    config_path = Path(config_path).resolve(); out = Path(out).resolve()
    if out.exists():
        raise ValueError("输出目录已存在；请选择新目录保留旧证据")
    config = json.loads(config_path.read_text(encoding="utf-8"))
    report = evaluate(config)
    report["configuration_sha256"] = hashlib.sha256(config_path.read_bytes()).hexdigest()
    out.mkdir(parents=True)
    (out/"report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    svg, frames = render_svg(report)
    (out/"route.svg").write_text(svg, encoding="utf-8")
    page = '''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>外围路线预览</title>
<style>body{margin:0;background:#f7f9fc;font-family:'Microsoft YaHei',sans-serif}svg{display:block;width:100%;max-width:1380px}footer{padding:12px 55px;background:#fff;position:sticky;bottom:0}input{width:350px;vertical-align:middle}span{margin-left:20px}</style>
'''+svg+'''<footer><label>动作终点：<input id="step" type="range" min="0" max="STEPS" value="0"></label><span id="value"></span></footer><script>
const frames=FRAMES;const slider=document.getElementById('step');
function draw(){const f=frames[+slider.value];document.getElementById('car-body').setAttribute('points',f.points);
['x1','y1','x2','y2'].forEach((k,i)=>document.getElementById('car-heading').setAttribute(k,f.arrow[i]));
document.getElementById('value').textContent=`步骤 ${slider.value}：X=${f.pose[0].toFixed(1)}mm，Y=${f.pose[1].toFixed(1)}mm，航向=${f.pose[2].toFixed(1)}°`;}slider.addEventListener('input',draw);draw();
</script></html>'''.replace("FRAMES", json.dumps(frames)).replace("STEPS", str(len(report["rows"])))
    (out/"route.html").write_text(page, encoding="utf-8")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    report = write_preview(args.config, args.out)
    print(json.dumps({k: report[k] for k in ["closed", "boundary_geometry_pass", "minimum_boundary_clearance_mm", "travel_cm", "end_pose", "firmware_executable", "current_style_conservative_budget_ms"]}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
