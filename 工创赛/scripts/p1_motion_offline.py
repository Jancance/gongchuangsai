"""Offline P1 log analysis. No serial, firmware write, reset or download API."""
import argparse
import ast
import csv
from datetime import datetime
import hashlib
import html
import json
import math
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[2]
MASK = 0xFFFFFFFF
LIMITS = [
    "TM wheel feedback is cached and asynchronous; base RPM is not four sent commands.",
    "BT1 is retained application-level communication history, not mechanical response.",
    "Encoder progress is not ground travel; recorded reference position is unavailable.",
    "Missing samples are unknown, not zero. Clock/buffer boundaries break chart lines.",
    "Decoded JSONL is used as captured; this tool does not independently recheck wire CRC.",
]


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8-sig"))


def save_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def safe_output(path, inputs=()):
    """Outputs must be new, and outside live firmware and frozen releases."""
    path = Path(path).resolve()
    for forbidden in [ROOT / "3.stm32f407工程", ROOT / "工创赛/releases", *map(Path, inputs)]:
        forbidden = forbidden.resolve()
        if path == forbidden or forbidden in path.parents:
            raise ValueError(f"output overlaps protected input: {forbidden}")
    path.mkdir(parents=True, exist_ok=False)
    return path


def number(record, key, scale=1):
    value = record.get(key)
    if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) and value != -2147483648:
        return value / scale
    return None


def host_seconds(record):
    return datetime.fromisoformat(record["host_utc"].replace("Z", "+00:00")).timestamp()


def tm_record(record):
    return record.get("direction") == "RX" and record.get("frame_type", "TM1") == "TM1" and "seq" in record and "phase" in record


def load_records(path):
    records, rejected = [], []
    for line_no, line in enumerate(Path(path).read_text(encoding="utf-8-sig").splitlines(), 1):
        if not line.strip():
            continue
        try:
            r = json.loads(line)
            if not isinstance(r, dict):
                raise ValueError("record must be an object")
            r = dict(r, _line=line_no, _host=host_seconds(r))
            if tm_record(r):
                for key in ("tick_ms", "seq", "phase"):
                    if type(r.get(key)) is not int or not 0 <= r[key] <= MASK:
                        raise ValueError(f"invalid TM {key}")
            records.append(r)
        except (ValueError, KeyError, TypeError) as exc:
            rejected.append({"line": line_no, "error": str(exc)})
    return records, rejected


def split_runs(records):
    """START and discontinuous TM clocks form explicit analysis blocks."""
    groups, boundaries, current, previous = [], [], [], None
    pending_start = False
    for r in records:
        if r.get("direction") == "TX" and ",START," in r.get("text", ""):
            pending_start = True
        if not tm_record(r):
            continue
        reason = None
        if previous:
            dt = (r["tick_ms"] - previous["tick_ms"]) & MASK
            host_dt = r["_host"] - previous["_host"]
            if pending_start:
                reason = "explicit_START"
            elif dt >= 0x80000000:
                reason = "device_clock_reset_or_old_buffer"
            elif host_dt < 0 or dt > max(2000, host_dt * 1000 + 2000):
                reason = "device_host_clock_discontinuity"
        if reason and current:
            groups.append(current)
            boundaries.append({"line": r["_line"], "reason": reason})
            current = []
        current.append(r)
        previous, pending_start = r, False
    if current:
        groups.append(current)
    return groups, boundaries


def analyze(path, run_index=None):
    path = Path(path)
    records, rejected = load_records(path)
    groups, boundaries = split_runs(records)
    if not groups:
        raise ValueError("no usable TM telemetry")
    # Prefer the last block with actual backend/VM activity. A post-reset idle
    # tail must not silently replace the motion record.
    if run_index is None:
        active = [i for i, group in enumerate(groups) if any(r.get("backend_active") or r.get("pe2_active") or r.get("phase") in (1, 2, 4) for r in group)]
        run_index = active[-1] if active else len(groups) - 1
    if not 0 <= run_index < len(groups):
        raise ValueError(f"block index must be 0..{len(groups)-1}")
    chosen = groups[run_index]
    origin, origin_host = chosen[0]["tick_ms"], chosen[0]["_host"]
    rows, elapsed, gaps, previous = [], 0, [], None
    for r in chosen:
        if previous:
            delta = (r["tick_ms"] - previous["tick_ms"]) & MASK
            elapsed += delta
            seq_gap = (r["seq"] - previous["seq"]) & MASK
            if delta > 300 or seq_gap not in (0, 1):
                gaps.append({"at_s": elapsed / 1000, "device_gap_ms": delta, "sequence_delta": seq_gap})
        moving_reference = r.get("backend_active") == 1 and r.get("backend_state", 6) in (5, 6)
        row = {"time_s": elapsed / 1000, "host_time_s": r["_host"] - origin_host,
               "device_tick_ms": r["tick_ms"], "phase": r["phase"],
               "base_rpm": number(r, "base_rpm") if moving_reference else None,
               "reported_base_rpm": number(r, "base_rpm"),
               "correction_rpm": number(r, "correction_rpm"), "heading_error_deg": number(r, "error_mdeg", 1000),
               "encoder_progress_cm": number(r, "distance_progress_milli_mm", 10000) if r.get("distance_mode") else None,
               "target_cm": number(r, "distance_target_milli_mm", 10000) if r.get("distance_mode") else None,
               "vm_active": number(r, "pe2_active"), "distance_fault_reason": r.get("distance_fault_reason"),
               "break_before": bool(previous and delta > 300)}
        row.update({f"rpm{w}": number(r, f"rpm{w}") for w in range(1, 5)})
        rows.append(row)
        previous = r
    batches, seen = [], set()
    upper_line = groups[run_index + 1][0]["_line"] if run_index + 1 < len(groups) else math.inf
    for r in records:
        if r.get("frame_type") != "BT1" or r["_line"] < chosen[0]["_line"] or r["_line"] >= upper_line:
            continue
        try:
            key = (r["session"], r["batch_id"], r["start_ms"])
            t = (r["start_ms"] - origin) & MASK
            duration = (r["end_ms"] - r["start_ms"]) & MASK
            queue = (r["start_ms"] - r["generated_ms"]) & MASK
            if key in seen or t > elapsed or duration >= 0x80000000 or queue >= 0x80000000:
                continue
            seen.add(key)
            batches.append({"time_s": t / 1000, "batch_id": r["batch_id"], "session": r["session"],
                            "batch_ms": duration, "queue_ms": queue, "outcome": r["outcome"],
                            "ack_mask": r["ack_mask"], "base_rpm": r.get("base_rpm"),
                            "correction_rpm": r.get("correction_rpm"), "overwritten": r.get("overwritten"),
                            "cumulative_max_queue_ms": r.get("max_queue_ms"),
                            "cumulative_max_batch_ms": r.get("max_batch_ms"),
                            "cumulative_max_call_ms": r.get("max_call_ms"),
                            "cumulative_success": r.get("total_success"), "cumulative_interrupted": r.get("total_interrupted"),
                            "send_span_ms": ((r["send_ms4"] - r["send_ms1"]) & MASK) if r.get("sent_mask") == 15 else None,
                            "call_ms": [((r[f"reply_ms{w}"] - r[f"send_ms{w}"]) & MASK) if r.get("ack_mask", 0) & (1 << (w-1)) else None for w in range(1, 5)]})
        except (KeyError, TypeError):
            rejected.append({"line": r["_line"], "error": "incomplete BT1"})
    summary_path = path.with_suffix(".summary.json")
    capture_summary = read_json(summary_path) if summary_path.exists() else {}
    last = chosen[-1]
    notes = list(LIMITS)
    if last.get("distance_fault_reason"):
        notes.append(f"Last distance fault reason: {last['distance_fault_reason']}; preserve original failure, no automatic retune/retry.")
    return {"input": str(path.resolve()), "input_sha256": sha256(path), "block_index": run_index,
            "block_count": len(groups), "block_sizes": [len(g) for g in groups], "boundaries": boundaries,
            "rejected": rejected, "rows": rows, "batches": batches, "gaps": gaps,
            "capture_status": capture_summary.get("status", "unknown"), "capture_error": capture_summary.get("error"),
            "metrics": {"duration_s": elapsed / 1000, "tm_samples": len(rows), "unique_retained_batches": len(batches),
                        "max_abs_heading_error_deg": max((abs(r["heading_error_deg"]) for r in rows if r["heading_error_deg"] is not None), default=None),
                        "encoder_final_cm": rows[-1]["encoder_progress_cm"], "last_phase": last["phase"],
                        "distance_fault_reason": last.get("distance_fault_reason"),
                        "last_vm_active": last.get("pe2_active"), "last_wheel_rpm": [last.get(f"rpm{w}") for w in range(1, 5)],
                        "max_retained_batch_ms": max((r["batch_ms"] for r in batches), default=None)},
            "limits": notes}


def write_csv(path, rows):
    if not rows:
        return
    with Path(path).open("w", encoding="utf-8-sig", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def chart_html(report):
    subtitle = (f"{Path(report['input']).name}。原主机结果：{report['capture_status']}；"
                f"所选数据块末态phase：{report['metrics']['last_phase']}；距离故障码：{report['metrics']['distance_fault_reason']}。"
                "图表生成成功不代表运动测试成功。基速仅画后端START/RUN，待机配置保留在CSV；四轮反馈为有符号异步缓存值。")
    specs = [['基速参考与前端纠偏（RPM）', ['base_rpm', 'correction_rpm'], 'rows'],
             ['四轮缓存反馈（RPM）', ['rpm1', 'rpm2', 'rpm3', 'rpm4'], 'rows'],
             ['校准编码进度与目标（cm，非地面位移）', ['encoder_progress_cm', 'target_cm'], 'rows'],
             ['航向误差（度）', ['heading_error_deg'], 'rows'],
             ['整批通信与排队（ms，非电机响应时间）', ['batch_ms', 'queue_ms'], 'batches']]
    return interactive_chart_html('P1 离线运动分析', subtitle, {"rows": report['rows'], "batches": report['batches']}, specs, report['limits'])


def interactive_chart_html(title, subtitle, data, specs, notes):
    """Shared offline renderer; caller supplies explicit source labels and units."""
    payload = json.dumps(dict(data, specs=specs), allow_nan=False).replace("<", "\\u003c")
    title, subtitle = html.escape(title), html.escape(subtitle)
    notes = "".join(f"<li>{html.escape(s)}</li>" for s in notes)
    return '''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>''' + title + '''</title>
<style>body{font:15px system-ui;margin:24px;background:#f6f8fb;color:#1a2940}section{background:white;padding:16px;margin:16px 0;border-radius:10px}svg{width:100%;height:230px}label{margin-right:14px}text{font:11px system-ui;fill:#42516a}input{width:110px}#tip{white-space:pre-wrap}</style>
<h1>''' + title + '''</h1><p>''' + subtitle + '''</p><p>各图共用时间轴，虚线为事件时刻；可选择时间范围、开关曲线和读取邻近样本。</p>
<p>时间范围（秒）：<input id="lo" type="number" value="0" step="0.1"> 至 <input id="hi" type="number" step="0.1"><button id="apply">更新范围</button><button id="reset">全部</button></p>
<div id="plots"></div><section id="tip">移动鼠标查看最邻近采样。曲线的来源与单位见各图标题和数据边界。</section><section><b>数据边界</b><ul>''' + notes + '''</ul></section>
<script id="data" type="application/json">''' + payload + '''</script><script>
const data=JSON.parse(document.getElementById('data').textContent), colors=['#1769aa','#de6a19','#158153','#9c3eab','#a33a47'];
const specs=data.specs;
const ns='http://www.w3.org/2000/svg',el=(tag,attr,parent)=>{let e=document.createElementNS(ns,tag);for(let k in attr)e.setAttribute(k,attr[k]);parent.appendChild(e);return e};
function draw(){let lo=+document.getElementById('lo').value,hi=+document.getElementById('hi').value;if(!(hi>lo))return;document.getElementById('plots').replaceChildren();
for(const [title,keys,source,interpolation] of specs){let sec=document.createElement('section'),h=document.createElement('h3');h.textContent=title;sec.appendChild(h);document.getElementById('plots').appendChild(sec);let svg=el('svg',{viewBox:'0 0 1000 230'},sec),rs=data[source].filter(r=>r.time_s>=lo&&r.time_s<=hi),ys=rs.flatMap(r=>keys.map(k=>r[k])).filter(v=>typeof v==='number');
if(!ys.length){el('text',{x:50,y:80},svg).textContent='无可用记录';continue;}let ymin=Math.min(...ys,0),ymax=Math.max(...ys);if(ymax===ymin)ymax=ymin+1;let x=t=>60+(t-lo)/(hi-lo)*900,y=v=>190-(v-ymin)/(ymax-ymin)*160;
for(let i=0;i<=4;i++){let v=ymin+(ymax-ymin)*i/4;el('line',{x1:60,x2:960,y1:y(v),y2:y(v),stroke:'#e3e8ef'},svg);el('text',{x:0,y:y(v)+4},svg).textContent=v.toFixed(2);let t=lo+(hi-lo)*i/4;el('text',{x:x(t),y:215},svg).textContent=t.toFixed(1)+'s';}
for(const mark of (data.markers||[])){if(mark.time_s<lo||mark.time_s>hi)continue;let line=el('line',{x1:x(mark.time_s),x2:x(mark.time_s),y1:25,y2:190,stroke:'#9babbc','stroke-dasharray':'4 4'},svg);el('title',{},line).textContent=mark.label;}
keys.forEach((k,i)=>{let on=true,label=document.createElement('label'),box=document.createElement('input');box.type='checkbox';box.checked=true;box.style.width='auto';label.style.color=colors[i];label.append(box,document.createTextNode(k));sec.insertBefore(label,svg);let d='',pen=false;for(let r of rs){if(r[k]===null||r[k]===undefined){pen=false;continue;}if(r.break_before)pen=false;if(pen&&interpolation==='step')d+='H'+x(r.time_s)+'V'+y(r[k])+' ';else d+=(pen?'L':'M')+x(r.time_s)+' '+y(r[k])+' ';pen=true;}let layer=el('g',{},svg);el('path',{d,fill:'none',stroke:colors[i],'stroke-width':1.7},layer);if(source==='batches')for(let r of rs)if(typeof r[k]==='number')el('circle',{cx:x(r.time_s),cy:y(r[k]),r:2.5,fill:colors[i]},layer);box.onchange=()=>{on=box.checked;layer.style.visibility=on?'visible':'hidden';};});
svg.onmousemove=e=>{let rect=svg.getBoundingClientRect(),t=lo+((e.clientX-rect.left)/rect.width*1000-60)/900*(hi-lo);let nearest=rs.reduce((best,r)=>!best||Math.abs(r.time_s-t)<Math.abs(best.time_s-t)?r:best,null);if(nearest)document.getElementById('tip').textContent=title+'\\n'+JSON.stringify(nearest,null,2);};}}
document.getElementById('hi').value=data.rows.at(-1).time_s||1;document.getElementById('apply').onclick=draw;document.getElementById('reset').onclick=()=>{document.getElementById('lo').value=0;document.getElementById('hi').value=data.rows.at(-1).time_s||1;draw();};draw();
</script></html>'''


def export_analysis(path, out, run_index=None):
    report = analyze(path, run_index)
    save_json(out / "analysis.json", report)
    write_csv(out / "timeline.csv", report["rows"])
    write_csv(out / "batches.csv", report["batches"])
    (out / "report.html").write_text(chart_html(report), encoding="utf-8")
    return report


def compare(manifest_path, out):
    manifest_path = Path(manifest_path).resolve()
    entries = read_json(manifest_path)["runs"]
    rows = []
    for i, item in enumerate(entries):
        log = (manifest_path.parent / item["log"]).resolve()
        report = analyze(log, item.get("block_index"))
        ground = item.get("ground", {})
        for key in ("forward_cm", "lateral_cm_right_positive", "target_cm"):
            if ground.get(key) is not None and (type(ground[key]) not in (int, float) or not math.isfinite(ground[key])):
                raise ValueError(f"manual ground field {key} must be finite number or null")
        actual, target = ground.get("forward_cm"), ground.get("target_cm")
        rows.append({"label": item["label"], "version": item.get("version", "unknown"),
                     "capture_status": report["capture_status"], **report["metrics"],
                     "ground_target_cm": target, "ground_forward_cm": actual,
                     "ground_error_cm": actual - target if actual is not None and target is not None else None,
                     "ground_lateral_cm_right_positive": ground.get("lateral_cm_right_positive"),
                     "smoothness": ground.get("smoothness", "未量/未判断"),
                     "physical_stopped": ground.get("stopped"), "vm_lamp_off": ground.get("vm_lamp_off"),
                     "conditions": item.get("conditions", "未保证同条件"), "log_sha256": report["input_sha256"]})
    save_json(out / "comparison.json", {"runs": rows, "limits": LIMITS + ["Different heading/load/floor or single samples cannot establish improvement."]})
    write_csv(out / "comparison.csv", rows)
    cols = ["label", "capture_status", "encoder_final_cm", "ground_forward_cm", "ground_error_cm", "ground_lateral_cm_right_positive", "distance_fault_reason", "smoothness"]
    table = '<table border="1" cellpadding="8"><tr>' + ''.join(f'<th>{html.escape(k)}</th>' for k in cols) + '</tr>'
    for row in rows:
        table += '<tr>' + ''.join(f'<td>{html.escape(str(row.get(k)))}</td>' for k in cols) + '</tr>'
    (out / "comparison.html").write_text('<!doctype html><meta charset="utf-8"><title>P1对照</title><h1>历史结果对照</h1><p>横偏右正左负；None表示未量。软件结果与地面尺量分别保留，不自动判定某版更优。</p>' + table + '</table>', encoding="utf-8")
    return rows


def literal_assignment(path, name):
    tree = ast.parse(Path(path).read_text(encoding="utf-8-sig"))
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == name for t in node.targets):
            return ast.literal_eval(node.value)
    raise ValueError(f"missing assignment: {name}")


def macro(path, name):
    text = Path(path).read_text(encoding="gbk")
    match = re.search(r'^\s*#define\s+' + re.escape(name) + r'\s+(\d+(?:\.\d+)?)(?:[UuFf])?\s*(?:/\*.*)?$', text, re.M)
    if not match:
        raise ValueError(f"missing literal macro {name}: {path}")
    return float(match[1])


def check_config(root=ROOT, package=None):
    root = Path(root)
    scripts = root / "工创赛/scripts"
    schema = read_json(root / "工创赛/docs/P1_SERIAL_MOTION_SCHEMA_2026-10-24.json")
    profile = schema["speed20_trial_2026_10_27"]
    firmware = root / "3.stm32f407工程/USER"
    checks = []
    def check(name, observed, expected):
        checks.append({"name": name, "ok": observed == expected, "observed": observed, "expected": expected})
    for name, key in [("APP_SPEED20_TRIAL_RPM", "max_base_rpm_including_position_reserve"),
                      ("APP_SPEED20_TRIAL_MM", "distance_mm"), ("APP_SPEED20_TRIAL_MM_S", "reference_peak_mm_s"),
                      ("APP_SPEED20_FEEDBACK_RPM", "feedback_trip_rpm"), ("APP_SPEED20_RUN_MS", "run_ms"),
                      ("APP_DISTANCE_SCURVE_ARRIVAL_MM", "arrival_tolerance_mm"),
                      ("APP_DISTANCE_SCURVE_FINAL_WAIT_MS", "reference_end_wait_ms")]:
        check(name, macro(firmware / "app_config.h", name), profile[key])
    plans = literal_assignment(scripts / "p1_serial_motion.py", "DEBUG_PLAN_IDS")
    check("host high plan", plans.get(profile["host_plan"]), profile["plan_id"])
    check("firmware high plan", macro(firmware / "app_entry.h", "DEBUG_PLAN_FORWARD210_20"), profile["plan_id"])
    expected_plans = {"default": 0, "left30": 1, "forward30": 2, "right30": 3, "route-a": 4, "forward210": 6, "forward210-20": 7}
    check("all host presets", plans, expected_plans)
    for host_name, c_name in [("default", "DEFAULT"), ("left30", "LEFT30"), ("forward30", "FORWARD30"),
                              ("right30", "RIGHT30"), ("route-a", "ROUTE_A"), ("forward210", "FORWARD210")]:
        check("preset " + host_name, macro(firmware / "app_entry.h", "DEBUG_PLAN_" + c_name), plans[host_name])
    check("schema high command", "PLAN" + str(profile["plan_id"]) in schema["debug_entry_presets_2026_10_25"]["commands"], True)
    # Inspect host comparison literals without importing any serial client code.
    tree = ast.parse((scripts / "p1_serial_motion.py").read_text(encoding="utf-8-sig"))
    comparisons = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Compare) and isinstance(node.left, ast.Subscript) and isinstance(node.left.slice, ast.Constant):
            for op, value in zip(node.ops, node.comparators):
                if isinstance(value, ast.Constant) and type(value.value) in (int, float):
                    comparisons.add((node.left.slice.value, type(op).__name__, value.value))
    for field, expected in [("base_rpm", profile["max_base_rpm_including_position_reserve"]), ("feedback_limit_rpm", profile["feedback_trip_rpm"])]:
        check("host admission " + field, (field, "Eq", expected) in comparisons, True)
    check("backend total deadline", macro(firmware / "chassis_auto_test.h", "CHASSIS_AUTO_TOTAL_MS"), profile["overall_backend_ms"])
    check("TM base fields", literal_assignment(scripts / "p1_serial_motion.py", "FIELDS"), schema["fields_in_order"])
    check("root default curve off", macro(firmware / "app_config.h", "APP_DISTANCE_SCURVE_ENABLE"), 0)
    check("root default high off", macro(firmware / "app_config.h", "APP_SPEED20_TRIAL_ENABLE"), 0)
    check("root default custom route off", macro(firmware / "route_config.h", "APP_ROUTE_USE_CUSTOM"), 0)
    check("root default route speed trial off", macro(firmware / "route_config.h", "APP_ROUTE_SPEED_TRIAL_ENABLE"), 0)
    if package:
        package = Path(package)
        manifest = read_json(package / "manifest.json")
        check("package host presets", literal_assignment(package / "host/p1_serial_motion.py", "DEBUG_PLAN_IDS"), plans)
        for flag in ("APP_DISTANCE_SCURVE_ENABLE", "APP_SPEED20_TRIAL_ENABLE"):
            check("candidate " + flag, macro(package / "firmware/USER/app_config.h", flag), 1)
        for source in sorted(firmware.glob("*")):
            if source.suffix not in (".c", ".h"):
                continue
            frozen = package / "firmware/USER" / source.name
            left = source.read_bytes()
            right = frozen.read_bytes() if frozen.is_file() else b""
            if source.name == "app_config.h":
                # Only these two enable flags intentionally differ between live
                # default and dedicated candidate; all other bytes must agree.
                for flag in ("APP_DISTANCE_SCURVE_ENABLE", "APP_SPEED20_TRIAL_ENABLE"):
                    pattern = rb'(#define[ \t]+' + flag.encode("ascii") + rb'[ \t]+)[01]U'
                    left = re.sub(pattern, rb'\g<1>0U', left)
                    right = re.sub(pattern, rb'\g<1>0U', right)
            check("candidate source matches live " + source.name, hashlib.sha256(right).hexdigest(), hashlib.sha256(left).hexdigest())
        for rel, expected in manifest["files"].items():
            candidate = (package / rel).resolve()
            if package.resolve() not in candidate.parents:
                raise ValueError("package manifest path escapes package")
            check("package hash " + rel, sha256(candidate) if candidate.is_file() else "missing", expected)
        check("candidate AXF", sha256(package / "firmware/Objects/gongxun-f407.axf"), manifest["axf_sha256"])
    return {"ok": all(c["ok"] for c in checks), "checks": checks,
            "limits": "Literal/config/hash consistency only; C behavior is checked separately. No hardware access."}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    subs = parser.add_subparsers(dest="action", required=True)
    p = subs.add_parser("analyze"); p.add_argument("log", type=Path); p.add_argument("--block-index", type=int); p.add_argument("--out", type=Path, required=True)
    p = subs.add_parser("compare"); p.add_argument("manifest", type=Path); p.add_argument("--out", type=Path, required=True)
    p = subs.add_parser("check"); p.add_argument("--package", type=Path); p.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    out = safe_output(args.out)
    if args.action == "analyze":
        report = export_analysis(args.log, out, args.block_index)
        print(json.dumps(report["metrics"]))
    elif args.action == "compare":
        print(f"Compared {len(compare(args.manifest, out))} runs")
    else:
        report = check_config(package=args.package)
        save_json(out / "config.json", report)
        print(f"Consistency: {report['ok']}, {len(report['checks'])} checks")
        return 0 if report["ok"] else 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
