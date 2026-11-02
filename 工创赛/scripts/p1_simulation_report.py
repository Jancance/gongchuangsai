"""Export explicitly labeled software simulation traces; never read a device."""
import html
import math
from pathlib import Path

from p1_motion_offline import interactive_chart_html, save_json, write_csv

TRACE_FIELDS = ["case_id", "time_s", "ref_position_mm", "ref_speed_mm_s", "ref_acceleration_mm_s2", "ref_jerk_mm_s3",
                "base_reference_rpm", "follow_rpm", "follow_rate_rpm_s", "backend_progress_mm", "state", "result", "cause", "distance_reason",
                "reached", "stop_mask", "cut_vm_request", "backend_profile_done"]
TRACE_FIELDS += [f"{name}{w}" for name in ("sent_rpm", "model_rpm", "cached_rpm", "feedback_age_ms", "encoder_mm", "speed_calls", "stop_calls") for w in range(1, 5)]
TRACE_FIELDS += ["model_calibrated_encoder_mm", "target_mm", "ref_acceleration_limit", "ref_jerk_limit", "ref_duration_s", "deceleration_start_s", "stop_requested"]
INTEGER_FIELDS = {"case_id", "state", "result", "cause", "distance_reason", "reached", "stop_mask", "cut_vm_request", "backend_profile_done", "stop_requested"}
INTEGER_FIELDS.update(f"{name}{w}" for name in ("sent_rpm", "cached_rpm", "feedback_age_ms", "speed_calls", "stop_calls") for w in range(1, 5))
CASE_LABELS = ["四轮相同响应", "四轮独立延迟0/20/40/60ms", "第三轮速度增益降低20%", "第三轮速度应答缺失", "第一轮位置应答缺失",
               "第二轮停止应答缺失", "第二轮反馈超速", "第四轮反馈反向", "航向命令过期", "停车期间CAN任务停顿"]
LIMITS = ["全部曲线来自电脑上的实际C代码模拟，没有实车日志或地面尺量。",
          "ref是规划参考；sent是桩接口接到的发送目标，阶梯显示；model是假定的一阶轮速响应；cached是后端缓存反馈。",
          "停车后仍显示原计划的参考曲线用于对照，不表示停车后继续下发；实际桩目标见sent和调用计数。",
          "backend进度来自异步采样，model_cal为模拟编码均值经现有前进比例校准；两者都不是地面位移。",
          "参考加速度和jerk检查不证明整数RPM命令或真实轮速的加速度、jerk连续。七段S曲线允许jerk分段跳变。",
          "10ms调度、瞬时桩交易；RUN前置状态由适配器设置，VM准备、HWT控制器、真实总线/RTOS时序、滑移不在模型内。",
          "异常场景中的model记录截至后端提出撤VM请求；模型未继续计算断电后的机械滑行或停车距离。"]


def decode_sample(values):
    if len(values) != len(TRACE_FIELDS) or any(not math.isfinite(v) for v in values):
        raise ValueError("simulation trace shape or nonfinite value")
    row = dict(zip(TRACE_FIELDS, values))
    for key in INTEGER_FIELDS:
        if abs(row[key] - round(row[key])) > .001:
            raise ValueError(f"noninteger simulation state: {key}")
        row[key] = round(row[key])
    if not 0 <= row["case_id"] < len(CASE_LABELS) or row["time_s"] < 0:
        raise ValueError("simulation case/time range")
    row.update(ref_v_cm_s=row["ref_speed_mm_s"] / 10, ref_a_cm_s2=row["ref_acceleration_mm_s2"] / 10,
               ref_j_cm_s3=row["ref_jerk_mm_s3"] / 10, ref_cm=row["ref_position_mm"] / 10,
               backend_cm=row["backend_progress_mm"] / 10, model_cal_cm=row["model_calibrated_encoder_mm"] / 10,
               target_cm=row["target_mm"] / 10, tracking_error_mm=row["ref_position_mm"] - row["backend_progress_mm"])
    return row


def check_trace(rows, case_id):
    if not rows:
        return {"ok": False, "errors": ["missing trace"], "metrics": {}, "markers": []}
    errors = []
    if any(r["case_id"] != case_id for r in rows):
        errors.append("mixed simulation cases")
    if any(b["time_s"] <= a["time_s"] or abs(b["time_s"] - a["time_s"] - .01) > .00001 for a, b in zip(rows, rows[1:])):
        errors.append("trace time is not monotonic 10ms")
    if any(not -0.001 <= r["ref_speed_mm_s"] <= 200.01 or
           abs(r["ref_acceleration_mm_s2"]) > r["ref_acceleration_limit"] + .001 or
           abs(r["ref_jerk_mm_s3"]) > r["ref_jerk_limit"] + .001 for r in rows):
        errors.append("reference speed/acceleration/jerk bound")
    if any(not 0 <= r["base_reference_rpm"] <= 52 or any(not 0 <= r[f"sent_rpm{w}"] <= 54 for w in range(1, 5)) for r in rows):
        errors.append("command RPM bound")
    if any(r["time_s"] >= r["deceleration_start_s"] and r["state"] in (5, 6) and not r["reached"] and r["base_reference_rpm"] < 1 for r in rows):
        errors.append("base reference dropped below 1RPM before terminal arrival")
    last = rows[-1]
    peak = max(r["ref_speed_mm_s"] for r in rows)
    if case_id < 2:
        if peak < 199.9:
            errors.append("nominal reference did not reach 200mm/s")
        if last["result"] != 1 or not last["reached"] or last["stop_mask"] != 15 or any(last[f"cached_rpm{w}"] != 0 for w in range(1, 5)):
            errors.append("nominal run missing arrival, all stop acknowledgments or cached zero feedback")
    elif last["result"] != 2 or last["cut_vm_request"] != 1:
        errors.append("fault injection missing fault result or cut VM request")
    markers = []
    for name, key in [("进入减速参考段", "deceleration_start_s"), ("计划参考终点", "ref_duration_s")]:
        markers.append({"time_s": rows[0][key], "label": name})
    if case_id >= 3:
        markers.append({"time_s": 3, "label": "测试适配器开始注入异常/停车请求"})
    for label, predicate in [("后端停车请求（不等于已发出停止指令）", lambda r: r["stop_requested"]),
                             ("后端编码到距", lambda r: r["reached"]), ("进入逐轮STOP", lambda r: r["state"] == 7),
                             ("进入停稳确认", lambda r: r["state"] == 8), ("后端DONE", lambda r: r["result"] == 1),
                             ("后端故障并请求撤VM", lambda r: r["result"] == 2)]:
        found = next((r for r in rows if predicate(r)), None)
        if found:
            markers.append({"time_s": found["time_s"], "label": label})
    stop = next((r for r in rows if r["stop_requested"] or r["reached"] or r["state"] in (7, 8) or r["result"] == 2), None)
    # Count actual target changes: do not infer a CAN transaction from a change
    # to the base reference; speed/stop call counters identify the stub calls.
    updates = [sum(max(0, b[f"speed_calls{w}"] - a[f"speed_calls{w}"]) for w in range(1, 5)) for a, b in zip(rows, rows[1:])]
    metrics = {"samples": len(rows), "duration_s": last["time_s"], "reference_peak_cm_s": peak / 10,
               "reference_max_abs_acceleration_cm_s2": max(abs(r["ref_acceleration_mm_s2"]) for r in rows) / 10,
               "reference_max_abs_jerk_cm_s3": max(abs(r["ref_jerk_mm_s3"]) for r in rows) / 10,
               "max_abs_tracking_error_mm": max(abs(r["tracking_error_mm"]) for r in rows),
               "last_backend_encoder_cm": last["backend_cm"], "last_model_calibrated_encoder_cm": last["model_cal_cm"],
               "reference_speed_at_first_stop_cm_s": stop["ref_v_cm_s"] if stop else None,
               "base_rpm_at_first_stop": stop["base_reference_rpm"] if stop else None,
               "max_speed_calls_per_10ms": max(updates, default=0), "final_result": last["result"],
               "final_cause": last["cause"], "final_distance_reason": last["distance_reason"]}
    return {"ok": not errors, "errors": errors, "metrics": metrics, "markers": markers}


def export_traces(out, samples):
    out = Path(out)
    cases = []
    for case_id, label in enumerate(CASE_LABELS):
        rows = [r for r in samples if r["case_id"] == case_id]
        review = check_trace(rows, case_id)
        review.update(case_id=case_id, label=label, expected="正常到距并停稳" if case_id < 2 else "原保护触发故障并请求撤VM")
        cases.append(review)
        if not rows:
            continue
        folder = out / f"case_{case_id:02}"; folder.mkdir()
        write_csv(folder / "trace.csv", rows)
        save_json(folder / "trace.json", {"source": "software_simulation_only", "rows": rows, "review": review, "limits": LIMITS})
        specs = [["规划参考速度（cm/s）", ["ref_v_cm_s"], "rows"],
                 ["规划参考加速度（cm/s²）", ["ref_a_cm_s2"], "rows"],
                 ["规划参考jerk（cm/s³，允许分段跳变）", ["ref_j_cm_s3"], "rows", "step"],
                 ["模拟桩收到的四轮发送目标（RPM，阶梯）", [f"sent_rpm{w}" for w in range(1, 5)], "rows", "step"],
                 ["假定的一阶模型轮速（RPM幅值）", [f"model_rpm{w}" for w in range(1, 5)], "rows"],
                 ["后端缓存轮速反馈（有符号RPM）", [f"cached_rpm{w}" for w in range(1, 5)], "rows", "step"],
                 ["参考与两种编码进度（cm，非地面位移）", ["ref_cm", "backend_cm", "model_cal_cm", "target_cm"], "rows"],
                 ["参考减后端编码进度（mm）", ["tracking_error_mm"], "rows"],
                 ["整数基速参考与位置跟随修正（RPM）", ["base_reference_rpm", "follow_rpm"], "rows", "step"],
                 ["后端状态与请求（5/6发送或运行，7停车，8停稳确认，9完成，10故障）", ["state", "reached", "stop_requested", "cut_vm_request"], "rows", "step"]]
        status = "软件轨迹检查通过" if review["ok"] else "软件轨迹检查失败：" + "; ".join(review["errors"])
        subtitle = f"{label}。{status}；预期：{review['expected']}。来源：电脑软件模拟，车辆未运行；200mm/s为规划参考峰值。"
        notes = list(LIMITS) + [f"事件 {m['time_s']:.3f}s：{m['label']}" for m in review["markers"]]
        (folder / "report.html").write_text(interactive_chart_html("P1 20cm/s纯软件模拟", subtitle, {"rows": rows, "markers": review["markers"]}, specs, notes), encoding="utf-8")
    report = {"ok": all(c["ok"] for c in cases), "source": "software_simulation_only", "sample_count": len(samples), "cases": cases, "limits": LIMITS}
    save_json(out / "trace_review.json", report)
    table = '<table border="1" cellpadding="9"><tr><th>模拟场景</th><th>检查</th><th>预期</th><th>参考峰值cm/s</th><th>结果码/原因</th></tr>'
    for c in cases:
        metrics = c["metrics"]
        table += f'<tr><td><a href="case_{c["case_id"]:02}/report.html">{html.escape(c["label"])}</a></td><td>{"通过" if c["ok"] else "失败"}</td><td>{c["expected"]}</td><td>{metrics.get("reference_peak_cm_s")}</td><td>{metrics.get("final_result")}/{metrics.get("final_cause")}</td></tr>'
    (out / "index.html").write_text('<!doctype html><meta charset="utf-8"><title>P1 20cm/s模拟总览</title><style>body{font:16px system-ui;margin:28px}table{border-collapse:collapse}td,th{border:1px solid #ccd5df}</style><h1>P1 20cm/s纯软件模拟</h1><p>所有数据来自电脑模拟；没有实车测试。两项正常场景与八项故障注入分别判断，触发预期保护不表示运动完成。</p>' + table + '</table><p>每项含10ms过程CSV/JSON和交互图，显示规划参考、桩发送目标、模型轮速、编码进度及停车/故障事件。</p>', encoding="utf-8")
    return report
