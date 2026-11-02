"""只读路线参数检查和预览；不打开串口，不编译、烧录或发车。"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import struct

ROOT = Path(__file__).resolve().parents[2]
USER = ROOT / "3.stm32f407工程/USER"
ERRORS = {1: "动作数必须为1～8", 2: "未知动作", 3: "距离或转角超限",
          4: "速度超限或不能表示为允许的整数RPM", 5: "名义最短时间已超过单段保护时限",
          6: "超过整组120秒保守保护预算", 7: "底盘几何参数无效"}
NUMBER = r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?[fF]?"
KINDS = {"FORWARD": 4, "LEFT": 5, "RIGHT": 6, "TURN": 3}
LABELS = {"FORWARD": "前进", "LEFT": "左移", "RIGHT": "右移", "TURN": "转向"}


def f32(value):
    return struct.unpack("<f", struct.pack("<f", value))[0]


def clean(text):
    return re.sub(r"/\*.*?\*/|//[^\n]*", "", text, flags=re.S)


def read_settings(path):
    text = clean(Path(path).read_bytes().decode("gbk"))
    def literal(name, pattern):
        matches = re.findall(rf"^\s*#define\s+{name}\s+({pattern})\s*$", text, re.M)
        if len(matches) != 1:
            raise ValueError(f"{name}必须且只能定义一次，使用允许的数字")
        return matches[0]
    trial = int(literal("APP_ROUTE_SPEED_TRIAL_ENABLE", "[01]"))
    caps = {}
    for kind in ["FORWARD", "LEFT", "RIGHT"]:
        try:
            value = f32(float(literal(f"APP_ROUTE_{kind}_MAX_CM_S", NUMBER).rstrip("fF")))
        except (OverflowError, ValueError) as exc:
            raise ValueError(f"{kind}候选上限不是可表示的数值") from exc
        if not 0 < value <= 20:
            raise ValueError(f"{kind}候选上限必须大于0且不超过20cm/s")
        caps[kind] = value
    return bool(trial), caps


def read_config(path):
    """Only a literal X-macro table is supported; never eval C expressions."""
    text = clean(Path(path).read_bytes().decode("gbk"))
    enabled = re.findall(r"^\s*#define\s+APP_ROUTE_USE_CUSTOM\s+([01])\s*$", text, re.M)
    if len(enabled) != 1:
        raise ValueError("APP_ROUTE_USE_CUSTOM必须且只能定义一次，值为0或1")
    flat = re.sub(r"\\\r?\n", " ", text)
    tables = re.findall(r"^\s*#define\s+APP_ROUTE_ROWS\(X\)[ \t]*(.*)$", flat, re.M)
    if len(tables) != 1:
        raise ValueError("找不到唯一APP_ROUTE_ROWS(X)表，请保留每行末尾续行符")
    pattern = re.compile(rf"\s*X\(\s*([A-Z]+)\s*,\s*({NUMBER})\s*,\s*({NUMBER})\s*\)")
    rows, cursor = [], 0
    while tables[0][cursor:].strip():
        match = pattern.match(tables[0], cursor)
        if not match:
            raise ValueError(f"第{len(rows)+1}段语法错误：只允许X(动作, 数值, 数值)")
        kind, amount, speed = match.groups()
        try:
            rows.append((kind, f32(float(amount.rstrip("fF"))), f32(float(speed.rstrip("fF")))))
        except (OverflowError, ValueError) as exc:
            raise ValueError(f"第{len(rows)+1}段数值无法表示为有限float") from exc
        cursor = match.end()
    # Reject extra rows stranded by a missing continuation, instead of silently ignoring them.
    without = re.sub(r"^\s*#define\s+APP_ROUTE_ROWS\(X\)[^\n]*$", "", flat, flags=re.M)
    without = re.sub(r"^\s*#(?:ifndef\s+ROUTE_CONFIG_H|define\s+ROUTE_CONFIG_H|endif|define\s+APP_ROUTE_USE_CUSTOM\s+[01])\s*$", "", without, flags=re.M)
    without = re.sub(rf"^\s*#define\s+(?:APP_ROUTE_SPEED_TRIAL_ENABLE\s+[01]|APP_ROUTE_(?:FORWARD|LEFT|RIGHT)_MAX_CM_S\s+{NUMBER})\s*$", "", without, flags=re.M)
    if without.strip():
        raise ValueError("配置文件含未识别内容或表外动作行，请检查续行符；预览不忽略额外C代码")
    return bool(int(enabled[0])), rows


def candidate_profile(kind, amount, speed, geometry, left_trim=False):
    """Float32 scalar planner equivalent; final firmware remains authoritative."""
    diameter = f32(geometry[0])
    factor = f32(f32(f32(math.pi) * diameter) / 60)
    if kind == "FORWARD":
        scale_text = (USER / "chassis_kinematics.h").read_bytes().decode("gbk")
        match = re.search(r"#define\s+CHASSIS_FORWARD_DISTANCE_SCALE\s+(" + NUMBER + ")", scale_text)
        if not match:
            raise ValueError("前进距离比例不再是数字，须更新预览适配")
        factor = f32(factor * f32(float(match[1].rstrip("fF"))))
    def constant(filename, name):
        text = (USER / filename).read_bytes().decode("gbk")
        found = re.search(r"^#define\s+" + name + r"\s+(" + NUMBER + r")[uU]?\s*$", text, re.M)
        if not found:
            raise ValueError(f"{name}不再是数字，须更新预览适配")
        return f32(float(found[1].rstrip("fF")))
    trim_ratio = 0.0
    if kind == "LEFT" and left_trim:
        factor = f32(factor * constant("app_config.h", "APP_LEFT_DISTANCE_SCALE"))
        trim_ratio = constant("app_config.h", "APP_LEFT_BACKWARD_RATIO")
    follow = constant("motion_route_trial.h", "ROUTE_TRIAL_FOLLOW_RPM")
    omega = constant("app_config.h", "APP_DISTANCE_SCURVE_FOLLOW_OMEGA")
    acceleration_limit = constant("app_config.h", "APP_DISTANCE_SCURVE_ACCEL_MM_S2")
    jerk_limit = constant("app_config.h", "APP_DISTANCE_SCURVE_JERK_MM_S3")
    final_wait = constant("app_config.h", "APP_DISTANCE_SCURVE_FINAL_WAIT_MS")
    margin = constant("motion_route_trial.h", "ROUTE_TRIAL_MARGIN_MS")
    target, distance = f32(speed * 10), f32(amount * 10)
    value = f32(target / factor)
    rpm = math.ceil(value) + 2
    acceleration = f32(acceleration_limit - f32(f32(f32(factor * 2) * follow) * omega))
    jerk = f32(jerk_limit - f32(f32(f32(f32(factor * 6) * follow) * omega) * omega))
    if trim_ratio:
        acceleration = f32(acceleration / f32(1 + trim_ratio))
        jerk = f32(jerk / f32(1 + trim_ratio))
    if not acceleration > 0 or not jerk > 0 or follow != 2:
        raise ValueError("候选曲线预算无效或跟随余量改变，须更新预览适配")
    tj = min(f32(acceleration / jerk), f32(math.sqrt(f32(target / jerk))))
    ta = max(0.0, f32(f32(target / f32(jerk * tj)) - tj))
    minimum_distance = f32(target * f32(f32(2 * tj) + ta))
    tv, peak = 0.0, target
    if distance >= minimum_distance:
        tv = f32(f32(distance - minimum_distance) / target)
    else:
        tj = f32(acceleration / jerk)
        if distance >= f32(f32(f32(2 * acceleration) * tj) * tj):
            ta = f32(.5 * f32(f32(math.sqrt(f32(f32(tj * tj) + f32(f32(4 * distance) / acceleration)))) - f32(3 * tj)))
        else:
            tj = f32(math.pow(f32(distance / f32(2 * jerk)), f32(1 / 3)))
            ta = 0.0
        peak = f32(f32(jerk * tj) * f32(tj + ta))
    duration = f32(sum([tj, ta, tj, tv, tj, ta, tj]))
    duration_ms = f32(duration * 1000)
    if not 0 < duration_ms < 56000 or not 3 <= rpm <= 52 or not 0 < peak <= target * 1.00001:
        return None
    return dict(rpm=rpm, reference_mm_s=target, peak_cm_s=peak / 10,
                plan_duration_s=duration, run_ms=math.ceil(duration_ms) + int(final_wait + margin),
                reference_acceleration_mm_s2=acceleration, reference_jerk_mm_s3=jerk)


def read_geometry(path):
    text = clean(Path(path).read_bytes().decode("gbk"))
    match = re.search(r"g_chassis_geometry\s*=\s*\{([^}]+)\}", text)
    if not match:
        raise ValueError("找不到当前底盘几何参数")
    parts = match[1].split(",")
    if len(parts) != 3 or any(not re.fullmatch(NUMBER, x.strip()) for x in parts):
        raise ValueError("几何参数已改为表达式，需更新预览适配，不能沿用旧数值")
    return tuple(f32(float(x.strip().rstrip("fF"))) for x in parts)


def validate(rows, geometry, heading_ms=30000, candidate=False, caps=None, left_trim=False, turn_trial=False, turn_dps=15):
    result = {"ok": False, "error": 0, "segment": 0, "budget_ms": 5000, "rows": []}
    def fail(code, segment=0):
        result.update(error=code, segment=segment, message=ERRORS[code])
        return result
    if not 1 <= len(rows) <= 8:
        return fail(1)
    if len(geometry) != 3 or not all(0 < x < 1000 for x in geometry):
        return fail(7)
    diameter, track, wheelbase = map(f32, geometry)
    caps = caps or dict.fromkeys(["FORWARD", "LEFT", "RIGHT"], 20.0)
    for index, (kind, amount, speed) in enumerate(rows, 1):
        if kind == "TURN":
            if not (5 <= amount <= 90 or -90 <= amount <= -5):
                return fail(3, index)
            factor = f32(f32(6 * diameter) / f32(track + wheelbase))
            minimum, maximum = 1, 5
            if turn_trial and len(rows)==1 and abs(amount)==90: maximum=27.2 if turn_dps==30 else 13.6
        elif kind in {"FORWARD", "LEFT", "RIGHT"}:
            if candidate:
                if not 10 <= amount <= 210:
                    return fail(3, index)
                if not 1.21 <= speed <= caps[kind] <= 20:
                    return fail(4, index)
                profile = candidate_profile(kind, amount, speed, geometry, left_trim=left_trim)
                if profile is None:
                    return fail(5, index)
                result["budget_ms"] += profile["run_ms"] + 6000
                if result["budget_ms"] > 120000:
                    return fail(6, index)
                result["rows"].append(dict(segment=index, kind=kind, amount=amount, requested_speed=speed,
                                           nominal_speed=speed, minimum_time_s=profile["plan_duration_s"], **profile))
                continue
            if not 10 <= amount <= 100:
                return fail(3, index)
            factor = f32(f32(f32(math.pi) * diameter) / 600)
            minimum, maximum = 3, 15
        else:
            return fail(2, index)
        if not f32(minimum * factor) <= speed <= f32(maximum * factor):
            return fail(4, index)
        rpm = int(f32(speed / factor))
        if not minimum <= rpm <= maximum:
            return fail(4, index)
        if kind=="TURN" and rpm>5 and (not turn_trial or rpm!=(27 if turn_dps==30 else 13)):
            return fail(4,index)
        duration = f32(f32(abs(amount) / f32(factor * rpm)) * 1000)
        limit_ms = heading_ms if kind == "TURN" else 30000
        if duration > limit_ms:
            return fail(5, index)
        result["budget_ms"] += limit_ms + 3500
        if result["budget_ms"] > 120000:
            return fail(6, index)
        result["rows"].append({"segment": index, "kind": kind, "amount": amount,
                               "requested_speed": speed, "rpm": rpm,
                               "nominal_speed": f32(factor * rpm), "minimum_time_s": duration / 1000})
    result.update(ok=True, message="参数检查通过；不是实车或避障验收")
    return result


def preview(config=USER / "route_config.h", geometry=USER / "chassis_kinematics.c", candidate=False):
    enabled, rows = read_config(config)
    trial, caps = read_settings(config)
    candidate = candidate or (enabled and trial)
    app_path = Path(config).parent / "app_config.h"
    app_text = app_path.read_bytes().decode("gbk") if app_path.exists() else ""
    trim_match = re.search(r"^#define\s+APP_LEFT_TRIM_ENABLE\s+([01])[uU]?\s*$", app_text, re.M)
    left_trim = bool(candidate and trim_match and trim_match[1] == "1")
    turn_match=re.search(r"^#define\s+APP_TURN_BRAKE_TRIAL_ENABLE\s+1[uU]?\s*$",app_text,re.M)
    turn_trial=bool(enabled and turn_match)
    turn_dps_match=re.search(r"^#define\s+APP_TURN_BRAKE_TRIAL_DPS\s+(15|30)[uU]?\s*$",app_text,re.M)
    turn_dps=int(turn_dps_match[1]) if turn_dps_match else 15
    result = validate(rows, read_geometry(geometry), candidate=candidate, caps=caps, left_trim=left_trim,turn_trial=turn_trial,turn_dps=turn_dps)
    result["turn_brake_trial_candidate"]=turn_trial
    result["left_trim_candidate"] = left_trim
    if left_trim:
        result["trim_note"] = "左移后向分量按整数RPM取整，低速可能为0；参考比例不代表实际地面补偿率。"

    result.update(candidate_preview=candidate, trial_switch=trial, direction_caps_cm_s=caps)
    result.update(custom_enabled=enabled, config_path=str(Path(config).resolve()),
                  config_sha256=hashlib.sha256(Path(config).read_bytes()).hexdigest(),
                  hardware_accessed=False,
                  limits="候选预览估计S曲线参考峰值和时间，最终以固件检查为准；未证明实际地速/定位/避障。120秒总保护保留。")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=USER / "route_config.h")
    parser.add_argument("--json", action="store_true", help="输出机器可读结果，不写文件")
    parser.add_argument("--candidate", action="store_true", help="即使开关关闭也只读预览新速度候选；不启用或下发")
    args = parser.parse_args()
    try:
        report = preview(args.config, candidate=args.candidate)
    except (ValueError, OSError, UnicodeError) as exc:
        report = {"ok": False, "message": str(exc), "hardware_accessed": False}
    if args.json:
        print(json.dumps(report, ensure_ascii=False, indent=2))
    else:
        print("自定义路线：" + ("启用（仅调试模式3的PLAN0；须重新编译烧录才改变板上程序）" if report.get("custom_enabled") else "未启用，默认仍为原四段测试"))
        if report.get("candidate_preview"):
            print("新速度候选预览：前进/左移/右移分别限速；未实车验收，预览不会启用开关。")
        for row in report.get("rows", []):
            turn = row["kind"] == "TURN"
            if "reference_mm_s" in row:
                print(f"第{row['segment']}段 {LABELS[row['kind']]} {row['amount']:g}cm；参考上限{row['requested_speed']:g}cm/s，"
                      f"参考峰值{row['peak_cm_s']:.3f}cm/s；基速余量上限{row['rpm']}RPM；"
                      f"参考时长{row['plan_duration_s']:.3f}s，阶段截止{row['run_ms']/1000:.3f}s；停稳后继续")
                continue
            print(f"第{row['segment']}段 {LABELS[row['kind']]} {row['amount']:g}{'度' if turn else 'cm'}；"
                  f"设置{row['requested_speed']:g}{'度/s' if turn else 'cm/s'} → {row['rpm']}RPM，"
                  f"名义量化上限{row['nominal_speed']:.3f}{'度/s' if turn else 'cm/s'}；停稳后继续")
        if report.get("error"):
            print(f"拒绝：第{report['segment']}段（0代表整表）：{report['message']}")
        else:
            print(report["message"])
        if "budget_ms" in report:
            print(f"保守保护预算：{report['budget_ms']/1000:g}/120秒；不是预计运行时间。")
        print("本工具不连接小车。配置检查不证明真实地速、平顺性、通行空间或避障。")
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
