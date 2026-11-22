"""Summarize one four-direction trial from already time-aligned samples."""
import argparse
import csv
import json
import math

AXES = {"forward": (1, 0), "reverse": (-1, 0), "left": (0, 1), "right": (0, -1)}
REQUIRED = (
    "time_ms", "ground_x_mm", "ground_y_mm", "ground_yaw_deg",
    "odom_x_mm", "odom_y_mm", "hwt_yaw_deg",
)


def signed_angle(deg):
    return (deg + 180.0) % 360.0 - 180.0


def analyze(rows, direction, planned_mm, stop_line_mm):
    if len(rows) < 2:
        raise ValueError("Need a synchronized start sample and a final physically stopped sample")
    dx, dy = AXES[direction]
    start = rows[0]
    along = []
    lateral = []
    odometry = []
    heading_error = {"hwt": []}
    if "icm_candidate_yaw_deg" in start:
        heading_error["icm_candidate"] = []
    for row in rows:
        x = row["ground_x_mm"] - start["ground_x_mm"]
        y = row["ground_y_mm"] - start["ground_y_mm"]
        along.append(dx * x + dy * y)
        lateral.append(-dy * x + dx * y)
        x = row["odom_x_mm"] - start["odom_x_mm"]
        y = row["odom_y_mm"] - start["odom_y_mm"]
        odometry.append(dx * x + dy * y)
        ground_yaw = row["ground_yaw_deg"] - start["ground_yaw_deg"]
        for source in heading_error:
            key = source + "_yaw_deg"
            heading_error[source].append(signed_angle(row[key] - start[key] - ground_yaw))
    step_ms = [b["time_ms"] - a["time_ms"] for a, b in zip(rows, rows[1:])]
    if min(step_ms) <= 0:
        raise ValueError("time_ms must strictly increase")
    interval_speed = [(b - a) * 100.0 / dt for a, b, dt in zip(along, along[1:], step_ms)]
    peak_interval = max(range(len(interval_speed)), key=interval_speed.__getitem__)
    return {
        "direction": direction,
        "planned_stop_mm": planned_mm,
        "ground_distance_mm": along[-1],
        "ground_minus_plan_mm": along[-1] - planned_mm,
        "odom_minus_ground_mm": odometry[-1] - along[-1],
        "odom_scale_candidate": along[-1] / odometry[-1] if odometry[-1] > 0 else None,
        "max_abs_cross_mm": max(map(abs, lateral)),
        "final_cross_mm": lateral[-1],
        "final_ground_heading_change_deg": signed_angle(
            rows[-1]["ground_yaw_deg"] - start["ground_yaw_deg"]
        ),
        "max_forward_crossing_of_stop_line_mm": max(0.0, max(along) - stop_line_mm),
        "stop_line_remaining_mm": stop_line_mm - along[-1],
        "peak_ground_interval_speed_cm_s": interval_speed[peak_interval],
        "peak_ground_interval_ms": step_ms[peak_interval],
        "physical_stop_verified_by_script": False,
        "sample_count": len(rows),
        "largest_sample_interval_ms": max(step_ms),
        "heading_deg": {
            source: {
                "rms_error": math.sqrt(sum(error ** 2 for error in values) / len(values)),
                "max_abs_error": max(map(abs, values)),
                "final_error": values[-1],
            }
            for source, values in heading_error.items()
        },
    }


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("csvfile", help="Time-aligned ground pose, odometry and heading samples")
    cli.add_argument("direction", choices=AXES)
    cli.add_argument("--planned-stop-mm", type=float, default=1664)
    cli.add_argument("--stop-line-mm", type=float, default=1920)
    args = cli.parse_args()
    if not 0 < args.planned_stop_mm <= args.stop_line_mm < math.inf:
        cli.error("Require 0 < planned stop <= stop line")
    try:
        with open(args.csvfile, encoding="utf-8-sig", newline="") as stream:
            reader = csv.DictReader(stream)
            columns = list(REQUIRED)
            if "icm_candidate_yaw_deg" in (reader.fieldnames or []):
                columns.append("icm_candidate_yaw_deg")
            if not all(key in (reader.fieldnames or []) for key in columns):
                raise ValueError("Missing required columns: " + ", ".join(columns))
            rows = [{key: float(row[key]) for key in columns} for row in reader]
        if any(not math.isfinite(value) for row in rows for value in row.values()):
            raise ValueError("All used samples must have finite values")
        result = analyze(rows, args.direction, args.planned_stop_mm, args.stop_line_mm)
    except (OSError, TypeError, ValueError) as exc:
        cli.error(str(exc))
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
