"""Summarize user-measured stopped-position errors by travel direction."""
import argparse
import csv
import math
import statistics
import sys

DIRECTIONS = ("forward", "reverse", "left", "right")
FIELDS = ("run", "direction", "forward_error_mm", "lateral_error_mm")


def summarize(rows):
    result = {}
    for direction in DIRECTIONS:
        selected = [row for row in rows if row["direction"] == direction]
        if not selected:
            continue
        forward = [row["forward_error_mm"] for row in selected]
        lateral = [row["lateral_error_mm"] for row in selected]
        result[direction] = {
            "runs": len(selected),
            "mean_forward_error_mm": statistics.mean(forward),
            "mean_lateral_error_mm": statistics.mean(lateral),
            "sample_sd_forward_mm": statistics.stdev(forward) if len(forward) > 1 else None,
            "sample_sd_lateral_mm": statistics.stdev(lateral) if len(lateral) > 1 else None,
            "next_run_forward_compensation_candidate_mm": -statistics.mean(forward),
            "next_run_lateral_compensation_candidate_mm": -statistics.mean(lateral),
        }
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csvfile", help="One row per stopped run, with signed errors in mm")
    args = parser.parse_args()
    try:
        with open(args.csvfile, encoding="utf-8-sig", newline="") as stream:
            reader = csv.DictReader(stream)
            if not all(name in (reader.fieldnames or []) for name in FIELDS):
                raise ValueError("Required columns: " + ", ".join(FIELDS))
            rows = []
            for row in reader:
                if row["direction"] not in DIRECTIONS or not row["run"]:
                    raise ValueError("Each row needs a run ID and a valid direction")
                forward = float(row["forward_error_mm"])
                lateral = float(row["lateral_error_mm"])
                if not math.isfinite(forward) or not math.isfinite(lateral):
                    raise ValueError("Position errors must be finite millimeters")
                rows.append(
                    {
                        "run": row["run"],
                        "direction": row["direction"],
                        "forward_error_mm": forward,
                        "lateral_error_mm": lateral,
                    }
                )
    except (OSError, ValueError) as error:
        parser.error(str(error))
    writer = csv.DictWriter(
        sys.stdout,
        fieldnames=(
            "direction",
            "runs",
            "mean_forward_error_mm",
            "mean_lateral_error_mm",
            "sample_sd_forward_mm",
            "sample_sd_lateral_mm",
            "next_run_forward_compensation_candidate_mm",
            "next_run_lateral_compensation_candidate_mm",
        ),
        lineterminator="\n",
    )
    writer.writeheader()
    for direction, values in summarize(rows).items():
        writer.writerow({"direction": direction, **values})


if __name__ == "__main__":
    main()
