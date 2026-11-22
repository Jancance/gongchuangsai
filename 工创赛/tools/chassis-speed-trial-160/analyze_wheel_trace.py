"""Compare drive RPM feedback with RPM estimated from encoder position deltas."""
import argparse
import csv
import math
import sys

COUNTS_PER_REVOLUTION = 65536
WHEEL_DIAMETER_MM = 77.0


def read_events(path):
    events = []
    with open(path, encoding="utf-8-sig") as stream:
        for line in stream:
            fields = line.strip().split(",")
            if fields[0] != "T160_W":
                continue
            if len(fields) != 6:
                raise ValueError("Malformed T160_W record")
            _, address, reply_ms, speed_decirpm, sign, counts = fields
            address = int(address)
            sign = int(sign)
            counts = int(counts)
            if address < 1 or address > 4 or sign not in (0, 1):
                raise ValueError("Invalid wheel address or position sign")
            events.append(
                {
                    "address": address,
                    "reply_ms": int(reply_ms),
                    "drive_rpm": int(speed_decirpm) / 10.0,
                    "signed_counts": -counts if sign else counts,
                }
            )
    return events


def estimate(events):
    previous = {}
    rows = []
    for event in events:
        address = event["address"]
        old = previous.get(address)
        previous[address] = event
        if old is None:
            continue
        elapsed_ms = event["reply_ms"] - old["reply_ms"]
        if elapsed_ms <= 0:
            raise ValueError("Wheel reply time must increase for each address")
        delta_counts = event["signed_counts"] - old["signed_counts"]
        encoder_rpm = delta_counts * 60000.0 / (COUNTS_PER_REVOLUTION * elapsed_ms)
        rows.append(
            {
                "address": address,
                "reply_ms": event["reply_ms"],
                "interval_ms": elapsed_ms,
                "drive_rpm": event["drive_rpm"],
                "encoder_rpm": encoder_rpm,
                "rpm_difference": event["drive_rpm"] - encoder_rpm,
                "drive_wheel_rim_mm_s": event["drive_rpm"] * math.pi * WHEEL_DIAMETER_MM / 60.0,
                "encoder_wheel_rim_mm_s": encoder_rpm * math.pi * WHEEL_DIAMETER_MM / 60.0,
            }
        )
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", help="CSV-like T160 serial log")
    args = parser.parse_args()
    try:
        rows = estimate(read_events(args.log))
    except (OSError, ValueError) as error:
        parser.error(str(error))
    writer = csv.DictWriter(
        sys.stdout,
        lineterminator="\n",
        fieldnames=(
            "address",
            "reply_ms",
            "interval_ms",
            "drive_rpm",
            "encoder_rpm",
            "rpm_difference",
            "drive_wheel_rim_mm_s",
            "encoder_wheel_rim_mm_s",
        ),
    )
    writer.writeheader()
    writer.writerows(rows)


if __name__ == "__main__":
    main()
