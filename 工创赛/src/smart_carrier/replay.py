from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Iterable


def read_jsonl(path: Path) -> Iterable[dict]:
    with path.open("r", encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, 1):
            line = line.strip()
            if not line:
                continue
            try:
                yield json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f"{path}:{line_number}: invalid JSON") from exc


def summarize(path: Path) -> dict[str, int]:
    counts: dict[str, int] = {}
    for record in read_jsonl(path):
        kind = str(record.get("type", "UNKNOWN"))
        counts[kind] = counts.get(kind, 0) + 1
    return counts


def main() -> int:
    parser = argparse.ArgumentParser(description="Summarize P2 offline replay JSONL")
    parser.add_argument("path", type=Path)
    args = parser.parse_args()
    print(json.dumps(summarize(args.path), ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
