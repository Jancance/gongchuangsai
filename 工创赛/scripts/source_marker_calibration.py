"""用实测标记点离线拟合/预览。无相机采集、串口或硬件控制。"""
from pathlib import Path
import argparse
import json
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from smart_carrier.source_markers import fit_markers, fit_paper, preview_points, check_paper_transfer


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("fit", "fit-paper", "preview", "check-paper-transfer"))
    parser.add_argument("input", type=Path)
    parser.add_argument("--calibration", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    inputs = [args.input] + ([args.calibration] if args.calibration else [])
    if any(args.output.resolve() == path.resolve() for path in inputs):
        parser.error("输出须与输入文件不同，保留实测原件")
    try:
        data = json.loads(args.input.read_text(encoding="utf-8-sig"))
        if args.mode == "fit":
            result = fit_markers(data)
        elif args.mode == "fit-paper":
            result = fit_paper(data)
        else:
            if args.calibration is None:
                parser.error("preview/check-paper-transfer需要--calibration")
            calibration = json.loads(args.calibration.read_text(encoding="utf-8-sig"))
            result = (check_paper_transfer(calibration, data) if args.mode == "check-paper-transfer"
                      else preview_points(calibration, data))
    except (ValueError, KeyError, TypeError, OSError) as exc:
        # 失败时仍输出明确拒绝报告，不能留下上次通过结果供误用。
        result = {"motion_allowed": False, "independent_check_passed": False, "error": str(exc)}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
    return 1 if ("error" in result or result.get("independent_check_passed") is False
                 or result.get("length_consistency_passed") is False) else 0


if __name__ == "__main__":
    raise SystemExit(main())
