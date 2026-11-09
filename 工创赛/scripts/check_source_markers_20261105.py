"""运行标记点单测及真实CLI合成回归，保存与现场数据隔离的证据。"""
from pathlib import Path
import hashlib
import importlib.util
import io
import json
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
OUT = ROOT / "evidence/integration/2026-11-05/source-markers"
OUT.mkdir(parents=True, exist_ok=True)


def main():
    suite = unittest.TestSuite()
    for pattern in ("test_source_markers.py", "test_source_paper.py", "test_calibration.py"):
        suite.addTests(unittest.defaultTestLoader.discover(str(ROOT / "tests"), pattern=pattern))
    log = io.StringIO()
    result = unittest.TextTestRunner(stream=log, verbosity=2).run(suite)
    (OUT / "unittest.txt").write_text(log.getvalue(), encoding="utf-8")
    spec = importlib.util.spec_from_file_location("marker_tests", ROOT / "tests/test_source_markers.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    dataset = module.fixture()
    (OUT / "synthetic_points.json").write_text(json.dumps(dataset, indent=2), encoding="utf-8")
    observation = module.MarkerTests().observation()
    (OUT / "synthetic_observation.json").write_text(json.dumps(observation, indent=2), encoding="utf-8")
    paper_spec = importlib.util.spec_from_file_location("paper_tests", ROOT / "tests/test_source_paper.py")
    paper_module = importlib.util.module_from_spec(paper_spec)
    paper_spec.loader.exec_module(paper_module)
    (OUT / "synthetic_paper.json").write_text(json.dumps(paper_module.fixture(), indent=2), encoding="utf-8")
    cli = ROOT / "scripts/source_marker_calibration.py"
    calls = [
        (["fit", str(OUT / "synthetic_points.json"), "--output", str(OUT / "synthetic_fit.json")], 0),
        (["preview", str(OUT / "synthetic_observation.json"), "--calibration", str(OUT / "synthetic_fit.json"),
          "--output", str(OUT / "synthetic_preview.json")], 0),
        (["fit", str(ROOT / "calibration/source_markers_template_20261105.json"),
          "--output", str(OUT / "empty_template_rejection.json")], 1),
        (["fit", str(OUT / "synthetic_points.json"), "--output", str(OUT / "synthetic_points.json")], 2),
        (["fit-paper", str(OUT / "synthetic_paper.json"), "--output", str(OUT / "synthetic_paper_fit.json")], 0),
        (["fit-paper", str(ROOT / "calibration/source_paper_template_20261105.json"),
          "--output", str(OUT / "empty_paper_rejection.json")], 1),
    ]
    checks = []
    for args, expected in calls:
        completed = subprocess.run([sys.executable, str(cli), *args], capture_output=True, text=True)
        checks.append({"mode": args[0], "expected_exit": expected, "actual_exit": completed.returncode,
                       "passed": completed.returncode == expected})
    sources = ["src/smart_carrier/source_markers.py", "scripts/source_marker_calibration.py",
               "scripts/pick_source_markers.py", "tests/test_source_markers.py", "tests/test_source_paper.py"]
    report = {"data_kind": "synthetic-only", "unit_tests": result.testsRun,
              "unit_failures": len(result.failures), "unit_errors": len(result.errors),
              "cli_checks": checks, "motion_allowed": False, "hardware_tested": False,
              "gui_tested_on_orangepi": False,
              "sha256": {p: hashlib.sha256((ROOT / p).read_bytes()).hexdigest() for p in sources},
              "passed": result.wasSuccessful() and all(c["passed"] for c in checks)}
    (OUT / "checks.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
