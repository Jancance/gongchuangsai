"""Offline tool regression cases; no hardware client is imported or instantiated."""
import ast
from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest
from unittest.mock import patch

import p1_motion_offline as m


def tm(tick, seq, host, **kwargs):
    r = dict(host_utc=f"2026-10-27T00:00:{host:06.3f}+00:00", direction="RX", tick_ms=tick, seq=seq,
             phase=2, backend_active=1, pe2_active=1, base_rpm=15, error_mdeg=100,
             distance_mode=1, distance_progress_milli_mm=100000, distance_target_milli_mm=300000,
             rpm1=15, rpm2=-15, rpm3=15, rpm4=-15)
    r.update(kwargs)
    return r


class OfflineTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def log(self, rows, status="software_complete"):
        p = self.root / "run.jsonl"
        p.write_text("\n".join(json.dumps(r) for r in rows) + "\n", encoding="utf-8")
        m.save_json(p.with_suffix(".summary.json"), {"status": status, "error": None})
        return p

    def test_wrap_remains_one_clock_block(self):
        r = m.analyze(self.log([tm(0xFFFFFFF0, 0xFFFFFFFF, 0), tm(84, 0, .1)]))
        self.assertEqual(r["block_count"], 1)
        self.assertEqual(r["rows"][-1]["time_s"], .1)
        self.assertEqual(r["gaps"], [])

    def test_reset_splits_and_never_reports_49day_duration(self):
        r = m.analyze(self.log([tm(90000, 900, 0), tm(50, 1, .1), tm(150, 2, .2)]))
        self.assertEqual(r["block_sizes"], [1, 2])
        self.assertEqual(r["metrics"]["duration_s"], .1)

    def test_buffered_device_jump_is_boundary(self):
        r = m.analyze(self.log([tm(100, 1, 0), tm(15000, 150, .01), tm(15100, 151, .11)]))
        self.assertEqual(r["block_sizes"], [1, 2])
        self.assertEqual(r["gaps"], [])

    def test_start_selects_actual_run(self):
        r = m.analyze(self.log([tm(0, 1, 0, phase=0, backend_active=0, pe2_active=0),
                              dict(host_utc="2026-10-27T00:00:00.05+00:00", direction="TX", text="RC1,1,0,START,1234"), tm(100, 2, .1)]))
        self.assertEqual(r["block_index"], 1)
        self.assertEqual(r["metrics"]["tm_samples"], 1)

    def test_reset_idle_does_not_hide_failed_motion(self):
        r = m.analyze(self.log([tm(5000, 50, 0, phase=4, distance_fault_reason=9), tm(0, 0, .1, phase=0, backend_active=0, pe2_active=0)]))
        self.assertEqual(r["block_index"], 0)
        self.assertEqual(r["metrics"]["distance_fault_reason"], 9)

    def test_missing_values_stay_unknown(self):
        r = m.analyze(self.log([tm(0, 0, 0, rpm1=None, error_mdeg=-2147483648)]))
        self.assertIsNone(r["rows"][0]["rpm1"])
        self.assertIsNone(r["metrics"]["max_abs_heading_error_deg"])

    def test_units_and_signed_feedback(self):
        r = m.analyze(self.log([tm(0, 0, 0)]))
        self.assertEqual(r["rows"][0]["encoder_progress_cm"], 10)
        self.assertEqual(r["rows"][0]["heading_error_deg"], .1)
        self.assertEqual(r["rows"][0]["rpm2"], -15)

    def test_idle_configuration_not_plotted_as_motion_reference(self):
        r = m.analyze(self.log([tm(0, 0, 0, backend_active=0, phase=0, base_rpm=52)]))
        self.assertIsNone(r["rows"][0]["base_rpm"])
        self.assertEqual(r["rows"][0]["reported_base_rpm"], 52)

    def test_gap_marks_disconnected_chart_line(self):
        r = m.analyze(self.log([tm(0, 0, 0), tm(600, 6, .6)]))
        self.assertTrue(r["rows"][1]["break_before"])
        self.assertEqual(r["gaps"][0]["device_gap_ms"], 600)

    def test_duplicate_history_deduplicated_even_after_last_TM(self):
        batch = dict(host_utc="2026-10-27T00:00:00.2+00:00", direction="RX", frame_type="BT1", session=1, batch_id=2,
                     start_ms=1010, end_ms=1077, generated_ms=1000, outcome=1, sent_mask=15, ack_mask=15)
        batch.update({f"send_ms{w}": 1010+(w-1)*22 for w in range(1, 5)})
        batch.update({f"reply_ms{w}": 1011+(w-1)*22 for w in range(1, 5)})
        r = m.analyze(self.log([tm(1000, 1, 0), tm(1100, 2, .1), batch, batch]))
        self.assertEqual(len(r["batches"]), 1)
        self.assertEqual(r["batches"][0]["time_s"], .01)
        self.assertEqual(r["batches"][0]["call_ms"], [1, 1, 1, 1])

    def test_malformed_input_retained_as_error(self):
        p = self.log([tm(0, 0, 0), tm(-1, 1, .1)])
        with p.open("a", encoding="utf-8") as f:
            f.write("not json\n")
        r = m.analyze(p)
        self.assertEqual(len(r["rejected"]), 2)

    def test_no_telemetry_rejected(self):
        with self.assertRaisesRegex(ValueError, "no usable"):
            m.analyze(self.log([]))

    def test_out_of_range_block_rejected(self):
        p = self.log([tm(0, 0, 0)])
        for index in (-1, 1):
            with self.assertRaisesRegex(ValueError, "block index"):
                m.analyze(p, index)

    def test_compare_preserves_fault_and_ground_difference(self):
        p = self.log([tm(0, 0, 0, distance_progress_milli_mm=2097486, distance_fault_reason=9)], "motion_fault_or_cancelled")
        manifest = self.root / "manifest.json"
        m.save_json(manifest, {"runs": [{"label": "B2", "log": p.name, "ground": {"target_cm": 210, "forward_cm": 209.25, "lateral_cm_right_positive": .75}}]})
        out = self.root / "output"; out.mkdir()
        row = m.compare(manifest, out)[0]
        self.assertEqual(row["capture_status"], "motion_fault_or_cancelled")
        self.assertEqual(row["encoder_final_cm"], 209.7486)
        self.assertEqual(row["ground_error_cm"], -.75)
        self.assertIsNone(row["physical_stopped"])

    def test_manual_unknown_not_converted_to_zero(self):
        p = self.log([tm(0, 0, 0)])
        manifest = self.root / "manifest.json"
        m.save_json(manifest, {"runs": [{"label": "unknown", "log": p.name}]})
        out = self.root / "output"; out.mkdir()
        self.assertIsNone(m.compare(manifest, out)[0]["ground_lateral_cm_right_positive"])

    def test_manual_nan_rejected(self):
        p = self.log([tm(0, 0, 0)])
        manifest = self.root / "manifest.json"
        manifest.write_text(json.dumps({"runs": [{"label": "bad", "log": p.name, "ground": {"forward_cm": float('nan')}}]}))
        out = self.root / "output"; out.mkdir()
        with self.assertRaisesRegex(ValueError, "finite"):
            m.compare(manifest, out)

    def test_protected_outputs_rejected(self):
        for path in [m.ROOT / "3.stm32f407工程/test", m.ROOT / "工创赛/releases/test"]:
            with self.assertRaises(ValueError):
                m.safe_output(path)
        existing = self.root / "existing"; existing.mkdir()
        with self.assertRaises(FileExistsError):
            m.safe_output(existing)

    def test_report_embedded_text_escaped(self):
        r = m.analyze(self.log([tm(0, 0, 0)]))
        r["input"] = "</script><script>alert(1)</script>"
        page = m.chart_html(r)
        self.assertNotIn("</script><script>alert", page)
        self.assertNotIn("https://", page)
        self.assertNotIn("fetch(", page)

    @unittest.skipUnless(shutil.which("node"), "Node not installed; JS syntax test skipped")
    def test_generated_chart_javascript_syntax(self):
        r = m.analyze(self.log([tm(0, 0, 0)]))
        javascript = re.findall(r"<script>(.*?)</script>", m.chart_html(r), re.S)[0]
        script = self.root / "chart.js"; script.write_text(javascript, encoding="utf-8")
        p = subprocess.run([shutil.which("node"), "--check", str(script)], capture_output=True, text=True)
        self.assertEqual(p.returncode, 0, p.stderr)

    def config_root(self):
        for rel in ["3.stm32f407工程/USER/app_config.h", "3.stm32f407工程/USER/app_entry.h", "3.stm32f407工程/USER/chassis_auto_test.h",
                    "3.stm32f407工程/USER/route_config.h",
                    "工创赛/scripts/p1_serial_motion.py", "工创赛/docs/P1_SERIAL_MOTION_SCHEMA_2026-10-24.json"]:
            dest = self.root / rel; dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(m.ROOT / rel, dest)
        return self.root

    def test_configuration_consistent(self):
        self.assertTrue(m.check_config(self.config_root())["ok"])

    def test_config_detects_speed_mismatch(self):
        root = self.config_root()
        p = root / "3.stm32f407工程/USER/app_config.h"
        p.write_bytes(p.read_bytes().replace(b"APP_SPEED20_TRIAL_MM_S 200.0f", b"APP_SPEED20_TRIAL_MM_S 300.0f"))
        failures = [c["name"] for c in m.check_config(root)["checks"] if not c["ok"]]
        self.assertIn("APP_SPEED20_TRIAL_MM_S", failures)

    def test_config_detects_accidentally_enabled_route_switches(self):
        root = self.config_root()
        path = root / "3.stm32f407工程/USER/route_config.h"
        original = path.read_bytes()
        for name in [b"APP_ROUTE_USE_CUSTOM", b"APP_ROUTE_SPEED_TRIAL_ENABLE"]:
            path.write_bytes(original.replace(name+b" 0", name+b" 1"))
            self.assertFalse(m.check_config(root)["ok"])

    def test_config_detects_host_limit_mismatch_without_import(self):
        root = self.config_root()
        p = root / "工创赛/scripts/p1_serial_motion.py"
        p.write_bytes(p.read_bytes().replace(b'r["feedback_limit_rpm"] == 80', b'r["feedback_limit_rpm"] == 90'))
        self.assertFalse(m.check_config(root)["ok"])

    def test_literal_reader_does_not_execute_module(self):
        p = self.root / "module.py"
        p.write_text('raise RuntimeError("never execute")\nDATA={"a":7}\n')
        self.assertEqual(m.literal_assignment(p, "DATA"), {"a": 7})

    def test_package_hash_tamper_detected(self):
        root = self.config_root()
        package = root / "package"
        shutil.copytree(root / "3.stm32f407工程", package / "firmware")
        (package / "host").mkdir()
        shutil.copyfile(root / "工创赛/scripts/p1_serial_motion.py", package / "host/p1_serial_motion.py")
        config = package / "firmware/USER/app_config.h"
        config.write_bytes(config.read_bytes().replace(b'APP_DISTANCE_SCURVE_ENABLE 0U', b'APP_DISTANCE_SCURVE_ENABLE 1U').replace(b'APP_SPEED20_TRIAL_ENABLE 0U', b'APP_SPEED20_TRIAL_ENABLE 1U'))
        axf = package / "firmware/Objects/gongxun-f407.axf"; axf.parent.mkdir(); axf.write_bytes(b"fixture, not ARM code")
        expected = m.sha256(axf)
        m.save_json(package / "manifest.json", {"files": {"firmware/Objects/gongxun-f407.axf": expected}, "axf_sha256": expected})
        self.assertTrue(m.check_config(root, package)["ok"])
        axf.write_bytes(b"changed fixture")
        self.assertFalse(m.check_config(root, package)["ok"])

    def test_analyzer_cannot_mutate_input(self):
        p = self.log([tm(0, 0, 0), tm(100, 1, .1)])
        before = m.sha256(p)
        out = self.root / "out"; out.mkdir()
        m.export_analysis(p, out)
        self.assertEqual(m.sha256(p), before)

    def test_offline_modules_do_not_import_hardware_client(self):
        for name in ["p1_motion_offline.py", "p1_four_wheel_sim.py", "p1_validate_offline.py", "p1_simulation_report.py"]:
            tree = ast.parse((Path(m.__file__).parent / name).read_text(encoding="utf-8"))
            imports = [n.module for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)]
            imports += [a.name for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names]
            self.assertNotIn("p1_serial_motion", imports)
            self.assertNotIn("serial", imports)
            self.assertNotIn("ctypes", imports)

    def test_validation_failure_reported_and_later_stages_run(self):
        import p1_validate_offline as v
        out = self.root / "out"; out.mkdir()
        fake = subprocess.CompletedProcess([], 1, "", "deliberate unit failure")
        with redirect_stdout(io.StringIO()), patch.object(v, "protected_hashes", return_value={"source": "same"}), patch.object(v.subprocess, "run", return_value=fake), patch.object(v, "check_config", return_value={"ok": True, "checks": []}):
            r = v.validate(out)
        self.assertEqual(r["status"], "failed")
        self.assertEqual(r["checks"][-1]["status"], "passed")
        self.assertTrue((out / "validation.json").exists())

    def test_validation_skips_never_claims_full_pass(self):
        import p1_validate_offline as v
        out = self.root / "out"; out.mkdir()
        fake = subprocess.CompletedProcess([], 0, "", "OK")
        with redirect_stdout(io.StringIO()), patch.object(v, "protected_hashes", return_value={}), patch.object(v.subprocess, "run", return_value=fake), patch.object(v, "check_config", return_value={"ok": True, "checks": []}):
            self.assertEqual(v.validate(out)["status"], "passed_with_skips")

    def test_validation_detects_protected_change(self):
        import p1_validate_offline as v
        out = self.root / "out"; out.mkdir()
        fake = subprocess.CompletedProcess([], 0, "", "OK")
        with redirect_stdout(io.StringIO()), patch.object(v, "protected_hashes", side_effect=[{"x": "a"}, {"x": "b"}]), patch.object(v.subprocess, "run", return_value=fake), patch.object(v, "check_config", return_value={"ok": True, "checks": []}):
            r = v.validate(out)
        self.assertEqual(r["status"], "failed")
        self.assertEqual(r["checks"][-1]["changed"], ["x"])


class SimulationTraceTests(unittest.TestCase):
    def sample(self, **changes):
        from p1_simulation_report import TRACE_FIELDS, decode_sample
        data = dict.fromkeys(TRACE_FIELDS, 0.0)
        data.update(case_id=2, ref_acceleration_limit=24, ref_jerk_limit=31, ref_duration_s=20,
                    deceleration_start_s=10, target_mm=2100, state=10, result=2, cut_vm_request=1)
        data.update(changes)
        return decode_sample([data[key] for key in TRACE_FIELDS])

    def test_trace_schema_and_nonfinite_rejected(self):
        from p1_simulation_report import TRACE_FIELDS, decode_sample
        for values in ([0.0], [float('nan')]*len(TRACE_FIELDS)):
            with self.assertRaises(ValueError):
                decode_sample(values)

    def test_trace_units_and_cached_sign_preserved(self):
        row = self.sample(ref_speed_mm_s=200, ref_acceleration_mm_s2=20, ref_jerk_mm_s3=-30, cached_rpm2=-49)
        self.assertEqual(row['ref_v_cm_s'], 20)
        self.assertEqual(row['ref_a_cm_s2'], 2)
        self.assertEqual(row['ref_j_cm_s3'], -3)
        self.assertEqual(row['cached_rpm2'], -49)

    def test_expected_fault_is_distinct_from_arrival(self):
        from p1_simulation_report import check_trace
        review = check_trace([self.sample()], 2)
        self.assertTrue(review['ok'])
        self.assertEqual(review['metrics']['final_result'], 2)
        self.assertFalse(check_trace([self.sample(result=1)], 2)['ok'])

    def test_nominal_requires_all_stop_ack_and_cached_zero(self):
        from p1_simulation_report import check_trace
        rows = [self.sample(case_id=0, ref_speed_mm_s=200, state=6, result=0, cut_vm_request=0),
                self.sample(case_id=0, time_s=.01, state=9, result=1, reached=1, stop_mask=15)]
        self.assertTrue(check_trace(rows, 0)['ok'])
        rows[-1]['cached_rpm1'] = 1
        self.assertFalse(check_trace(rows, 0)['ok'])
        rows[-1]['cached_rpm1'] = 0
        rows[-1]['stop_mask'] = 7
        self.assertFalse(check_trace(rows, 0)['ok'])

    def test_trace_time_and_bounds_detected(self):
        from p1_simulation_report import check_trace
        for rows in ([self.sample(), self.sample()], [self.sample(ref_speed_mm_s=210)],
                     [self.sample(ref_jerk_mm_s3=-32)], [self.sample(sent_rpm1=55)]):
            self.assertFalse(check_trace(rows, 2)['ok'])

    def test_terminal_base_zero_before_arrival_detected(self):
        from p1_simulation_report import check_trace
        row = self.sample(time_s=11, state=6, result=0, base_reference_rpm=0)
        self.assertIn('base reference dropped below 1RPM before terminal arrival', check_trace([row], 2)['errors'])

    def test_stop_request_not_mistaken_for_sent_stop(self):
        from p1_simulation_report import check_trace
        rows = [self.sample(case_id=9, time_s=3, state=6, result=0, cut_vm_request=0, stop_requested=1),
                self.sample(case_id=9, time_s=3.01, stop_requested=1)]
        review = check_trace(rows, 9)
        self.assertTrue(review['ok'])
        labels = [m['label'] for m in review['markers']]
        self.assertTrue(any('后端停车请求' in label for label in labels))
        self.assertNotIn('进入逐轮STOP', labels)

    def test_base_update_does_not_invent_CAN_transaction(self):
        from p1_simulation_report import check_trace
        rows = [self.sample(base_reference_rpm=10), self.sample(time_s=.01, base_reference_rpm=11)]
        self.assertEqual(check_trace(rows, 2)['metrics']['max_speed_calls_per_10ms'], 0)

    def test_missing_trace_never_passes(self):
        from p1_simulation_report import check_trace
        self.assertFalse(check_trace([], 0)['ok'])

    def test_trace_export_explicitly_labels_simulation(self):
        from p1_simulation_report import export_traces
        with tempfile.TemporaryDirectory() as temp:
            out = Path(temp)
            report = export_traces(out, [self.sample()])
            self.assertFalse(report['ok'])  # Other nine cases are absent.
            self.assertEqual(report['source'], 'software_simulation_only')
            page = (out / 'case_02/report.html').read_text(encoding='utf-8')
            self.assertIn('车辆未运行', page)
            self.assertIn('"step"', page)
            self.assertTrue((out / 'case_02/trace.csv').exists())


if __name__ == "__main__":
    unittest.main()
