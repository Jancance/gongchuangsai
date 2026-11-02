import copy
import json
import math
import tempfile
import unittest
from pathlib import Path

from p1_perimeter_preview import DEFAULT, boundary_clearance, corners, critical_turn_angles, evaluate, write_preview


class PerimeterPreviewTests(unittest.TestCase):
    def setUp(self):
        self.config=json.loads(DEFAULT.read_text(encoding="utf-8"))

    def test_complete_pose_closure_and_long_sideways(self):
        r=evaluate(self.config)
        self.assertTrue(r["closed"])
        self.assertTrue(r["boundary_geometry_pass"])
        self.assertTrue(r["returned_footprint_inside_start_zone"])
        self.assertEqual(r["travel_cm"],780)
        self.assertEqual(len(r["rows"]),12)
        self.assertAlmostEqual(r["end_pose"][2],-90)
        for kind in ["LEFT","RIGHT"]:
            self.assertTrue(any(x["kind"]==kind and x["amount"]==180 for x in r["rows"]))

    def test_map_directions_change_after_rotation(self):
        rows=evaluate(self.config)["rows"]
        self.assertAlmostEqual(rows[3]["end_pose"][0],300)
        self.assertAlmostEqual(rows[3]["end_pose"][1],300)
        self.assertAlmostEqual(rows[6]["end_pose"][1],2100)
        self.assertAlmostEqual(rows[9]["end_pose"][0],2100)

    def test_execution_is_blocked_even_if_geometry_passes(self):
        r=evaluate(self.config)
        self.assertFalse(r["firmware_executable"])
        self.assertFalse(r["vehicle_approved"])
        self.assertGreater(len(r["rows"]),r["current_firmware_capacity"])
        self.assertGreater(r["current_style_conservative_budget_ms"],120000)

    def test_margin_tighter_than_estimate_fails(self):
        self.config["required_boundary_margin_mm"]=10
        r=evaluate(self.config)
        self.assertFalse(r["boundary_geometry_pass"])
        self.assertAlmostEqual(r["minimum_boundary_clearance_mm"],8.5)

    def test_larger_real_body_is_not_silently_accepted(self):
        self.config["footprint"]["length_mm"]=320
        r=evaluate(self.config)
        self.assertFalse(r["boundary_geometry_pass"])
        self.assertFalse(r["returned_footprint_inside_start_zone"])

    def test_broken_last_adjustment_is_detected(self):
        self.config["actions"][-1]["amount"]=16
        self.assertFalse(evaluate(self.config)["closed"])

    def test_inner_region_exemption_must_be_explicit(self):
        self.config["field"]["inner_regions_allowed_for_trial"]=False
        with self.assertRaises(ValueError):evaluate(self.config)

    def test_nan_unknown_direction_and_zero_turn_rejected(self):
        for key,value in [("kind","JUMP"),("amount",float("nan")),("speed",float("inf"))]:
            c=copy.deepcopy(self.config);c["actions"][0][key]=value
            with self.assertRaises(ValueError):evaluate(c)
        self.config["actions"][4]["amount"]=0
        with self.assertRaises(ValueError):evaluate(self.config)

    def test_rotation_between_endpoints_can_hit_boundary(self):
        # Start/end rectangles fit, but an intermediate corner protrudes.
        endpoints=boundary_clearance([[150,500,0],[150,500,90]],283,254,(2400,2400))
        self.assertGreater(endpoints["left_mm"],0)
        angles=critical_turn_angles(0,90,283,254)
        swept=boundary_clearance([[150,500,a] for a in angles],283,254,(2400,2400))
        self.assertLess(swept["left_mm"],0)
        self.assertAlmostEqual(swept["left_mm"],150-math.hypot(283,254)/2)

    def test_exact_rotation_extrema_cover_dense_sampling(self):
        for start in [-270,-180,-90,-17,0,31,90,170]:
            for delta in [-90,-45,45,90]:
                exact=boundary_clearance([[1000,1000,a] for a in critical_turn_angles(start,delta,283,254)],283,254,(2400,2400))
                dense=boundary_clearance([[1000,1000,start+delta*i/1000] for i in range(1001)],283,254,(2400,2400))
                for key in exact:
                    self.assertLessEqual(exact[key],dense[key]+1e-9)
                    self.assertLess(dense[key]-exact[key],0.001)

    def test_preview_artifacts_do_not_overwrite_evidence(self):
        with tempfile.TemporaryDirectory() as temp:
            out=Path(temp)/"preview"
            write_preview(DEFAULT,out)
            for name in ["route.html","route.svg","report.json"]:self.assertTrue((out/name).is_file())
            with self.assertRaises(ValueError):write_preview(DEFAULT,out)


if __name__=="__main__":unittest.main()
