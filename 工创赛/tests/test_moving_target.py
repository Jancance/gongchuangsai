import unittest

from smart_carrier.moving_target import (
    MovingTargetPredictor, PredictorLimits, TargetMeasurement,
)


LIMITS = PredictorLimits(
    max_host_age_ms=150, min_interval_s=.02, max_interval_s=.25,
    max_target_speed_mm_s=200, max_acceleration_mm_s2=500,
    max_horizon_s=.25, max_position_error_bound_mm=5,
)


def obs(ms, x, *, identity="track-1", source="main", calibration="cal-1", **changes):
    fields=dict(track_id=identity, source_id=source, calibration_id=calibration,
                timestamp_ns=ms*1_000_000, field_xy_mm=(x, 50.),
                position_error_bound_mm=.2, coordinate_frame="field",
                timestamp_source="sensor", ego_motion_compensated=True,
                identity_valid=True, visible=True)
    fields.update(changes)
    return TargetMeasurement(**fields)


class MovingTargetTests(unittest.TestCase):
    def run_sample(self, predictor, measurement, *, horizon=.1, bounds=(0., 0., 100., 100.)):
        return predictor.update(measurement, now_ns=measurement.timestamp_ns + 10_000_000,
                                contact_horizon_s=horizon, workspace_bounds_mm=bounds)

    def test_predicts_forward_without_waiting_for_stop(self):
        p=MovingTargetPredictor(LIMITS)
        self.assertEqual(self.run_sample(p, obs(1000, 10)).status, "UNKNOWN")
        self.assertEqual(self.run_sample(p, obs(1100, 12)).status, "UNKNOWN")
        g=self.run_sample(p, obs(1200, 14))
        self.assertEqual(g.status,"GUIDANCE_ONLY")
        self.assertAlmostEqual(g.predicted_field_xy_mm[0],16.)
        self.assertAlmostEqual(g.velocity_field_mm_s[0],20.)
        self.assertGreater(g.estimated_error_bound_mm,0)

    def test_host_read_timestamp_and_missing_ego_pose_fail_closed(self):
        p=MovingTargetPredictor(LIMITS)
        self.assertEqual(self.run_sample(p, obs(1000, 10, timestamp_source="host_read_completion")).reason,
                         "SENSOR_TIMESTAMP_NOT_VERIFIED")
        self.assertEqual(self.run_sample(p, obs(1100, 12, ego_motion_compensated=False)).reason,
                         "METRIC_FRAME_OR_EGO_COMPENSATION_MISSING")

    def test_identity_or_calibration_change_resets_history(self):
        p=MovingTargetPredictor(LIMITS)
        for ms,x in [(1000,10),(1100,12),(1200,14)]: self.run_sample(p,obs(ms,x))
        self.assertEqual(self.run_sample(p,obs(1300,16,identity="track-2")).reason,
                         "NEED_THREE_FRESH_OBSERVATIONS")
        self.assertEqual(self.run_sample(p,obs(1400,18,identity="track-2",calibration="cal-2")).reason,
                         "NEED_THREE_FRESH_OBSERVATIONS")

    def test_stale_gap_unknown_and_workspace_reject(self):
        p=MovingTargetPredictor(LIMITS)
        self.assertEqual(p.update(obs(1000,10),now_ns=1_200_000_000,
                                  contact_horizon_s=.1,workspace_bounds_mm=(0,0,100,100)).reason,
                         "STALE_OR_FUTURE_MEASUREMENT")
        self.run_sample(p,obs(1000,10));self.run_sample(p,obs(1100,12))
        self.assertEqual(self.run_sample(p,obs(1400,18)).reason,"NEED_THREE_FRESH_OBSERVATIONS")
        self.run_sample(p,obs(1500,20))
        self.assertEqual(self.run_sample(p,obs(1600,22),bounds=(0,0,23,100)).reason,
                         "PREDICTION_OUTSIDE_WORKSPACE")

    def test_bad_motion_or_uncertainty_never_produces_guidance(self):
        p=MovingTargetPredictor(LIMITS)
        self.run_sample(p,obs(1000,10));self.run_sample(p,obs(1100,12))
        self.assertEqual(self.run_sample(p,obs(1200,90)).reason,"SPEED_OUT_OF_RANGE")
        self.assertEqual(self.run_sample(p,obs(1300,10),horizon=None).reason,"CONTACT_TIME_NOT_MEASURED")
        p=MovingTargetPredictor(LIMITS)
        self.run_sample(p,obs(1000,10,position_error_bound_mm=3))
        self.run_sample(p,obs(1100,12,position_error_bound_mm=3))
        self.assertEqual(self.run_sample(p,obs(1200,14,position_error_bound_mm=3)).reason,
                         "PREDICTION_ERROR_TOO_LARGE")

    def test_occlusion_rejects_old_track(self):
        p=MovingTargetPredictor(LIMITS)
        for ms,x in [(1000,10),(1100,12),(1200,14)]:self.run_sample(p,obs(ms,x))
        self.assertEqual(self.run_sample(p,obs(1300,16,visible=False)).reason,
                         "TARGET_NOT_VISIBLE_OR_IDENTITY_UNCERTAIN")
        self.assertEqual(self.run_sample(p,obs(1400,18)).reason,"NEED_THREE_FRESH_OBSERVATIONS")


if __name__=="__main__":unittest.main()
