"""Actual MATLAB/replay comparison and motion-admission boundaries."""
import json
from pathlib import Path
import unittest
from smart_carrier.vision_manipulation.turntable_phase import (
    IndexedDiscPhase, StopEvent, actual_rise_clearance, confirmed_target_region,
)

ROOT = Path(__file__).resolve().parents[1]


class TurntablePhaseTests(unittest.TestCase):
    def test_duplicate_ready_burst_does_not_break_direction_learning(self):
        phase = IndexedDiscPhase('disc')
        colors = (5, 1, 0)
        for i in range(4):
            self.assertTrue(phase.record_stop(StopEvent(i + 1, i * 5.7, colors), context_id='disc',
                                             remaining_colors=(1, 5), after_motion=True))
            self.assertFalse(phase.record_stop(StopEvent(i + 2, i * 5.7 + .1, colors), context_id='disc',
                                              remaining_colors=(1, 5), after_motion=True))
            colors = (colors[1], colors[2], colors[0])
        self.assertIsNotNone(phase.predict_next(17.2, remaining_colors=(1, 5)))

    def test_actual_recording_matches_matlab_predictions(self):
        evidence = ROOT / 'evidence/integration'
        proof = json.loads((evidence / '2026-11-14/turntable-prediction/matlab-result.json').read_text(encoding='utf-8'))
        data = json.loads((evidence / '2026-11-13/source-auto-index-observe-20261113-a/matlab-cadence-result.json').read_text(encoding='utf-8'))
        windows = data['complete_gate_windows_at_least_150ms'][1:]
        phase = IndexedDiscPhase('disc-regions')
        predictions = []
        for i, row in enumerate(windows):
            if i >= 4:
                prediction = phase.predict_next(windows[i - 1][0], remaining_colors=(2, 1, 5))
                self.assertIsNotNone(prediction)
                self.assertEqual(prediction.colors, tuple(map(int, row[3:6])))
                self.assertFalse(prediction.starts_grasp)
                predictions.append(prediction.stop_start_s)
            self.assertTrue(phase.record_stop(StopEvent(i + 1, row[0], tuple(map(int, row[3:6]))),
                                             context_id='disc-regions', remaining_colors=(2, 1, 5), after_motion=True))
        for actual, expected in zip(predictions, proof['predicted_stop_start_s']):
            self.assertAlmostEqual(actual, expected, places=8)
        self.assertEqual(len(predictions), len(proof['predicted_stop_start_s']))

    def test_two_then_one_remaining_are_real_targets(self):
        phase = IndexedDiscPhase('disc')
        colors = (2, 1, 5)
        for i in range(4):
            phase.record_stop(StopEvent(i + 1, i * 5.7, colors), context_id='disc',
                              remaining_colors=(2, 1, 5), after_motion=True)
            colors = (colors[1], colors[2], colors[0])
        colors = tuple(c if c != 2 else 0 for c in colors)
        self.assertTrue(phase.record_stop(StopEvent(5, 22.8, colors), context_id='disc',
                                         remaining_colors=(1, 5), after_motion=True))
        prediction = phase.predict_next(23, remaining_colors=(5,))
        self.assertEqual(sum(c != 0 for c in prediction.colors), 1)
        arguments = dict(target_color=5, observed_colors=(0, 5, 0), remaining_colors=(5,),
                         observed_s=23, now_s=23.1, stationary=True,
                         pose_id='low65', observed_pose_id='low65')
        self.assertEqual(confirmed_target_region(**arguments), 1)
        for changes in [dict(observed_colors=(0, 0, 0)), dict(observed_colors=(5, 5, 0)),
                        dict(now_s=24), dict(stationary=False), dict(observed_pose_id='high65')]:
            self.assertIsNone(confirmed_target_region(**(arguments | changes)))

    def test_old_events_gaps_and_reversal_do_not_keep_old_prediction(self):
        phase = IndexedDiscPhase('disc')
        colors = (2, 1, 5)
        for i in range(4):
            self.assertTrue(phase.record_stop(StopEvent(i + 1, i * 5.7, colors), context_id='disc',
                                             remaining_colors=(2, 1, 5), after_motion=True))
            colors = (colors[1], colors[2], colors[0])
        self.assertFalse(phase.record_stop(phase.events[-1], context_id='disc',
                                          remaining_colors=(2, 1, 5), after_motion=True))
        self.assertIsNone(phase.predict_next(100, remaining_colors=(2, 1, 5)))
        self.assertTrue(phase.record_stop(StopEvent(6, 30, colors), context_id='disc',
                                         remaining_colors=(2, 1, 5), after_motion=True))
        self.assertIsNone(phase.predict_next(30, remaining_colors=(2, 1, 5)))
        phase.reset('new-pose-map')
        self.assertIsNone(phase.predict_next(30, remaining_colors=(2, 1, 5)))

    def test_clearance_uses_fresh_measured_rise_and_strict_boundary(self):
        pick = 118800 * 360 / 3200
        for rise, clear in [(0, False), (64.9, False), (65, False), (65.01, True)]:
            current = pick - rise * 1600 * 360 / 3200
            measured, accepted = actual_rise_clearance(pick_position_deg=pick,
                                                      current_position_deg=current, feedback_age_ms=100)
            self.assertAlmostEqual(measured, rise)
            self.assertEqual(accepted, clear)
        self.assertEqual(actual_rise_clearance(pick_position_deg=pick, current_position_deg=0,
                                              feedback_age_ms=151), (None, False))


if __name__ == '__main__':
    unittest.main()
