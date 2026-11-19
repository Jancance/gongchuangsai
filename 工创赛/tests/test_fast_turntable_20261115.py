import json
import math
from pathlib import Path
import unittest

import numpy as np
from smart_carrier.vision_manipulation.fast_turntable import FastTurntable

R = Path(__file__).resolve().parents[1]
E = R / 'evidence/integration/2026-11-15/source-rim-reacquire'


class FastTests(unittest.TestCase):
    def setUp(self):
        assert json.loads((E / 'fast-observation/matlab-result.json').read_text())['passed']
        self.config = json.loads((E / 'fast-observation/orbit-config.json').read_text())

    def regions(self, t, omega, config=None):
        cfg = config or self.config
        result = []
        for color, offset in ((2, .3), (1, 2.4), (5, 4.5)):
            theta = omega * t + offset
            point = np.asarray(cfg['center_px']) + np.linalg.solve(
                cfg['whitening'], [math.cos(theta), math.sin(theta)])
            result.append(dict(color_id=color, center_x_px=point[0], center_y_px=point[1]))
        return result

    def test_second_frame_schedules_both_directions_without_a_turn(self):
        for omega in (-.36, .36):
            model = FastTurntable(self.config)
            self.assertIsNone(model.update(self.regions(0, omega), 0, 0, 2, [2, 1, 5]))
            result = model.update(self.regions(.1, omega), .1, .1, 2, [2, 1, 5])
            self.assertIsNotNone(result)
            self.assertFalse(result['learned_one_turn'])
            self.assertAlmostEqual(result['angular_speed_rad_s'], omega)
            for i, delay in enumerate(result['start_delay_ms']):
                theta = omega * (.1 + delay / 1000 + model.leads[i]) + .3
                target = model._phase(model.tool_points[i])
                self.assertAlmostEqual(math.atan2(math.sin(theta-target), math.cos(theta-target)), 0)

    def test_gap_duplicate_and_camera_motion_restart_only_the_pair(self):
        model = FastTurntable(self.config)
        model.update(self.regions(0, .36), 0, 0, 2, [2, 1, 5])
        self.assertIsNone(model.update(self.regions(1, .36), 1, 1, 2, [2, 1, 5]))
        self.assertIsNotNone(model.update(self.regions(1.1, .36), 1.1, 1.1, 2, [2, 1, 5]))
        duplicate = self.regions(1.2, .36)
        duplicate.append(duplicate[0])
        self.assertIsNone(model.update(duplicate, 1.2, 1.2, 2, [2, 1, 5]))
        self.assertIsNone(model.update(self.regions(1.3, .36), 1.3, 1.3, 2, [2, 1, 5]))
        self.assertIsNotNone(model.update(self.regions(1.4, .36), 1.4, 1.4, 2, [2, 1, 5]))
        model.pause()
        self.assertIsNone(model.update(self.regions(2, .36), 2, 2, 2, [2, 1, 5]))

    def test_recorded_frames_generate_predictions_without_whole_turn_reset(self):
        model = FastTurntable(self.config)
        rows = json.loads((E / 'field-no-pick/observations-summary.json').read_text())
        first = None
        predicted = 0
        for row in rows:
            if row['state'].get('state') != 6:
                continue
            stamp = row['observation_host_timestamp']
            first = stamp if first is None else first
            result = model.update(row['regions'], stamp, stamp, 2, [2, 1, 5])
            if result is not None:
                predicted += 1
                if predicted == 1:
                    self.assertLess(stamp - first, 1)
        self.assertGreater(predicted, 0)


if __name__ == '__main__':
    unittest.main()
