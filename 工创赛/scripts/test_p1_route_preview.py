import math
from pathlib import Path
import tempfile
import unittest

from p1_route_preview import USER, f32, preview, read_config, validate


class RoutePreviewTests(unittest.TestCase):
    geometry = (77.0, 212.0, 206.0)

    def check_error(self, rows, code, segment=1):
        result = validate(rows, self.geometry)
        self.assertFalse(result['ok'])
        self.assertEqual((result['error'], result['segment']), (code, segment))

    def test_live_example_is_inactive_and_converts_units(self):
        result = preview()
        self.assertTrue(result['ok'])
        self.assertFalse(result['custom_enabled'])
        self.assertEqual([r['rpm'] for r in result['rows']], [12, 7, 4])
        self.assertEqual(result['budget_ms'], 105500)
        self.assertFalse(result['hardware_accessed'])

    def test_range_and_unknown_action(self):
        for kind in ['FORWARD', 'LEFT', 'RIGHT']:
            for amount in [-1, 0, 9.9, 100.1, math.inf, math.nan]:
                self.check_error([(kind, amount, 5)], 3)
        self.check_error([('BACK', 30, 5)], 2)
        for amount in [0, 4.9, 90.1, -90.1, math.nan]:
            self.check_error([('TURN', amount, 5)], 3)

    def test_speed_rejects_nonfinite_negative_and_high_profile(self):
        for kind, amount in [('FORWARD', 30), ('TURN', 90)]:
            for speed in [-1, 0, .1, 20, math.inf, math.nan]:
                self.check_error([(kind, amount, speed)], 4)

    def test_integer_quantization_does_not_round_up(self):
        result = validate([('FORWARD', 10, f32(4.0)), ('TURN', -30, f32(3.0))], self.geometry)
        self.assertTrue(result['ok'])
        self.assertEqual([x['rpm'] for x in result['rows']], [9, 2])
        for row in result['rows']:
            self.assertLessEqual(row['nominal_speed'], row['requested_speed'])

    def test_duration_and_total_budget(self):
        self.check_error([('FORWARD', 100, 1.3)], 5)
        self.check_error([('TURN', 90, 1.2)], 5)
        self.check_error([('FORWARD', 10, 5)] * 4, 6, 4)
        self.check_error([], 1, 0)
        self.check_error([('FORWARD', 10, 5)] * 9, 1, 0)

    def test_geometry(self):
        for geometry in [(0, 212, 206), (77, math.nan, 206), (77, 212, math.inf)]:
            self.assertEqual(validate([('FORWARD', 30, 5)], geometry)['error'], 7)

    def test_strict_parser_does_not_ignore_missing_continuation(self):
        original = (USER / 'route_config.h').read_bytes().decode('gbk')
        changes = [original.replace('30.0f', '30.0f + 10.0f'),
                   original.replace('5.0f) \\\r\n', '5.0f)\r\n', 1),
                   original + '\nX(RIGHT, 30, 5)\n',
                   original.replace('APP_ROUTE_USE_CUSTOM 0', 'APP_ROUTE_USE_CUSTOM 2')]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'route.h'
            for text in changes:
                path.write_bytes(text.encode('gbk'))
                with self.assertRaises(ValueError):
                    read_config(path)

    def test_on_switch_and_negative_turn(self):
        original = (USER / 'route_config.h').read_bytes().decode('gbk')
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'route.h'
            path.write_bytes(original.replace('APP_ROUTE_USE_CUSTOM 0', 'APP_ROUTE_USE_CUSTOM 1')
                             .replace('90.0f', '-90.0f').encode('gbk'))
            result = preview(path)
            self.assertTrue(result['ok'])
            self.assertTrue(result['custom_enabled'])
            self.assertEqual(result['rows'][-1]['amount'], -90)

    def test_candidate_three_directions_and_short_segment_peak(self):
        for kind in ['FORWARD', 'LEFT', 'RIGHT']:
            short = validate([(kind, 10, 20)], self.geometry, candidate=True)
            long = validate([(kind, 210, 20)], self.geometry, candidate=True)
            self.assertTrue(short['ok'])
            self.assertTrue(long['ok'])
            self.assertLess(short['rows'][0]['peak_cm_s'], 20)
            self.assertAlmostEqual(long['rows'][0]['peak_cm_s'], 20, places=3)
            self.assertEqual(long['rows'][0]['reference_mm_s'], 200)
            self.assertLessEqual(long['rows'][0]['rpm'], 52)

    def test_candidate_caps_are_independent(self):
        caps = {'FORWARD': 20, 'LEFT': 15, 'RIGHT': 10}
        self.assertTrue(validate([('FORWARD', 30, 20)], self.geometry, candidate=True, caps=caps)['ok'])
        self.assertFalse(validate([('LEFT', 30, 20)], self.geometry, candidate=True, caps=caps)['ok'])
        self.assertFalse(validate([('RIGHT', 30, 15)], self.geometry, candidate=True, caps=caps)['ok'])
        self.assertTrue(validate([('RIGHT', 30, 10)], self.geometry, candidate=True, caps=caps)['ok'])

    def test_candidate_dynamic_budget_retains_total_cutoff(self):
        self.assertTrue(validate([('FORWARD', 10, 20)] * 7, self.geometry, candidate=True)['ok'])
        self.assertEqual(validate([('FORWARD', 210, 20)] * 8, self.geometry, candidate=True)['error'], 6)
        self.assertEqual(validate([('FORWARD', 210, 1.21)], self.geometry, candidate=True)['error'], 5)

    def test_candidate_rejects_invalid_distance_and_speed(self):
        for kind in ['FORWARD', 'LEFT', 'RIGHT']:
            for amount in [0, 9.9, 210.1, math.nan, math.inf]:
                self.assertEqual(validate([(kind, amount, 20)], self.geometry, candidate=True)['error'], 3)
            for speed in [0, 1.2, 20.1, math.nan, math.inf]:
                self.assertEqual(validate([(kind, 30, speed)], self.geometry, candidate=True)['error'], 4)

    def test_candidate_preview_does_not_enable_live_switches(self):
        before = (USER / 'route_config.h').read_bytes()
        report = preview(candidate=True)
        self.assertTrue(report['candidate_preview'])
        self.assertFalse(report['custom_enabled'])
        self.assertFalse(report['trial_switch'])
        self.assertEqual((USER / 'route_config.h').read_bytes(), before)

    def test_candidate_settings_are_strict(self):
        original = (USER / 'route_config.h').read_bytes().decode('gbk')
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'route.h'
            for modified in [original.replace('MAX_CM_S    20.0f', 'MAX_CM_S    30.0f'),
                             original.replace('APP_ROUTE_SPEED_TRIAL_ENABLE 0', 'APP_ROUTE_SPEED_TRIAL_ENABLE 2')]:
                path.write_bytes(modified.encode('gbk'))
                with self.assertRaises(ValueError):
                    preview(path)


if __name__ == '__main__':
    unittest.main()
