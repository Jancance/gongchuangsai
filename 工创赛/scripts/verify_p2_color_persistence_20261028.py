"""在临时配置中验证现有调色保存及普通主程序重启加载，不打开硬件。"""
from copy import deepcopy
import hashlib
import importlib
import json
from pathlib import Path
import sys
import tempfile
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
APP = ROOT / 'output/p2_vision_generic135_pixel1_20261027/vision_app'


def main():
    sys.path.insert(0, str(APP/'code'))
    color = importlib.import_module('颜色标定')
    app = importlib.import_module('main')
    source = APP/'code/config/material_color_thresholds.json'
    original_bytes = source.read_bytes()
    original = color.load_color_config(source)
    candidate = deepcopy(original)
    green = next(item for item in candidate['colors'] if item['name'] == 'green')
    green['ranges'] = color.bounds_to_ranges((45, 80, 70, 255, 45, 255))
    assert candidate != original
    checks = []
    with tempfile.TemporaryDirectory(prefix='p2-color-', dir=ROOT/'tmp') as tmp:
        path = Path(tmp)/'field.json'
        color.save_color_config(path, candidate)
        assert color.load_color_config(path) == candidate
        checks.append('actual_save_load_roundtrip')
        assert all(a == b for a, b in zip(original['colors'], candidate['colors']) if a['name'] != 'green')
        assert {k:v for k,v in original.items() if k != 'colors'} == {k:v for k,v in candidate.items() if k != 'colors'}
        checks.append('other_five_colors_and_auxiliary_config_preserved')
        with patch.object(sys, 'argv', ['main.py', '--check', '--material-color-config', str(path)]), \
             patch.object(app.cv2, 'VideoCapture', side_effect=AssertionError('camera must stay closed')):
            args = app.parse_args()
            app.initialize(args)
            assert app.material_follower.color_config == candidate
            assert app.material_pickup_detector.color_config == candidate
            assert type(app.serial_link).__name__ == 'DisabledSerialLink'
            checks.append('ordinary_main_initialization_loads_saved_config_for_follow_and_material')
            updated = deepcopy(candidate)
            next(item for item in updated['colors'] if item['name'] == 'green')['ranges'] = color.bounds_to_ranges((46, 79, 71, 254, 46, 254))
            color.save_color_config(path, updated)
            assert app.material_follower.color_config == candidate
            checks.append('running_detector_requires_restart_to_load_new_values')
            app.initialize(args)
            assert app.material_follower.color_config == updated
            assert app.material_pickup_detector.color_config == updated
            checks.append('reinitialization_loads_second_saved_config')
        before = path.read_bytes()
        invalid = deepcopy(candidate)
        invalid['colors'][0]['ranges'][0]['lower'][0] = 200
        try:
            color.save_color_config(path, invalid)
        except color.ColorCalibrationError:
            pass
        else:
            raise AssertionError('invalid HSV config must be rejected')
        assert path.read_bytes() == before
        checks.append('invalid_save_leaves_previous_config_intact')
    assert source.read_bytes() == original_bytes
    report = dict(reviewed_on='2026-10-28', checks=checks, passed=len(checks),
                  source_config_sha256=hashlib.sha256(original_bytes).hexdigest(),
                  source_unchanged=True, camera_opened=False, serial='DISABLED',
                  board_gui_verified=False, three_minute_rehearsal_passed=False,
                  all_color_branches_configurable=False,
                  remaining='Fixed pale HSV evidence ranges remain in project_detector.py and project_pale_component.py; GUI tuning and full detector acceptance are separate.')
    out = ROOT/'evidence/p2-camera/2026-10-28/color-persistence'
    out.mkdir(parents=True, exist_ok=True)
    (out/'review.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf8')
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
