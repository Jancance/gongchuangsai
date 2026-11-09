"""Check actual A/M startup and the tool-to-detector HSV save/load path offline."""
from pathlib import Path
import importlib
import json
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
APP = ROOT/'output/source-workflow-20261105/vision_app'
sys.path.insert(0, str(APP/'code'))
colors = importlib.import_module('颜色标定')
main = importlib.import_module('main')
checks = 0
with tempfile.TemporaryDirectory(prefix='source_hsv_', dir=ROOT/'tmp') as folder:
    path = Path(folder)/'material_color_thresholds.json'
    document = colors.load_color_config(APP/'code/config/material_color_thresholds.json')
    document['colors'][1]['ranges'][0]['lower'][1] = 91
    colors.save_color_config(path, document)
    saved = colors.load_color_config(path)
    assert saved['colors'][1]['ranges'][0]['lower'][1] == 91
    checks += 1
    sys.argv = ['main.py', '--material-color-config', str(path)]
    args = main.parse_args()
    main.initialize(args)
    assert main.current_mode == main.Mode.TURNTABLE
    assert main.material_pickup_detector.color_config == saved
    assert main.serial_link.status == 'DISABLED_P2_INTEGRATION_PENDING'
    checks += 3
    # Original tool defaults and main defaults both refer to this app's file.
    assert colors.DEFAULT_CONFIG.resolve() == (APP/'code/config/material_color_thresholds.json').resolve()
    sys.argv = ['main.py']
    assert main.parse_args().material_color_config.resolve() == colors.DEFAULT_CONFIG.resolve()
    checks += 2
    main.handle_key(ord('m'))
    assert main.current_mode == main.Mode.MATERIAL_PICKUP
    main.handle_key(ord('a'))
    assert main.current_mode == main.Mode.TURNTABLE
    checks += 2
out = ROOT/'evidence/integration/2026-11-05/source-workflow'
out.mkdir(parents=True, exist_ok=True)
report = dict(checks=checks, failures=0, hsv_tool_save_and_actual_M_detector_load=True,
              default_A_and_M_key=True, serial_disabled=True, camera_opened=False,
              hardware_acceptance=False)
(out/'offline-checks.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
print(json.dumps(report))
