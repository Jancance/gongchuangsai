"""Reproduce the old 720p clamp and validate full-frame selection offline."""
from pathlib import Path
import importlib.util
import json
import sys
import tempfile

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
CODE = ROOT / 'output/source-workflow-20261105/vision_app/code'
sys.path.insert(0, str(CODE))
spec = importlib.util.spec_from_file_location('regions_fixed', CODE / '物料抓取区域标定.py')
fixed = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixed)
# Use a separate namespace with __file__ for the preserved original source.
old_ns = {'__file__': str(CODE / '物料抓取区域标定.py'), '__name__': 'regions_old'}
exec(compile((CODE / '物料抓取区域标定.py.before-region-fix-20261105').read_text(encoding='utf-8'),
             old_ns['__file__'], 'exec'), old_ns)


def drag(selector, first, last):
    selector.mouse_callback(cv2.EVENT_LBUTTONDOWN, *first, 0, None)
    selector.mouse_callback(cv2.EVENT_MOUSEMOVE, *last, 0, None)
    selector.mouse_callback(cv2.EVENT_LBUTTONUP, *last, 0, None)


document = fixed.build_default_material_pickup_config()
legacy = old_ns['RegionSelector'](document)
legacy.draw(np.zeros((1080, 1920, 3), np.uint8))
drag(legacy, (0, 0), (1919, 1079))
assert legacy.rectangles[0][1] == dict(x=0, y=0, width=1279, height=719)
checks = 1
for width, height in [(1920, 1080), (1280, 720), (1280, 960)]:
    selector = fixed.RegionSelector(document)
    selector.draw(np.zeros((height, width, 3), np.uint8))
    drag(selector, (-20, -30), (width + 20, height + 30))
    assert selector.rectangles[0][1] == dict(x=0, y=0, width=width-1, height=height-1)
    selector.undo()
    assert selector.rectangles == []
    checks += 2
selector = fixed.RegionSelector(document)
drag(selector, (10, 10), (200, 200))
assert not selector.rectangles  # No image, no selection.
checks += 1
selector.draw(np.zeros((1080, 1920, 3), np.uint8))
for first, last in [((0, 0), (1919, 1079)), ((100, 200), (500, 600)),
                    ((1400, 200), (1850, 650)), ((750, 780), (1200, 1079))]:
    drag(selector, first, last)
assert selector.complete
checks += 1
with tempfile.TemporaryDirectory(dir=ROOT / 'tmp') as temp:
    config = Path(temp) / 'regions.json'
    assert selector.save(config)
    loaded = fixed.load_material_pickup_config(config)
    assert loaded['regions']['down']['y'] == 780
    assert loaded['regions']['right']['x'] == 1400
    assert loaded['calibrated']
    checks += 4
report = dict(checks=checks, failures=0, old_clamp_reproduced=[1279, 719],
              full_frame=[1920, 1080], lower_and_right_regions_saved_and_reloaded=True,
              camera_opened=False, motion_commands_sent=False, board_installed=False)
out = ROOT / 'evidence/integration/2026-11-05/source-workflow/region-bounds-checks.json'
out.write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
print(json.dumps(report))
