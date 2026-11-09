"""Isolated exact-output candidate: reuse current HSV in chromatic fallback."""
from pathlib import Path
import hashlib
import json
import shutil

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'output/p2_vision_generic120_black1_20261024/vision_app'
APP = ROOT / 'output/p2_vision_generic120_perf2_20261024/vision_app'


def build():
    if APP.exists():
        raise FileExistsError('Preserve existing candidate: ' + str(APP))
    shutil.copytree(BASE, APP, ignore=shutil.ignore_patterns('__pycache__', '*.pyc', 'diagnostics'))
    path = APP / 'code/project_detector.py'
    source = path.read_text(encoding='utf-8')
    replacements = [
        ('def chromatic_fallback(frame, masks, rejected, accepted,geometry_masks=None,face_settings=None):',
         'def chromatic_fallback(frame, masks, rejected, accepted,geometry_masks=None,face_settings=None,hsv=None):'),
        ('    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)\n    value=hsv[:,:,2]\n',
         '    # 复用本帧主检测的HSV，保留蓝/浅蓝圆面分支所需完整通道。\n'
         '    if hsv is None:\n'
         '        hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)\n'
         '    value=hsv[:,:,2]\n'),
        ("chromatic_fallback(frame,calibrated_masks,rejected,accepted,geometry_masks,color_config.get('round_face_color_candidate')):",
         "chromatic_fallback(frame,calibrated_masks,rejected,accepted,geometry_masks,color_config.get('round_face_color_candidate'),hsv=hsv):"),
    ]
    for old, new in replacements:
        if source.count(old) != 1:
            raise RuntimeError('Unexpected base source: ' + old)
        source = source.replace(old, new)
    path.write_text(source, encoding='utf-8')
    manifest = {'base': 'black1', 'changed_file': 'code/project_detector.py',
                'before_sha256': hashlib.sha256((BASE/'code/project_detector.py').read_bytes()).hexdigest(),
                'after_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                'scope': 'Reuse same-frame HSV only; standalone fallback unchanged',
                'hardware_acceptance': False, 'motion_allowed': False}
    (APP/'PERF2_MANIFEST.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    return manifest


if __name__ == '__main__':
    print(json.dumps(build(), indent=2))
