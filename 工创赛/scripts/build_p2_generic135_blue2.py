"""从perf1生成独立蓝色凸实度边界候选，保留全部原发布文件。"""
from pathlib import Path
import hashlib
import json
import shutil

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'output/p2_vision_generic135_perf1_20261026/vision_app'
APP = ROOT / 'output/p2_vision_generic135_blue2_20261026/vision_app'


def build():
    if APP.exists():
        raise RuntimeError('Refuse to overwrite existing candidate')
    shutil.copytree(BASE, APP, ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
    path = APP / 'code/project_detector.py'
    source = path.read_text(encoding='utf-8')
    old = 'and circularity>=.60 and solidity>=.95)'
    new = '''and circularity>=.60
                              and solidity>=(.94 if blue_settings.get('solidity94_candidate',False) else .95))'''
    assert source.count(old) == 1
    source = source.replace(old, new)
    old_call = 'hsv=hsv,blue_settings=blue_settings,pale_cache=pale_cache):'
    assert source.count(old_call) == 1
    # 原0.95分支和原圆弧分支先执行，原本有效的中心来源保持不变。
    source = source.replace(old_call, "hsv=hsv,blue_settings=dict(blue_settings,solidity94_candidate=False),pale_cache=pale_cache):")
    anchor = "        if color_config.get('black_edge_recovery_candidate',{}).get('enabled',False) and not any(r['label']=='black' for r in records):"
    assert source.count(anchor) == 1
    source = source.replace(anchor, '''        # 仅当原投影/圆弧分支均无蓝色时，补查当前帧凸实度边界候选。
        if blue_settings and blue_settings.get('enabled',False) and blue_settings.get('solidity94_candidate',False) and not any(r['label']=='blue' for r in records):
            for row in chromatic_fallback(frame,calibrated_masks,rejected,accepted,
                    calibrated_masks,color_config.get('round_face_color_candidate'),hsv=hsv,blue_settings=blue_settings,pale_cache=pale_cache):
                records.append(row)
                if draw_overlay:
                    x,y,w,ht=row['bbox_px'];cv2.rectangle(drawing,(x,y),(x+w,y+ht),(0,255,0),2)
                    cv2.putText(drawing,row['label'],(x,max(25,y-10)),cv2.FONT_HERSHEY_SIMPLEX,.8,(0,255,0),2)
''' + anchor)
    path.write_text(source, encoding='utf-8')
    path = APP / 'code/config/material_color_thresholds.json'
    config = json.loads(path.read_text(encoding='utf-8'))
    config['blue_projected_silhouette_candidate']['solidity94_candidate'] = True
    config['blue_projected_silhouette_candidate']['solidity94_note'] = (
        'E blue2 candidate: solidity >=0.94 instead of historical 0.95 only for current '
        'projected blue silhouette; retains aspect, circularity, purity and all other gates. '
        'Observed false rejection 0.949852; independent live validation pending. '
        'Runs only after original strict and hinted-circle branches both fail to find blue. '
        'Disable this flag to restore perf1 exactly. Historical note above describes blue1.')
    path.write_text(json.dumps(config, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    source_files = {p.relative_to(BASE).as_posix(): sha(p) for p in BASE.rglob('*')
                    if p.is_file() and '__pycache__' not in p.parts and p.suffix != '.pyc'}
    changed = [name for name, digest in source_files.items() if sha(APP / name) != digest]
    assert sorted(changed) == ['code/config/material_color_thresholds.json', 'code/project_detector.py']
    manifest = {'base': 'generic135 perf1 2026-10-26', 'source_files': source_files,
                'changed_files': changed, 'candidate_files': {name: sha(APP / name) for name in changed},
                'motion_allowed': False, 'hardware_acceptance': False,
                'scope': 'Engineering blue projected silhouette solidity94 opt-in; not a general shape gate change.'}
    (APP / 'BLUE2_MANIFEST.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    print(APP)


if __name__ == '__main__':
    build()
