"""Build isolated black1 from ring1, retaining all controls and safety gates."""
from pathlib import Path
import shutil, json, hashlib

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT/'output/p2_vision_generic120_ring1_20261024/vision_app'
APP = ROOT/'output/p2_vision_generic120_black1_20261024/vision_app'


def build():
    shutil.copytree(BASE, APP, dirs_exist_ok=True,
                    ignore=shutil.ignore_patterns('__pycache__','*.pyc','diagnostics'))
    p = APP/'code/project_detector.py'
    source = p.read_text(encoding='utf-8')
    helper = (ROOT/'scripts/p2_black_face_candidate.py').read_text(encoding='utf-8')
    helper = helper[helper.index('def black_face_supported_hull'):]
    marker = 'def analyze(frame,color_config=None,prior_regions=None,draw_overlay=True):'
    assert source.count(marker) == 1
    source = source.replace(marker, helper+'\n\n'+marker)
    marker = "    if calibrated_masks is not None:\n        for row in records:\n"
    assert source.count(marker) == 1
    source = source.replace(marker, """    if calibrated_masks is not None and not any(r['label']=='black' for r in records):
        supported_black=black_face_supported_hull(frame,calibrated_masks,rejected,accepted,v,
            color_config.get('black_face_support_candidate'),circle_diagnostics)
        records.extend(supported_black)
        if draw_overlay:
            for row in supported_black:
                x,y,w,ht=row['bbox_px']
                cv2.rectangle(drawing,(x,y),(x+w,y+ht),(0,255,0),2)
                cv2.putText(drawing,'black',(x,max(25,y-10)),cv2.FONT_HERSHEY_SIMPLEX,.8,(0,255,0),2)
"""+marker)
    p.write_text(source, encoding='utf-8')
    p = APP/'code/config/material_color_thresholds.json'
    cfg = json.loads(p.read_text(encoding='utf-8'))
    cfg['black_face_support_candidate'] = {'enabled':True,
        'note':'E candidate: current dark circular face and edge evidence supports parent hull; no calibrated grasp point; board validation pending'}
    p.write_text(json.dumps(cfg,ensure_ascii=False,indent=2),encoding='utf-8')
    changes = []
    for n in ['code/project_detector.py','code/config/material_color_thresholds.json']:
        changes.append({'file':n,'before_sha256':hashlib.sha256((BASE/n).read_bytes()).hexdigest(),
                        'after_sha256':hashlib.sha256((APP/n).read_bytes()).hexdigest()})
    manifest = {'base':'ring1','changes':changes,'hsv_changed':False,'camera_changed':False,
                'time_gate_changed':False,'hardware_acceptance':False,'motion_allowed':False}
    (APP/'BLACK1_MANIFEST.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    return manifest


if __name__ == '__main__':
    print(json.dumps(build(),indent=2))
