"""从完整blue2复制独立浅蓝投影候选；不覆盖原发布或修改板端。"""
from pathlib import Path
import hashlib
import json
import shutil

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'output/p2_vision_generic135_blue2_20261026/vision_app'
APP = ROOT / 'output/p2_vision_generic135_lightblue1_20261027/vision_app'
HELPER = '''"""本帧浅蓝投影分量候选；中心是轮廓中心，不是圆面或标定抓点。"""
import cv2
import numpy as np


def recover_pale_component(hsv, masks, rejected, accepted):
    if not rejected:
        return []
    height, width = hsv.shape[:2]
    # 复用已有S189浅蓝证据范围；只在补充分支内使用，不修改六色主蒙版。
    pale = cv2.inRange(hsv, (90, 25, 100), (115, 189, 255))
    contours, _ = cv2.findContours(pale, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for contour in contours:
        if cv2.contourArea(contour) < 1000:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        if x <= 1 or y <= 1 or x+w >= width-1 or y+h >= height-1:
            continue
        m = cv2.moments(contour)
        point = (m['m10']/m['m00'], m['m01']/m['m00'])
        if not any(cv2.pointPolygonTest(p, point, False) >= 0 for p in rejected):
            continue
        if any(cv2.pointPolygonTest(p, point, False) >= 0 for p in accepted):
            continue
        # 与已有蓝投影分支相同的7x7局部闭合，限制在本分量框内。
        local = np.zeros((h+8, w+8), np.uint8)
        local[4:4+h, 4:4+w] = pale[y:y+h, x:x+w]
        local = cv2.morphologyEx(local, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
        shapes, _ = cv2.findContours(local, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not shapes:
            continue
        shape = max(shapes, key=cv2.contourArea)
        area = cv2.contourArea(shape)
        perimeter = cv2.arcLength(shape, True)
        circularity = 4*np.pi*area/max(perimeter**2, 1.)
        solidity = area/max(cv2.contourArea(cv2.convexHull(shape)), 1.)
        bx, by, bw, bh = cv2.boundingRect(shape)
        # 工程候选，沿用蓝投影的严格形状/纯度标准，必须经独立实拍验证。
        if not (1.4 < max(bw/bh, bh/bw) <= 1.8 and circularity >= .60 and solidity >= .94):
            continue
        roi = np.zeros(local.shape, np.uint8)
        cv2.drawContours(roi, [shape], -1, 255, -1)
        roi = cv2.erode(roi, np.ones((7, 7), np.uint8))[4:4+h, 4:4+w] > 0
        crop = hsv[y:y+h, x:x+w]
        valid = roi & (crop[:, :, 2] < 250)
        usable = float(valid.sum()/max(roi.sum(), 1))
        if usable < .35 or valid.sum() < 500:
            continue
        names = {1:'red', 2:'yellow', 3:'blue', 4:'green', 5:'black', 6:'lightblue'}
        palette = dict(masks)
        palette[6] = pale
        palette[3] = cv2.bitwise_and(masks[3], cv2.bitwise_not(pale))
        ranges = {names[k]: mask[y:y+h, x:x+w] > 0 for k,mask in palette.items()}
        scores = {name: float(mask[valid].mean()) for name, mask in ranges.items()}
        if scores['lightblue'] < .90 or scores['lightblue']-max(v for k,v in scores.items() if k != 'lightblue') < .65:
            continue
        m = cv2.moments(shape)
        center = [x-4+m['m10']/m['m00'], y-4+m['m01']/m['m00']]
        candidates.append({
            'label': 'lightblue', 'reason': 'CURRENT_PROJECTED_PALE_COMPONENT',
            'proposal_source': 'projected_pale_component',
            'bbox_px': [x-4+bx, y-4+by, bw, bh], 'area_px2': float(area),
            'silhouette_center_px': center, 'circularity': float(circularity),
            'solidity': float(solidity), 'usable_fraction': usable,
            'color_fractions': scores,
            'center_reference': 'current_pale_silhouette_not_face_or_calibrated_grasp_point',
            'search_reference': 'current_frame_rejected_parent',
        })
    # 多个不同浅蓝分量不能合并或随意选一个；仍交由上层拒绝歧义。
    return candidates if len(candidates) == 1 else []
'''


def build():
    if APP.exists():
        raise RuntimeError('Refuse to overwrite existing candidate')
    shutil.copytree(BASE, APP, ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
    (APP / 'code/project_pale_component.py').write_text(HELPER, encoding='utf-8')
    (APP / 'LIGHTBLUE1_DEPLOY.md').write_text((ROOT/'docs/P2_CAMERA135_LIGHTBLUE1_2026-10-27.md').read_text(encoding='utf-8'), encoding='utf-8')
    path = APP / 'code/project_detector.py'
    source = path.read_text(encoding='utf-8')
    anchor = "        if color_config.get('black_edge_recovery_candidate',{}).get('enabled',False) and not any(r['label']=='black' for r in records):"
    assert source.count(anchor) == 1
    insert = '''        # 只有原有全部颜色/圆弧分支仍无浅蓝时，尝试本帧投影分量。
        if color_config.get('pale_projected_component_candidate',{}).get('enabled',False) and not any(r['label']=='lightblue' for r in records):
            from project_pale_component import recover_pale_component
            for row in recover_pale_component(hsv,calibrated_masks,rejected,accepted):
                records.append(row)
                if draw_overlay:
                    x,y,w,ht=row['bbox_px'];cv2.rectangle(drawing,(x,y),(x+w,y+ht),(0,255,0),2)
                    cv2.putText(drawing,'lightblue',(x,max(25,y-10)),cv2.FONT_HERSHEY_SIMPLEX,.8,(0,255,0),2)
'''
    source = source.replace(anchor, insert + anchor)
    path.write_text(source, encoding='utf-8')
    path = APP / 'code/config/material_color_thresholds.json'
    cfg = json.loads(path.read_text(encoding='utf-8'))
    cfg['pale_projected_component_candidate'] = {
        'enabled': True, 'requires_live_validation': True,
        'note': 'E candidate, missing-only current S189 pale projected component; aspect (1.4,1.8], circularity>=0.60, solidity>=0.94, purity>=0.90. Pixel silhouette center, NOT top face or grasp point. No old valid centers replaced; no arc threshold change. Disable to restore blue2.'}
    path.write_text(json.dumps(cfg, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    old = {p.relative_to(BASE).as_posix(): sha(p) for p in BASE.rglob('*') if p.is_file() and '__pycache__' not in p.parts and p.suffix != '.pyc'}
    changes = [n for n,d in old.items() if sha(APP/n) != d]
    manifest = {'base': str(BASE.relative_to(ROOT)), 'source_files': old,
                'changed_files': changes, 'added_files': ['code/project_pale_component.py','LIGHTBLUE1_DEPLOY.md'],
                'candidate_files': {n: sha(APP/n) for n in changes+['code/project_pale_component.py','LIGHTBLUE1_DEPLOY.md']},
                'motion_allowed': False, 'hardware_acceptance': False}
    assert sorted(changes) == ['code/config/material_color_thresholds.json', 'code/project_detector.py']
    (APP/'LIGHTBLUE1_MANIFEST.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    print(APP)


if __name__ == '__main__':
    build()
