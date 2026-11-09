"""135°蓝色投影轮廓候选；仅生成隔离目录，不修改已发布包。"""
from pathlib import Path
import hashlib
import json
import shutil
from zipfile import ZipFile, ZIP_DEFLATED

ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'output/p2_vision_generic135_20261026/vision_app'
APP=ROOT/'output/p2_vision_generic135_blue1_20261026/vision_app'

def replace(text,old,new):
    if old not in text:raise RuntimeError('Source mismatch: '+old[:70])
    return text.replace(old,new)

def build():
    # 允许恢复未改动的中断复制；任何已有候选修改都拒绝覆盖。
    if APP.exists():
        for p in APP.rglob('*'):
            if p.is_file() and '__pycache__' not in p.parts and p.suffix!='.pyc':
                q=SOURCE/p.relative_to(APP)
                if not q.is_file() or p.read_bytes()!=q.read_bytes():
                    raise RuntimeError('Candidate already modified: '+str(p))
    else:
        shutil.copytree(SOURCE,APP,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
    p=APP/'code/project_detector.py';text=p.read_text(encoding='utf-8')
    text=replace(text,'face_settings=None,hsv=None):','face_settings=None,hsv=None,blue_settings=None):')
    text=replace(text,'    for color_id in (1,2,3,4,6):','    for color_id in ((3,) if blue_settings else (1,2,3,4,6)):')
    text=replace(text,'''            if circularity<.60 or solidity<.90 or max(rw/rh,rh/rw)>1.4:
                continue
            roi=np.zeros((rh,rw),np.uint8)''','''            # 当前独立蓝色投影轮廓可比圆面更长；不改主轮廓/圆弧门。
            projected_blue = (color_id==3 and blue_settings and blue_settings.get('enabled',False)
                              and 1.4<max(rw/rh,rh/rw)<=1.8
                              and circularity>=.60 and solidity>=.95)
            if circularity<.60 or solidity<.90 or (max(rw/rh,rh/rw)>1.4 and not projected_blue):
                continue
            roi=np.zeros((rh,rw),np.uint8)''')
    text=replace(text,'''            moment=cv2.moments(refined)
            center=[moment['m10']/moment['m00'],moment['m01']/moment['m00']]''','''            if projected_blue and (scores['blue']<.90 or scores['blue']-scores[ranking[1]]<.65):
                continue
            moment=cv2.moments(refined)
            center=[moment['m10']/moment['m00'],moment['m01']/moment['m00']]''')
    text=replace(text,"""            if label=='blue':
                conflict=conflicting_pale_evidence""","""            if projected_blue:
                row.update(reason='CURRENT_PROJECTED_BLUE_SILHOUETTE',
                           proposal_source='projected_blue_component',
                           center_reference='current_blue_silhouette_not_calibrated_grasp_point')
            if label=='blue':
                conflict=conflicting_pale_evidence""")
    old='        for row in hinted_color_circles(frame,calibrated_masks,rejected,records,prior_regions,v,circle_diagnostics,geometry_masks):'
    new='''        blue_settings=color_config.get('blue_projected_silhouette_candidate')
        if blue_settings and blue_settings.get('enabled',False) and not any(r['label']=='blue' for r in records):
            # 使用本帧原蓝色蒙版，不合并白色高光或印刷黑环，不靠历史蓝框恢复。
            for row in chromatic_fallback(frame,calibrated_masks,rejected,accepted,
                    calibrated_masks,color_config.get('round_face_color_candidate'),hsv=hsv,blue_settings=blue_settings):
                records.append(row)
                if draw_overlay:
                    x,y,w,ht=row['bbox_px'];cv2.rectangle(drawing,(x,y),(x+w,y+ht),(0,255,0),2)
                    cv2.putText(drawing,row['label'],(x,max(25,y-10)),cv2.FONT_HERSHEY_SIMPLEX,.8,(0,255,0),2)
'''+old
    text=replace(text,old,new);p.write_text(text,encoding='utf-8')
    p=APP/'code/config/material_color_thresholds.json';cfg=json.loads(p.read_text(encoding='utf-8'))
    cfg['blue_projected_silhouette_candidate']={'enabled':True,'requires_live_validation':True,
        'note':'E: current isolated blue silhouette, aspect (1.4,1.8], circularity>=0.60, solidity>=0.95, blue fraction>=0.90; not a grasp point'}
    p.write_text(json.dumps(cfg,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    shutil.copy2(ROOT/'docs/P2_GENERIC135_BLUE1_2026-10-26.md',APP/'BLUE1_DEPLOY.md')
    hashes={str(p.relative_to(SOURCE)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest() for p in SOURCE.rglob('*') if p.is_file() and '__pycache__' not in p.parts and p.suffix!='.pyc'}
    changed=[n for n,h in hashes.items() if hashlib.sha256((APP/n).read_bytes()).hexdigest()!=h]
    (APP/'BLUE1_MANIFEST.json').write_text(json.dumps({'base':'135 candidate 2026-10-26',
        'changed_files':changed,'source_files':hashes,'motion_allowed':False,
        'note':'Only detector proposal and opt-in flag changed. 150ms and HSV unchanged; hardware acceptance pending.'},indent=2),encoding='utf-8')
    archive=APP.parent.with_suffix('.zip')
    with ZipFile(archive,'w',ZIP_DEFLATED) as z:
        for p in sorted(APP.rglob('*')):
            if p.is_file() and '__pycache__' not in p.parts and p.suffix!='.pyc':z.write(p,p.relative_to(APP.parent).as_posix())
    print(archive,hashlib.sha256(archive.read_bytes()).hexdigest())

if __name__=='__main__':build()
