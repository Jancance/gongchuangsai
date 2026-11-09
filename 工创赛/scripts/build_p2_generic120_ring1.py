"""构建圆环背景拒绝候选；不覆盖perf1或修改相机/HSV门限。"""
from pathlib import Path
import hashlib
import json
import shutil

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'output/p2_vision_generic120_perf1_20261023/vision_app'
APP=ROOT/'output/p2_vision_generic120_ring1_20261024/vision_app'

HELPER='''
def printed_ring_evidence(frame,black_mask,row,settings):
    """黑色物料候选中的多圈黑白交替证据；不依赖场地固定ROI或编号。"""
    if not settings or not settings.get('enabled',False):return None
    x,y,w,h=map(int,row['bbox_px'])
    height,width=frame.shape[:2]
    if min(w,h)<settings['minimum_axis_px'] or x<0 or y<0 or x+w>width or y+h>height:return None
    cx,cy=row['silhouette_center_px']
    yy,xx=np.ogrid[y:y+h,x:x+w]
    radius=np.sqrt(((xx-cx)/(w*.5))**2+((yy-cy)/(h*.5))**2)
    black=black_mask[y:y+h,x:x+w]>0
    core=radius<=settings['core_radius_ratio']
    core_fraction=float(black[core].mean()) if core.any() else 0.
    # 中心有充分连续黑色证据时，不能因载物下方外露的印刷圆环否决真黑料。
    if core_fraction>=settings['solid_core_black_fraction']:return None
    bins=settings['radial_bins']
    index=np.floor(radius*bins).astype(np.int32)
    counts=np.bincount(index.ravel(),minlength=bins+2)
    dark_counts=np.bincount(index.ravel(),weights=black.ravel(),minlength=bins+2)
    occupancy=dark_counts/np.maximum(counts,1)
    start=int(settings['annulus_inner_ratio']*bins)
    end=min(len(occupancy),int(settings['annulus_outer_ratio']*bins)+1)
    bands=0;separated=True
    for fraction in occupancy[start:end]:
        if fraction<=settings['light_gap_max_fraction']:separated=True
        elif fraction>=settings['dark_band_min_fraction'] and separated:
            bands+=1;separated=False
    if bands<settings['minimum_dark_bands']:return None
    return {'dark_band_count':bands,'core_black_fraction':core_fraction,
            'scope':'Current normalized radial black/white pattern; not a metric pose or digit identity'}

'''


def build():
    shutil.copytree(BASE,APP,dirs_exist_ok=True,ignore=shutil.ignore_patterns('__pycache__','*.pyc','diagnostics'))
    code=APP/'code'
    path=code/'project_detector.py';text=path.read_text(encoding='utf-8')
    marker='def analyze(frame,color_config=None,prior_regions=None,draw_overlay=True):'
    assert text.count(marker)==1
    text=text.replace(marker,HELPER+marker)
    marker='        records.append(row)\n        if row[\'label\']!=\'UNKNOWN\':accepted.append(contour)'
    assert text.count(marker)==1
    text=text.replace(marker,'''        if calibrated_masks is not None and row['label']=='black':
            evidence=printed_ring_evidence(frame,calibrated_masks[5],row,color_config.get('printed_ring_rejection_candidate'))
            if evidence is not None:
                row.update(label='UNKNOWN',reason='PRINTED_CONCENTRIC_RING_PATTERN',printed_ring_evidence=evidence)
                rejected.append(contour)
        records.append(row)
        if row['label']!='UNKNOWN':accepted.append(contour)''')
    # 严格边缘恢复保留；同样不能将有充分印刷纹理证据的黑色恢复当物料。
    marker="    return {'version':VERSION,'regions':records,'circle_diagnostics':circle_diagnostics,"
    assert text.count(marker)==1
    text=text.replace(marker,'''    if calibrated_masks is not None:
        for row in records:
            if row['label']!='black' or row.get('proposal_source')=='combined_silhouette':continue
            evidence=printed_ring_evidence(frame,calibrated_masks[5],row,color_config.get('printed_ring_rejection_candidate'))
            if evidence is not None:
                row.update(label='UNKNOWN',reason='PRINTED_CONCENTRIC_RING_PATTERN',printed_ring_evidence=evidence)
'''+marker)
    path.write_text(text,encoding='utf-8')
    path=code/'config/material_color_thresholds.json'
    cfg=json.loads(path.read_text(encoding='utf-8'))
    cfg['printed_ring_rejection_candidate']={
        'enabled':True,'minimum_axis_px':40,'radial_bins':80,
        'core_radius_ratio':0.5,'solid_core_black_fraction':0.65,
        'annulus_inner_ratio':0.5,'annulus_outer_ratio':1.03,
        'dark_band_min_fraction':0.45,'light_gap_max_fraction':0.15,
        'minimum_dark_bands':3,
        'note':'E candidate: reject multiple alternating concentric print bands only for black proposals; physical difficult-set validation pending'}
    path.write_text(json.dumps(cfg,ensure_ascii=False,indent=2),encoding='utf-8')
    changes=[]
    for name in ('code/project_detector.py','code/config/material_color_thresholds.json'):
        changes.append({'file':name,'before_sha256':hashlib.sha256((BASE/name).read_bytes()).hexdigest(),
                        'after_sha256':hashlib.sha256((APP/name).read_bytes()).hexdigest()})
    report={'base':'perf1','changes':changes,'hsv_thresholds_changed':False,'camera_controls_changed':False,
            'hardware_acceptance':False,'motion_allowed':False}
    (APP/'RING1_MANIFEST.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    readme=APP/'README.md'
    readme.write_text('> 2026-10-24：本目录为ring1印刷圆环背景拒绝候选，先读RING1_DEPLOY.md；下文保留原版本历史。\n\n'+readme.read_text(encoding='utf-8'),encoding='utf-8')
    return report


if __name__=='__main__':print(json.dumps(build(),ensure_ascii=False,indent=2))
