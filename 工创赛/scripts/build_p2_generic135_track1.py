"""独立像素跟踪坐标候选：保留检测结果，跟踪使用统一的本帧浅蓝投影中心。"""
from pathlib import Path
import hashlib
import json
import shutil

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT/'output/p2_vision_generic135_lightblue1_20261027/vision_app'
APP = ROOT/'output/p2_vision_generic135_track1_20261027/vision_app'
TOOL = ROOT/'output/p2_generic135_track1_audit_20261027'

HELPER = '''"""把颜色检出与像素跟踪参考点分开；不输出顶面、毫米或抓点。"""
import cv2
import numpy as np
from project_pale_component import recover_pale_component


def prepare_tracking_regions(frame, regions, config):
    settings = (config or {}).get('pale_tracking_reference_candidate', {})
    if not settings.get('enabled', False):
        return regions, None
    pale = [r for r in regions if r.get('label') == 'lightblue']
    if len(pale) != 1:
        # 原始同色歧义继续交给原跟踪器，不能先挑一个再统一坐标。
        return regions, {'reason':'NO_UNIQUE_PALE_DETECTION','detected_count':len(pale)}
    from 颜色标定 import _mask_from_hsv
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    masks = {int(item['color_id']):_mask_from_hsv(hsv,item['ranges']) for item in config['colors']}
    combined = np.zeros(frame.shape[:2], np.uint8)
    for mask in masks.values():
        cv2.bitwise_or(combined, mask, dst=combined)
    combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, np.ones((5,5),np.uint8))
    parents,_ = cv2.findContours(combined,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    candidates = recover_pale_component(hsv,masks,parents,[],allow_round=True)
    # 缺少本帧唯一投影证据时仍保留颜色检出，但撤销该帧浅蓝跟踪坐标。
    # 不回退到另一种中心、不输出历史位置，也不重写原检测器区域。
    if len(candidates) != 1:
        return [r for r in regions if r.get('label') != 'lightblue'], {'reason':'NO_UNIQUE_CURRENT_PALE_TRACKING_COMPONENT'}
    geometry = candidates[0]
    raw = pale[0]
    x,y,w,h = raw['bbox_px']
    gx,gy,gw,gh = geometry['bbox_px']
    overlap = max(0,min(x+w,gx+gw)-max(x,gx))*max(0,min(y+h,gy+gh)-max(y,gy))
    if overlap/max(min(w*h,gw*gh),1) < .5:
        return [r for r in regions if r.get('label') != 'lightblue'], {'reason':'PALE_DETECTION_TRACKING_COMPONENT_DISJOINT'}
    row = dict(raw)
    for key in ('bbox_px','area_px2','silhouette_center_px','center_reference'):
        row[key] = geometry[key]
    row['proposal_source'] = 'current_pale_silhouette_tracking'
    row.pop('radius_px', None)
    diagnostic = {'reason':'CURRENT_PALE_TRACKING_COMPONENT_VALIDATED',
                  'tracking_reference':'current_pale_silhouette_not_face_or_grasp',
                  'detector_proposal_source':raw.get('proposal_source'),
                  'detector_center_px':raw['silhouette_center_px'],
                  'tracking_center_px':geometry['silhouette_center_px'],
                  'tracking_bbox_px':geometry['bbox_px']}
    return [row if r is raw else r for r in regions], diagnostic
'''


def main():
    if APP.exists() or TOOL.exists():
        raise RuntimeError('Refuse to overwrite candidate')
    shutil.copytree(BASE,APP,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
    (APP/'code/project_pale_tracking.py').write_text(HELPER,encoding='utf8')
    component = APP/'code/project_pale_component.py'
    source = component.read_text(encoding='utf8')
    source = source.replace('def recover_pale_component(hsv, masks, rejected, accepted):', 'def recover_pale_component(hsv, masks, rejected, accepted, allow_round=False):')
    source = source.replace('1.4 < max(bw/bh, bh/bw) <= 1.8', '(0.999 if allow_round else 1.4) < max(bw/bh, bh/bw) <= 1.8')
    component.write_text(source,encoding='utf8')
    path = APP/'code/project_follow.py'
    s = path.read_text(encoding='utf8')
    anchor = "        tracked=self.tracker.update(detection['regions'],now)"
    replacement = '''        from project_pale_tracking import prepare_tracking_regions
        tracking_regions,tracking_diagnostic=prepare_tracking_regions(frame,detection['regions'],self.color_config)
        tracked=self.tracker.update(tracking_regions,now)'''
    assert s.count(anchor)==1
    s=s.replace(anchor,replacement)
    s=s.replace('self.tracker=TabletopTracker();self.history={}', 'self.tracker=TabletopTracker();self.history={};self.raw_pale_hint=None')
    s=s.replace('self.tracker.clear();self.history.clear()', 'self.tracker.clear();self.history.clear();self.raw_pale_hint=None')
    old_hints="        hints=[dict(t) for t in self.tracker.tracks.values() if 0<=now-t['read_time']<=.5]"
    new_hints="""        coordinate_mode=bool((self.color_config or {}).get('pale_tracking_reference_candidate',{}).get('enabled',False))
        hints=[dict(self.raw_pale_hint if coordinate_mode and t['color']=='lightblue' and self.raw_pale_hint is not None else t)
               for t in self.tracker.tracks.values() if 0<=now-t['read_time']<=.5]"""
    assert s.count(old_hints)==1
    s=s.replace(old_hints,new_hints)
    point="        from project_pale_tracking import prepare_tracking_regions"
    extra="""        if coordinate_mode:
            raw=[r for r in detection['regions'] if r.get('label')=='lightblue']
            if len(raw)==1:
                r=raw[0]
                self.raw_pale_hint={'color':'lightblue','bbox_px':r['bbox_px'],'center_px':r['silhouette_center_px'],
                                    'proposal_source':r.get('proposal_source','combined_silhouette'),'radius_px':r.get('radius_px')}
            elif len(raw)>1:self.raw_pale_hint=None
"""
    assert s.count(point)==1
    s=s.replace(point,extra+point)
    old = "            source=next((r.get('proposal_source','combined_silhouette') for r in detection['regions']\n                         if r['label']==t['color']),'combined_silhouette')"
    assert s.count(old)==1
    s=s.replace(old,"            source=t.get('proposal_source','combined_silhouette')")
    anchor = "        # 跟踪/预测/组装也占时间，最终交付前再次检查，不延长原150ms门。"
    assert s.count(anchor)==1
    s=s.replace(anchor,"        if tracking_diagnostic is not None:result['tracking_coordinate_diagnostics']=tracking_diagnostic\n"+anchor)
    path.write_text(s,encoding='utf8')
    path=APP/'code/config/material_color_thresholds.json'
    cfg=json.loads(path.read_text(encoding='utf8'))
    cfg['pale_tracking_reference_candidate']={'enabled':True,'requires_live_validation':True,
        'note':'E pixel tracking only: unique current S189 projected silhouette center for every lightblue track, independent of detector source. Keep raw detector regions; missing canonical geometry suppresses current track, never falls back to another center. NOT top face or grasp point. Disable to restore lightblue1.'}
    path.write_text(json.dumps(cfg,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    (APP/'TRACK1_DEPLOY.md').write_text('''# track1独立像素坐标候选

完整继承lightblue1功能与文件，当前以TRACK1_MANIFEST.json为准。仅在follow层统一浅蓝跟踪参考点为本帧整体投影中心，原检测器区域保留。投影证据缺失时不输出该帧浅蓝跟踪坐标，不混入圆心/旧帧位置。颜色是否检出与跟踪坐标是否有效分开记录；本坐标不是顶面/毫米/抓点。关闭pale_tracking_reference_candidate.enabled恢复lightblue1。150ms/500ms门、禁用串口/动作不变。

独立解压到/home/orangepi/p2-generic135-track1-20261027，先run.py --check。使用独立track1-audit入口，仍--watch-color lightblue，同场景约90秒取证；出现误报/时效恶化保留证据，回退原lightblue1目录。具体检查与未闭环见项目docs/P2_CAMERA135_TRACK1_2026-10-27.md。
''',encoding='utf8')
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    sources={p.relative_to(BASE).as_posix():sha(p) for p in BASE.rglob('*') if p.is_file() and '__pycache__' not in p.parts and p.suffix!='.pyc'}
    changed=sorted(n for n,d in sources.items() if sha(APP/n)!=d)
    assert sorted(changed)==['code/config/material_color_thresholds.json','code/project_follow.py','code/project_pale_component.py']
    added=['code/project_pale_tracking.py','TRACK1_DEPLOY.md']
    manifest={'base':str(BASE.relative_to(ROOT)),'source_files':sources,'changed_files':changed,'added_files':added,
              'candidate_files':{n:sha(APP/n) for n in changed+added},'hardware_acceptance':False,'motion_allowed':False}
    (APP/'TRACK1_MANIFEST.json').write_text(json.dumps(manifest,indent=2),encoding='utf8')
    diagnostic_base=ROOT/'output/p2_generic135_rejection_audit_20261026'
    shutil.copytree(diagnostic_base,TOOL,ignore=shutil.ignore_patterns('__pycache__','*.pyc','MANIFEST.json'))
    p=TOOL/'p2_color_follow_audit.py'
    text=p.read_text(encoding='utf8')
    anchor="    hashes['diagnostic_entry']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()"
    assert text.count(anchor)==1
    text=text.replace(anchor,"    for name in ('project_pale_component.py','project_pale_tracking.py'):\n        if (code/name).is_file():hashes[name]=hashlib.sha256((code/name).read_bytes()).hexdigest()\n"+anchor)
    p.write_text(text,encoding='utf8')
    (TOOL/'MANIFEST.json').write_text(json.dumps({'source':'rejection-audit 2026-10-26','changes':'Record both pale helper module hashes; recording behavior unchanged.',
        'files':{p.name:sha(p) for p in TOOL.glob('*.py')}},indent=2),encoding='utf8')
    print(APP)


if __name__=='__main__':
    main()
