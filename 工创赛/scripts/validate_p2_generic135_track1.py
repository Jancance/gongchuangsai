"""跟踪参考点候选的实拍回放、时效/歧义/当前几何检查与完整功能验证。"""
import collections
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
from unittest.mock import patch
import zipfile

import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'output/p2_vision_generic135_lightblue1_20261027/vision_app'
APP=ROOT/'output/p2_vision_generic135_track1_20261027/vision_app'
TOOL=ROOT/'output/p2_generic135_track1_audit_20261027'
OUT=ROOT/'evidence/p2-camera/2026-10-27/track1'


def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def main():
    OUT.mkdir(exist_ok=True)
    sys.path.insert(0,str(APP/'code'))
    cv2.setNumThreads(8)
    from project_pale_tracking import prepare_tracking_regions
    from project_follow import MaterialFollower
    import project_follow as follower
    import project_detector as detector
    from 颜色标定 import _mask_from_hsv
    cfg=json.loads((APP/'code/config/material_color_thresholds.json').read_text(encoding='utf8'))
    off=copy.deepcopy(cfg);off['pale_tracking_reference_candidate']['enabled']=False
    original=json.loads((BASE/'code/config/material_color_thresholds.json').read_text(encoding='utf8'))
    stripped=copy.deepcopy(cfg);stripped.pop('pale_tracking_reference_candidate');assert stripped==original
    assert (APP/'code/project_detector.py').read_bytes()==(BASE/'code/project_detector.py').read_bytes()
    old_component=load('original_pale_component',BASE/'code/project_pale_component.py')
    new_component=load('new_pale_component',APP/'code/project_pale_component.py')
    old_follower=load('original_pale_follower',BASE/'code/project_follow.py')
    report={'checks':{},'saved_image_results':[],'hardware_acceptance':False,'motion_allowed':False}
    files=[]
    for folder in ('blue-pair-diagnostic','blue1-board-diagnostic','perf1-board-diagnostic','rejection-audit'):
        files.extend((folder+'/'+p.name,p.read_bytes()) for p in (ROOT/'evidence/p2-camera/2026-10-26'/folder).glob('*raw.png'))
    for folder in ('blue2/run-20261027-175625-750469','lightblue/run-20261027-181512-402452','lightblue/run-20261027-183210-098652'):
        files.extend((folder+'/'+p.name,p.read_bytes()) for p in (ROOT/'evidence/p2-camera/2026-10-27'/folder).glob('*raw.png'))
    for archive in ('20261024-121400-134643','20261024-120512-675071','20261024-114659-072282',
                    '20261024-112215-162612','20261024-111235-577533','20261024-104324-197600',
                    '20261024-105943-475482','20261007-000604-409723'):
        with zipfile.ZipFile(ROOT/'evidence'/(archive+'.zip')) as z:
            files.extend((archive+'/'+n,z.read(n)) for n in z.namelist() if n.endswith('-raw.png'))
    files.extend((p.name,p.read_bytes()) for p in (ROOT/'evidence/six_color_crosscheck_2026-09-24/inputs').glob('*.png'))
    seen=set(); current_points=[]; availability=collections.Counter()
    for name,data in files:
        digest=hashlib.sha256(data).hexdigest()
        if digest in seen:continue
        seen.add(digest);frame=cv2.imdecode(np.frombuffer(data,np.uint8),1)
        raw,_=detector.analyze(frame,cfg,draw_overlay=False);regions=raw['regions'];before=copy.deepcopy(regions)
        disabled,diagnostic=prepare_tracking_regions(frame,regions,off)
        assert disabled is regions and diagnostic is None
        tracking,diagnostic=prepare_tracking_regions(frame,regions,cfg)
        assert regions==before
        assert [r for r in tracking if r.get('label')!='lightblue']==[r for r in regions if r.get('label')!='lightblue']
        pale=[r for r in regions if r.get('label')=='lightblue']; tracked=[r for r in tracking if r.get('label')=='lightblue']
        if len(pale)==1:
            availability['has_canonical' if tracked else 'no_canonical']+=1
            report['saved_image_results'].append({'file':name,'raw_source':pale[0].get('proposal_source'),
                                                 'diagnostic':diagnostic,'track_present':bool(tracked)})
        if name.startswith('lightblue/run-20261027-183210-098652/'):
            assert len(tracked)==1,('current run canonical missing',name)
            current_points.append(tracked[0]['silhouette_center_px'])
            # 完整follower验证：原检测器区域不变；跟踪输出用统一中心。
            with patch.object(follower.time,'monotonic',return_value=100.01):
                obj=MaterialFollower(cfg);result=obj.detect(frame,timestamp=100.)
            assert result['detector_regions']==regions
            target=next(t for t in result['targets'] if t['color']=='lightblue')
            np.testing.assert_allclose(target['center_px'],tracked[0]['silhouette_center_px'])
            assert target['proposal_source']=='current_pale_silhouette_tracking'
            # 禁用候选时两个完整follower在同一可控时钟下结果逐字段一致。
            with patch.object(follower.time,'monotonic',return_value=100.01):
                a=old_follower.MaterialFollower(original).detect(frame,timestamp=100.)
                b=MaterialFollower(off).detect(frame,timestamp=100.)
            assert a==b
        # 补充分量分类的默认判定保持；只在跟踪显式allow_round。
        hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        masks={int(item['color_id']):_mask_from_hsv(hsv,item['ranges']) for item in cfg['colors']}
        combined=np.zeros(frame.shape[:2],np.uint8)
        for mask in masks.values():cv2.bitwise_or(combined,mask,dst=combined)
        parents,_=cv2.findContours(combined,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        assert old_component.recover_pale_component(hsv,masks,parents,[])==new_component.recover_pale_component(hsv,masks,parents,[])
    report['unique_real_images']=len(seen)
    report['canonical_availability_with_unique_raw_pale']=dict(availability)
    report['current_run_saved_image_count']=len(current_points)
    report['current_run_saved_center_range_xy_px']=np.ptp(np.array(current_points),axis=0).tolist()
    report['checks']['raw_detection_preserved_and_disabled_results_exact']=True
    report['checks']['original_component_classification_unchanged']=True
    print('real replay',len(seen),'availability',dict(availability),'current saved range',report['current_run_saved_center_range_xy_px'],flush=True)
    # 合成当前几何检查：不通过平滑掩盖真实移动，缺帧/歧义/不相交均拒绝坐标。
    raw={'label':'lightblue','bbox_px':[50,70,500,220],'area_px2':20000.,'silhouette_center_px':[200.,180.],
         'proposal_source':'chromatic_circle','radius_px':55.}
    raw_result={'regions':[raw],'circle_diagnostics':[]}
    frames=[]
    color=tuple(int(x) for x in cv2.cvtColor(np.uint8([[[102,130,190]]]),cv2.COLOR_HSV2BGR)[0,0])
    for x in (200,220):
        f=np.full((360,640,3),240,np.uint8);cv2.ellipse(f,(x,180),(82,50),0,0,360,color,-1);frames.append(f)
    a,_=prepare_tracking_regions(frames[0],[raw],cfg);b,_=prepare_tracking_regions(frames[1],[raw],cfg)
    assert b[0]['silhouette_center_px'][0]-a[0]['silhouette_center_px'][0]>19
    for kind in ('blank','two_components','disjoint','duplicate','round'):
        f=frames[0].copy();rr=[raw]
        if kind=='blank':f[:]=240
        if kind=='two_components':cv2.ellipse(f,(470,180),(82,50),0,0,360,color,-1)
        if kind=='disjoint':rr=[dict(raw,bbox_px=[500,10,30,30])]
        if kind=='duplicate':rr=[raw,dict(raw)]
        if kind=='round':f[:]=240;cv2.circle(f,(200,180),50,color,-1)
        tr,diag=prepare_tracking_regions(f,rr,cfg)
        if kind=='duplicate':assert tr is rr
        elif kind=='round':assert len(tr)==1
        else:assert not tr
    with patch.object(follower,'analyze',return_value=(raw_result,None)):
        obj=MaterialFollower(cfg)
        with patch.object(follower.time,'monotonic',return_value=100.01):obj.detect(frames[0],timestamp=100.)
        with patch.object(follower.time,'monotonic',return_value=100.11),patch.object(follower,'analyze',return_value=(raw_result,None)) as analyzer:
            result=obj.detect(frames[1],timestamp=100.1)
            hint=analyzer.call_args.kwargs['prior_regions'][0]
            assert hint['bbox_px']==raw['bbox_px'] and hint['proposal_source']==raw['proposal_source']
            assert hint['center_px']==raw['silhouette_center_px']
        assert result['targets'][0]['geometry_reference_changed'] is False
        with patch.object(follower.time,'monotonic',side_effect=[100.21,100.22,100.351]):
            stale=obj.detect(frames[1],timestamp=100.2)
        assert stale['state']=='STALE_RESULT' and not stale['targets'] and not obj.tracker.tracks
        assert obj.raw_pale_hint is None
        with patch.object(follower.time,'monotonic',return_value=101.):
            stale=obj.detect(frames[1],timestamp=100.5)
        assert stale['state']=='STALE_FRAME' and not stale['targets']
    report['checks']['current_motion_no_smoothing_no_ghost_duplicate_disjoint_round']=True
    report['checks']['detector_search_hints_keep_raw_geometry']=True
    report['checks']['150ms_input_and_final_result_reset']=True
    sample=ROOT/'evidence/p2-camera/2026-10-27/lightblue/run-20261027-183210-098652/first-frame-raw.png'
    calls=[('check',[sys.executable,'-X','utf8',str(APP/'run.py'),'--check']),
           ('audit-check',[sys.executable,'-X','utf8',str(TOOL/'p2_color_follow_audit.py'),'--app',str(APP),'--check']),
           ('audit-tests',[sys.executable,'-X','utf8','-m','unittest','discover','-s',str(ROOT/'tests'),'-p','test_color_follow_audit.py'])]
    for mode in ('follow','turntable','material','ring','stack','measurement'):
        args=[sys.executable,'-X','utf8',str(APP/'run.py'),'--image',str(sample),'--mode',mode,'--output-dir',str(OUT/mode)]
        if mode=='measurement':args+=['--measure-points','500','620','1400','620']
        calls.append((mode,args))
    for feature in ('color','turntable-settings','pickup-regions','stack-lines'):
        calls.append((feature,[sys.executable,'-X','utf8',str(APP/'run.py'),'--feature',feature,'--help']))
    for label,args in calls:
        proc=subprocess.run(args,capture_output=True,text=True,encoding='utf8',timeout=60)
        (OUT/(label+'.log')).write_text(proc.stdout+proc.stderr,encoding='utf8')
        assert proc.returncode==0,(label,proc.stdout,proc.stderr)
        report['checks'][label]=True
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    manifest=json.loads((APP/'TRACK1_MANIFEST.json').read_text())
    manifest['changed_files']=sorted(n for n,d in manifest['source_files'].items() if sha(APP/n)!=d)
    assert manifest['changed_files']==['code/config/material_color_thresholds.json','code/project_follow.py','code/project_pale_component.py']
    for n,d in manifest['source_files'].items():assert sha(BASE/n)==d
    manifest['candidate_files']={n:sha(APP/n) for n in manifest['changed_files']+manifest['added_files']}
    (APP/'TRACK1_MANIFEST.json').write_text(json.dumps(manifest,indent=2),encoding='utf8')
    for folder in (APP,TOOL):
        archive=folder.parent.with_suffix('.zip') if folder==APP else folder.with_suffix('.zip')
        assert not archive.exists()
        relative=APP.parent if folder==APP else folder
        with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as z:
            for p in sorted(folder.rglob('*')):
                if p.is_file() and '__pycache__' not in p.parts and p.suffix!='.pyc':z.write(p,p.relative_to(relative).as_posix())
        with zipfile.ZipFile(archive) as z:
            assert z.testzip() is None
            expected={p.relative_to(relative).as_posix() for p in folder.rglob('*') if p.is_file() and '__pycache__' not in p.parts and p.suffix!='.pyc'}
            assert set(z.namelist())==expected
            for n in z.namelist():assert z.read(n)==(relative/n).read_bytes()
        report[archive.name+'_sha256']=sha(archive)
    report['checks']['both_zip_crc_member_set_and_bytes']=True
    (OUT/'validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
    print(json.dumps({'checks':report['checks'],'images':len(seen)},ensure_ascii=False))


if __name__=='__main__':
    main()
