"""浅蓝缺失补充分支的实拍回放、负样本、保留功能及包完整性检查。"""
import collections
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import zipfile

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'output/p2_vision_generic135_blue2_20261026/vision_app'
APP = ROOT / 'output/p2_vision_generic135_lightblue1_20261027/vision_app'
OUT = ROOT / 'evidence/p2-camera/2026-10-27/lightblue'


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main():
    cv2.setNumThreads(8)
    sys.path.insert(0, str(APP/'code'))
    old = load('lightblue1_old', BASE/'code/project_detector.py')
    new = load('lightblue1_new', APP/'code/project_detector.py')
    read = lambda p: json.loads(p.read_text(encoding='utf-8'))
    cfg = read(APP/'code/config/material_color_thresholds.json')
    off = copy.deepcopy(cfg)
    off['pale_projected_component_candidate']['enabled'] = False
    original = read(BASE/'code/config/material_color_thresholds.json')
    only_added = copy.deepcopy(cfg)
    only_added.pop('pale_projected_component_candidate')
    assert only_added == original
    files = []
    for folder in ('blue-pair-diagnostic', 'blue1-board-diagnostic', 'perf1-board-diagnostic', 'rejection-audit'):
        files.extend((folder+'/'+p.name, p.read_bytes()) for p in (ROOT/'evidence/p2-camera/2026-10-26'/folder).glob('*raw.png'))
    for folder in ('blue2/run-20261027-175625-750469', 'lightblue/run-20261027-181512-402452'):
        files.extend((folder+'/'+p.name, p.read_bytes()) for p in (ROOT/'evidence/p2-camera/2026-10-27'/folder).glob('*raw.png'))
    for archive in ('20261024-121400-134643', '20261024-120512-675071', '20261024-114659-072282',
                    '20261024-112215-162612', '20261024-111235-577533', '20261024-104324-197600',
                    '20261024-105943-475482', '20261007-000604-409723'):
        with zipfile.ZipFile(ROOT/'evidence'/(archive+'.zip')) as z:
            files.extend((archive+'/'+n, z.read(n)) for n in z.namelist() if n.endswith('-raw.png'))
    files.extend((p.name,p.read_bytes()) for p in (ROOT/'evidence/six_color_crosscheck_2026-09-24/inputs').glob('*.png'))
    hints = [{'color':'blue','bbox_px':[348,511,265,170],'center_px':[480,596],'proposal_source':'projected_blue_component'},
             {'color':'lightblue','bbox_px':[1274,524,237,155],'center_px':[1388,604]}]
    report = {'changes': [], 'checks': {}, 'hardware_acceptance':False, 'motion_allowed':False}
    seen = set()
    for name, data in files:
        digest = hashlib.sha256(data).hexdigest()
        if digest in seen:
            continue
        seen.add(digest)
        frame = cv2.imdecode(np.frombuffer(data,np.uint8),1)
        for prior in (None, hints):
            a,_ = old.analyze(frame,original,prior_regions=prior,draw_overlay=False)
            disabled,_ = new.analyze(frame,off,prior_regions=prior,draw_overlay=False)
            assert disabled == a, ('disabled changed old result',name)
            b,_ = new.analyze(frame,cfg,prior_regions=prior,draw_overlay=False)
            if b != a:
                added = [r for r in b['regions'] if r not in a['regions']]
                assert not any(r['label']=='lightblue' for r in a['regions']), ('old valid pale changed',name)
                assert len(added)==1 and added[0]['proposal_source']=='projected_pale_component', (name,added)
                assert [r for r in b['regions'] if r not in added] == a['regions'], ('other results changed',name)
                assert b['circle_diagnostics'] == a['circle_diagnostics']
                report['changes'].append({'file':name,'history':prior is not None,'added':added})
        if name.endswith('first-nonstale-lightblue-missing-raw.png'):
            result,overlay = new.analyze(frame,cfg,draw_overlay=True)
            assert sorted(r['label'] for r in result['regions'] if r['label']!='UNKNOWN') == ['blue','lightblue']
            cv2.imencode('.png',overlay)[1].tofile(OUT/'lightblue1-recovered-overlay.png')
    report['unique_real_images'] = len(seen)
    report['checks']['disabled_full_results_equal_and_existing_valid_centers_preserved'] = True
    print('real-image replay passed', len(seen), 'changed combinations',len(report['changes']),flush=True)
    helper = load('pale_component_test', APP/'code/project_pale_component.py')
    for case in ('valid','border','elongated','round','concave','clipped','no_parent','accepted_parent','two_components','dark_blue','white','black','yellow','green','red'):
        frame = np.full((360,640,3),240,np.uint8)
        hsv_color = {'dark_blue':(110,230,180),'white':(0,0,240),'black':(0,0,15),'yellow':(28,220,190),'green':(60,220,170),'red':(0,220,190),'clipped':(102,130,255)}.get(case,(102,130,190))
        bgr = tuple(int(x) for x in cv2.cvtColor(np.uint8([[hsv_color]]),cv2.COLOR_HSV2BGR)[0,0])
        center = (60,180) if case=='border' else (200,180)
        axes = (110,35) if case=='elongated' else ((50,50) if case=='round' else (82,50))
        cv2.ellipse(frame,center,axes,0,0,360,bgr,-1)
        if case=='concave': cv2.rectangle(frame,(190,120),(220,180),(240,240,240),-1)
        if case=='two_components': cv2.ellipse(frame,(470,180),axes,0,0,360,bgr,-1)
        hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        # 用实际主配置生成所有竞争颜色蒙版。
        from 颜色标定 import _mask_from_hsv
        masks = {int(item['color_id']):_mask_from_hsv(hsv,item['ranges']) for item in cfg['colors']}
        parent = np.array([[[1,1]],[[638,1]],[[638,358]],[[1,358]]],np.int32)
        result = helper.recover_pale_component(hsv,masks,[] if case=='no_parent' else [parent],[parent] if case=='accepted_parent' else [])
        assert len(result)==(1 if case=='valid' else 0), ('negative case failed',case,result)
    report['checks']['15_component_positive_and_negative_cases'] = True
    from project_follow import MaterialFollower
    import time
    result=MaterialFollower(cfg).detect(frame,timestamp=time.monotonic()-.2)
    assert result['state']=='STALE_FRAME' and not result['targets'] and not result['motion_allowed']
    report['checks']['150ms_stale_input_rejected'] = True
    for color in ((0,0,230),(0,220,220),(220,0,0),(0,180,0),(15,15,15),(230,190,120)):
        frame=np.full((480,640,3),240,np.uint8)
        cv2.circle(frame,(320,240),65,color,-1)
        for config in (cfg,None):
            a,oa=old.analyze(frame,config,draw_overlay=True)
            b,ob=new.analyze(frame,config,draw_overlay=True)
            assert a==b
            np.testing.assert_array_equal(oa,ob)
    report['checks']['six_colors_and_unconfigured_results_overlays_preserved'] = True
    sample=OUT/'run-20261027-181512-402452/first-nonstale-lightblue-missing-raw.png'
    calls=[('check',[sys.executable,'-X','utf8',str(APP/'run.py'),'--check'])]
    for mode in ('follow','turntable','material','ring','stack','measurement'):
        args=[sys.executable,'-X','utf8',str(APP/'run.py'),'--image',str(sample),'--mode',mode,'--output-dir',str(OUT/('lightblue1-'+mode))]
        if mode=='measurement': args += ['--measure-points','500','620','1400','620']
        calls.append((mode,args))
    for feature in ('color','turntable-settings','pickup-regions','stack-lines'):
        calls.append((feature,[sys.executable,'-X','utf8',str(APP/'run.py'),'--feature',feature,'--help']))
    for label,args in calls:
        proc=subprocess.run(args,capture_output=True,text=True,encoding='utf8',timeout=60)
        (OUT/('lightblue1-'+label+'.log')).write_text(proc.stdout+proc.stderr,encoding='utf8')
        assert proc.returncode==0,(label,proc.stdout,proc.stderr)
        report['checks'][label]=True
    manifest=read(APP/'LIGHTBLUE1_MANIFEST.json')
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    for name,digest in manifest['source_files'].items():
        assert sha(BASE/name)==digest
        if name not in manifest['changed_files']: assert sha(APP/name)==digest
    # 刷新构建后评审修订的候选文件哈希；原文件哈希保留。
    manifest['candidate_files']={name:sha(APP/name) for name in manifest['changed_files']+manifest['added_files']}
    (APP/'LIGHTBLUE1_MANIFEST.json').write_text(json.dumps(manifest,indent=2),encoding='utf8')
    archive=APP.parent.with_suffix('.zip')
    assert not archive.exists(),'Do not overwrite a release ZIP'
    with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as z:
        for path in sorted(APP.rglob('*')):
            if path.is_file() and '__pycache__' not in path.parts and path.suffix!='.pyc':
                z.write(path,path.relative_to(APP.parent).as_posix())
    with zipfile.ZipFile(archive) as z:
        assert z.testzip() is None
        for name in z.namelist(): assert z.read(name)==(APP.parent/name).read_bytes()
    report['checks']['release_source_preserved_zip_crc_and_member_bytes']=True
    report['zip_sha256']=sha(archive)
    (OUT/'lightblue1-validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
    print(json.dumps({'checks':report['checks'],'images':len(seen),'changes':len(report['changes']),'zip':str(archive)},ensure_ascii=False))


if __name__=='__main__':
    main()
