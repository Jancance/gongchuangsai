"""用已存独立原图回归蓝色候选，不操作相机或执行器。"""
from pathlib import Path
from zipfile import ZipFile
import collections
import copy
import hashlib
import importlib.util
import json
import subprocess
import sys
import time

import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'output/p2_vision_generic135_20261026/vision_app'
APP=ROOT/'output/p2_vision_generic135_blue1_20261026/vision_app'
EVIDENCE=ROOT/'evidence/p2-camera/2026-10-26/blue1'

def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module

def main():
    EVIDENCE.mkdir(parents=True,exist_ok=True)
    cv2.setNumThreads(8)
    sys.path.insert(0,str(APP/'code'))
    old=load('blue1_baseline',BASE/'code/project_detector.py')
    new=load('blue1_candidate',APP/'code/project_detector.py')
    cfg=json.loads((APP/'code/config/material_color_thresholds.json').read_text(encoding='utf-8'))
    original=json.loads((BASE/'code/config/material_color_thresholds.json').read_text(encoding='utf-8'))
    without_flag=copy.deepcopy(cfg);without_flag.pop('blue_projected_silhouette_candidate')
    assert without_flag==original
    report={'motion_allowed':False,'hardware_acceptance':False,'checks':{},'historical_changes':[]}
    seen=set();dev=[]
    for p in sorted((ROOT/'evidence/p2-camera/2026-10-26/blue-pair-diagnostic').glob('*raw.png')):
        data=p.read_bytes();digest=hashlib.sha256(data).hexdigest()
        if digest in seen:continue
        seen.add(digest);frame=cv2.imdecode(np.frombuffer(data,np.uint8),1)
        before,_=old.analyze(frame,original,draw_overlay=False)
        after,overlay=new.analyze(frame,cfg,draw_overlay=True)
        labels=sorted(r['label'] for r in after['regions'] if r['label']!='UNKNOWN')
        assert labels==['blue','lightblue'],(p,labels)
        assert old.analyze(frame,original,draw_overlay=False)[0]==new.analyze(frame,without_flag,draw_overlay=False)[0]
        dev.append({'file':p.name,'before':sorted(r['label'] for r in before['regions'] if r['label']!='UNKNOWN'),'after':labels})
        if p.name=='first-frame-raw.png':cv2.imencode('.png',overlay)[1].tofile(EVIDENCE/'first-frame-overlay.png')
    report['development_unique_frames']=dev
    archives=['20261024-121400-134643','20261024-120512-675071','20261024-114659-072282',
              '20261024-112215-162612','20261024-111235-577533','20261024-104324-197600',
              '20261024-105943-475482','20261007-000604-409723']
    count=0
    for archive in archives:
        with ZipFile(ROOT/'evidence'/(archive+'.zip')) as z:
            for name in z.namelist():
                if not name.endswith('-raw.png'):continue
                data=z.read(name);digest=hashlib.sha256(data).hexdigest()
                if digest in seen:continue
                seen.add(digest);frame=cv2.imdecode(np.frombuffer(data,np.uint8),1)
                a,_=old.analyze(frame,original,draw_overlay=False)
                disabled,_=new.analyze(frame,without_flag,draw_overlay=False)
                assert a==disabled,('disabled-path regression',archive,name)
                b,_=new.analyze(frame,cfg,draw_overlay=False)
                aa=collections.Counter(r['label'] for r in a['regions'] if r['label']!='UNKNOWN')
                bb=collections.Counter(r['label'] for r in b['regions'] if r['label']!='UNKNOWN')
                assert not aa-bb,('lost known detection',archive,name,aa,bb)
                if aa!=bb:report['historical_changes'].append({'archive':archive,'file':name,'before':dict(aa),'after':dict(bb)})
                count+=1
    report['historical_unique_frames']=count
    # 单个静态场景不能证明实时性能；此处不报告Windows计时为板端性能。
    from project_follow import MaterialFollower
    stale=MaterialFollower(cfg).detect(np.zeros((1080,1920,3),np.uint8),timestamp=time.monotonic()-1)
    assert stale['state']=='STALE_FRAME' and not stale['targets'] and not stale['motion_allowed']
    report['checks']['150ms_input_gate']='passed'
    for mode in ('turntable','material','ring','stack','follow','measurement'):
        args=[sys.executable,'-X','utf8',str(APP/'run.py'),'--image',str(ROOT/'camera135_wblock_20261026_183726.png'),'--mode',mode,'--output-dir',str(EVIDENCE/mode)]
        if mode=='measurement':args+=['--measure-points','500','620','1400','620']
        result=subprocess.run(args,capture_output=True,text=True,encoding='utf-8',timeout=60)
        (EVIDENCE/(mode+'.log')).write_text(result.stdout+result.stderr,encoding='utf-8')
        assert result.returncode==0,(mode,result.stderr,result.stdout)
        report['checks']['offline-'+mode]='passed'
    for feature in ('color','turntable-settings','pickup-regions','stack-lines'):
        result=subprocess.run([sys.executable,'-X','utf8',str(APP/'run.py'),'--feature',feature,'--help'],capture_output=True,text=True,encoding='utf-8',timeout=30)
        assert result.returncode==0,(feature,result.stderr)
        report['checks']['help-'+feature]='passed'
    manifest=json.loads((APP/'BLUE1_MANIFEST.json').read_text())
    assert sorted(manifest['changed_files'])==['code/config/material_color_thresholds.json','code/project_detector.py']
    for name,h in manifest['source_files'].items():
        if name not in manifest['changed_files']:assert hashlib.sha256((APP/name).read_bytes()).hexdigest()==h,name
    report['checks']['other_source_files']='byte-identical'
    archive=APP.parent.with_suffix('.zip')
    with ZipFile(archive) as z:
        assert z.testzip() is None
        for name in z.namelist():assert z.read(name)==(APP.parent/name).read_bytes(),name
    report['zip_sha256']=hashlib.sha256(archive.read_bytes()).hexdigest()
    (EVIDENCE/'validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({'development_unique':len(dev),'historical_unique':count,'historical_changes':report['historical_changes'],'checks':report['checks']},ensure_ascii=False,indent=2))

if __name__=='__main__':main()
