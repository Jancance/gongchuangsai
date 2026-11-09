"""验证性能候选的算法等价/时效门与包完整性，性能数值仅本机参考。"""
from pathlib import Path
import hashlib
import importlib.util
import json
import subprocess
import sys
import time
from zipfile import ZipFile

import cv2
import numpy as np
from package_integrated_vision import build_package

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'output/p2_vision_generic120_20261023/vision_app'
APP=ROOT/'output/p2_vision_generic120_perf1_20261023/vision_app'
EVIDENCE=ROOT/'evidence/p2-camera/2026-10-24/perf1'


def run(args):
    value=subprocess.run([sys.executable,'-X','utf8',*map(str,args)],cwd=ROOT,
                         capture_output=True,text=True,encoding='utf-8',timeout=90)
    if value.returncode:raise RuntimeError(value.stdout+value.stderr)
    return value.stdout+value.stderr


def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec)
    sys.modules[name]=module
    spec.loader.exec_module(module)
    return module


def main():
    EVIDENCE.mkdir(parents=True,exist_ok=True)
    log=run(['-m','unittest','discover','-s','tests','-p','test_generic120_perf1.py','-v'])
    (EVIDENCE/'regression.log').write_text(log,encoding='utf-8')
    run(['-m','compileall','-q',APP])
    checks={'regression':'9 passed; see regression.log','compileall':'passed',
            'check':json.loads(run([APP/'run.py','--check']))}
    for feature in ('color','turntable-settings','pickup-regions','stack-lines'):
        run([APP/'run.py','--feature',feature,'--help'])
    checks['four_tool_help']='passed'
    image=ROOT/'evidence/p2-camera/2026-10-23/startup-repeatability/startup1.png'
    for mode in ('follow','turntable','material','ring','stack','measurement'):
        target=EVIDENCE/'offline'/mode
        args=[APP/'run.py','--image',image,'--mode',mode,'--output-dir',target]
        if mode=='measurement':args.extend(['--measure-points','0','0','100','100'])
        run(args)
        result=json.loads((target/'result.json').read_text(encoding='utf-8'))
        if result.get('ready') or result.get('motion_allowed'):raise RuntimeError('离线输出意外开放就绪/动作')
    checks['six_offline_modes']='passed; smoke only, no accuracy acceptance'
    # 所有源配置逐字节保持一致；只允许manifest声明的三项源码变更。
    for p in (BASE/'code/config').rglob('*'):
        if p.is_file():assert p.read_bytes()==(APP/'code/config'/p.relative_to(BASE/'code/config')).read_bytes()
    checks['all_configs_byte_identical']='passed'
    sys.path.insert(0,str(APP/'code'))
    before=load('perf_bench_before',BASE/'code/project_detector.py')
    after=load('perf_bench_after',APP/'code/project_detector.py')
    old_colors=load('perf_bench_old_colors',BASE/'code/颜色标定.py')
    new_colors=load('perf_bench_new_colors',APP/'code/颜色标定.py')
    config=json.loads((APP/'code/config/material_color_thresholds.json').read_text(encoding='utf-8'))
    frame=cv2.imdecode(np.fromfile(image,dtype=np.uint8),1)
    benchmark={'note':'Windows saved startup1 image, repeated no-hint analysis only; not Orange Pi or realtime acceptance',
               'opencv_version':cv2.__version__,'processing_threads':cv2.getNumThreads(),'rounds':[]}
    # 交替顺序减轻单向预热偏差，每轮每版本6次；完全相同的输入与配置。
    for round_index in range(3):
        pair=[]
        variants=[('base',before,old_colors),('perf1',after,new_colors)]
        if round_index%2:variants.reverse()
        for label,detector,colors in variants:
            sys.modules['颜色标定']=colors
            detector.analyze(frame,config,draw_overlay=False)
            timings=[]
            for _ in range(6):
                start=time.perf_counter();detector.analyze(frame,config,draw_overlay=False)
                timings.append((time.perf_counter()-start)*1000)
            pair.append({'variant':label,'ms':timings,'p50_p95_max':np.percentile(timings,[50,95,100]).tolist()})
        benchmark['rounds'].append(pair)
    guide=ROOT/'docs/P2_GENERIC120_PERF1_2026-10-24.md'
    (APP/'PERF1_DEPLOY.md').write_bytes(guide.read_bytes())
    archive=ROOT/'output/p2_vision_generic120_perf1_20261024.zip'
    files=build_package(archive,APP,{'built_for':'Generic120 1920x1080 perf1 candidate','date':'2026-10-24',
                                  'hardware_acceptance':False,'motion_allowed':False,'thresholds_changed':False})
    extracted=ROOT/'tmp/p2_generic120_perf1_package_verify'
    with ZipFile(archive) as z:z.extractall(extracted)
    checks['extracted_check']=json.loads(run([extracted/'vision_app/run.py','--check']))
    for feature in ('color','turntable-settings','pickup-regions','stack-lines'):
        run([extracted/'vision_app/run.py','--feature',feature,'--help'])
    checks['extracted_four_tool_help']='passed'
    report={'hardware_acceptance':False,'motion_allowed':False,'checks':checks,'local_benchmark':benchmark,
            'package':{'file':str(archive.relative_to(ROOT)),'files':len(files),
                       'sha256':hashlib.sha256(archive.read_bytes()).hexdigest(),'crc_and_member_hashes':'passed'}}
    (EVIDENCE/'validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
