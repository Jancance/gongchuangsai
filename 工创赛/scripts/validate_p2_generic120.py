"""验证并打包120度候选；仅本机离线检查，不连接板端。"""
from pathlib import Path
import hashlib
import json
import subprocess
import sys
from zipfile import ZipFile

from package_integrated_vision import build_package

ROOT = Path(__file__).resolve().parents[1]
APP = ROOT / 'output/p2_vision_generic120_20261023/vision_app'
EVIDENCE = ROOT / 'evidence/p2-camera/2026-10-23/main-integration'


def run(arguments):
    result = subprocess.run([sys.executable,'-X','utf8',*map(str,arguments)], cwd=ROOT,
                            capture_output=True,text=True,encoding='utf-8',timeout=90)
    if result.returncode:
        raise RuntimeError(result.stdout + result.stderr)
    return result.stdout + result.stderr


def main():
    EVIDENCE.mkdir(parents=True,exist_ok=True)
    report = {'hardware_acceptance':False,'motion_allowed':False,'checks':{},'offline':{}}
    for pattern in ('test_camera120_startup.py','test_cyqm_integration.py','test_p2_generic120_integration.py'):
        output=run(['-m','unittest','discover','-s','tests','-p',pattern,'-v'])
        (EVIDENCE/(pattern+'.log')).write_text(output,encoding='utf-8')
        report['checks'][pattern]='passed; see log'
    report['checks']['compileall']=run(['-m','compileall','-q',str(APP)]) or 'passed'
    report['checks']['check']=json.loads(run([APP/'run.py','--check']))
    image=ROOT/'evidence/p2-camera/2026-10-23/startup-repeatability/startup1.png'
    for mode in ('follow','turntable','material','ring','stack','measurement'):
        target=EVIDENCE/'offline'/mode
        args=[APP/'run.py','--image',image,'--mode',mode,'--output-dir',target]
        if mode=='measurement':args.extend(['--measure-points','0','0','100','100'])
        run(args)
        result=json.loads((target/'result.json').read_text(encoding='utf-8'))
        if result.get('motion_allowed') is True or result.get('ready') is True:
            raise RuntimeError('未标定离线输出错误开放动作/就绪')
        report['offline'][mode]={'completed':True,'result':str((target/'result.json').relative_to(ROOT)),
                                'note':'仅功能冒烟，不评判颜色/位置正确率'}
    archive=ROOT/'output/p2_vision_generic120_20261023.zip'
    files=build_package(archive,APP,{'built_for':'Generic USB 10bb:2b08 1920x1080 candidate',
                         'date':'2026-10-23','source_archive_sha256':'26bb0c9b03a7d7e564e2193ff34fd6de16bf7f39c43de1beac37ec28cc2ee318'})
    report['package']={'path':str(archive.relative_to(ROOT)), 'files':len(files),
                       'sha256':hashlib.sha256(archive.read_bytes()).hexdigest(),'crc_and_member_hashes':'passed'}
    # 从真实ZIP解压副本检查中文入口、配置与文件完整性。
    extracted=ROOT/'tmp/p2_generic120_package_verify'
    with ZipFile(archive) as z:z.extractall(extracted)
    report['checks']['extracted_check']=json.loads(run([extracted/'vision_app/run.py','--check']))
    for feature in ('color','turntable-settings','pickup-regions','stack-lines'):
        run([extracted/'vision_app/run.py','--feature',feature,'--help'])
    report['checks']['extracted_four_tool_help']='passed'
    (EVIDENCE/'validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
