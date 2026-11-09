"""Validate and package isolated ring1; no camera/robot access."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
from zipfile import ZipFile

from package_integrated_vision import build_package

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'output/p2_vision_generic120_perf1_20261023/vision_app'
APP=ROOT/'output/p2_vision_generic120_ring1_20261024/vision_app'
EVIDENCE=ROOT/'evidence/p2-camera/2026-10-24/ring1'


def run(args):
    value=subprocess.run([sys.executable,'-X','utf8',*map(str,args)],cwd=ROOT,capture_output=True,
                         text=True,encoding='utf-8',timeout=90)
    if value.returncode:raise RuntimeError(value.stdout+value.stderr)
    return value.stdout+value.stderr


def main():
    EVIDENCE.mkdir(parents=True,exist_ok=True)
    log=run(['-m','unittest','discover','-s','tests','-p','test_generic120_ring1.py','-v'])
    (EVIDENCE/'regression.log').write_text(log,encoding='utf-8')
    checks={'regression':'8 passed; see regression.log','check':json.loads(run([APP/'run.py','--check']))}
    run(['-m','compileall','-q',APP]);checks['compileall']='passed'
    for feature in ('color','turntable-settings','pickup-regions','stack-lines'):
        run([APP/'run.py','--feature',feature,'--help'])
    checks['four_tool_help']='passed'
    from zipfile import ZipFile
    raw=EVIDENCE/'false-positive-input.png'
    with ZipFile(ROOT/'evidence/20261024-104324-197600.zip') as z:
        raw.write_bytes(z.read('first-green-with-black-raw.png'))
    for mode in ('follow','turntable','material','ring','stack','measurement'):
        target=EVIDENCE/'offline'/mode
        args=[APP/'run.py','--image',raw,'--mode',mode,'--output-dir',target]
        if mode=='measurement':args+=['--measure-points','0','0','100','100']
        run(args)
        result=json.loads((target/'result.json').read_text(encoding='utf-8'))
        assert not result.get('ready') and not result.get('motion_allowed')
    checks['six_offline_modes']='passed; smoke only'
    before=json.loads((BASE/'code/config/material_color_thresholds.json').read_text(encoding='utf-8'))
    after=json.loads((APP/'code/config/material_color_thresholds.json').read_text(encoding='utf-8'))
    del after['printed_ring_rejection_candidate'];assert before==after
    for path in (BASE/'code/config').iterdir():
        if path.is_file() and path.name!='material_color_thresholds.json':
            assert path.read_bytes()==(APP/'code/config'/path.name).read_bytes()
    for name in ('project_camera.py','project_capture.py','project_follow.py','project_tracker.py','main.py'):
        assert (BASE/'code'/name).read_bytes()==(APP/'code'/name).read_bytes()
    checks['hsv_camera_follow_tracker_and_other_configs_unchanged']='passed'
    (APP/'RING1_DEPLOY.md').write_bytes((ROOT/'docs/P2_PRINTED_RING_REJECTION_2026-10-24.md').read_bytes())
    archive=ROOT/'output/p2_vision_generic120_ring1_20261024.zip'
    files=build_package(archive,APP,{'built_for':'Generic120 ring1 print-background rejection candidate',
                                   'date':'2026-10-24','motion_allowed':False,'hardware_acceptance':False})
    extracted=ROOT/'tmp/p2_ring1_package_verify'
    with ZipFile(archive) as z:z.extractall(extracted)
    checks['extracted_check']=json.loads(run([extracted/'vision_app/run.py','--check']))
    # 已部署83取证包装器可直接指定新应用，无须再改包装器。
    checks['exposure83_wrapper_check']=json.loads(run([
        ROOT/'output/p2_green83_ring_capture.py','--app',extracted/'vision_app',
        '--audit-dir',ROOT/'output/p2_perf1_green_capture_20261024','--check']))
    report={'hardware_acceptance':False,'motion_allowed':False,'checks':checks,
            'package':{'file':str(archive.relative_to(ROOT)),'files':len(files),
                       'sha256':hashlib.sha256(archive.read_bytes()).hexdigest(),'crc_and_member_hashes':'passed'}}
    (EVIDENCE/'validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
