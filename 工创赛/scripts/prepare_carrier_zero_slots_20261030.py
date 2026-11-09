"""按用户明确决定改0/120/240度候选，仅更新实际配置头；不烧录。"""
from pathlib import Path
import hashlib
import json
import re
import shutil
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
ACTUAL=Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
STAGE=ROOT/'tmp/carrier_zero_slots_20261030/project'
EV=ROOT/'evidence/p2-arm/2026-10-30/carrier-zero-slots'
ASSET=ROOT/'output/carrier_zero_slots_20261030/carrier_config.h'
REL='HARDWARE/CARRIER/carrier_config.h'
PROJECT='gongxun-f407.uvprojx'

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def save(name,value):
    (EV/name).write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def verify(folder,m,after):
    for n,h in m['protected'].items():assert sha(folder/n)==h,n
    assert sha(folder/REL)==m['after' if after else 'before'],REL
def build(folder,log):
    r=subprocess.run(['C:/Keil_v5/UV4/UV4.exe','-r',str(folder/PROJECT),'-j0','-o',str(log)],
                     capture_output=True,timeout=120)
    text=log.read_text(encoding='utf-8',errors='replace')
    assert r.returncode==0 and re.search(r'0 Error\(s\), 0 Warning\(s\)',text),text[-4000:]
    assert 'compiling carrier.c...' in text and 'compiling carrier_pwm.c...' in text
    print(text[-650:],flush=True)

if '--build' in sys.argv or '--install' in sys.argv:
    m=json.loads((EV/'manifest.json').read_text(encoding='utf-8'))
    h=json.loads((EV/'host-review.json').read_text(encoding='utf-8'))
    assert h['passed'] and h['configuration_sha256']==m['after']
    verify(STAGE,m,True)
    if '--build' in sys.argv:
        build(STAGE,EV/'stage-build.log');verify(STAGE,m,True)
        m['stage_build_pass']=True;save('manifest.json',m)
    else:
        assert m['stage_build_pass'];verify(ACTUAL,m,False)
        shutil.copyfile(STAGE/REL,ACTUAL/REL)
        m['installed']=True;save('installed.json',m)
        build(ACTUAL,EV/'actual-build.log');verify(ACTUAL,m,True)
        m['actual_build_pass']=True;save('installed.json',m)
        print('Installed one configuration header; no flash or hardware commands.',flush=True)
    raise SystemExit(0)

assert not STAGE.exists()
EV.mkdir(parents=True,exist_ok=True)
shutil.copytree(ACTUAL,STAGE,ignore=shutil.ignore_patterns('Objects','Listings','.git','.vscode','*.uvguix.*'))
(EV/'original').mkdir(exist_ok=True)
assert not (EV/'original/carrier_config.h').exists()
shutil.copyfile(ACTUAL/REL,EV/'original/carrier_config.h')
shutil.copyfile(ASSET,STAGE/REL)
protected={p.relative_to(ACTUAL).as_posix():sha(p) for p in ACTUAL.rglob('*')
           if p.is_file() and p.suffix.lower() in ('.c','.h','.uvprojx')
           and p.relative_to(ACTUAL).as_posix()!=REL}
m={'date':'2026-10-30','file':REL,'before':sha(ACTUAL/REL),'after':sha(STAGE/REL),
   'protected':protected,'project':PROJECT,'stage_build_pass':False,
   'user_decision':'直接让舵机旋转到0°，安装对准固定取放位置，每转120°为下一个槽',
   'supersedes':'assistant135-degree assembly proposal; prior evidence retained',
   'nominal_targets_deg':[0,120,240],'nominal_pulses_us':[500,1389,2278],
   'default_target_deg':0,'default_enable':0,'default_request':0,
   'motion_or_slot_alignment_verified':False,'flash_performed':False,'hardware_commands_sent':False}
save('manifest.json',m);verify(STAGE,m,True);verify(ACTUAL,m,False)
print('Prepared one header; protected source/project files:',len(protected))
