"""夹取高度接近：新增有界可调下降步长，下降300RPM；不烧录或发运动。"""
from pathlib import Path
import hashlib
import json
import shutil
import re
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
ACTUAL=Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
STAGE=ROOT/'tmp/lift_down_step_20261030/project'
EV=ROOT/'evidence/p2-arm/2026-10-30/lift-down-step'
FILES=['HARDWARE/LIFT/lift_config.h','HARDWARE/LIFT/lift.h','HARDWARE/LIFT/lift.c',
       'HARDWARE/LIFT/lift_protocol.c','SYSTEM/CAN/can.c']
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def save(n,v):(EV/n).write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def verify(folder,m,after):
    for n,h in m['protected'].items():assert sha(folder/n)==h,n
    for e in m['files']:assert sha(folder/e['file'])==e['after' if after else 'before'],e['file']
def build(folder,log):
    r=subprocess.run(['C:/Keil_v5/UV4/UV4.exe','-r',str(folder/'gongxun-f407.uvprojx'),
                      '-j0','-o',str(log)],capture_output=True,timeout=120)
    text=log.read_text(encoding='utf-8',errors='replace')
    assert r.returncode==0 and re.search(r'0 Error\(s\), 0 Warning\(s\)',text),text[-4000:]
    print(text[-650:],flush=True)
if '--build' in sys.argv or '--install' in sys.argv:
    m=json.loads((EV/'manifest.json').read_text(encoding='utf-8'))
    h=json.loads((EV/'host-review.json').read_text(encoding='utf-8'))
    assert h['passed'] and h['source_sha256']=={e['file']:e['after'] for e in m['files']}
    verify(STAGE,m,True)
    if '--build' in sys.argv:
        build(STAGE,EV/'stage-build.log');verify(STAGE,m,True)
        m['stage_build_pass']=True;save('manifest.json',m)
    else:
        assert m['stage_build_pass'];verify(ACTUAL,m,False)
        for e in m['files']:shutil.copyfile(STAGE/e['file'],ACTUAL/e['file'])
        m['installed']=True;save('installed.json',m)
        build(ACTUAL,EV/'actual-build.log');verify(ACTUAL,m,True)
        m['actual_build_pass']=True;save('installed.json',m)
        print('Installed five source files; no flash or hardware commands.',flush=True)
    raise SystemExit(0)
assert not STAGE.exists()
EV.mkdir(parents=True,exist_ok=True)
shutil.copytree(ACTUAL,STAGE,ignore=shutil.ignore_patterns('Objects','Listings','.git','.vscode','*.uvguix.*'))
for n in FILES:
    p=EV/'original'/n;p.parent.mkdir(parents=True,exist_ok=True)
    assert not p.exists();shutil.copyfile(ACTUAL/n,p)
replacements=[]
def change(n,old,new):
    p=STAGE/n;t=p.read_bytes()
    old=old.encode('utf-8');new=new.encode('utf-8')
    assert t.count(old)==1,(n,old)
    p.write_bytes(t.replace(old,new,1))
    replacements.append((n,old,new))
change(FILES[0],'#define LIFT_UP_MAX_PULSES    233333U',
       '#define LIFT_UP_MAX_PULSES    233333U\n/* 夹取前下降：可调脉冲，默认仍5000；50000只是粗估30mm上限。 */\n#define LIFT_DOWN_MAX_PULSES  50000U\n#define LIFT_DOWN_JOG_RPM     300U')
change(FILES[1],'extern volatile uint32_t g_lift_up_pulses;',
       'extern volatile uint32_t g_lift_up_pulses;\n/* 请求1向下使用：1~50000，默认5000，接单锁存；不是毫米值。 */\nextern volatile uint32_t g_lift_down_pulses;')
change(FILES[2],'volatile uint32_t g_lift_up_pulses = LIFT_JOG_PULSES;',
       'volatile uint32_t g_lift_up_pulses = LIFT_JOG_PULSES;\nvolatile uint32_t g_lift_down_pulses = LIFT_JOG_PULSES;')
change(FILES[2],'s_motion_pulses = (request == 2U) ? g_lift_up_pulses : LIFT_JOG_PULSES;',
       's_motion_pulses = (request == 2U) ? g_lift_up_pulses : g_lift_down_pulses;')
change(FILES[2],'LIFT_UP_MAX_PULSES : LIFT_JOG_PULSES)',
       'LIFT_UP_MAX_PULSES : LIFT_DOWN_MAX_PULSES)')
change(FILES[3],'    if (out == 0 || direction > 1U || pulses == 0U ||',
       '    uint32_t rpm = (direction == 1U) ? LIFT_JOG_RPM : LIFT_DOWN_JOG_RPM;\n    if (out == 0 || direction > 1U || pulses == 0U ||')
change(FILES[3],'LIFT_UP_MAX_PULSES : LIFT_JOG_PULSES) || LIFT_JOG_RPM == 0U || LIFT_JOG_RPM > 3000U)',
       'LIFT_UP_MAX_PULSES : LIFT_DOWN_MAX_PULSES) || rpm == 0U || rpm > 3000U)')
change(FILES[3],'out[2] = (uint8_t)(LIFT_JOG_RPM >> 8);','out[2] = (uint8_t)(rpm >> 8);')
change(FILES[3],'out[3] = (uint8_t)LIFT_JOG_RPM;','out[3] = (uint8_t)rpm;')
change(FILES[4],'#include "extension_config.h"','#include "extension_config.h"\n#include "lift_config.h"')
# Restrict only node5's downward position packet; keep node1..4/node6 guards unchanged.
target='''    if (length != 12U || request[0] != 0xFDU || request[1] > 1U ||
        speed_rpm == 0U || speed_rpm > 3000U ||
        request[4] == 0U || request[9] != 2U || request[10] != 0U ||
        request[11] != 0x6BU) return 7U;'''
new=target.replace('speed_rpm == 0U || speed_rpm > 3000U ||',
                   'speed_rpm == 0U || speed_rpm > 3000U ||\n        (request[1] == 0U && speed_rpm > LIFT_DOWN_JOG_RPM) ||')
change(FILES[4],target,new)
change(FILES[4],'pulses > (request[1] == 1U ? 233333U : 5000U)',
       'pulses > (request[1] == 1U ? LIFT_UP_MAX_PULSES : LIFT_DOWN_MAX_PULSES)')
for n in FILES:
    recovered=(STAGE/n).read_bytes()
    for path,old,new in reversed(replacements):
        if path==n:
            assert recovered.count(new)==1
            recovered=recovered.replace(new,old,1)
    assert recovered==(ACTUAL/n).read_bytes(),n
protected={p.relative_to(ACTUAL).as_posix():sha(p) for p in ACTUAL.rglob('*')
           if p.is_file() and p.suffix.lower() in ('.c','.h','.uvprojx')
           and p.relative_to(ACTUAL).as_posix() not in FILES}
m={'date':'2026-10-30','files':[{'file':n,'before':sha(ACTUAL/n),'after':sha(STAGE/n)} for n in FILES],
   'protected':protected,'stage_build_pass':False,
   'default_request':0,'default_down_pulses':5000,'down_max_pulses':50000,'down_rpm':300,
   'up_max_pulses_unchanged':233333,'up_rpm_unchanged':3000,
   'user_gap_estimate_mm':30,'coarse_prior_ratio':'user previously selected5000 pulses approximately3mm; not metric calibration',
   'next_candidate_pulses':40000,'next_candidate_coarse_mm':24,
   'nominal_remaining_mm_not_guaranteed':6,
   'flash_performed':False,'hardware_commands_sent':False}
save('manifest.json',m);verify(STAGE,m,True);verify(ACTUAL,m,False)
print('Prepared5files; preserved other source/project files:',len(protected))
