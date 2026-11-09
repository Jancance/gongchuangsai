"""固定动作模式3：副本构建/实码桩验证后才可安装，不下载、不发动作。"""
from pathlib import Path
import hashlib
import json
import re
import shutil
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
ACTUAL=Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
STAGE=ROOT/'tmp/arm_fixed_replay_20261101/project'
EV=ROOT/'evidence/p2-arm/2026-11-01/fixed-replay'
FILES=['USER/arm_sequence.c','USER/arm_sequence.h','USER/main.c']
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def save(name,value):(EV/name).write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def once(s,a,b):
    assert s.count(a)==1,a
    return s.replace(a,b,1)
def build(project,log):
    result=subprocess.run(['C:/Keil_v5/UV4/UV4.exe','-b',str(project/'gongxun-f407.uvprojx'),'-j0','-o',str(log)],timeout=120)
    text=log.read_text(encoding='utf-8',errors='replace')
    assert result.returncode==0 and re.search(r'0 Error\(s\), 0 Warning\(s\)',text),text[-4000:]
    print(text[-850:])
if '--build' in sys.argv or '--install' in sys.argv:
    m=json.loads((EV/'manifest.json').read_text(encoding='utf-8'))
    for n in FILES:assert sha(STAGE/n)==m['after'][n],n
    if '--build' in sys.argv:
        build(STAGE,EV/'stage-build.log');m['stage_build_pass']=True
    else:
        h=json.loads((EV/'host-review.json').read_text(encoding='utf-8'))
        assert m['stage_build_pass'] and h['passed'] and h['source_sha256']==m['after']
        for n,v in m['before'].items():assert sha(ACTUAL/n)==v,n
        for n,v in m['protected'].items():assert sha(ACTUAL/n)==v,n
        for n in FILES:shutil.copyfile(STAGE/n,ACTUAL/n)
        m['installed']=True;save('manifest.json',m)
        build(ACTUAL,EV/'actual-build.log')
        for n,v in m['after'].items():assert sha(ACTUAL/n)==v,n
        for n,v in m['protected'].items():assert sha(ACTUAL/n)==v,n
        m['actual_build_pass']=True
    save('manifest.json',m)
    raise SystemExit(0)
assert not STAGE.exists()
EV.mkdir(parents=True,exist_ok=True)
shutil.copytree(ACTUAL,STAGE,ignore=shutil.ignore_patterns('Objects','Listings','.git','.vscode','*.uvguix.*'))
for n in FILES:
    p=EV/'original'/n;p.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ACTUAL/n,p)
p=STAGE/'USER/arm_sequence.c';s=p.read_text(encoding='utf-8')
s=once(s,'#include "lift_home.h"','#include "lift_home.h"\n#include "dm_can2.h"')
s=once(s,'    Gripper_CancelControlled();\n    if(LiftHome_IsBusy()',
       '    Gripper_CancelControlled();\n    if(s_mode==3U){g_dm_jog_request=3U;g_dm_point_enable=0U;}\n    if(LiftHome_IsBusy()')
fragment=(ROOT/'handoff/arm_fixed_replay_20261101/replay.c.fragment').read_text(encoding='utf-8')
s=once(s,'void ArmSequence_Service(uint32_t ready,uint32_t other_busy,uint32_t now)',
       fragment+'\nvoid ArmSequence_Service(uint32_t ready,uint32_t other_busy,uint32_t now)')
s=once(s,'    uint32_t request=g_arm_sequence_request,result;',
       '    uint32_t request=g_arm_sequence_request,result;\n    if((s_busy && s_mode==3U) || (!s_busy && g_arm_sequence_mode==3U))\n    {ReplayService(ready,other_busy,now);return;}')
p.write_text(s,encoding='utf-8')
p=STAGE/'USER/arm_sequence.h';s=p.read_text(encoding='utf-8')
s=once(s,'#endif','''/* 模式3固定动作：源下降g_arm_pick_down_pulses须现场示教；0拒绝。
 * 顶部/源料/空槽及路径人工确认后，top_start_ready=1，enable=1，request=1一次。
 * 不等待中途人工确认，不重复循环；state7/1仅软件结束，不自动登记库存。
 * 20张开、21转源、22源下降、23夹紧、24源回升、25转车载、26车载下放、27松开、28回升。
 * result9为旋转/保持失败；其他结果沿用旧定义。 */
extern volatile float g_arm_replay_pick_deg,g_arm_replay_place_deg;
uint32_t ArmSequence_OwnsDM(void);
uint32_t ArmSequence_AllowDMRequest(uint32_t request);
#endif''')
p.write_text(s,encoding='utf-8')
p=STAGE/'USER/main.c';s=p.read_bytes().decode('latin1')
# 仅动作组仲裁允许自身DM请求，保持其余CAN/底盘与编码字节。
a=s.index('static void App_ArmSequenceGuard(void)');b=s.index('static void App_CarrierStep(void)',a)
part=s[a:b]
part=once(part,'        if(g_dm_jog_request!=3U)g_dm_jog_request=0U;',
          '        if(!ArmSequence_AllowDMRequest(g_dm_jog_request))g_dm_jog_request=0U;')
s=s[:a]+part+s[b:]
s=once(s,'(Carrier_IsBusy() || DM_CAN2_JogIsBusy() || (LiftHome_IsBusy() && !ArmSequence_OwnsHome()) ||',
       '(Carrier_IsBusy() || (DM_CAN2_JogIsBusy() && !ArmSequence_OwnsDM()) || (LiftHome_IsBusy() && !ArmSequence_OwnsHome()) ||')
p.write_bytes(s.encode('latin1'))
protected={p.relative_to(ACTUAL).as_posix():sha(p) for p in ACTUAL.rglob('*') if p.is_file()
           and p.suffix.lower() in ('.c','.h','.uvprojx') and p.relative_to(ACTUAL).as_posix() not in FILES}
save('manifest.json',dict(before={n:sha(ACTUAL/n) for n in FILES},after={n:sha(STAGE/n) for n in FILES},
                         protected=protected,stage_build_pass=False,actual_build_pass=False,installed=False,
                         source_pick_down_pulses_default=0,flash=False,hardware_commands=False))
print('Prepared three-file fixed replay; other source/project files protected:',len(protected))
