"""只延长模式2的人工确认窗口并记录故障阶段；运动/回零超时不变。"""
from pathlib import Path
import hashlib,json,shutil,subprocess,re,sys
ROOT=Path(__file__).resolve().parents[1]
ACTUAL=Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
STAGE=ROOT/'tmp/arm_confirm_wait_20261031/project'
EV=ROOT/'evidence/p2-arm/2026-10-31/arm-confirm-wait'
FILES=['USER/arm_sequence.c','USER/arm_sequence.h']
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def save(n,v):(EV/n).write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def build(folder,log):
    r=subprocess.run(['C:/Keil_v5/UV4/UV4.exe','-b',str(folder/'gongxun-f407.uvprojx'),'-j0','-o',str(log)],timeout=120)
    s=log.read_text(encoding='utf-8',errors='replace');assert r.returncode==0 and re.search(r'0 Error\(s\), 0 Warning\(s\)',s),s[-4000:]
    print(s[-800:],flush=True)
if '--build' in sys.argv or '--install' in sys.argv:
    m=json.loads((EV/'manifest.json').read_text(encoding='utf-8'))
    for e in m['files']:assert sha(STAGE/e['file'])==e['after']
    if '--build' in sys.argv:
        build(STAGE,EV/'stage-build.log');m['stage_build_pass']=True;save('manifest.json',m)
    else:
        h=json.loads((EV/'host-review.json').read_text())
        assert m['stage_build_pass'] and h['passed'] and h['source_sha256']=={e['file']:e['after'] for e in m['files']}
        for n,h in m['protected'].items():assert sha(ACTUAL/n)==h,n
        for e in m['files']:assert sha(ACTUAL/e['file'])==e['before'],e['file']
        for e in m['files']:shutil.copyfile(STAGE/e['file'],ACTUAL/e['file'])
        save('installed.json',m);build(ACTUAL,EV/'actual-build.log')
        for e in m['files']:assert sha(ACTUAL/e['file'])==e['after']
        for n,h in m['protected'].items():assert sha(ACTUAL/n)==h,n
        m['actual_build_pass']=True;save('installed.json',m)
    raise SystemExit(0)
assert not STAGE.exists();EV.mkdir(parents=True,exist_ok=True)
shutil.copytree(ACTUAL,STAGE,ignore=shutil.ignore_patterns('Objects','Listings','.git','.vscode','*.uvguix.*'))
for n in FILES:
    p=EV/'original'/n;p.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ACTUAL/n,p)
def once(s,a,b):assert s.count(a)==1,a;return s.replace(a,b,1)
p=STAGE/'USER/arm_sequence.h';s=p.read_text(encoding='utf-8')
s=once(s,'extern volatile uint32_t g_arm_sequence_elapsed_ms;', '''extern volatile uint32_t g_arm_sequence_elapsed_ms;
/* 当前阶段计时；故障/中止时冻结，清锁存或新接单清零。 */
extern volatile uint32_t g_arm_sequence_phase_elapsed_ms;
/* 保存发生故障/中止前的阶段，不以故障态9覆盖原阶段。 */
extern volatile uint32_t g_arm_sequence_fault_state;
/* 仅模式2看护调试：state2/state12人工确认窗口120秒。正式运行不使用此入口。 */''')
p.write_text(s,encoding='utf-8')
p=STAGE/'USER/arm_sequence.c';s=p.read_text(encoding='utf-8')
s=once(s,'volatile uint32_t g_arm_sequence_elapsed_ms=0U;', 'volatile uint32_t g_arm_sequence_elapsed_ms=0U;\nvolatile uint32_t g_arm_sequence_phase_elapsed_ms=0U,g_arm_sequence_fault_state=0U;')
s=once(s,'static void Fail(uint32_t result,uint32_t state)\n{', '''static uint32_t ConfirmTimeout(void)
{
    /* 两分钟只用于模式2人工看护确认；不改变任何电机/回零超时。 */
    return s_mode==2U?120000U:15000U;
}
static void Fail(uint32_t result,uint32_t state)
{
    if(g_arm_sequence_state!=8U && g_arm_sequence_state!=9U)
        g_arm_sequence_fault_state=g_arm_sequence_state;''')
s=once(s,'{g_arm_sequence_state=state;s_phase=now;s_settling=0U;}', '{g_arm_sequence_state=state;s_phase=now;s_settling=0U;g_arm_sequence_phase_elapsed_ms=0U;}')
s=once(s,'g_arm_sequence_state=0U;g_arm_sequence_result=0U;return;', 'g_arm_sequence_state=0U;g_arm_sequence_result=0U;g_arm_sequence_fault_state=0U;g_arm_sequence_phase_elapsed_ms=0U;return;')
s=once(s,'        g_arm_sequence_result=0U;g_arm_sequence_elapsed_ms=0U;', '        g_arm_sequence_result=0U;g_arm_sequence_elapsed_ms=0U;g_arm_sequence_fault_state=0U;')
s=once(s,'    g_arm_sequence_elapsed_ms=now-s_started;', '    g_arm_sequence_elapsed_ms=now-s_started;g_arm_sequence_phase_elapsed_ms=now-s_phase;')
assert s.count('else if(now-s_phase>15000U)Fail(7U,9U);')==3
# 只替换state2/state12的确认等待；中间回零阶段的15秒组包络保持。
a=s.index('    case 2U:');b=s.index('    case 3U:',a)
s=s[:a]+s[a:b].replace('now-s_phase>15000U','now-s_phase>ConfirmTimeout()')+s[b:]
a=s.index('    case 12U:');b=s.index('    case 13U:',a)
s=s[:a]+s[a:b].replace('now-s_phase>15000U','now-s_phase>ConfirmTimeout()')+s[b:]
p.write_text(s,encoding='utf-8')
protected={p.relative_to(ACTUAL).as_posix():sha(p) for p in ACTUAL.rglob('*') if p.is_file() and p.suffix.lower() in ('.c','.h','.uvprojx') and p.relative_to(ACTUAL).as_posix() not in FILES}
save('manifest.json',{'date':'2026-10-31','files':[{'file':n,'before':sha(ACTUAL/n),'after':sha(STAGE/n)} for n in FILES],'protected':protected,'stage_build_pass':False,'mode2_confirm_timeout_ms':120000,'legacy_confirm_timeout_ms':15000,'flash':False,'hardware_commands':False})
print('Prepared2-file debug wait/phase diagnostics change; protected',len(protected),'other sources/project files.')
