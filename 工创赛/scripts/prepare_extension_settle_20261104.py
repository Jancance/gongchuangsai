"""Stage a bounded exact-zero settling barrier; never touch hardware or actual source."""
from pathlib import Path
import shutil, hashlib, json

R=Path(__file__).resolve().parents[1]
A=Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
S=R/'tmp/extension_settle_20261104'
E=R/'evidence/integration/2026-11-04/extension-settle'
E.mkdir(parents=True,exist_ok=True)
assert not S.exists()
shutil.copytree(A,S,ignore=shutil.ignore_patterns('Objects','ObjectsOffline','Listings','.git'))
original={}
def edit(rel,fn):
    p=S/rel;b=p.read_bytes();original[rel]=hashlib.sha256(b).hexdigest()
    try:u=b.decode('utf-8');enc='utf-8'
    except UnicodeDecodeError:u=b.decode('gbk');enc='gbk'
    (E/'before'/rel).parent.mkdir(parents=True,exist_ok=True)
    (E/'before'/rel).write_bytes(b)
    p.write_bytes(fn(u.replace('\r\n','\n')).replace('\n','\r\n').encode(enc))

def extension(u):
    marker='static uint32_t eh_last_command,eh_last_exchange,eh_last_flags;'
    assert u.count(marker)==1
    u=u.replace(marker,marker+'''
volatile uint32_t g_eh_first_step=0U,g_eh_speed_sign=0xffffffffU,g_eh_speed_rpm=0xffffffffU;
''')
    marker=' return eh_last_exchange==1U;}'
    assert u.count(marker)==1
    u=u.replace(marker,''' if(command==0x35U){
     g_eh_speed_sign=eh_last_exchange==1U?(uint32_t)eh_reply.Data[1]:0xffffffffU;
     g_eh_speed_rpm=eh_last_exchange==1U?(((uint32_t)eh_reply.Data[2]<<8)|eh_reply.Data[3]):0xffffffffU;
 }
'''+marker)
    marker='        g_eh_first_phase=g_extension_home_state;g_eh_first_reason=reason;'
    assert u.count(marker)==1
    u=u.replace(marker,'''#if !APP_MOTOR_OFFLINE_ONLY
        g_eh_first_step=eh_phase;
#endif
'''+marker)
    marker='    g_extension_hold_enable_count=g_extension_hold_samples=g_extension_hold_confirmed=0U;'
    u=u.replace(marker,'''    g_eh_first_step=0U;g_eh_speed_sign=g_eh_speed_rpm=0xffffffffU;
'''+marker)
    old='if(eh_reply.Data[2] || eh_reply.Data[3]){eh_fail(5U);return;}eh_phase=5U;return;'
    assert u.count(old)==1
    u=u.replace(old,'''/* 交接允许在既有2秒截止内等待精确零速，不提前放行或使能。 */
            if(eh_reply.Data[2] || eh_reply.Data[3])return;eh_phase=5U;return;''')
    old='if(eh_reply.Data[2] || eh_reply.Data[3]){eh_fail(5U);return;}eh_phase=9U;return;'
    assert u.count(old)==1
    u=u.replace(old,'''/* 非零样本中断连续确认；不重新使能，仍受同一截止约束。 */
            if(eh_reply.Data[2] || eh_reply.Data[3]){g_extension_hold_samples=0U;eh_phase=7U;return;}eh_phase=9U;return;''')
    return u
edit('CHASSIS/USER/extension_home_only.inc',extension)
def header(u):
    i=u.rindex('#endif')
    return u[:i]+'extern volatile uint32_t g_eh_first_step,g_eh_speed_sign,g_eh_speed_rpm;\n'+u[i:]
edit('CHASSIS/USER/startup_axes.h',header)
def telemetry(u):
    u=u.replace('static char frame[192]','static char frame[256]')
    old='"EX2,'+','.join(['%lu']*15)+',"'
    assert u.count(old)==1
    u=u.replace(old,'"EX3,'+','.join(['%lu']*18)+',"')
    old='(unsigned long)g_extension_hold_ready_tick,(unsigned long)g_extension_hold_delta_mdeg);'
    assert u.count(old)==1
    return u.replace(old,'''(unsigned long)g_extension_hold_ready_tick,(unsigned long)g_extension_hold_delta_mdeg,
        (unsigned long)g_eh_first_step,(unsigned long)g_eh_speed_sign,(unsigned long)g_eh_speed_rpm);''')
edit('CHASSIS/USER/vehicle_mission.inc',telemetry)
(E/'manifest.json').write_text(json.dumps(dict(actual=str(A),stage=str(S),original=original,
    changed={n:hashlib.sha256((S/n).read_bytes()).hexdigest() for n in original}),ensure_ascii=False,indent=2),encoding='utf-8')
print(S)
