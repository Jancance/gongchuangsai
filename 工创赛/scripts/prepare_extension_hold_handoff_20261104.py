"""Guarded post-home STOP/enable/readback barrier, staged from actual source."""
from pathlib import Path
import json,hashlib
R=Path(__file__).resolve().parents[1];E=R/'evidence/integration/2026-11-04/extension-vm-final';M=E/'manifest.json'
m=json.loads(M.read_text(encoding='utf-8'));A=Path(m['actual']);S=Path(m['stage'])
def edit(rel,fn):
 b=(A/rel).read_bytes()
 try:u=b.decode('utf-8');enc='utf-8'
 except UnicodeDecodeError:u=b.decode('gbk');enc='gbk'
 q=E/'before-install'/rel;q.parent.mkdir(parents=True,exist_ok=True);q.write_bytes(b)
 (S/rel).write_bytes(fn(u.replace('\r\n','\n')).replace('\n','\r\n').encode(enc));m['original'][rel]=hashlib.sha256(b).hexdigest()
def params(u):
 marker='#define VEHICLE_EXTENSION_HOME_DIRECTION 0U';assert u.count(marker)==1
 return u.replace(marker,'''#define VEHICLE_EXTENSION_HOLD_CONFIRM_SAMPLES 3U
#define VEHICLE_EXTENSION_HOLD_TIMEOUT_MS 2000U
#define VEHICLE_EXTENSION_HOLD_POSITION_TOL_DEG 2.0f
'''+marker)
edit('USER/vehicle_parameters.h',params)
def header(u):
 idx=u.rindex('#endif');return u[:idx]+'''#define EXTENSION_HOLD_HANDSHAKE_V1 1U
/* Completion proof is retained after faults; it is not a motion permit. */
extern volatile uint32_t g_extension_hold_enable_count,g_extension_hold_samples;
extern volatile uint32_t g_extension_hold_confirmed,g_extension_hold_ready_tick,g_extension_hold_delta_mdeg;
'''+u[idx:]
edit('CHASSIS/USER/startup_axes.h',header)
def extension(u):
 u='#include <math.h>\n'+u
 marker='static uint32_t eh_last,eh_started,eh_stop_pending;'
 u=u.replace(marker,'''volatile uint32_t g_extension_hold_enable_count=0U,g_extension_hold_samples=0U;
volatile uint32_t g_extension_hold_confirmed=0U,g_extension_hold_ready_tick=0U,g_extension_hold_delta_mdeg=0U;
static uint32_t eh_hold_begin;
static float eh_hold_position;
'''+marker)
 old='''{return StartupAxis_Exchange(q,length,3U,&eh_reply)==1U &&
    (eh_reply.Data[1]==2U || (q[0]==0x9aU && eh_reply.Data[1]==0x9fU));}''';assert old in u
 u=u.replace(old,'''{eh_last_command=q[0];eh_last_exchange=0U;
 eh_last_exchange=StartupAxis_Exchange(q,length,3U,&eh_reply);
 eh_last_flags=eh_last_exchange==1U?(uint32_t)eh_reply.Data[1]:0xffffffffU;
 return eh_last_exchange==1U && (eh_reply.Data[1]==2U || (q[0]==0x9aU && eh_reply.Data[1]==0x9fU));}''')
 marker='    eh_begin=eh_last=(uint32_t)xTaskGetTickCount();'
 u=u.replace(marker,'''    if(VEHICLE_EXTENSION_HOLD_CONFIRM_SAMPLES<1U || VEHICLE_EXTENSION_HOLD_CONFIRM_SAMPLES>5U ||
       VEHICLE_EXTENSION_HOLD_TIMEOUT_MS<500U || VEHICLE_EXTENSION_HOLD_TIMEOUT_MS>5000U ||
       !(VEHICLE_EXTENSION_HOLD_POSITION_TOL_DEG>0.0f && VEHICLE_EXTENSION_HOLD_POSITION_TOL_DEG<=5.0f))
    {eh_fail(7U);return 0U;}
    g_extension_hold_enable_count=g_extension_hold_samples=g_extension_hold_confirmed=0U;
    g_extension_hold_ready_tick=g_extension_hold_delta_mdeg=eh_hold_begin=0U;
'''+marker)
 u=u.replace('if(g_startup_axes_state!=4U && g_extension_home_elapsed_ms>=(eh_started?11000U:2500U))',
             'if(g_startup_axes_state!=4U && !(g_startup_axes_state==3U && eh_phase>=3U) && g_extension_home_elapsed_ms>=(eh_started?11000U:2500U))')
 marker='    if(now-eh_poll<50U)return;eh_poll=now;'
 assert u.count(marker)==1
 block='''    if(g_startup_axes_state==3U && eh_phase>=3U && now-eh_hold_begin>=VEHICLE_EXTENSION_HOLD_TIMEOUT_MS)
    {eh_fail(3U);return;}
'''+marker+'''
    if(g_startup_axes_state==3U && eh_phase>=3U){
        g_extension_home_state=9U;
        switch(eh_phase){
        case 3U:
            if(!eh_query(0x3a,3U)){eh_fail(4U);return;}
            if(eh_reply.Data[1]&0x0cU){eh_fail(5U);return;}eh_phase=4U;return;
        case 4U:
            if(!eh_query(0x35,5U)){eh_fail(4U);return;}
            if(eh_reply.Data[2] || eh_reply.Data[3]){eh_fail(5U);return;}eh_phase=5U;return;
        case 5U:
            if(!eh_control(stop,4U)){eh_fail(4U);return;}eh_phase=6U;return;
        case 6U:
            g_extension_hold_enable_count++;
            if(!eh_control(enable,5U)){eh_fail(4U);return;}eh_phase=7U;return;
        case 7U:
            if(!eh_query(0x3a,3U)){eh_fail(4U);return;}
            if(eh_reply.Data[1]&0x0cU){eh_fail(5U);return;}
            if(!(eh_reply.Data[1]&1U)){g_extension_hold_samples=0U;return;}
            eh_phase=8U;return;
        case 8U:
            if(!eh_query(0x35,5U)){eh_fail(4U);return;}
            if(eh_reply.Data[2] || eh_reply.Data[3]){eh_fail(5U);return;}eh_phase=9U;return;
        default:
            if(!eh_query(0x36,7U) || eh_reply.Data[1]>1U){eh_fail(4U);return;}
            raw=((uint32_t)eh_reply.Data[2]<<24)|((uint32_t)eh_reply.Data[3]<<16)|((uint32_t)eh_reply.Data[4]<<8)|eh_reply.Data[5];
            g_startup_extension_current_deg=(eh_reply.Data[1]?-1.0f:1.0f)*(float)raw*0.1f;
            {float delta=fabsf(g_startup_extension_current_deg-eh_hold_position);
             g_extension_hold_delta_mdeg=delta<4000000.0f?(uint32_t)(delta*1000.0f):0xffffffffU;
             if(!(delta<=VEHICLE_EXTENSION_HOLD_POSITION_TOL_DEG)){eh_fail(5U);return;}}
            if(++g_extension_hold_samples>=VEHICLE_EXTENSION_HOLD_CONFIRM_SAMPLES){
                g_extension_hold_confirmed=1U;g_extension_hold_ready_tick=now;eh_started=0U;
                g_extension_home_state=8U;g_extension_home_result=g_extension_home_reported=1U;g_startup_axes_state=4U;
            }else eh_phase=7U;
            return;
        }
    }
'''
 u=u.replace(marker,block)
 old='''        g_startup_extension_feedback_valid=1U;g_extension_home_capture=0U;eh_started=0U;
        g_extension_home_state=8U;g_extension_home_result=g_extension_home_reported=1U;g_startup_axes_state=4U;'''
 assert old in u
 u=u.replace(old,'''        g_startup_extension_feedback_valid=1U;g_extension_home_capture=0U;
        eh_hold_position=g_startup_extension_current_deg;g_startup_extension_target_deg=eh_hold_position;
        eh_hold_begin=now;eh_phase=3U;g_extension_home_state=9U;
        /* Not ready until STOP, one enable and three fresh readback bundles. */''')
 return u
edit('CHASSIS/USER/extension_home_only.inc',extension)
def telemetry(u):
 old='"EX1,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,"';assert old in u
 u=u.replace(old,'"EX2,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,"')
 old='(unsigned long)g_eh_first_tick);';assert u.count(old)==1
 return u.replace(old,'''(unsigned long)g_eh_first_tick,(unsigned long)g_extension_hold_enable_count,
        (unsigned long)g_extension_hold_samples,(unsigned long)g_extension_hold_confirmed,
        (unsigned long)g_extension_hold_ready_tick,(unsigned long)g_extension_hold_delta_mdeg);''')
edit('CHASSIS/USER/vehicle_mission.inc',telemetry)
m['changed']={k:hashlib.sha256((S/k).read_bytes()).hexdigest() for k in m['original']}
M.write_text(json.dumps(m,ensure_ascii=False,indent=2),encoding='utf-8');print('Prepared 4-file explicit post-home hold confirmation')
