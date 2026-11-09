"""Remove reverse-sign faults from the actual chassis backend; offline verification."""
from pathlib import Path
import hashlib, json, shutil, subprocess

R=Path(__file__).resolve().parents[1]
A=Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
S=R/'tmp/remove_reverse_fault_20261103'
E=R/'evidence/integration/2026-11-03/remove-reverse-fault'
E.mkdir(parents=True,exist_ok=True)
if not S.exists():
    shutil.copytree(A,S,ignore=shutil.ignore_patterns('Objects','ObjectsOffline','Listings','.git'))
changed=['CHASSIS/USER/chassis_direction_guard.h','CHASSIS/USER/chassis_auto_test.c']
before={}
for rel in changed:
    p=E/'before-install'/rel;p.parent.mkdir(parents=True,exist_ok=True)
    shutil.copyfile(A/rel,p);before[rel]=hashlib.sha256(p.read_bytes()).hexdigest()
header='''#ifndef CHASSIS_DIRECTION_GUARD_H
#define CHASSIS_DIRECTION_GUARD_H
#include <stdint.h>

/* User decision 2026-11-03: no reverse-sign fault or reverse timeout.
 * Keep successful command acknowledgement and sample metadata for telemetry.
 * Missing command acknowledgement remains a separate fault (detail 12).
 * Communication freshness, speed magnitude, position and stop checks live
 * in the backend and remain independent of feedback sign. */
#define DIRECTION_NO_TARGET 12U
typedef struct {
    int32_t target_deci;
    uint32_t valid, ack_ms, direction_ms, sample_ms, sample_seen;
    uint32_t reverse_samples; /* Legacy HF3 field: always zero. */
} ChassisDirectionGuard;

static __inline void DirectionGuard_Reset(ChassisDirectionGuard *s)
{ ChassisDirectionGuard empty = {0}; *s = empty; }

static __inline void DirectionGuard_Ack(ChassisDirectionGuard *s,
    int32_t target_deci, uint32_t now_ms)
{
    if (!s->valid || !s->target_deci || !target_deci ||
        ((s->target_deci < 0) != (target_deci < 0))) {
        s->direction_ms = now_ms;
        s->sample_seen = 0U;
    }
    s->target_deci = target_deci; s->ack_ms = now_ms; s->valid = 1U;
}

static __inline uint32_t DirectionGuard_Check(ChassisDirectionGuard *s,
    uint32_t now_ms, uint32_t sample_ms)
{
    if (!s->valid) return DIRECTION_NO_TARGET;
    if ((uint32_t)(now_ms-sample_ms) >= 0x80000000U ||
        (uint32_t)(sample_ms-s->direction_ms) >= 0x80000000U) return 0U;
    if (s->sample_seen && (uint32_t)(sample_ms-s->sample_ms) >= 0x80000000U) return 0U;
    s->sample_seen=1U; s->sample_ms=sample_ms;
    return 0U;
}
#endif
'''
(S/changed[0]).write_bytes(header.replace('\n','\r\n').encode('ascii'))
p=S/changed[1];s=(A/changed[1]).read_bytes().decode('gbk').replace('\r\n','\n')
lo=s.index('                uint32_t endpoint_window=0U,reason;')
hi=s.index('                if(reason)',lo)
s=s[:lo]+'''                uint32_t reason;
                /* Reverse feedback sign is not a fault (user decision). */
                reason=DirectionGuard_Check(&s_direction_guard[i],(uint32_t)now*portTICK_PERIOD_MS,
                    (uint32_t)s_speed_tick[i]*portTICK_PERIOD_MS);
'''+s[hi:]
lo=s.index('    /* Pending low-speed reverse remains bounded')
hi=s.index('    run_age = (TickType_t)(now - s_run_tick);',lo)
s=s[:lo]+s[hi:]
# Replace the now-misleading speed-check comment without altering other text.
lo=s.index('            /*',s.index('TickType_t direction_age ='))
hi=s.index('            if (!(raw',lo)
s=s[:lo]+'            /* Keep magnitude limit; feedback sign alone never trips. */\n'+s[hi:]
p.write_bytes(s.replace('\n','\r\n').encode('gbk'))

base=(R/'evidence/integration/2026-11-03/no-motion/handoff-backend-unit.c').read_text(encoding='utf-8')
base=base[:base.index('int main(void)')]
unit=base+r'''
static void seed(int32_t target,float speed){
 ChassisAutoPreparedTranslation p;reset();
 ck(ChassisAuto_BeginSequence());ChassisAuto_SafetyUpdate(1);ck(prepare(&p));ck(ChassisAuto_BeginPreparedTranslation(&p,0));
 test_tick=140723;s_total_tick=138800;s_run_tick=138800;s_run_started=1;g_chassis_auto_state=CHASSIS_AUTO_START;
 s_translation_base_rpm=1;s_distance_progress_mm=145.572f;
 s_distance_reference=(MotionProfileSample){150,0,0,0};
 DirectionGuard_Reset(&s_direction_guard[0]);DirectionGuard_Ack(&s_direction_guard[0],target,140712);
 g_chassis_auto_driver_rpm[0]=speed;s_speed_tick[0]=test_tick;
}
int main(void){
 /* Both signs, endpoint and non-endpoint, sustained and larger reverse. */
 for(int sign=-1;sign<=1;sign+=2)for(unsigned endpoint=0;endpoint<=1;endpoint++){
  seed(sign*1000,-sign*100);if(!endpoint)s_distance_progress_mm=20;
  for(unsigned n=0;n<10;n++){
   test_tick+=100;s_speed_tick[0]=test_tick;Auto_CheckWheelSpeedLocked(0,test_tick);
   ck(g_chassis_auto_active && !g_chassis_auto_cause && !g_chassis_auto_fault_diagnostic.reason);
  }
  ck(s_direction_guard[0].reverse_samples==0);
 }
 seed(20,-2);Auto_CheckWheelSpeedLocked(0,test_tick);ck(g_chassis_auto_active);
 /* Absolute overspeed, unacknowledged commands and stale telemetry still trip. */
 seed(20,-10000);Auto_CheckWheelSpeedLocked(0,test_tick);
 ck(!g_chassis_auto_active && g_chassis_auto_fault_diagnostic.reason==9);
 seed(20,-2);s_direction_guard[0].valid=0;Auto_CheckWheelSpeedLocked(0,test_tick);
 ck(!g_chassis_auto_active && g_chassis_auto_fault_diagnostic.reason==12);
 seed(20,-2);s_heading_command_tick=test_tick;Auto_CheckLocked(test_tick);
 ck(!g_chassis_auto_active && g_chassis_auto_fault_diagnostic.reason==2);
 /* Original warm-VM admission fix and global fault rejection remain. */
 reset();ck(App_MotionAccept());ck(s_vm_requested);
 for(unsigned fault=0;fault<3;fault++){
  seed(20,-2);if(!fault)g_rtos_fault=1;if(fault==1)g_safety_fault=1;if(fault==2)g_can_ctrl_fault_latched=1;
  Auto_CheckLocked(test_tick);ck(!g_chassis_auto_active);
 }
 ck(!test_depth && !hardware_calls);
 printf("REMOVE_REVERSE checks=%u failures=%u\n",checks,failures);return failures?1:0;
}
'''
u=E/'backend-unit.c';u.write_text(unit,encoding='utf-8');exe=E/'backend-unit.exe'
cmd=['C:/mingw/mingw64/bin/gcc.exe','-std=c99','-DAPP_MOTOR_OFFLINE_ONLY=0',
     '-I'+str(R/'evidence/integration/2026-11-03/no-motion/backend-stubs'),'-I'+str(S/'CHASSIS/USER'),str(u),
     str(S/'CHASSIS/USER/chassis_kinematics.c'),str(S/'CHASSIS/USER/motion_profile.c'),'-lm','-o',str(exe)]
c=subprocess.run(cmd,capture_output=True,text=True,errors='replace');assert c.returncode==0,c.stderr
c=subprocess.run([str(exe)],capture_output=True,text=True)
(E/'backend-check.log').write_text(c.stdout+c.stderr,encoding='utf-8');print(c.stdout)
assert c.returncode==0,c.stdout
m=dict(actual=str(A),stage=str(S),original=before,changed={rel:hashlib.sha256((S/rel).read_bytes()).hexdigest() for rel in changed},
       checks_passed=True,actual_complete_backend=True,peripherals_mocked=True,hardware_accessed=False,installed=False,flashed=False)
(E/'manifest.json').write_text(json.dumps(m,ensure_ascii=False,indent=2),encoding='utf-8')
