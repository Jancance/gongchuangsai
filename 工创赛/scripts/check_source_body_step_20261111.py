"""Actual C owner/guard/backend-entry fault injection, simulated hardware."""
from pathlib import Path
import argparse,json,re,subprocess
R=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser();parser.add_argument('--project');parser.add_argument('--evidence')
args=parser.parse_args()
P=R/(args.project or 'tmp/source_body_step_20261111/project');U=P/'CHASSIS/USER'
E=R/(args.evidence or 'evidence/integration/2026-11-11/source-body-step');E.mkdir(parents=True,exist_ok=True)
proof=json.loads((R/'evidence/integration/2026-11-11/source-body-step/matlab-result.json').read_text());assert proof['passed']
def function(text,name):
    match=re.search(r'^(?:static\s+)?(?:uint32_t|void|float)\s+'+name+r'\([^;]*?\)\s*\{',text,re.M|re.S)
    assert match,name
    start=match.start();pos=match.end();level=1
    while level:
        if text[pos]=='{':level+=1
        elif text[pos]=='}':level-=1
        pos+=1
    return text[start:pos]
def build(name,source):
    src=E/(name+'.c');exe=E/(name+'.exe');src.write_text(source,encoding='utf-8')
    p=subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-Wno-misleading-indentation',
        '-Wno-unused-function','-I',str(U),str(src),'-lm','-o',str(exe)],capture_output=True,text=True)
    log=p.stdout+p.stderr
    if p.returncode==0:
        p=subprocess.run([str(exe)],capture_output=True,text=True,timeout=15);log+=p.stdout+p.stderr
    (E/(name+'.log')).write_text(log,encoding='utf-8');print(log,flush=True);assert p.returncode==0,name

prefix=r'''
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
static unsigned checks;
#define CK(x) do{checks++;if(!(x)){fprintf(stderr,"FAIL %d %s\n",__LINE__,#x);exit(1);}}while(0)
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define LRC_DONE 4U
#define MOVE_LEFT 1U
#define MOTION_DONE 2U
#define MOTION_CANCELLED 3U
#define MOTION_FAULT 4U
typedef struct{unsigned state,session,active,rotation_stop_mask;}ChassisAutoStatus;
typedef struct{unsigned state,backend_active,distance_mode,distance_reached,distance_direction,backend_result,backend_cause,stop_mask;float distance_target_cm;}MotionStatus;
typedef struct{unsigned top_scale;int safe_position_raw;}LiftRulerController;
typedef struct{unsigned state;}CombinedTest;
typedef struct{float unused;}MotionProfile;
static LiftRulerController g_lift_ruler_controller;
static CombinedTest g_combined_test;
static ChassisAutoStatus fake_body;
static MotionStatus fake_status;
static unsigned hold=1,heading=1,exclusive=1,step_power_hold=1;
static unsigned backend_submit=1,submit_calls,stop_calls,finish_calls;
static volatile unsigned g_lift_compare_top_valid,g_lift_compare_backoff_valid,g_vehicle_lift_reference_valid,g_vehicle_lift_feedback_valid;
static unsigned g_lift_ruler_state,g_lift_ruler_request,g_lift_ruler_cause,g_lift_ruler_stop_pending,g_lift_ruler_samples;
static volatile unsigned g_vehicle_arm_stop_pending,g_vehicle_arm_cause,g_it_state,g_it_request,g_it_cause,g_it_rejected,g_it_auto_state,g_it_sequence;
static unsigned g_chassis_auto_active,g_safety_fault,g_rtos_fault,g_can_ctrl_fault_latched,g_safety_ready,g_can_test_result,g_can_guard_ready;
static unsigned s_motion_session_open,s_motion_has_action,s_motion_list_submit;
static float g_vehicle_lift_current_deg;
static struct{unsigned valid;}s_route_heading_reference;
static struct{float yaw_deg;}s_heading_source;
static float s_route_heading_target;
static int RouteHeading_Ensure(void*p,float yaw,unsigned sticky){(void)p;(void)yaw;CK(sticky==1);return 1;}
static int RouteHeading_Target(void*p,float offset,float*target){(void)p;CK(offset==0);*target=s_heading_source.yaw_deg;return 1;}
static int StartupAxes_Holding(void){return hold;}
static int VehicleArm_Busy(void){return 0;}
static int Stage2Body_Owns(void){return 0;}
static int App_HeadingRuntimeSource(unsigned now){(void)now;return heading;}
static int App_HeadingRuntimeExclusive(void){return exclusive;}
static void ChassisAuto_ReadStatus(ChassisAutoStatus *out){*out=fake_body;}
static int MotionReadStatus(MotionStatus *out){*out=fake_status;return 1;}
static void MotionStop(void){stop_calls++;}
static int MotionFinish(void){finish_calls++;s_motion_session_open=0;return step_power_hold;}
static int MotionRoute_Build(unsigned direction,float distance,float speed,MotionProfile*p,unsigned*rpm,unsigned*run)
{(void)p;CK(direction==1 && distance==10 && speed==15);*rpm=6;*run=4843;return 1;}
static int App_MoveSubmit(unsigned d,unsigned rpm,unsigned run,float distance,float speed)
{CK(d==1 && rpm==6 && run==4843 && distance==10 && speed==15 && s_motion_list_submit==1);submit_calls++;s_motion_session_open=backend_submit;return backend_submit;}
'''
owner=(U/'source_body_step.inc').read_text(encoding='utf-8')
owner_tests=r'''
static void reset(void){
 g_source_body_step_request=g_source_body_step_state=g_source_body_step_reason=g_source_body_step_submitted=g_source_body_step_session=0;
 g_lift_compare_top_valid=g_lift_compare_backoff_valid=g_vehicle_lift_reference_valid=g_vehicle_lift_feedback_valid=1;
 g_lift_ruler_state=4;g_lift_ruler_controller.top_scale=100;g_lift_ruler_controller.safe_position_raw=81000;g_vehicle_lift_current_deg=810;
 g_it_state=2;g_it_auto_state=2;g_it_request=g_it_cause=g_it_rejected=0;
 g_lift_ruler_request=g_lift_ruler_cause=g_lift_ruler_stop_pending=g_lift_ruler_samples=0;
 g_vehicle_arm_stop_pending=g_vehicle_arm_cause=0;g_it_sequence=12;
 g_chassis_auto_active=g_safety_fault=g_rtos_fault=g_can_ctrl_fault_latched=0;
 g_safety_ready=g_can_test_result=1;g_can_guard_ready=0;
 s_motion_session_open=s_motion_has_action=s_motion_list_submit=0;g_combined_test.state=0;
 hold=heading=exclusive=backend_submit=step_power_hold=1;submit_calls=stop_calls=finish_calls=0;
 memset(&fake_body,0,sizeof(fake_body));memset(&fake_status,0,sizeof(fake_status));
}
static void begin(void){g_source_body_step_request=1;SourceBodyStep_Service(100);CK(g_source_body_step_state==1);}
static void submit(void){begin();g_can_guard_ready=1;SourceBodyStep_Service(120);CK(g_source_body_step_state==2 && submit_calls==1);}
int main(void){
 reset();begin();CK(SourceBodyStep_KeepsPower() && SourceBodyStep_WheelWindow());
 SourceBodyStep_Service(110);CK(submit_calls==0);g_can_guard_ready=1;SourceBodyStep_Service(120);CK(submit_calls==1);
 fake_body.session=17;fake_body.active=1;fake_status.backend_active=1;fake_status.distance_mode=1;
 SourceBodyStep_Service(130);CK(g_source_body_step_session==17);
 fake_body.active=0;fake_status.backend_active=0;fake_status.state=MOTION_DONE;fake_status.distance_reached=1;
 fake_status.distance_direction=1;fake_status.distance_target_cm=1;fake_status.backend_result=1;fake_status.stop_mask=15;
 SourceBodyStep_Service(140);CK(g_source_body_step_state==3 && finish_calls==1 && SourceBodyStep_KeepsPower() && !SourceBodyStep_WheelWindow());
 SourceBodyStep_Service(150);CK(submit_calls==1);g_source_body_step_request=1;SourceBodyStep_Service(160);CK(submit_calls==1);
 reset();submit();fake_status.state=MOTION_DONE;SourceBodyStep_Service(130);CK(g_source_body_step_state==8 && !finish_calls && stop_calls==1);
 fake_body.rotation_stop_mask=0;SourceBodyStep_Service(140);CK(g_source_body_step_state==8);
 fake_body.rotation_stop_mask=15;SourceBodyStep_Service(150);CK(g_source_body_step_state==9 && g_source_body_step_reason==7);
 reset();begin();g_source_body_step_request=9;SourceBodyStep_Service(110);CK(g_source_body_step_state==9 && !submit_calls && !stop_calls);
 reset();submit();g_source_body_step_request=9;SourceBodyStep_Service(130);CK(stop_calls==1 && g_source_body_step_state==8 && SourceBodyStep_KeepsPower());
 reset();begin();heading=0;SourceBodyStep_Service(110);CK(g_source_body_step_state==9 && !submit_calls);
 reset();submit();g_lift_ruler_samples++;SourceBodyStep_Service(130);CK(g_source_body_step_state==8 && stop_calls==1 && SourceBodyStep_KeepsPower());
 reset();begin();SourceBodyStep_Service(12100);CK(g_source_body_step_state==9 && !submit_calls);
 reset();begin();backend_submit=0;g_can_guard_ready=1;SourceBodyStep_Service(120);CK(g_source_body_step_state==9 && g_source_body_step_reason==5);
 reset();g_it_state=1;g_source_body_step_request=1;SourceBodyStep_Service(100);CK(!submit_calls && g_source_body_step_reason==1);
 printf("PASS actual body-step owner %u assertions; stale DONE/cancel/changed mechanics/heading loss/deadline/one-shot; simulated backend.\n",checks);
}
'''
build('owner-check',prefix+owner+owner_tests)

guard=(P/'CHASSIS/SYSTEM/SAFETY/safety.c').read_text(encoding='gb18030')
guard_fn=function(guard,'Safety_CanGuardUpdate')
guard_unit=r'''
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
typedef uint32_t TickType_t;
static unsigned checks,clock_ms;
#define CK(x) do{checks++;if(!(x)){fprintf(stderr,"FAIL %d %s\n",__LINE__,#x);exit(1);}}while(0)
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define pdMS_TO_TICKS(x) (x)
#define SAFETY_CAN_FRESH_MS 1000U
#define SAFETY_CAN_BOOT_MS 3000U
#define APP_HEADING_POWER_SETTLE_MS 1000U
#define SAFETY_FAULT_CAN_PROTECT 0x40U
#define SAFETY_FAULT_CAN_DATA 0x20U
static unsigned g_can_guard_ready,g_can_guard_bad_nodes,g_can_guard_cause,g_safety_fault;
volatile unsigned g_lift_ruler_state=4,g_startup_axes_state=4,g_chassis_auto_request,g_it_state=2,g_it_lift_permit;
static unsigned s_can_requested,s_can_was_ready,s_can_power_tick,s_can_enabled=1,s_can_valid=15,s_can_protect;
static unsigned s_can_ticks[4],window;
static unsigned xTaskGetTickCount(void){return clock_ms;}
static int SourceBodyStep_WheelWindow(void){return window;}
'''
guard_tests=r'''
int main(void){
 clock_ms=100;Safety_CanGuardUpdate(1);CK(!g_can_guard_ready && !g_safety_fault);
 window=1;Safety_CanGuardUpdate(1);CK(!g_can_guard_ready && !g_safety_fault);
 clock_ms=1500;for(int i=0;i<4;i++)s_can_ticks[i]=1400+i*10;
 Safety_CanGuardUpdate(1);CK(g_can_guard_ready && !g_safety_fault);
 s_can_ticks[2]=0;Safety_CanGuardUpdate(1);CK(!g_can_guard_ready && (g_safety_fault&0x20) && g_can_guard_bad_nodes==4);
 g_safety_fault=0;s_can_ticks[2]=1420;s_can_protect=8;
 Safety_CanGuardUpdate(1);CK(!g_can_guard_ready && (g_safety_fault&0x40) && g_can_guard_bad_nodes==8);
 printf("PASS actual guard %u assertions: DONE/HOLD skipped only without wheel window; real stale/protect still fail.\n",checks);
}
'''
build('guard-check',guard_unit+guard_fn+guard_tests)

backend=(U/'chassis_auto_test.c').read_text(encoding='gb18030')
held=function(backend,'ChassisAuto_BeginHeldSequence');ordinary=function(backend,'ChassisAuto_BeginSequence')
held_unit=r'''
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
static unsigned checks;
#define CK(x) do{checks++;if(!(x)){fprintf(stderr,"FAIL %d %s\n",__LINE__,#x);exit(1);}}while(0)
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define CHASSIS_AUTO_IDLE 0U
typedef struct{unsigned active;}PowerCommandStatus;
static unsigned window=1,holding=1,powered=1,healthy=1;
static unsigned s_vm_requested=1,s_sequence_hold,s_used,s_sequence_continue;
static unsigned g_chassis_auto_active,g_chassis_auto_cut_vm,g_chassis_auto_result,g_chassis_auto_cause,g_can_ctrl_fault_latched,g_chassis_auto_state,g_rtos_fault,g_safety_fault;
static int SourceBodyStep_WheelWindow(void){return window;}
static int StartupAxes_Holding(void){return holding;}
static void Power_ReadCommandStatus(PowerCommandStatus*p){p->active=powered;}
static int Auto_BaseHealthy(void){return healthy;}
'''
held_tests=r'''
int main(void){
 CK(!ChassisAuto_BeginSequence());CK(ChassisAuto_BeginHeldSequence() && s_sequence_hold);
 s_sequence_hold=0;healthy=0;CK(!ChassisAuto_BeginHeldSequence());healthy=1;
 g_chassis_auto_cause=3;CK(!ChassisAuto_BeginHeldSequence() && g_chassis_auto_cause==3);g_chassis_auto_cause=0;
 g_chassis_auto_result=2;CK(!ChassisAuto_BeginHeldSequence() && g_chassis_auto_result==2);g_chassis_auto_result=0;
 holding=0;CK(!ChassisAuto_BeginHeldSequence());holding=1;
 powered=0;CK(!ChassisAuto_BeginHeldSequence());powered=1;
 s_vm_requested=0;CK(ChassisAuto_BeginHeldSequence() && s_vm_requested==1);s_sequence_hold=0;
 window=0;CK(!ChassisAuto_BeginHeldSequence());s_vm_requested=0;CK(ChassisAuto_BeginSequence());
 printf("PASS actual held/normal backend entry %u assertions; physical power/guard/holding required, no fault/result cleared.\n",checks);
}
'''
build('held-entry-check',held_unit+held+ordinary+held_tests)

# Check actual integration dispatch: mechanics cannot start in wheel fault/
# pendingSTOP states, but the existing global stop request still passes.
integration=(U/'integration10_complete.inc').read_text(encoding='utf-8')
start=integration.index(' if(SourceBodyStep_WheelWindow() && request){')
end=integration.index(' /*',start)
dispatch=integration[start:end]
dispatch_unit=r'''
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
static unsigned window=1,g_source_body_step_request,g_it_rejected;
static int SourceBodyStep_WheelWindow(void){return window;}
static unsigned dispatch(unsigned request){
'''+dispatch+r'''
return request;}
int main(void){
if(dispatch(16)!=0 || g_it_rejected!=13)return 1;
if(dispatch(9)!=9 || g_source_body_step_request!=9)return 2;
window=0;if(dispatch(16)!=16)return 3;
puts("PASS actual mechanical dispatch rejects launch throughout wheel ownership/fault, preserves STOP, restores normal dispatch after DONE.");return 0;}
'''
build('dispatch-check',dispatch_unit)

# Compare the actual backend's trajectory formula against local MATLAB.
profile=(U/'motion_profile.c').read_text(encoding='gb18030')
helpers='\n'.join(function(profile,n) for n in ['Profile_Finite','Profile_Min','Profile_Abs','Profile_LimitsValid','MotionProfile_Prepare'])
trajectory='#include <stdint.h>\n#include <stdio.h>\n#include <math.h>\n#include <float.h>\n#include <stdlib.h>\n#include "motion_profile.h"\n'+helpers
trajectory+=f'''\nint main(void){{MotionProfile p;MotionProfileLimits lim={{15.0f,{proof['reserved_acceleration_mm_s2']}f,{proof['reserved_jerk_mm_s3']}f}};
if(!MotionProfile_Prepare(&p,10.0f,&lim) || fabsf(p.duration_s-{proof['profile_duration_s']}f)>0.00001f)return 1;
printf("PASS actual C10mm trajectory agrees MATLAB duration%.6fs; not motor tracking.\\n",p.duration_s);return 0;}}\n'''
build('trajectory-check',trajectory)
(E/'checks.json').write_text(json.dumps(dict(passed=True,hardware_access=False,actual_owner_and_guard_and_backend_entry=True,
    matlab_c_trajectory_match=True,mechanical_scheduler_unchanged=(U/'integration13_immediate.inc').read_bytes()==(R/'tmp/pzq_release50_20261111/project/CHASSIS/USER/integration13_immediate.inc').read_bytes()),indent=2)+'\n')
