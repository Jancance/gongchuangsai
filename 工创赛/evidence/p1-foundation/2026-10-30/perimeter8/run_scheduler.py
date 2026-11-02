"""Extract actual candidate admission/scheduler; stub segment completion, no hardware."""
import argparse
import json
from pathlib import Path
import re
import shutil
import struct
import subprocess
import sys

sys.path.insert(0,'C:\\Users\\lenovo\\Desktop\\工创赛\\工创赛\\scripts')
from p1_turn_fault_audit import function
ROOT=Path('C:\\Users\\lenovo\\Desktop\\工创赛')

def run(candidate,out):
    candidate=Path(candidate).resolve();out=Path(out).resolve();out.mkdir(parents=True,exist_ok=False)
    shutil.copytree(candidate/'USER',out/'src');shutil.copytree(Path('C:\\Users\\lenovo\\Desktop\\工创赛\\工创赛\\scripts')/'p1_offline_tests/stubs',out/'stubs')
    main=(out/'src/main.c').read_bytes().decode('gbk').replace('\r\n','\n')
    prefix=(Path('C:\\Users\\lenovo\\Desktop\\工创赛\\工创赛\\scripts')/'p1_offline_tests/route_speed.c').read_text(encoding='utf8').split('/* FRONT_TYPE */')[0]
    prefix='#include <stdint.h>\nstatic uint32_t g_vm_test_request,test_power;\n'+prefix
    prefix=prefix.replace('p->active=1U','p->active=test_power')
    end=main.index('} ForwardHoldDebug;')+len('} ForwardHoldDebug;');start=main.rfind('typedef struct',0,end)
    forward=main[start:end]
    end=main.index('} CombinedTestDebug;')+len('} CombinedTestDebug;');start=main.rfind('typedef struct',0,end)
    combined=main[start:end]
    types=forward+'\n'+combined
    prelude=r'''
#include "src/route_plan.h"
static ForwardHoldDebug g_forward_hold;
static CombinedTestDebug g_combined_test;
static MotionAction s_motion_actions[MOTION_ACTION_CAPACITY];
static uint32_t s_motion_action_count,g_debug_plan_id,s_motion_list_submit,s_motion_has_action;
#define MOTION_ACTION_COUNT s_motion_action_count
static uint32_t s_combined_turn,s_combined_mode,s_combined_base_rpm,s_combined_run_ms;
static float s_combined_distance_mm,s_combined_reference_mm_s;
static uint32_t s_motion_session_open,s_combined_keep_vm,s_combined_begin_ms,s_combined_stage_ms,s_combined_gap_ms,s_combined_end_ms,s_combined_powered_segment;
static uint32_t g_heading_exec_trip,g_heading_exec_phase,g_heading_exec_request,g_heading_exec_state,s_heading_exec,g_heading_stop_trace[4];
static float g_heading_exec_relative_deg,g_heading_exec_error_deg;
static uint32_t s_motion_turn_max_rpm,g_translation_hold_selftest,g_motion_plan_valid,s_serial_start_permit;
static uint32_t g_forward_hold_trace_count,g_motion_fault_diag_valid,g_can_jog_request;
static struct {float yaw_deg;}s_heading_source;
static ChassisAutoStatus mock_status;
static uint32_t step_calls,finish_calls,abort_calls,source_ok;
volatile uint32_t scheduler_context[8];
#define HR_IDLE 0U
#define HR_WAIT 1U
#define HR_RUN 2U
#define HR_DONE 3U
#define HR_FAULT 4U
#define HR_CANCELLED 5U
#define HS_IDLE 0U
#define COMP_ENTRY_IDLE 0U
static void HeadingSession_Init(uint32_t *s){*s=0;}
static void HWT101_DiagnosticArm(void){}
static uint32_t HWT101_TimingBegin(void){return 0;}
static void HWT101_TimingEnd(uint32_t a,uint32_t b){(void)a;(void)b;}
static void App_TimingRecordCompleted(uint32_t i,uint32_t session,uint32_t now){(void)i;(void)session;(void)now;}
static void MockReadStatus(ChassisAutoStatus *p){*p=mock_status;}
static void MockAbort(void){abort_calls++;}
#define ChassisAuto_ReadStatus MockReadStatus
#define ChassisAuto_Abort MockAbort
static void Power_Disable(void){test_power=0;}
static uint32_t App_MotionAccept(void){g_vm_test_request=test_power=1;return 1;}
static uint32_t App_HeadingRuntimeSource(uint32_t now){(void)now;return source_ok;}
static uint32_t App_HeadingRuntimeExclusive(void){return 1;}
static uint32_t App_HeadingRuntimeOwns(const ChassisAutoStatus *p){(void)p;return 1;}
static uint32_t App_ForwardHoldOwns(const ChassisAutoStatus *p){(void)p;return 1;}
static void App_HeadingRuntimeTrip(uint32_t cause){(void)cause;g_heading_exec_trip=1;}
static void App_ForwardHoldTrip(uint32_t cause){(void)cause;g_forward_hold.trip=1;}
static void App_HeadingRuntimeSafety(void){}
static void App_ForwardHoldSafety(void){}
static void segment_done(void){
 step_calls++;mock_status.active=0;mock_status.state=CHASSIS_AUTO_DONE;
 mock_status.result=1;mock_status.cause=0;mock_status.rotation_stop_mask=15;mock_status.cut_vm=0;
 mock_status.session++;g_heading_exec_phase=g_forward_hold.phase=HR_DONE;
}
static void App_HeadingRuntimeStep(void){if(g_heading_exec_phase!=HR_DONE)segment_done();}
static void App_ForwardHoldStep(void){if(g_forward_hold.phase!=HR_DONE)segment_done();}
static uint32_t MotionGetState(void){return MOTION_BUSY;}
static uint32_t MotionFinish(void){if(!g_vm_test_request)return 0;finish_calls++;g_vm_test_request=test_power=0;return 1;}
#define App_StageDistanceMm() s_combined_distance_mm
#define App_StageRunMs() s_combined_run_ms
static uint32_t App_MoveSubmit(uint32_t,uint32_t,uint32_t,float,float);
static uint32_t MoveStart(uint32_t d,uint32_t rpm,uint32_t ms){return App_MoveSubmit(d,rpm,ms,0,0);}
static uint32_t MoveDistanceStart(uint32_t d,uint32_t rpm,float cm){return App_MoveSubmit(d,rpm,MotionDistanceRunMsFor(d,rpm,cm*10),cm*10,0);}
'''
    prelude += "\nstatic uint32_t idle_status_calls,idle_position_calls;\nstatic void App_CanPollStep(void){idle_status_calls++;}\nstatic void MockIdleOdometryPoll(void){idle_position_calls++;}\n#define ChassisAuto_OdometryPoll MockIdleOdometryPoll\n"
    functions='\n'.join(function(main,n) for n in ['App_ComboIdlePoll','App_MotionPlanValid','App_MoveSubmit','App_PerimeterTurnStart','App_CombinedLoadAction','App_CombinedTestStop','App_CombinedTestSafety','App_CombinedTestStep'])
    tests=r'''
static void reset_scheduler(void){
 static const RouteRow rows[]={APP_ROUTE_ROWS(ROUTE_ROW)};RouteCheck rc;
 memset(&g_combined_test,0,sizeof(g_combined_test));memset(&g_forward_hold,0,sizeof(g_forward_hold));
 memset(&mock_status,0,sizeof(mock_status));
 test_tick=1000;g_vm_test_request=test_power=0;g_can_ctrl_fault_latched=0;
 g_heading_exec_trip=g_heading_exec_phase=g_heading_exec_request=0;
 s_motion_session_open=s_motion_has_action=s_combined_turn=0;
 step_calls=finish_calls=abort_calls=0;source_ok=g_safety_ready=g_can_guard_ready=1;
 CHECK(RoutePlan_Convert(rows,8,&g_chassis_geometry,s_motion_actions,&rc));s_motion_action_count=8;
 g_debug_plan_id=0;g_motion_plan_valid=App_MotionPlanValid();CHECK(g_motion_plan_valid);
 g_translation_hold_selftest=1;s_serial_start_permit=1;
}
uint32_t test_entry(void){
 uint32_t i;g_combined_test.state=0;
 App_ComboIdlePoll();CHECK(idle_status_calls==1 && idle_position_calls==0);
 g_combined_test.state=1;
 for(i=0;i<40;i++){uint32_t before=idle_status_calls+idle_position_calls;App_ComboIdlePoll();CHECK(idle_status_calls+idle_position_calls==before+1);}
 CHECK(idle_status_calls==21 && idle_position_calls==20);
 g_combined_test.state=2;for(i=0;i<40;i++)App_ComboIdlePoll();CHECK(idle_status_calls==41 && idle_position_calls==40);
 g_combined_test.state=4;App_ComboIdlePoll();CHECK(idle_status_calls==42 && idle_position_calls==40);
 reset_scheduler();g_combined_test.request=1;
 for(i=0;i<8;i++){
  if(i)test_tick+=1001;
  App_CombinedTestStep();
  if(i==7U && g_combined_test.state==1U)App_CombinedTestStep();
  if(g_combined_test.completed!=i+1 && !first_failure) {
   scheduler_context[0]=i;scheduler_context[1]=g_combined_test.state;scheduler_context[2]=g_combined_test.cause;
   scheduler_context[3]=g_combined_test.completed;scheduler_context[4]=step_calls;
   scheduler_context[5]=g_forward_hold.phase;scheduler_context[6]=g_heading_exec_phase;scheduler_context[7]=g_debug_plan_id;
  }
  CHECK(g_combined_test.completed==i+1);CHECK(g_combined_test.index==i);
  CHECK(!test_depth);CHECK(!g_combined_test.cause);
  if(s_motion_actions[i].kind==MOTION_TURN) {
   CHECK(s_motion_turn_max_rpm==27);CHECK(s_combined_distance_mm==0);CHECK(s_combined_reference_mm_s==0);
  }else CHECK(s_combined_mode==s_motion_actions[i].kind-MOTION_FORWARD_DISTANCE);
  if(i<7){CHECK(g_combined_test.state==2);App_CombinedTestStep();CHECK(g_combined_test.completed==i+1);}
 }
 CHECK(g_combined_test.state==3);CHECK(step_calls==8);CHECK(finish_calls==1);CHECK(!g_vm_test_request&&!test_power);
 App_CombinedTestStep();CHECK(step_calls==8);
 reset_scheduler();g_combined_test.request=1;App_CombinedTestStep();g_combined_test.request=2;
 App_CombinedTestSafety();CHECK(g_combined_test.state==5);CHECK(!g_vm_test_request&&!test_power);CHECK(abort_calls);
 reset_scheduler();g_combined_test.request=1;App_CombinedTestStep();test_tick=s_combined_begin_ms+APP_PERIMETER_GROUP_MS;
 App_CombinedTestSafety();CHECK(g_combined_test.state==4&&g_combined_test.cause==2);
 reset_scheduler();g_combined_test.request=1;App_CombinedTestStep();source_ok=0;
 App_CombinedTestSafety();CHECK(g_combined_test.state==4&&g_combined_test.cause==3);
 reset_scheduler();g_combined_test.request=1;App_CombinedTestStep();mock_status.session++;
 test_tick+=1001;App_CombinedTestStep();CHECK(g_combined_test.state==4&&g_combined_test.cause==4);
 reset_scheduler();s_motion_actions[1].reference_mm_s=201;CHECK(!App_MotionPlanValid());
 return first_failure?(0x80000000U|first_failure):checks;
}
'''
    (out/'unit.c').write_bytes((prefix+types+prelude+functions+tests).encode('gbk'))
    (out/'unit.sct').write_text('LR_ROM 0x08000000 0x00100000 { ER_ROM 0x08000000 0x00100000 { unit.o (i.test_entry, +First) *(+RO) } RW_RAM 0x20000000 0x00020000 { *(+RW,+ZI) } }\n')
    arm=Path('D:/Keil_v5/ARM/arm5_compiler/bin');logs=[]
    objects=[('unit.c','unit.o'),('src/chassis_kinematics.c','kinematics.o'),('src/motion_profile.c','profile.o')]
    commands=[[str(arm/'armcc.exe'),'--cpu','Cortex-M3','--c99','--split_sections','-O1','-I','src','-I','stubs','-c',s,'-o',o] for s,o in objects]
    commands.append([str(arm/'armlink.exe'),'--entry','test_entry','--scatter','unit.sct']+[o for s,o in objects]+['--library_type=microlib','-o','unit.axf'])
    for cmd in commands:
        p=subprocess.run(cmd,cwd=out,capture_output=True,text=True,errors='replace');logs.append(dict(command=cmd,returncode=p.returncode,output=p.stdout+p.stderr));(out/'build.json').write_text(json.dumps(logs,indent=2))
        if p.returncode:raise RuntimeError(p.stdout+p.stderr)
    sys.path.insert(0,str(ROOT/'工创赛/tmp/position_alignment_deps'));import unicorn as uc;from unicorn import arm_const as reg
    m=uc.Uc(uc.UC_ARCH_ARM,uc.UC_MODE_THUMB|uc.UC_MODE_MCLASS);m.mem_map(0x08000000,0x100000);m.mem_map(0x20000000,0x20000)
    b=(out/'unit.axf').read_bytes();h=struct.unpack_from('<16sHHIIIIIHHHHHH',b)
    for i in range(h[10]):
        kind,off,addr,_,size,_,_,_=struct.unpack_from('<IIIIIIII',b,h[5]+i*h[9])
        if kind==1 and size:m.mem_write(addr,b[off:off+size])
    stop=0x080ff000;m.reg_write(reg.UC_ARM_REG_SP,0x2001fff0);m.reg_write(reg.UC_ARM_REG_LR,stop|1);m.emu_start(h[4]|1,stop,count=10000000)
    value=m.reg_read(reg.UC_ARM_REG_R0);report=dict(passed=value<0x80000000 and m.reg_read(reg.UC_ARM_REG_PC)==stop,assertions=value if value<0x80000000 else None,failure_line=value&0x7fffffff if value&0x80000000 else None,hardware_accessed=False,limits='Actual route/admission/combined scheduler; segment completion/platform are stubs.')
    symbols=subprocess.check_output([str(arm/'fromelf.exe'),'--text','-s','unit.axf'],cwd=out,text=True)
    addr=int(re.search(r'\bscheduler_context\s+(0x[0-9a-fA-F]+)',symbols)[1],16)
    report['context']=list(struct.unpack('<8I',m.mem_read(addr,32)))
    (out/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
    if not report['passed']:raise RuntimeError('Scheduler C check failed')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--candidate',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args();run(a.candidate,a.out)
