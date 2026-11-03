"""Execute actual stage wiring and combined-route code with mocked peripherals."""
from pathlib import Path
from vehicle_check_paths import firmware_root
import importlib.util
import json
import subprocess

ROOT=Path(__file__).resolve().parents[1]
FW=firmware_root()/'CHASSIS'
OUT=ROOT/'evidence/integration/2026-11-03/early-scan-startup'


def function(text, name):
    start=text.index('static void '+name+'(')
    lo=text.index('{',start);depth=1;end=lo+1
    while depth:
        depth+=(text[end]=='{')-(text[end]=='}');end+=1
    return text[start:end]


spec=importlib.util.spec_from_file_location('old_stage1',ROOT/'scripts/check_map_stage1.py')
old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
unit=old.UNIT[:old.UNIT.index('static void reset_fixture(void)')]
unit=unit.replace('typedef struct {uint32_t trip;} ForwardHoldDebug;',
    'typedef struct {uint32_t trip,phase,request,run_started,backend_session;float error_deg;} ForwardHoldDebug;')
unit=unit.replace('typedef struct {uint32_t request,state,index,cause,completed;} CombinedTestDebug;',
    'typedef struct {uint32_t request,state,index,cause,completed,phase[8],session[8],elapsed_ms[8];float final_yaw[8],final_error[8];} CombinedTestDebug;')
unit=unit.replace('uint32_t active,result,cause,rotation_stop_mask,session;',
    'uint32_t active,result,cause,rotation_stop_mask,session,state,distance_reached,cut_vm;')
unit=unit.replace('#include "stage1_runtime.inc"',r'''
#define CHASSIS_AUTO_IDLE 0U
#define CHASSIS_AUTO_DONE 9U
uint32_t g_stage1_stop_requested,g_stage1_motion_skipped,SAOMA_DataReady;
uint32_t g_startup_axes_state;
static uint32_t StartupAxes_Holding(void){return g_startup_axes_state==4;}
static uint32_t StartupAxes_Begin(void){g_startup_axes_state=4;return 1;}
static void StartupAxes_Cancel(void){g_startup_axes_state=6;}

static uint32_t display_result=SCREEN_OK,display_calls;
static char displayed[16];
static uint32_t Screen_ShowTaskCode(const char *code){display_calls++;strcpy(displayed,code);return display_result;}
#include "stage1_runtime.inc"
''')
unit+=r'''
#define HR_WAIT 1U
#define HR_RUN 2U
#define HR_DONE 3U
#define HR_FAULT 4U
#define HR_CANCELLED 5U
#define MOTION_BUSY 1U
#define MOTION_FAULT 4U
#define MOTION_CANCELLED 5U
#define APP_PERIMETER_GROUP_MS 300000U
static uint32_t s_combined_end_ms,s_combined_begin_ms,s_combined_stage_ms,s_combined_gap_ms;
static uint32_t s_motion_list_submit,g_translation_hold_selftest,g_motion_plan_valid;
static uint32_t g_heading_exec_phase_dummy,g_radar_servo_departure_done=1;
static float g_heading_exec_error_deg;
static struct {float yaw_deg;} s_heading_source;
static uint32_t load_calls,finish_calls,fixture_phase=HR_DONE;
uint32_t g_chassis_auto_request,g_can_jog_request;
static uint32_t App_HeadingRuntimeSource(uint32_t now){return 1;}
static uint32_t App_HeadingRuntimeExclusive(void){return 1;}
static uint32_t RouteHeading_Init(RouteHeadingReference *r,float f){r->valid=1;return 1;}
static uint32_t App_CombinedLoadAction(uint32_t index){load_calls++;return 1;}
static uint32_t MotionGetState(void){return 0;}
static float App_StageDistanceMm(void){return 150;}
static uint32_t App_StageRunMs(void){return 10000;}
static void App_CombinedTestStop(uint32_t cause){g_combined_test.state=cause==1?5:4;g_combined_test.cause=cause;}
static void App_HeadingRuntimeStep(void){g_heading_exec_phase=fixture_phase;}
static void App_ForwardHoldStep(void){g_forward_hold.phase=fixture_phase;}
static uint32_t RadarServo_OnDeparture(uint32_t zone){return 1;}
static uint32_t RadarZoneSwitch_Lock(void){return 1;}
static uint32_t App_HeadingRuntimeOwns(const ChassisAutoStatus *p){return 1;}
static uint32_t App_ForwardHoldOwns(const ChassisAutoStatus *p){return 1;}
static uint32_t MotionFinish(void){finish_calls++;g_vm_test_request=0;s_motion_session_open=0;s_combined_keep_vm=0;return 1;}
static void App_TimingRecordCompleted(uint32_t a,uint32_t b,uint32_t c){}
'''
main=(FW/'USER/main.c').read_bytes().decode('gbk')
unit+=function(main,'App_CombinedTestStep')
unit+=r'''
static void reset(void){
 memset((void*)&g_combined_test,0,sizeof(g_combined_test));memset((void*)&g_forward_hold,0,sizeof(g_forward_hold));
 memset(&fixture_backend,0,sizeof(fixture_backend));memset(&fixture_power,0,sizeof(fixture_power));
 g_stage1_state=g_stage1_cause=g_stage1_key_request=g_stage1_denied=g_stage1_scan_valid=g_stage1_run_id=0;
 g_stage2_ready=g_stage2_run_id=g_stage1_stop_requested=g_stage1_motion_skipped=g_startup_axes_state=0;
 g_heading_exec_trip=g_serial_motion_link_lost=g_safety_fault=g_rtos_fault=g_can_ctrl_fault_latched=0;
 s_motion_session_open=g_vm_test_request=stop_calls=0;g_radar_zone_ready=1;g_radar_zone_selected=1;
 g_saoma_accept_count=g_scan_display_count=10;g_saoma_last_result=1;g_scan_display_result=SCREEN_OK;
 g_saoma_frame_start_ms=g_saoma_frame_end_ms=0;ready=rearm_ok=tx_ok=1;tick=100;
 display_result=SCREEN_OK;display_calls=load_calls=finish_calls=0;fixture_phase=HR_DONE;
 g_forward_hold.phase=HR_IDLE;s_combined_stage_ms=0;
 strcpy(SAOMA_Data,"156+123+516+231");
}
static void start(void){g_stage1_key_request=1;App_Stage1Service();if(g_stage1_state==7)App_Stage1Service();g_startup_axes_state=0;}
static void frame(const char *code,uint32_t begin,uint32_t end){strcpy(SAOMA_Data,code);g_saoma_accept_count++;g_saoma_frame_start_ms=begin;g_saoma_frame_end_ms=end;SAOMA_DataReady=1;}
static void stopped(void){g_combined_test.state=3;fixture_backend.state=CHASSIS_AUTO_DONE;fixture_backend.result=1;fixture_backend.rotation_stop_mask=15;fixture_backend.active=0;fixture_power.active=g_vm_test_request=s_motion_session_open=0;}
static void run_tests(void){
 FieldStage1Plan p;ck(FieldMap_Stage1(&p));ck(p.forward_cm==15 && p.left_cm==105);
 reset();ready=0;start();ck(!g_stage1_run_id && g_stage1_denied==1);
#if APP_MOTOR_OFFLINE_ONLY
 ready=1;start();ck(!g_stage1_run_id && !g_combined_test.request);return;
#else
 reset();start();ck(g_stage1_state==1 && g_stage1_run_id==1);
 frame("156+123+516+231",90,180);tick=200;App_Stage1Service();ck(!g_stage1_scan_valid);
 frame("156+123+516+236",110,190);App_Stage1Service();ck(!g_stage1_scan_valid);
 frame("156+123+516+231",110,120);tick=800;App_Stage1Service();ck(!g_stage1_scan_valid);
 frame("156+123+516+231",810,820);tick=830;App_Stage1Service();
 ck(g_stage1_state==6 && g_stage1_scan_valid && g_stage1_stop_requested && !stop_calls && !g_stage2_ready);
 App_Stage1Display(1);ck(!display_calls);
 frame("123+123+321+321",840,850);tick=860;App_Stage1Service();ck(!strcmp(g_stage1_task_code,"156+123+516+231"));
 stopped();fixture_backend.rotation_stop_mask=14;App_Stage1Service();ck(!g_stage2_ready && g_stage1_state==6);
 fixture_backend.rotation_stop_mask=15;fixture_power.active=1;App_Stage1Service();ck(g_stage1_state==6);
 fixture_power.active=0;App_Stage1Service();ck(g_stage1_state==2 && !g_stage2_ready);
 App_Stage1Display(0);ck(!display_calls);App_Stage1Display(1);ck(display_calls==1 && !strcmp(displayed,"156+123+516+231"));
 App_Stage1Service();ck(g_stage1_state==3 && g_stage2_ready && g_stage2_run_id==1);
 App_Stage1Display(1);ck(display_calls==1);g_safety_fault=1;App_Stage1Service();ck(!g_stage2_ready && g_stage1_state==4);
 reset();start();tick=200;stopped();g_combined_test.completed=2;App_Stage1Service();ck(g_stage1_state==2);
 frame("123+123+321+321",210,220);tick=230;App_Stage1Service();App_Stage1Display(1);App_Stage1Service();
 ck(g_stage2_ready && !g_stage1_stop_requested && !strcmp(displayed,"123+123+321+321"));
 reset();start();tick=200;stopped();App_Stage1Service();tick=8200;App_Stage1Service();ck(g_stage1_state==4 && g_stage1_cause==3);
 reset();start();frame("156+123+516+231",110,120);tick=130;App_Stage1Service();tick=5130;App_Stage1Service();ck(stop_calls==1 && g_stage1_cause==8 && !g_stage2_ready);
 reset();start();frame("156+123+516+231",110,120);tick=130;App_Stage1Service();g_stage1_key_request=2;App_Stage1Service();ck(g_stage1_state==5 && stop_calls==1 && !g_stage2_ready);
 reset();start();tick=200;stopped();App_Stage1Service();frame("156+123+516+231",210,220);tick=230;App_Stage1Service();display_result=0;
 App_Stage1Display(1);tick+=250;App_Stage1Display(1);tick+=250;App_Stage1Display(1);tick+=250;App_Stage1Display(1);
 ck(display_calls==3 && g_stage1_state==4 && g_stage1_cause==7 && !g_stage2_ready);
 reset();start();frame("156+123+516+231",110,120);tick=130;App_Stage1Service();App_CombinedTestStep();
 ck(g_stage1_motion_skipped && g_combined_test.state==3 && !load_calls && !g_combined_test.completed);
 App_Stage1Service();App_Stage1Display(1);App_Stage1Service();ck(g_stage2_ready);
 /* Actual route function: no next segment after en-route QR, no fabricated completed distance. */
 reset();g_combined_test.state=2;g_combined_test.completed=1;g_combined_test.index=0;g_stage1_stop_requested=1;
 s_motion_session_open=1;fixture_power.active=1;g_vm_test_request=1;tick=2000;App_CombinedTestStep();
 ck(!load_calls && g_combined_test.index==0 && finish_calls==1 && g_combined_test.state==2);
 fixture_power.active=0;App_CombinedTestStep();ck(g_combined_test.state==3 && g_combined_test.completed==1 && !load_calls);
 reset();g_combined_test.state=1;g_combined_test.index=0;g_stage1_stop_requested=1;s_motion_session_open=1;
 fixture_backend.state=CHASSIS_AUTO_DONE;fixture_backend.result=1;fixture_backend.rotation_stop_mask=15;fixture_backend.distance_reached=0;
 tick=1000;App_CombinedTestStep();ck(g_combined_test.state==3 && !g_combined_test.completed && !load_calls && finish_calls==1);
 reset();g_combined_test.state=1;g_stage1_stop_requested=1;g_combined_test.request=2;App_CombinedTestStep();ck(g_combined_test.state==5);
 reset();g_combined_test.state=1;g_stage1_stop_requested=1;fixture_phase=HR_FAULT;App_CombinedTestStep();ck(g_combined_test.state==4);
 /* Tick wrap: a new frame across rollover is accepted, pre-run bytes are not. */
 reset();tick=0xfffffff0U;start();tick=20;frame("156+123+516+231",2,10);App_Stage1Service();ck(g_stage1_scan_valid);
#endif
}
int main(void){run_tests();printf("checks=%u failures=%u\n",checks,failures);for(uint32_t i=0;i<failures && i<16;i++)printf("failed check %u\n",failure_checks[i]);return failures?1:0;}
'''
(OUT/'early-scan-unit.c').write_text(unit,encoding='utf-8')
reports=[]
for offline in [0,1]:
    exe=OUT/f'early-scan-{offline}.exe'
    cmd=['C:/mingw/mingw64/bin/gcc.exe','-std=c99',f'-DAPP_MOTOR_OFFLINE_ONLY={offline}','-DVEHICLE_SCAN_BYPASS=0',
         '-I'+str(FW/'USER'),str(OUT/'early-scan-unit.c')]
    cmd += [str(FW/'USER'/p) for p in ['field_map.c','app_entry.c','motion_profile.c','chassis_kinematics.c']]
    cmd += ['-lm','-o',str(exe)]
    p=subprocess.run(cmd,capture_output=True,text=True,errors='replace')
    (OUT/f'host-build-{offline}.log').write_text(p.stdout+p.stderr,encoding='utf-8')
    assert p.returncode==0,p.stderr[-6000:]
    p=subprocess.run([str(exe)],capture_output=True,text=True)
    print(offline,p.stdout,flush=True);assert p.returncode==0,p.stdout
    reports.append(dict(offline=offline,output=p.stdout,passed=True))
(OUT/'host-check.json').write_text(json.dumps(dict(cases=reports,actual_c=True,
    peripherals_mocked=True,hardware_accessed=False),indent=2),encoding='utf-8')

# Exercise the real boot PWM selection, including the configurable carrier angle.
pose_unit=r'''
#include <stdint.h>
#include <stdio.h>
#include "startup_pose.h"
extern volatile uint32_t g_startup_gripper_result,g_startup_carrier_result;
static uint32_t init_gripper,init_carrier,writes,last_pulse;
static float gripper_angle;
void Gripper_Init(void){init_gripper++;}
uint8_t jiazhua(float angle){gripper_angle=angle;return 1;}
uint32_t CarrierPwm_Init(void){init_carrier++;return 1;}
uint32_t CarrierPwm_Write(uint32_t pulse){writes++;last_pulse=pulse;return 1;}
int main(void){StartupPose_Init();
#if APP_MOTOR_OFFLINE_ONLY
 if(init_gripper || init_carrier || writes)return 1;
#else
 if(init_gripper!=1 || init_carrier!=1 || gripper_angle!=140.0f || g_startup_gripper_result!=1)return 2;
#if POSE_EXPECT_INVALID
 if(writes || g_startup_carrier_result!=3)return 3;
#elif POSE_EXPECT_PULSE
 if(writes!=1 || last_pulse!=POSE_EXPECT_PULSE || g_startup_carrier_result!=1)return 4;
#else
 if(writes || g_startup_carrier_result)return 5;
#endif
#endif
 puts("boot pose passed");return 0;}
'''
(OUT/'pose-unit.c').write_text(pose_unit,encoding='utf-8')
pose_cases=[]
for label,offline,angle,pulse,invalid in [('unset',0,None,0,0),('90deg',0,'90.0f',1167,0),
    ('270deg',0,'270.0f',2500,0),('out-of-range',0,'271.0f',0,1),('offline',1,'90.0f',0,0)]:
    exe=OUT/('pose-'+label+'.exe')
    cmd=['C:/mingw/mingw64/bin/gcc.exe','-std=c99',f'-DAPP_MOTOR_OFFLINE_ONLY={offline}',
        f'-DPOSE_EXPECT_PULSE={pulse}',f'-DPOSE_EXPECT_INVALID={invalid}',
        '-I'+str(FW/'USER'),'-I'+str(FW.parent/'HARDWARE/GRIPPER'),'-I'+str(FW.parent/'HARDWARE/CARRIER')]
    if angle is not None:cmd+=['-DSTARTUP_CARRIER_ANGLE_DEG='+angle]
    cmd += [str(OUT/'pose-unit.c'),str(FW/'USER/startup_pose.c'),'-o',str(exe)]
    p=subprocess.run(cmd,capture_output=True,text=True,errors='replace');assert p.returncode==0,p.stderr
    p=subprocess.run([str(exe)],capture_output=True,text=True);assert p.returncode==0,(label,p.returncode)
    pose_cases.append(dict(case=label,passed=True));print(label,p.stdout.strip(),flush=True)
(OUT/'pose-check.json').write_text(json.dumps(dict(cases=pose_cases,actual_c=True,
    peripherals_mocked=True,hardware_accessed=False),indent=2),encoding='utf-8')
