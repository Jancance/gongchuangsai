from pathlib import Path
import ctypes,hashlib,json,re,struct,subprocess,sys,xml.etree.ElementTree as ET
R=Path.cwd();E=Path(__file__).resolve().parent;U=R/'3.stm32f407工程/CHASSIS/USER';B=Path('D:/Keil_v5/ARM/arm5_compiler/Bin');T=E/'actual-c';T.mkdir(exist_ok=True)
def short(p):
 b=ctypes.create_unicode_buffer(32768);assert ctypes.windll.kernel32.GetShortPathNameW(str(p),b,len(b));return b.value
prefix=r'''
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_odometry.h"
#include "vehicle_tuning.h"
#include "motion_trial20.h"
#include "app_motion_runtime.h"
#include "app_entry.h"
#include "heading_source.h"
#include "chassis_auto_test.h"
#include "app_can_diagnostics.h"
#include "power.h"
volatile uint32_t g_safety_fault,g_rtos_fault,g_can_ctrl_fault_latched,g_chassis_auto_active,g_wheel_test_active,g_debug_plan_id;
static uint32_t clock_ms,combined_state,completed,route_id,vm,jog,count,critical_depth,inject_read;
static MotionAction actions[8];static AppHeadingSource heading;
static ChassisOdometrySamples backend,diagnostic;static PowerCommandStatus power;
static RouteHeadingReference reference;
static const AppMotionObservation live={.action_count=&count,.actions=actions};
static AppOdometryInputs inputs={&vm,&jog,&combined_state,&completed,&reference,&route_id};
const AppMotionObservation *AppMotion_Live(void){return &live;}
TickType_t xTaskGetTickCount(void){return clock_ms;}
void vPortEnterCritical(void){critical_depth++;}
void vPortExitCritical(void){critical_depth--;}
uint32_t App_HeadingSourceRead(AppHeadingSource *s){*s=heading;return 1;}
void Power_ReadCommandStatus(PowerCommandStatus *s){if(inject_read){AppMapPose m;uint32_t save=clock_ms;inject_read=0;clock_ms=save+100;AppOdometry_ReadMap(&m);clock_ms=save;}*s=power;}
void ChassisAuto_ReadOdometrySamples(ChassisOdometrySamples *s){*s=backend;}
void AppCanDiagnostics_ReadPositionSamples(ChassisOdometrySamples *s){*s=diagnostic;}
uint32_t ChassisAuto_ReadStatus(ChassisAutoStatus *s){memset(s,0,sizeof(*s));s->session=1;return 1;}
static void step_at(uint32_t ms){clock_ms=ms;AppOdometry_Step(&inputs);}
static void yaw_at(uint32_t ms,float y){heading.healthy=1;heading.valid_mask=3;heading.yaw_deg=y;heading.angle_ms=heading.rate_ms=ms;}
static void wheel(ChassisOdometrySamples *s,uint32_t i,uint32_t ms,float mm){
 int32_t raw=(int32_t)(mm*65536.0f/(3.14159265358979323846f*77.0f));
 if(g_chassis_forward_direction[i])raw=-raw;
 s->mask|=1UL<<i;s->ms[i]=ms;s->sign[i]=raw<0;s->raw[i]=(uint32_t)(raw<0?-raw:raw);
}
static void frame(uint32_t ms,float forward,float left,float yaw){uint32_t i;
 for(i=0;i<4;i++)wheel(&backend,i,ms,forward+((i==0||i==3)?left:-left));yaw_at(ms,yaw);step_at(ms);
}
static void reset_at(uint32_t ms){
 inject_read=0;clock_ms=ms;combined_state=completed=route_id=vm=jog=0;count=1;
 g_safety_fault=g_rtos_fault=g_can_ctrl_fault_latched=g_chassis_auto_active=g_wheel_test_active=0;
 memset(&backend,0,sizeof(backend));memset(&diagnostic,0,sizeof(diagnostic));memset(&heading,0,sizeof(heading));
 memset(&reference,0,sizeof(reference));memset(actions,0,sizeof(actions));power.active=1;power.enable_count=1;
 AppOdometry_Init();AppOdometry_ResetRecords();
}
'''
tests=r'''
volatile uint32_t review_assertions;
#define CK(x) do{review_assertions++;if(!(x))return __LINE__;}while(0)
#define CLOSE(a,b,e) CK(fabsf((a)-(b))<(e))
__attribute__((section("i.test_entry"))) uint32_t test_entry(void){
 AppMapPose m,old;uint32_t i,k,run;float origin;
#if APP_CONTINUOUS_MAP_ENABLE
 for(k=1;k<=2;k++){
  reset_at(100);AppOdometry_ReadMap(&m);CK(m.state==APP_MAP_OFF&&!m.pose.valid);
  AppOdometry_RequestMapRun(1,k);AppOdometry_ReadMap(&m);CK(m.run_id==1&&m.state==APP_MAP_WAIT_BASELINE&&!m.pose.valid);
  frame(100,0,0,17);AppOdometry_ReadMap(&m);CK(m.based&&m.pose.valid&&m.state==APP_MAP_TRACKING);origin=k==1?2250:150;
  CLOSE(m.pose.x_mm,origin,.001f);CLOSE(m.pose.y_mm,150,.001f);CLOSE(m.pose.heading_deg,90,.001f);CK(AppOdometry_MapTrackingRun()==1);
  CK(m.map_version==VEHICLE_MAP_VERSION&&m.source==APP_MAP_RAW_WHEEL_HWT&&!m.nav_applied&&!m.calibration_scope);
  old=m;g_debug_plan_id=8;actions[0].distance_mm=2100;frame(120,0,0,17);AppOdometry_ReadMap(&m);CLOSE(m.pose.x_mm,old.pose.x_mm,.001f);CLOSE(m.pose.y_mm,old.pose.y_mm,.001f);
  for(i=1;i<=20;i++)frame(120+i*20,i*10.0f,0,17);
  AppOdometry_ReadMap(&m);CK(m.pose.valid);CLOSE(m.pose.y_mm,150+200*CHASSIS_FORWARD_DISTANCE_SCALE,.02f);CLOSE(m.pose.x_mm,origin,.02f);
  old=m;
  /* Local reset/manual SetPose, plan and backend-session cache reset leave map unchanged. */
  AppOdometry_ResetRecords();g_odometry_pose_x_mm=123;g_odometry_pose_y_mm=456;g_odometry_pose_heading_deg=-30;g_odometry_pose_request=1;power.active=0;
  /* Do not tick powered-off: local SetPose is exercised independently below. */
  power.active=1;g_odometry_pose_request=0;route_id=4;g_debug_plan_id=19;backend.mask=0;
  yaw_at(540,17);step_at(540);AppOdometry_ReadMap(&m);CK(m.pose.valid);CLOSE(m.pose.y_mm,old.pose.y_mm,.001f);
  for(i=1;i<=20;i++)frame(540+i*20,200,i*10.0f,17);
  AppOdometry_ReadMap(&m);CK(m.pose.valid);CLOSE(m.pose.x_mm,origin-200,.02f);CLOSE(m.pose.y_mm,old.pose.y_mm,.02f);
  /* Duplicate sources do not integrate twice; sample age is independent of publish time. */
  old=m;step_at(950);AppOdometry_ReadMap(&m);CK(m.pose.sample_ms==940&&m.pose.age_ms==10);CLOSE(m.pose.x_mm,old.pose.x_mm,.001f);
  /* A stale wheel freezes permanently for the same run. */
  backend.mask=0;yaw_at(1340,17);step_at(1340);AppOdometry_ReadMap(&m);CK(m.state==APP_MAP_GAP&&!m.pose.valid&&m.pose.continuity_lost);old=m;
  AppOdometry_RequestMapRun(1,3-k);AppOdometry_RequestMapRun(0,k);frame(1360,210,200,17);AppOdometry_ReadMap(&m);CK(m.run_id==1&&m.start_id==k&&m.state==APP_MAP_GAP);CLOSE(m.pose.x_mm,old.pose.x_mm,.001f);
  AppOdometry_RequestMapRun(2,0);AppOdometry_ReadMap(&m);CK(m.run_id==2&&m.state==APP_MAP_NO_START&&!m.pose.valid&&!m.based);
  frame(1380,210,200,17);AppOdometry_ReadMap(&m);CK(m.run_id==2&&m.state==APP_MAP_NO_START&&!m.pose.valid);
  AppOdometry_RequestMapRun(3,k);AppOdometry_RequestMapRun(2,0);AppOdometry_RequestMapRun(3,0);AppOdometry_ReadMap(&m);CK(m.run_id==3&&m.start_id==k&&!m.pose.valid);
  g_chassis_auto_active=1;frame(1400,210,200,17);AppOdometry_ReadMap(&m);CK(m.state==APP_MAP_GAP&&(m.pose.reasons&APP_MAP_NO_BASELINE));
 }
 /* True asynchronous histories: mask15 at120 is not an interpolated baseline; ready at210. */
 reset_at(0);AppOdometry_RequestMapRun(1,1);
 for(i=1;i<=24;i++){
  uint32_t ms=i*10;
  if(i%3==0)wheel(&diagnostic,(i/3-1)%4,ms,0);
  yaw_at(ms,0);step_at(ms);AppOdometry_ReadMap(&m);
  if(i==12)CK(diagnostic.mask==15&&!m.based&&!m.pose.valid);
  if(i<21)CK(!m.pose.valid);else CK(m.pose.valid&&m.based&&m.pose.sample_ms<=ms);
 }
 /* Cache selection uses whichever source is newer, then ignores old cache replays. */
 reset_at(100);AppOdometry_RequestMapRun(1,2);frame(100,0,0,0);
 for(i=0;i<4;i++)wheel(&diagnostic,i,120,10);yaw_at(120,0);step_at(120);AppOdometry_ReadMap(&m);CK(m.pose.valid);CLOSE(m.pose.y_mm,150+10*CHASSIS_FORWARD_DISTANCE_SCALE,.02f);
 diagnostic.mask=0;step_at(130);AppOdometry_ReadMap(&m);CK(m.pose.valid&&m.pose.sample_ms==120);
 /* Same-time equal values duplicate safely; conflicting, future and malformed values gap. */
 for(k=0;k<4;k++){
  reset_at(100);AppOdometry_RequestMapRun(1,1);frame(100,0,0,0);diagnostic=backend;
  if(k==1)diagnostic.raw[0]++;
  if(k==2)diagnostic.ms[0]=121;
  if(k==3)diagnostic.sign[0]=2;
  yaw_at(120,0);step_at(120);AppOdometry_ReadMap(&m);CK(k==0?m.pose.valid:(m.state==APP_MAP_GAP&&!m.pose.valid));
 }
 /* Heading stale/invalid, known power epoch change, RTOS fault stay invalid. */
 for(k=0;k<5;k++){
  reset_at(100);AppOdometry_RequestMapRun(1,1);frame(100,0,0,0);
  if(k==0)clock_ms=200;
  if(k==1){heading.valid_mask=0;clock_ms=120;}
  if(k==2){power.active=0;clock_ms=120;}
  if(k==3){power.enable_count++;clock_ms=120;}
  if(k==4){g_rtos_fault=1;clock_ms=120;}
  step_at(clock_ms);AppOdometry_ReadMap(&m);CK(m.state==APP_MAP_GAP&&!m.pose.valid);
  power.active=1;g_rtos_fault=0;frame(220,0,0,0);AppOdometry_ReadMap(&m);CK(m.state==APP_MAP_GAP&&!m.pose.valid);
 }
 /* New run rejects prebegin caches; local pose request cannot clear a map gap. */
 reset_at(100);AppOdometry_RequestMapRun(1,1);frame(100,0,0,0);clock_ms=120;AppOdometry_RequestMapRun(2,2);step_at(120);AppOdometry_ReadMap(&m);CK(!m.based&&!m.pose.valid);
 frame(140,0,0,0);AppOdometry_ReadMap(&m);CK(m.pose.valid&&m.run_id==2);power.active=0;g_odometry_pose_request=1;g_odometry_pose_x_mm=123;g_odometry_pose_y_mm=456;g_odometry_pose_heading_deg=-30;yaw_at(160,0);step_at(160);AppOdometry_ReadMap(&m);CK(g_odometry_pose_result==1&&m.state==APP_MAP_GAP&&!m.pose.valid);CLOSE(g_chassis_odometry.x_mm,123,.001f);
 /* u32 time wrap keeps source age and displacement; run serial comparison uses half-range. */
 reset_at(0xfffffff0U);AppOdometry_RequestMapRun(0xfffffffeU,1);frame(0xfffffff0U,0,0,179);frame(4,1,0,-179);AppOdometry_ReadMap(&m);CK(m.pose.valid&&m.pose.sample_ms==4&&m.pose.age_ms==0);CLOSE(m.pose.heading_deg,92,.001f);
 AppOdometry_RequestMapRun(0xffffffffU,2);AppOdometry_RequestMapRun(1,1);AppOdometry_RequestMapRun(0xfffffffeU,2);AppOdometry_ReadMap(&m);CK(m.run_id==1&&m.start_id==1&&!m.pose.valid);
 /* Read-only getter also refuses aged data if the CAN owner has not run. */
 reset_at(100);AppOdometry_RequestMapRun(1,1);frame(100,0,0,0);clock_ms=200;AppOdometry_ReadMap(&m);CK(m.state==APP_MAP_GAP&&!m.pose.valid&&m.pose.age_ms==100);CK(!AppOdometry_MapTrackingRun());
 frame(210,0,0,0);AppOdometry_ReadMap(&m);CK(m.state==APP_MAP_GAP&&!m.pose.valid);
 /* Getter can preempt after the CAN owner captured its mailbox. */
 reset_at(100);AppOdometry_RequestMapRun(1,1);frame(100,0,0,0);inject_read=1;frame(210,1,0,0);AppOdometry_ReadMap(&m);CK(m.state==APP_MAP_GAP&&!m.pose.valid&&m.pose.sample_ms==100);CLOSE(m.pose.y_mm,150,.001f);
 CK(critical_depth==0);
#else
 reset_at(100);AppOdometry_RequestMapRun(1,1);frame(100,0,0,0);frame(120,10,0,0);AppOdometry_ReadMap(&m);CK(m.state==APP_MAP_OFF&&!m.pose.valid&&!m.run_id);CK(!AppOdometry_MapTrackingRun());CK(g_chassis_odometry.valid);CLOSE(g_chassis_odometry.x_mm,10*CHASSIS_FORWARD_DISTANCE_SCALE,.02f);
#endif
 return 0;
}
'''
prefix += '\n#include "motion_control.h"\nstatic uint32_t motion_state;\nuint32_t MotionGetState(void){return motion_state;}\n'
tests=tests.replace(" CK(critical_depth==0);","\n#if APP_MAP_NAV_RESIDUAL_ENABLE\n {\n  uint32_t d,j;float f,l,q,cf,cl,cx,cy;\n  static const float cross[4]={APP_FORWARD_NAV_RESIDUAL_COEFFICIENT,APP_LEFT_NAV_RESIDUAL_COEFFICIENT,APP_RIGHT_NAV_RESIDUAL_COEFFICIENT,APP_BACKWARD_NAV_RESIDUAL_COEFFICIENT};\n  static const float main[4]={APP_FORWARD_MAIN_NAV_COEFFICIENT,APP_LEFT_MAIN_NAV_COEFFICIENT,APP_RIGHT_MAIN_NAV_COEFFICIENT,APP_BACKWARD_MAIN_NAV_COEFFICIENT};\n  for(d=0;d<4;d++){\n   reset_at(100);motion_state=MOTION_IDLE;AppOdometry_RequestMapRun(1,1);frame(100,0,0,17);\n   clock_ms=110;motion_state=MOTION_BUSY;g_debug_plan_id=0;AppOdometry_MapActionAccepted(d,10,300,150);\n   /* Old common sample before acceptance cannot establish the new anchor. */\n   step_at(110);AppOdometry_ReadMap(&m);CK(!m.nav_applied&&m.action_epoch);\n   frame(120,0,0,17);AppOdometry_ReadMap(&m);CK(!m.nav_applied);\n   f=d==0?10:(d==3?-10:0);l=d==1?10:(d==2?-10:0);\n   frame(140,f,l,17);AppOdometry_ReadMap(&m);CK(m.pose.valid&&m.nav_applied);\n   q=(d==0||d==3)?fabsf(m.raw_y_mm-150):fabsf(m.raw_x_mm-2250);\n   cf=.5f*q*((d==1||d==2)?cross[d]:(d==0?main[d]:-main[d]));\n   cl=.5f*q*((d==0||d==3)?cross[d]:(d==1?main[d]:-main[d]));\n   CLOSE(m.correction_x_mm,-cl,.001f);CLOSE(m.correction_y_mm,cf,.001f);\n   CLOSE(m.pose.x_mm,m.raw_x_mm+m.correction_x_mm,.001f);CLOSE(m.pose.y_mm,m.raw_y_mm+m.correction_y_mm,.001f);\n   CK(m.calibration_scope==APP_MAP_CAL_EXTRAPOLATED&&m.source==APP_MAP_CALIBRATED_WHEEL_HWT);\n   old=m;for(j=0;j<20;j++){step_at(140);AppOdometry_ReadMap(&m);CLOSE(m.pose.x_mm,old.pose.x_mm,.0001f);CLOSE(m.pose.y_mm,old.pose.y_mm,.0001f);}\n   /* Net reverse movement reduces this segment's overlay, no abs-distance accumulation. */\n   frame(160,f*.5f,l*.5f,17);AppOdometry_ReadMap(&m);CLOSE(m.correction_x_mm,old.correction_x_mm*.5f,.001f);CLOSE(m.correction_y_mm,old.correction_y_mm*.5f,.001f);\n   cx=m.correction_x_mm;cy=m.correction_y_mm;motion_state=MOTION_DONE;\n   frame(180,f*.8f,l*.8f,17);AppOdometry_ReadMap(&m);CLOSE(m.correction_x_mm,cx,.0001f);CLOSE(m.correction_y_mm,cy,.0001f);CK(m.calibration_flags&APP_MAP_CAL_TAIL);\n   /* Unsupported rotation preserves the historical correction and actual heading. */\n   clock_ms=190;motion_state=MOTION_BUSY;AppOdometry_MapActionAccepted(4,0,0,0);\n   for(j=1;j<=45;j++)frame(180+j*20,f*.8f,l*.8f,17+j*2.0f);AppOdometry_ReadMap(&m);CLOSE(m.pose.heading_deg,180,.001f);CLOSE(m.correction_x_mm,cx,.0001f);CLOSE(m.correction_y_mm,cy,.0001f);CK(m.calibration_flags&APP_MAP_CAL_UNSUPPORTED);\n   /* A new forward segment at actual 180deg adds only its own rotated correction. */\n   clock_ms=1090;AppOdometry_MapActionAccepted(0,10,900,150);frame(1100,f*.8f,l*.8f,107);AppOdometry_ReadMap(&old);\n   frame(1120,f*.8f+10,l*.8f,107);AppOdometry_ReadMap(&m);q=old.raw_x_mm-m.raw_x_mm;\n   CLOSE(m.correction_x_mm,cx-.5f*q*main[0],.001f);CLOSE(m.correction_y_mm,cy-.5f*q*cross[0],.001f);\n   /* Local reset never removes global history. */\n   old=m;AppOdometry_ResetRecords();AppOdometry_ReadMap(&m);CLOSE(m.pose.x_mm,old.pose.x_mm,.0001f);\n   /* Getter preemption must freeze RAW and corrected at one published sample. */\n   inject_read=1;frame(1230,f*.8f+11,l*.8f,107);AppOdometry_ReadMap(&m);\n   CK(m.state==APP_MAP_GAP&&!m.pose.valid&&m.pose.sample_ms==old.pose.sample_ms);\n   CLOSE(m.pose.x_mm,old.pose.x_mm,.0001f);CLOSE(m.raw_x_mm,old.raw_x_mm,.0001f);CLOSE(m.correction_x_mm,old.correction_x_mm,.0001f);\n   /* A true new run resets correction and ignores the old accepted action. */\n   clock_ms=1240;AppOdometry_RequestMapRun(2,2);frame(1240,f*.8f+11,l*.8f,107);AppOdometry_ReadMap(&m);\n   CK(m.pose.valid&&!m.nav_applied&&!m.calibration_scope);CLOSE(m.correction_x_mm,0,.0001f);CLOSE(m.pose.x_mm,150,.001f);\n  }\n  /* Reference/extrapolated classification is narrower than execution legality. */\n  reset_at(100);motion_state=MOTION_IDLE;AppOdometry_RequestMapRun(1,1);frame(100,0,0,0);\n  clock_ms=110;motion_state=MOTION_BUSY;g_debug_plan_id=8;AppOdometry_MapActionAccepted(0,100,2100,0);frame(120,0,0,0);frame(140,20,0,0);AppOdometry_ReadMap(&m);\n#if APP_BASELINE40_ACTIVE && !APP_CONTROL_DISTANCE_RATIO_ENABLE\n  CK(m.calibration_scope==APP_MAP_CAL_REFERENCE);\n#else\n  CK(m.calibration_scope==APP_MAP_CAL_EXTRAPOLATED);\n#endif\n  clock_ms=150;g_debug_plan_id=15;AppOdometry_MapActionAccepted(0,100,300,0);frame(160,20,0,0);frame(180,40,0,0);AppOdometry_ReadMap(&m);CK(m.calibration_scope&APP_MAP_CAL_EXTRAPOLATED);\n  /* First source-aligned heading is 87, not a nominal command or initial map90. */\n  reset_at(100);motion_state=MOTION_IDLE;AppOdometry_RequestMapRun(1,1);frame(100,0,0,17);\n  clock_ms=110;motion_state=MOTION_BUSY;AppOdometry_MapActionAccepted(0,10,300,150);frame(120,0,0,14);frame(140,20,0,14);AppOdometry_ReadMap(&m);\n  q=sqrtf((m.raw_x_mm-2250)*(m.raw_x_mm-2250)+(m.raw_y_mm-150)*(m.raw_y_mm-150));\n  cx=.5f*q*(cosf(87*.01745329252f)*main[0]-sinf(87*.01745329252f)*cross[0]);\n  cy=.5f*q*(sinf(87*.01745329252f)*main[0]+cosf(87*.01745329252f)*cross[0]);\n  CLOSE(m.correction_x_mm,cx,.001f);CLOSE(m.correction_y_mm,cy,.001f);CLOSE(m.pose.heading_deg,87,.001f);\n }\n#endif\n#if APP_MAP_NAV_RESIDUAL_ENABLE\n {\n  float saved;\n  reset_at(100);motion_state=MOTION_IDLE;AppOdometry_RequestMapRun(1,1);frame(100,0,0,0);\n  clock_ms=110;motion_state=MOTION_BUSY;\n  AppOdometry_MapActionAccepted(0,10,300,150);AppOdometry_MapActionAccepted(1,10,300,150);AppOdometry_MapActionAccepted(2,10,300,150);\n  frame(120,0,0,0);AppOdometry_ReadMap(&m);CK(m.action_epoch==3&&!m.nav_applied&&m.calibration_direction==2);\n  frame(140,0,-10,0);AppOdometry_ReadMap(&m);CK(m.nav_applied&&m.calibration_scope==APP_MAP_CAL_EXTRAPOLATED);\n  saved=m.correction_y_mm;frame(160,0,-10,0);AppOdometry_ReadMap(&m);CLOSE(m.correction_y_mm,saved,.0001f);\n  AppOdometry_RequestMapRun(2,2);frame(180,0,-10,0);frame(200,0,-20,0);AppOdometry_ReadMap(&m);\n  CK(m.pose.valid&&!m.nav_applied&&!m.calibration_scope&&!m.action_epoch);CLOSE(m.correction_x_mm,0,.0001f);\n }\n#endif\n /* Actual production heading continuation helper, all four translation directions. */\n {\n  RouteHeadingReference r={0};RouteHeadingContinuation c={0};uint32_t a,d;float base,target,yaw;\n  for(a=0;a<12;a++){\n   base=HeadingController_WrapDeg(-179.0f+a*33.0f);c.valid=0;\n   CK(RouteHeading_BeginContinued(&r,&c,1,base));CLOSE(r.origin_deg,base,.001f);\n   target=HeadingController_WrapDeg(base-90);RouteHeading_Complete(&c,1,target);yaw=HeadingController_WrapDeg(target+2);\n   for(d=0;d<4;d++){\n    CK(RouteHeading_BeginContinued(&r,&c,1,yaw));CLOSE(r.origin_deg,target,.001f);\n    CK(RouteHeading_Target(&r,-90,&base));CLOSE(HeadingController_ErrorDeg(base,HeadingController_WrapDeg(target-90)),0,.001f);\n   }\n   CK(RouteHeading_BeginContinued(&r,&c,2,yaw));CLOSE(r.origin_deg,yaw,.001f);\n  }\n }\n"+"\n CK(critical_depth==0);")
entry=(U/'app_entry.c').read_text('gbk');a=entry.index('uint32_t AppDebugPlan40Allowed');b=entry.index('uint32_t AppDebugPlan_Load',a)
prefix+='\n'+entry[a:b]+'\n'
(T/'unit.c').write_bytes((prefix+tests).encode('ascii'))
control=ET.parse(U.parents[1]/'gongxun-f407.uvprojx').getroot().find('./Targets/Target/TargetOption/TargetArmAds/Cads/VariousControls')
opts=['--cpu','Cortex-M3','--c99','--gnu','--split_sections','-O1','--diag_suppress=177,550','-DVEHICLE_INTEGRATED_ENABLE=0','-DVEHICLE_STAGE1_ENABLE=0','-DAPP_VM_CALIBRATION_ONLY_ENABLE=0','-DAPP_VM_ONLY_ENABLE=0','-DAPP_WHEEL_ENABLE_HOLD_ENABLE=0']
opts+=['-D'+d for d in control.findtext('Define').split(',')]
for inc in control.findtext('IncludePath').split(';'):opts+=['-I',short((U.parents[1]/inc).resolve())]
def run_cmd(cmd,log):
 cp=subprocess.run(cmd,cwd=T,capture_output=True,timeout=90);(T/log).write_bytes(cp.stdout+cp.stderr);assert cp.returncode==0,(cp.stdout+cp.stderr).decode('gbk',errors='replace')
sys.path.insert(0,str(R/'工创赛/tmp/position_alignment_deps'));import unicorn as uc
from unicorn import arm_const as reg
report={'hardware_accessed':False,'software_only':True,'actual_odometry_integrator':True,'stubbed':'RTOS critical/tick, heading/cache/power providers and MotionGetState; real AppOdometry_Step and real ChassisOdometry_* and kinematics','cases':{},'source_sha256':{f:hashlib.sha256((U/f).read_bytes()).hexdigest() for f in ['app_odometry.c','app_odometry.h','app_config.h','chassis_odometry.c','chassis_kinematics.c','route_heading_reference.h','app_motion_runtime.c']}}
for mode,flag,nav,cal,baseline,ratio in [('map_cal',1,0,1,0,0),('map_cal_reference',1,0,1,1,0),('map_cal_legacy',1,1,1,1,1),('map_raw',1,0,0,1,0),('map_off',0,0,0,1,0)]:
 objects=[]
 for p in [T/'unit.c',U/'app_odometry.c',U/'chassis_odometry.c',U/'chassis_kinematics.c',U/'heading_controller.c']:
  obj=mode+'_'+p.stem+'.o';objects.append(obj);run_cmd([str(B/'armcc.exe'),*opts,'-DAPP_BASELINE40_ENABLE='+str(baseline),'-DAPP_MAP_NAV_RESIDUAL_ENABLE='+str(cal),'-DAPP_CONTROL_DISTANCE_RATIO_ENABLE='+str(ratio),'-DAPP_CONTINUOUS_MAP_ENABLE='+str(flag),'-DAPP_LEFT_NAV_RESIDUAL_ENABLE='+str(nav),'-DAPP_DIRECTION_NAV_RESIDUAL_ENABLE='+str(nav),'-DAPP_MAIN_NAV_RESIDUAL_ENABLE='+str(nav),'-c',short(p),'-o',obj],obj+'.log')
 (T/'unit.sct').write_text('LR_ROM 0x08000000 0x100000 { ER_ROM 0x08000000 0x100000 { '+objects[0]+' (i.test_entry, +First) *(+RO) } RW_RAM 0x20000000 0x50000 { *(+RW,+ZI) } }\n')
 run_cmd([str(B/'armlink.exe'),'--entry','test_entry','--scatter','unit.sct',*objects,'--library_type=microlib','-o',mode+'.axf'],mode+'_link.log')
 data=(T/(mode+'.axf')).read_bytes();elf=struct.unpack_from('<16sHHIIIIIHHHHHH',data)
 machine=uc.Uc(uc.UC_ARCH_ARM,uc.UC_MODE_THUMB|uc.UC_MODE_MCLASS)
 for addr,size in [(0x08000000,0x100000),(0x20000000,0x50000),(0xE0000000,0x100000)]:machine.mem_map(addr,size)
 for n in range(elf[12]):
  _,kind,flags,addr,offset,size,*_=struct.unpack_from('<IIIIIIIIII',data,elf[6]+n*elf[11])
  if flags&2 and size:machine.mem_write(addr,b'\0'*size if kind==8 else data[offset:offset+size])
 stop=0x080ff000;machine.reg_write(reg.UC_ARM_REG_SP,0x2004fff0);machine.reg_write(reg.UC_ARM_REG_LR,stop|1)
 machine.emu_start(elf[4]|1,stop,timeout=60000000,count=200000000);assert machine.reg_read(reg.UC_ARM_REG_PC)==stop
 result=machine.reg_read(reg.UC_ARM_REG_R0)
 symbols=subprocess.check_output([str(B/'fromelf.exe'),'--text','-s',mode+'.axf'],cwd=T).decode(errors='replace')
 address=int(re.search(r'\breview_assertions\s+(0x[0-9a-fA-F]+)',symbols)[1],16);count=struct.unpack('<I',machine.mem_read(address,4))[0]
 report['cases'][mode]={'assertions':count,'failed_line':result,'status':'PASS' if result==0 else 'FAIL'}
 (E/'actual_c_checks.json').write_text(json.dumps(report,indent=2),'utf8')
 if result:print((prefix+tests).splitlines()[result-1]);raise SystemExit(result)
print(json.dumps(report['cases']))
