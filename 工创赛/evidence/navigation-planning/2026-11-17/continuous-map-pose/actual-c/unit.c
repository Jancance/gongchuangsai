
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
void Power_ReadCommandStatus(PowerCommandStatus *s){if(inject_read){AppMapPose m;uint32_t save=clock_ms;inject_read=0;clock_ms=200;AppOdometry_ReadMap(&m);clock_ms=save;}*s=power;}
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

uint32_t AppDebugPlan40Allowed(uint32_t id,uint32_t direction,uint32_t rpm,float mm)
{
    return MotionTrial40Allowed(direction,rpm,mm) &&
        ((((id==DEBUG_PLAN_FORWARD210_40 && direction==0U) ||
           (id==DEBUG_PLAN_LEFT210_40 && direction==1U) ||
           (id==DEBUG_PLAN_RIGHT210_40 && direction==2U) ||
           (id==DEBUG_PLAN_BACKWARD210_40 && direction==3U)) && mm==APP_BASELINE40_MM) ||
         (id==DEBUG_PLAN_RIGHT150_40 && direction==2U && mm==1500.0f) ||
         (id==DEBUG_PLAN_RIGHT30_40 && direction==2U && mm==300.0f) ||
         (id==DEBUG_PLAN_FORWARD150_40 && direction==0U && mm==1500.0f) ||
         (id==DEBUG_PLAN_FORWARD30_40 && direction==0U && mm==300.0f) ||
         (id==DEBUG_PLAN_BACKWARD150_40 && direction==3U && mm==1500.0f) ||
         (id==DEBUG_PLAN_BACKWARD30_40 && direction==3U && mm==300.0f) ||
         (id==DEBUG_PLAN_LEFT150_40 && direction==1U && mm==1500.0f) ||
         (id==DEBUG_PLAN_LEFT30_40 && direction==1U && mm==300.0f));
}


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
  CLOSE(m.pose.x_mm,origin,.001f);CLOSE(m.pose.y_mm,150,.001f);CLOSE(m.pose.heading_deg,90,.001f);
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
 reset_at(100);AppOdometry_RequestMapRun(1,1);frame(100,0,0,0);clock_ms=200;AppOdometry_ReadMap(&m);CK(m.state==APP_MAP_GAP&&!m.pose.valid&&m.pose.age_ms==100);
 frame(210,0,0,0);AppOdometry_ReadMap(&m);CK(m.state==APP_MAP_GAP&&!m.pose.valid);
 /* Getter can preempt after the CAN owner captured its mailbox. */
 reset_at(100);AppOdometry_RequestMapRun(1,1);frame(100,0,0,0);inject_read=1;frame(210,1,0,0);AppOdometry_ReadMap(&m);CK(m.state==APP_MAP_GAP&&!m.pose.valid&&m.pose.sample_ms==100);CLOSE(m.pose.y_mm,150,.001f);
 CK(critical_depth==0);
#else
 reset_at(100);AppOdometry_RequestMapRun(1,1);frame(100,0,0,0);frame(120,10,0,0);AppOdometry_ReadMap(&m);CK(m.state==APP_MAP_OFF&&!m.pose.valid&&!m.run_id);CK(g_chassis_odometry.valid);CLOSE(g_chassis_odometry.x_mm,10*CHASSIS_FORWARD_DISTANCE_SCALE,.02f);
#endif
 return 0;
}
