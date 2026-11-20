
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_odometry.h"
#include "app_motion_runtime.h"
#include "app_entry.h"
#include "app_icm_observer.h"
#include "heading_source.h"
#include "chassis_auto_test.h"
#include "power.h"
volatile uint32_t g_safety_fault,g_rtos_fault,g_can_ctrl_fault_latched,g_chassis_auto_active,g_wheel_test_active,g_debug_plan_id;
static uint32_t clock_ms,combined_state,completed,route_id,vm,jog,count,allow_pose=1,manual_probe,probe_failed,probe_count,critical_depth;
static MotionAction actions[8];static ChassisOdometryStatus desired;static AppHeadingSource heading;
static RouteHeadingReference reference;
static const AppMotionObservation live={.action_count=&count,.actions=actions};
static AppOdometryInputs inputs={&vm,&jog,&combined_state,&completed,&reference,&route_id};
const AppMotionObservation *AppMotion_Live(void){return &live;}
TickType_t xTaskGetTickCount(void){return clock_ms;}
void vPortEnterCritical(void){critical_depth++;}
void vPortExitCritical(void){critical_depth--;}
uint32_t App_HeadingSourceRead(AppHeadingSource *s){*s=heading;return 1;}
void Power_ReadCommandStatus(PowerCommandStatus *s){memset(s,0,sizeof(*s));}
void ChassisAuto_ReadOdometrySamples(ChassisOdometrySamples *s){memset(s,0,sizeof(*s));s->mask=15;}
uint32_t ChassisAuto_ReadStatus(ChassisAutoStatus *s){memset(s,0,sizeof(*s));s->session=1;return 1;}
void ChassisOdometry_Init(ChassisOdometry *c){memset(c,0,sizeof(*c));}
uint32_t ChassisOdometry_SetPose(ChassisOdometry *c,float x,float y,float h,float raw){
 if(manual_probe){AppOdometryBaseline40 a,b;AppOdometry_ReadBaseline40(&a);AppOdometry_ReadNavigation40(&b);probe_count++;if(a.anchor_valid||b.anchor_valid)probe_failed=1;}
 if(!allow_pose)return 0;c->status.x_mm=x;c->status.y_mm=y;c->status.heading_deg=h;c->heading_offset=h-raw;return 1;
}
void ChassisOdometry_Wheel(ChassisOdometry *c,uint32_t i,uint32_t sign,uint32_t raw,uint32_t ms){(void)c;(void)i;(void)sign;(void)raw;(void)ms;}
void ChassisOdometry_Yaw(ChassisOdometry *c,float yaw,uint32_t ms){(void)c;(void)yaw;(void)ms;}
void ChassisOdometry_Invalidate(ChassisOdometry *c){(void)c;}
void ChassisOdometry_Update(ChassisOdometry *c,uint32_t now){(void)now;c->status=desired;}
static void tick(void){clock_ms+=5;heading.angle_ms=heading.rate_ms=clock_ms;AppOdometry_Step(&inputs);}
static void init_case(uint32_t id,uint32_t n,uint32_t kind,uint32_t rpm,float mm){
 AppOdometry_Init();AppOdometry_ResetRecords();g_debug_plan_id=id;count=n;memset(actions,0,sizeof(actions));actions[0].kind=kind;actions[0].base_rpm=rpm;actions[0].distance_mm=mm;
 vm=jog=completed=route_id=manual_probe=probe_failed=probe_count=0;combined_state=1;allow_pose=1;reference.valid=1;reference.origin_deg=10;
 memset(&heading,0,sizeof(heading));heading.healthy=1;heading.valid_mask=3;heading.yaw_deg=10;
 memset(&desired,0,sizeof(desired));desired.x_mm=desired.y_mm=2100;desired.heading_deg=-89.8f;desired.valid=1;desired.sample_ms=clock_ms+5;tick();combined_state=2;
}
static void move_to(float F,float L){desired.x_mm=2100+L;desired.y_mm=2100-F;desired.sample_ms=clock_ms+5;desired.updates++;tick();}

volatile uint32_t review_assertions;
#define CK(x) do{review_assertions++;if(!(x))return __LINE__;}while(0)
static uint32_t verify(uint32_t scope){
 AppOdometryBaseline40 raw,nav,meta;ChassisOdometryStatus g;float angle,c,s,L,correction;
 AppOdometry_ReadBaseline40(&raw);AppOdometry_ReadNavigation40(&nav);g=g_chassis_odometry;
 CK(sizeof(raw)==64);CK(!memcmp(&raw.pose,&desired,sizeof(desired)));CK(!memcmp(&g,&nav.pose,sizeof(g)));
 meta=nav;meta.pose.x_mm=raw.pose.x_mm;meta.pose.y_mm=raw.pose.y_mm;CK(!memcmp(&meta,&raw,sizeof(raw)));
 angle=raw.path_heading_deg*.01745329251994329577f;c=cosf(angle);s=sinf(angle);
 L=-s*(raw.pose.x_mm-raw.origin_x_mm)+c*(raw.pose.y_mm-raw.origin_y_mm);
 correction=(APP_LEFT_NAV_RESIDUAL_ENABLE&&scope)?(.5f*.0179055385f*((L>0)?L:0)):0;
 CK(fabsf(nav.pose.x_mm-(raw.pose.x_mm+c*correction))<.001f);CK(fabsf(nav.pose.y_mm-(raw.pose.y_mm+s*correction))<.001f);CK(critical_depth==0);return 0;
}
__attribute__((section("i.test_entry"))) uint32_t test_entry(void){
 uint32_t id,n,kind,r,k,result,i;float ds[3]={300,1500,2100};AppOdometryBaseline40 raw,nav,saved;
 for(id=8;id<=19;id++)for(n=0;n<3;n++)for(kind=4;kind<=6;kind++)for(r=101;r<=102;r++)for(k=0;k<3;k++){
  uint32_t scope=n==1&&kind==5&&r==102&&((id==9&&ds[k]==2100)||(id==18&&ds[k]==1500));
  init_case(id,n,kind,r,ds[k]);move_to(-30,1500);result=verify(scope);if(result)return result;
 }
 for(i=0;i<4;i++){
  init_case(9,1,5,102,2100);AppOdometry_ResetRecords();combined_state=1;
  if(i==0)actions[0].run_ms=1;if(i==1)actions[0].reference_mm_s=1;if(i==2)actions[0].relative_deg=1;if(i==3)route_id=1;
  tick();combined_state=2;move_to(-30,1500);result=verify(0);if(result)return result;
 }
 init_case(9,1,5,102,2100);move_to(-30,1500);result=verify(1);if(result)return result;
 AppOdometry_ReadNavigation40(&saved);tick();AppOdometry_ReadNavigation40(&nav);CK(!memcmp(&saved,&nav,sizeof(nav)));
 g_debug_plan_id=10;actions[0].kind=6;g_chassis_auto_active=0;combined_state=9;tick();result=verify(1);if(result)return result;
 completed=1;tick();CK(g_perimeter_records[1].pose.x_mm==desired.x_mm&&g_perimeter_records[1].pose.y_mm==desired.y_mm);
 desired.valid=0;desired.reasons=10;desired.age_ms=401;desired.continuity_lost=1;heading.healthy=0;tick();result=verify(1);if(result)return result;
 AppOdometry_ReadNavigation40(&saved);AppOdometry_ResetRecords();AppOdometry_ReadBaseline40(&raw);AppOdometry_ReadNavigation40(&nav);
 CK(!raw.anchor_valid&&!nav.anchor_valid);CK(nav.pose.x_mm==saved.pose.x_mm&&nav.pose.y_mm==saved.pose.y_mm);tick();result=verify(1);if(result)return result;
 heading.healthy=1;manual_probe=1;allow_pose=0;g_odometry_pose_request=1;tick();CK(g_odometry_pose_result==2&&!probe_failed&&probe_count==1);result=verify(1);if(result)return result;
 allow_pose=1;g_odometry_pose_request=1;g_odometry_pose_x_mm=20;g_odometry_pose_y_mm=30;g_odometry_pose_heading_deg=40;desired.x_mm=20;desired.y_mm=30;tick();CK(g_odometry_pose_result==1&&!probe_failed&&probe_count==2);result=verify(0);if(result)return result;manual_probe=0;
 init_case(9,1,5,102,2100);move_to(0,-100);result=verify(1);if(result)return result;move_to(0,0);result=verify(1);if(result)return result;
 init_case(18,1,5,102,1500);move_to(-30,1500);result=verify(1);if(result)return result;
 /* Actual ICM Step, calibrated while fixture remains stationary at this pose. */
 {IcmRawStatus rr;AppIcmObserverStatus observed;float expected;
  memset(&rr,0,sizeof(rr));rr.state=ICM_RAW_READY;rr.last_error=ICM_RAW_OK;rr.fresh=1;rr.accel_config=0x38;rr.gyro_config=0x28;rr.raw_accel[2]=-8192;
  for(i=0;i<230;i++){desired.sample_ms=clock_ms+5;tick();rr.sample_tick_ms=clock_ms;rr.sample_sequence++;rr.edge_sequence++;AppIcmObserver_Step(&rr);}
  AppIcmObserver_Read(&observed);expected=-30+(APP_LEFT_NAV_RESIDUAL_ENABLE?.5f*.0179055385f*1500:0);
  CK(observed.cal_samples==200);CK(observed.flags&ICM_OBS_WHEEL);CK(fabsf(observed.wheel_f_mm-expected)<.003f);CK(fabsf(observed.wheel_r_mm+1500)<.003f);CK(observed.valid);CK(fabsf(observed.position_f_mm-expected)<.003f);
 }
 return 0;
}
