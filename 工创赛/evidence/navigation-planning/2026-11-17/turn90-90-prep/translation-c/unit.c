
#include <stdint.h>
#include <string.h>
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_config.h"
#include "app_entry.h"
#include "motion_distance_limits.h"
#include "app_fusion_capture.h"
#include "app_odometry.h"
#include "chassis_auto_test.h"
#include "heading_source.h"
#include "power.h"
static uint32_t clock_ms;
static ChassisAutoStatus motion;
static ChassisWheelLog wheel;
static ChassisOdometrySamples position;
#if APP_BASELINE40_ACTIVE
static AppOdometryBaseline40 od;
#endif
static AppHeadingSource heading;
volatile uint32_t g_vm_test_request,g_safety_fault,g_rtos_fault,g_can_ctrl_fault_latched;
uint32_t SystemCoreClock=168000000U;
TickType_t xTaskGetTickCount(void){return clock_ms;}
uint32_t ChassisAuto_ReadStatus(ChassisAutoStatus *v){*v=motion;return 1U;}
uint32_t ChassisAuto_ReadWheelLog(ChassisWheelLog *v){*v=wheel;return 1U;}
void ChassisAuto_ReadOdometrySamples(ChassisOdometrySamples *v){*v=position;}
#if APP_BASELINE40_ACTIVE
void AppOdometry_ReadBaseline40(AppOdometryBaseline40 *v){*v=od;}
#endif
uint32_t App_HeadingSourceRead(AppHeadingSource *v){*v=heading;return 1U;}
#include <math.h>
static uint32_t parse_plan(const char *p){if((strncmp(p,"PLAN",4U)==0 && p[4]>='0' && p[4]<='9' && p[5]=='\0') ||
             strcmp(p,"PLAN10")==0 || strcmp(p,"PLAN11")==0 || strcmp(p,"PLAN12")==0 || strcmp(p,"PLAN13")==0 ||
             strcmp(p,"PLAN14")==0 || strcmp(p,"PLAN15")==0 || strcmp(p,"PLAN16")==0 || strcmp(p,"PLAN17")==0 || strcmp(p,"PLAN18")==0 || strcmp(p,"PLAN19")==0 ||
              strcmp(p,"PLAN20")==0 || strcmp(p,"PLAN21")==0)return p[5] ? ((uint32_t)(p[4]-'0')*10U+(uint32_t)(p[5]-'0')) : (uint32_t)(p[4]-'0');return 99U;}


volatile uint32_t review_assertions;
#define CK(x) do{review_assertions++;if(!(x))return __LINE__;}while(0)
__attribute__((section("i.test_entry"))) uint32_t test_entry(void){
 uint32_t id,d,r,k,i,n,b,z;MotionAction a[8];ChassisMotorCommand cmd[4];
 uint32_t dirs[12]={0,1,2,3,2,2,0,0,3,3,1,1};
 float mm[12]={2100,2100,2100,2100,1500,300,1500,300,1500,300,1500,300};
 uint32_t rpms[]={0,99,100,101,102,103};
 float ds[]={0,299,300,301,1499,1500,1501,2099,2100,2101};
 const char *bad[]={"PLAN","PLAN22","PLAN019","PLAN19x","PLAN190","PLAN020","PLAN12x","PLAN120","PLAN+12","PLAN-1","PLAN00","PLAN1 "};
 CK(DEBUG_PLAN_MAX==21 && DEBUG_PLAN_FORWARD150_40==14 && DEBUG_PLAN_FORWARD30_40==15);
 CK(DEBUG_PLAN_BACKWARD150_40==16 && DEBUG_PLAN_BACKWARD30_40==17 && DEBUG_PLAN_LEFT150_40==18 && DEBUG_PLAN_LEFT30_40==19);
 for(id=8;id<=19;id++){char text[7]={'P','L','A','N',0,0,0};if(id<10)text[4]='0'+id;else{text[4]='1';text[5]='0'+id-10;}CK(parse_plan(text)==id);}
 for(i=0;i<sizeof(bad)/sizeof(bad[0]);i++)CK(parse_plan(bad[i])==99);
 CK(parse_plan("PLAN20")==20);CK(parse_plan("PLAN21")==21);
 for(id=0;id<=20;id++)for(d=0;d<4;d++)for(r=0;r<6;r++)for(k=0;k<10;k++){
  uint32_t expected=APP_BASELINE40_ACTIVE && id>=8 && id<=19 && d==dirs[id-8] && ds[k]==mm[id-8] && rpms[r]==((d==0||d==3)?100:102);
  CK(AppDebugPlan40Allowed(id,d,rpms[r],ds[k])==expected);
 }
 for(d=0;d<4;d++)for(r=0;r<6;r++)for(k=0;k<10;k++){
  uint32_t expected=APP_BASELINE40_ACTIVE && (ds[k]==2100||ds[k]==1500||ds[k]==300) && rpms[r]==((d==0||d==3)?100:102);
  CK(MotionTrial40Allowed(d,rpms[r],ds[k])==expected);
 }
 for(id=8;id<=19;id++){
  CK(AppDebugPlan_Load(id,a,&n)==APP_BASELINE40_ACTIVE);
  if(APP_BASELINE40_ACTIVE){CK(n==1 && a[0].kind==dirs[id-8]+4);CK(a[0].base_rpm==((dirs[id-8]==0||dirs[id-8]==3)?100:102));CK(a[0].distance_mm==mm[id-8]);CK(!a[0].run_ms && a[0].relative_deg==0);}
 }
 CK(!AppDebugPlan_Load(14,0,&n));CK(!AppDebugPlan_Load(19,a,0));CK(!AppDebugPlan_Load(20,a,&n));
#if APP_BASELINE40_ACTIVE
 CK(!AppDebugPlan_Load(0,a,&n));CK(!AppDebugPlan_Load(5,a,&n));
 for(d=0;d<4;d++)for(k=0;k<3;k++){
  MotionProfile p;MotionProfileLimits lim;MotionProfileSample sample;
  float factor,t,previous,dist[3]={300,1500,2100};uint32_t cap=(d==0||d==3)?100:102;
  CK(MotionDistanceAllowed(d,cap,dist[k]));CK(MotionDistanceFeedbackLimit(d,cap,dist[k])==130 && MotionDistanceRunMsFor(d,cap,dist[k])==12779);
  CK(Chassis_CalibrateDistanceMm(d,3.14159265358979323846f*77.0f/60.0f,&factor));
  CK(fabsf(factor-((d==0||d==3)?4.119704268f:4.031710572f))<.0001f);
  lim.speed=MotionDistanceReferenceSpeed(factor,cap);
  lim.acceleration=MotionDistanceAcceleration(d,cap,dist[k])-factor*4;
  lim.jerk=MotionDistanceJerk(d,cap,dist[k])-factor*12;
  CK(fabsf(MotionDistanceAcceleration(d,cap,dist[k])-((d==1&&APP_LEFT40_DRIFT_TRIM_ENABLE)?150.0f/1.02f:150))<.001f);
  CK(fabsf(MotionDistanceJerk(d,cap,dist[k])-((d==1&&APP_LEFT40_DRIFT_TRIM_ENABLE)?300.0f/1.02f:300))<.001f);
  CK(MotionProfile_Prepare(&p,dist[k],&lim));CK(p.valid && p.limits.speed==400 && p.peak_speed<=400);
  if(k==0)CK(p.peak_speed>160 && p.peak_speed<170);else CK(fabsf(p.peak_speed-400)<.01f);
  CK(p.duration_s+2<12.779f);previous=0;
  for(t=0;t<p.duration_s;t+=.02f){
   CK(MotionProfile_Sample(&p,t,&sample)==MOTION_PROFILE_RUNNING);
   CK(sample.position_mm>=previous-.003f && sample.position_mm<=dist[k]+.003f);
   CK(sample.speed_mm_s>=-.003f && sample.speed_mm_s<=p.peak_speed+.03f);
   CK(fabsf(sample.acceleration_mm_s2)<=lim.acceleration+.03f);
   CK(fabsf(sample.jerk_mm_s3)<=lim.jerk+.03f);previous=sample.position_mm;
  }
  CK(MotionProfile_Sample(&p,p.duration_s,&sample)==MOTION_PROFILE_DONE);
  CK(sample.position_mm==dist[k] && sample.speed_mm_s==0 && sample.acceleration_mm_s2==0);
 }
 for(d=0;d<4;d++)for(b=0;b<=((d==0||d==3)?100:102);b++)for(z=0;z<3;z++){
  int32_t w[4],F,L;float h=(float)((int32_t)z*2-2);int32_t trim=(d==1&&APP_LEFT40_DRIFT_TRIM_ENABLE)?(int32_t)(.02f*b+.5f):0;
  if(d==1&&APP_LEFT40_DRIFT_TRIM_ENABLE)CK(Chassis_Left40DriftCommands(b,h,cmd));else CK(Chassis_TranslationProfileCommands(d,b,h,cmd));
  for(i=0;i<4;i++){CK(cmd[i].speed_raw<1300);w[i]=cmd[i].direction?-(int32_t)cmd[i].speed_raw:(int32_t)cmd[i].speed_raw;if(g_chassis_forward_direction[i])w[i]=-w[i];}
  F=w[0]+w[1]+w[2]+w[3];L=w[0]-w[1]-w[2]+w[3];
  CK(F==(d==0?(int32_t)b*40:(d==3?-(int32_t)b*40:-trim*40)));
  CK(L==(d==1?(int32_t)b*40:(d==2?-(int32_t)b*40:0)));
  if(!b)for(i=0;i<4;i++)CK(cmd[i].speed_raw==0);
 }
#endif
 return 0;
}
