#include <stdint.h>
#include <math.h>
#include "chassis_kinematics.h"
#include "app_entry.h"
#include "motion_trial20.h"
static uint32_t gate_actual(uint32_t g_debug_plan_id,uint32_t s_translation_mode,
 uint32_t s_distance_max_rpm,float s_distance_target_mm,uint32_t s_route_trial) {
#if APP_BASELINE40_ACTIVE && APP_RIGHT40_FORWARD_TRIM_ENABLE
 return (((g_debug_plan_id==DEBUG_PLAN_RIGHT210_40 || g_debug_plan_id==DEBUG_PLAN_RIGHT150_40 || g_debug_plan_id==DEBUG_PLAN_RIGHT30_40) &&
                s_translation_mode==2U && !s_route_trial &&
                AppDebugPlan40Allowed(g_debug_plan_id,s_translation_mode,s_distance_max_rpm,s_distance_target_mm))) ? 1U : 0U;
#else
 return 0U;
#endif
}

volatile float maxima[5];
volatile uint32_t old_checksum=2166136261U,command_cases,gate_cases,gate_enabled;
static uint32_t checks;
#define CHECK(x) do {checks++;if(!(x))return 0x80000000U|__LINE__;}while(0)
static float ab(float v){return v<0?-v:v;}
static void maxf(uint32_t n,float v){if(v>maxima[n])maxima[n]=v;}
static float decode(ChassisMotorCommand *c,uint32_t i){
 float q=(float)c[i].speed_raw*0.1f;
 return c[i].direction==g_chassis_forward_direction[i]?q:-q;
}
static void dirty(ChassisMotorCommand *c){uint32_t i;for(i=0;i<4;i++)c[i].speed_raw=1234U;}
static uint32_t zero(ChassisMotorCommand *c){uint32_t i;for(i=0;i<4;i++)if(c[i].speed_raw)return 0U;return 1U;}
uint32_t test_entry(void) {
 uint32_t b,i,j,k,mode,id,rpm,route,n; int32_t hi;
 float h,t,q[4],ideal[4],F,L,H,requested,mm,expected,A,J;
 float lengths[7]={300,1500,2100,299,1501,2099,0};
 float special[8]={-.051f,-.05f,-.049f,.049f,.05f,.051f,-1.999f,1.999f};
 union {uint32_t u;float f;} nan,inf;
 ChassisMotorCommand c[4]; nan.u=0x7fc00000U;inf.u=0x7f800000U;lengths[6]=nan.f;
 for(b=0;b<=102;b++)for(hi=-200;hi<=208;hi++) {
  requested=hi<=200?(float)hi*.01f:special[hi-201];
  h=requested;if(h>(float)b)h=(float)b;if(h<-(float)b)h=-(float)b;
  CHECK(Chassis_Right40ForwardCommands(b,requested,c)==1U);command_cases++;
  t=.01f*(float)b;
  ideal[0]=-(float)b+t-h;ideal[1]=(float)b+t+h;
  ideal[2]=(float)b+t-h;ideal[3]=-(float)b+t+h;
  for(i=0;i<4;i++) {
   q[i]=decode(c,i);CHECK(c[i].address==i+1U);CHECK(c[i].speed_raw<=1050U);
   maxf(0,ab(q[i]-ideal[i]));CHECK(ab(q[i]-ideal[i])<=.0501f);maxf(4,ab(q[i]));
  }
  F=(q[0]+q[1]+q[2]+q[3])*.25f;L=(q[0]-q[1]-q[2]+q[3])*.25f;
  H=(-q[0]+q[1]-q[2]+q[3])*.25f;
  maxf(1,ab(F-t));maxf(2,ab(L+(float)b));maxf(3,ab(H-h));
  CHECK(ab(F-t)<=.0501f&&ab(L+(float)b)<=.0501f&&ab(H-h)<=.0501f);
  CHECK(F>=-.00001f&&q[0]<=0&&q[1]>=0&&q[2]>=0&&q[3]<=0);
  if(!b)CHECK(zero(c));
  /* Existing left integer trim remains exactly as before. */
  CHECK(Chassis_Left40DriftCommands(b,requested,c)==1U);
  t=(float)(uint32_t)(.02f*(float)b+.5f);
  ideal[0]=(float)b-t-h;ideal[1]=-(float)b-t+h;
  ideal[2]=-(float)b-t-h;ideal[3]=(float)b-t+h;
  for(i=0;i<4;i++)CHECK(ab(decode(c,i)-ideal[i])<=.0501f);
  /* Record all generic mode outputs for enable0/1 comparison. */
  for(mode=0;mode<4;mode++) {
   CHECK(Chassis_TranslationProfileCommands(mode,b,requested,c)==1U);
   for(i=0;i<4;i++) {old_checksum^=c[i].speed_raw|((uint32_t)c[i].direction<<16);old_checksum*=16777619U;}
  }
 }
 for(i=0;i<7;i++) {
  float bad[7]={-2.001f,2.001f,nan.f,inf.f,-inf.f,0,0};
  dirty(c);CHECK(Chassis_Right40ForwardCommands(i==5?103U:(i==6?0xffffffffU:100U),bad[i],c)==0U);CHECK(zero(c));
 }
 CHECK(Chassis_Right40ForwardCommands(10,0,0)==0U);
 for(id=0;id<=22;id++)for(mode=0;mode<=4;mode++)for(rpm=100;rpm<=103;rpm++)
 for(j=0;j<7;j++)for(route=0;route<=1;route++) {
  mm=lengths[j];n=gate_actual(id,mode,rpm,mm,route);gate_cases++;gate_enabled+=n;
  k=APP_RIGHT40_FORWARD_TRIM_ENABLE&&!route&&mode==2U&&rpm==102U&&
    ((id==10U&&mm==2100)||(id==12U&&mm==1500)||(id==13U&&mm==300));
  CHECK(n==k);
 }
 CHECK(gate_enabled==(APP_RIGHT40_FORWARD_TRIM_ENABLE?3U:0U));
 for(mode=0;mode<=4;mode++)for(rpm=100;rpm<=103;rpm++)for(j=0;j<7;j++) {
  mm=lengths[j];k=(mm==300||mm==1500||mm==2100)&&
    (((mode==0||mode==3)&&rpm==100)||((mode==1||mode==2)&&rpm==102));
  expected=k?150.0f:APP_DISTANCE_SCURVE_ACCEL_MM_S2;
  if(k&&mode==1)expected=150.0f/1.02f;
  if(k&&mode==2&&APP_RIGHT40_FORWARD_TRIM_ENABLE)expected=150.0f/1.01f;
  A=MotionDistanceAcceleration(mode,rpm,mm);CHECK(ab(A-expected)<.0001f);
  expected=k?300.0f:APP_DISTANCE_SCURVE_JERK_MM_S3;
  if(k&&mode==1)expected=300.0f/1.02f;
  if(k&&mode==2&&APP_RIGHT40_FORWARD_TRIM_ENABLE)expected=300.0f/1.01f;
  J=MotionDistanceJerk(mode,rpm,mm);CHECK(ab(J-expected)<.0001f);
  if(k)CHECK(MotionDistanceFeedbackLimit(mode,rpm,mm)==130U);
 }
 return checks;
}
