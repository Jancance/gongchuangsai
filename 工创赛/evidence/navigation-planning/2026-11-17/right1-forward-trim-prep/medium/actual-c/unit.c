
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
#include <math.h>
static uint32_t Auto_PrepareBasicCommands(ChassisMotorCommand commands[3][CHASSIS_WHEEL_COUNT])
{
    uint32_t i, j;
    for (i = 0U; i < 3U; i++)
    {
        if (!Chassis_BasicTestCommands(&g_chassis_geometry, i, commands[i])) return 0U;
        for (j = 0U; j < CHASSIS_WHEEL_COUNT; j++)
            commands[i][j].speed_raw = (uint16_t)(10U*CHASSIS_AUTO_TARGET_RPM);
#if APP_HWT_TURN_TEST && APP_HWT_TURN_CLOCKWISE
        if (i == 2U)
            for (j = 0U; j < CHASSIS_WHEEL_COUNT; j++) commands[i][j].direction ^= 1U;
#endif
    }
    return 1U;
}
static uint32_t Auto_PrepareTranslationEx(uint32_t motion_mode, uint32_t base_rpm,
    uint32_t run_ms, float distance_mm,float route_speed,ChassisAutoPreparedTranslation *prepared)
{
    if (!prepared) return 0U;
    prepared->valid = 0U;prepared->route_reference_mm_s=0.0f;
    if(route_speed!=0.0f && !MotionRoute_ShapeAllowed(motion_mode,base_rpm,run_ms,distance_mm,route_speed)) return 0U;
    if (__get_BASEPRI() || __get_PRIMASK() || motion_mode > 3U ||
        base_rpm < 3U || (base_rpm > 15U && route_speed==0.0f && !MotionTrial20Allowed(motion_mode,base_rpm,distance_mm) && !MotionTrial40Allowed(motion_mode,base_rpm,distance_mm))) return 0U;
    if (distance_mm != 0.0f)
    {
#if APP_POSITION_DIAGNOSTIC
        if ((!route_speed && (!MotionDistanceAllowed(motion_mode,base_rpm,distance_mm) ||
                run_ms != MotionDistanceRunMsFor(motion_mode,base_rpm,distance_mm))) ||
            !Chassis_DistanceSelfTest()) return 0U;
#else
        return 0U;
#endif
    }
    else if (!run_ms || run_ms > 10000U) return 0U;
    if (!Auto_PrepareBasicCommands(prepared->commands)) return 0U;
#if APP_DISTANCE_SCURVE_ACTIVE
    if (distance_mm>0.0f)
    { if (!Chassis_TranslationProfileCommands(motion_mode,0U,0,prepared->commands[2])) return 0U; }
    else
#endif
    if (!Chassis_TranslationHoldCommandsAtSpeed(motion_mode,base_rpm,0,prepared->commands[2])) return 0U;
#if APP_DISTANCE_SCURVE_ACTIVE
    prepared->profile.valid = 0U;
    if (distance_mm > 0.0f)
    {
        MotionProfileLimits limits;
        float factor;
        if (!Chassis_CalibrateDistanceMm(motion_mode,
                3.14159265358979323846f*g_chassis_geometry.wheel_diameter_mm/60.0f,&factor)) return 0U;
        limits.speed = MotionDistanceReferenceSpeed(factor,base_rpm);
        limits.acceleration = MotionDistanceAcceleration(motion_mode,base_rpm,distance_mm) -
            factor*2.0f*MotionDistanceFollowRpm(base_rpm)*APP_DISTANCE_SCURVE_FOLLOW_OMEGA;
        limits.jerk = MotionDistanceJerk(motion_mode,base_rpm,distance_mm) -
            factor*6.0f*MotionDistanceFollowRpm(base_rpm)*
                APP_DISTANCE_SCURVE_FOLLOW_OMEGA*APP_DISTANCE_SCURVE_FOLLOW_OMEGA;
        if(route_speed!=0.0f) {
            uint32_t expected_rpm,expected_ms;
            if(!MotionRoute_Build(motion_mode,distance_mm,route_speed,&prepared->profile,&expected_rpm,&expected_ms) ||
                expected_rpm!=base_rpm || expected_ms!=run_ms) return 0U;
        } else if(!MotionProfile_Prepare(&prepared->profile,distance_mm,&limits)) return 0U;
        if (
            prepared->profile.duration_s*1000.0f + APP_DISTANCE_SCURVE_FINAL_WAIT_MS >= (float)run_ms ||
            !Chassis_TranslationProfileCommands(motion_mode,0U,0,prepared->commands[2])) return 0U;
    }
#endif
    prepared->motion_mode = motion_mode; prepared->base_rpm = base_rpm;
    prepared->run_ms = run_ms; prepared->distance_mm = distance_mm;
    prepared->route_reference_mm_s=route_speed;
    prepared->valid = 1U;
    return 1U;
}
uint32_t ChassisAuto_PrepareTranslation(uint32_t mode,uint32_t rpm,uint32_t run_ms,
    float mm,ChassisAutoPreparedTranslation *prepared)
{ return Auto_PrepareTranslationEx(mode,rpm,run_ms,mm,0.0f,prepared); }
uint32_t validate_only(const ChassisAutoPreparedTranslation *prepared,
    uint32_t expected_session)
{
    uint32_t accepted = 0U;
    if(prepared && prepared->route_reference_mm_s!=0.0f &&
        !MotionRoute_ShapeAllowed(prepared->motion_mode,prepared->base_rpm,prepared->run_ms,
            prepared->distance_mm,prepared->route_reference_mm_s)) return 0U;
    if (!prepared || prepared->valid != 1U || prepared->motion_mode > 3U ||
        prepared->base_rpm < 3U || (prepared->base_rpm > 15U && prepared->route_reference_mm_s==0.0f &&
            !MotionTrial20Allowed(prepared->motion_mode,prepared->base_rpm,prepared->distance_mm) &&
            !MotionTrial40Allowed(prepared->motion_mode,prepared->base_rpm,prepared->distance_mm))) return 0U;
    if (prepared->distance_mm != 0.0f)
    {
#if APP_POSITION_DIAGNOSTIC
        if (!prepared->route_reference_mm_s &&
            (!MotionDistanceAllowed(prepared->motion_mode,prepared->base_rpm,prepared->distance_mm) ||
             prepared->run_ms != MotionDistanceRunMsFor(prepared->motion_mode,prepared->base_rpm,prepared->distance_mm))) return 0U;
#else
        return 0U;
#endif
    }
    else if (!prepared->run_ms || prepared->run_ms > 10000U) return 0U;
#if APP_DISTANCE_SCURVE_ACTIVE
    if (prepared->distance_mm > 0.0f)
    {
        float factor;
        MotionProfileLimits route_limits;uint32_t route_rpm;
        if(prepared->route_reference_mm_s!=0.0f) {
            if(!MotionRoute_Limits(prepared->motion_mode,prepared->distance_mm,prepared->route_reference_mm_s,&route_limits,&route_rpm) ||
                prepared->base_rpm!=route_rpm || prepared->run_ms!=MotionRoute_Deadline(prepared->profile.duration_s) ||
                prepared->profile.limits.speed!=route_limits.speed ||
                prepared->profile.limits.acceleration!=route_limits.acceleration ||
                prepared->profile.limits.jerk!=route_limits.jerk) return 0U;
        }
        if (!Chassis_CalibrateDistanceMm(prepared->motion_mode,
                3.14159265358979323846f*g_chassis_geometry.wheel_diameter_mm/60.0f,&factor) ||
            (!prepared->route_reference_mm_s && (prepared->profile.limits.speed != MotionDistanceReferenceSpeed(factor,prepared->base_rpm) ||
            prepared->profile.limits.acceleration != MotionDistanceAcceleration(prepared->motion_mode,prepared->base_rpm,prepared->distance_mm) -
                factor*2.0f*MotionDistanceFollowRpm(prepared->base_rpm)*APP_DISTANCE_SCURVE_FOLLOW_OMEGA ||
            prepared->profile.limits.jerk != MotionDistanceJerk(prepared->motion_mode,prepared->base_rpm,prepared->distance_mm) -
                factor*6.0f*MotionDistanceFollowRpm(prepared->base_rpm)*
                    APP_DISTANCE_SCURVE_FOLLOW_OMEGA*APP_DISTANCE_SCURVE_FOLLOW_OMEGA)) ||
            prepared->profile.boundary_s[0] != 0.0f || prepared->profile.position[0] != 0.0f ||
            prepared->profile.speed[0] != 0.0f || prepared->profile.acceleration[0] != 0.0f)
            return 0U;
    }
    if (prepared->distance_mm > 0.0f &&
        (prepared->profile.valid != 1U || prepared->profile.distance_mm != prepared->distance_mm ||
         !(prepared->profile.duration_s > 0.0f) ||
         !(prepared->profile.duration_s*1000.0f + APP_DISTANCE_SCURVE_FINAL_WAIT_MS < (float)prepared->run_ms))) return 0U;
#endif
    return 1U;
}
static uint32_t selected_right(uint32_t g_debug_plan_id,uint32_t s_translation_mode,uint32_t s_distance_max_rpm,float s_distance_target_mm,uint32_t s_route_trial){
#if APP_BASELINE40_ACTIVE && APP_RIGHT40_FORWARD_TRIM_ENABLE
return 
                ((g_debug_plan_id==DEBUG_PLAN_RIGHT210_40 || g_debug_plan_id==DEBUG_PLAN_RIGHT150_40 || g_debug_plan_id==DEBUG_PLAN_RIGHT30_40) &&
                s_translation_mode==2U && !s_route_trial &&
                AppDebugPlan40Allowed(g_debug_plan_id,s_translation_mode,s_distance_max_rpm,s_distance_target_mm));
#else
return 0U;
#endif
}

volatile uint32_t review_assertions;
#define CK(x) do{review_assertions++;if(!(x))return __LINE__;}while(0)
__attribute__((section("i.test_entry"))) uint32_t test_entry(void){
 uint32_t id,d,r,k,i,n,b,z,route;MotionAction a[8];ChassisMotorCommand cmd[4];
 uint32_t dirs[12]={0,1,2,3,2,2,0,0,3,3,1,1};
 float mm[12]={2100,2100,2100,2100,1500,300,1500,300,1500,300,1500,300};
 uint32_t rpms[5]={99,100,101,102,103};float ds[6]={299,300,1500,1501,2100,2101};
 for(id=0;id<=22;id++)for(d=0;d<4;d++)for(r=0;r<5;r++)for(k=0;k<6;k++){
  uint32_t good=APP_BASELINE40_ACTIVE && id>=8 && id<=19 && d==dirs[id-8] && ds[k]==mm[id-8] && rpms[r]==((d==0||d==3)?100:102);
  CK(AppDebugPlan40Allowed(id,d,rpms[r],ds[k])==good);
  for(route=0;route<2;route++)CK(selected_right(id,d,rpms[r],ds[k],route)==(good && APP_RIGHT40_FORWARD_TRIM_ENABLE && !route && d==2 && (id==10||id==12||id==13)));
 }
 for(id=8;id<=19;id++){
  CK(AppDebugPlan_Load(id,a,&n)==APP_BASELINE40_ACTIVE);
  if(APP_BASELINE40_ACTIVE){ChassisAutoPreparedTranslation p,bad;
   CK(n==1 && a[0].kind==dirs[id-8]+4 && a[0].distance_mm==mm[id-8]);
   CK(ChassisAuto_PrepareTranslation(dirs[id-8],a[0].base_rpm,12779,mm[id-8],&p));
   CK(p.valid && p.profile.valid && p.profile.distance_mm==mm[id-8]);
   CK(p.profile.duration_s*1000+2000<12779);CK(validate_only(&p,0));
   bad=p;bad.profile.limits.acceleration+=.1f;CK(!validate_only(&bad,0));
   bad=p;bad.profile.limits.jerk+=.1f;CK(!validate_only(&bad,0));
   bad=p;bad.profile.distance_mm+=1;CK(!validate_only(&bad,0));
   for(i=0;i<4;i++)CK(p.commands[2][i].speed_raw==0);
  }
 }
 CK(!ChassisAuto_PrepareTranslation(2,102,12779,2101,0));CK(!validate_only(0,0));
#if APP_BASELINE40_ACTIVE
 for(d=0;d<4;d++)for(k=0;k<3;k++){
  float dist[3]={300,1500,2100};uint32_t cap=(d==0||d==3)?100:102;
  float denom=(d==1&&APP_LEFT40_DRIFT_TRIM_ENABLE)?1.02f:((d==2&&APP_RIGHT40_FORWARD_TRIM_ENABLE)?1.01f:1.0f);
  CK(MotionDistanceAcceleration(d,cap,dist[k])==150.0f/denom);
  CK(MotionDistanceJerk(d,cap,dist[k])==300.0f/denom);
  CK(MotionDistanceFeedbackLimit(d,cap,dist[k])==130 && MotionDistanceRunMsFor(d,cap,dist[k])==12779);
 }
 for(b=0;b<=102;b++)for(z=0;z<=160;z++){
  float requested=((int32_t)z-80)*.025f,limit=(b<2?b:2),h=requested,t=.01f*b,w[4],ideal[4];
  if(h>limit)h=limit;if(h< -limit)h=-limit;
  ideal[0]=-((float)b)+t-h;ideal[1]=(float)b+t+h;ideal[2]=(float)b+t-h;ideal[3]=-((float)b)+t+h;
  CK(Chassis_Right40ForwardCommands(b,requested,cmd));
  for(i=0;i<4;i++){
   CK(cmd[i].speed_raw<=1060);w[i]=(float)cmd[i].speed_raw*.1f;
   if(cmd[i].direction!=g_chassis_forward_direction[i])w[i]=-w[i];
   CK(fabsf(w[i]-ideal[i])<=.0501f);if(!b)CK(cmd[i].speed_raw==0);
  }
  CK(fabsf((w[0]+w[1]+w[2]+w[3])*.25f-t)<=.0501f);
  CK(fabsf((w[0]-w[1]-w[2]+w[3])*.25f+(float)b)<=.0501f);
  CK(fabsf((-w[0]+w[1]-w[2]+w[3])*.25f-h)<=.0501f);
 }
 CK(!Chassis_Right40ForwardCommands(103,0,cmd));CK(!Chassis_Right40ForwardCommands(102,2.01f,cmd));CK(!Chassis_Right40ForwardCommands(102,-2.01f,cmd));CK(!Chassis_Right40ForwardCommands(102,0,0));
 {union{uint32_t u;float f;} q;q.u=0x7fc00000;CK(!Chassis_Right40ForwardCommands(102,q.f,cmd));for(i=0;i<4;i++)CK(!cmd[i].speed_raw);q.u=0x7f800000;CK(!Chassis_Right40ForwardCommands(102,q.f,cmd));}
#else
 {ChassisAutoPreparedTranslation p;CK(!ChassisAuto_PrepareTranslation(2,102,12779,2100,&p));}
#endif
 return 0;
}
