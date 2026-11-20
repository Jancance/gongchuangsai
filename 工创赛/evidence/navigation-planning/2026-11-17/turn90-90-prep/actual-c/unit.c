
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
#include "heading_session.h"
#include <math.h>
#include "motion_route_trial.h"
static MotionAction s_motion_actions[8];
static uint32_t test_count;
volatile uint32_t g_debug_plan_id;
#define MOTION_ACTION_COUNT test_count
static uint32_t App_MotionPlanValid(void)
{
    uint32_t i, budget = 5000U;
    if (!MOTION_ACTION_COUNT || MOTION_ACTION_COUNT > MOTION_ACTION_CAPACITY) return 0U;
    if(AppDebugTurn90Plan(g_debug_plan_id) &&
        (MOTION_ACTION_COUNT!=1U || s_motion_actions[0].kind!=MOTION_TURN ||
         !AppDebugTurn90Allowed(g_debug_plan_id,s_motion_actions[0].base_rpm,s_motion_actions[0].relative_deg)))return 0U;
    for (i = 0U; i < MOTION_ACTION_COUNT; i++)
    {
        const MotionAction *action = &s_motion_actions[i];
        if(action->reference_mm_s!=0.0f) {
            if(g_debug_plan_id!=DEBUG_PLAN_DEFAULT || !APP_ROUTE_SPEED_TRIAL_ACTIVE ||
                action->kind<MOTION_FORWARD_DISTANCE || action->kind>MOTION_BACKWARD_DISTANCE ||
                action->relative_deg!=0.0f || !MotionRoute_ShapeAllowed(action->kind-MOTION_FORWARD_DISTANCE,
                    action->base_rpm,action->run_ms,action->distance_mm,action->reference_mm_s)) return 0U;
            budget+=action->run_ms+6000U;continue;
        }
        if (action->kind <= MOTION_RIGHT)
        {
            if ((action->base_rpm < 3U) || (action->base_rpm > 15U) ||
                (action->run_ms == 0U) || (action->run_ms > 8000U) ||
                (action->relative_deg != 0.0f) || action->distance_mm != 0.0f) return 0U;
            budget += action->run_ms + 3500U;
        }
        else if (action->kind == MOTION_TURN)
        {
            if ((action->base_rpm > 5U && !(APP_TURN_BRAKE_TRIAL_ACTIVE &&
                (MOTION_ACTION_COUNT>=1U && MOTION_ACTION_COUNT<=MOTION_ACTION_CAPACITY) && (g_debug_plan_id==DEBUG_PLAN_DEFAULT || AppDebugTurn90Plan(g_debug_plan_id)) &&
                action->base_rpm==APP_TURN_BRAKE_MAX_RPM &&
                (action->relative_deg==90.0f || action->relative_deg==-90.0f))) || action->run_ms || action->distance_mm != 0.0f ||
                !(((action->relative_deg >= 5.0f) && (action->relative_deg <= 90.0f)) ||
                  ((action->relative_deg <= -5.0f) && (action->relative_deg >= -90.0f)))) return 0U;
            budget += APP_HEADING_RUN_MS + 3500U;
        }
        else if (action->kind >= MOTION_FORWARD_DISTANCE && action->kind <= MOTION_BACKWARD_DISTANCE)
        {
            if (!APP_POSITION_DIAGNOSTIC || !Chassis_DistanceSelfTest() ||
                action->base_rpm < 3U || action->run_ms ||
                action->relative_deg != 0.0f ||
                !MotionDistanceAllowed(action->kind-MOTION_FORWARD_DISTANCE,action->base_rpm,action->distance_mm)) return 0U;
            if (action->base_rpm>15U &&
                (MOTION_ACTION_COUNT!=1U || (g_debug_plan_id!=DEBUG_PLAN_FORWARD210_20 &&
                 !AppDebugPlan40Allowed(g_debug_plan_id,action->kind-MOTION_FORWARD_DISTANCE,action->base_rpm,action->distance_mm)))) return 0U;
            budget += MotionDistanceRunMsFor(action->kind-MOTION_FORWARD_DISTANCE,
                action->base_rpm,action->distance_mm) + 3500U;
        }
        else return 0U;
    }
    return (budget <= APP_PERIMETER_GROUP_MS) ? 1U : 0U;
}

volatile uint32_t review_assertions;
volatile float results[54][7];
#define CK(x) do{review_assertions++;if(!(x))return __LINE__;}while(0)
__attribute__((section("i.test_entry"))) uint32_t test_entry(void){
 uint32_t id,n,j,tauid,lagid,gid,di,ix=0;MotionAction action[8];uint32_t count;ChassisMotorCommand cmds[4];
 const float taus[3]={.04f,.08f,.15f},gains[3]={.9f,1.0f,1.1f};
 for(id=0;id<=22;id++){
  uint32_t good=(id>=8 && id<=21);CK(AppDebugPlan_Load(id,action,&count)==good);
  if(id==20||id==21){CK(count==1 && action[0].kind==MOTION_TURN && action[0].base_rpm==82);CK(action[0].relative_deg==(id==20?90.0f:-90.0f));CK(!action[0].run_ms && !action[0].distance_mm && !action[0].reference_mm_s);}
 }
 CK(!AppDebugTurn90Allowed(20,81,90));CK(!AppDebugTurn90Allowed(20,82,-90));CK(!AppDebugTurn90Allowed(21,82,90));CK(!AppDebugTurn90Allowed(20,82,89));
 CK(Chassis_TurnBrakeCommands(81.4f,cmds));for(j=0;j<4;j++)CK(cmds[j].speed_raw==814);
 CK(!Chassis_TurnBrakeCommands(82.1f,cmds));
 for(tauid=0;tauid<3;tauid++)for(lagid=0;lagid<3;lagid++)for(gid=0;gid<3;gid++)for(di=0;di<2;di++,ix++){
  HeadingSession session;HeadingSessionInput in={0};HeadingSessionOutput out={0};
  float theta=0,rate=0,command=0,peak=0,dir=di?1.0f:-1.0f;float angles[64]={0},rates[64]={0},commands[64]={0};uint32_t stable=0,done=0;
  HeadingSession_Init(&session);in.max_rpm=82;in.healthy=in.platform_ok=1;in.sequence=1;
  CK(HeadingSession_Start(&session,&in,dir*90,&out));CK(session.brake.profile.valid && fabsf(session.brake.profile.peak_speed-90)<.001f);
  HeadingSession_ReportBegin(&session,1,&out);in.backend_active=in.backend_running=1;
  for(n=0;n<3000;n++){
   uint32_t source=n>=4?n-4:0,delay=lagid==0?0:(lagid==1?9:18);float observed=n>=4?angles[source%64]:0,velocity=n>=4?rates[source%64]:0;
   in.now_ms=(n+1)*5;in.sample_ms=in.now_ms;in.sequence++;in.yaw_deg=HeadingController_WrapDeg(observed);in.rate_dps=velocity;
   HeadingSession_Step(&session,&in,&out);CK(out.action!=HS_ACTION_ABORT && session.state!=HS_FAULT);CK(dir*out.command_rpm>=-.001f && fabsf(out.command_rpm)<=81.401f);
   if(out.action==HS_ACTION_STOP){HeadingSession_ReportStop(&session,1,&out);in.backend_active=in.backend_running=0;in.backend_result=1;in.backend_stop_mask=15;in.now_ms+=5;in.sample_ms=in.now_ms;in.sequence++;HeadingSession_Step(&session,&in,&out);CK(out.action==HS_ACTION_COMPLETE);done=1;break;}
   if(n%9==0)command=out.command_rpm;commands[n%64]=command;
   angles[n%64]=theta;rates[n%64]=rate;
   {float target=(n>=delay?commands[(n-delay)%64]:0)*(6.0f*77/418)*gains[gid];rate+=(target-rate)*(1-expf(-.005f/taus[tauid]));theta+=rate*.005f;}
   if(fabsf(rate)>peak)peak=fabsf(rate);
  }
  CK(done && fabsf(90-dir*theta)<=1.0f);results[ix][0]=theta;results[ix][1]=peak;results[ix][2]=n*.005f;results[ix][3]=taus[tauid];results[ix][4]=lagid*.044f;results[ix][5]=gains[gid];results[ix][6]=dir;
 }
 {HeadingSession a;HeadingSessionInput in={0};HeadingSessionOutput out={0};in.max_rpm=82;in.healthy=in.platform_ok=1;in.sequence=1;HeadingSession_Init(&a);CK(HeadingSession_Start(&a,&in,90,&out));HeadingSession_Cancel(&a,&out);CK(out.action==HS_ACTION_ABORT);
  HeadingSession_Init(&a);CK(HeadingSession_Start(&a,&in,90,&out));in.now_ms=101;in.sample_ms=0;in.sequence++;HeadingSession_Step(&a,&in,&out);CK(out.action==HS_ACTION_ABORT);
  in.now_ms=in.sample_ms=0;in.sequence=1;HeadingSession_Init(&a);CK(HeadingSession_Start(&a,&in,90,&out));HeadingSession_ReportBegin(&a,1,&out);in.backend_running=in.backend_active=1;in.now_ms=in.sample_ms=5;in.sequence++;in.yaw_deg=91.1f;HeadingSession_Step(&a,&in,&out);CK(out.action==HS_ACTION_ABORT);
 }

 for(id=20;id<=21;id++){
  g_debug_plan_id=id;CK(AppDebugPlan_Load(id,s_motion_actions,&test_count));CK(App_MotionPlanValid());
  s_motion_actions[0].relative_deg=-s_motion_actions[0].relative_deg;CK(!App_MotionPlanValid());s_motion_actions[0].relative_deg=-s_motion_actions[0].relative_deg;
  test_count=2;CK(!App_MotionPlanValid());test_count=1;s_motion_actions[0].distance_mm=1;CK(!App_MotionPlanValid());
 }
 {float starts[3]={37.0f,170.0f,-170.0f};uint32_t z;for(z=0;z<3;z++)for(di=0;di<2;di++){
  HeadingSession a;HeadingSessionInput in={0};HeadingSessionOutput out={0};float relative=di?90.0f:-90.0f;
  HeadingSession_Init(&a);in.max_rpm=82;in.healthy=in.platform_ok=1;in.sequence=1;in.yaw_deg=starts[z];
  CK(HeadingSession_Start(&a,&in,relative,&out));CK(fabsf(HeadingController_ErrorDeg(out.target_deg,starts[z])-relative)<.001f);
  if(starts[z]==37.0f)CK(fabsf(out.target_deg-(relative>0?127.0f:-53.0f))<.001f);
 }}
 return 0;
}
