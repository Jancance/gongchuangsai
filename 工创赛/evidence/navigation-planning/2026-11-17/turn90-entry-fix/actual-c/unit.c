
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_config.h"
#include "app_entry.h"
#include "app_motion_runtime.h"
#include "app_heading_runtime.h"
#include "heading_source.h"
#include "route_heading_reference.h"
#include "motion_control.h"
#undef taskENTER_CRITICAL
#undef taskEXIT_CRITICAL
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
static MotionAction s_motion_actions[8];
static uint32_t s_motion_action_count;
#define MOTION_ACTION_COUNT s_motion_action_count
volatile uint32_t g_debug_plan_id;
volatile CombinedTestDebug g_combined_test;
volatile uint32_t g_heading_exec_trip,g_heading_exec_phase,g_heading_exec_state,g_heading_exec_request,g_motion_fault_diag_valid;
volatile float g_heading_exec_relative_deg;
volatile HeadingStopTrace g_heading_stop_trace;
static uint32_t s_motion_list_submit,s_combined_turn,s_motion_turn_max_rpm,s_combined_base_rpm,s_combined_run_ms,s_motion_has_action;
static float s_combined_distance_mm,s_combined_reference_mm_s;
static uint32_t mode,turn_calls,perimeter_calls,move_calls,reset_calls;
static RouteHeadingReference s_route_heading_reference;
static float s_route_heading_target,source_yaw;
static uint32_t s_route_heading_turn_active;
volatile float g_route_heading_targets[8],g_route_heading_start_yaw[8];
static uint32_t HWT101_TimingBegin(void){return 1;}
static void HWT101_TimingEnd(uint32_t kind,uint32_t tick){(void)kind;(void)tick;}
static void HWT101_DiagnosticArm(void){}
void AppHeadingRuntime_ResetSession(void){reset_calls++;}
uint32_t App_HeadingSourceRead(AppHeadingSource *heading){memset(heading,0,sizeof(*heading));heading->yaw_deg=source_yaw;return 1;}
static uint32_t App_MotionAccept(void){return 1;}
static uint32_t App_MoveSubmit(uint32_t d,uint32_t rpm,uint32_t ms,float mm,float ref){move_calls++;return 1;}
uint32_t MoveDistanceStart(uint32_t d,uint32_t rpm,float cm){move_calls++;return 1;}
uint32_t MoveStart(uint32_t d,uint32_t rpm,uint32_t ms){move_calls++;return 1;}
float MotionRpmToCmPerSec(float rpm){return rpm*(3.14159265358979323846f*77.0f/600.0f);}
static AppMotionObservation live;
const AppMotionObservation *AppMotion_Live(void){return &live;}
static uint32_t RealTurnStart(uint32_t direction, float angle_deg, uint32_t max_rpm)
{
    uint32_t accepted = 0U;
    if (direction > TURN_CCW || !(angle_deg >= MOTION_TURN_MIN_DEG && angle_deg <= MOTION_TURN_MAX_DEG) ||
        max_rpm < 1U || (max_rpm > MOTION_TURN_MAX_RPM &&
            !(APP_TURN_BRAKE_TRIAL_ACTIVE && s_motion_list_submit &&
              MOTION_ACTION_COUNT==1U && (g_debug_plan_id==DEBUG_PLAN_DEFAULT ||
               AppDebugTurn90Allowed(g_debug_plan_id,max_rpm,(direction==TURN_CCW)?angle_deg:-angle_deg)) &&
              angle_deg==90.0f && max_rpm==APP_TURN_BRAKE_MAX_RPM))) return 0U;
    taskENTER_CRITICAL();
    if (!g_heading_exec_trip &&
        (g_heading_exec_phase == HR_IDLE || g_heading_exec_phase == HR_DONE) &&
        App_MotionAccept())
    {
        HWT101_DiagnosticArm();g_motion_fault_diag_valid=0U;
        s_combined_turn = 1U; s_motion_turn_max_rpm = max_rpm;
        /* 转向不能继承前一平移段的动态距离、速度与截止。 */
        s_combined_distance_mm=s_combined_reference_mm_s=0.0f;
        s_combined_base_rpm=s_combined_run_ms=0U;
        AppHeadingRuntime_ResetSession();
        memset((void *)&g_heading_stop_trace, 0, sizeof(g_heading_stop_trace));
        g_heading_exec_phase = HR_IDLE; g_heading_exec_state = HS_IDLE;
        g_heading_exec_relative_deg = (direction == TURN_CCW) ? angle_deg : -angle_deg;
        g_heading_exec_request = 1U; s_motion_has_action = 1U; accepted = 1U;
    }
    taskEXIT_CRITICAL();
    return accepted;
}
static uint32_t RealPerimeterTurnStart(uint32_t direction, float angle_deg, uint32_t max_rpm)
{
    uint32_t accepted = 0U;
    if (direction > TURN_CCW || !(angle_deg >= MOTION_TURN_MIN_DEG && angle_deg <= MOTION_TURN_MAX_DEG) ||
        max_rpm < 1U || (max_rpm > MOTION_TURN_MAX_RPM &&
            !(APP_TURN_BRAKE_TRIAL_ACTIVE && s_motion_list_submit &&
              MOTION_ACTION_COUNT>=1U && MOTION_ACTION_COUNT<=MOTION_ACTION_CAPACITY && g_debug_plan_id==DEBUG_PLAN_DEFAULT &&
              angle_deg==90.0f && max_rpm==APP_TURN_BRAKE_MAX_RPM))) return 0U;
    taskENTER_CRITICAL();
    if (!g_heading_exec_trip &&
        (g_heading_exec_phase == HR_IDLE || g_heading_exec_phase == HR_DONE) &&
        App_MotionAccept())
    {
        HWT101_DiagnosticArm();g_motion_fault_diag_valid=0U;
        s_combined_turn = 1U; s_motion_turn_max_rpm = max_rpm;
        /* 转向不能继承前一平移段的动态距离、速度与截止。 */
        s_combined_distance_mm=s_combined_reference_mm_s=0.0f;
        s_combined_base_rpm=s_combined_run_ms=0U;
        AppHeadingRuntime_ResetSession();
        memset((void *)&g_heading_stop_trace, 0, sizeof(g_heading_stop_trace));
        g_heading_exec_phase = HR_IDLE; g_heading_exec_state = HS_IDLE;
        g_heading_exec_relative_deg = (direction == TURN_CCW) ? angle_deg : -angle_deg;
        g_heading_exec_request = 1U; s_motion_has_action = 1U; accepted = 1U;
    }
    taskEXIT_CRITICAL();
    return accepted;
}

uint32_t TurnStart(uint32_t d,float a,uint32_t rpm){turn_calls++;return RealTurnStart(d,a,rpm);}
static uint32_t App_PerimeterTurnStart(uint32_t d,float a,uint32_t rpm){perimeter_calls++;return RealPerimeterTurnStart(d,a,rpm);}
static uint32_t App_CombinedLoadAction(uint32_t index)
{
    AppHeadingSource heading;
    const MotionAction *action;
    uint32_t accepted, load_cycle, i;
    float planned_offset=0.0f;
    if (index >= MOTION_ACTION_COUNT || !MOTION_ACTION_COUNT || MOTION_ACTION_COUNT > MOTION_ACTION_CAPACITY ||
        (g_debug_plan_id != DEBUG_PLAN_DEFAULT &&
         !(MOTION_ACTION_COUNT==1U && index==0U &&
           s_motion_actions[0].kind>=MOTION_FORWARD_DISTANCE && s_motion_actions[0].kind<=MOTION_BACKWARD_DISTANCE &&
           AppDebugPlan40Allowed(g_debug_plan_id,s_motion_actions[0].kind-MOTION_FORWARD_DISTANCE,
               s_motion_actions[0].base_rpm,s_motion_actions[0].distance_mm)) &&
         !(MOTION_ACTION_COUNT==1U && index==0U && s_motion_actions[0].kind==MOTION_TURN &&
           !s_motion_actions[0].run_ms && s_motion_actions[0].distance_mm==0.0f &&
           s_motion_actions[0].reference_mm_s==0.0f &&
           AppDebugTurn90Allowed(g_debug_plan_id,s_motion_actions[0].base_rpm,
               s_motion_actions[0].relative_deg)))) return 0U;
    load_cycle = HWT101_TimingBegin();
    /* 目标只从起点和动作表生成，不从上一段实际终点重新设定。 */
    for(i=0U;i<=index;i++) planned_offset=HeadingController_WrapDeg(planned_offset+s_motion_actions[i].relative_deg);
    if(!RouteHeading_Target(&s_route_heading_reference,planned_offset,&s_route_heading_target)) return 0U;
    action = &s_motion_actions[index];
    s_route_heading_turn_active = action->kind == MOTION_TURN && !AppDebugTurn90Plan(g_debug_plan_id);
    g_route_heading_targets[index]=s_route_heading_target;
    (void)App_HeadingSourceRead(&heading);
    g_route_heading_start_yaw[index]=heading.yaw_deg;
    s_motion_list_submit = 1U;
    if(action->reference_mm_s!=0.0f)
        accepted=App_MoveSubmit(action->kind-MOTION_FORWARD_DISTANCE,action->base_rpm,
            action->run_ms,action->distance_mm,action->reference_mm_s);
    else if (action->kind == MOTION_TURN && AppDebugTurn90Plan(g_debug_plan_id))
        accepted = TurnStart((action->relative_deg > 0.0f) ? TURN_CCW : TURN_CW,
            (action->relative_deg > 0.0f) ? action->relative_deg : -action->relative_deg,
            action->base_rpm);
    else if (action->kind == MOTION_TURN)
        accepted = App_PerimeterTurnStart((action->relative_deg > 0.0f) ? TURN_CCW : TURN_CW,
            (action->relative_deg > 0.0f) ? action->relative_deg : -action->relative_deg,
            action->base_rpm ? action->base_rpm : 5U);
    else if (action->kind >= MOTION_FORWARD_DISTANCE && action->kind <= MOTION_BACKWARD_DISTANCE)
        accepted = MoveDistanceStart(action->kind - MOTION_FORWARD_DISTANCE, action->base_rpm, action->distance_mm / 10.0f);
    else accepted = MoveStart(action->kind, action->base_rpm, action->run_ms);
    s_motion_list_submit = 0U;
    HWT101_TimingEnd(2U, load_cycle);
    return accepted;
}
static int32_t App_SerialMilli(float value)
{
    if (!(value >= -2000.0f && value <= 2000.0f)) return (-2147483647L - 1L);
    return (int32_t)(value * 1000.0f);
}
static void make_telemetry(int32_t *values){
 ChassisAutoStatus status={0};
 memset(values,0,64*sizeof(*values));
 values[38]=(g_combined_test.state==0U)?(s_motion_actions[0].kind==MOTION_TURN?15U:s_motion_actions[0].base_rpm):(s_combined_turn?15U:s_combined_base_rpm);
 values[39]=130;
    values[56] = (int32_t)((*AppMotion_Live()->turn) ? MOTION_TURN :
        (((*AppMotion_Live()->distance_mm) > 0.0f) ? (MOTION_FORWARD_DISTANCE + (*AppMotion_Live()->mode)) : (*AppMotion_Live()->mode)));
    if (status.distance_mode && (status.distance_base_rpm >= 3U || status.distance_profile_mode))
        values[38] = (int32_t)status.distance_base_rpm;
    if(AppDebugTurn90Plan(g_debug_plan_id)) {
        values[38]=82;values[39]=130;
        if (g_combined_test.state==0U && !(*AppMotion_Live()->has_action) &&
            (*AppMotion_Live()->action_count)==1U) values[56]=(int32_t)AppMotion_Live()->actions[0].kind;
    }
    values[57] = ((*AppMotion_Live()->turn) || AppDebugTurn90Plan(g_debug_plan_id)) ? (-2147483647L - 1L) :
        App_SerialMilli(MotionRpmToCmPerSec((float)values[38]));
    /* 新候选字段显示参考速度上限，不能把含2RPM余量的基速当成参考速度。 */
    if(APP_ROUTE_SPEED_TRIAL_ACTIVE && g_debug_plan_id==DEBUG_PLAN_DEFAULT && !(*AppMotion_Live()->turn))
        values[57]=App_SerialMilli(((g_combined_test.state==0U) ?
            AppMotion_Live()->actions[0].reference_mm_s : (*AppMotion_Live()->reference_mm_s))/10.0f);
    if (APP_BASELINE40_ACTIVE && (g_debug_plan_id==DEBUG_PLAN_FORWARD210_40 || g_debug_plan_id==DEBUG_PLAN_LEFT210_40 ||
        g_debug_plan_id==DEBUG_PLAN_RIGHT210_40 || g_debug_plan_id==DEBUG_PLAN_BACKWARD210_40 || g_debug_plan_id==DEBUG_PLAN_RIGHT150_40 || g_debug_plan_id==DEBUG_PLAN_RIGHT30_40 ||
        g_debug_plan_id==DEBUG_PLAN_FORWARD150_40 || g_debug_plan_id==DEBUG_PLAN_FORWARD30_40 || g_debug_plan_id==DEBUG_PLAN_BACKWARD150_40 || g_debug_plan_id==DEBUG_PLAN_BACKWARD30_40 || g_debug_plan_id==DEBUG_PLAN_LEFT150_40 || g_debug_plan_id==DEBUG_PLAN_LEFT30_40))
    {
        values[57]=40000; /* 40cm/s reference ceiling, not wheel/ground measurement. */
        if (g_combined_test.state==0U && !(*AppMotion_Live()->has_action) &&
            (*AppMotion_Live()->action_count)==1U) values[56]=(int32_t)AppMotion_Live()->actions[0].kind;
    }

}
static void reset_case(uint32_t id,float yaw){
 memset(s_motion_actions,0,sizeof(s_motion_actions));memset((void*)&g_combined_test,0,sizeof(g_combined_test));
 s_motion_action_count=0;g_debug_plan_id=id;s_combined_turn=mode=s_motion_has_action=s_motion_list_submit=0;
 s_combined_distance_mm=s_combined_reference_mm_s=0;s_combined_base_rpm=s_combined_run_ms=0;
 g_heading_exec_trip=g_heading_exec_phase=g_heading_exec_state=g_heading_exec_request=0;
 s_motion_turn_max_rpm=5;turn_calls=perimeter_calls=move_calls=reset_calls=0;
 memset(&s_route_heading_reference,0,sizeof(s_route_heading_reference));RouteHeading_Init(&s_route_heading_reference,yaw);
 source_yaw=yaw;live.action_count=&s_motion_action_count;live.turn=&s_combined_turn;live.mode=&mode;
 live.has_action=&s_motion_has_action;live.distance_mm=&s_combined_distance_mm;live.reference_mm_s=&s_combined_reference_mm_s;live.actions=s_motion_actions;
}

volatile uint32_t review_assertions,case_count;
volatile float debug_values[8];
volatile int32_t exported_telemetry[3][64];
volatile float exported_sessions[36][7];
#define CK(x) do{review_assertions++;if(!(x))return __LINE__;}while(0)
__attribute__((section("i.test_entry"))) uint32_t test_entry(void){
 uint32_t id,z,dr,step,n,prepared_ok,done,index=0;float starts[3]={37,170,-170};uint32_t steps[3]={10,20,40};
 int32_t values[64];
 reset_case(8,37);CK(AppDebugPlan_Load(8,s_motion_actions,&s_motion_action_count));make_telemetry(values);memcpy((void*)exported_telemetry[0],values,sizeof(values));CK(values[56]==4 && values[57]==40000);
 for(id=20;id<=21;id++){
  reset_case(id,37);CK(AppDebugPlan_Load(id,s_motion_actions,&s_motion_action_count));make_telemetry(values);
  CK(values[56]==3 && values[57]==(-2147483647L-1L) && values[38]==82 && values[39]==130);memcpy((void*)exported_telemetry[id-19],values,sizeof(values));
  s_motion_action_count=2;CK(!App_CombinedLoadAction(0));s_motion_action_count=1;CK(!App_CombinedLoadAction(1));
  s_motion_actions[0].relative_deg=-s_motion_actions[0].relative_deg;CK(!App_CombinedLoadAction(0));s_motion_actions[0].relative_deg=-s_motion_actions[0].relative_deg;
  s_motion_actions[0].relative_deg*=.99f;CK(!App_CombinedLoadAction(0));s_motion_actions[0].relative_deg=id==20?90:-90;
  s_motion_actions[0].base_rpm=81;CK(!App_CombinedLoadAction(0));s_motion_actions[0].base_rpm=82;
  s_motion_actions[0].run_ms=1;CK(!App_CombinedLoadAction(0));s_motion_actions[0].run_ms=0;
  s_motion_actions[0].distance_mm=1;CK(!App_CombinedLoadAction(0));s_motion_actions[0].distance_mm=0;
  s_motion_actions[0].reference_mm_s=1;CK(!App_CombinedLoadAction(0));s_motion_actions[0].reference_mm_s=0;
  s_motion_actions[0].kind=MOTION_FORWARD;CK(!App_CombinedLoadAction(0));s_motion_actions[0].kind=MOTION_TURN;
  CK(!turn_calls && !perimeter_calls && !move_calls);
  for(z=0;z<3;z++)for(dr=0;dr<2;dr++)for(step=0;step<3;step++){
   HeadingSession s_heading_prepared;HeadingSessionInput input={0};HeadingSessionOutput output={0};MotionProfileSample sample;
   AppHeadingRuntimeInputs inlinks={0};const AppHeadingRuntimeInputs *inputs=&inlinks;
   float base=starts[z]+(dr?.01f:0),relative=id==20?90:-90,s_heading_exec_relative;
   reset_case(id,starts[z]);CK(AppDebugPlan_Load(id,s_motion_actions,&s_motion_action_count));CK(App_CombinedLoadAction(0));
   CK(turn_calls==1 && !perimeter_calls && !move_calls && reset_calls==1 && !s_route_heading_turn_active && s_combined_turn && s_motion_has_action && g_heading_exec_request==1);
   CK(g_heading_exec_relative_deg==relative && s_motion_turn_max_rpm==82);
   input.max_rpm=s_motion_turn_max_rpm;input.healthy=input.platform_ok=1;input.sequence=1;input.yaw_deg=base;
   inlinks.route_turn_active=&s_route_heading_turn_active;inlinks.route_target=&s_route_heading_target;
   s_heading_exec_relative=g_heading_exec_relative_deg;HeadingSession_Init(&s_heading_prepared);
            prepared_ok=(*inputs->route_turn_active) ?
                HeadingSession_StartReference(&s_heading_prepared,&input,
                    s_heading_exec_relative,(*inputs->route_target),&output) :
                HeadingSession_Start(&s_heading_prepared,&input,
                    s_heading_exec_relative,&output);

   CK(prepared_ok && s_heading_prepared.brake.profile.valid && fabsf(s_heading_prepared.brake.profile.peak_speed-90)<.001f);
   CK(fabsf(HeadingController_ErrorDeg(output.target_deg,base)-relative)<.001f);
   HeadingSession_ReportBegin(&s_heading_prepared,1,&output);
   input.backend_active=1;for(n=40;n<=200;n+=40){input.now_ms=input.sample_ms=n;input.sequence++;HeadingSession_Step(&s_heading_prepared,&input,&output);CK(output.action!=HS_ACTION_ABORT);}
   debug_values[0]=s_heading_prepared.brake.tick;debug_values[1]=output.command_rpm;debug_values[2]=s_heading_prepared.turn_running_seen;debug_values[3]=s_heading_prepared.state;debug_values[4]=s_heading_prepared.cause;debug_values[5]=s_heading_prepared.turn_trial;
   CK(s_heading_prepared.brake.tick==200 && output.command_rpm==0 && !s_heading_prepared.turn_running_seen);
   input.backend_running=1;done=0;
   for(n=steps[step];n<8000;n+=steps[step]){
    input.now_ms=input.sample_ms=200+n;input.sequence++;
    MotionProfile_Sample(&s_heading_prepared.brake.profile,(n-steps[step])/1000.0f,&sample);
    input.yaw_deg=HeadingController_WrapDeg(base+(relative>0?1:-1)*sample.position_mm);input.rate_dps=(relative>0?1:-1)*sample.speed_mm_s;
    if(n>=400)input.backend_running=0;
    HeadingSession_Step(&s_heading_prepared,&input,&output);CK(output.action!=HS_ACTION_ABORT);
    CK(s_heading_prepared.brake.profile_tick==200+steps[step] && s_heading_prepared.turn_running_seen);
    if(output.action==HS_ACTION_STOP){
     HeadingSession_ReportStop(&s_heading_prepared,1,&output);input.backend_active=0;input.backend_result=1;input.backend_stop_mask=7;
     HeadingSession_Step(&s_heading_prepared,&input,&output);CK(output.action!=HS_ACTION_COMPLETE);
     input.backend_stop_mask=15;HeadingSession_Step(&s_heading_prepared,&input,&output);CK(output.action==HS_ACTION_COMPLETE);done=1;break;
    }
   }
   CK(done);exported_sessions[index][0]=id;exported_sessions[index][1]=base;exported_sessions[index][2]=output.target_deg;
   exported_sessions[index][3]=s_heading_prepared.brake.profile.peak_speed;exported_sessions[index][4]=steps[step];exported_sessions[index][5]=s_heading_prepared.state;exported_sessions[index][6]=n;index++;case_count++;
   g_combined_test.state=3;make_telemetry(values);CK(values[56]==3 && values[57]==(-2147483647L-1L));
  }
 }
 for(id=8;id<=19;id++){
  reset_case(id,37);CK(AppDebugPlan_Load(id,s_motion_actions,&s_motion_action_count));CK(App_CombinedLoadAction(0));CK(move_calls==1 && !turn_calls && !perimeter_calls && !s_route_heading_turn_active);
  reset_case(id,37);CK(AppDebugPlan_Load(id,s_motion_actions,&s_motion_action_count));s_motion_action_count=2;CK(!App_CombinedLoadAction(0));
 }
 reset_case(0,37);s_motion_action_count=1;s_motion_actions[0].kind=MOTION_TURN;s_motion_actions[0].base_rpm=82;s_motion_actions[0].relative_deg=90;
 CK(App_CombinedLoadAction(0));CK(perimeter_calls==1 && !turn_calls && s_route_heading_turn_active);
 reset_case(0,37);s_motion_action_count=1;s_motion_actions[0].kind=MOTION_FORWARD_DISTANCE;s_motion_actions[0].base_rpm=52;s_motion_actions[0].reference_mm_s=200;
 CK(App_CombinedLoadAction(0));CK(move_calls==1 && !turn_calls && !perimeter_calls);
 return 0;
}
