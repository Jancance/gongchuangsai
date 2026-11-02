/* Offline only. Actual backend, route converter and extracted frontend;
 * CAN/VM/RTOS below are stubs. RUN setup is seeded, not hardware validation. */
#include "src/app_entry.c"
#include "src/chassis_auto_test.c"
#include "src/motion_control.h"
#include <math.h>
#include <string.h>
uint32_t test_tick,test_depth,test_primask;
volatile uint32_t g_rtos_fault,g_safety_fault,g_safety_ready,g_can_guard_ready;
volatile uint32_t g_motion_diag_fault_latched,g_motion_auto_stop_state,g_motion_auto_stop_cut_vm;
volatile uint32_t g_wheel_test_active,g_wheel_test_cut_vm,g_can_ctrl_fault_latched;
volatile uint32_t g_can_poll_result[4],g_can_poll_flags[4],g_can_poll_count[4];
volatile TickType_t g_can_poll_tick[4];
static uint32_t checks,first_failure,model_mode,model_step,model_fault,model_delayed,model_count;
static float model_gain,position[4],velocity[4],target[4],history[4][32];
volatile uint32_t failure_context[6];
volatile uint32_t model_results[60][9];
volatile uint32_t parity_outputs[160][4];
#define CHECK(x) do {checks++;if(!(x)&&!first_failure){first_failure=__LINE__;failure_context[0]=model_mode;failure_context[1]=model_fault;failure_context[2]=model_step;failure_context[3]=g_chassis_auto_cause;failure_context[4]=s_distance_fault_reason;failure_context[5]=model_count;}}while(0)
void Power_ReadCommandStatus(PowerCommandStatus *p){p->active=1U;}
uint32_t CAN1_ReadStatus(uint8_t a,CanRxMsg*r){(void)a;r->Data[1]=1;return 1;}
uint32_t CAN1_ReadSpeed(uint8_t a,CanRxMsg*r,int32_t*v)
{
    int32_t logical=(int32_t)(velocity[a-1]>=0 ? velocity[a-1]+0.5f : velocity[a-1]-0.5f);
    (void)r;
    if(model_fault==1U&&a==2U&&model_step>=100U)logical=81;
    if(model_fault==2U&&a==3U&&model_step>=100U)return 0U;
    if(model_fault==5U&&a==4U&&model_step>=100U)logical=-logical;
    *v=g_chassis_forward_direction[a-1] ? -logical : logical;return 1U;
}
uint32_t CAN1_ReadPosition(uint8_t a,CanRxMsg*r,uint8_t*sign,uint32_t*raw)
{
    float driver=position[a-1]*(g_chassis_forward_direction[a-1] ? -1.0f : 1.0f);
    (void)r;
    if(model_fault==3U&&a==1U&&model_step>=100U)return 0U;
    *sign=driver<0;*raw=(uint32_t)(fabsf(driver)*65536.0f/(3.14159265358979323846f*77.0f)+0.5f);return 1U;
}
uint32_t CAN1_SetMotorEnable(uint8_t a,uint8_t b,CanRxMsg*r){(void)a;(void)b;(void)r;return 1U;}
uint32_t CAN1_SetEmmSpeedRaw(uint8_t a,uint8_t direction,uint16_t rpm,uint8_t accel,CanRxMsg*r)
{
    (void)r;CHECK(test_depth==0U);CHECK(rpm<=54U);CHECK(accel==0U);
    target[a-1]=(direction==g_chassis_forward_direction[a-1]) ? (float)rpm : -(float)rpm;
    return 1U;
}
uint32_t CAN1_StopMotor(uint8_t a,CanRxMsg*r)
{
    (void)r;CHECK(test_depth==0U);
    if(model_fault==4U&&a==2U)return 0U;
    target[a-1]=0;return 1U;
}
typedef struct
{
    uint32_t request, phase, trip, backend_session;
    uint32_t run_started, stopping, elapsed_ms, final_valid;
    int32_t correction_rpm;
    float start_deg, target_deg, error_deg, final_deg;
    float rate_dps, peak_error_deg, peak_rate_dps;
    uint32_t sample_count, max_sample_gap_ms;
    /* 已接受的非零纠偏提交样本统计，停车后保留，不代表CAN执行次数。 */
    uint32_t motion_mode; /* 编译选择，仅观察，不用Watch切换方向。 */
    uint32_t correction_nonzero_samples, correction_peak_abs_rpm;
    int32_t correction_last_nonzero_rpm;
} ForwardHoldDebug;
static ForwardHoldDebug g_forward_hold;
static uint32_t s_combined_turn,s_combined_mode,s_combined_base_rpm,s_combined_run_ms;
static float s_combined_distance_mm,s_combined_reference_mm_s;
static uint32_t g_forward_hold_trace_count,g_motion_fault_diag_valid,s_motion_has_action;
static uint32_t s_motion_list_submit,g_debug_plan_id,s_motion_action_count;
static MotionAction s_motion_actions[MOTION_ACTION_CAPACITY];
#define MOTION_ACTION_COUNT s_motion_action_count
#define HR_IDLE 0U
#define HR_DONE 3U
static uint32_t mock_accept;
/* Only HeadingSession initialization is stubbed; TurnStart admission and
 * translation-parameter reset are extracted from the actual main.c. */
static uint32_t s_motion_turn_max_rpm,s_heading_exec,g_heading_stop_trace[4];
static uint32_t g_heading_exec_trip,g_heading_exec_phase,g_heading_exec_state,g_heading_exec_request;
static float g_heading_exec_relative_deg;
#define HS_IDLE 0U
static void HeadingSession_Init(uint32_t *session){*session=0U;}
static uint32_t App_MotionAccept(void){return mock_accept;}
static void HWT101_DiagnosticArm(void){}
static uint32_t HWT101_TimingBegin(void){return 0U;}
static void HWT101_TimingEnd(uint32_t a,uint32_t b){(void)a;(void)b;}
static uint32_t App_MotionPlanValid(void)
{
    uint32_t i, budget = 5000U;
    if (!MOTION_ACTION_COUNT || MOTION_ACTION_COUNT > MOTION_ACTION_CAPACITY) return 0U;
    for (i = 0U; i < MOTION_ACTION_COUNT; i++)
    {
        const MotionAction *action = &s_motion_actions[i];
        if(action->reference_mm_s!=0.0f) {
            if(g_debug_plan_id!=DEBUG_PLAN_DEFAULT || !APP_ROUTE_SPEED_TRIAL_ACTIVE ||
                action->kind<MOTION_FORWARD_DISTANCE || action->kind>MOTION_RIGHT_DISTANCE ||
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
            if (action->base_rpm > 5U || action->run_ms || action->distance_mm != 0.0f ||
                !(((action->relative_deg >= 5.0f) && (action->relative_deg <= 90.0f)) ||
                  ((action->relative_deg <= -5.0f) && (action->relative_deg >= -90.0f)))) return 0U;
            budget += APP_HEADING_RUN_MS + 3500U;
        }
        else if (action->kind >= MOTION_FORWARD_DISTANCE && action->kind <= MOTION_RIGHT_DISTANCE)
        {
            if (!APP_POSITION_DIAGNOSTIC || !Chassis_DistanceSelfTest() ||
                action->base_rpm < 3U || action->run_ms ||
                action->relative_deg != 0.0f ||
                !MotionDistanceAllowed(action->kind-MOTION_FORWARD_DISTANCE,action->base_rpm,action->distance_mm)) return 0U;
            if (action->base_rpm>15U &&
                (MOTION_ACTION_COUNT!=1U || g_debug_plan_id!=DEBUG_PLAN_FORWARD210_20)) return 0U;
            budget += MotionDistanceRunMsFor(action->kind-MOTION_FORWARD_DISTANCE,
                action->base_rpm,action->distance_mm) + 3500U;
        }
        else return 0U;
    }
    return (budget <= 120000U) ? 1U : 0U;
}
static uint32_t App_MoveSubmit(uint32_t direction, uint32_t rpm, uint32_t run_ms, float distance_mm,float route_speed)
{
    ForwardHoldDebug empty = {0};
    uint32_t accepted = 0U;
    if(route_speed!=0.0f) {
        if(!APP_POSITION_DIAGNOSTIC || !s_motion_list_submit || g_debug_plan_id!=DEBUG_PLAN_DEFAULT ||
            !MotionRoute_ShapeAllowed(direction,rpm,run_ms,distance_mm,route_speed) ||
            !Chassis_DistanceSelfTest()) return 0U;
    }
    else if (distance_mm!=0.0f)
    {
        if (!APP_POSITION_DIAGNOSTIC || !MotionDistanceAllowed(direction,rpm,distance_mm) ||
            run_ms!=MotionDistanceRunMsFor(direction,rpm,distance_mm) || !Chassis_DistanceSelfTest()) return 0U;
        if (rpm>15U && (g_debug_plan_id!=DEBUG_PLAN_FORWARD210_20 || !s_motion_list_submit)) return 0U;
    }
    else if (direction > MOVE_RIGHT || rpm < MOTION_MOVE_MIN_RPM ||
        rpm > MOTION_MOVE_MAX_RPM || !run_ms || run_ms > MOTION_MOVE_MAX_MS) return 0U;
    taskENTER_CRITICAL();
    if (!g_forward_hold.trip &&
        (g_forward_hold.phase == HR_IDLE || g_forward_hold.phase == HR_DONE) &&
        App_MotionAccept())
    {
        s_combined_turn = 0U; s_combined_mode = direction;
        s_combined_base_rpm = rpm; s_combined_run_ms = run_ms; s_combined_distance_mm = distance_mm;
        s_combined_reference_mm_s=route_speed;
        empty.motion_mode = direction; g_forward_hold = empty;
        HWT101_DiagnosticArm(); g_motion_fault_diag_valid = 0U;
        g_forward_hold_trace_count = 0U; g_forward_hold.request = 1U;
        s_motion_has_action = 1U; accepted = 1U;
    }
    taskEXIT_CRITICAL();
    return accepted;
}
uint32_t TurnStart(uint32_t direction, float angle_deg, uint32_t max_rpm)
{
    uint32_t accepted = 0U;
    if (direction > TURN_CCW || !(angle_deg >= MOTION_TURN_MIN_DEG && angle_deg <= MOTION_TURN_MAX_DEG) ||
        max_rpm < 1U || max_rpm > MOTION_TURN_MAX_RPM) return 0U;
    taskENTER_CRITICAL();
    if (!g_heading_exec_trip &&
        (g_heading_exec_phase == HR_IDLE || g_heading_exec_phase == HR_DONE) &&
        App_MotionAccept())
    {
        s_combined_turn = 1U; s_motion_turn_max_rpm = max_rpm;
        /* 转向不能继承前一平移段的动态距离、速度与截止。 */
        s_combined_distance_mm=s_combined_reference_mm_s=0.0f;
        s_combined_base_rpm=s_combined_run_ms=0U;
        HeadingSession_Init(&s_heading_exec);
        memset((void *)&g_heading_stop_trace, 0, sizeof(g_heading_stop_trace));
        g_heading_exec_phase = HR_IDLE; g_heading_exec_state = HS_IDLE;
        g_heading_exec_relative_deg = (direction == TURN_CCW) ? angle_deg : -angle_deg;
        g_heading_exec_request = 1U; s_motion_has_action = 1U; accepted = 1U;
    }
    taskEXIT_CRITICAL();
    return accepted;
}
static uint32_t App_CombinedLoadAction(uint32_t index)
{
    const MotionAction *action;
    uint32_t accepted, load_cycle;
    if (index >= MOTION_ACTION_COUNT) return 0U;
    load_cycle = HWT101_TimingBegin();
    action = &s_motion_actions[index]; s_motion_list_submit = 1U;
    if(action->reference_mm_s!=0.0f)
        accepted=App_MoveSubmit(action->kind-MOTION_FORWARD_DISTANCE,action->base_rpm,
            action->run_ms,action->distance_mm,action->reference_mm_s);
    else if (action->kind == MOTION_TURN)
        accepted = TurnStart((action->relative_deg > 0.0f) ? TURN_CCW : TURN_CW,
            (action->relative_deg > 0.0f) ? action->relative_deg : -action->relative_deg,
            action->base_rpm ? action->base_rpm : 5U);
    else if (action->kind >= MOTION_FORWARD_DISTANCE && action->kind <= MOTION_RIGHT_DISTANCE)
        accepted = MoveDistanceStart(action->kind - MOTION_FORWARD_DISTANCE, action->base_rpm, action->distance_mm / 10.0f);
    else accepted = MoveStart(action->kind, action->base_rpm, action->run_ms);
    s_motion_list_submit = 0U;
    HWT101_TimingEnd(2U, load_cycle);
    return accepted;
}
static const uint32_t parity[][7]={
{0U,99U,0U,0U,0U,0U,0U},
{0U,99U,12U,0U,0U,0U,0U},
{0U,99U,50U,0U,0U,0U,0U},
{0U,99U,100U,0U,0U,0U,0U},
{0U,99U,150U,0U,0U,0U,0U},
{0U,99U,200U,0U,0U,0U,0U},
{0U,99U,201U,0U,0U,0U,0U},
{0U,100U,0U,0U,0U,0U,0U},
{0U,100U,12U,0U,0U,0U,0U},
{0U,100U,50U,1U,15U,8965U,40285U},
{0U,100U,100U,1U,27U,8965U,40285U},
{0U,100U,150U,1U,39U,8965U,40285U},
{0U,100U,200U,1U,51U,8965U,40285U},
{0U,100U,201U,0U,0U,0U,0U},
{0U,300U,0U,0U,0U,0U,0U},
{0U,300U,12U,0U,0U,0U,0U},
{0U,300U,50U,1U,15U,12896U,50000U},
{0U,300U,100U,1U,27U,11954U,75438U},
{0U,300U,150U,1U,39U,11954U,75438U},
{0U,300U,200U,1U,51U,11954U,75438U},
{0U,300U,201U,0U,0U,0U,0U},
{0U,2100U,0U,0U,0U,0U,0U},
{0U,2100U,12U,0U,0U,0U,0U},
{0U,2100U,50U,1U,15U,48896U,50000U},
{0U,2100U,100U,1U,27U,30022U,100000U},
{0U,2100U,150U,1U,39U,25147U,150000U},
{0U,2100U,200U,1U,51U,23773U,200000U},
{0U,2100U,201U,0U,0U,0U,0U},
{0U,2101U,0U,0U,0U,0U,0U},
{0U,2101U,12U,0U,0U,0U,0U},
{0U,2101U,50U,0U,0U,0U,0U},
{0U,2101U,100U,0U,0U,0U,0U},
{0U,2101U,150U,0U,0U,0U,0U},
{0U,2101U,200U,0U,0U,0U,0U},
{0U,2101U,201U,0U,0U,0U,0U},
{1U,99U,0U,0U,0U,0U,0U},
{1U,99U,12U,0U,0U,0U,0U},
{1U,99U,50U,0U,0U,0U,0U},
{1U,99U,100U,0U,0U,0U,0U},
{1U,99U,150U,0U,0U,0U,0U},
{1U,99U,200U,0U,0U,0U,0U},
{1U,99U,201U,0U,0U,0U,0U},
{1U,100U,0U,0U,0U,0U,0U},
{1U,100U,12U,0U,0U,0U,0U},
{1U,100U,50U,1U,15U,8918U,40672U},
{1U,100U,100U,1U,27U,8918U,40672U},
{1U,100U,150U,1U,40U,8918U,40672U},
{1U,100U,200U,1U,52U,8918U,40672U},
{1U,100U,201U,0U,0U,0U,0U},
{1U,300U,0U,0U,0U,0U,0U},
{1U,300U,12U,0U,0U,0U,0U},
{1U,300U,50U,1U,15U,12850U,50000U},
{1U,300U,100U,1U,27U,11885U,76095U},
{1U,300U,150U,1U,40U,11885U,76095U},
{1U,300U,200U,1U,52U,11885U,76095U},
{1U,300U,201U,0U,0U,0U,0U},
{1U,2100U,0U,0U,0U,0U,0U},
{1U,2100U,12U,0U,0U,0U,0U},
{1U,2100U,50U,1U,15U,48850U,50000U},
{1U,2100U,100U,1U,27U,29944U,100000U},
{1U,2100U,150U,1U,40U,25039U,150000U},
{1U,2100U,200U,1U,52U,23633U,200000U},
{1U,2100U,201U,0U,0U,0U,0U},
{1U,2101U,0U,0U,0U,0U,0U},
{1U,2101U,12U,0U,0U,0U,0U},
{1U,2101U,50U,0U,0U,0U,0U},
{1U,2101U,100U,0U,0U,0U,0U},
{1U,2101U,150U,0U,0U,0U,0U},
{1U,2101U,200U,0U,0U,0U,0U},
{1U,2101U,201U,0U,0U,0U,0U},
{2U,99U,0U,0U,0U,0U,0U},
{2U,99U,12U,0U,0U,0U,0U},
{2U,99U,50U,0U,0U,0U,0U},
{2U,99U,100U,0U,0U,0U,0U},
{2U,99U,150U,0U,0U,0U,0U},
{2U,99U,200U,0U,0U,0U,0U},
{2U,99U,201U,0U,0U,0U,0U},
{2U,100U,0U,0U,0U,0U,0U},
{2U,100U,12U,0U,0U,0U,0U},
{2U,100U,50U,1U,15U,8918U,40672U},
{2U,100U,100U,1U,27U,8918U,40672U},
{2U,100U,150U,1U,40U,8918U,40672U},
{2U,100U,200U,1U,52U,8918U,40672U},
{2U,100U,201U,0U,0U,0U,0U},
{2U,300U,0U,0U,0U,0U,0U},
{2U,300U,12U,0U,0U,0U,0U},
{2U,300U,50U,1U,15U,12850U,50000U},
{2U,300U,100U,1U,27U,11885U,76095U},
{2U,300U,150U,1U,40U,11885U,76095U},
{2U,300U,200U,1U,52U,11885U,76095U},
{2U,300U,201U,0U,0U,0U,0U},
{2U,2100U,0U,0U,0U,0U,0U},
{2U,2100U,12U,0U,0U,0U,0U},
{2U,2100U,50U,1U,15U,48850U,50000U},
{2U,2100U,100U,1U,27U,29944U,100000U},
{2U,2100U,150U,1U,40U,25039U,150000U},
{2U,2100U,200U,1U,52U,23633U,200000U},
{2U,2100U,201U,0U,0U,0U,0U},
{2U,2101U,0U,0U,0U,0U,0U},
{2U,2101U,12U,0U,0U,0U,0U},
{2U,2101U,50U,0U,0U,0U,0U},
{2U,2101U,100U,0U,0U,0U,0U},
{2U,2101U,150U,0U,0U,0U,0U},
{2U,2101U,200U,0U,0U,0U,0U},
{2U,2101U,201U,0U,0U,0U,0U}
};
#if APP_ROUTE_SPEED_TRIAL_ACTIVE
static void backend_reset(void)
{
    uint32_t i,j;
    test_tick=1000U;test_depth=test_primask=0U;
    g_rtos_fault=g_safety_fault=g_can_ctrl_fault_latched=0U;
    g_safety_ready=g_can_guard_ready=1U;
    g_motion_diag_fault_latched=g_motion_auto_stop_state=g_motion_auto_stop_cut_vm=0U;
    g_wheel_test_active=g_wheel_test_cut_vm=0U;
    for(i=0;i<4;i++){position[i]=velocity[i]=target[i]=0;for(j=0;j<32;j++)history[i][j]=0;}
    ChassisAuto_Init();CHECK(ChassisAuto_BeginSequence());ChassisAuto_SafetyUpdate(1U);
}
static void front_reset(void)
{
    ForwardHoldDebug empty={0};g_forward_hold=empty;mock_accept=1U;
    s_motion_has_action=s_motion_list_submit=0U;g_debug_plan_id=0U;
}
static void model(uint32_t mode,float mm,float speed,uint32_t delayed,uint32_t fault)
{
    MotionProfile profile;uint32_t rpm,run_ms,w,step;
    ChassisAutoPreparedTranslation p;
    model_mode=mode;model_fault=fault;model_delayed=delayed;model_step=0;
    model_gain=delayed ? 0.97f : 1.03f;
    backend_reset();
    CHECK(MotionRoute_Build(mode,mm,speed,&profile,&rpm,&run_ms));
    CHECK(ChassisAuto_PrepareRouteTranslation(mode,rpm,run_ms,mm,speed,&p));
    CHECK(ChassisAuto_BeginPreparedTranslation(&p,g_chassis_auto_session));
    CHECK(s_route_trial&&s_translation_base_rpm==0U&&s_profile_active);
    g_chassis_auto_state=CHASSIS_AUTO_RUN;s_run_started=1U;s_run_tick=s_profile_follow_tick=test_tick;
    s_status_valid=s_speed_valid=s_position_valid=15U;s_endpoints[s_endpoint_index].start_mask=15U;
    for(w=0;w<4;w++){
        s_status[w]=1U;s_status_tick[w]=s_speed_tick[w]=s_position_tick[w]=test_tick;
        s_position.sign[w]=s_position.raw[w]=0U;
        s_distance_seen_ms[w]=test_tick;s_distance_seen_raw[w]=s_distance_seen_sign[w]=0U;
    }
    for(step=0;step<6500U&&g_chassis_auto_active;step++) {
        MotionProfileSample sample;
        model_step=step;test_tick=1000U+step*10U;
        for(w=0;w<4;w++) {
            uint32_t lag=delayed ? 4U*w+4U : 0U;
            float demand=step>lag ? history[w][(step+31U-lag)%32U] : 0.0f;
            velocity[w]+=(demand*model_gain-velocity[w])*(0.01f/(delayed ? 0.08f : 0.02f));
            position[w]+=velocity[w]*(3.14159265358979323846f*77.0f/60.0f)*0.01f;
        }
        if(!(fault==6U&&step>=100U))s_heading_command_tick=test_tick;
        if((fault==4U||fault==7U)&&step==100U)CHECK(ChassisAuto_RequestStop());
        Auto_CheckLocked(test_tick);ChassisAuto_CanStep();
        for(w=0;w<4;w++)history[w][step%32U]=target[w];
        CHECK(test_depth==0U);CHECK(s_translation_base_rpm<=rpm);
        CHECK(MotionProfile_Sample(&profile,step*0.01f,&sample));
        CHECK(sample.speed_mm_s>=-0.001f&&sample.speed_mm_s<=speed+0.01f);
        CHECK(fabsf(sample.acceleration_mm_s2)<=profile.limits.acceleration+0.01f);
        CHECK(fabsf(sample.jerk_mm_s3)<=profile.limits.jerk+0.01f);
    }
    CHECK(step<6500U);
    if(!fault) {CHECK(g_chassis_auto_result==1U);CHECK(s_distance_reached);CHECK(g_chassis_auto_stop_mask[2]==15U);}
    else if(fault==7U) {CHECK(g_chassis_auto_result==1U);CHECK(!s_distance_reached);CHECK(g_chassis_auto_stop_mask[2]==15U);}
    else {CHECK(g_chassis_auto_result==2U);CHECK(g_chassis_auto_cut_vm);CHECK(!s_distance_reached);}
    CHECK(model_count<60U);
    model_results[model_count][0]=mode;model_results[model_count][1]=(uint32_t)mm;
    model_results[model_count][2]=(uint32_t)speed;model_results[model_count][3]=delayed;
    model_results[model_count][4]=fault;model_results[model_count][5]=g_chassis_auto_result;
    model_results[model_count][6]=g_chassis_auto_cause;model_results[model_count][7]=s_distance_fault_reason;
    model_results[model_count][8]=step*10U;model_count++;
}
#endif
uint32_t test_entry(void)
{
    MotionProfile profile;uint32_t rpm=0,run_ms=0;
    CHECK(!MotionRoute_Build(3U,300,200,&profile,&rpm,&run_ms));
#if APP_ROUTE_SPEED_TRIAL_ACTIVE
    uint32_t i,d,s,k,count;ChassisAutoPreparedTranslation p,bad;
    MotionAction actions[8];RouteRow rows[8];RouteCheck report;
    for(i=0;i<sizeof(parity)/sizeof(parity[0]);i++) {
        uint32_t ok=MotionRoute_Build(parity[i][0],(float)parity[i][1],(float)parity[i][2],&profile,&rpm,&run_ms);
        CHECK(ok==parity[i][3]);
        if(ok) {
            CHECK(rpm==parity[i][4]);CHECK(fabsf((float)run_ms-(float)parity[i][5])<=2.0f);
            CHECK(fabsf(profile.peak_speed-(float)parity[i][6]*0.001f)<=0.01f);
            parity_outputs[i][0]=rpm;parity_outputs[i][1]=run_ms;
            parity_outputs[i][2]=(uint32_t)(profile.peak_speed*1000.0f);
            parity_outputs[i][3]=(uint32_t)(profile.duration_s*1000000.0f);
        }
    }
    /* Direction caps must remain independent, including altered-cap builds. */
    for(d=0;d<3;d++) {
        CHECK(!MotionRoute_Build(d,300,MotionRoute_MaxSpeed(d)+1.0f,&profile,&rpm,&run_ms));
        CHECK(!MotionRoute_Build(d,99,100,&profile,&rpm,&run_ms));
        CHECK(!MotionRoute_Build(d,2101,100,&profile,&rpm,&run_ms));
        CHECK(!MotionRoute_Build(d,300,-1,&profile,&rpm,&run_ms));
        CHECK(MotionRoute_Build(d,100,MotionRoute_MaxSpeed(d),&profile,&rpm,&run_ms));
        CHECK(profile.peak_speed<MotionRoute_MaxSpeed(d));
    }
    for(i=0;i<8;i++) {rows[i].kind=MOTION_FORWARD_DISTANCE;rows[i].amount=10;rows[i].speed=10;}
    CHECK(RoutePlan_Convert(rows,7,&g_chassis_geometry,actions,&report));
    CHECK(report.budget_ms<=120000U);
    for(i=0;i<8;i++)rows[i].amount=210;
    CHECK(!RoutePlan_Convert(rows,8,&g_chassis_geometry,actions,&report)&&report.error==ROUTE_BUDGET);
    CHECK(AppDebugPlan_Load(0,actions,&count)&&count==3U);
#if FULL20_TABLE
    CHECK(actions[0].reference_mm_s==200.0f&&actions[1].reference_mm_s==200.0f&&actions[2].reference_mm_s==200.0f);
    CHECK(actions[0].kind==4U&&actions[1].kind==5U&&actions[2].kind==6U);
#else
    CHECK(actions[0].reference_mm_s==50.0f);
#endif
    for(d=0;d<3;d++) {
        backend_reset();CHECK(MotionRoute_Build(d,300,100,&profile,&rpm,&run_ms));
        CHECK(ChassisAuto_PrepareRouteTranslation(d,rpm,run_ms,300,100,&p));
        bad=p;bad.route_reference_mm_s=201;CHECK(!ChassisAuto_BeginPreparedTranslation(&bad,g_chassis_auto_session));
        bad=p;bad.base_rpm++;CHECK(!ChassisAuto_BeginPreparedTranslation(&bad,g_chassis_auto_session));
        bad=p;bad.run_ms++;CHECK(!ChassisAuto_BeginPreparedTranslation(&bad,g_chassis_auto_session));
        bad=p;bad.profile.limits.speed+=1;CHECK(!ChassisAuto_BeginPreparedTranslation(&bad,g_chassis_auto_session));
        CHECK(!ChassisAuto_PrepareTranslation(d,52,run_ms,300,&bad));
        CHECK(!ChassisAuto_PrepareRouteTranslation(d,rpm,run_ms,300,0,&bad));
        CHECK(!ChassisAuto_PrepareRouteTranslation(d,rpm,run_ms,0,100,&bad));
        front_reset();CHECK(!App_MoveSubmit(d,rpm,run_ms,300,100));
        s_motion_list_submit=1;g_debug_plan_id=4;CHECK(!App_MoveSubmit(d,rpm,run_ms,300,100));
        g_debug_plan_id=0;g_forward_hold.trip=1;CHECK(!App_MoveSubmit(d,rpm,run_ms,300,100));
        g_forward_hold.trip=0;mock_accept=0;CHECK(!App_MoveSubmit(d,rpm,run_ms,300,100));
        mock_accept=1;CHECK(App_MoveSubmit(d,rpm,run_ms,300,100));
        CHECK(s_combined_reference_mm_s==100&&s_combined_mode==d&&s_combined_run_ms==run_ms);
        front_reset();s_motion_action_count=1;
        s_motion_actions[0]=(MotionAction){MOTION_FORWARD_DISTANCE+d,rpm,run_ms,0,300,100};
        CHECK(App_MotionPlanValid());CHECK(App_CombinedLoadAction(0));
        CHECK(!s_motion_list_submit&&s_combined_reference_mm_s==100);
    }
    front_reset();g_heading_exec_trip=g_heading_exec_phase=0U;
    s_combined_distance_mm=300.0f;s_combined_reference_mm_s=200.0f;s_combined_run_ms=9000U;
    CHECK(TurnStart(TURN_CCW,90.0f,4U));
    CHECK(s_combined_turn==1U&&s_motion_turn_max_rpm==4U);
    CHECK(s_combined_distance_mm==0.0f&&s_combined_reference_mm_s==0.0f&&s_combined_run_ms==0U);
#if !ALTERED_CAPS && !FULL20_TABLE
    for(d=0;d<3;d++)model(d,100,200,1,0);
    for(d=0;d<3;d++)for(s=0;s<3;s++)for(k=0;k<2;k++)for(i=0;i<2;i++)
        model(d,k ? 2100.0f : 300.0f,100.0f+50.0f*s,i,0);
    for(d=0;d<3;d++)for(k=1;k<=7;k++)model(d,2100,200,1,k);
#endif
#else
    CHECK(!MotionRoute_Build(0,300,200,&profile,&rpm,&run_ms));
    CHECK(!MotionRoute_Build(1,300,200,&profile,&rpm,&run_ms));
    CHECK(!MotionRoute_Build(2,300,200,&profile,&rpm,&run_ms));
#endif
    return first_failure ? 0x80000000U|first_failure : checks;
}

uint32_t MoveStart(uint32_t d,uint32_t r,uint32_t ms){return App_MoveSubmit(d,r,ms,0,0);}
uint32_t MoveDistanceStart(uint32_t d,uint32_t r,float cm){float mm=cm*10;return App_MoveSubmit(d,r,MotionDistanceRunMsFor(d,r,mm),mm,0);}
