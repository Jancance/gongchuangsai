#include <stdint.h>
static uint32_t g_vm_test_request,test_power;
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
volatile float trim_model_positions[60][3];
volatile uint32_t parity_outputs[160][4];
#define CHECK(x) do {checks++;if(!(x)&&!first_failure){first_failure=__LINE__;failure_context[0]=model_mode;failure_context[1]=model_fault;failure_context[2]=model_step;failure_context[3]=g_chassis_auto_cause;failure_context[4]=s_distance_fault_reason;failure_context[5]=model_count;}}while(0)
void Power_ReadCommandStatus(PowerCommandStatus *p){p->active=test_power;}
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
typedef struct
{
    uint32_t request, state, index, cause, completed;
    uint32_t phase[MOTION_ACTION_CAPACITY], session[MOTION_ACTION_CAPACITY], elapsed_ms[MOTION_ACTION_CAPACITY];
    float final_yaw[MOTION_ACTION_CAPACITY], final_error[MOTION_ACTION_CAPACITY];
} CombinedTestDebug;
#include "src/route_plan.h"
static ForwardHoldDebug g_forward_hold;
static CombinedTestDebug g_combined_test;
static MotionAction s_motion_actions[MOTION_ACTION_CAPACITY];
static uint32_t s_motion_action_count,g_debug_plan_id,s_motion_list_submit,s_motion_has_action;
#define MOTION_ACTION_COUNT s_motion_action_count
static uint32_t s_combined_turn,s_combined_mode,s_combined_base_rpm,s_combined_run_ms;
static float s_combined_distance_mm,s_combined_reference_mm_s;
static uint32_t s_motion_session_open,s_combined_keep_vm,s_combined_begin_ms,s_combined_stage_ms,s_combined_gap_ms,s_combined_end_ms,s_combined_powered_segment;
static uint32_t g_heading_exec_trip,g_heading_exec_phase,g_heading_exec_request,g_heading_exec_state,s_heading_exec,g_heading_stop_trace[4];
static float g_heading_exec_relative_deg,g_heading_exec_error_deg;
static uint32_t s_motion_turn_max_rpm,g_translation_hold_selftest,g_motion_plan_valid,s_serial_start_permit;
static uint32_t g_forward_hold_trace_count,g_motion_fault_diag_valid,g_can_jog_request;
static struct {float yaw_deg;}s_heading_source;
static ChassisAutoStatus mock_status;
static uint32_t step_calls,finish_calls,abort_calls,source_ok;
volatile uint32_t scheduler_context[8];
#define HR_IDLE 0U
#define HR_WAIT 1U
#define HR_RUN 2U
#define HR_DONE 3U
#define HR_FAULT 4U
#define HR_CANCELLED 5U
#define HS_IDLE 0U
#define COMP_ENTRY_IDLE 0U
static void HeadingSession_Init(uint32_t *s){*s=0;}
static void HWT101_DiagnosticArm(void){}
static uint32_t HWT101_TimingBegin(void){return 0;}
static void HWT101_TimingEnd(uint32_t a,uint32_t b){(void)a;(void)b;}
static void App_TimingRecordCompleted(uint32_t i,uint32_t session,uint32_t now){(void)i;(void)session;(void)now;}
static void MockReadStatus(ChassisAutoStatus *p){*p=mock_status;}
static void MockAbort(void){abort_calls++;}
#define ChassisAuto_ReadStatus MockReadStatus
#define ChassisAuto_Abort MockAbort
static void Power_Disable(void){test_power=0;}
static uint32_t App_MotionAccept(void){g_vm_test_request=test_power=1;return 1;}
static uint32_t App_HeadingRuntimeSource(uint32_t now){(void)now;return source_ok;}
static uint32_t App_HeadingRuntimeExclusive(void){return 1;}
static uint32_t App_HeadingRuntimeOwns(const ChassisAutoStatus *p){return p->session==1U;}
static uint32_t App_ForwardHoldOwns(const ChassisAutoStatus *p){(void)p;return 1;}
static void App_HeadingRuntimeTrip(uint32_t cause){(void)cause;g_heading_exec_trip=1;}
static void App_ForwardHoldTrip(uint32_t cause){(void)cause;g_forward_hold.trip=1;}
static void App_HeadingRuntimeSafety(void){}
static void App_ForwardHoldSafety(void){}
static uint32_t test_hold;
static void segment_done(void){ if(test_hold)return;
 step_calls++;mock_status.active=0;mock_status.state=CHASSIS_AUTO_DONE;
 mock_status.result=1;mock_status.cause=0;mock_status.rotation_stop_mask=15;mock_status.cut_vm=0;
 mock_status.session++;g_heading_exec_phase=g_forward_hold.phase=HR_DONE;
}
static void App_HeadingRuntimeStep(void){if(g_heading_exec_phase!=HR_DONE)segment_done();}
static void App_ForwardHoldStep(void){if(g_forward_hold.phase!=HR_DONE)segment_done();}
static uint32_t MotionGetState(void){return MOTION_BUSY;}
static uint32_t MotionFinish(void){if(!g_vm_test_request)return 0;finish_calls++;g_vm_test_request=test_power=0;return 1;}
#define App_StageDistanceMm() s_combined_distance_mm
#define App_StageRunMs() s_combined_run_ms
static uint32_t App_MoveSubmit(uint32_t,uint32_t,uint32_t,float,float);
static uint32_t MoveStart(uint32_t d,uint32_t rpm,uint32_t ms){return App_MoveSubmit(d,rpm,ms,0,0);}
static uint32_t MoveDistanceStart(uint32_t d,uint32_t rpm,float cm){return App_MoveSubmit(d,rpm,MotionDistanceRunMsFor(d,rpm,cm*10),cm*10,0);}

static uint32_t idle_status_calls,idle_position_calls;
static void App_CanPollStep(void){idle_status_calls++;}
static void MockIdleOdometryPoll(void){idle_position_calls++;}
#define ChassisAuto_OdometryPoll MockIdleOdometryPoll

#include "src/route_heading_reference.h"
static RouteHeadingReference s_route_heading_reference;
static float s_route_heading_target;
static uint32_t s_route_heading_turn_active;
static volatile float g_route_heading_targets[8],g_route_heading_start_yaw[8];
static void App_ComboIdlePoll(void)
{
    static uint32_t position_turn;
    uint32_t combined = (g_combined_test.state==1U || g_combined_test.state==2U);
    if(combined && position_turn) {
        position_turn=0U;
        ChassisAuto_OdometryPoll();
    } else {
        position_turn=combined;
        App_CanPollStep();
    }
}
static uint32_t App_MotionPlanValid(void)
{
    uint32_t i, budget = 5000U;
    if (!MOTION_ACTION_COUNT || MOTION_ACTION_COUNT > MOTION_ACTION_CAPACITY) return 0U;
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
                (MOTION_ACTION_COUNT==1U || MOTION_ACTION_COUNT==8U) && g_debug_plan_id==DEBUG_PLAN_DEFAULT &&
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
                (MOTION_ACTION_COUNT!=1U || g_debug_plan_id!=DEBUG_PLAN_FORWARD210_20)) return 0U;
            budget += MotionDistanceRunMsFor(action->kind-MOTION_FORWARD_DISTANCE,
                action->base_rpm,action->distance_mm) + 3500U;
        }
        else return 0U;
    }
    return (budget <= APP_PERIMETER_GROUP_MS) ? 1U : 0U;
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
static uint32_t App_PerimeterTurnStart(uint32_t direction, float angle_deg, uint32_t max_rpm)
{
    uint32_t accepted = 0U;
    if (direction > TURN_CCW || !(angle_deg >= MOTION_TURN_MIN_DEG && angle_deg <= MOTION_TURN_MAX_DEG) ||
        max_rpm < 1U || (max_rpm > MOTION_TURN_MAX_RPM &&
            !(APP_TURN_BRAKE_TRIAL_ACTIVE && s_motion_list_submit &&
              MOTION_ACTION_COUNT==1U && g_debug_plan_id==DEBUG_PLAN_DEFAULT &&
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
    uint32_t accepted, load_cycle, i;
    float planned_offset=0.0f;
    if (index >= MOTION_ACTION_COUNT || MOTION_ACTION_COUNT != 1U ||
        g_debug_plan_id != DEBUG_PLAN_DEFAULT) return 0U;
    load_cycle = HWT101_TimingBegin();
    /* 目标只从起点和动作表生成，不从上一段实际终点重新设定。 */
    for(i=0U;i<=index;i++) planned_offset += s_motion_actions[i].relative_deg;
    if(!RouteHeading_Target(&s_route_heading_reference,planned_offset,&s_route_heading_target)) return 0U;
    action = &s_motion_actions[index];
    s_route_heading_turn_active = action->kind == MOTION_TURN;
    g_route_heading_targets[index]=s_route_heading_target;
    g_route_heading_start_yaw[index]=s_heading_source.yaw_deg;
    s_motion_list_submit = 1U;
    if(action->reference_mm_s!=0.0f)
        accepted=App_MoveSubmit(action->kind-MOTION_FORWARD_DISTANCE,action->base_rpm,
            action->run_ms,action->distance_mm,action->reference_mm_s);
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
static void App_CombinedTestStop(uint32_t cause)
{
    /* STOP是幂等的；已完成/失败不覆盖首因，不清原单段trip。 */
    if (g_combined_test.state >= 3U)
    { g_vm_test_request = 0U; Power_Disable(); return; }
    s_motion_session_open = s_combined_keep_vm = 0U;
    s_combined_end_ms = (uint32_t)xTaskGetTickCount();
    g_combined_test.cause = cause;
    g_combined_test.state = (cause == 1U) ? 5U : 4U;
    g_forward_hold.request = 0U;
    g_heading_exec_request = 0U;
    if (s_combined_turn) App_HeadingRuntimeTrip(1U);
    else App_ForwardHoldTrip(1U);
    ChassisAuto_Abort();
    g_vm_test_request = 0U;
    Power_Disable();
}
static void App_CombinedTestSafety(void)
{
    uint32_t now = (uint32_t)xTaskGetTickCount();
    taskENTER_CRITICAL();
    if ((g_combined_test.state == 1U) || (g_combined_test.state == 2U))
    {
        if (g_combined_test.request == 2U) App_CombinedTestStop(1U);
        else if ((uint32_t)(now - s_combined_begin_ms) >= APP_PERIMETER_GROUP_MS)
            App_CombinedTestStop(2U);
        else if (!App_HeadingRuntimeSource(now) || !App_HeadingRuntimeExclusive() ||
                 g_can_ctrl_fault_latched) App_CombinedTestStop(3U);
        else if ((g_combined_test.state == 2U) &&
                 ((g_vm_test_request != 1U) || (g_safety_ready != 1U) ||
                  (g_can_guard_ready != 1U))) App_CombinedTestStop(3U);
        else if (g_combined_test.state == 1U)
        {
            if (s_combined_turn) App_HeadingRuntimeSafety();
            else App_ForwardHoldSafety();
        }
    }
    else if (s_motion_session_open)
    {
        if (!App_HeadingRuntimeSource(now) || !App_HeadingRuntimeExclusive() ||
            g_can_ctrl_fault_latched) App_CombinedTestStop(3U);
        else if (MotionGetState() == MOTION_BUSY)
        {
            if (s_combined_turn) App_HeadingRuntimeSafety();
            else App_ForwardHoldSafety();
        }
        else if (g_vm_test_request != 1U || g_can_guard_ready != 1U)
            App_CombinedTestStop(3U);
    }
    else { g_vm_test_request = 0U; }
    taskEXIT_CRITICAL();
}
static void App_CombinedTestStep(void)
{
    ChassisAutoStatus status;
    PowerCommandStatus power;
    uint32_t now = (uint32_t)xTaskGetTickCount(), request, phase, index;
    taskENTER_CRITICAL();
    request = g_combined_test.request;
    g_combined_test.request = 0U;
    if (APP_IS_COMPETITION && !AppCompetitionEntry_MotionAllowed())
    {
        s_serial_start_permit=0U;
        g_forward_hold.request=g_heading_exec_request=0U;
        g_chassis_auto_request=g_can_jog_request=0U;
        g_vm_test_request=0U;
        taskEXIT_CRITICAL(); return;
    }
    if (request == 2U && (g_combined_test.state < 3U))
    { App_CombinedTestStop(1U); taskEXIT_CRITICAL(); return; }
    if (g_combined_test.state == 0U && s_motion_session_open)
    {
        /* 独立API不启动动作表，仍由唯一CAN任务服务。 */
        if (MotionGetState() == MOTION_BUSY)
        {
            if ((uint32_t)(now - s_combined_stage_ms) >=
                ((App_StageDistanceMm()>0.0f) ? (App_StageRunMs()+5000U) : 35000U))
            { App_CombinedTestStop(2U); taskEXIT_CRITICAL(); return; }
            taskEXIT_CRITICAL();
            if (s_combined_turn) App_HeadingRuntimeStep();
            else App_ForwardHoldStep();
            return;
        }
        if (MotionGetState() == MOTION_FAULT || MotionGetState() == MOTION_CANCELLED)
            App_CombinedTestStop(5U);
        taskEXIT_CRITICAL(); return;
    }
    if (g_combined_test.state == 0U)
    {
        /* 其他单段Watch入口在本模式不参与调度。整组每次上电只运行一次。 */
        g_forward_hold.request = 0U; g_heading_exec_request = 0U;
        if (request != 1U) { taskEXIT_CRITICAL(); return; }
        if (s_serial_start_permit != 1U)
        { App_CombinedTestStop(3U); taskEXIT_CRITICAL(); return; }
        s_serial_start_permit = 0U;
        (void)ChassisAuto_ReadStatus(&status); Power_ReadCommandStatus(&power);
        if (status.active || status.result || status.cut_vm || power.active ||
            g_vm_test_request || !App_HeadingRuntimeSource(now) ||
            !App_HeadingRuntimeExclusive() || g_can_ctrl_fault_latched ||
            (g_translation_hold_selftest != 1U) || (g_motion_plan_valid != 1U) ||
            (g_forward_hold.phase != HR_IDLE) || (g_heading_exec_phase != HR_IDLE))
        { App_CombinedTestStop(3U); taskEXIT_CRITICAL(); return; }
        if(!RouteHeading_Init(&s_route_heading_reference,s_heading_source.yaw_deg))
        { App_CombinedTestStop(3U);taskEXIT_CRITICAL();return; }
        s_combined_powered_segment = 0U;
        s_combined_begin_ms = s_combined_stage_ms = now;
        g_combined_test.index = 0U;
        if (!App_CombinedLoadAction(0U))
        { App_CombinedTestStop(3U); taskEXIT_CRITICAL(); return; }
        g_combined_test.state = 1U;
    }
    if ((g_combined_test.state < 3U) && (g_combined_test.index >= MOTION_ACTION_COUNT))
    { App_CombinedTestStop(4U); taskEXIT_CRITICAL(); return; }
    if (g_combined_test.state >= 3U)
    { g_forward_hold.request = g_heading_exec_request = 0U;
      g_vm_test_request = 0U; taskEXIT_CRITICAL(); return; }
    if (g_combined_test.state == 2U)
    {
        /* 保留上一段诊断，仅本组成功且仍上电的停稳会话允许释放。 */
        if ((uint32_t)(now - s_combined_gap_ms) < 1000U)
        { taskEXIT_CRITICAL(); return; }
        Power_ReadCommandStatus(&power); (void)ChassisAuto_ReadStatus(&status);
        if ((g_vm_test_request != 1U) || (power.active != 1U) ||
            status.session != g_combined_test.session[g_combined_test.index])
        { App_CombinedTestStop(4U); taskEXIT_CRITICAL(); return; }
        g_combined_test.index++;
        index = g_combined_test.index;
        s_combined_powered_segment = 1U;
        s_combined_stage_ms = now;
        if (!App_CombinedLoadAction(index))
        { App_CombinedTestStop(4U); taskEXIT_CRITICAL(); return; }
        g_combined_test.state = 1U;
    }
    if (g_combined_test.state == 1U)
    {
        if ((uint32_t)(now - s_combined_stage_ms) >=
                ((App_StageDistanceMm()>0.0f) ? (App_StageRunMs()+5000U) : 35000U))
        { App_CombinedTestStop(2U); taskEXIT_CRITICAL(); return; }
        /* 单段执行可能等待CAN，必须先退出临界区；Safety仍可抢占取消。 */
        taskEXIT_CRITICAL();
        if (s_combined_turn) App_HeadingRuntimeStep();
        else App_ForwardHoldStep();
        taskENTER_CRITICAL();
        if (g_combined_test.state >= 3U)
        { taskEXIT_CRITICAL(); return; }
        phase = s_combined_turn ? g_heading_exec_phase : g_forward_hold.phase;
        if (phase == HR_FAULT || phase == HR_CANCELLED)
        { App_CombinedTestStop(5U); taskEXIT_CRITICAL(); return; }
        if (phase == HR_DONE)
        {
            (void)ChassisAuto_ReadStatus(&status); Power_ReadCommandStatus(&power);
            /* 正常DONE也会置cut_vm请求断电，此处等待关电而非判故障。 */
            if (status.active || status.state != CHASSIS_AUTO_DONE ||
                status.result != 1U || status.cause || status.rotation_stop_mask != 15U ||
                !(s_combined_turn ? App_HeadingRuntimeOwns(&status) : App_ForwardHoldOwns(&status)))
            { App_CombinedTestStop(4U); taskEXIT_CRITICAL(); return; }
            if ((g_combined_test.index + 1U) == MOTION_ACTION_COUNT)
            {
                s_motion_list_submit = 1U;
                (void)MotionFinish();
                s_motion_list_submit = 0U;
            }
            /* 等安全任务确实发出关闭命令，不把VM请求0当作PE2已关。 */
            if (((g_combined_test.index + 1U) < MOTION_ACTION_COUNT) ?
                ((g_vm_test_request == 1U) && (power.active == 1U) && !status.cut_vm) :
                (!g_vm_test_request && !power.active))
            {
                index = g_combined_test.index;
                g_combined_test.phase[index] = phase;
                g_combined_test.session[index] = status.session;
                g_combined_test.elapsed_ms[index] = now - s_combined_stage_ms;
                g_combined_test.final_yaw[index] = s_heading_source.yaw_deg;
                g_combined_test.final_error[index] = s_combined_turn ?
                    g_heading_exec_error_deg : g_forward_hold.error_deg;
#if APP_SERIAL_MOTION_CONTROL
                if (!s_combined_turn) App_TimingRecordCompleted(index, status.session, now);
#endif
                g_combined_test.completed++;
                s_combined_gap_ms = now;
                g_combined_test.state = ((index + 1U) == MOTION_ACTION_COUNT) ? 3U : 2U;
                if ((index + 1U) == MOTION_ACTION_COUNT) s_combined_end_ms = now;
            }
        }
    }
    taskEXIT_CRITICAL();
}
static void reset_scheduler(void){
 static const RouteRow rows[]={APP_ROUTE_ROWS(ROUTE_ROW)};RouteCheck rc;
 memset(&g_combined_test,0,sizeof(g_combined_test));memset(&g_forward_hold,0,sizeof(g_forward_hold));
 memset(&mock_status,0,sizeof(mock_status));
 memset(&s_route_heading_reference,0,sizeof(s_route_heading_reference));
 s_heading_source.yaw_deg=173.0f;
 test_hold=0;test_tick=1000;g_vm_test_request=test_power=0;g_can_ctrl_fault_latched=0;
 g_heading_exec_trip=g_heading_exec_phase=g_heading_exec_request=0;
 s_motion_session_open=s_motion_has_action=s_combined_turn=0;
 step_calls=finish_calls=abort_calls=0;source_ok=g_safety_ready=g_can_guard_ready=1;
 CHECK(RoutePlan_Convert(rows,1,&g_chassis_geometry,s_motion_actions,&rc));s_motion_action_count=1;
 g_debug_plan_id=0;g_motion_plan_valid=App_MotionPlanValid();CHECK(g_motion_plan_valid);
 g_translation_hold_selftest=1;s_serial_start_permit=1;
}
uint32_t test_entry(void){
 uint32_t i;g_combined_test.state=0;
 App_ComboIdlePoll();CHECK(idle_status_calls==1 && idle_position_calls==0);
 g_combined_test.state=1;
 for(i=0;i<40;i++){uint32_t before=idle_status_calls+idle_position_calls;App_ComboIdlePoll();CHECK(idle_status_calls+idle_position_calls==before+1);}
 CHECK(idle_status_calls==21 && idle_position_calls==20);
 g_combined_test.state=2;for(i=0;i<40;i++)App_ComboIdlePoll();CHECK(idle_status_calls==41 && idle_position_calls==40);
 g_combined_test.state=4;App_ComboIdlePoll();CHECK(idle_status_calls==42 && idle_position_calls==40);
 reset_scheduler();g_combined_test.request=1;
 for(i=0;i<1;i++){
  if(i)test_tick+=1001;
  App_CombinedTestStep();
  if(i==0U && g_combined_test.state==1U)App_CombinedTestStep();
  if(g_combined_test.completed!=i+1 && !first_failure) {
   scheduler_context[0]=i;scheduler_context[1]=g_combined_test.state;scheduler_context[2]=g_combined_test.cause;
   scheduler_context[3]=g_combined_test.completed;scheduler_context[4]=step_calls;
   scheduler_context[5]=g_forward_hold.phase;scheduler_context[6]=g_heading_exec_phase;scheduler_context[7]=g_debug_plan_id;
  }
  CHECK(g_combined_test.completed==i+1);CHECK(g_combined_test.index==i);
  CHECK(!test_depth);CHECK(!g_combined_test.cause);
  {static const float offsets[1]={90};
   CHECK(s_route_heading_reference.origin_deg==173.0f);
   CHECK(fabsf(HeadingController_ErrorDeg(s_route_heading_target,173.0f+offsets[i]))<0.001f);
   CHECK(s_route_heading_turn_active==(s_motion_actions[i].kind==MOTION_TURN));
   s_heading_source.yaw_deg-=0.5f; /* Fresh measurements must not rebase the next target. */
  }
  if(s_motion_actions[i].kind==MOTION_TURN) {
   CHECK(s_motion_turn_max_rpm==27);CHECK(s_combined_distance_mm==0);CHECK(s_combined_reference_mm_s==0);
  }else CHECK(s_combined_mode==s_motion_actions[i].kind-MOTION_FORWARD_DISTANCE);
  if(i<0U){CHECK(g_combined_test.state==2);App_CombinedTestStep();CHECK(g_combined_test.completed==i+1);}
 }
 CHECK(g_combined_test.state==3);CHECK(step_calls==1);CHECK(finish_calls==1);CHECK(!g_vm_test_request&&!test_power);
 App_CombinedTestStep();CHECK(step_calls==1);
 reset_scheduler();test_hold=1;g_combined_test.request=1;App_CombinedTestStep();g_combined_test.request=2;
 App_CombinedTestSafety();CHECK(g_combined_test.state==5);CHECK(!g_vm_test_request&&!test_power);CHECK(abort_calls);
 reset_scheduler();test_hold=1;g_combined_test.request=1;App_CombinedTestStep();test_tick=s_combined_begin_ms+APP_PERIMETER_GROUP_MS;
 App_CombinedTestSafety();CHECK(g_combined_test.state==4&&g_combined_test.cause==2);
 reset_scheduler();test_hold=1;g_combined_test.request=1;App_CombinedTestStep();source_ok=0;
 App_CombinedTestSafety();CHECK(g_combined_test.state==4&&g_combined_test.cause==3);
 reset_scheduler();test_hold=1;g_combined_test.request=1;App_CombinedTestStep();mock_status.session++;test_hold=0;
 test_tick+=1001;App_CombinedTestStep();CHECK(g_combined_test.state==4&&g_combined_test.cause==4);
 reset_scheduler();s_motion_actions[0].base_rpm=28;CHECK(!App_MotionPlanValid());
 return first_failure?(0x80000000U|first_failure):checks;
}
