#include <stdint.h>
static void CaptureFailure(uint32_t line);
static uint32_t g_vm_test_request,test_power;
/* Offline only. Actual backend, route converter and extracted frontend;
 * CAN/VM/RTOS below are stubs. RUN setup is seeded, not hardware validation. */
#include "src/app_entry.c"
#include "can.h"
uint32_t CAN1_ReadXSystemSnapshot(uint8_t,CanRxMsg*,float*);
uint32_t CAN1_SetXSpeedRaw(uint8_t,uint8_t,uint16_t,uint16_t,CanRxMsg*);
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
#define CHECK(x) do{checks++;if(!(x)&&!first_failure)CaptureFailure(__LINE__);}while(0)
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

uint32_t CAN1_ReadXSystemSnapshot(uint8_t a,CanRxMsg*r,float*v){(void)a;(void)r;*v=0;return 1;}
uint32_t CAN1_SetXSpeedRaw(uint8_t a,uint8_t d,uint16_t v,uint16_t accel,CanRxMsg*r){(void)a;(void)d;(void)v;(void)accel;(void)r;return 1;}
typedef struct
{
    uint32_t request, phase, trip, backend_session;
    uint32_t run_started, stopping, elapsed_ms, final_valid;
    float correction_rpm;
    float start_deg, target_deg, error_deg, final_deg;
    float rate_dps, peak_error_deg, peak_rate_dps;
    uint32_t sample_count, max_sample_gap_ms;
    /* 已接受的非零纠偏提交样本统计，停车后保留，不代表CAN执行次数。 */
    uint32_t motion_mode; /* 编译选择，仅观察，不用Watch切换方向。 */
    uint32_t correction_nonzero_samples;
    float correction_peak_abs_rpm;
    float correction_last_nonzero_rpm;
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
static struct {float yaw_deg,rate_dps;}s_heading_source;
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
static uint32_t App_HeadingRuntimeOwns(const ChassisAutoStatus *p){(void)p;return 1;}
static uint32_t App_ForwardHoldOwns(const ChassisAutoStatus *p){(void)p;return 1;}
static void App_HeadingRuntimeTrip(uint32_t cause){(void)cause;g_heading_exec_trip=1;}
static void App_ForwardHoldTrip(uint32_t cause){(void)cause;g_forward_hold.trip=1;}
static void App_HeadingRuntimeSafety(void){}
static void App_ForwardHoldSafety(void){}
static void segment_done(void){
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

#include <stdio.h>
#include "src/runtime_route.h"
static RuntimeRoute s_runtime_route;
static RuntimeRoutePrepared s_runtime_prepared;
static uint32_t s_route_ack_pending,s_route_ack_id,s_route_ack_op,s_route_ack_result;
static uint32_t s_perimeter_odom_started,s_perimeter_completed_seen;
static float s_perimeter_targets[9][3];
static uint32_t g_perimeter_records[9][8];
static uint32_t g_serial_motion_state,g_serial_motion_result,g_serial_motion_rejected,g_serial_motion_accepted,g_serial_motion_link_lost;
static uint32_t s_serial_token_seq,s_serial_token_tick,s_serial_arm_ms,s_serial_keepalive_ms;
static uint32_t s_serial_tx_seq,s_serial_tx_valid,s_serial_proto_ok;
static uint32_t FrontendRearm(uint32_t session){
 uint32_t ok=ChassisAuto_RearmCompleted(session);
 if(ok){mock_status.state=CHASSIS_AUTO_IDLE;mock_status.result=0;mock_status.cut_vm=0;}
 return ok;}
volatile uint32_t route_context[18];
static void CaptureFailure(uint32_t line){first_failure=line;
 route_context[0]=s_route_ack_result;route_context[1]=g_serial_motion_state;route_context[2]=g_serial_motion_result;
 route_context[3]=s_runtime_route.stage_mask;route_context[4]=s_runtime_route.committed.id;route_context[5]=s_runtime_route.ready;
 route_context[6]=g_combined_test.state;route_context[7]=g_combined_test.completed;route_context[8]=s_runtime_route.committed.count;
 route_context[9]=test_tick;route_context[10]=g_chassis_auto_state;route_context[11]=g_motion_plan_valid;route_context[12]=g_combined_test.cause;route_context[13]=g_forward_hold.phase;route_context[14]=g_heading_exec_phase;route_context[15]=mock_status.result;route_context[16]=s_motion_session_open;route_context[17]=s_serial_start_permit;}

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
                (MOTION_ACTION_COUNT>=1U && MOTION_ACTION_COUNT<=MOTION_ACTION_CAPACITY) && g_debug_plan_id==DEBUG_PLAN_DEFAULT &&
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
static uint32_t App_DebugSelectPlan(uint32_t id)
{
    MotionAction actions[MOTION_ACTION_CAPACITY];
    ChassisAutoStatus status;
    PowerCommandStatus power;
    uint32_t i,count;
    if (!AppDebugPlan_Load(id,actions,&count)) return 0U;
    taskENTER_CRITICAL();
    (void)ChassisAuto_ReadStatus(&status); Power_ReadCommandStatus(&power);
    if (g_combined_test.state || g_combined_test.request || s_motion_session_open ||
        s_motion_has_action || g_forward_hold.phase != HR_IDLE ||
        g_heading_exec_phase != HR_IDLE || g_forward_hold.request || g_heading_exec_request ||
        g_vm_test_request || power.active || status.active || status.result || status.cut_vm)
    { taskEXIT_CRITICAL(); return 0U; }
    for(i=0U;i<count;i++) s_motion_actions[i]=actions[i];
    s_motion_action_count=count; g_debug_plan_id=id;
    g_motion_plan_valid=App_MotionPlanValid();
    taskEXIT_CRITICAL(); return g_motion_plan_valid;
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
    if (index >= MOTION_ACTION_COUNT || !MOTION_ACTION_COUNT || MOTION_ACTION_COUNT > MOTION_ACTION_CAPACITY ||
        g_debug_plan_id != DEBUG_PLAN_DEFAULT) return 0U;
    load_cycle = HWT101_TimingBegin();
    /* 目标只从起点和动作表生成，不从上一段实际终点重新设定。 */
    for(i=0U;i<=index;i++) planned_offset=HeadingController_WrapDeg(planned_offset+s_motion_actions[i].relative_deg);
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
        else if (s_runtime_route.committed.id && s_runtime_route.consumed &&
                 g_combined_test.state==1U && g_combined_test.index<s_runtime_route.committed.count &&
                 (uint32_t)(now-s_combined_stage_ms)>=s_runtime_route.committed.rows[g_combined_test.index].timeout_ms)
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
static uint16_t App_SerialCrc(const char *data, uint32_t length)
{
    uint16_t crc = 0xFFFFU;
    uint32_t i, bit;
    for (i = 0U; i < length; i++)
    {
        crc ^= (uint16_t)((uint16_t)(uint8_t)data[i] << 8);
        for (bit = 0U; bit < 8U; bit++)
            crc = (uint16_t)((crc & 0x8000U) ? ((crc << 1) ^ 0x1021U) : (crc << 1));
    }
    return crc;
}
static uint32_t App_SerialUInt(const char **text, uint32_t *out)
{
    uint32_t n = 0U, digit;
    const char *p = *text;
    if (*p < '0' || *p > '9') return 0U;
    while (*p >= '0' && *p <= '9')
    {
        digit = (uint32_t)(*p - '0');
        if (n > (0xFFFFFFFFU - digit) / 10U) return 0U;
        n = n * 10U + digit; p++;
    }
    if (*p != ',') return 0U;
    *text = p + 1; *out = n; return 1U;
}
static uint32_t App_SerialIdleReady(uint32_t now)
{
#if APP_X_OFFLINE_ONLY
    (void)now;return 0U;
#else
    ChassisAutoStatus status;
    PowerCommandStatus power;
    (void)ChassisAuto_ReadStatus(&status);
    Power_ReadCommandStatus(&power);
#if APP_TEST_MODE == 3U
    if ((g_combined_test.state != 0U) || (g_combined_test.request != 0U) ||
        (g_motion_plan_valid != 1U)) return 0U;
#endif
    return ((g_forward_hold.phase == HR_IDLE) && (g_forward_hold.request == 0U) &&
        (g_vm_test_request == 0U) && (power.active == 0U) &&
        (status.active == 0U) && (status.result == 0U) && (status.cut_vm == 0U) &&
        (g_translation_hold_selftest == 1U) && (s_serial_proto_ok == 1U) &&
        App_HeadingRuntimeSource(now) && App_HeadingRuntimeExclusive() &&
        (g_can_ctrl_fault_latched == 0U) &&
        (s_heading_source.rate_dps >= -3.0f && s_heading_source.rate_dps <= 3.0f)) ? 1U : 0U;
#endif
}
static uint32_t App_SerialCompletedStopped(void)
{
    ChassisAutoStatus status;
    PowerCommandStatus power;
    if (g_serial_motion_state != 3U || g_serial_motion_result != 2U) return 0U;
#if APP_TEST_MODE == 3U
    if (g_combined_test.state != 3U || g_combined_test.cause ||
        g_combined_test.completed != MOTION_ACTION_COUNT) return 0U;
#else
    if (g_forward_hold.phase != HR_DONE || g_forward_hold.trip) return 0U;
#endif
    Power_ReadCommandStatus(&power); (void)ChassisAuto_ReadStatus(&status);
    return (!g_vm_test_request && !power.active && !status.active &&
        status.result == 1U && !status.cause && status.rotation_stop_mask == 15U &&
        !g_can_ctrl_fault_latched && !g_rtos_fault && !g_safety_fault) ? 1U : 0U;
}
static void App_SerialReject(uint32_t reason)
{ g_serial_motion_rejected++; g_serial_motion_result = reason; }
static uint32_t App_RuntimeRouteEditable(uint32_t now)
{
    if(g_serial_motion_state==0U && App_SerialIdleReady(now))return 1U;
    return App_SerialCompletedStopped() && !s_motion_session_open &&
        !g_heading_exec_trip && !g_forward_hold.trip &&
        App_HeadingRuntimeSource(now) && App_HeadingRuntimeExclusive() &&
        !g_serial_motion_link_lost && !g_can_ctrl_fault_latched;
}
static uint32_t App_RuntimeRouteInstall(const RuntimeRoutePrepared *p,uint32_t now)
{
    uint32_t i;ChassisAutoStatus status;
    ForwardHoldDebug forward={0};CombinedTestDebug combined={0};
    if(!App_RuntimeRouteEditable(now))return 0U;
    if(g_combined_test.state==3U) {
        (void)ChassisAuto_ReadStatus(&status);
        if(!FrontendRearm(status.session))return 0U;
        g_combined_test=combined;g_forward_hold=forward;
        g_heading_exec_phase=HR_IDLE;g_heading_exec_state=HS_IDLE;g_heading_exec_request=0U;
        HeadingSession_Init(&s_heading_exec);
        s_motion_has_action=s_combined_turn=s_combined_powered_segment=0U;
        s_combined_keep_vm=s_serial_start_permit=0U;
        s_serial_token_seq=s_serial_token_tick=s_serial_arm_ms=s_serial_keepalive_ms=0U;
        g_serial_motion_state=g_serial_motion_result=0U;
    }
    for(i=0U;i<p->count;i++)s_motion_actions[i]=p->actions[i];
    s_motion_action_count=p->count;g_debug_plan_id=DEBUG_PLAN_DEFAULT;
    /* 转换器已校验类型、速度、距离、时限和总预算，此处只做有界复制。 */
    g_motion_plan_valid=1U;
    s_route_heading_reference=(RouteHeadingReference){0};
    s_route_heading_turn_active=0U;
    s_perimeter_odom_started=s_perimeter_completed_seen=0U;
    memset((void*)g_perimeter_records,0,sizeof(g_perimeter_records));
    memcpy(s_perimeter_targets,p->local_targets,sizeof(s_perimeter_targets));
    RuntimeRoute_Install(&s_runtime_route,p);
    return 1U;
}
static void App_RuntimeRouteCommand(const char *line)
{
    RuntimeRouteMessage m;uint32_t result,now=(uint32_t)xTaskGetTickCount();
    result=RuntimeRoute_Decode(line,&m);
    if(result==RQ_OK) {
        taskENTER_CRITICAL();
        if(m.op==RQ_GO) {
            if(m.seq!=s_serial_token_seq || m.tick!=s_serial_token_tick)result=RQ_STALE;
            else if((g_serial_motion_state==2U || App_SerialCompletedStopped()) &&
                    s_runtime_route.consumed && s_runtime_route.committed.id==m.id &&
                    s_runtime_route.committed.digest==m.digest)result=RQ_DUPLICATE;
            else if(g_serial_motion_state!=1U || (uint32_t)(now-s_serial_arm_ms)>=2000U ||
                    !App_SerialIdleReady(now))result=RQ_BUSY;
            else {
                result=RuntimeRoute_Start(&s_runtime_route,m.id,m.digest);
                if(result==RQ_OK) {
                    s_serial_start_permit=1U;s_serial_keepalive_ms=now;
                    g_combined_test.request=1U;
                    g_serial_motion_state=2U;g_serial_motion_result=2U;
                    g_serial_motion_accepted++;
                }
            }
        } else if(!s_serial_tx_valid || !s_serial_tx_seq ||
                  (uint32_t)(s_serial_tx_seq-1U-m.seq)>5U || (uint32_t)(now-m.tick)>=500U)
            result=RQ_STALE;
        else if(!App_RuntimeRouteEditable(now))result=RQ_BUSY;
        else if(m.op!=RQ_COMMIT)result=RuntimeRoute_Stage(&s_runtime_route,&m,now);
        taskEXIT_CRITICAL();
        if(m.op==RQ_COMMIT && result==RQ_OK) {
            /* 浮点轨迹转换在临界区外完成，不阻塞独立安全任务。 */
            result=RuntimeRoute_Prepare(&s_runtime_route,&m,now,&s_runtime_prepared);
            if(result==RQ_OK) {
                taskENTER_CRITICAL();now=(uint32_t)xTaskGetTickCount();
                if(!s_runtime_route.staging || m.id!=s_runtime_route.stage_id ||
                   (uint32_t)(now-s_runtime_route.stage_ms)>=RQ_UPLOAD_MS)result=RQ_EXPIRED;
                else if(!App_RuntimeRouteInstall(&s_runtime_prepared,now))result=RQ_BUSY;
                taskEXIT_CRITICAL();
            }
        }
    }
    taskENTER_CRITICAL();
    s_runtime_route.last_result=result;
    s_route_ack_id=m.id;s_route_ack_op=m.op;s_route_ack_result=result;s_route_ack_pending=1U;
    taskEXIT_CRITICAL();
}
static void App_SerialMotionCommand(char *line)
{
    if(strncmp(line,"RQ1,",4U)==0) { App_RuntimeRouteCommand(line);return; }
    char *last = strrchr(line, ',');
    const char *p;
    uint32_t seq, tick, crc = 0U, i, digit, now;
    if ((strncmp(line, "RC1,", 4U) != 0) || (last == NULL) || strlen(last + 1) != 4U)
    { App_SerialReject(4U); return; }
    for (i = 1U; i <= 4U; i++)
    {
        char c = last[i];
        if (c >= '0' && c <= '9') digit = (uint32_t)(c - '0');
        else if (c >= 'A' && c <= 'F') digit = (uint32_t)(c - 'A') + 10U;
        else { App_SerialReject(4U); return; }
        crc = (crc << 4) | digit;
    }
    if ((uint32_t)App_SerialCrc(line, (uint32_t)(last - line + 1)) != crc)
    { App_SerialReject(4U); return; }
    *last = '\0'; p = line + 4;
    if (!App_SerialUInt(&p, &seq) || !App_SerialUInt(&p, &tick))
    { App_SerialReject(4U); return; }
    taskENTER_CRITICAL();
    now = (uint32_t)xTaskGetTickCount();
    if (strcmp(p, "STOP") == 0)
    {
        s_serial_start_permit = 0U;
#if APP_TEST_MODE == 3U
        App_CombinedTestStop(1U);
#else
        if ((g_forward_hold.phase == HR_WAIT) || (g_forward_hold.phase == HR_RUN) ||
            (g_forward_hold.request == 1U)) g_forward_hold.request = 2U;
#endif
        g_serial_motion_state = 3U; g_serial_motion_result = 3U;
        g_serial_motion_accepted++;
    }
#if APP_TEST_MODE == 3U
    else if (strncmp(p,"PLAN",4U)==0 && p[4]>='0' && p[4]<=('0'+DEBUG_PLAN_MAX) && p[5]=='\0')
    {
        if (!s_serial_tx_seq || (uint32_t)(s_serial_tx_seq-1U-seq)>5U ||
            (uint32_t)(now-tick)>=500U) App_SerialReject(5U);
        else if (g_serial_motion_state || s_runtime_route.staging || s_runtime_route.committed.id || !App_SerialIdleReady(now) ||
                 !App_DebugSelectPlan((uint32_t)(p[4]-'0'))) App_SerialReject(6U);
        else g_serial_motion_accepted++;
    }
#endif
    else if (strcmp(p, "ARM") == 0)
    {
        if ((s_serial_tx_seq == 0U) || ((uint32_t)(s_serial_tx_seq - 1U - seq) > 5U) ||
            ((uint32_t)(now - tick) >= 500U)) App_SerialReject(5U);
        else if ((g_serial_motion_state != 0U) || !App_SerialIdleReady(now) ||
                 !s_runtime_route.ready || s_runtime_route.staging || s_runtime_route.consumed) App_SerialReject(6U);
        else
        {
            s_serial_token_seq = seq; s_serial_token_tick = tick; s_serial_arm_ms = now;
            g_serial_motion_state = 1U; g_serial_motion_result = 1U;
            g_serial_motion_accepted++;
        }
    }
    else if ((seq != s_serial_token_seq) || (tick != s_serial_token_tick)) App_SerialReject(5U);
    else if (strcmp(p, "START") == 0)
    {
#if APP_TEST_MODE == 3U
        App_SerialReject(6U);
#else
        if (g_serial_motion_state == 2U) { /* 重复帧只确认，不重新启动或续期。 */ }
        else if ((g_serial_motion_state != 1U) || ((uint32_t)(now - s_serial_arm_ms) >= 2000U) ||
            !App_SerialIdleReady(now)) App_SerialReject(6U);
        else
        {
            s_serial_start_permit = 1U; s_serial_keepalive_ms = now;
#if APP_TEST_MODE == 3U
            g_combined_test.request = 1U;
#else
            g_forward_hold.request = 1U;
#endif
            g_serial_motion_state = 2U; g_serial_motion_result = 2U;
            g_serial_motion_accepted++;
        }
#endif
    }
    else if (strcmp(p, "KEEP") == 0)
    {
        if (g_serial_motion_state == 2U)
        { s_serial_keepalive_ms = now; g_serial_motion_accepted++; }
        else if (App_SerialCompletedStopped())
        { g_serial_motion_accepted++; } /* 同令牌已在上方核对；终态只确认，不续期。 */
        else App_SerialReject(6U);
    }
    else App_SerialReject(4U);
    taskEXIT_CRITICAL();
}
static void App_SerialMotionByte(uint8_t data)
{
    static char line[RQ_LINE_MAX];
    static uint32_t used, invalid, last_ms;
    uint32_t now = (uint32_t)xTaskGetTickCount();
    if ((uint32_t)(now - last_ms) >= 200U) { used = 0U; invalid = 0U; }
    last_ms = now;
    if (data == '\r') return;
    if (data == '\n')
    {
        if (!invalid && used) { line[used] = '\0'; App_SerialMotionCommand(line); }
        used = invalid = 0U; return;
    }
    if (invalid) return;
    if (data < 32U || data > 126U || used >= sizeof(line) - 1U)
    { invalid = 1U; App_SerialReject(4U); return; }
    line[used++] = (char)data;
}
static void App_SerialMotionSafety(void)
{
    uint32_t now;
    taskENTER_CRITICAL();
    now = (uint32_t)xTaskGetTickCount();
    if ((g_serial_motion_state == 1U) && ((uint32_t)(now - s_serial_arm_ms) >= 2000U))
    { g_serial_motion_state = 0U; g_serial_motion_result = 7U; s_serial_start_permit = 0U; }
    if (g_serial_motion_state == 2U)
    {
#if APP_TEST_MODE == 3U
        if (g_combined_test.state >= 3U) g_serial_motion_state = 3U;
        else if ((uint32_t)(now - s_serial_keepalive_ms) >= 1000U)
        {
            s_serial_start_permit = 0U; g_serial_motion_link_lost = 1U;
            g_serial_motion_result = 8U; g_serial_motion_state = 3U;
            App_CombinedTestStop(1U);
        }
#else
        if ((g_forward_hold.phase == HR_IDLE) && (g_forward_hold.request == 2U))
        {
            s_serial_start_permit = 0U;
            g_serial_motion_state = 3U; g_serial_motion_result = 3U;
            App_ForwardHoldTrip(1U);
        }
        else if (g_forward_hold.phase >= HR_DONE) g_serial_motion_state = 3U;
        else if ((uint32_t)(now - s_serial_keepalive_ms) >= 1000U)
        {
            s_serial_start_permit = 0U; g_serial_motion_link_lost = 1U;
            g_serial_motion_result = 8U; g_serial_motion_state = 3U;
            App_ForwardHoldTrip(1U);
        }
#endif
    }
    taskEXIT_CRITICAL();
}
static void reset_scheduler(void){
 static const RouteRow rows[]={APP_ROUTE_ROWS(ROUTE_ROW)};RouteCheck rc;
 memset(&g_combined_test,0,sizeof(g_combined_test));memset(&g_forward_hold,0,sizeof(g_forward_hold));
 memset(&mock_status,0,sizeof(mock_status));
 memset(&s_route_heading_reference,0,sizeof(s_route_heading_reference));
 s_heading_source.yaw_deg=173.0f;
 test_tick=1000;g_vm_test_request=test_power=0;g_can_ctrl_fault_latched=0;
 g_heading_exec_trip=g_heading_exec_phase=g_heading_exec_request=0;
 s_motion_session_open=s_motion_has_action=s_combined_turn=0;
 step_calls=finish_calls=abort_calls=0;source_ok=g_safety_ready=g_can_guard_ready=1;
 CHECK(RoutePlan_Convert(rows,8,&g_chassis_geometry,s_motion_actions,&rc));s_motion_action_count=8;
 g_debug_plan_id=0;g_motion_plan_valid=App_MotionPlanValid();CHECK(g_motion_plan_valid);
 g_translation_hold_selftest=1;s_serial_start_permit=1;
}
static void serial_line(const char *prefix)
{
 char frame[RQ_LINE_MAX];uint32_t n=(uint32_t)strlen(prefix);
 (void)snprintf(frame,sizeof(frame),"%s%04X",prefix,RuntimeRoute_Crc(prefix,n));
 App_SerialMotionCommand(frame);
}
static void upload_route(uint32_t id,uint32_t count)
{
 static const RuntimeRouteRow all[8]={
 {4,18144,200,35000},{6,18000,200,35000},{3,-90000,30000,35000},{3,-90000,30000,35000},
 {4,18144,200,35000},{3,90000,30000,35000},{3,90000,30000,35000},{5,18000,200,35000}};
 char text[150];uint32_t i;
 (void)snprintf(text,sizeof(text),"RQ1,99,%lu,%lu,BEGIN,%lu,",(unsigned long)test_tick,(unsigned long)id,(unsigned long)count);
 serial_line(text);CHECK(s_route_ack_result==RQ_OK);CHECK(!g_vm_test_request);
 serial_line(text);CHECK(s_route_ack_result==RQ_DUPLICATE);
 for(i=0;i<count;i++) {
  (void)snprintf(text,sizeof(text),"RQ1,99,%lu,%lu,ROW,%lu,%lu,%ld,%lu,%lu,",
   (unsigned long)test_tick,(unsigned long)id,(unsigned long)i,(unsigned long)all[i].kind,
   (long)all[i].amount,(unsigned long)all[i].speed,(unsigned long)all[i].timeout_ms);
  serial_line(text);CHECK(s_route_ack_result==RQ_OK);
  serial_line(text);CHECK(s_route_ack_result==RQ_DUPLICATE);
 }
 (void)snprintf(text,sizeof(text),"RQ1,99,%lu,%lu,COMMIT,%lu,",(unsigned long)test_tick,
  (unsigned long)id,(unsigned long)RuntimeRoute_Digest(id,count,all));
 serial_line(text);CHECK(s_route_ack_result==RQ_OK);CHECK(s_runtime_route.ready);
 CHECK(MOTION_ACTION_COUNT==count);CHECK(App_MotionPlanValid());CHECK(!g_vm_test_request&&!test_power);
 serial_line(text);CHECK(s_route_ack_result==RQ_DUPLICATE);CHECK(s_runtime_route.ready);
}
static void setup_route(void)
{
 reset_scheduler();ChassisAuto_Init();RuntimeRoute_Init(&s_runtime_route);
 g_serial_motion_state=g_serial_motion_result=g_serial_motion_rejected=g_serial_motion_accepted=0;
 g_serial_motion_link_lost=0;s_serial_tx_seq=100;s_serial_tx_valid=s_serial_proto_ok=1;
 s_heading_source.rate_dps=0;s_route_ack_pending=0;
}
static void start_route(void)
{
 char text[150];
 (void)snprintf(text,sizeof(text),"RC1,99,%lu,ARM,",(unsigned long)test_tick);
 serial_line(text);CHECK(g_serial_motion_state==1);
 (void)snprintf(text,sizeof(text),"RQ1,99,%lu,%lu,GO,%lu,",(unsigned long)test_tick,
  (unsigned long)s_runtime_route.committed.id,(unsigned long)s_runtime_route.committed.digest);
 serial_line(text);CHECK(s_route_ack_result==RQ_OK);CHECK(g_combined_test.request==1);
 serial_line(text);CHECK(s_route_ack_result==RQ_DUPLICATE);CHECK(g_combined_test.request==1);
}
static void finish_route(uint32_t count)
{
 uint32_t i;
 for(i=0;i<count;i++) {
  if(i)test_tick+=1001;
  App_CombinedTestStep();
  if(i==count-1 && g_combined_test.state==1)App_CombinedTestStep();
  CHECK(g_combined_test.completed==i+1);
 }
 App_SerialMotionSafety();CHECK(g_combined_test.state==3);CHECK(g_serial_motion_state==3);
 CHECK(!g_vm_test_request&&!test_power);CHECK(App_SerialCompletedStopped());
}
uint32_t test_entry(void)
{
 RuntimeRoute q;RuntimeRouteMessage m={0};RuntimeRoutePrepared prepared;
 uint32_t i,n,old;char text[150];
 for(n=1;n<=8;n++) {
  setup_route();upload_route(1,n);start_route();finish_route(n);
  CHECK(step_calls==n);CHECK(finish_calls==1);
 }
 /* A new route after normal completion reuses only successful backend state. */
 g_chassis_auto_session=mock_status.session;g_chassis_auto_state=CHASSIS_AUTO_DONE;
 g_chassis_auto_result=1;g_chassis_auto_cause=0;g_chassis_auto_segment=2;
 g_chassis_auto_stop_mask[2]=15;
 for(i=0;i<4;i++)g_chassis_auto_driver_rpm[i]=0;
 upload_route(2,2);CHECK(g_chassis_auto_state==CHASSIS_AUTO_IDLE);start_route();finish_route(2);
 CHECK(step_calls==10);CHECK(finish_calls==2);
 /* Fault/cancel states cannot be reset by upload. */
 old=s_runtime_route.committed.id;g_combined_test.state=4;g_combined_test.cause=5;
 g_chassis_auto_state=CHASSIS_AUTO_FAULT;g_chassis_auto_result=2;g_chassis_auto_cause=8;
 (void)snprintf(text,sizeof(text),"RQ1,99,%lu,3,BEGIN,1,",(unsigned long)test_tick);
 serial_line(text);CHECK(s_route_ack_result==RQ_BUSY);CHECK(s_runtime_route.committed.id==old);
 CHECK(g_chassis_auto_state==CHASSIS_AUTO_FAULT);CHECK(!ChassisAuto_RearmCompleted(g_chassis_auto_session));
 setup_route();upload_route(1,2);start_route();
 serial_line("RQ1,99,1000,2,BEGIN,1,");CHECK(s_route_ack_result==RQ_BUSY);
 CHECK(s_runtime_route.committed.id==1&&s_runtime_route.committed.count==2);
 /* Keepalive loss cancels the whole route and never advances another segment. */
 test_tick+=1000;App_SerialMotionSafety();CHECK(g_combined_test.state==5);
 CHECK(g_serial_motion_link_lost);CHECK(!g_vm_test_request&&!test_power);
 setup_route();upload_route(1,1);start_route();
 test_tick=s_serial_keepalive_ms+35000;App_CombinedTestStep();
 /* The independent route deadline also cuts a blocked first segment. */
 g_combined_test.state=1;g_combined_test.index=0;s_combined_stage_ms=test_tick-35000;
 App_CombinedTestSafety();CHECK(g_combined_test.state==4&&g_combined_test.cause==2);
 /* CRC, parser bounds, stage timeout, missing/changed/duplicate rows. */
 RuntimeRoute_Init(&q);m.id=1;m.op=RQ_BEGIN;m.count=1;
 CHECK(RuntimeRoute_Stage(&q,&m,0)==RQ_OK);m.op=RQ_COMMIT;
 CHECK(RuntimeRoute_Prepare(&q,&m,1,&prepared)==RQ_MISSING);
 m.op=RQ_ROW;m.index=0;m.row=(RuntimeRouteRow){4,1000,200,35000};
 CHECK(RuntimeRoute_Stage(&q,&m,2)==RQ_OK);CHECK(RuntimeRoute_Stage(&q,&m,3)==RQ_DUPLICATE);
 m.row.amount=2000;CHECK(RuntimeRoute_Stage(&q,&m,4)==RQ_ID);
 m.op=RQ_COMMIT;m.digest=1;CHECK(RuntimeRoute_Prepare(&q,&m,5,&prepared)==RQ_DIGEST);
 m.digest=RuntimeRoute_Digest(1,1,q.staged);CHECK(RuntimeRoute_Prepare(&q,&m,10000,&prepared)==RQ_EXPIRED);
 RuntimeRoute_Tick(&q,10000);CHECK(!q.ready&&!q.staging);
 m.op=RQ_BEGIN;CHECK(RuntimeRoute_Stage(&q,&m,10001)==RQ_ID);
 m.id=2;CHECK(RuntimeRoute_Stage(&q,&m,10001)==RQ_OK);
 m.op=RQ_ROW;m.index=1;CHECK(RuntimeRoute_Stage(&q,&m,10002)==RQ_RANGE);
 m.index=0;m.row.kind=7;CHECK(RuntimeRoute_Stage(&q,&m,10002)==RQ_RANGE);
 CHECK(RuntimeRoute_Decode("RQ1,1,1,1,BEGIN,1,0000",&m)==RQ_FORMAT);
 CHECK(RuntimeRoute_Decode("RQ1,42949672960,1,1,BEGIN,1,0000",&m)==RQ_FORMAT);
 /* Stale command tokens and unknown serial START cannot launch a route. */
 setup_route();serial_line("RQ1,99,0,1,BEGIN,1,");CHECK(s_route_ack_result==RQ_STALE);
 serial_line("RC1,99,1000,ARM,");CHECK(g_serial_motion_state==0);
 CHECK(!g_combined_test.request&&!g_vm_test_request);
 upload_route(1,1);serial_line("RC1,99,1000,ARM,");CHECK(g_serial_motion_state==1);
 serial_line("RC1,99,1000,START,");CHECK(!g_combined_test.request&&!g_vm_test_request);
 serial_line("RQ1,99,1000,1,GO,1,");CHECK(s_route_ack_result==RQ_ID);CHECK(!g_combined_test.request);
 return first_failure?(0x80000000U|first_failure):checks;
}
