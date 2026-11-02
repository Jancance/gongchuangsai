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

#include "src/heading_session.h"
#include "src/hwt101.h"
#define HR_IDLE 0U
#define HR_WAIT 1U
#define HR_RUN 2U
#define HR_DONE 3U
#define HR_FAULT 4U
#define HR_CANCELLED 5U
#define HEADING_SOURCE_FRESH_MS 100U
#define App_StageStartVm() 0U
#define App_StageDoneVm() 0U
typedef struct {uint32_t sequence,sample_ms,angle_ms,rate_ms,healthy,valid_mask;float yaw_deg,rate_dps;} AppHeadingSource;
static AppHeadingSource s_heading_source;
static uint32_t s_heading_source_ready,g_heading_exec_request,g_heading_exec_phase,g_heading_exec_trip;
static uint32_t g_heading_exec_state,g_heading_exec_cause,g_heading_exec_action,g_heading_exec_backend_session;
static int32_t g_heading_exec_rpm;
static float g_heading_exec_target_deg,g_heading_exec_error_deg,g_heading_exec_relative_deg;
static float s_heading_exec_relative;
static uint32_t g_vm_test_request,s_heading_exec_wait_ms,s_motion_turn_max_rpm;
volatile uint32_t g_hwt_uart_errors,g_hwt_overflows,g_hwt_checksum_errors,g_hwt_update_tick;
volatile uint32_t g_hwt_version_result,g_hwt_valid_mask,g_hwt_angle_age_ms,g_hwt_rate_age_ms;
volatile uint32_t g_hwt_cal_active,g_hwt_cal_request,g_hwt_cal_end_tx_result,g_hwt_version_state,g_hwt_version_request,g_hwt_a7_request;
volatile float g_hwt_yaw_deg,g_hwt_wz_dps;
static HeadingSession s_heading_exec,s_heading_prepared;
static ChassisAutoPreparedHeading s_heading_backend_prepared;
static uint32_t g_heading_prepare_cycles,g_heading_prepare_max_cycles;
typedef struct {uint32_t captured;} HeadingStopAngleSample;
static struct {HeadingStopAngleSample start,zero_request,reached,completed;uint32_t started,zero_submit_accepted;float target_deg;} g_heading_stop_trace;
static uint32_t mode,prepare_calls,test_can_steps,mock_exclusive,mock_power,mock_clock;
static void App_HeadingSourceObserve(void){}
static void App_HeadingStopObserve(uint32_t now){(void)now;}
static void App_HeadingStopCapture(HeadingStopAngleSample *p,uint32_t now){(void)now;p->captured=1;}
static uint32_t App_HeadingRuntimeExclusive(void){return mock_exclusive;}
static uint32_t App_CanJogReady(void){return g_safety_ready&&g_can_guard_ready;}
static uint32_t App_HeadingPowerReady(uint32_t now){(void)now;return mock_power;}
static uint32_t App_HeadingRuntimeOwns(const ChassisAutoStatus *p){return p->session==g_heading_exec_backend_session&&p->heading_mode==1&&p->segment==2;}
static uint32_t App_HeadingRuntimeSource(uint32_t now);
static void App_MotionFaultCapture(uint32_t cause);
static void App_HeadingRuntimeTrip(uint32_t cause);
#define MOTION_FAULT_DIAG_FIELDS 38U
static volatile uint32_t g_motion_fault_diag[38],g_motion_fault_diag_valid;
static uint32_t s_motion_fault_timing[HWT_TIMING_FIELDS];
static struct {uint32_t phase;} g_forward_hold;
static uint32_t g_heading_fault_phase,g_heading_fault_source_flags,g_heading_fault_sample_age_ms;
static uint32_t g_heading_fault_angle_age_ms,g_heading_fault_rate_age_ms,g_heading_fault_hwt_update_age_ms,g_heading_fault_tick_ms;
void HWT101_ReadDiagnostic(HwtFaultDiagnostic *p){HwtFaultDiagnostic empty={0};*p=empty;p->arm_sequence=4;p->uart_ore=g_hwt_uart_errors;}
void HWT101_ReadTiming(uint32_t *p,uint32_t now){uint32_t i;for(i=0;i<HWT_TIMING_FIELDS;i++)p[i]=now+i;}
uint32_t HWT101_TimingBegin(void){return ++mock_clock;}
void HWT101_TimingEnd(uint32_t scope,uint32_t start){(void)scope;(void)start;CHECK(test_depth>0);}
void Test_PrepareHook(void){
 CHECK(test_depth==0);prepare_calls++;
 if(mode==1)g_heading_exec_request=2;
 if(mode==2)s_heading_source.healthy=0;
 if(mode==3)g_hwt_uart_errors++;
 if(mode==4)test_tick+=101;
 if(mode==5)g_vm_test_request=0;
 if(mode==6)g_chassis_auto_session++;
 if(mode==7)App_HeadingRuntimeTrip(3);
 if(mode==8)mock_exclusive=0;
 if(mode==9)mock_power=0;
}
static __inline uint32_t App_HeadingFrameAge(const AppHeadingSource *s,uint32_t now,uint32_t bit)
{ return (s->valid_mask & bit) ? now-((bit==1U)?s->angle_ms:s->rate_ms) : 0xFFFFFFFFU; }

static void App_HeadingSourcePublish(void)
{
    AppHeadingSource value = {0};
    uint32_t now;
    taskENTER_CRITICAL();
    now = (uint32_t)xTaskGetTickCount();
    value.sample_ms = now;
    /* 用真实接收帧龄期还原时间，不能用本次发布时刻冒充新帧。 */
    value.valid_mask=g_hwt_valid_mask & 3U;
    if(g_hwt_angle_age_ms==0xFFFFFFFFU)value.valid_mask &= ~1U;
    if(g_hwt_rate_age_ms==0xFFFFFFFFU)value.valid_mask &= ~2U;
    value.angle_ms=(value.valid_mask & 1U) ? g_hwt_update_tick-g_hwt_angle_age_ms : 0U;
    value.rate_ms=(value.valid_mask & 2U) ? g_hwt_update_tick-g_hwt_rate_age_ms : 0U;
    value.yaw_deg = g_hwt_yaw_deg;
    value.rate_dps = g_hwt_wz_dps;
    value.healthy = ((g_hwt_version_result == 1U) &&
        (value.valid_mask == 3U) &&
        (g_hwt_angle_age_ms < HEADING_SOURCE_FRESH_MS) &&
        (g_hwt_rate_age_ms < HEADING_SOURCE_FRESH_MS) &&
        ((uint32_t)(now - g_hwt_update_tick) < HEADING_SOURCE_FRESH_MS) &&
        ((uint32_t)(App_HeadingFrameAge(&value,now,1U)) < HEADING_SOURCE_FRESH_MS) &&
        ((uint32_t)(App_HeadingFrameAge(&value,now,2U)) < HEADING_SOURCE_FRESH_MS) &&
        (value.yaw_deg >= -180.0f && value.yaw_deg <= 180.0f) &&
        (value.rate_dps >= -2000.0f && value.rate_dps <= 2000.0f) &&
        (g_hwt_cal_active == 0U) && (g_hwt_cal_request == 0U) &&
        (g_hwt_cal_end_tx_result != 2U) &&
        (g_hwt_version_state == 0U) && (g_hwt_version_request == 0U) &&
        (g_hwt_a7_request == 0U)) ? 1U : 0U;
    /* 序号只在完整发布时递增；读者不修改它，允许uint32回绕。 */
    value.sequence = s_heading_source.sequence + 1U;
    s_heading_source = value;
    s_heading_source_ready = 1U;
    taskEXIT_CRITICAL();
}
static uint32_t App_HeadingRuntimeSource(uint32_t now)
{
    return ((s_heading_source_ready == 1U) && (s_heading_source.healthy == 1U) &&
        ((uint32_t)(now - s_heading_source.sample_ms) < HS_SOURCE_MS) &&
        ((uint32_t)(App_HeadingFrameAge(&s_heading_source,now,1U)) < HS_SOURCE_MS) &&
        ((uint32_t)(App_HeadingFrameAge(&s_heading_source,now,2U)) < HS_SOURCE_MS) &&
        (g_hwt_cal_active == 0U) && (g_hwt_cal_request == 0U) &&
        (g_hwt_cal_end_tx_result != 2U) &&
        (g_hwt_version_state == 0U) && (g_hwt_version_request == 0U) &&
        (g_hwt_a7_request == 0U)) ? 1U : 0U;
}
static void App_MotionFaultCapture(uint32_t cause)
{
    HwtFaultDiagnostic hwt;
    ChassisAutoFaultDiagnostic backend;
    ChassisAutoStatus status;
    uint32_t now = (uint32_t)xTaskGetTickCount();
    if(g_motion_fault_diag_valid)return;
    HWT101_ReadDiagnostic(&hwt); (void)ChassisAuto_ReadStatus(&status);
    g_motion_fault_diag[0] = now;
    g_motion_fault_diag[1] = hwt.arm_sequence;
    g_motion_fault_diag[2] = cause;
    g_motion_fault_diag[3] = (g_heading_exec_phase==HR_WAIT || g_heading_exec_phase==HR_RUN) ?
        g_heading_exec_phase : g_forward_hold.phase;
    g_motion_fault_diag[4] = status.state;
    g_motion_fault_diag[5] = status.cause;
    g_motion_fault_diag[6] = status.session;
    g_motion_fault_diag[7] = App_HeadingRuntimeSource(now);
    g_motion_fault_diag[8] = App_HeadingFrameAge(&s_heading_source,now,1U);
    g_motion_fault_diag[9] = App_HeadingFrameAge(&s_heading_source,now,2U);
    g_motion_fault_diag[10] = g_hwt_uart_errors;
    g_motion_fault_diag[11] = g_hwt_overflows;
    g_motion_fault_diag[12] = g_hwt_checksum_errors;
    g_motion_fault_diag[13] = hwt.base_uart_errors;
    g_motion_fault_diag[14] = hwt.base_overflows;
    g_motion_fault_diag[15] = hwt.base_checksum_errors;
    g_motion_fault_diag[16] = hwt.uart_ore;
    g_motion_fault_diag[17] = hwt.uart_fe;
    g_motion_fault_diag[18] = hwt.uart_ne;
    g_motion_fault_diag[19] = hwt.uart_pe;
    g_motion_fault_diag[20] = hwt.last_uart_sr;
    g_motion_fault_diag[21] = hwt.event_tick;
    g_motion_fault_diag[22] = hwt.event_reason;
    g_motion_fault_diag[23] = hwt.event_sr;
    g_motion_fault_diag[24] = hwt.event_uart_errors;
    g_motion_fault_diag[25] = hwt.event_overflows;
    g_motion_fault_diag[26] = hwt.event_checksum_errors;
    g_motion_fault_diag[27] = hwt.event_seen;
    g_motion_fault_diag[28] = now - s_heading_source.sample_ms;
    ChassisAuto_ReadFaultDiagnostic(&backend);
    g_motion_fault_diag[29] = backend.reason;
    g_motion_fault_diag[30] = backend.state;
    g_motion_fault_diag[31] = backend.io_result;
    g_motion_fault_diag[32] = backend.address;
    g_motion_fault_diag[33] = backend.command_age_ms;
    g_motion_fault_diag[34] = backend.status_mask;
    g_motion_fault_diag[35] = backend.speed_mask;
    g_motion_fault_diag[36] = backend.feedback_age_ms;
    g_motion_fault_diag[37] = backend.tick_ms;
    HWT101_ReadTiming(s_motion_fault_timing, now);
    g_motion_fault_diag_valid = 1U;
}
static void App_HeadingRuntimeTrip(uint32_t cause)
{
    uint32_t now, flags;
    taskENTER_CRITICAL();
    if (g_heading_exec_trip == 0U)
    {
        now = (uint32_t)xTaskGetTickCount();
        App_MotionFaultCapture(cause);
        g_heading_fault_phase = g_heading_exec_phase;
        g_heading_fault_tick_ms = now;
        g_heading_fault_sample_age_ms = now - s_heading_source.sample_ms;
        g_heading_fault_angle_age_ms = App_HeadingFrameAge(&s_heading_source,now,1U);
        g_heading_fault_rate_age_ms = App_HeadingFrameAge(&s_heading_source,now,2U);
        g_heading_fault_hwt_update_age_ms = now - g_hwt_update_tick;
        flags = 0U;
        if (s_heading_source_ready != 1U) flags |= 0x001U;
        if (s_heading_source.healthy != 1U) flags |= 0x002U;
        if (g_heading_fault_sample_age_ms >= HS_SOURCE_MS) flags |= 0x004U;
        if (g_heading_fault_angle_age_ms >= HS_SOURCE_MS) flags |= 0x008U;
        if (g_heading_fault_rate_age_ms >= HS_SOURCE_MS) flags |= 0x010U;
        if (g_hwt_cal_active != 0U) flags |= 0x020U;
        if (g_hwt_cal_request != 0U) flags |= 0x040U;
        if (g_hwt_cal_end_tx_result == 2U) flags |= 0x080U;
        if (g_hwt_version_state != 0U) flags |= 0x100U;
        if (g_hwt_version_request != 0U) flags |= 0x200U;
        if (g_hwt_a7_request != 0U) flags |= 0x400U;
        g_heading_fault_source_flags = flags;
        g_heading_exec_trip = cause;
    }
    g_heading_exec_rpm = 0;
    g_chassis_auto_request = 0U;
    ChassisAuto_Abort();
    g_vm_test_request = 0U;
    taskEXIT_CRITICAL();
}
static void App_HeadingRuntimeStep(void)
{
    HeadingSessionInput input = {0};
    HeadingSessionOutput output = {0};
    ChassisAutoStatus status;
    uint32_t request, accepted = 0U, now, record_zero = 0U;
    uint32_t critical_cycle,prepare_cycle,prepared_ok,expected_session;
    uint32_t uart_base,overflow_base,checksum_base;

    App_HeadingSourceObserve();
    taskENTER_CRITICAL();
    critical_cycle=HWT101_TimingBegin();
    now = (uint32_t)xTaskGetTickCount();
    App_HeadingStopObserve(now);
    request = g_heading_exec_request;
    g_heading_exec_request = 0U;
    (void)ChassisAuto_ReadStatus(&status);
    if (g_heading_exec_phase >= HR_DONE)
    {
        /* 终态保持观察结果；新request不排队，上电后只运行一次。 */
        g_vm_test_request = (g_heading_exec_phase == HR_DONE) ? App_StageDoneVm() : 0U;
        HWT101_TimingEnd(0U,critical_cycle);
        taskEXIT_CRITICAL();
        return;
    }
    if ((request == 2U) &&
        ((g_heading_exec_phase == HR_WAIT) || (g_heading_exec_phase == HR_RUN)))
        App_HeadingRuntimeTrip(1U);
    if ((request == 1U) && (g_heading_exec_phase == HR_IDLE))
    {
        s_heading_exec_relative = g_heading_exec_relative_deg;
        /* 本轮只接受正负5度或30度，不允许Watch任意扩大角度。 */
        if (((s_heading_exec_relative != 5.0f) && (s_heading_exec_relative != -5.0f) &&
             (s_heading_exec_relative != 30.0f) && (s_heading_exec_relative != -30.0f) &&
             !((APP_TEST_MODE == 3U) &&
               (((s_heading_exec_relative >= 5.0f) && (s_heading_exec_relative <= 90.0f)) ||
                ((s_heading_exec_relative <= -5.0f) && (s_heading_exec_relative >= -90.0f))))) ||
            !App_HeadingRuntimeSource(now) || !App_HeadingRuntimeExclusive() ||
            (g_vm_test_request != App_StageStartVm()) || (g_heading_exec_trip != 0U) ||
            (status.active != 0U) || (status.result != 0U) ||
            (status.cut_vm != 0U) ||
            !(s_heading_source.rate_dps >= -3.0f && s_heading_source.rate_dps <= 3.0f))
            App_HeadingRuntimeTrip(3U);
        else
        {
#if APP_TEST_MODE != 3U
            HWT101_DiagnosticArm();g_motion_fault_diag_valid=0U;
#endif
            s_heading_exec_wait_ms = now;
            g_heading_exec_phase = HR_WAIT;
            g_vm_test_request = 1U;
        }
    }
    if ((g_heading_exec_phase == HR_WAIT) && (g_heading_exec_trip == 0U))
    {
        if (!App_HeadingRuntimeSource(now)) App_HeadingRuntimeTrip(2U);
        else if (!App_HeadingRuntimeExclusive() || g_vm_test_request != 1U)
            App_HeadingRuntimeTrip(3U);
        else if ((uint32_t)(now - s_heading_exec_wait_ms) >= 5000U)
            App_HeadingRuntimeTrip(4U);
        else if ((App_CanJogReady() == 1U) && (App_HeadingPowerReady(now) == 1U))
        {
            input.now_ms = now;
#if APP_TEST_MODE == 3U
            input.max_rpm = s_motion_turn_max_rpm;
#endif
            input.sample_ms = s_heading_source.sample_ms;
            input.sequence = s_heading_source.sequence;
            input.healthy = input.platform_ok = 1U;
            input.yaw_deg = s_heading_source.yaw_deg;
            input.rate_dps = s_heading_source.rate_dps;
            input.backend_active = status.active;
            input.backend_running = (status.state == CHASSIS_AUTO_RUN) ? 1U : 0U;
            input.backend_result = status.result;
            input.backend_stop_mask = status.rotation_stop_mask;
            expected_session=status.session;
            uart_base=g_hwt_uart_errors;overflow_base=g_hwt_overflows;
            checksum_base=g_hwt_checksum_errors;
            HWT101_TimingEnd(0U,critical_cycle);taskEXIT_CRITICAL();
            prepare_cycle=HWT101_TimingBegin();
            HeadingSession_Init(&s_heading_prepared);
            prepared_ok=HeadingSession_Start(&s_heading_prepared,&input,
                s_heading_exec_relative,&output);
            if(prepared_ok)prepared_ok=ChassisAuto_PrepareHeading(0U,output.command_rpm,
                s_heading_prepared.turn_trial ? s_heading_prepared.initial_sign : 0,
                &s_heading_backend_prepared);
            g_heading_prepare_cycles=HWT101_TimingBegin()-prepare_cycle;
            if(g_heading_prepare_cycles>g_heading_prepare_max_cycles)
                g_heading_prepare_max_cycles=g_heading_prepare_cycles;
            taskENTER_CRITICAL();critical_cycle=HWT101_TimingBegin();
            now=(uint32_t)xTaskGetTickCount();
            (void)ChassisAuto_ReadStatus(&status);
            /* Preparation never grants motion: recheck cancellation, source epoch and ownership. */
            if(g_heading_exec_request==2U)App_HeadingRuntimeTrip(1U);
            else if(g_heading_exec_trip!=0U) { /* Keep the first safety failure. */ }
            else if(!App_HeadingRuntimeSource(now) || now-input.sample_ms>=HS_SOURCE_MS ||
                uart_base!=g_hwt_uart_errors || overflow_base!=g_hwt_overflows ||
                checksum_base!=g_hwt_checksum_errors)App_HeadingRuntimeTrip(2U);
            else if(g_heading_exec_phase!=HR_WAIT || !App_HeadingRuntimeExclusive() ||
                g_vm_test_request!=1U || !App_CanJogReady() || !App_HeadingPowerReady(now))
                App_HeadingRuntimeTrip(3U);
            else if(now-s_heading_exec_wait_ms>=5000U)App_HeadingRuntimeTrip(4U);
            else if(status.session!=expected_session || status.active || status.result || status.cut_vm)
                App_HeadingRuntimeTrip(5U);
            else if(!prepared_ok)App_HeadingRuntimeTrip(6U);
            else
            {
                s_heading_exec=s_heading_prepared;
                prepare_cycle=HWT101_TimingBegin();
                accepted=ChassisAuto_BeginPreparedHeading(&s_heading_backend_prepared,expected_session);
                HWT101_TimingEnd(1U,prepare_cycle);
                HeadingSession_ReportBegin(&s_heading_exec, accepted, &output);
                if (accepted != 1U) App_HeadingRuntimeTrip(6U);
                else
                {
                    (void)ChassisAuto_ReadStatus(&status);
                    g_heading_exec_backend_session = status.session;
                    g_heading_exec_phase = HR_RUN;
                    s_heading_exec_wait_ms = now;
                    g_heading_stop_trace.target_deg = output.target_deg;
                    App_HeadingStopCapture(&g_heading_stop_trace.start, now);
                    g_heading_stop_trace.started = 1U;
                }
            }
        }
    }
    if ((g_heading_exec_phase == HR_RUN) && (g_heading_exec_trip == 0U))
    {
        (void)ChassisAuto_ReadStatus(&status);
        if (!App_HeadingRuntimeOwns(&status)) App_HeadingRuntimeTrip(5U);
        else
        {
            input.now_ms = now;
            input.sample_ms = s_heading_source.sample_ms;
            input.sequence = s_heading_source.sequence;
            input.healthy = App_HeadingRuntimeSource(now);
            input.platform_ok = ((App_HeadingRuntimeExclusive() == 1U) &&
                (g_vm_test_request == 1U) &&
                ((App_CanJogReady() == 1U) ||
                 ((s_heading_exec.state == HS_STOPPING) &&
                  (status.active == 0U) && (status.result == 1U) &&
                  (status.rotation_stop_mask == 0x0FU)))) ? 1U : 0U;
            input.yaw_deg = s_heading_source.yaw_deg;
            input.rate_dps = s_heading_source.rate_dps;
            input.backend_active = status.active;
            input.backend_running = (status.state == CHASSIS_AUTO_RUN) ? 1U : 0U;
            input.backend_result = status.result;
            input.backend_stop_mask = status.rotation_stop_mask;
            HeadingSession_Step(&s_heading_exec, &input, &output);
            if (output.action == HS_ACTION_UPDATE)
            {
                record_zero = ((output.command_rpm == 0) &&
                    (!s_heading_exec.turn_trial || s_heading_exec.brake.moved) &&
                    (g_heading_stop_trace.zero_request.captured == 0U)) ? 1U : 0U;
                if (record_zero != 0U)
                    App_HeadingStopCapture(&g_heading_stop_trace.zero_request, now);
                accepted = ChassisAuto_UpdateHeading(output.command_rpm);
                if (record_zero != 0U)
                    g_heading_stop_trace.zero_submit_accepted = accepted;
                HeadingSession_ReportUpdate(&s_heading_exec, accepted, &output);
            }
            else if (output.action == HS_ACTION_STOP)
            {
                App_HeadingStopCapture(&g_heading_stop_trace.reached, now);
                accepted = ChassisAuto_RequestStop();
                HeadingSession_ReportStop(&s_heading_exec, accepted, &output);
            }
            if (output.action == HS_ACTION_ABORT) App_HeadingRuntimeTrip(6U);
            else if (output.action == HS_ACTION_COMPLETE)
            {
                App_HeadingStopCapture(&g_heading_stop_trace.completed, now);
                g_heading_exec_phase = HR_DONE;
                g_vm_test_request = App_StageDoneVm();
            }
        }
    }
    if (g_heading_exec_trip != 0U)
    {
        if (g_heading_exec_trip == 1U)
        {
            HeadingSession_Cancel(&s_heading_exec, &output);
            g_heading_exec_phase = HR_CANCELLED;
        }
        else
        {
            /* 只让CAN任务终结会话；故障不伪装成正常到位。 */
            input.healthy = input.platform_ok = 0U;
            input.backend_result = 2U;
            input.now_ms = now;
            HeadingSession_Step(&s_heading_exec, &input, &output);
            g_heading_exec_phase = HR_FAULT;
        }
        output.action = HS_ACTION_ABORT;
        output.command_rpm = 0;
    }
    g_heading_exec_state = output.state;
    g_heading_exec_cause = output.cause;
    g_heading_exec_action = output.action;
    g_heading_exec_rpm = output.command_rpm;
    if (g_heading_exec_phase != HR_IDLE)
    {
        g_heading_exec_target_deg = s_heading_exec.controller.target_deg;
        g_heading_exec_error_deg = s_heading_exec.controller.error_deg;
    }
    HWT101_TimingEnd(0U,critical_cycle);
    taskEXIT_CRITICAL();

    /* 真正的CAN交易放在临界区外，仍只有这一个CAN任务。 */
    if ((g_heading_exec_phase == HR_RUN) && (g_heading_exec_trip == 0U))
        test_can_steps++;
}
static void reset_test(void){
 test_tick=2000;test_depth=0;test_can_steps=prepare_calls=0;mock_exclusive=mock_power=1;
 g_rtos_fault=g_safety_fault=g_motion_diag_fault_latched=g_motion_auto_stop_state=g_motion_auto_stop_cut_vm=0;
 g_wheel_test_active=g_wheel_test_cut_vm=g_can_ctrl_fault_latched=0;g_safety_ready=g_can_guard_ready=1;
 ChassisAuto_Init();ChassisAuto_SafetyUpdate(1);
 g_heading_exec_request=g_heading_exec_trip=0;g_heading_exec_phase=HR_WAIT;g_vm_test_request=1;
 g_motion_fault_diag_valid=0;g_hwt_uart_errors=g_hwt_overflows=g_hwt_checksum_errors=0;
 g_hwt_cal_active=g_hwt_cal_request=g_hwt_cal_end_tx_result=g_hwt_version_state=g_hwt_version_request=g_hwt_a7_request=0;
 g_hwt_version_result=1;g_hwt_valid_mask=3;g_hwt_update_tick=test_tick;
 g_hwt_angle_age_ms=g_hwt_rate_age_ms=0;g_hwt_yaw_deg=177;g_hwt_wz_dps=0;
 App_HeadingSourcePublish();s_heading_source_ready=1;
 s_motion_turn_max_rpm=APP_TURN_BRAKE_MAX_RPM;s_heading_exec_relative=90;s_heading_exec_wait_ms=test_tick;
 HeadingSession_Init(&s_heading_exec);memset(&g_heading_stop_trace,0,sizeof(g_heading_stop_trace));
}
uint32_t test_entry(void){
 uint32_t n,session;ChassisAutoPreparedHeading prepared;const uint32_t causes[10]={0,1,2,2,2,3,5,3,3,3};
 for(mode=0;mode<10;mode++){
  reset_test();session=g_chassis_auto_session;App_HeadingRuntimeStep();
  CHECK(test_depth==0);CHECK(prepare_calls==1);CHECK(g_heading_exec_trip==causes[mode]);
  if(mode==0){CHECK(g_chassis_auto_active==1);CHECK(g_chassis_auto_session==session+1);
   CHECK(s_turn_brake_trial);CHECK(g_heading_exec_phase==HR_RUN);CHECK(test_can_steps==1);
   for(n=0;n<4;n++)CHECK(s_commands[2][n].speed_raw==0);
  }else{CHECK(!g_chassis_auto_active);CHECK(!g_vm_test_request);CHECK(!test_can_steps);
   CHECK(g_motion_fault_diag_valid==1);CHECK(g_motion_fault_diag[2]==causes[mode]);
   App_HeadingRuntimeTrip(8);CHECK(g_motion_fault_diag[2]==causes[mode]);
   CHECK(g_heading_exec_trip==causes[mode]);
   App_HeadingRuntimeStep();CHECK(prepare_calls==1);CHECK(g_heading_exec_trip==causes[mode]);}
 }
 mode=0;reset_test();g_hwt_update_tick=test_tick=22930;
 g_hwt_valid_mask=0;g_hwt_angle_age_ms=g_hwt_rate_age_ms=0xffffffffU;App_HeadingSourcePublish();
 CHECK(!s_heading_source.healthy);CHECK(App_HeadingFrameAge(&s_heading_source,22938,1)==0xffffffffU);
 CHECK(App_HeadingFrameAge(&s_heading_source,22938,2)==0xffffffffU);
 g_hwt_valid_mask=2;g_hwt_rate_age_ms=7;App_HeadingSourcePublish();
 CHECK(App_HeadingFrameAge(&s_heading_source,22938,1)==0xffffffffU);
 CHECK(App_HeadingFrameAge(&s_heading_source,22938,2)==15);
 g_hwt_valid_mask=3;test_tick=g_hwt_update_tick=3;g_hwt_angle_age_ms=g_hwt_rate_age_ms=7;
 App_HeadingSourcePublish();CHECK(s_heading_source.healthy);CHECK(App_HeadingFrameAge(&s_heading_source,3,1)==7);
 reset_test();session=g_chassis_auto_session;
 CHECK(ChassisAuto_PrepareHeading(0,0,1,&prepared));CHECK(!g_chassis_auto_active);
 CHECK(!ChassisAuto_BeginPreparedHeading(&prepared,session+1));
 prepared.commands[2][0].speed_raw=1;CHECK(!ChassisAuto_BeginPreparedHeading(&prepared,session));
 CHECK(!g_chassis_auto_active);
 CHECK(ChassisAuto_PrepareHeading(0,5,0,&prepared));CHECK(ChassisAuto_BeginPreparedHeading(&prepared,session));
 CHECK(!s_turn_brake_trial);CHECK(!ChassisAuto_BeginPreparedHeading(&prepared,session));
 return first_failure ? (0x80000000U|first_failure) : checks;
}
