#include <math.h>
/* 仅由startup_axes.c包含；全部CAN操作仍在唯一CAN任务执行。
 * 驱动本会话完成不等于已经触挡或机械原点验收，需现场观察。
 */
volatile uint32_t g_startup_axes_state=0U,g_startup_axes_cause=0U,g_startup_axes_vm_request=0U;
volatile uint32_t g_startup_extension_feedback_valid=0U;
volatile float g_extension_home_reference_deg=0.0f;
volatile uint32_t g_extension_home_reference_valid=0U;
volatile float g_startup_extension_current_deg=0.0f,g_startup_extension_target_deg=0.0f;
volatile uint32_t g_extension_home_state=0U,g_extension_home_result=0U;
volatile uint32_t g_extension_home_flags=0U,g_extension_home_reported=0U,g_extension_home_elapsed_ms=0U;
volatile uint32_t g_extension_home_capture=0U,g_extension_home_done_notice=0U;
volatile uint32_t g_extension_home_abort_result=0U,g_extension_home_stop_result=0U;
volatile uint32_t g_eh_first_reason=0U,g_eh_first_command=0U,g_eh_first_exchange=0U;
volatile uint32_t g_eh_first_flags=0U,g_eh_first_tick=0U,g_eh_first_service_gap=0U,g_eh_first_phase=0U;
static uint32_t eh_last_command,eh_last_exchange,eh_last_flags;
volatile uint32_t g_eh_first_step=0U,g_eh_speed_sign=0xffffffffU,g_eh_speed_rpm=0xffffffffU;

volatile uint32_t g_extension_hold_enable_count=0U,g_extension_hold_samples=0U;
volatile uint32_t g_extension_hold_confirmed=0U,g_extension_hold_ready_tick=0U,g_extension_hold_delta_mdeg=0U;
static uint32_t eh_hold_begin;
static float eh_position_units_per_deg=10.0f;
static float eh_hold_position;
static uint32_t eh_last,eh_started,eh_stop_pending;
/* 保位基准只在 ENABLE 之后取一次：STOP->ENABLE 之间机械被负载顶回的
 * 位移不是保位漂移，不能拿回零瞬间的位置当基准。 */
static uint32_t eh_hold_base_locked = 0U;
#if !APP_MOTOR_OFFLINE_ONLY
static uint32_t eh_begin,eh_motion,eh_poll,eh_seen,eh_phase;
#endif
#if !APP_MOTOR_OFFLINE_ONLY
static CanRxMsg eh_reply;
uint32_t StartupAxis_ReadHomeParameters(uint8_t out[17]);
static uint32_t eh_query(uint8_t command,uint8_t length)
{uint8_t q[2]={command,0x6b};
 eh_last_command=command;eh_last_exchange=0U;
 eh_last_exchange=StartupAxis_Exchange(q,2U,length,&eh_reply);
 eh_last_flags=eh_last_exchange==1U?(uint32_t)eh_reply.Data[1]:0xffffffffU;
 if(command==0x35U){
     g_eh_speed_sign=eh_last_exchange==1U?(uint32_t)eh_reply.Data[1]:0xffffffffU;
     g_eh_speed_rpm=eh_last_exchange==1U?(((uint32_t)eh_reply.Data[2]<<8)|eh_reply.Data[3]):0xffffffffU;
 }
 return eh_last_exchange==1U;}
static uint32_t eh_control(const uint8_t *q,uint8_t length)
{eh_last_command=q[0];eh_last_exchange=0U;
 eh_last_exchange=StartupAxis_Exchange(q,length,3U,&eh_reply);
 eh_last_flags=eh_last_exchange==1U?(uint32_t)eh_reply.Data[1]:0xffffffffU;
 return eh_last_exchange==1U && (eh_reply.Data[1]==2U || (q[0]==0x9aU && eh_reply.Data[1]==0x9fU));}
static uint32_t eh_u16(const uint8_t *p){return ((uint32_t)p[0]<<8)|p[1];}
#endif
static void eh_fail(uint32_t reason)
{
    taskENTER_CRITICAL();
    if(!g_eh_first_reason){
        g_eh_first_command=eh_last_command;g_eh_first_exchange=eh_last_exchange;g_eh_first_flags=eh_last_flags;
        g_eh_first_tick=(uint32_t)xTaskGetTickCount();g_eh_first_service_gap=g_eh_first_tick-eh_last;
#if !APP_MOTOR_OFFLINE_ONLY
        g_eh_first_step=eh_phase;
#endif
        g_eh_first_phase=g_extension_home_state;g_eh_first_reason=reason;
    }
    taskEXIT_CRITICAL();
    if(g_startup_axes_state!=5U && g_startup_axes_state!=6U){
        g_extension_home_result=reason;g_startup_axes_cause=reason;g_extension_home_state=9U;
        g_startup_axes_state=5U;
    }
    g_extension_home_reported=0U;
#if APP_FAULT_CUT_VM_ENABLE
    g_startup_axes_vm_request=0U;
#endif
    if(eh_started)eh_stop_pending=1U;
}
/* 2026-11-08: was a hardcoded `return 0U`, which silently disabled every consumer
 * of Holding() -- DM_CAN2_ModuleService (so CAN2 was never clocked and the 4310
 * could never be read), App_CombinedTestStep and the vehicle-arm body service.
 * This profile does maintain both fields, so report them honestly. */
uint32_t StartupAxes_Holding(void){return g_startup_axes_state==4U && g_startup_axes_vm_request==1U;}
uint32_t StartupAxes_NormalStopHold(void){return 0U;}
uint32_t StartupAxes_Begin(void)
{
#if APP_MOTOR_OFFLINE_ONLY
    return 0U;
#else
    if(g_startup_axes_state!=0U)return 0U;
    /* 必须显式按键；固定目标参考有效位不用于放行碰撞回零。 */
    if(VEHICLE_EXTENSION_HOME_DIRECTION!=0U || VEHICLE_EXTENSION_HOME_RPM!=30U ||
       VEHICLE_EXTENSION_HOME_DRIVER_TIMEOUT_MS!=10000U ||
       VEHICLE_EXTENSION_HOME_DETECT_RPM!=300U || VEHICLE_EXTENSION_HOME_DETECT_CURRENT_MA!=800U ||
       VEHICLE_EXTENSION_HOME_DETECT_TIME_MS!=60U){eh_fail(7U);return 0U;}
    if(VEHICLE_EXTENSION_HOLD_CONFIRM_SAMPLES<1U || VEHICLE_EXTENSION_HOLD_CONFIRM_SAMPLES>5U ||
       VEHICLE_EXTENSION_HOLD_TIMEOUT_MS<500U || VEHICLE_EXTENSION_HOLD_TIMEOUT_MS>5000U ||
       !(VEHICLE_EXTENSION_HOLD_POSITION_TOL_DEG>0.0f && VEHICLE_EXTENSION_HOLD_POSITION_TOL_DEG<=5.0f))
    {eh_fail(7U);return 0U;}
    g_eh_first_step=0U;g_eh_speed_sign=g_eh_speed_rpm=0xffffffffU;
    g_extension_hold_enable_count=g_extension_hold_samples=g_extension_hold_confirmed=0U;
    g_extension_hold_ready_tick=g_extension_hold_delta_mdeg=eh_hold_begin=0U;
    eh_begin=eh_last=(uint32_t)xTaskGetTickCount();eh_phase=eh_started=eh_seen=eh_stop_pending=0U;eh_hold_base_locked=0U;
    g_extension_home_capture=g_extension_home_done_notice=g_extension_home_reported=0U;
    g_extension_home_reference_valid=0U;
    g_startup_axes_cause=g_extension_home_result=0U;
    g_startup_axes_state=g_extension_home_state=1U;g_startup_axes_vm_request=1U;return 1U;
#endif
}
void StartupAxes_Cancel(void)
{
    if(g_startup_axes_state==5U)return;
    g_startup_axes_state=6U;g_startup_axes_cause=2U;g_extension_home_state=10U;
    g_extension_home_result=2U;g_extension_home_reported=0U;
#if APP_FAULT_CUT_VM_ENABLE
    g_startup_axes_vm_request=0U;
#endif
    if(eh_started)eh_stop_pending=1U;
}
void StartupAxes_Safety(uint32_t healthy)
{
    uint32_t now=(uint32_t)xTaskGetTickCount();
    if(g_startup_axes_state>=1U && g_startup_axes_state<=4U){
        if(!healthy)eh_fail(6U);
        /* LiftRuler-only build serialises CAN1 behind CAN2 reads; a single DM
           point move can hold the sole CAN task past 150ms. Keep the watchdog
           but tolerate the longest observed inter-service gap. */
        else if((uint32_t)(now-eh_last)>900U)eh_fail(8U);
    }
}
void StartupAxes_Service(uint32_t platform_ready,uint32_t can_guard_ready)
{
#if !APP_MOTOR_OFFLINE_ONLY
    uint32_t now=(uint32_t)xTaskGetTickCount(),raw;PowerCommandStatus power;uint8_t p[17];
    const uint8_t enable[5]={0xf3,0xab,1,0,0x6b},home[4]={0x9a,2,0,0x6b};
    const uint8_t abort_home[3]={0x9c,0x48,0x6b},stop[4]={0xfe,0x98,0,0x6b};
    /* 安全任务只撤销供电/发布停止，不能跨任务访问CAN。 */
    if(eh_stop_pending){
        g_extension_home_capture=0U;
        g_extension_home_abort_result=eh_control(abort_home,3U);
        g_extension_home_stop_result=eh_control(stop,4U);
        if(g_extension_home_abort_result && g_extension_home_stop_result){eh_started=eh_stop_pending=0U;}
        return;
    }
    if(g_startup_axes_state<1U || g_startup_axes_state>4U)return;
    eh_last=now;
    if(!platform_ready){eh_fail(6U);return;}
    g_extension_home_elapsed_ms=now-(eh_started?eh_motion:eh_begin);
    if(g_startup_axes_state!=4U && !(g_startup_axes_state==3U && eh_phase>=3U) && g_extension_home_elapsed_ms>=(eh_started?11000U:2500U))
    {eh_fail(3U);return;}
    Power_ReadCommandStatus(&power);if(!power.active || !can_guard_ready)return;
    if(g_startup_axes_state==1U)g_startup_axes_state=2U;
    if(g_startup_axes_state==2U){
        g_extension_home_state=eh_phase+1U;
        switch(eh_phase){
        case 0U:
            if(!eh_query(0x1a,4U) || (eh_reply.Data[2]&3U)){eh_fail(4U);return;}eh_position_units_per_deg=(eh_reply.Data[2]&0x80U)?100.0f:10.0f;break;
        case 1U:
            if(StartupAxis_ReadHomeParameters(p)!=1U){eh_fail(4U);return;}
            if(p[0]!=0x22 || p[16]!=0x6b || p[2]!=VEHICLE_EXTENSION_HOME_DIRECTION ||
               eh_u16(p+3)!=VEHICLE_EXTENSION_HOME_RPM ||
               (((uint32_t)p[5]<<24)|((uint32_t)p[6]<<16)|((uint32_t)p[7]<<8)|p[8])!=VEHICLE_EXTENSION_HOME_DRIVER_TIMEOUT_MS ||
               eh_u16(p+9)!=VEHICLE_EXTENSION_HOME_DETECT_RPM || eh_u16(p+11)!=VEHICLE_EXTENSION_HOME_DETECT_CURRENT_MA ||
               eh_u16(p+13)!=VEHICLE_EXTENSION_HOME_DETECT_TIME_MS || p[15]!=0U){eh_fail(7U);return;}break;
        case 2U:
            if(!eh_query(0x3a,3U) || (eh_reply.Data[1]&0x0cU)){eh_fail(5U);return;}
            if(!(eh_reply.Data[1]&1U) && !eh_control(enable,5U)){eh_fail(4U);return;}break;
        case 3U:
            if(!eh_query(0x3b,3U)){eh_fail(4U);return;}
            g_extension_home_flags=eh_reply.Data[1];
            if((g_extension_home_flags&3U)!=3U || (g_extension_home_flags&0x3cU)){eh_fail(5U);return;}break;
        case 4U:
            if(!eh_query(0x35,5U)){eh_fail(4U);return;}
            if(eh_reply.Data[2] || eh_reply.Data[3]){eh_fail(5U);return;}break;
        default:
#if VEHICLE_STARTUP_GROUP_ENABLE
            /* 两轴准备屏障：尚未联合释放时不能先触发回零。 */
            if(!g_startup_group_go)return;
#endif
            eh_started=1U;eh_motion=now;g_extension_home_capture=g_extension_home_done_notice=0U;
            if(!eh_control(home,4U)){eh_fail(4U);return;}
            g_startup_axes_state=3U;g_extension_home_state=6U;eh_phase=0U;eh_poll=now;return;
        }
        eh_phase++;return;
    }
    if(g_startup_axes_state==3U && eh_phase>=3U && now-eh_hold_begin>=VEHICLE_EXTENSION_HOLD_TIMEOUT_MS)
    {eh_fail(3U);return;}
    if(now-eh_poll<50U)return;eh_poll=now;
    if(g_startup_axes_state==3U && eh_phase>=3U){
        g_extension_home_state=9U;
        switch(eh_phase){
        case 3U:
            if(!eh_query(0x3a,3U)){eh_fail(4U);return;}
            if(eh_reply.Data[1]&0x0cU){eh_fail(5U);return;}eh_phase=4U;return;
        case 4U:
            if(!eh_query(0x35,5U)){eh_fail(4U);return;}
            /* 交接允许在既有2秒截止内等待精确零速，不提前放行或使能。 */
            if(eh_reply.Data[2] || eh_reply.Data[3])return;eh_phase=5U;return;
        case 5U:
            if(!eh_control(stop,4U)){eh_fail(4U);return;}eh_phase=6U;return;
        case 6U:
            g_extension_hold_enable_count++;
            if(!eh_control(enable,5U)){eh_fail(4U);return;}eh_phase=7U;return;
        case 7U:
            if(!eh_query(0x3a,3U)){eh_fail(4U);return;}
            if(eh_reply.Data[1]&0x0cU){eh_fail(5U);return;}
            if(!(eh_reply.Data[1]&1U)){g_extension_hold_samples=0U;return;}
            eh_phase=8U;return;
        case 8U:
            if(!eh_query(0x35,5U)){eh_fail(4U);return;}
            /* 非零样本中断连续确认；不重新使能，仍受同一截止约束。 */
            if(eh_reply.Data[2] || eh_reply.Data[3]){g_extension_hold_samples=0U;eh_phase=7U;return;}eh_phase=9U;return;
        default:
            if(!eh_query(0x36,7U) || eh_reply.Data[1]>1U){eh_fail(4U);return;}
            raw=((uint32_t)eh_reply.Data[2]<<24)|((uint32_t)eh_reply.Data[3]<<16)|((uint32_t)eh_reply.Data[4]<<8)|eh_reply.Data[5];
            g_startup_extension_current_deg=(eh_reply.Data[1]?-1.0f:1.0f)*(float)raw/eh_position_units_per_deg;
            if(eh_hold_base_locked==0U){
                eh_hold_position=g_startup_extension_current_deg;
                g_startup_extension_target_deg=eh_hold_position;
                eh_hold_base_locked=1U;
            }
            {float delta=fabsf(g_startup_extension_current_deg-eh_hold_position);
             g_extension_hold_delta_mdeg=delta<4000000.0f?(uint32_t)(delta*1000.0f):0xffffffffU;
             if(!(delta<=VEHICLE_EXTENSION_HOLD_POSITION_TOL_DEG)){eh_fail(5U);return;}}
            if(++g_extension_hold_samples>=VEHICLE_EXTENSION_HOLD_CONFIRM_SAMPLES){
                g_extension_hold_confirmed=1U;g_extension_hold_ready_tick=now;eh_started=0U;
                g_extension_home_reference_deg=eh_hold_position;
                g_extension_home_reference_valid=1U;
                g_extension_home_state=8U;g_extension_home_result=g_extension_home_reported=1U;g_startup_axes_state=4U;
            }else eh_phase=7U;
            return;
        }
    }

    if(g_startup_axes_state==4U){
        /* 回零后保持供电，不再发送运动；故障或取消仍切VM。 */
        /* 2026-11-08: 0x0c covers bit2 (running) AND bit3 (stall protect). While the
         * parallel transfer drives this axis, bit2 is set and the hold service used to
         * declare a fault within 50ms, aborting IT3 mid-move (axes_cause=5 -> it_fail(3)).
         * Only the stall-protect bit matters for holding. */
        if(!eh_query(0x3a,3U) || !(eh_reply.Data[1]&1U) || (eh_reply.Data[1]&0x08U))eh_fail(5U);
        return;
    }
    if(eh_phase==0U){
        if(!eh_query(0x3b,3U)){eh_fail(4U);return;}g_extension_home_flags=eh_reply.Data[1];
        if((g_extension_home_flags&3U)!=3U || (g_extension_home_flags&0x38U)){eh_fail(5U);return;}
        if(g_extension_home_flags&4U)eh_seen=1U;
        if(!(g_extension_home_flags&0x0cU) && (eh_seen || g_extension_home_done_notice))eh_phase=1U;
    }else if(eh_phase==1U){
        if(!eh_query(0x35,5U)){eh_fail(4U);return;}
        if(eh_reply.Data[2] || eh_reply.Data[3]){eh_phase=0U;return;}eh_phase=2U;
    }else{
        if(!eh_query(0x36,7U) || eh_reply.Data[1]>1U){eh_fail(4U);return;}
        raw=((uint32_t)eh_reply.Data[2]<<24)|((uint32_t)eh_reply.Data[3]<<16)|((uint32_t)eh_reply.Data[4]<<8)|eh_reply.Data[5];
        g_startup_extension_current_deg=(eh_reply.Data[1]?-1.0f:1.0f)*(float)raw/eh_position_units_per_deg;
        g_startup_extension_feedback_valid=1U;g_extension_home_capture=0U;
        /* Immutable for this homing cycle; hold/rebound samples must not rebase travel. */
        eh_hold_position=g_startup_extension_current_deg;g_startup_extension_target_deg=eh_hold_position;
        eh_hold_begin=now;eh_phase=3U;g_extension_home_state=9U;
        /* Not ready until STOP, one enable and three fresh readback bundles. */
    }
#else
    (void)platform_ready;(void)can_guard_ready;
#endif
}
void StartupAxes_SetExtensionHoldTarget(float target)
{
    /* Was a no-op: SourcePrep moved the axis to 70 mm and the controller never
     * learned the new hold point, so the arm relied on the driver alone. */
    if (!(target >= -100000.0f && target <= 100000.0f)) return;
    eh_hold_position = target;
    g_startup_extension_target_deg = target;
}
