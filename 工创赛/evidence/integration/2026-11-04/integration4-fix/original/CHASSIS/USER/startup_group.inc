/* 复用已验伸缩回零控制器，保留独立状态，再由本编排统一释放/取消。
 * 只有唯一CAN任务发总线命令；安全任务只发布停止并撤VM。
 */
#include "startup_pose.h"
extern volatile uint32_t g_dm_startup_go;
volatile uint32_t g_startup_group_go=0U,g_startup_group_pwm_result=0U;
#define g_startup_axes_state g_group_extension_state
#define g_startup_axes_cause g_group_extension_cause
#define g_startup_axes_vm_request g_group_extension_vm_request
#define StartupAxes_Holding GroupExtension_Holding
#define StartupAxes_NormalStopHold GroupExtension_NormalStopHold
#define StartupAxes_Begin GroupExtension_Begin
#define StartupAxes_Cancel GroupExtension_Cancel
#define StartupAxes_Safety GroupExtension_Safety
#define StartupAxes_Service GroupExtension_Service
#define StartupAxes_SetExtensionHoldTarget GroupExtension_SetHoldTarget
#include "extension_home_only.inc"
#undef g_startup_axes_state
#undef g_startup_axes_cause
#undef g_startup_axes_vm_request
#undef StartupAxes_Holding
#undef StartupAxes_NormalStopHold
#undef StartupAxes_Begin
#undef StartupAxes_Cancel
#undef StartupAxes_Safety
#undef StartupAxes_Service
#undef StartupAxes_SetExtensionHoldTarget

volatile uint32_t g_startup_axes_state=0U,g_startup_axes_cause=0U,g_startup_axes_vm_request=0U;
volatile uint32_t g_startup_group_release_ms=0U;
static uint32_t sg_begin,sg_last;
#if !APP_MOTOR_OFFLINE_ONLY
static uint32_t sg_dm_started;
#endif
static void StartupGroup_Fail(uint32_t cause)
{
    if(g_startup_axes_state!=5U && g_startup_axes_state!=6U){
        g_startup_axes_state=5U;g_startup_axes_cause=cause;
    }
    GroupExtension_Cancel();g_dm_jog_request=3U;
#if APP_FAULT_CUT_VM_ENABLE
    g_startup_axes_vm_request=0U;
#endif
}
uint32_t StartupAxes_Holding(void)
{return g_startup_axes_state==4U && g_startup_axes_vm_request==1U;}
uint32_t StartupAxes_NormalStopHold(void)
{
    ChassisAutoStatus s;
    if(!StartupAxes_Holding())return 0U;
    (void)ChassisAuto_ReadStatus(&s);
    return !s.active && s.state==CHASSIS_AUTO_DONE && s.result==1U && !s.cause && s.rotation_stop_mask==15U;
}
uint32_t StartupAxes_Begin(void)
{
#if APP_MOTOR_OFFLINE_ONLY
    return 0U;
#else
    if(g_startup_axes_state!=0U)return 0U;
    if(!StartupPose_GroupValid() || !(STARTUP_DM_SPEED_SCALE>0.0f && STARTUP_DM_SPEED_SCALE<=1.0f)){
        StartupGroup_Fail(1U);return 0U;
    }
    g_startup_group_go=g_startup_group_pwm_result=g_startup_group_release_ms=0U;
    g_dm_startup_go=0U;sg_dm_started=0U;sg_begin=sg_last=(uint32_t)xTaskGetTickCount();
    if(!GroupExtension_Begin()){StartupGroup_Fail(11U);return 0U;}
    g_startup_axes_cause=0U;g_startup_axes_state=1U;g_startup_axes_vm_request=1U;return 1U;
#endif
}
void StartupAxes_Cancel(void)
{
    if(g_startup_axes_state!=5U){g_startup_axes_state=6U;g_startup_axes_cause=2U;}
    GroupExtension_Cancel();g_dm_jog_request=3U;
#if APP_FAULT_CUT_VM_ENABLE
    g_startup_axes_vm_request=0U;
#endif
}
void StartupAxes_Safety(uint32_t healthy)
{
    uint32_t now=(uint32_t)xTaskGetTickCount();
    if(g_startup_axes_state>=1U && g_startup_axes_state<=4U){
        GroupExtension_Safety(healthy);
        if(!healthy || (uint32_t)(now-sg_last)>150U)StartupGroup_Fail(2U);
        else if(g_group_extension_state>=5U)StartupGroup_Fail(11U);
        else if(g_dm_jog_first_fault)StartupGroup_Fail(7U);
        else if(g_startup_axes_state!=4U && (uint32_t)(now-sg_begin)>=15000U)StartupGroup_Fail(3U);
    }
}
void StartupAxes_Service(uint32_t platform_ready,uint32_t can_guard_ready)
{
#if !APP_MOTOR_OFFLINE_ONLY
    uint32_t now=(uint32_t)xTaskGetTickCount();PowerCommandStatus power;
    if(g_startup_axes_state==5U || g_startup_axes_state==6U){
        GroupExtension_Service(0U,0U);
        if(g_dm_jog_request || DM_CAN2_JogIsBusy() || g_dm_posvel_holding)DM_CAN2_ModuleService(0U,0U);
        return;
    }
    if(g_startup_axes_state<1U || g_startup_axes_state>4U)return;
    sg_last=now;
    if(!platform_ready){StartupGroup_Fail(2U);return;}
    Power_ReadCommandStatus(&power);
    if(!power.active || !can_guard_ready){
        if(g_startup_group_go)StartupGroup_Fail(2U);
        else GroupExtension_Service(platform_ready,can_guard_ready);
        return;
    }
    if(g_startup_axes_state==1U)g_startup_axes_state=2U;
    if(!sg_dm_started){
        g_dm_point_enable=1U;g_dm_posvel_speed_deg_s=DM_CAN2_POSVEL_SPEED_DEG_S*STARTUP_DM_SPEED_SCALE;
        g_dm_jog_request=6U;sg_dm_started=1U;
    }
    DM_CAN2_ModuleService(1U,0U);
    if(g_dm_jog_first_fault || (g_dm_jog_state==8U && !DM_CAN2_SourceTransitionPending())){StartupGroup_Fail(7U);return;}
    GroupExtension_Service(platform_ready,can_guard_ready);
    if(g_group_extension_state>=5U){StartupGroup_Fail(11U);return;}
    DM_CAN2_ModuleService(1U,0U);
    if(g_dm_jog_first_fault || (g_dm_jog_state==8U && !DM_CAN2_SourceTransitionPending())){StartupGroup_Fail(7U);return;}
    if(!g_startup_group_go && g_group_extension_state==2U && eh_phase>=5U &&
       g_dm_jog_state==3U && g_dm_can2_motor_state==1U){
        /* PWM、CAN1回零、CAN2放行在同一服务轮串行提交，非硬件同步。 */
        /* 临界区只含PWM寄存器操作和标志发布，不在临界区内等CAN。 */
        taskENTER_CRITICAL();
        if(g_startup_axes_state==2U && g_startup_axes_vm_request && g_group_extension_state==2U){
            g_startup_group_pwm_result=StartupPose_GroupCommand();
            if(g_startup_group_pwm_result==1U && g_startup_axes_state==2U && g_startup_axes_vm_request){
                g_startup_group_release_ms=now;g_startup_group_go=g_dm_startup_go=1U;
            }
        }
        taskEXIT_CRITICAL();
        if(g_startup_axes_state!=2U || !g_startup_axes_vm_request)return;
        if(g_startup_group_pwm_result!=1U){StartupGroup_Fail(12U);return;}
        GroupExtension_Service(platform_ready,can_guard_ready);
        if(g_startup_axes_state>=5U || !g_startup_axes_vm_request){GroupExtension_Cancel();return;}
        if(g_group_extension_state>=5U){StartupGroup_Fail(11U);return;}
        taskENTER_CRITICAL();
        if(g_startup_axes_state==2U && g_startup_axes_vm_request && g_group_extension_state==3U)
            g_startup_axes_state=3U;
        taskEXIT_CRITICAL();
        if(g_startup_axes_state!=3U || !g_startup_axes_vm_request){GroupExtension_Cancel();return;}
        if(g_group_extension_state>=5U){StartupGroup_Fail(11U);return;}
        DM_CAN2_ModuleService(1U,0U);
    }
    taskENTER_CRITICAL();
    if(g_startup_axes_state==3U && g_startup_axes_vm_request && g_group_extension_state==4U && g_extension_home_reported &&
       g_dm_jog_state==7U && g_dm_jog_result==1U && g_dm_posvel_holding &&
       (uint32_t)(now-g_startup_group_release_ms)>=STARTUP_GROUP_SERVO_SETTLE_MS){
        g_startup_axes_state=4U;g_dm_posvel_speed_deg_s=DM_CAN2_POSVEL_SPEED_DEG_S;
    }
    taskEXIT_CRITICAL();
#else
    (void)platform_ready;(void)can_guard_ready;
#endif
}
void StartupAxes_SetExtensionHoldTarget(float target)
{
    /* Forward to the extension controller instead of dropping the request. */
    GroupExtension_SetHoldTarget(target);
}
