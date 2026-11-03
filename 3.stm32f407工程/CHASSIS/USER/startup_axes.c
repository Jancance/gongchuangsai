#include "startup_axes.h"
#include "vehicle_arm.h"
#include "../../USER/startup_pose_config.h"
#include "app_config.h"
#include "dm_can2.h"
#include "can.h"
#include "chassis_auto_test.h"
#include "power.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <string.h>
/* 0未启动1供电2准备3并行运行4到位保持5失败6取消7未填写参考。
 * 按键前不使能、不设零、不自动回零。旧驱动坐标不能替代机械参考。 */
volatile uint32_t g_startup_axes_state=0U,g_startup_axes_cause=0U,g_startup_axes_vm_request=0U;
volatile uint32_t g_startup_extension_feedback_valid=0U;
volatile float g_startup_extension_current_deg=0.0f,g_startup_extension_target_deg=0.0f;
volatile float g_startup_extension_zero_deg=STARTUP_EXTENSION_ZERO_DEG;
static uint32_t s_begin_ms,s_last_service;
#if !APP_MOTOR_OFFLINE_ONLY
static uint32_t s_phase,s_dm_ready,s_ext_ready,s_started;
static uint32_t s_poll_ms,s_settled,s_monitor;
static uint8_t s_options,s_move[15];
static CanRxMsg s_reply;
static float s_target,s_start;
static uint32_t s_ext_done,s_send_move,s_stall_since,s_stall_active,s_flags;
#endif
volatile uint32_t g_startup_extension_reference_valid=STARTUP_EXTENSION_REFERENCE_VALID;
#if !APP_MOTOR_OFFLINE_ONLY
static uint32_t s_home_phase,s_home_done,s_home_seen_running,s_home_begin,s_home_poll;
static uint32_t StartupExtension_HomeStep(uint32_t now);
#endif

extern volatile uint32_t g_dm_startup_go;

#if !APP_MOTOR_OFFLINE_ONLY
static uint32_t exchange(const uint8_t *q,uint8_t length,uint8_t n)
{return StartupAxis_Exchange(q,length,n,&s_reply);}
static uint32_t query(uint8_t command,uint8_t n)
{uint8_t q[2]={command,0x6b};return exchange(q,2U,n);}
static uint32_t control(const uint8_t *q,uint8_t n)
{return exchange(q,n,3U)==1U && (s_reply.Data[1]==2U || (q[0]==0xfd && s_reply.Data[1]==0x9f));}
#endif
static void fail(uint32_t cause)
{g_startup_extension_reference_valid=0U;if(g_startup_axes_state!=5U || !g_startup_axes_cause)g_startup_axes_cause=cause;g_startup_axes_state=5U;g_startup_axes_vm_request=0U;g_dm_jog_request=3U;}
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
    if(STARTUP_EXTENSION_HOME_ENABLE>1U || STARTUP_EXTENSION_HOME_TEST_ONLY>1U ||
       (STARTUP_EXTENSION_HOME_TEST_ONLY && !STARTUP_EXTENSION_HOME_ENABLE) || !STARTUP_EXTENSION_HOME_RPM ||
       STARTUP_EXTENSION_HOME_RPM>30U || STARTUP_EXTENSION_HOME_TIMEOUT_MS!=11000U ||
       !(STARTUP_EXTENSION_ZERO_DEG>=-10000.0f && STARTUP_EXTENSION_ZERO_DEG<=10000.0f))return 0U;
    g_startup_extension_zero_deg=STARTUP_EXTENSION_ZERO_DEG;
    s_home_phase=s_home_done=s_home_seen_running=s_home_begin=s_home_poll=0U;
    g_startup_extension_home_state=g_startup_extension_home_active=g_startup_extension_home_notice=0U;
    if(STARTUP_EXTENSION_HOME_ENABLE)g_startup_extension_reference_valid=0U;
    /* 机械零参考及目标由操作者填写；不以默认0和粗标冒充已建立坐标。 */
    if(!(VEHICLE_EXTENSION_PULSES_PER_MM>0.0f && VEHICLE_EXTENSION_PULSES_PER_MM<100000.0f) ||
       VEHICLE_X_ACCEL_RPM_S==0U || VEHICLE_X_ACCEL_RPM_S>65535U ||
       !(STARTUP_EXTENSION_RPM>0.0f && STARTUP_EXTENSION_RPM<=VEHICLE_EXTENSION_RPM) ||
       !(STARTUP_EXTENSION_TARGET_MM>=0.0f && STARTUP_EXTENSION_TARGET_MM<=50.0f) ||
       !(STARTUP_EXTENSION_ZERO_DEG>=-10000.0f && STARTUP_EXTENSION_ZERO_DEG<=10000.0f) ||
       !(STARTUP_EXTENSION_EXTEND_SIGN==1 || STARTUP_EXTENSION_EXTEND_SIGN==-1))
    {g_startup_axes_state=7U;g_startup_axes_cause=1U;return 0U;}
    s_target=g_startup_extension_zero_deg+STARTUP_EXTENSION_EXTEND_SIGN*
        STARTUP_EXTENSION_TARGET_MM*VEHICLE_EXTENSION_PULSES_PER_MM*360.0f/3200.0f;
    g_startup_extension_target_deg=s_target;
    g_startup_axes_state=1U;g_startup_axes_cause=0U;g_startup_axes_vm_request=1U;
    s_begin_ms=s_last_service=(uint32_t)xTaskGetTickCount();
    s_phase=s_dm_ready=s_ext_ready=s_started=s_ext_done=s_settled=s_monitor=s_send_move=s_stall_active=0U;
    g_dm_startup_go=0U;return 1U;
#endif
}
void StartupAxes_Cancel(void)
{g_startup_extension_reference_valid=0U;if(g_startup_axes_state!=5U)g_startup_axes_state=6U;g_startup_axes_vm_request=0U;g_dm_jog_request=3U;}
void StartupAxes_Safety(uint32_t healthy)
{
    uint32_t now=(uint32_t)xTaskGetTickCount();ChassisAutoStatus backend;
    (void)ChassisAuto_ReadStatus(&backend);
    if(g_startup_axes_state>=1U && g_startup_axes_state<=4U) {
        if(g_dm_jog_first_fault){fail(7U);return;}
        if(backend.state>=CHASSIS_AUTO_FAULT || backend.cause){fail(10U);return;}
        if(!healthy || (uint32_t)(now-s_last_service)>150U)fail(2U);
        else if(g_startup_axes_state!=4U && (uint32_t)(now-s_begin_ms)>=15000U)fail(3U);
    }
}
#if !APP_MOTOR_OFFLINE_ONLY
static uint32_t read_position(void)
{
    uint32_t raw;g_startup_extension_feedback_valid=0U;
    if(query(0x36U,7U)!=1U || s_reply.Data[1]>1U)return 0U;
    raw=((uint32_t)s_reply.Data[2]<<24)|((uint32_t)s_reply.Data[3]<<16)|((uint32_t)s_reply.Data[4]<<8)|s_reply.Data[5];
    g_startup_extension_current_deg=(s_reply.Data[1]?-1.0f:1.0f)*(float)raw*0.1f;g_startup_extension_feedback_valid=1U;return 1U;
}
#endif
#include "startup_extension_home.inc"

void StartupAxes_Service(uint32_t platform_ready,uint32_t can_guard_ready)
{
#if !APP_MOTOR_OFFLINE_ONLY
    uint32_t now=(uint32_t)xTaskGetTickCount(),angle,scale,speed,home_result;float delta;
    PowerCommandStatus power;const uint8_t enable[5]={0xf3,0xab,1,0,0x6b};
    const uint8_t stop[4]={0xfe,0x98,0,0x6b};
    if(!g_startup_axes_state || g_startup_axes_state==7U || g_startup_axes_state==8U)return;
    s_last_service=now;
    if(g_startup_axes_state==5U || g_startup_axes_state==6U) {
        if(g_startup_extension_home_active){
            const uint8_t abort_home[3]={0x9c,0x48,0x6b};
            uint32_t abort_ack,stop_ok;
            g_startup_extension_home_abort_result=exchange(abort_home,3U,3U);
            abort_ack=g_startup_extension_home_abort_result==1U && s_reply.Data[1]==2U;
            stop_ok=control(stop,4U);
            if(stop_ok && (abort_ack ||
               (query(0x3bU,3U)==1U && (s_reply.Data[1]&3U)==3U && !(s_reply.Data[1]&0x3cU) &&
                query(0x35U,5U)==1U && s_reply.Data[1]<=1U && !s_reply.Data[2] && !s_reply.Data[3])))
                g_startup_extension_home_active=0U;
        }
        if(g_dm_jog_request || DM_CAN2_JogIsBusy() || g_dm_posvel_holding)DM_CAN2_ModuleService(0U,0U);
        if(s_started){(void)control(stop,4U);s_started=0U;}return;
    }
    if(!platform_ready){fail(2U);return;}
    Power_ReadCommandStatus(&power);
    if(!power.active || !can_guard_ready)return;
    if(g_startup_axes_state==1U){g_startup_axes_state=2U;}
    if(g_startup_axes_state==2U) {
        /* 一轮最多一个伸缩交易，DM与CAN1共用同一个任务，无并发FIFO访问。 */
        if(s_phase==0U) {
            if(query(0x1aU,4U)!=1U){fail(4U);return;}
            s_options=s_reply.Data[2];
            if(s_options&3U){fail(5U);return;}s_phase++;return;
        }
        if(s_phase==1U && STARTUP_EXTENSION_HOME_ENABLE && !s_home_done){
            home_result=StartupExtension_HomeStep(now);
            if(!home_result){g_startup_extension_home_state=9U;fail(11U);return;}
            if(home_result==2U)return;
            if(STARTUP_EXTENSION_HOME_TEST_ONLY){
                /* 只测回零不继续转臂/发车；撤VM后不沿用有效机械坐标。 */
                g_startup_axes_state=8U;g_startup_axes_cause=0U;g_startup_axes_vm_request=0U;
                g_startup_extension_reference_valid=0U;return;
            }
        }
        if(s_phase==1U) {
            if(!read_position()){fail(4U);return;}s_start=g_startup_extension_current_deg;
            if(!g_startup_extension_reference_valid){
                uint8_t read_z[2]={0x1a,0x6b};uint32_t zraw;
                extern volatile float g_vehicle_lift_current_deg;
                extern volatile uint32_t g_vehicle_lift_feedback_valid;
                g_vehicle_lift_feedback_valid=0U;
                if(VehicleArm_Exchange(5U,read_z,2U,4U,&s_reply)==1U && !(s_reply.Data[2]&3U)){
                    read_z[0]=0x36;
                    if(VehicleArm_Exchange(5U,read_z,2U,7U,&s_reply)==1U && s_reply.Data[1]<=1U){
                    zraw=((uint32_t)s_reply.Data[2]<<24)|((uint32_t)s_reply.Data[3]<<16)|((uint32_t)s_reply.Data[4]<<8)|s_reply.Data[5];
                    g_vehicle_lift_current_deg=(s_reply.Data[1]?-1.0f:1.0f)*(float)zraw*0.1f;g_vehicle_lift_feedback_valid=1U;
                    }
                }
                g_startup_axes_state=8U;g_startup_axes_cause=1U;g_startup_axes_vm_request=0U;return;}
            delta=s_target-s_start;
            if(!(delta>=-180.0f && delta<=180.0f)){fail(6U);return;}
            scale=(s_options&0x80U)?100U:10U;
            angle=(uint32_t)(fabsf(delta)*(float)scale+0.5f);
            memset(s_move,0,sizeof(s_move));s_move[0]=0xfd;s_move[1]=delta<0.0f;
            s_move[2]=s_move[4]=(uint8_t)(VEHICLE_X_ACCEL_RPM_S>>8);s_move[3]=s_move[5]=(uint8_t)VEHICLE_X_ACCEL_RPM_S;
            speed=(uint32_t)(STARTUP_EXTENSION_RPM*10.0f+0.5f);s_move[6]=(uint8_t)(speed>>8);s_move[7]=(uint8_t)speed; /* 默认106.0RPM、45055RPM/s。 */
            s_move[8]=(uint8_t)(angle>>24);s_move[9]=(uint8_t)(angle>>16);s_move[10]=(uint8_t)(angle>>8);s_move[11]=(uint8_t)angle;
            s_move[12]=2U;s_move[14]=0x6b;
            s_send_move=fabsf(delta)>0.2f;
            g_dm_point_enable=1U;g_dm_posvel_speed_deg_s=DM_CAN2_POSVEL_SPEED_DEG_S*STARTUP_DM_SPEED_SCALE;
            g_dm_jog_request=6U;s_phase++;s_started=1U;
        }
        DM_CAN2_ModuleService(1U,0U);
        if(g_dm_jog_state==8U && !(g_vehicle_arm_dm_owned && g_dm_jog_result==2U)){fail(7U);return;}
        if(!s_ext_ready) {if(!control(enable,5U)){fail(4U);return;}s_ext_ready=1U;}
        if(g_dm_jog_state==3U && g_dm_can2_motor_state==1U)s_dm_ready=1U;
        if(s_dm_ready && s_ext_ready) {
            /* 两轴准备屏障通过后同一服务轮释放；CAN1/CAN2并行运动，非硬件同步。 */
            g_dm_startup_go=1U;
            if(s_send_move && !control(s_move,15U)){fail(4U);return;}
            g_startup_axes_state=3U;s_poll_ms=now;
        }
        return;
    }
    DM_CAN2_ModuleService(1U,0U);
    if(g_dm_jog_state==8U && !(g_vehicle_arm_dm_owned && g_dm_jog_result==2U)){fail(7U);return;}
    if(g_vehicle_arm_axis_owned!=6U && (uint32_t)(now-s_poll_ms)>=50U) {
        s_poll_ms=now;
        if(s_monitor==0U) {
            if(query(0x3aU,3U)!=1U || !(s_reply.Data[1]&1U) || (s_reply.Data[1]&0x08U)) {fail(8U);return;}
            s_flags=s_reply.Data[1];
            if(s_flags&4U){if(!s_stall_active){s_stall_active=1U;s_stall_since=now;}else if(now-s_stall_since>=2000U){fail(8U);return;}}
            else s_stall_active=0U;
        } else if(s_monitor==1U) {
            if(!read_position()){fail(4U);return;}
            if(fabsf(g_startup_extension_current_deg-s_target)>2.0f && g_startup_axes_state==4U){fail(9U);return;}
        } else {
            if(query(0x35U,5U)!=1U){fail(4U);return;}
            if((s_reply.Data[2] || s_reply.Data[3]) || (s_flags&0x0cU) || (s_send_move && !(s_flags&2U)) || fabsf(g_startup_extension_current_deg-s_target)>0.3f){s_settled=0U;s_ext_done=0U;}
            else if(++s_settled>=3U)s_ext_done=1U;
        }
        s_monitor=(s_monitor+1U)%3U;
    }
    if(g_startup_axes_state==3U && s_ext_done && g_dm_jog_state==7U &&
       g_dm_jog_result==1U && g_dm_posvel_holding) {
        g_startup_axes_state=4U;g_dm_posvel_speed_deg_s=DM_CAN2_POSVEL_SPEED_DEG_S;
    }
#else
    (void)platform_ready;(void)can_guard_ready;
#endif
}

void StartupAxes_SetExtensionHoldTarget(float target){
#if !APP_MOTOR_OFFLINE_ONLY
    s_target=target;g_startup_extension_target_deg=target;
#else
    (void)target;
#endif
}
