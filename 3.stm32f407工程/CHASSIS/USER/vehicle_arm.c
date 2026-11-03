#include "vehicle_arm.h"
#include "app_config.h"
#include "startup_axes.h"
#include "dm_can2.h"
#include "gripper.h"
#include "carrier_pwm.h"
#include "carrier_config.h"
#include "can.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <string.h>
volatile uint32_t g_vehicle_arm_state=0U,g_vehicle_arm_cause=0U;
volatile uint32_t g_vehicle_arm_dm_owned=0U,g_vehicle_arm_axis_owned=0U;
volatile float g_vehicle_lift_current_deg=0.0f;
volatile uint32_t g_vehicle_lift_feedback_valid=0U;
volatile uint32_t g_vehicle_lift_reference_valid=VEHICLE_LIFT_REFERENCE_VALID;
volatile uint32_t g_vehicle_arm_stop_pending=0U,g_vehicle_arm_stop_result=0U;
#if !APP_MOTOR_OFFLINE_ONLY
static VehicleArmRow s_row;
static uint32_t s_begin,s_phase,s_item,s_samples,s_wait,s_cycle,s_stall_active,s_stall_ms,s_flags,s_did_move;
static float s_goal,s_before,s_current;
static uint8_t s_address,s_options,s_move[15];
static CanRxMsg s_rx;
#endif
extern volatile uint32_t g_dm_integrated_owner;
extern void StartupAxes_SetExtensionHoldTarget(float target);
uint32_t VehicleArm_Busy(void){return g_vehicle_arm_state==1U;}
static void arm_fail(uint32_t why)
{g_vehicle_arm_stop_pending=g_vehicle_arm_axis_owned;if(g_vehicle_arm_state!=3U || !g_vehicle_arm_cause)g_vehicle_arm_cause=why;g_vehicle_arm_state=3U;g_dm_jog_request=3U;StartupAxes_Cancel();}
void VehicleArm_Cancel(void)
{arm_fail(2U);}
#if !APP_MOTOR_OFFLINE_ONLY
static uint32_t query(uint8_t command,uint8_t n)
{uint8_t q[2]={command,0x6b};return VehicleArm_Exchange(s_address,q,2U,n,&s_rx)==1U;}
static uint32_t position(void)
{
    uint32_t raw;
    if(!query(0x36,7U) || s_rx.Data[1]>1U)return 0U;
    raw=((uint32_t)s_rx.Data[2]<<24)|((uint32_t)s_rx.Data[3]<<16)|((uint32_t)s_rx.Data[4]<<8)|s_rx.Data[5];
    s_current=(s_rx.Data[1]?-1.0f:1.0f)*(float)raw*0.1f;
    if(s_address==5U){g_vehicle_lift_current_deg=s_current;g_vehicle_lift_feedback_valid=1U;}
    return 1U;
}
#endif
uint32_t VehicleArm_Begin(const VehicleArmRow *row,uint32_t item)
{
#if APP_MOTOR_OFFLINE_ONLY
    (void)row;(void)item;return 0U;
#else
    if(!row || VehicleArm_Busy() || !StartupAxes_Holding())return 0U;
    s_row=*row;s_item=item;s_phase=s_samples=s_wait=s_stall_active=s_did_move=0U;s_cycle=0U;
    s_begin=(uint32_t)xTaskGetTickCount();g_vehicle_arm_cause=0U;
    g_vehicle_arm_state=1U;return 1U;
#endif
}
void VehicleArm_Service(void)
{
#if !APP_MOTOR_OFFLINE_ONLY
    uint32_t now=(uint32_t)xTaskGetTickCount(),pulse,units,speed;
    float target,delta;uint8_t q[5]={0xf3,0xab,1,0,0x6b};
    if(g_vehicle_arm_stop_pending==5U || g_vehicle_arm_stop_pending==6U) {
        const uint8_t stop[4]={0xfe,0x98,0,0x6b};
        g_vehicle_arm_stop_result=VehicleArm_Exchange((uint8_t)g_vehicle_arm_stop_pending,stop,4U,3U,&s_rx);
        g_vehicle_arm_stop_pending=0U;return;
    }
    if(!VehicleArm_Busy())return;
    if(!StartupAxes_Holding() || (VEHICLE_AXIS_TIMEOUT_MS && now-s_begin>=VEHICLE_AXIS_TIMEOUT_MS)){arm_fail(3U);return;}
    if(s_row.op==2U) {
        if(!s_phase) {
            if(!(s_row.target>=0 && s_row.target<=180 && s_row.speed>0 && s_row.speed<=300)){arm_fail(4U);return;}
            g_vehicle_arm_dm_owned=g_dm_integrated_owner=1U;
            g_dm_point_enable=1U;g_dm_posvel_speed_deg_s=s_row.speed;
            g_dm_pick_target_deg=g_dm_place_target_deg=s_row.target;
            g_dm_jog_request=s_row.target>=g_dm_can2_logical_position_deg?4U:5U;
            s_cycle=g_dm_can2_read_cycle;s_phase=1U;return;
        }
        if(g_dm_jog_state==8U && g_dm_jog_result!=2U){arm_fail(5U);return;}
        if(g_dm_can2_read_cycle!=s_cycle && g_dm_jog_state==7U && g_dm_jog_result==1U &&
           g_dm_posvel_holding && now-g_dm_can2_feedback_tick<100U &&
           fabsf(g_dm_can2_logical_position_deg-s_row.target)<=3.0f) {
            g_vehicle_arm_dm_owned=0U;g_vehicle_arm_state=2U;
        }return;
    }
    if(s_row.op==4U || s_row.op==5U || s_row.op==6U) {
        if(!s_phase) {
            if(s_row.op==4U && !jiazhua(s_row.target)){arm_fail(4U);return;}
            if(s_row.op==5U) {
                target=s_row.target;
                if(target==-2.0f)target=s_item==0?VEHICLE_CARRIER_SLOT1_DEG:(s_item==1?VEHICLE_CARRIER_SLOT2_DEG:VEHICLE_CARRIER_SLOT3_DEG);
                if(!(target>=0 && target<=270)){arm_fail(4U);return;}
                pulse=CARRIER_PULSE_MIN_US+(uint32_t)(target*(CARRIER_PULSE_MAX_US-CARRIER_PULSE_MIN_US)/270.0f+0.5f);
                if(!CarrierPwm_Write(pulse)){arm_fail(4U);return;}
            }
            s_wait=now;s_phase=1U;
        }
        if(now-s_wait>=s_row.settle_ms)g_vehicle_arm_state=2U;
        return;
    }
    if(s_row.op!=1U && s_row.op!=3U){arm_fail(4U);return;}
    if(s_phase==0U) {
        if(s_row.op==1U) {
            s_address=5U;
            if(g_vehicle_lift_reference_valid!=1U || !(s_row.target>=0 && s_row.target<=VEHICLE_LIFT_MAX_TARGET_PULSES))
            {g_vehicle_arm_axis_owned=5U;(void)position();arm_fail(6U);return;}
            s_goal=VEHICLE_LIFT_TOP_DEG+VEHICLE_LIFT_DOWN_SIGN*s_row.target*360.0f/3200.0f;
        } else {
            s_address=6U;
            if(!(s_row.target>=0 && s_row.target<=50)){arm_fail(4U);return;}
            s_goal=STARTUP_EXTENSION_ZERO_DEG+STARTUP_EXTENSION_EXTEND_SIGN*s_row.target*VEHICLE_EXTENSION_PULSES_PER_MM*360.0f/3200.0f;
        }
        if(!(s_row.speed>0 && s_row.speed<=(s_address==5U?VEHICLE_LIFT_RPM:VEHICLE_EXTENSION_RPM))){arm_fail(4U);return;}
        g_vehicle_arm_axis_owned=s_address;
        if(!query(0x1a,4U) || (s_rx.Data[2]&3U)){arm_fail(7U);return;}
        s_options=s_rx.Data[2];s_phase=1U;return;
    }
    if(s_phase==1U) {
        if(!position()){arm_fail(7U);return;}s_before=s_current;
        delta=s_goal-s_current;
        if(!(fabsf(delta)<=(s_address==5U?VEHICLE_LIFT_MAX_TARGET_PULSES*360.0f/3200.0f:180.0f))){arm_fail(8U);return;}
        s_did_move=fabsf(delta)>0.2f;
        units=(uint32_t)(fabsf(delta)*((s_options&0x80U)?100.0f:10.0f)+0.5f);
        speed=(uint32_t)(s_row.speed*10.0f+0.5f);
        memset(s_move,0,15);s_move[0]=0xfd;s_move[1]=delta<0;
        s_move[2]=s_move[4]=(uint8_t)(VEHICLE_X_ACCEL_RPM_S>>8);s_move[3]=s_move[5]=(uint8_t)VEHICLE_X_ACCEL_RPM_S;
        s_move[6]=(uint8_t)(speed>>8);s_move[7]=(uint8_t)speed;
        s_move[8]=units>>24;s_move[9]=units>>16;s_move[10]=units>>8;s_move[11]=units;
        s_move[12]=2;s_move[14]=0x6b;s_phase=2U;return;
    }
    if(s_phase==2U) {
        if(!query(0x35,5U) || s_rx.Data[2] || s_rx.Data[3]){arm_fail(7U);return;}
        s_phase=3U;return;
    }
    if(s_phase==3U) {
        if(VehicleArm_Exchange(s_address,q,5U,3U,&s_rx)!=1U || s_rx.Data[1]!=2U){arm_fail(7U);return;}
        s_phase=4U;return;
    }
    if(s_phase==4U) {
        if(s_did_move &&
           (VehicleArm_Exchange(s_address,s_move,15U,3U,&s_rx)!=1U || (s_rx.Data[1]!=2U && s_rx.Data[1]!=0x9fU)))
        {arm_fail(7U);return;}s_phase=5U;return;
    }
    if(s_phase==5U) {
        if(!query(0x3a,3U) || !(s_rx.Data[1]&1U) || (s_rx.Data[1]&0x08U)){arm_fail(9U);return;}
        s_flags=s_rx.Data[1];
        if(s_flags&4U){if(s_address==5U){arm_fail(9U);return;}if(!s_stall_active){s_stall_active=1;s_stall_ms=now;}else if(now-s_stall_ms>=2000U){arm_fail(9U);return;}}
        else s_stall_active=0U;
        s_phase=6U;return;
    }
    if(s_phase==6U) {
        if(!position()){arm_fail(7U);return;}
        if(s_current<fminf(s_before,s_goal)-2.0f || s_current>fmaxf(s_before,s_goal)+2.0f){arm_fail(8U);return;}
        s_phase=7U;return;
    }
    if(!query(0x35,5U)){arm_fail(7U);return;}
    if(!s_rx.Data[2] && !s_rx.Data[3] && !(s_flags&0x0cU) && (!s_did_move || (s_flags&2U)) && fabsf(s_current-s_goal)<=0.3f)s_samples++;else s_samples=0;
    if(s_samples>=3U) {
        if(s_address==6U)StartupAxes_SetExtensionHoldTarget(s_goal);
        g_vehicle_arm_axis_owned=0U;g_vehicle_arm_state=2U;
    } else s_phase=5U;
#endif
}
