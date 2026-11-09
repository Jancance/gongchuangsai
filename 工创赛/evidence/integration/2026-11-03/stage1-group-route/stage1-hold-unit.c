
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "vehicle_parameters.h"
typedef struct {uint8_t Data[8];} CanRxMsg;
typedef struct {uint32_t active;} PowerCommandStatus;
static uint32_t tick,powered=1,command_count,starts,enables,aborts,stops,reads;
static uint32_t bad_params,fail_command,home_flags=3,speed,motor_flags=3,instant_notice;
static uint32_t checks,failures;
static void ck(uint32_t v){checks++;if(!v){failures++;printf("failed check %u\n",checks);}}
uint32_t xTaskGetTickCount(void){return tick;}
void Power_ReadCommandStatus(PowerCommandStatus *p){p->active=powered;}
uint32_t StartupAxis_Exchange(const uint8_t*,uint8_t,uint8_t,void*);
uint32_t StartupAxis_ReadHomeParameters(uint8_t*);

#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define DM_CAN2_POSVEL_SPEED_DEG_S 300.0f
#define CHASSIS_AUTO_DONE 9U
typedef struct {uint32_t active,state,result,cause,rotation_stop_mask;} ChassisAutoStatus;
static ChassisAutoStatus chassis;
void ChassisAuto_ReadStatus(ChassisAutoStatus *p){*p=chassis;}
volatile uint32_t g_dm_startup_go,g_dm_jog_first_fault;
uint32_t g_dm_point_enable,g_dm_jog_request,g_dm_jog_state,g_dm_can2_motor_state,g_dm_jog_result,g_dm_posvel_holding;
float g_dm_posvel_speed_deg_s;
static uint32_t dm_prepared,dm_done,dm_bad,dm_calls,pwm_calls,pwm_ok=1,bad_pwm_params,pwm_cancel,home_cancel,dm_run_calls;
void StartupAxes_Cancel(void);
uint32_t StartupPose_GroupValid(void){return !bad_pwm_params;}
uint32_t StartupPose_GroupCommand(void){pwm_calls++;if(pwm_cancel)StartupAxes_Cancel();return pwm_ok;}
uint32_t DM_CAN2_JogIsBusy(void){return g_dm_jog_state>0 && g_dm_jog_state<7;}
void DM_CAN2_ModuleService(uint32_t ready,uint32_t busy){
 (void)ready;(void)busy;dm_calls++;
 if(g_dm_jog_request==3){g_dm_jog_request=0;g_dm_jog_state=8;g_dm_posvel_holding=0;return;}
 if(dm_bad){g_dm_jog_state=8;g_dm_jog_first_fault=14;return;}
 if(g_dm_jog_request==6){g_dm_jog_request=0;g_dm_jog_state=1;}
 if(dm_prepared && !g_dm_startup_go){g_dm_jog_state=3;g_dm_can2_motor_state=1;}
 if(g_dm_startup_go){dm_run_calls++;g_dm_jog_state=dm_done?7:4;g_dm_jog_result=dm_done;g_dm_posvel_holding=dm_done;}
}
#include "startup_group.inc"

uint32_t StartupAxis_ReadHomeParameters(uint8_t p[17]){
 reads++;memset(p,0,17);p[0]=0x22;p[3]=0;p[4]=bad_params?31:30;
 p[7]=0x27;p[8]=0x10;p[9]=1;p[10]=0x2c;p[11]=3;p[12]=0x20;p[14]=60;p[16]=0x6b;return 1;
}
uint32_t StartupAxis_Exchange(const uint8_t *q,uint8_t length,uint8_t reply_length,void *out){
 CanRxMsg *p=(CanRxMsg*)out;(void)reply_length;command_count++;memset(p,0,sizeof(*p));p->Data[0]=q[0];p->Data[1]=2;
 if(q[0]==0x9a){starts++;ck(g_startup_group_go && g_dm_startup_go && pwm_calls==1 && g_startup_axes_state==2);g_extension_home_capture=1;g_extension_home_done_notice=instant_notice;}
 else if(q[0]==0xf3)enables++;
 else if(q[0]==0x9c)aborts++;
 else if(q[0]==0xfe)stops++;
 else if(q[0]==0x3b)p->Data[1]=home_flags;
 else if(q[0]==0x3a)p->Data[1]=motor_flags;
 else if(q[0]==0x35)p->Data[2]=speed;
 else if(q[0]==0x36){p->Data[1]=0;p->Data[5]=123;}
 if(q[0]==0x9a && home_cancel)StartupAxes_Cancel();
 if(q[0]==0x9a)ck(length==4 && q[1]==2 && q[2]==0 && q[3]==0x6b);
 return fail_command==q[0]?5:1;
}

static void reset(void){
 tick=100;powered=1;command_count=starts=enables=aborts=stops=reads=0;
 bad_params=fail_command=speed=instant_notice=0;home_flags=motor_flags=3;
 g_startup_axes_state=g_startup_axes_cause=g_startup_axes_vm_request=0;
 g_group_extension_state=g_group_extension_cause=g_group_extension_vm_request=0;
 g_extension_home_state=g_extension_home_result=g_extension_home_reported=0;
 g_extension_home_capture=g_extension_home_done_notice=0;
 eh_begin=eh_motion=eh_last=eh_poll=eh_started=eh_seen=eh_stop_pending=eh_phase=0;
 g_startup_group_go=g_startup_group_pwm_result=g_startup_group_release_ms=0;
 sg_begin=sg_last=sg_dm_started=0;
 dm_prepared=dm_done=dm_bad=dm_calls=pwm_calls=bad_pwm_params=pwm_cancel=home_cancel=dm_run_calls=0;pwm_ok=1;
 g_dm_jog_state=g_dm_can2_motor_state=g_dm_jog_result=g_dm_posvel_holding=g_dm_jog_first_fault=0;
 g_dm_jog_request=g_dm_startup_go=0;
}
static void step(void){tick+=20;StartupAxes_Service(1,1);StartupAxes_Safety(1);}
static void steps(unsigned count){for(unsigned i=0;i<count;i++)step();}
static void release(void){dm_prepared=1;ck(StartupAxes_Begin());steps(8);ck(starts==1 && pwm_calls==1 && g_startup_group_go);}

static uint32_t g_safety_ready=1,vm_request,power_enabled,g_motion_auto_stop_cut_vm,g_wheel_test_cut_vm,g_chassis_auto_cut_vm;
static void Power_Enable(void){power_enabled=1;}
static void Power_Disable(void){power_enabled=0;}
static void actual_power_decision(void){
    if ((g_safety_ready == 1U) && (vm_request == 1U || g_startup_axes_vm_request==1U) &&
        (g_motion_auto_stop_cut_vm == 0U) &&
        (g_wheel_test_cut_vm == 0U) &&
        ((g_chassis_auto_cut_vm == 0U) || StartupAxes_NormalStopHold()))
    {
        Power_Enable();
    }
    else
    {
        Power_Disable();
    }

}
int main(void){
 reset();release();dm_done=1;home_flags=7;steps(4);home_flags=3;steps(100);ck(StartupAxes_Holding());
 chassis=(ChassisAutoStatus){0,9,1,0,15};g_chassis_auto_cut_vm=1;
 actual_power_decision();ck(power_enabled && StartupAxes_NormalStopHold());
 chassis.cause=1;actual_power_decision();ck(!power_enabled);chassis.cause=0;
 chassis.active=1;actual_power_decision();ck(!power_enabled);chassis.active=0;
 chassis.rotation_stop_mask=7;actual_power_decision();ck(!power_enabled);chassis.rotation_stop_mask=15;
 chassis.state=8;actual_power_decision();ck(!power_enabled);chassis.state=9;
 g_safety_ready=0;actual_power_decision();ck(!power_enabled);g_safety_ready=1;
 g_motion_auto_stop_cut_vm=1;actual_power_decision();ck(!power_enabled);g_motion_auto_stop_cut_vm=0;
 g_wheel_test_cut_vm=1;actual_power_decision();ck(!power_enabled);g_wheel_test_cut_vm=0;
 StartupAxes_Cancel();actual_power_decision();ck(!power_enabled);
 printf("group route hold checks=%u failures=%u\n",checks,failures);return failures?1:0;
}
