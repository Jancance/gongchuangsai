
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

int main(void){
#if APP_MOTOR_OFFLINE_ONLY
 ck(!StartupAxes_Begin());StartupAxes_Service(1,1);ck(!starts && !pwm_calls && !dm_calls && !g_startup_axes_vm_request);
#else
 reset();steps(20);ck(!command_count && !pwm_calls && !dm_calls && !g_startup_axes_vm_request);
 ck(StartupAxes_Begin());ck(!StartupAxes_Begin());steps(10);
 ck(eh_phase==5 && !starts && !pwm_calls && !g_dm_startup_go);
 dm_prepared=1;step();ck(starts==1 && pwm_calls==1 && g_startup_axes_state==3);
 ck(g_dm_posvel_speed_deg_s==150.0f);
 dm_done=1;home_flags=7;steps(4);home_flags=3;steps(12);
 ck(g_extension_home_reported && g_group_extension_state==4 && !StartupAxes_Holding());
 steps(80);ck(StartupAxes_Holding() && g_dm_posvel_holding && starts==1 && pwm_calls==1);
 StartupAxes_Cancel();step();ck(g_startup_axes_state==6 && !g_startup_axes_vm_request && !g_dm_posvel_holding);ck(!StartupAxes_Begin());
 reset();bad_params=1;dm_prepared=1;ck(StartupAxes_Begin());steps(5);ck(g_startup_axes_state==5 && !starts && !pwm_calls && !g_startup_axes_vm_request);
 reset();bad_pwm_params=1;ck(!StartupAxes_Begin() && !command_count && !dm_calls && !pwm_calls);
 reset();pwm_ok=0;dm_prepared=1;ck(StartupAxes_Begin());steps(8);ck(!starts && pwm_calls==1 && !g_dm_startup_go && g_startup_axes_cause==12);
 reset();dm_bad=1;ck(StartupAxes_Begin());steps(4);ck(!starts && !pwm_calls && g_startup_axes_cause==7 && !g_startup_axes_vm_request);
 reset();release();dm_bad=1;step();step();ck(!g_startup_axes_vm_request && aborts==1 && stops==1 && !g_dm_posvel_holding);
 reset();release();StartupAxes_Cancel();step();ck(aborts==1 && stops==1 && !g_startup_axes_vm_request && !g_dm_posvel_holding);
 reset();release();StartupAxes_Safety(0);step();ck(g_startup_axes_state==5 && !g_startup_axes_vm_request && stops==1);
 reset();release();tick+=151;StartupAxes_Safety(1);step();ck(g_startup_axes_state==5 && !g_startup_axes_vm_request && stops==1);
 reset();dm_prepared=1;fail_command=0x9a;ck(StartupAxes_Begin());steps(8);ck(starts==1 && !g_startup_axes_vm_request && stops==1 && !g_dm_posvel_holding && !dm_run_calls);
 reset();release();home_flags=0x23;steps(5);ck(g_startup_axes_state==5 && !g_startup_axes_vm_request && !g_dm_posvel_holding);
 reset();ck(StartupAxes_Begin());steps(130);ck(g_startup_axes_state==5 && !starts && !pwm_calls);
 reset();release();powered=0;step();step();ck(g_startup_axes_state==5 && !g_startup_axes_vm_request && !g_dm_posvel_holding);
 reset();pwm_cancel=1;dm_prepared=1;ck(StartupAxes_Begin());steps(8);ck(!starts && !g_startup_group_go && g_startup_axes_state==6 && !g_startup_axes_vm_request);
 reset();home_cancel=1;dm_prepared=1;ck(StartupAxes_Begin());steps(8);ck(starts==1 && g_startup_axes_state==6 && !g_startup_axes_vm_request && stops==1 && !g_dm_posvel_holding && !dm_run_calls);
 reset();release();StartupAxes_Cancel();fail_command=0x9c;step();ck(eh_stop_pending && !StartupAxes_Begin());
#endif
 printf("startup group checks=%u failures=%u\n",checks,failures);return failures?1:0;
}
