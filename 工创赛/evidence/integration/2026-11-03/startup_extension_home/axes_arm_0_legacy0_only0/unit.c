
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#define APP_CONFIG_H
#define CHASSIS_AUTO_TEST_H
#define CHASSIS_AUTO_DONE 9U
#define CHASSIS_AUTO_FAULT 10U
typedef struct {uint32_t state,active,result,cause,rotation_stop_mask;} ChassisAutoStatus;
static ChassisAutoStatus backend;
uint32_t ChassisAuto_ReadStatus(ChassisAutoStatus *s){*s=backend;return 1;}
volatile uint32_t g_vehicle_arm_dm_owned=0,g_vehicle_arm_axis_owned=0;
volatile float g_vehicle_lift_current_deg=0.0f;
volatile uint32_t g_vehicle_lift_feedback_valid=0U;
uint32_t VehicleArm_Busy(void){return 0;}
static uint32_t tick=100,power_on=1,dm_prepared,dm_done,dm_calls,ext_enable,ext_moves,bad_barrier,fail_query;
static float ext_current=-36.0f;
static uint32_t ext_flags=3,ext_speed;
static uint8_t move_data[15];
volatile uint32_t g_dm_startup_go,g_dm_jog_first_fault;
uint32_t g_dm_point_enable,g_dm_jog_request,g_dm_jog_state;
uint32_t g_dm_can2_motor_state,g_dm_jog_result,g_dm_posvel_holding;
float g_dm_posvel_speed_deg_s;
uint32_t xTaskGetTickCount(void){return tick;}
uint32_t DM_CAN2_JogIsBusy(void){return g_dm_jog_state>0 && g_dm_jog_state<7;}
void DM_CAN2_ModuleService(uint32_t ready,uint32_t busy){
 dm_calls++;
 if(g_dm_jog_request==3){g_dm_jog_request=0;g_dm_jog_state=8;g_dm_posvel_holding=0;return;}
 if(g_dm_jog_request==6){g_dm_jog_request=0;g_dm_jog_state=1;}
 if(dm_prepared && !g_dm_startup_go){g_dm_jog_state=3;g_dm_can2_motor_state=1;}
 if(g_dm_startup_go){g_dm_jog_state=dm_done?7:4;g_dm_jog_result=dm_done;g_dm_posvel_holding=dm_done;}
}
#include "startup_axes.c"
void Power_ReadCommandStatus(PowerCommandStatus *p){p->active=power_on;}
static uint32_t home_started,home_running,home_bad_param,home_no_start_ack,home_abort,home_stale,home_immediate,home_default_mode;
uint32_t StartupAxis_ReadHomeParameters(uint8_t out[17]){
 const uint8_t p[17]={0x22,2,0,0,30,0,0,0x27,0x10,1,0x2c,3,0x20,0,60,0,0x6b};
 memcpy(out,p,17);out[1]=home_default_mode;if(home_bad_param)out[4]=60;return fail_query?5:1;
}
uint32_t StartupAxis_Exchange(const uint8_t *q,uint8_t length,uint8_t n,void *out){
 CanRxMsg *p=out;uint32_t raw=(uint32_t)(fabsf(ext_current)*10.0f+0.5f);memset(p,0,sizeof(*p));
 if(fail_query)return 5;
 if(q[0]==0x9a){home_started++;p->Data[1]=home_immediate?0x9f:2;if(home_immediate)ext_current=0;return home_no_start_ack?5:1;}
 if(q[0]==0x9c){home_abort++;p->Data[1]=2;return 1;}
 if(q[0]==0x3b){p->Data[1]=(home_started && home_running)?7:3;return 1;}
 if(q[0]==0x1a){p->Data[2]=0x80;return 1;}
 if(q[0]==0x36){p->Data[1]=ext_current<0;p->Data[2]=raw>>24;p->Data[3]=raw>>16;p->Data[4]=raw>>8;p->Data[5]=raw;return 1;}
 if(q[0]==0x3a){p->Data[1]=ext_flags;return 1;}
 if(q[0]==0x35){p->Data[2]=ext_speed>>8;p->Data[3]=ext_speed;return 1;}
 if(q[0]==0xf3){ext_enable++;p->Data[1]=2;return 1;}
 if(q[0]==0xfd){ext_moves++;memcpy(move_data,q,15);if(!g_dm_startup_go || !dm_prepared || !ext_enable)bad_barrier++;p->Data[1]=2;return 1;}
 if(q[0]==0xfe){p->Data[1]=2;return 1;}
 return 7;
}
uint32_t VehicleArm_Exchange(uint8_t address,const uint8_t *q,uint8_t n,uint8_t reply_n,void *out){return StartupAxis_Exchange(q,n,reply_n,out);}
volatile uint32_t checks,failed,first_failure;
#define C(x) do{checks++;if(!(x)){failed++;if(!first_failure)first_failure=__LINE__;}}while(0)
static void reset(void){
 tick=100;power_on=1;dm_prepared=dm_done=dm_calls=ext_enable=ext_moves=bad_barrier=fail_query=0;
 ext_current=-36;ext_flags=3;ext_speed=0;g_dm_jog_state=g_dm_can2_motor_state=g_dm_posvel_holding=g_dm_jog_result=0;
 g_dm_jog_request=g_dm_startup_go=g_dm_jog_first_fault=0;g_startup_axes_state=g_startup_axes_vm_request=0;
 memset(&backend,0,sizeof(backend));g_startup_extension_reference_valid=1;
}
static void step(void){tick+=20;StartupAxes_Service(1,1);}
static uint32_t power_enabled,vm_request,g_safety_ready,g_motion_auto_stop_cut_vm,g_wheel_test_cut_vm,g_chassis_auto_cut_vm;
static void Power_Enable(void){power_enabled=1;}
static void Power_Disable(void){power_enabled=0;}
static void actual_power_decision(void)
{
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

static void home_reset(void){
 reset();home_started=home_running=home_bad_param=home_no_start_ack=home_abort=home_stale=home_immediate=home_default_mode=0;
 g_startup_extension_reference_valid=0;
}
static void until_start(void){for(uint32_t i=0;i<20 && !home_started && g_startup_axes_state!=5;i++)step();}
void test_entry(void){
#if APP_MOTOR_OFFLINE_ONLY
 C(!StartupAxes_Begin());C(!g_startup_axes_vm_request);StartupAxes_Service(1,1);C(!ext_enable && !dm_calls && !home_started);
#else
 home_reset();StartupAxes_Service(1,1);C(!home_started && !ext_enable && !dm_calls);
 for(uint32_t mode=0;mode<=5;mode++){
  home_reset();home_default_mode=mode;C(StartupAxes_Begin());until_start();C(home_started==1 && !dm_calls);
 }
 home_reset();home_default_mode=6;C(StartupAxes_Begin());until_start();C(!home_started && !ext_enable && g_startup_axes_state==5);
 home_reset();home_default_mode=255;C(StartupAxes_Begin());until_start();C(!home_started && !ext_enable && g_startup_axes_state==5);
 home_reset();

 C(StartupAxes_Begin());C(!g_startup_extension_reference_valid);until_start();C(home_started==1 && ext_enable==1 && !dm_calls && !ext_moves);
 home_running=1;for(uint32_t i=0;i<5;i++)step();C(g_startup_extension_home_active && !g_startup_extension_reference_valid);
 home_running=0;ext_current=0;for(uint32_t i=0;i<6;i++)step();C(g_startup_extension_home_state==8 && g_startup_extension_reference_valid);
 C(!g_startup_extension_home_active && !ext_moves);dm_prepared=dm_done=1;
 for(uint32_t i=0;i<45;i++)step();C(StartupAxes_Holding() && !ext_moves && home_started==1);
 StartupAxes_Cancel();C(!g_startup_extension_reference_valid);
 home_reset();C(StartupAxes_Begin());home_bad_param=1;until_start();C(g_startup_axes_state==5 && !home_started && !ext_enable && !g_startup_axes_vm_request);
 home_reset();C(StartupAxes_Begin());home_no_start_ack=1;until_start();C(g_startup_axes_state==5 && !g_startup_extension_reference_valid);step();C(home_abort && !g_startup_extension_home_active);
 home_reset();C(StartupAxes_Begin());until_start();for(uint32_t i=0;i<20;i++)step();C(!g_startup_extension_reference_valid && g_startup_extension_home_active);
 tick+=11000;step();C(g_startup_axes_state==5 && !g_startup_axes_vm_request);step();C(home_abort);
 home_reset();home_immediate=1;C(StartupAxes_Begin());until_start();for(uint32_t i=0;i<8;i++)step();C(g_startup_extension_reference_valid && g_startup_extension_home_state==8);
 home_reset();C(StartupAxes_Begin());until_start();g_startup_extension_home_notice=1;ext_current=0;for(uint32_t i=0;i<8;i++)step();C(g_startup_extension_reference_valid);
 home_reset();C(StartupAxes_Begin());until_start();home_running=1;for(uint32_t i=0;i<5;i++)step();home_running=0;ext_current=10001;
 for(uint32_t i=0;i<7;i++)step();C(g_startup_axes_state==5 && !g_startup_extension_reference_valid && !g_startup_axes_vm_request);
 home_reset();C(StartupAxes_Begin());until_start();home_running=1;for(uint32_t i=0;i<5;i++)step();home_running=0;ext_current=-30.4f;
 for(uint32_t i=0;i<8;i++)step();C(g_startup_extension_home_state==8 && g_startup_extension_reference_valid);
 C(fabsf(g_startup_extension_zero_deg+30.4f)<0.01f && fabsf(g_startup_extension_target_deg+30.4f)<0.01f);
 home_reset();C(StartupAxes_Begin());until_start();StartupAxes_Cancel();step();C(home_abort && !g_startup_axes_vm_request && !g_startup_extension_reference_valid);
 home_reset();C(StartupAxes_Begin());until_start();StartupAxes_Safety(0);C(g_startup_axes_state==5 && !g_startup_axes_vm_request);step();C(home_abort);
 home_reset();C(StartupAxes_Begin());until_start();tick+=160;StartupAxes_Safety(1);C(g_startup_axes_state==5 && !g_startup_axes_vm_request);
#endif
}
