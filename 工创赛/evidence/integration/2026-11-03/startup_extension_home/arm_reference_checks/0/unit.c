
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#define APP_CONFIG_H
uint32_t g_dm_jog_request,g_dm_jog_state,g_dm_jog_result,g_dm_posvel_holding,g_dm_point_enable,g_dm_can2_read_cycle,g_dm_can2_feedback_tick;
float g_dm_posvel_speed_deg_s,g_dm_pick_target_deg,g_dm_place_target_deg,g_dm_can2_logical_position_deg;
volatile uint32_t g_dm_integrated_owner;
volatile float g_startup_extension_zero_deg=0.0f;
static uint32_t tick=100,holding=1,moves,enable_calls,stop_calls,grips,pwm_calls,io_fail;
static uint8_t last_move[15],flags=3;
static float position5,position6,held_extension,last_grip;
static uint32_t pwm;
uint32_t xTaskGetTickCount(void){return tick;}
uint32_t StartupAxes_Holding(void){return holding;}
void StartupAxes_Cancel(void){holding=0;}
void StartupAxes_SetExtensionHoldTarget(float v){held_extension=v;}
void Gripper_Init(void){}
uint8_t jiazhua(float v){if(!(v>=0 && v<=270))return 0;grips++;last_grip=v;return 1;}
uint32_t CarrierPwm_Write(uint32_t p){pwm_calls++;pwm=p;return 1;}
#include "vehicle_arm.c"
uint32_t VehicleArm_Exchange(uint8_t a,const uint8_t *q,uint8_t n,uint8_t rn,void *out){
 CanRxMsg *r=out;float *p=a==5?&position5:&position6;uint32_t raw=(uint32_t)(fabsf(*p)*10+0.5f);memset(r,0,sizeof(*r));
 if(io_fail)return 5;
 if(q[0]==0x1a){r->Data[2]=0x80;return 1;}
 if(q[0]==0x36){r->Data[1]=*p<0;r->Data[2]=raw>>24;r->Data[3]=raw>>16;r->Data[4]=raw>>8;r->Data[5]=raw;return 1;}
 if(q[0]==0x35)return 1;
 if(q[0]==0x3a){r->Data[1]=flags;return 1;}
 if(q[0]==0xf3){enable_calls++;r->Data[1]=2;return 1;}
 if(q[0]==0xfd){moves++;memcpy(last_move,q,15);raw=((uint32_t)q[8]<<24)|((uint32_t)q[9]<<16)|((uint32_t)q[10]<<8)|q[11];*p+=(q[1]?-1.0f:1.0f)*(float)raw/100.0f;r->Data[1]=2;return 1;}
 if(q[0]==0xfe){stop_calls++;r->Data[1]=2;return 1;}return 7;
}
volatile uint32_t checks,failures,first_failure;
#define C(x) do{checks++;if(!(x)){failures++;if(!first_failure)first_failure=__LINE__;}}while(0)
static void reset(void){tick=100;holding=1;moves=enable_calls=stop_calls=grips=pwm_calls=io_fail=0;flags=3;
 g_startup_extension_zero_deg=0;position5=position6=0;g_vehicle_arm_state=g_vehicle_arm_axis_owned=g_vehicle_arm_stop_pending=0;
 g_vehicle_lift_reference_valid=1;g_dm_jog_state=g_dm_jog_result=g_dm_posvel_holding=g_dm_can2_read_cycle=0;}
static void steps(uint32_t n){while(n--){tick+=10;VehicleArm_Service();}}
void test_entry(void){VehicleArmRow r;
#if APP_MOTOR_OFFLINE_ONLY
 r=(VehicleArmRow){4,140,0,0};C(!VehicleArm_Begin(&r,0));VehicleArm_Service();C(!moves && !pwm_calls && !grips);
#else
 reset();g_startup_extension_zero_deg=-30.4f;position6=-30.4f;
 r=(VehicleArmRow){3,10,106,0};C(VehicleArm_Begin(&r,0));steps(20);
 C(g_vehicle_arm_state==2 && fabsf(position6+66.4f)<0.02f && fabsf(held_extension+66.4f)<0.02f);
 r=(VehicleArmRow){3,0,106,0};C(VehicleArm_Begin(&r,0));steps(20);
 C(g_vehicle_arm_state==2 && fabsf(position6+30.4f)<0.02f && moves==2);
 C(VehicleArm_Begin(&r,0));steps(20);C(g_vehicle_arm_state==2 && moves==2);

 reset();r=(VehicleArmRow){1,3200,1000,0};C(VehicleArm_Begin(&r,0));steps(20);C(g_vehicle_arm_state==2 && position5==360 && moves==1);
 C(last_move[1]==0 && last_move[12]==2 && last_move[6]==0x27 && last_move[7]==0x10);
 r=(VehicleArmRow){1,3200,1000,0};C(VehicleArm_Begin(&r,0));steps(20);C(g_vehicle_arm_state==2 && moves==1); /* same absolute target */
 r=(VehicleArmRow){1,0,1000,0};C(VehicleArm_Begin(&r,0));steps(20);C(g_vehicle_arm_state==2 && position5==0 && last_move[1]==1);
 reset();r=(VehicleArmRow){1,3200,1000,0};C(VehicleArm_Begin(&r,0));steps(5);flags=1;steps(20);C(g_vehicle_arm_state==1);flags=3;steps(12);C(g_vehicle_arm_state==2);
 reset();r=(VehicleArmRow){3,10,106,0};C(VehicleArm_Begin(&r,0));steps(20);C(g_vehicle_arm_state==2 && fabsf(position6+36)<0.01 && held_extension==position6);
 reset();r=(VehicleArmRow){3,51,106,0};C(VehicleArm_Begin(&r,0));steps(2);C(g_vehicle_arm_state==3 && !moves && !holding);
 reset();g_vehicle_lift_reference_valid=0;r=(VehicleArmRow){1,5000,1000,0};C(VehicleArm_Begin(&r,0));steps(2);C(g_vehicle_arm_state==3 && !moves);
 reset();r=(VehicleArmRow){1,105001,1000,0};C(VehicleArm_Begin(&r,0));steps(2);C(g_vehicle_arm_state==3 && !moves);
 reset();r=(VehicleArmRow){4,135,0,500};C(VehicleArm_Begin(&r,0));steps(1);C(grips==1 && last_grip==135 && g_vehicle_arm_state==1);tick+=500;VehicleArm_Service();C(g_vehicle_arm_state==2);
 reset();r=(VehicleArmRow){5,-2,0,100};C(VehicleArm_Begin(&r,2));steps(1);C(pwm_calls==1 && pwm==2278);tick+=100;VehicleArm_Service();C(g_vehicle_arm_state==2);
 reset();r=(VehicleArmRow){4,NAN,0,500};C(VehicleArm_Begin(&r,0));steps(1);C(g_vehicle_arm_state==3 && !grips);
 reset();r=(VehicleArmRow){1,3200,1000,0};C(VehicleArm_Begin(&r,0));steps(5);flags=9;steps(1);C(g_vehicle_arm_state==3 && !holding);steps(1);C(stop_calls==1);
 reset();r=(VehicleArmRow){2,180,150,0};C(VehicleArm_Begin(&r,0));g_dm_can2_read_cycle=3;steps(1);C(g_dm_jog_request==4 && g_dm_posvel_speed_deg_s==150);
 g_dm_jog_state=7;g_dm_jog_result=g_dm_posvel_holding=1;g_dm_can2_feedback_tick=tick;steps(1);C(g_vehicle_arm_state==1); /* old DONE must not complete */
 g_dm_can2_read_cycle=4;g_dm_can2_logical_position_deg=180;g_dm_can2_feedback_tick=tick;steps(1);C(g_vehicle_arm_state==2 && !g_vehicle_arm_dm_owned);
 reset();r=(VehicleArmRow){1,3200,1000,0};C(VehicleArm_Begin(&r,0));steps(1);VehicleArm_Cancel();steps(1);C(stop_calls==1 && !holding && g_vehicle_arm_state==3);
#endif
}
