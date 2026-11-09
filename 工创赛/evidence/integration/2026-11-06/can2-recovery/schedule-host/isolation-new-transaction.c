#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "source_parallel_flow.h"
#include "lift_ruler_calibration.h"
LiftRulerController g_lift_ruler_controller;
unsigned g_can_ctrl_fault_latched;
unsigned g_can_guard_ready;
unsigned g_can_test_result;
unsigned g_chassis_auto_active;
unsigned g_dm_can2_feedback_tick;
unsigned g_dm_can2_feedback_valid;
float g_dm_can2_logical_position_deg;
unsigned g_dm_can2_read_cycle;
unsigned g_dm_can2_read_request;
unsigned g_dm_can2_state;
unsigned g_dm_jog_first_fault;
unsigned g_dm_jog_request;
unsigned g_dm_posvel_holding;
unsigned g_extension_hold_confirmed;
float g_extension_home_reference_deg;
unsigned g_extension_home_reference_valid;
unsigned g_lift_ruler_cause;
unsigned g_lift_ruler_rejected;
unsigned g_lift_ruler_request;
unsigned g_lift_ruler_samples;
unsigned g_lift_ruler_state;
unsigned g_rtos_fault;
unsigned g_safety_fault;
unsigned g_safety_ready;
unsigned g_source_parallel_state;
unsigned g_startup_axes_cause;
unsigned g_startup_axes_state;
float g_startup_extension_current_deg;
float g_vehicle_lift_current_deg;
unsigned g_vehicle_lift_feedback_valid;
unsigned g_vehicle_lift_reference_valid;
unsigned g_vm_test_request;
unsigned tick,owned,starts,cancels,services,closes,opens,carrier,checks,failures;
SPF_Command sent;
#define CK(v) do{checks++;if(!(v)){failures++;printf("DISPATCH FAIL %d %s\n",__LINE__,#v);}}while(0)
unsigned xTaskGetTickCount(void){return tick;}
unsigned SingleTeach_ReferencesReady(void){return g_startup_axes_state==4 && g_extension_home_reference_valid && g_extension_hold_confirmed && g_vm_test_request;}
unsigned VehicleSourceParallel_Owns(void){return owned;}
void VehicleSourceParallel_Cancel(void){cancels++;}
void VehicleArm_Service(void){services++;}
unsigned VehicleArm_Busy(void){return owned;}
unsigned DM_CAN2_JogIsBusy(void){return 0;}
unsigned StartupAxes_Begin(void){starts++;g_startup_axes_state=1;return 1;}
void StartupAxes_Cancel(void){cancels++;}
unsigned normal_can1,stop_can1;
void StartupAxes_Service(unsigned a,unsigned b){(void)a;(void)b;if(g_startup_axes_state>=5)stop_can1++;else normal_can1++;}
unsigned jiazhua(float angle){if(angle==140)closes++;else if(angle==210)opens++;return 1;}
unsigned CarrierPwm_Write(unsigned pulse){carrier=pulse;return 1;}
unsigned VehicleArm_BeginSourceParallel(const SPF_Command*c,unsigned v,unsigned d){(void)d;CK(v==1);sent=*c;owned=1;g_source_parallel_state=1;return 1;}
#include "integration10_complete.inc"
static void req(unsigned v){g_it_request=v;Integration10_Service();}
static void advance(unsigned ms){tick+=ms;g_dm_can2_feedback_tick=tick;Integration10_Service();}
static void pose_start(void){g_dm_can2_read_cycle++;g_dm_can2_feedback_valid=1;g_dm_can2_feedback_tick=tick;Integration10_Service();CK(owned);}
static void pose_done(void){owned=0;g_source_parallel_state=3;advance(20);CK(g_it_state==2);}

int main(void){
 (void)pose_start;(void)pose_done;
 g_safety_ready=g_can_test_result=g_vm_test_request=g_can_guard_ready=1;
 g_lift_ruler_state=4;g_lift_ruler_controller.top_valid=1;g_lift_ruler_controller.scale=g_lift_ruler_controller.top_scale=100;
 g_lift_ruler_samples=1;g_startup_axes_state=4;g_extension_home_reference_valid=g_extension_hold_confirmed=1;
 g_dm_can2_state=5;g_dm_can2_read_cycle=7;g_it_axes=1;g_it_extension_mm=0;g_it_lift_pulses=0;
 req(3);CK(g_it_state==1 && g_it_cause==0 && normal_can1==0);CK(g_dm_can2_read_request==2);
 g_dm_can2_read_request=0;g_dm_can2_state=1;g_dm_can2_read_cycle=8;advance(10);CK(normal_can1==0 && g_it_state==1);
 g_startup_axes_state=5;g_startup_axes_cause=8;advance(10);CK(stop_can1==1 && g_it_state==9);
 owned=0;g_it_state=g_it_cause=0;g_startup_axes_state=4;g_startup_axes_cause=0;g_dm_can2_state=5;g_dm_can2_read_cycle=10;
 req(7);CK(g_it_state==1 && !g_it_cause);g_dm_can2_read_request=0;g_dm_can2_read_cycle=11;advance(10);CK(g_it_state==9 && g_it_cause==6);
 printf("ISOLATION_NEW_TRANSACTION checks=%u failures=%u\n",checks,failures);return failures?1:0;
}
