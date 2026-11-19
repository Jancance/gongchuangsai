/* Tests the actual stage3 owner and dispatcher with explicit hardware events. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../../USER/vehicle_parameters.h"
#include "stage3_cycle.h"
#include "region_dispatch.h"
static unsigned checks;
#define CHECK(x) do{checks++;if(!(x)){fprintf(stderr,"FAIL line%d: %s\n",__LINE__,#x);exit(1);}}while(0)
#include "source_parallel_flow.h"
#include "lift_ruler_calibration.h"
#define SPF_DM 1U
#define SPF_EXT 2U
#define SPF_LIFT 4U
#define SPF_MAX_LIFT_PULSES 29700U
#define SOURCE_LIFT_PULSES_PER_MM 400.0f
static uint32_t g_it_state,g_it_request,g_it_cause,g_it_rejected,g_it_action,g_it_axes;
static float g_it_dm_deg,g_it_extension_mm,g_it_lift_pulses,g_it_carrier_auto_deg,g_it_carrier_deg;
static uint32_t g_safety_ready=1,g_can_test_result=1,g_safety_fault,g_rtos_fault,g_can_ctrl_fault_latched;
static uint32_t g_chassis_auto_active,g_startup_axes_cause,g_lift_ruler_cause,g_dm_jog_first_fault,g_low_state;
static uint32_t g_source_body_step_request,g_source_body_step_state,g_source_body_step_reason;
static uint32_t g_source_align_chain_active,g_source_align_command_seq,g_source_align_direction;
volatile float g_source_align_turn_deg;
static float g_source_align_distance_mm;
static uint32_t g_low_wheel_hold_ready,g_low_wheel_hold_stage,g_low_wheel_hold_request;
static uint32_t g_source_cycle_state,g_source_cycle_request;
static float g_dm_can2_logical_position_deg,g_extension_home_reference_deg,g_startup_extension_current_deg,g_vehicle_lift_current_deg;
static uint32_t g_vehicle_lift_feedback_valid=1,g_startup_extension_feedback_valid=1;
static LiftRulerController g_lift_ruler_controller;
static uint32_t g_extension_home_reference_valid=1,g_extension_hold_confirmed=1;
static uint32_t existing_hold;
static uint32_t StartupAxes_Holding(void){return existing_hold;}
volatile uint32_t g_lift_diag_request,g_lift_diag_cycle,g_lift_diag_done,g_lift_diag_result[6];
volatile uint8_t g_lift_diag_raw[6][8];
static volatile uint32_t g_recovery_diag_request,g_recovery_diag_cycle,g_recovery_diag_done,g_recovery_diag_result[5];
static volatile uint8_t g_recovery_diag_raw[4][8];
static uint32_t g_dm_can2_read_cycle,g_dm_can2_read_request;
static unsigned source_keys,source_scans,source_services,source_bytes,source_tx,uart_tx;
static uint32_t Low_Busy(void){return 0;}
static uint32_t SourceBodyStep_Busy(void){return g_source_body_step_state==1 || g_source_body_step_state==2;}
static uint32_t SourceBodyStep_WheelWindow(void){return SourceBodyStep_Busy() || g_source_align_chain_active;}
static uint32_t SingleTeach_ReferencesReady(void){return 1;}
static uint32_t it_dm_fresh(void){return 1;}
static uint32_t it_at_top(void)
{return g_lift_ruler_controller.top_valid && g_lift_ruler_controller.backoff_valid &&
 g_vehicle_lift_feedback_valid && g_lift_ruler_controller.top_scale &&
 fabsf(g_vehicle_lift_current_deg-(float)g_lift_ruler_controller.safe_position_raw/
       (float)g_lift_ruler_controller.top_scale)<=SOURCE_TOP_TOLERANCE_DEG;}
static void SourceCycle_KeyEvent(void){source_keys++;}
static void SourceCycle_ScanService(void){source_scans++;}
static void SourceCycle_Service(uint32_t now){(void)now;source_services++;}
static uint32_t SourceCycle_SharedByte(uint8_t b,uint32_t n,uint32_t a){(void)b;(void)n;(void)a;source_bytes++;return 1;}
static void SourceCycle_Telemetry(uint32_t now){(void)now;source_tx++;}
static uint8_t USART6_VisionSend(const uint8_t *p,uint16_t n){CHECK(n>10 && !memcmp(p,"S31,",4));uart_tx++;return 1;}
static uint16_t SC_Crc(const char *p,uint32_t n){uint16_t c=0xffff;uint32_t i,j;for(i=0;i<n;i++){c^=(uint16_t)(uint8_t)p[i]<<8;for(j=0;j<8;j++)c=(uint16_t)((c&0x8000)?((uint32_t)c<<1)^0x1021U:(uint32_t)c<<1);}return c;}
#include "stage3_cycle.inc"
#include "region_dispatch.inc"

static int64_t actual;
static uint32_t bus(void*x,const uint8_t*q,uint32_t n,uint32_t rn,uint8_t*r){
 uint32_t a=(uint32_t)actual;(void)x;(void)n;memset(r,0,rn);r[0]=q[0];r[rn-1]=0x6b;
 if(q[0]!=0x36)return 0;r[2]=a>>24;r[3]=a>>16;r[4]=a>>8;r[5]=a;return 1;
}
static void setup(uint32_t scale,uint32_t depth){
 LiftRulerController*c=&g_lift_ruler_controller;LiftRuler_Init(c);c->state=LRC_DONE;
 c->top_valid=c->backoff_valid=1;c->top_scale=c->scale=scale;c->safe_position_raw=(int64_t)1800*360*scale/3200;
 actual=(int64_t)depth*360*scale/3200;g_vehicle_lift_current_deg=(float)actual/scale;
 g_region_dispatch_area=3;g_stage3_fault=g_it_rejected=g_it_request=g_it_cause=0;
 g_it_state=2;s3_pending=0;g_vehicle_lift_feedback_valid=1;
}
int main(void){
 for(uint32_t scale=10;scale<=100;scale*=10){
  for(uint32_t phase=0;phase<4;phase++){
   uint32_t p=(uint32_t[]){0,1,6,11}[phase];setup(scale,1800);
   g_stage3_state=p?5:1;g_stage3_phase=p;Stage3_Service(1000);
   CHECK(!g_it_request && !s3_pending && g_stage3_phase==p+1);
   CHECK(g_lift_ruler_controller.state==LRC_DONE && !g_lift_ruler_controller.cause);
   setup(scale,82000);g_stage3_state=p?5:1;g_stage3_phase=p;Stage3_Service(1000);
   CHECK(g_it_request==4 && s3_pending==4);
   CHECK(LiftRuler_StartReturnWithLimit(&g_lift_ruler_controller,1000,1,82000));
   g_lift_ruler_controller.state=LRC_MOVING;g_lift_ruler_controller.phase=7;
   LiftRuler_Service(&g_lift_ruler_controller,1010,1,1,1,bus,0);
   CHECK(!g_lift_ruler_controller.cause && g_lift_ruler_controller.phase==8);
  }
  setup(scale,1800);CHECK(LiftRuler_StartReturnWithLimit(&g_lift_ruler_controller,1000,1,82000));
  g_lift_ruler_controller.state=LRC_MOVING;g_lift_ruler_controller.phase=7;
  LiftRuler_Service(&g_lift_ruler_controller,1010,1,1,1,bus,0);
  CHECK(g_lift_ruler_controller.cause==LRC_CAUSE_POSITION && g_lift_ruler_controller.stop_pending);
  setup(scale,600);g_lift_ruler_controller.safe_position_raw=(int64_t)600*360*scale/3200;
  g_stage3_state=1;g_stage3_phase=0;Stage3_Service(1000);CHECK(!g_it_request && g_stage3_phase==1);
  setup(scale,82000);g_lift_ruler_controller.reuse_count=8;
  CHECK(LiftRuler_StartReturnWithLimit(&g_lift_ruler_controller,1000,1,82000));
  CHECK(g_lift_ruler_controller.reuse_count==9 && g_lift_ruler_controller.reuse_reference);
  g_lift_ruler_controller.state=LRC_DONE;
  CHECK(LiftRuler_StartReturnWithLimit(&g_lift_ruler_controller,1010,1,82000));
  CHECK(!g_lift_ruler_controller.reuse_count && !g_lift_ruler_controller.reuse_reference);
  setup(scale,1800);g_vehicle_lift_feedback_valid=0;g_stage3_state=1;g_stage3_phase=0;
  Stage3_Service(1000);CHECK(g_it_request==4); /* invalid feedback never skips */
 }
 printf("PASS actual Stage3 owner + actual LRC + actual it_at_top predicate: %u checks; both scales, prep/3 ensure-top, deep return, zero-delta rejection and invalid-feedback retained\n",checks);
 return 0;
}
