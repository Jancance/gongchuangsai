/* Tests the actual stage3 owner and dispatcher with explicit hardware events. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "stage3_cycle.h"
#include "region_dispatch.h"
static unsigned checks;
#define CHECK(x) do{checks++;if(!(x)){fprintf(stderr,"FAIL line%d: %s\n",__LINE__,#x);exit(1);}}while(0)
#define SPF_DM 1U
#define SPF_EXT 2U
#define SPF_LIFT 4U
#define SPF_MAX_LIFT_PULSES 29700U
#define SOURCE_LIFT_PULSES_PER_MM 400.0f
#define VEHICLE_EXTENSION_PULSES_PER_MM 294.4f
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
static struct {uint32_t top_scale,top_valid;int64_t top_position_raw;} g_lift_ruler_controller={10,0,0};
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
static uint32_t it_at_top(void){return g_lift_ruler_controller.top_valid && g_vehicle_lift_current_deg==0;}
static void SourceCycle_KeyEvent(void){source_keys++;}
static void SourceCycle_ScanService(void){source_scans++;}
static void SourceCycle_Service(uint32_t now){(void)now;source_services++;}
static uint32_t SourceCycle_SharedByte(uint8_t b,uint32_t n,uint32_t a){(void)b;(void)n;(void)a;source_bytes++;return 1;}
static void SourceCycle_Telemetry(uint32_t now){(void)now;source_tx++;}
static uint8_t USART6_VisionSend(const uint8_t *p,uint16_t n){CHECK(n>10 && !memcmp(p,"S31,",4));uart_tx++;return 1;}
static uint16_t SC_Crc(const char *p,uint32_t n){uint16_t c=0xffff;uint32_t i,j;for(i=0;i<n;i++){c^=(uint16_t)(uint8_t)p[i]<<8;for(j=0;j<8;j++)c=(uint16_t)((c&0x8000)?((uint32_t)c<<1)^0x1021U:(uint32_t)c<<1);}return c;}
#include "stage3_cycle.inc"
#include "region_dispatch.inc"
static uint32_t tick;
static unsigned motions,commands,grips,releases,slots,dm_moves,x_moves,z_moves;
static void service(void){RegionDispatch_Service(tick);}
static void complete(void){
 CHECK(g_it_request);g_it_action=g_it_request;g_it_request=0;g_it_state=2;commands++;
 if(g_it_action==3){motions++;
  if(g_it_axes==SPF_DM){g_dm_can2_logical_position_deg=g_it_dm_deg;dm_moves++;}
  if(g_it_axes==SPF_EXT){g_startup_extension_current_deg=-g_it_extension_mm*294.4f*360/3200;x_moves++;}
  if(g_it_axes==SPF_LIFT){g_vehicle_lift_current_deg=g_it_lift_pulses*360/3200;z_moves++;}
 }
 if(g_it_action==4 || g_it_action==1){g_vehicle_lift_current_deg=0;g_lift_ruler_controller.top_valid=1;}
 if(g_it_action==5)grips++;
 if(g_it_action==6)releases++;
 if(g_it_action==10){tick+=700;slots++;}else tick+=10;
 service();
}
static void frame(int dx,int dy,int yaw,uint32_t age,uint32_t seq,int corrupt){
 char b[192];unsigned i;int n=snprintf(b,sizeof b,"O31,%u,%u,%u,1,%d,%d,%d,%u,%d,%u,",g_stage3_run,g_stage3_pose,seq,dx,dy,yaw,(unsigned)ceil(sqrt((double)dx*dx+(double)dy*dy)),yaw,age);
 snprintf(b+n,sizeof b-(unsigned)n,"%04X\r\n",SC_Crc(b,(uint32_t)n));if(corrupt)b[n]='Z';
 for(i=0;i<strlen(b);i++)CHECK(RegionDispatch_SharedByte((uint8_t)b[i],tick,0));
}
static void reset(void){
 g_region_dispatch_area=0;g_region_dispatch_rejected=0;g_stage3_state=g_stage3_fault=g_stage3_request=0;
 g_stage3_run=g_stage3_pose=g_stage3_item=g_stage3_phase=g_stage3_align_done=0;
 g_stage3_body_steps=0;g_source_cycle_state=g_source_cycle_request=0;
 g_it_state=g_it_request=g_it_cause=g_it_rejected=g_it_action=0;
 g_source_body_step_state=g_source_body_step_request=g_source_body_step_reason=g_source_align_chain_active=g_source_align_command_seq=0;
 g_chassis_auto_active=g_startup_axes_cause=g_lift_ruler_cause=g_dm_jog_first_fault=g_low_state=0;
 g_safety_fault=g_rtos_fault=g_can_ctrl_fault_latched=0;
 g_low_wheel_hold_ready=g_low_wheel_hold_stage=g_low_wheel_hold_request=0;
 s3_pending=s3_valid=s3_seq=s3_active=s3_used=0;tick=0;
 g_lift_ruler_controller.top_valid=existing_hold=0;
 g_stage3_test_first_item=0;g_stage3_test_count=3;g_stage3_pick_depth=-1;
 g_stage3_carrier_yaw=36.5f;
 for(unsigned i=0;i<3;i++){g_stage3_colors[i]=(unsigned[]){2,1,5}[i];g_stage3_positions[i]=i+1;
  g_stage3_target_yaw[i]=i==1?180:-1;g_stage3_target_x[i]=i==1?0:-1;g_stage3_target_depth[i]=-1;}
 g_vehicle_lift_current_deg=g_dm_can2_logical_position_deg=g_startup_extension_current_deg=0;
 commands=motions=grips=releases=slots=dm_moves=x_moves=z_moves=0;
}
static void prepare(void){
 CHECK(RegionDispatch_Arrive(3));service();
 CHECK(g_stage3_state==1 && g_it_request==1);
 for(unsigned i=0;i<6;i++)complete();
 CHECK(g_stage3_state==2 && g_stage3_pose==2);
}
static void aligned(void){
 frame(0,0,0,0,1,0);service();CHECK(g_stage3_state==4 && g_stage3_align_done);
 service();CHECK(g_low_wheel_hold_request);g_low_wheel_hold_request=0;g_low_wheel_hold_ready=1;g_low_wheel_hold_stage=13;service();
}
static void teach_tests(void){
 for(unsigned chain=0;chain<2;chain++)for(unsigned config=0;config<2;config++){
  reset();prepare();g_lift_ruler_controller.top_valid=1;
  g_low_wheel_hold_ready=1;g_low_wheel_hold_stage=13;
  g_source_align_chain_active=chain;
  if(config){g_stage3_pick_depth=10000;for(unsigned n=0;n<3;n++){g_stage3_target_depth[n]=12000;g_stage3_target_yaw[n]=n==1?180:150;g_stage3_target_x[n]=0;}}
  uint32_t oldpose=g_stage3_pose;g_stage3_request=30;service();
  CHECK(g_stage3_state==4 && !g_stage3_align_done && !g_it_request && !s3_pending);
  CHECK(g_stage3_pose==oldpose+1 && !s3_valid && !g_low_wheel_hold_ready && !g_low_wheel_hold_stage);
  CHECK(g_source_body_step_request==(chain?7U:0U));
  if(chain){service();CHECK(!g_low_wheel_hold_request);g_source_body_step_request=0;g_source_align_chain_active=0;}
  service();CHECK(g_low_wheel_hold_request==1 && !g_it_request);
  g_low_wheel_hold_ready=1;g_low_wheel_hold_stage=13;g_low_wheel_hold_request=0;service();
  CHECK(g_stage3_state==7 && !g_stage3_align_done && !g_it_request);
  /* Leave while Z is low and X extended: config is deliberately optional. */
  g_vehicle_lift_current_deg=900;g_startup_extension_current_deg=-900;g_dm_can2_logical_position_deg=36.5f;
  g_stage3_request=2;service();CHECK(g_stage3_state==1 && g_stage3_phase==0 && !g_it_request);
  CHECK(!g_low_wheel_hold_ready && !g_low_wheel_hold_stage && !g_low_wheel_hold_request);
  service();CHECK(g_it_request==4);complete();CHECK(g_it_request==2);
  complete();CHECK(g_it_request==7);complete();CHECK(g_it_request==6);
  complete();CHECK(g_it_request==3 && g_it_axes==SPF_EXT && g_it_extension_mm==0);
  complete();CHECK(g_it_request==3 && g_it_axes==SPF_DM && g_it_dm_deg==180);
  complete();CHECK(g_stage3_state==2 && !g_stage3_align_done && !g_it_request);
 }
 reset();prepare();g_it_request=3;g_stage3_request=30;uint32_t pose=g_stage3_pose;service();
 CHECK(g_stage3_state==2 && g_it_request==3 && g_stage3_rejected && g_stage3_pose==pose);
 reset();prepare();s3_pending=4;g_stage3_request=30;service();CHECK(g_stage3_state==2 && s3_pending==4 && g_stage3_rejected);
 reset();prepare();g_source_body_step_state=2;g_stage3_request=30;service();CHECK(g_stage3_state==2 && g_stage3_rejected);
 reset();prepare();g_lift_ruler_controller.top_valid=1;g_stage3_request=30;service();g_stage3_request=9;service();
 CHECK(g_stage3_state==9 && g_stage3_fault==2 && !g_it_request);
}
int main(void){
 unsigned i,j,a,b,c,before;uint32_t pose;
 teach_tests();
 CHECK(SC_Crc("123456789",9)==0x29b1);
 reset();RegionDispatch_KeyEvent();service();CHECK(g_stage3_state==10 && g_it_carrier_deg==0 && !motions);
 complete();CHECK(tick==700 && g_stage3_state==10 && !g_it_request);
 tick=2699;service();CHECK(!g_it_request);tick=2700;service();CHECK(g_it_carrier_deg==120 && g_it_request==10);
 complete();CHECK(tick==3400);tick=5399;service();CHECK(!g_it_request);tick=5400;service();CHECK(g_it_carrier_deg==240);
 complete();CHECK(tick==6100 && g_stage3_state==11 && !motions);service();CHECK(g_stage3_state==11);
 RegionDispatch_KeyEvent();service();for(i=0;i<6;i++)complete();CHECK(g_stage3_state==2);
 reset();prepare();pose=g_stage3_pose;
 frame(0,0,0,0,1,1);service();CHECK(g_stage3_state==2);
 frame(0,0,0,201,1,0);service();CHECK(g_stage3_state==2);
 frame(0,0,0,200,1,0);tick++;service();CHECK(g_stage3_state==2);
 frame(2000,2000,1200,0,2,0);service();CHECK(g_stage3_state==3 && g_source_body_step_request==3 && fabsf(g_source_align_turn_deg-1.2f)<.00001f && Stage3_FineTurnAllowed());
 CHECK(g_stage3_pose==pose+1);frame(0,0,0,0,3,0);CHECK(!s3_valid);
 g_source_body_step_request=0;g_source_body_step_state=3;g_source_align_command_seq++;g_source_align_chain_active=1;service();CHECK(g_stage3_state==2 && g_stage3_pose==pose+2);
 frame(2000,2000,1000,0,1,0);service();CHECK(g_stage3_state==3 && g_source_body_step_request==2 && g_source_align_distance_mm==2);
 g_source_body_step_request=0;g_source_align_command_seq++;service();CHECK(g_stage3_state==2);
 frame(1000,1000,1000,0,1,0);service();CHECK(g_stage3_state==4 && g_source_body_step_request==7);
 g_source_body_step_request=0;g_source_align_chain_active=0;service();CHECK(g_low_wheel_hold_request);
 g_low_wheel_hold_ready=1;service();CHECK(g_stage3_state==7 && !g_it_request);
 before=motions;for(i=0;i<100;i++){tick+=100;service();}CHECK(motions==before && !g_it_request);
 /* Center-only profile does not require positions1/3. */
 g_stage3_test_first_item=1;g_stage3_test_count=1;g_stage3_pick_depth=10000;g_stage3_target_depth[1]=12000;
 g_stage3_request=2;service();CHECK(g_stage3_state==5 && g_stage3_item==1);
 g_stage3_target_depth[1]=29000;g_stage3_colors[1]=6;service();CHECK(g_it_request==10 && g_it_carrier_deg==120);
 for(i=0;g_stage3_state==5 && i<100;i++){if(g_it_request)complete();else service();}
 CHECK(i<100 && g_stage3_state==6 && grips==1 && releases==3 && slots==2 && g_it_carrier_deg==240 && g_dm_can2_logical_position_deg==0 && g_startup_extension_current_deg==0);
 CHECK(s3_depth[1]==12000 && s3_colors[1]==1 && g_stage3_body_steps==2);
 /* Every target permutation and different 3-color permutation uses locked objects. */
 const unsigned perm[6][3]={{1,2,3},{1,3,2},{2,1,3},{2,3,1},{3,1,2},{3,2,1}};
 for(a=0;a<6;a++)for(b=0;b<6;b++){
  reset();prepare();g_stage3_pick_depth=10000;
  for(c=0;c<3;c++){g_stage3_colors[c]=perm[b][c];g_stage3_positions[c]=perm[a][c];g_stage3_target_yaw[c]=150+30*c;g_stage3_target_x[c]=c==1?0:20*c;g_stage3_target_depth[c]=12000+1000*c;}
  aligned();CHECK(g_stage3_state==5);
  for(i=0;g_stage3_state==5 && i<100;i++){if(g_it_request)complete();else service();}
  CHECK(i<100 && g_stage3_state==6 && grips==3 && releases==5 && slots==4 && g_stage3_body_steps==0);
  for(j=0;j<3;j++)CHECK(s3_positions[j]==perm[a][j] && s3_colors[j]==perm[b][j]);
 }
 /* All owner states retain STOP and do not restart. */
 const unsigned states[]={1,2,3,4,5,7,10};
 for(i=0;i<sizeof states/sizeof states[0];i++){
  reset();g_region_dispatch_area=3;g_stage3_state=states[i];g_it_state=1;g_source_body_step_state=2;
  g_stage3_request=9;service();CHECK(g_stage3_state==9 && g_stage3_fault==2 && g_it_request==9 && g_source_body_step_request==9);
  g_it_request=g_source_body_step_request=0;service();CHECK(!g_it_request && !g_source_body_step_request);
 }
 reset();g_region_dispatch_area=3;g_stage3_state=2;g_it_request=10;g_source_body_step_request=3;
 g_stage3_request=9;service();CHECK(g_it_request==9 && g_source_body_step_request==9 && g_stage3_state==9);
 reset();prepare();g_stage3_request=20;g_source_align_direction=2;g_source_align_distance_mm=5;pose=g_stage3_pose;service();
 CHECK(g_stage3_state==3 && g_stage3_pose==pose+1 && g_source_body_step_request==2);
 g_source_body_step_request=0;g_source_align_command_seq++;g_source_body_step_state=3;service();CHECK(g_stage3_state==2 && g_stage3_pose==pose+2);
 /* Capture waits for new diagnostic cycles, then preserves fixed center XY. */
 g_stage3_state=7;g_stage3_teach_request=3;service();CHECK(!g_stage3_teach_result && g_lift_diag_request==1 && g_recovery_diag_request==1);
 g_lift_diag_raw[3][2]=0;g_lift_diag_raw[3][3]=0;g_lift_diag_raw[3][4]=35;g_lift_diag_raw[3][5]=40; /* 9000 /10 =900deg */
 g_lift_diag_raw[4][2]=0;g_lift_diag_result[3]=g_lift_diag_result[4]=1;
 g_recovery_diag_result[0]=g_recovery_diag_result[3]=1;g_recovery_diag_raw[3][4]=1;g_recovery_diag_raw[3][5]=1;
 g_lift_diag_done=g_recovery_diag_done=1;g_lift_diag_cycle++;g_recovery_diag_cycle++;g_dm_can2_read_cycle++;
 g_dm_can2_logical_position_deg=185;service();
 CHECK(g_stage3_teach_result==1 && g_stage3_target_yaw[1]==180 && g_stage3_target_x[1]==0 && fabsf(g_stage3_target_depth[1]-8000)<.01f);
 g_stage3_state=6;g_stage3_request=1;service();CHECK(g_stage3_state==1 && g_stage3_target_depth[1]>0);
 reset();g_region_dispatch_area=1;RegionDispatch_KeyEvent();RegionDispatch_ScanService();service();CHECK(source_keys==1 && source_scans==1 && source_services==1);
 g_source_cycle_state=7;CHECK(!RegionDispatch_Arrive(3));g_source_cycle_state=8;CHECK(RegionDispatch_Arrive(3));
 g_lift_ruler_controller.top_valid=1;existing_hold=1;before=source_services;service();CHECK(!g_it_request && g_stage3_phase==1);service();CHECK(g_it_request==7);for(i=0;i<4;i++)complete();CHECK(g_stage3_state==2);
 RegionDispatch_ScanService();CHECK(source_services==before && source_scans==1);
 Stage3_Telemetry(1000);CHECK(uart_tx==1);
 printf("PASS actual stage3+dispatcher: %u checks, load boundaries, CRC/age/pose, fine yaw, Euclidean XY, unconfigured wait, center-only,36 task mappings, STOP/exclusion\n",checks);
 return 0;
}
