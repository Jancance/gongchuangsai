#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
/* Avoid Windows CRT assertion dialogs in an unattended offline fixture. */
#undef assert
#define assert(c) do { if(!(c)) { fprintf(stderr,"FAIL %s:%d: %s\n",__FILE__,__LINE__,#c); exit(1); } } while(0)
#define SPF_STAGE3_MAX_LIFT_PULSES 82000U
#define SOURCE_LIFT_PULSES_PER_MM 400.0f
#define VEHICLE_LIFT_DOWN_SIGN 1
#define SPF_DM 1U
#define SPF_EXT 2U
#define SPF_LIFT 4U
#define S3_SYNC_PLACE_PHASE 9U
#define S3_SYNC_RETURN_PHASE 12U
#define S3_RECOVER_FIRST_PHASE 17U
static uint32_t g_stage3_cycle_mode,g_stage3_first_mapping_valid,g_stage3_first_colors[3],g_stage3_first_positions[3];
static uint32_t g_stage3_run=1,g_stage3_locked_run,g_stage3_fault,g_stage3_rejected;
static uint32_t s3_locked_top_scale,s3_teach_pending,g_stage3_teach_request,g_source_body_step_request;
static int64_t s3_locked_top_raw;
static float s3_locked_extension_home,g_extension_home_reference_deg;
static uint32_t platform=1,refs=1,g_low_wheel_hold_ready=1;
static uint32_t S3_Platform(void){return platform;}
static uint32_t SingleTeach_ReferencesReady(void){return refs;}
static uint32_t g_stage3_task_run=1,g_stage3_task_batch=1,g_stage3_test_first_item,g_stage3_test_count=3;
static uint32_t g_stage3_colors[3]={5,1,2},g_stage3_positions[3]={3,1,2},g_stage3_slot_colors[3]={1,2,5};
static float g_stage3_carrier_yaw=36.5f,g_stage3_pick_depth=8003;
static float g_stage3_target_yaw[3]={220,180,140},g_stage3_target_x[3]={30,0,30};
static float g_stage3_target_depth[3]={59400,61400,60600},g_stage3_recovery_depth[3]={-1,-1,-1};
static uint32_t s3_first,s3_count,s3_round_trip,s3_cycle_mode,s3_colors[3],s3_positions[3],s3_slots[3];
static float s3_pick,s3_carrier,s3_yaw[3],s3_x[3],s3_depth[3],s3_recovery_depth[3];
static uint32_t g_stage3_item,g_stage3_phase,g_stage3_state,g_stage3_placed_count,g_stage3_recovered_count;
static uint32_t s3_pending,s3_tray_valid,g_stage3_tray_prefetch_item,s3_tray_tick,s3_preclose_started,s3_preclose_tick;
static uint32_t g_it_axes,g_it_request,g_it_carrier_deg,poses,fault,complete,top;
static float g_it_extension_mm,g_it_lift_pulses;
static struct {uint32_t top_scale;int64_t safe_position_raw,top_position_raw;} g_lift_ruler_controller={100,6750,0};
static uint32_t it_at_top(void){return top;}
static void S3_Fail(uint32_t n){fault=n;g_stage3_state=9;}
static void S3_Pose(void){poses++;}
static void S3_Command(uint32_t n){s3_pending=g_it_request=n;}
static uint32_t S3_Complete(void){if(!complete)return 0;s3_pending=g_it_request=0;return 1;}
static void S3_Axis(uint32_t axis,float n){g_it_axes=axis;if(axis==SPF_LIFT)g_it_lift_pulses=n;else if(axis==SPF_EXT)g_it_extension_mm=n;S3_Command(3);}
static uint32_t jiazhua(float n){assert(n==160);return 1;}
static uint32_t S3_PrefetchNextTray(uint32_t i,uint32_t now){(void)i;(void)now;return 1;}
static void S3_IndexItemSlot(void){uint32_t i=0;while(i<3 && s3_slots[i]!=s3_colors[g_stage3_item])i++;assert(i<3);g_it_carrier_deg=i*120;S3_Command(10);}
// INSERT_EXTRACTED_FIRMWARE
static void completed(uint32_t now){if(g_it_request==4)top=1;complete=1;service(now);complete=0;}
int main(void){
 uint32_t i,ticks;
 assert(S3_Config());
 g_stage3_cycle_mode=1;assert(!S3_Config());
 for(i=0;i<3;i++)g_stage3_recovery_depth[i]=60000+i*100;
 assert(S3_Config());
 S3_LockConfig();
 /* Execute actual latest-baseline phase switch and real pending-completion gate. */
 for(ticks=0;ticks<1000 && g_stage3_state==5;ticks++){
  if(s3_pending){
   uint32_t before=g_stage3_phase;
   service(ticks*1000);assert(g_stage3_phase==before);
   completed(ticks*1000);
  }else service(ticks*1000);
  if(g_stage3_phase>=17 && g_stage3_phase<=28){
   assert(g_stage3_placed_count==3);
   if(g_stage3_phase==19 && s3_pending)assert(g_it_lift_pulses==s3_recovery_depth[s3_positions[g_stage3_item]-1]);
   if(g_stage3_phase==24 && s3_pending)assert(s3_slots[g_it_carrier_deg/120]==s3_colors[g_stage3_item]);
   if(g_stage3_phase==25 && s3_pending)assert(g_it_request==4);
   if(g_stage3_phase==26 && s3_pending)assert(g_it_request==17);
  }
 }
 assert(!fault && g_stage3_state==6 && g_stage3_placed_count==3 && g_stage3_recovered_count==3);
 assert(g_stage3_first_mapping_valid && g_stage3_first_positions[0]==3);
 g_stage3_cycle_mode=2;g_stage3_task_batch=2;
 g_stage3_colors[0]=2;g_stage3_colors[1]=5;g_stage3_colors[2]=1;
 assert(S3_Config());S3_LockConfig();
 assert(s3_positions[0]==2 && s3_positions[1]==3 && s3_positions[2]==1);
 for(i=0;i<3;i++)assert(s3_depth[i]==g_stage3_target_depth[i]-24000);
 g_stage3_first_positions[0]=1;assert(!S3_Config());assert(s3_positions[1]==3);
 g_stage3_first_positions[0]=0;assert(!S3_Config());
 g_stage3_first_positions[0]=4;assert(!S3_Config());
 g_stage3_first_positions[0]=3;
 g_stage3_first_colors[0]=1;assert(!S3_Config());
 g_stage3_first_colors[0]=5;assert(S3_Config());
 g_stage3_first_mapping_valid=0;assert(!S3_Config());
 g_stage3_cycle_mode=0;assert(S3_Config());S3_LockConfig();
 for(i=0;i<3;i++)assert(s3_depth[i]==g_stage3_target_depth[i]);
 for(ticks=0;ticks<1000 && g_stage3_state==5;ticks++){
  if(s3_pending)completed(ticks*1000);else service(ticks*1000);
 }
 assert(g_stage3_state==6 && g_stage3_placed_count==3 && !g_stage3_recovered_count);
 assert(g_stage3_first_colors[0]==5 && g_stage3_first_positions[0]==3);
 /* A default three-place run can explicitly recover its already placed objects. */
 assert(g_stage3_locked_run==g_stage3_run);
 top=0;request32(32);assert(g_stage3_state==6 && g_stage3_rejected==1);
 top=1;g_stage3_run++;request32(32);assert(g_stage3_state==6 && g_stage3_rejected==2);
 g_stage3_run--;g_extension_home_reference_deg=1;request32(32);assert(g_stage3_state==6 && g_stage3_rejected==3);
 g_extension_home_reference_deg=0;g_stage3_recovery_depth[0]=-1;request32(32);assert(g_stage3_state==6 && g_stage3_rejected==4);
 g_stage3_recovery_depth[0]=60000;request32(32);
 assert(g_stage3_state==5 && g_stage3_phase==17 && g_stage3_placed_count==3 && !g_stage3_recovered_count);
 assert(s3_positions[0]==3 && s3_positions[1]==1 && s3_positions[2]==2);
 for(ticks=0;ticks<1000 && g_stage3_state==5;ticks++){
  if(s3_pending)completed(ticks*1000);else service(ticks*1000);
 }
 assert(g_stage3_state==6 && g_stage3_placed_count==3 && g_stage3_recovered_count==3);
 request32(32);assert(g_stage3_state==6 && g_stage3_rejected==5);
 /* Match real LRC int64_t reference storage, including signed large raw values. */
 for(i=0;i<2;i++){
  g_lift_ruler_controller.top_position_raw=i?-8589934611LL:8589934611LL;
  S3_LockConfig();
  assert(s3_locked_top_raw==g_lift_ruler_controller.top_position_raw);
  g_stage3_state=6;g_stage3_placed_count=3;g_stage3_recovered_count=0;s3_pending=g_it_request=0;top=1;
  assert(S3_BeginRecoverPlaced());
 }
 puts("PASS extracted Stage3 config/lock/action switch: modes, mapping, recovery after place3, original slots, real-top release, pending completion");
 return 0;
}
