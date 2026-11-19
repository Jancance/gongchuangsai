#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "source_parallel_flow.h"
#define APP_MOTOR_OFFLINE_ONLY 0
#define VEHICLE_EXTENSION_PULSES_PER_MM 315.5f
static unsigned checks;
#define CHECK(x) do{checks++;if(!(x)){fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x);exit(1);}}while(0)
typedef struct{uint32_t options,position_tick,position_cycle;float current,before;}VSP_Axis;
static VSP_Axis vsp_axis[2];
static SPF_Group vsp_group;
static struct{uint32_t top_valid,top_scale;}g_lift_ruler_controller;
static uint32_t g_it_request,g_it_state,g_it_action,g_it_sequence,g_it_cause,g_it_rejected,g_it_axes;
static uint32_t g_lift_ruler_request,g_lift_ruler_stop_pending,g_it_lead5_grip_done;
static float g_it_dm_deg,g_it_extension_mm,g_it_lift_pulses,g_it_carrier_auto_deg;
static float g_startup_extension_current_deg,g_extension_home_reference_deg,g_dm_can2_logical_position_deg;
static uint32_t g_vis_material_valid,g_vis_mat_cycle,g_vis_mat_tick,g_vis_mat_left,g_vis_mat_right,g_vis_mat_down;
static uint32_t tick,platform,top,body,fresh,grips,releases,failures,owned,query_fail,query_delay;
static float measured_depth;
static uint32_t xTaskGetTickCount(void){return tick;}
static float SingleTeach_TopDeg(void){return 0.0f;}
static uint32_t VehicleSourceParallel_Owns(void){return owned;}
static uint32_t VSP_Position(uint8_t address,VSP_Axis *a){
 if(query_fail)return 0U;
 tick+=query_delay;
 a->current=address==5U?measured_depth*360.0f/3200.0f:g_startup_extension_current_deg;
 a->position_tick=tick;a->position_cycle++;return 1U;
}
static uint32_t it_platform(void){return platform;}
static uint32_t it_at_top(void){return top;}
static uint32_t it_dm_fresh(void){return fresh;}
static uint32_t SingleTeach_ReferencesReady(void){return 1U;}
static uint32_t SourceBodyStep_Busy(void){return body;}
static uint32_t SourceBodyStep_WheelWindow(void){return body;}
static uint32_t VehicleArm_Busy(void){return owned;}
static uint32_t SourceStage2_IsActive(void){return 0U;}
static uint32_t Stage2Fast_IsActive(void){return 0U;}
static void it_fail(uint32_t cause){failures++;g_it_cause=cause;g_it_state=9U;}
static uint32_t jiazhua(float angle){if(angle==140)grips++;else if(angle==210)releases++;else CHECK(0);return 1U;}
static uint32_t Low_Busy(void);
static uint32_t Region_Busy(void);
#include "integration13_immediate.inc"
#include "pzq_near_cycle.inc"
#include "pzq_region_batch.inc"
#include "pzq_low_pick.inc"

static void reset(void){
 tick=1000;platform=fresh=1;top=body=owned=query_fail=query_delay=0;
 g_lift_ruler_controller.top_valid=1;g_lift_ruler_controller.top_scale=100;
 g_it_state=2;g_it_request=g_it_sequence=g_it_cause=g_it_rejected=g_it_action=0;
 g_it_auto_state=g_pzq_near_state=g_region_state=0;
 g_lift_ruler_request=g_lift_ruler_stop_pending=g_it_lead5_grip_done=0;
 g_low_request=g_low_state=g_low_phase=g_low_fault=g_low_rejected=0;
 g_low_accepted_sequence=g_low_done_sequence=g_low_clear_valid=0;
 g_low_sequence=1;g_low_slot=1;g_low_color=2;g_low_region=1;
 g_low_pose_version=4;g_low_scene_id=9;g_low_target_yaw=163.5f;g_low_target_extension_mm=159;
 g_extension_home_reference_deg=0;g_startup_extension_current_deg=-65*315.5f*360/3200;
 g_dm_can2_logical_position_deg=180;measured_depth=62400;
 g_vis_material_valid=1;g_vis_mat_cycle=10;g_vis_mat_tick=tick;
 g_vis_mat_left=5;g_vis_mat_right=2;g_vis_mat_down=1;
 g_low_confirm_frame_cycle=g_low_confirm_pose_sequence=0;
 grips=releases=failures=0;memset(vsp_axis,0,sizeof(vsp_axis));memset(&vsp_group,0,sizeof(vsp_group));
}
static void complete(void){
 uint32_t request=g_it_request;CHECK(request==3 || request==14 || request==15);
 g_it_request=0;g_it_action=request;g_it_sequence++;tick+=100;
 if(request==3){
  if(g_it_axes&SPF_DM)g_dm_can2_logical_position_deg=g_it_dm_deg;
  if(g_it_axes&SPF_EXT)g_startup_extension_current_deg=-g_it_extension_mm*315.5f*360/3200;
  if(g_it_axes&SPF_LIFT)measured_depth=g_it_lift_pulses;
 }else if(request==14){
  measured_depth=118820;vsp_axis[1].current=measured_depth*360/3200;
  vsp_axis[1].position_cycle=1;vsp_axis[1].position_tick=tick;
 }else{
  top=1;measured_depth=7200;g_dm_can2_logical_position_deg=37;
  g_startup_extension_current_deg=0;
 }
 g_it_state=2;Low_Service(tick);
}
static void approach(void){
 g_low_request=1;Low_Service(tick);CHECK(Low_Busy() && g_it_request==3);
 CHECK(!it_auto_start(tick));CHECK(!PZQNear_TryStart(1,2,tick));CHECK(!Region_TryStart(tick));
 CHECK(Low_XYAllowed());complete();CHECK(g_low_phase==2 && g_it_axes==SPF_EXT);
 CHECK(Low_XYAllowed());complete();CHECK(g_low_phase==3 && !g_it_request);
 CHECK(!Low_PickAllowed() && !Low_XYAllowed());
}
static void confirm(void){
 g_low_confirm_pose_sequence=g_low_accepted_sequence;g_low_confirm_frame_cycle=g_vis_mat_cycle;
 g_vis_mat_tick=tick;g_low_request=2;Low_Service(tick);
 CHECK(g_low_phase==4 && g_it_request==14);CHECK(Low_PickAllowed());
}
static void run_piece(unsigned region,unsigned slot){
 float yaw,extension;reset();g_low_slot=slot;g_low_region=region;
 g_low_target_yaw=yaw=region==0?194.5f:(region==1?163.5f:180);
 g_low_target_extension_mm=extension=region==0?163:(region==1?159:0);
 approach();CHECK(g_it_carrier_auto_deg==(slot-1)*120.0f);
 /* Bad confirmation is a soft wait. Producer edits do not change the latched pose. */
 g_low_target_yaw=0;g_low_target_extension_mm=170;
 g_low_request=2;Low_Service(tick);CHECK(g_low_phase==3 && g_low_rejected==1 && !failures);
 g_low_confirm_pose_sequence=g_low_accepted_sequence;g_low_confirm_frame_cycle=g_vis_mat_cycle;
 g_vis_mat_tick=tick-201;g_low_request=2;Low_Service(tick);
 CHECK(g_low_phase==3 && !g_it_request && !failures);
 confirm();CHECK(g_it_dm_deg==yaw && g_it_extension_mm==extension);
 complete();CHECK(g_low_phase==5 && g_it_request==15 && grips==1);
 CHECK(fabsf(g_low_actual_pick_depth_pulses-118820)<.1f);
 CHECK(!Low_TransferGate(tick,118820-65*1600,1));
 CHECK(!Low_TransferGate(tick,118820-66*1600,0));CHECK(!g_low_clear_valid);
 CHECK(Low_TransferGate(tick,118820-65.01f*1600,1));CHECK(g_low_clear_valid);
 complete();CHECK(g_low_phase==6 && releases==1 && !g_it_request);
 tick=low_phase_tick+49;Low_Service(tick);CHECK(!g_it_request);
 tick++;Low_Service(tick);CHECK(g_low_phase==7 && g_it_request==3);
 complete();CHECK(g_low_phase==8 && g_it_extension_mm==65);
#ifdef LOW_OBSERVE_SETTLE_EXTENSION
 {
  uint32_t saved_state=g_it_state;g_it_state=1;
  vsp_group.command.kind=SPF_CMD_TRANSFER;vsp_group.command.axes_mask=SPF_EXT;
  vsp_group.command.goal.extension_mm=65;vsp_axis[0].before=0;
  CHECK(Low_ObserveReturnSettling());
  g_low_phase=2;CHECK(!Low_ObserveReturnSettling());g_low_phase=8;
  top=0;CHECK(!Low_ObserveReturnSettling());top=1;
  vsp_group.command.goal.extension_mm=64;CHECK(!Low_ObserveReturnSettling());
  vsp_group.command.goal.extension_mm=65;vsp_axis[0].before=10;
  CHECK(!Low_ObserveReturnSettling());vsp_axis[0].before=0;g_it_state=saved_state;
 }
#endif
 complete();CHECK(g_low_phase==9 && g_it_lift_pulses==62400);
 complete();CHECK(g_low_state==2 && g_low_done_sequence==1 && !failures);
 CHECK(g_dm_can2_logical_position_deg==180 && fabsf(measured_depth-62400)<1);
 g_low_request=1;Low_Service(tick);CHECK(g_low_rejected==3 && !g_it_request);
}
int main(void){
 unsigned region,slot;
 for(region=0;region<3;region++)for(slot=1;slot<=3;slot++)run_piece(region,slot);
 reset();g_low_sequence=0;CHECK(!Low_TryStart(tick));
 reset();g_low_pose_version=0;CHECK(!Low_TryStart(tick));
 reset();g_low_target_yaw=NAN;CHECK(!Low_TryStart(tick));
 reset();g_low_target_extension_mm=171;CHECK(!Low_TryStart(tick));
 reset();g_low_region=2;CHECK(!Low_TryStart(tick));
 reset();measured_depth=63000;CHECK(!Low_TryStart(tick));
 reset();query_fail=1;CHECK(!Low_TryStart(tick));
 reset();body=1;CHECK(!Low_TryStart(tick));
 reset();g_region_state=1;CHECK(!Low_TryStart(tick));
 reset();approach();g_low_confirm_pose_sequence=1;g_low_confirm_frame_cycle=10;
 query_delay=110;g_vis_mat_tick=tick;g_low_request=2;Low_Service(tick);
 CHECK(g_low_phase==3 && !g_it_request && !failures); /* expired during actual queries */
 reset();approach();confirm();complete(); /* stale actual pickup feedback must fault */
 CHECK(g_low_phase==5);platform=0;Low_Service(tick+1);CHECK(g_low_state==9 && failures==1);
 reset();approach();g_low_request=9;Low_Service(tick);CHECK(g_low_state==9 && g_it_request==9);
 reset();approach();Low_Service(tick+50000);CHECK(g_low_state==1 && g_low_phase==3 && !failures);
 reset();CHECK(Low_TryStart(tick));Low_Service(tick+16000);CHECK(g_low_state==9 && failures==1);
 printf("PASS actual low owner: %u checks; 3regions/3slots, new confirmation, actual strict65, return low65, old-owner exclusion\n",checks);
 return 0;
}
