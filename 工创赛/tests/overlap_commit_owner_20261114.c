#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "source_parallel_flow.h"
#include "vehicle_parameters.h"
#include "can.h"
#define APP_MOTOR_OFFLINE_ONLY 0
static unsigned checks;
#define CHECK(x) do{checks++;if(!(x)){fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x);exit(1);}}while(0)
typedef struct{uint32_t options,position_tick,position_cycle;float current,before;}VSP_Axis;
static VSP_Axis vsp_axis[2];static SPF_Group vsp_group;
static struct{uint32_t top_valid,top_scale,operation,backoff_valid,samples;int64_t position_after_raw,safe_position_raw;}g_lift_ruler_controller;
static uint32_t g_it_request,g_it_state,g_it_action,g_it_sequence,g_it_cause,g_it_rejected;
static uint32_t g_it_lead5_grip_done,g_it_carrier_auto_done,g_it_elapsed_ms,g_it_lift_permit;
static uint32_t g_lift_ruler_request,g_lift_ruler_stop_pending,g_lift_ruler_cause,g_startup_axes_cause,g_dm_jog_first_fault;
static uint32_t g_vis_material_valid,g_vis_mat_cycle,g_vis_mat_tick,g_vis_mat_left,g_vis_mat_right,g_vis_mat_down;
static uint32_t g_region_state,g_it_auto_state,g_pzq_near_state,g_chassis_auto_active;
static volatile uint32_t g_chassis_auto_request;
static float g_it_dm_deg,g_it_extension_mm,g_it_carrier_auto_deg,g_vehicle_lift_current_deg;
static float g_dm_can2_logical_position_deg,g_extension_home_reference_deg;
static uint32_t g_source_parallel_state,g_source_parallel_done_mask,g_source_parallel_stop_mask;
static uint32_t tick,platform=1,reference=1,body,owned,grips,releases,pulse_apply_at,failures,hold_calls,idle_reports;
static uint32_t vsp_fast_lift;
static float depth=600,extension=65;
typedef struct {SPF_Command command;uint32_t phase,tick;float before_depth,before_extension,before_yaw;}Record;
static Record records[16];static unsigned recorded;
static uint32_t xTaskGetTickCount(void){return tick;}
static float SingleTeach_TopDeg(void){return 0;}
static uint32_t it_platform(void){return platform && !g_chassis_auto_active;}
static uint32_t it_dm_fresh(void){return 1;}
static uint32_t it_at_top(void){return g_lift_ruler_controller.backoff_valid &&
 fabsf(g_vehicle_lift_current_deg-(float)g_lift_ruler_controller.safe_position_raw/100)<=SOURCE_TOP_TOLERANCE_DEG;}
static uint32_t SingleTeach_ReferencesReady(void){return reference;}
static uint32_t VehicleSourceParallel_Owns(void){return owned;}
static uint32_t VehicleArm_Busy(void){return owned;}
static uint32_t SourceBodyStep_Busy(void){return body;}
static uint32_t SourceBodyStep_WheelWindow(void){return body;}
static uint32_t Stage2Fast_IsActive(void){return 0;}
static uint32_t SourceStage2_IsActive(void){return 0;}
static uint32_t VSP_Position(uint8_t address,VSP_Axis *a){
 tick+=2;a->current=address==5?depth*360/3200:-extension*VEHICLE_EXTENSION_PULSES_PER_MM*360/3200;
 a->position_tick=tick;a->position_cycle++;if(address==5)g_vehicle_lift_current_deg=a->current;return 1;
}
static void it_fail(uint32_t why){failures++;g_it_state=9;g_it_cause=why;owned=0;}
static uint32_t jiazhua(float angle){
 CHECK(angle==140 || angle==160 || angle==210);
 if(angle==140)grips++;
 if(angle==210)releases++;
 pulse_apply_at=(tick/20+1)*20;return 1;
}
static uint32_t CarrierPwm_Write(uint32_t pulse){CHECK(pulse>=500 && pulse<=2500);return 1;}
static uint32_t Gripper_PulseApplied(void){return tick>=pulse_apply_at;}
static uint32_t CAN1_ReadStatus(uint8_t address,CanRxMsg *reply){CHECK(address>=1 && address<=4);reply->Data[1]=3;return 1;}
static uint32_t CAN1_ReadSpeed(uint8_t address,CanRxMsg *reply,float *speed){(void)reply;CHECK(address>=1 && address<=4);*speed=0;return 1;}
static uint32_t CAN1_HoldWheelCurrentPosition(uint8_t address,CanRxMsg *reply){(void)reply;CHECK(address>=1 && address<=4);hold_calls++;return 1;}
static void ChassisAuto_ReportIdleSpeed(uint32_t address,float speed,uint32_t when){CHECK(address<4 && speed==0 && when==tick);idle_reports++;}
static uint32_t VehicleArm_BeginSourceParallel(const SPF_Command *command,uint32_t sweep,uint32_t driving){
 CHECK(!owned && sweep==1 && driving==0 && recorded<16);
 CHECK(command->extension_rpm==SPF_EXT_RPM && command->lift_rpm==1846.4f && command->stepper_accel_rpm_s==600);
 records[recorded].command=*command;records[recorded].tick=tick;
 records[recorded].before_depth=depth;records[recorded].before_extension=extension;
 records[recorded++].before_yaw=g_dm_can2_logical_position_deg;
 memset(vsp_axis,0,sizeof(vsp_axis));memset(&vsp_group,0,sizeof(vsp_group));vsp_group.command=*command;
 vsp_group.first_start_ms=tick+10;vsp_group.last_start_ms=tick+20;
 owned=1;g_source_parallel_state=1;g_source_parallel_done_mask=0;return 1;
}
static void Low_ReadonlyDiag(void){}
#include "tested-commit-owner.inc"

static void reset(void){
 memset(records,0,sizeof(records));recorded=0;tick=1000;
 owned=body=failures=grips=releases=hold_calls=idle_reports=0;platform=reference=1;
 depth=600;extension=65;g_vehicle_lift_current_deg=depth*360/3200;
 memset(&g_lift_ruler_controller,0,sizeof(g_lift_ruler_controller));
 g_lift_ruler_controller.top_valid=g_lift_ruler_controller.backoff_valid=1;
 g_lift_ruler_controller.top_scale=100;g_lift_ruler_controller.safe_position_raw=(int64_t)(g_vehicle_lift_current_deg*100);
 g_it_state=2;g_it_request=g_it_action=g_it_sequence=g_it_cause=g_it_rejected=0;
 g_lift_ruler_request=g_lift_ruler_stop_pending=g_lift_ruler_cause=g_startup_axes_cause=g_dm_jog_first_fault=0;
 g_region_state=g_it_auto_state=g_pzq_near_state=g_chassis_auto_active=g_chassis_auto_request=0;
 g_source_parallel_state=g_source_parallel_done_mask=g_source_parallel_stop_mask=0;
 g_dm_can2_logical_position_deg=180;g_extension_home_reference_deg=0;
 g_low_request=g_low_state=g_low_phase=g_low_fault=g_low_rejected=0;
 g_low_accepted_sequence=g_low_done_sequence=0;g_low_sequence=1;
 g_low_slot=1;g_low_color=2;g_low_region=2;g_low_target_yaw=180;g_low_target_extension_mm=0;
 g_low_pose_version=g_low_scene_id=1;g_low_confirm_pose_sequence=1;g_low_confirm_frame_cycle=10;
 g_vis_material_valid=1;g_vis_mat_cycle=10;g_vis_mat_tick=tick;
 g_vis_mat_left=1;g_vis_mat_right=5;g_vis_mat_down=2;
 g_low_wheel_hold_request=g_low_wheel_hold_ready=g_low_wheel_hold_stage=0;
}
static void lock_wheels(void){
 unsigned i;g_low_wheel_hold_request=1;
 for(i=0;i<12;i++){tick+=10;Low_Service(tick);}
 CHECK(g_low_wheel_hold_ready==1 && hold_calls==4 && idle_reports==4 && !recorded);
}
static void finish_group(void){
 SPF_Command command=vsp_group.command;uint32_t axes=command.axes_mask;
 CHECK(owned);tick+=100;
 if(axes&SPF_DM)g_dm_can2_logical_position_deg=command.goal.dm_deg;
 if(axes&SPF_EXT){extension=command.goal.extension_mm;vsp_axis[0].current=-extension*VEHICLE_EXTENSION_PULSES_PER_MM*360/3200;vsp_axis[0].position_tick=tick;vsp_axis[0].position_cycle=1;}
 if(axes&SPF_LIFT){depth=command.goal.lift_pulses;vsp_axis[1].current=depth*360/3200;vsp_axis[1].position_tick=tick;vsp_axis[1].position_cycle=1;g_vehicle_lift_current_deg=vsp_axis[1].current;}
 g_source_parallel_state=3;g_source_parallel_done_mask=axes;owned=0;Low_Service(tick);
}
static void start_region(unsigned region,unsigned slot){
 reset();lock_wheels();g_low_region=region;g_low_slot=slot;
 g_low_target_yaw=region==0?194.5f:(region==1?163.1f:180);
 g_low_target_extension_mm=region==0?165.7f:(region==1?164:0);
 g_vis_mat_left=region==0?2:1;g_vis_mat_right=region==1?2:5;g_vis_mat_down=region==2?2:1;
 if(region==0)g_vis_mat_down=5;
 if(region==1)g_vis_mat_down=1;
 g_vis_mat_tick=tick;g_low_request=1;Low_Service(tick);
 CHECK(Low_Busy() && g_low_accepted_sequence==1 && g_it_action==17);
 CHECK(records[0].command.axes_mask==(region==2?(SPF_EXT|SPF_LIFT):SPF_DM));
 /* Camera loss and changed colours after commit cannot cancel the piece. */
 g_vis_material_valid=0;g_vis_mat_left=g_vis_mat_right=g_vis_mat_down=0;
}
static void piece(unsigned region,unsigned slot){
 uint32_t previous;unsigned loops=0;start_region(region,slot);
 while(Low_Busy() && ++loops<200){
  if(g_low_phase==6){
   CHECK(releases==1 && g_dm_can2_logical_position_deg==37 && extension==0);
   previous=recorded;tick=pulse_apply_at;Low_Service(tick);
   CHECK(recorded==previous);tick+=49;Low_Service(tick);CHECK(recorded==previous);
   tick++;Low_Service(tick);CHECK(recorded==previous+1 && g_low_phase==7);
   CHECK(records[recorded-1].command.axes_mask==7 && g_low_release_wait_ms==50);
   CHECK(g_low_release_pulse_tick>=low_begin+g_low_release_command_ms);
  }else if(!owned){
   tick+=10;Low_Service(tick); /* existing carrier700ms settle */
  }else{
   if(g_low_phase==2)CHECK(vsp_group.command.axes_mask==6 && vsp_group.command.goal.lift_pulses==15600);
   if(g_low_phase==5){
    CHECK(vsp_group.command.goal.extension_mm==fmaxf(0,g_low_target_extension_mm-60));
    CHECK(!(vsp_group.command.axes_mask&SPF_DM));
   }
   if(g_low_phase==12)CHECK(extension==0 && g_low_clear_valid && g_low_actual_rise_mm>65);
   finish_group();
  }
 }
 CHECK(g_low_state==2 && g_low_phase==10 && g_low_done_sequence==1 && !failures);
 CHECK(grips==1 && releases==1 && extension==65 && fabsf(depth-600)<.1f && g_dm_can2_logical_position_deg==180);
 CHECK(g_low_pick_overlap_axes==6 && g_low_return_overlap_axes==7 && g_it_state==2 && !g_it_request);
 CHECK(g_low_group_count==recorded);
 for(loops=0;loops<recorded;loops++)CHECK(g_low_group_done_ms[loops]>=g_low_group_started_ms[loops] && g_low_group_skew_ms[loops]<=90);
}
int main(void){
 unsigned region,slot;for(region=0;region<3;region++)for(slot=1;slot<=3;slot++)piece(region,slot);
 reset();g_low_request=1;Low_Service(tick);CHECK(!Low_Busy() && !recorded && g_low_rejected==1 && !failures);
 reset();lock_wheels();g_low_confirm_frame_cycle=0;g_low_request=1;Low_Service(tick);CHECK(!Low_Busy() && !recorded);
 reset();lock_wheels();g_vis_material_valid=0;g_low_request=1;Low_Service(tick);CHECK(Low_Busy() && recorded==1);
 start_region(2,1);g_low_request=9;Low_Service(tick);CHECK(g_low_state==9 && g_low_fault==2 && failures==1);
 start_region(0,1);g_source_parallel_state=4;Low_Service(tick);CHECK(g_low_state==9 && failures==1);
 start_region(1,1);platform=0;Low_Service(tick);CHECK(g_low_state==9 && failures==1 && platform==0);
 reset();g_low_state=1;g_low_phase=5;g_low_actual_pick_depth_pulses=29700;
 CHECK(!Low_TransferGate(tick,29700-65*400,1));CHECK(!Low_TransferGate(tick,29700-66*400,0));
 CHECK(Low_TransferGate(tick,29700-65.01f*400,1));
 printf("PASS actual committed owner: %u checks,3regions/3slots, no camera cancellation, overlap, real65, pulse+50, parallel return, hardware stop\n",checks);
 return 0;
}
