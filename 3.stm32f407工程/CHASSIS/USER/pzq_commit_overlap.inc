/* One fresh two-frame observation commits one complete piece.
 * No camera dependency exists after commit. Real platform/driver faults and STOP remain.
 * Public g_low names retain the supervised host ABI; this owner uses real SPF groups. */
#include "can.h"
extern volatile uint32_t g_chassis_auto_request;
volatile uint32_t g_low_request,g_low_state,g_low_phase,g_low_fault,g_low_rejected;
volatile uint32_t g_low_sequence,g_low_accepted_sequence,g_low_done_sequence;
volatile uint32_t g_low_slot=1U,g_low_color,g_low_region,g_low_pose_version,g_low_scene_id;
volatile float g_low_target_yaw=180.0f,g_low_target_extension_mm;
volatile uint32_t g_low_confirm_pose_sequence,g_low_confirm_frame_cycle;
volatile uint32_t g_low_elapsed_ms,g_low_clear_ms,g_low_clear_valid,g_low_release_ms;
volatile uint32_t g_low_grip_command_ms; /* Diagnostic, not physical contact confirmation. */
volatile float g_low_actual_pick_depth_pulses,g_low_actual_rise_mm;
volatile uint32_t g_low_commit_version=1U,g_low_commit_frame_cycle;
volatile uint32_t g_low_release_command_ms,g_low_release_pulse_tick,g_low_release_wait_ms;
volatile uint32_t g_low_pick_overlap_axes,g_low_lift_retract_axes,g_low_return_overlap_axes;
volatile uint32_t g_low_group_count,g_low_group_phase[8],g_low_group_axes[8];
volatile uint32_t g_low_group_skew_ms[8],g_low_group_started_ms[8],g_low_group_done_ms[8];
volatile uint32_t g_low_wheel_hold_request,g_low_wheel_hold_ready,g_low_wheel_hold_stage;
volatile uint32_t g_low_wheel_hold_result[4],g_low_wheel_hold_cycle;
static uint32_t low_begin,low_phase_tick,low_group_sequence,low_slot,low_release_applied;
static uint32_t low_last_piece,low_mission_piece;
static uint32_t low_tray_tick,low_pulse_tick,low_group_record_index;
static uint32_t low_top_done_sequence,low_top_done_piece;
static uint32_t low_departure_clear_piece;
volatile uint32_t g_low_departure_clear_ms;
static uint32_t Low_DepartureClear(void)
{return low_departure_clear_piece && low_departure_clear_piece==g_low_accepted_sequence;}
static float low_yaw,low_extension,low_measured_depth,low_pick_depth;
static uint32_t low_depth_tick,low_ext_tick;

static uint32_t Low_Busy(void){return g_low_state==1U;}
static uint32_t Low_Loaded(void)
{return Low_Busy() && (g_low_phase==5U || g_low_phase==11U || g_low_phase==12U);}
static uint32_t Low_ObserveReturnSettling(void){return 0U;}
static uint32_t Low_PoseAllowed(void)
{return Low_Busy() && g_low_phase==4U && g_it_dm_deg==low_yaw;}
static uint32_t Low_PickAllowed(void){return Low_PoseAllowed();}
static uint32_t Low_XYAllowed(void)
{return Low_Busy() && (g_low_phase==1U ||
 (g_low_phase==8U && low_top_done_sequence && low_top_done_piece==g_low_accepted_sequence));}
static float Low_CommandPickDepth(uint32_t region)
{return (float)SOURCE_PICK_PULSES-(region<2U?10.0f*SOURCE_LIFT_PULSES_PER_MM:0.0f);}
static uint32_t Low_ReadDepth(float *depth)
{
#if APP_MOTOR_OFFLINE_ONLY
 (void)depth;return 0U;
#else
 VSP_Axis sample;memset(&sample,0,sizeof(sample));
 if(!g_lift_ruler_controller.top_valid || !g_lift_ruler_controller.top_scale ||
    VehicleSourceParallel_Owns())return 0U;
 sample.options=g_lift_ruler_controller.top_scale==100U?0x80U:0U;
 if(!VSP_Position(5U,&sample))return 0U;
 *depth=(sample.current-SingleTeach_TopDeg())*(3200.0f/360.0f);
 low_measured_depth=*depth;low_depth_tick=(uint32_t)xTaskGetTickCount();
 return *depth>=0.0f && *depth<=(float)SPF_MAX_LIFT_PULSES+400.0f;
#endif
}
static uint32_t Low_ExtensionAt(float mm)
{
#if APP_MOTOR_OFFLINE_ONLY
 (void)mm;return 0U;
#else
 VSP_Axis sample;memset(&sample,0,sizeof(sample));
 if(VehicleSourceParallel_Owns() || !VSP_Position(6U,&sample))return 0U;
 low_ext_tick=(uint32_t)xTaskGetTickCount();
 return fabsf(sample.current-g_extension_home_reference_deg-
  STARTUP_EXTENSION_EXTEND_SIGN*mm*VEHICLE_EXTENSION_PULSES_PER_MM*360.0f/3200.0f)<=
  (mm==0.0f?SOURCE_EXTENSION_ZERO_DONE_DEG:SOURCE_EXTENSION_DONE_DEG);
#endif
}
static uint32_t Low_TransferGate(uint32_t now,float depth,uint32_t fresh)
{
 if(!Low_Loaded() || !fresh || depth<0.0f)return 0U;
 g_low_actual_rise_mm=floorf(g_low_actual_pick_depth_pulses-depth)/SOURCE_LIFT_PULSES_PER_MM;
 /* Keep the prior nominal absolute release height after the side Z offset. */
 if(g_low_actual_rise_mm<=65.0f-((float)SOURCE_PICK_PULSES-low_pick_depth)/SOURCE_LIFT_PULSES_PER_MM)return 0U;
 if(!g_low_clear_valid){g_low_clear_valid=1U;g_low_clear_ms=now-low_begin;}
 return 1U;
}
/* This runs in the existing sole CAN owner, never a second reader/task.
 * DM prepare can retarget immediately: do not release it before actual gates. */
static void Low_ReleaseLoadedTransfer(uint32_t now)
{
#if !APP_MOTOR_OFFLINE_ONLY
 VSP_Axis *z=&vsp_axis[1],*x=&vsp_axis[0];float depth;
 if(!Low_Loaded() || g_low_phase!=5U || !VehicleSourceParallel_Owns() ||
    vsp_group.command.sequence!=low_group_sequence || vsp_group.state!=2U ||
    vsp_group.delayed_mask!=SPF_DM || (vsp_group.release_mask&SPF_DM) ||
    !(vsp_group.done&SPF_LIFT) || !SingleTeach_ReferencesReady())return;
 /* SPF stops polling completed axes. Refresh only those, without restarting
  * them; an absent EXT axis still needs actual home feedback in this group. */
 if(!z->position_cycle || (uint32_t)(now-z->position_tick)>150U){
  if(!VSP_Position(5U,z))return;
 }
 now=(uint32_t)xTaskGetTickCount();
 if(!(vsp_group.command.axes_mask&SPF_EXT) || (vsp_group.done&SPF_EXT)){
  if(!x->position_cycle || (uint32_t)(now-x->position_tick)>150U){
   if(!VSP_Position(6U,x))return;
  }
 }
 now=(uint32_t)xTaskGetTickCount();
 if(!z->position_cycle || !x->position_cycle ||
    (uint32_t)(now-z->position_tick)>150U || (uint32_t)(now-x->position_tick)>150U)return;
 depth=(z->current-SingleTeach_TopDeg())*(3200.0f/360.0f);
 if(!(depth>=0.0f && fabsf(depth-(float)SOURCE_RETURN_PULSES)<=SOURCE_TOP_TOLERANCE_DEG*(3200.0f/360.0f)) ||
    !(fabsf(x->current-g_extension_home_reference_deg)<=30.0f*VEHICLE_EXTENSION_PULSES_PER_MM*360.0f/3200.0f))return;
 if(Low_TransferGate(now,depth,1U))vsp_group.release_mask|=SPF_DM;
#else
 (void)now;
#endif
}

static void Low_Fail(uint32_t why)
{if(!g_low_fault)g_low_fault=why;g_low_state=9U;it_fail(why);}

/* The only new wheel control is relative-zero Emm position holding.
 * It is armed while stopped before observation, never interleaved with arm motion. */
/* BEGIN WHEEL DIAG ACK£ºÓ¦´ðÖ»¹©Õï¶Ï£¬²»ÄÜÖ¤Ã÷³ÖÐøËøÂÖ¡£ */
#include "stage3_cycle.h"
typedef struct {
 uint32_t cycle,result,tick,valid,data[3];
} WheelDiagAck;
volatile WheelDiagAck g_wheel_diag_ack[4];
volatile uint32_t g_wheel_diag_epoch,g_wheel_diag_run,g_wheel_diag_attempt_mask;
volatile uint32_t g_wheel_diag_ready_valid,g_wheel_diag_ready_tick;
static void WheelDiag_Reset(void)
{
 g_wheel_diag_ready_valid=0U;g_wheel_diag_ready_tick=0U;
 g_wheel_diag_attempt_mask=0U;
 memset((void*)g_wheel_diag_ack,0,sizeof(g_wheel_diag_ack));
 g_wheel_diag_run=Stage3_Owns()?g_stage3_run:0U;
 g_wheel_diag_epoch=g_low_wheel_hold_cycle;
}
static void WheelDiag_Record(uint32_t address,uint32_t result,const CanRxMsg *reply)
{
 volatile WheelDiagAck *ack=&g_wheel_diag_ack[address-1U];
 uint32_t valid=(reply->IDE==CAN_Id_Extended && reply->RTR==CAN_RTR_Data &&
  reply->ExtId==(address<<8U) && reply->DLC==3U && reply->Data[0]==0xfdU && reply->Data[2]==0x6bU);
 ack->cycle=g_wheel_diag_epoch;ack->result=result;ack->tick=(uint32_t)xTaskGetTickCount();
 ack->valid=valid;
 ack->data[0]=valid?reply->Data[0]:0U;
 ack->data[1]=valid?reply->Data[1]:0U;
 ack->data[2]=valid?reply->Data[2]:0U;
 g_wheel_diag_attempt_mask|=1UL<<(address-1U);
}
/* END WHEEL DIAG ACK */

static void WheelHold_Service(void)
{
 uint32_t address,result;CanRxMsg reply;float speed;
 if(g_chassis_auto_active || g_chassis_auto_request)g_low_wheel_hold_ready=0U;
 if(g_low_wheel_hold_request){
  g_low_wheel_hold_request=0U;
  g_low_wheel_hold_ready=0U;g_low_wheel_hold_stage=0U;
  if(Low_Busy() || g_it_state!=2U || g_it_cause || g_it_rejected || !SingleTeach_ReferencesReady() || g_chassis_auto_active ||
     SourceBodyStep_Busy() || SourceBodyStep_WheelWindow() || !it_platform())return;
  memset((void*)g_low_wheel_hold_result,0,sizeof(g_low_wheel_hold_result));
  g_low_wheel_hold_stage=1U;g_low_wheel_hold_ready=0U;g_low_wheel_hold_cycle++;
  WheelDiag_Reset();
 }
 if(!g_low_wheel_hold_stage || g_low_wheel_hold_stage>12U || Low_Busy())return;
 if(!it_platform() || g_chassis_auto_active){g_low_wheel_hold_stage=99U;return;}
 address=(g_low_wheel_hold_stage-1U)/3U+1U;memset(&reply,0,sizeof(reply));
 switch((g_low_wheel_hold_stage-1U)%3U){
 case 0U:
  result=CAN1_ReadStatus((uint8_t)address,&reply);
  if(result!=1U || !(reply.Data[1]&1U) || (reply.Data[1]&12U)){
   g_low_wheel_hold_result[address-1U]=result==1U?7U:result;g_low_wheel_hold_stage=99U;return;}
  break;
 case 1U:
  result=CAN1_ReadSpeed((uint8_t)address,&reply,&speed);
  if(result!=1U || speed!=0.0f){g_low_wheel_hold_result[address-1U]=result==1U?7U:result;g_low_wheel_hold_stage=99U;return;}
  ChassisAuto_ReportIdleSpeed(address-1U,speed,(uint32_t)xTaskGetTickCount());
  break;
 default:
  result=CAN1_HoldWheelCurrentPosition((uint8_t)address,&reply);
  WheelDiag_Record(address,result,&reply);
  g_low_wheel_hold_result[address-1U]=result;
  if(result!=1U){g_low_wheel_hold_stage=99U;return;}
  break;
 }
 g_low_wheel_hold_stage++;
 if(g_low_wheel_hold_stage==13U)g_low_wheel_hold_ready=1U;
 if(g_low_wheel_hold_stage==13U){
  g_wheel_diag_ready_tick=(uint32_t)xTaskGetTickCount();g_wheel_diag_ready_valid=1U;
 }
}

static uint32_t Low_Group(uint32_t phase,uint32_t axes,float yaw,float extension,float depth)
{
#if APP_MOTOR_OFFLINE_ONLY
 (void)phase;(void)axes;(void)yaw;(void)extension;(void)depth;return 0U;
#else
 SPF_Command command;
 if(phase==11U && (axes!=SPF_LIFT || yaw!=36.5f || extension!=0.0f ||
    depth!=(float)(SOURCE_RETURN_PULSES+SPF_CARRIER_EXTRA_DOWN_PULSES)))return 0U;
 if(phase==7U && (axes!=SPF_LIFT || yaw!=0.0f || extension!=0.0f ||
    depth!=(float)SOURCE_OBSERVE_PULSES))return 0U;
 if(phase==8U && (!low_top_done_sequence || low_top_done_piece!=g_low_accepted_sequence ||
    g_low_phase!=7U || VehicleSourceParallel_Owns() || g_source_parallel_state!=3U ||
    vsp_group.command.sequence!=low_top_done_sequence ||
    vsp_group.command.axes_mask!=g_low_lift_retract_axes ||
    g_source_parallel_done_mask!=g_low_lift_retract_axes ||
    !it_at_top() || axes!=(SPF_DM|SPF_EXT) || depth!=(float)SOURCE_OBSERVE_PULSES ||
    yaw!=(low_last_piece?0.0f:180.0f) || extension!=(low_last_piece?0.0f:65.0f)))return 0U;
 memset(&command,0,sizeof(command));
 command.sequence=++g_it_sequence;command.kind=phase==2U || phase==4U?SPF_CMD_PICK:SPF_CMD_TRANSFER;
 command.axes_mask=axes;command.goal.dm_deg=yaw;command.goal.extension_mm=extension;
 command.goal.lift_pulses=depth;command.dm_deg_s=SPF_DM_DEG_S;
 command.extension_rpm=SPF_EXT_RPM;command.lift_rpm=VEHICLE_SOURCE_LIFT_RPM;
 if(phase==11U){command.kind=SPF_CMD_CARRIER_LOWER;command.lift_rpm=SPF_CARRIER_LOWER_LIFT_RPM;}
 if(phase==7U){
  command.kind=SPF_CMD_SOURCE_EMPTY_OBSERVE_RETURN;
  command.lift_rpm=SPF_SOURCE_EMPTY_OBSERVE_RETURN_LIFT_RPM;
 }
 command.stepper_accel_rpm_s=SPF_STEPPER_ACCEL;command.keep_vm_on=1U;
 if(!VehicleArm_BeginSourceParallel(&command,1U,0U))return 0U;
 vsp_fast_lift=(axes&SPF_LIFT)?1U:0U;
 if(phase==5U){vsp_group.delayed_mask=SPF_DM;vsp_group.release_mask=axes&~SPF_DM;}
 /* Explicit Stage2 scope only: two steppers cache before one broadcast.
  * Loaded transfer retains the existing delayed-DM clearance gate. */
 if(phase==4U || phase==5U || (phase==7U && low_slot<3U)){
  vsp_pick_sync=((axes&(SPF_EXT|SPF_LIFT))==(SPF_EXT|SPF_LIFT));
  g_dm_stage2_sync=vsp_pick_sync && phase!=5U && (axes&SPF_DM);
 }
 low_group_record_index=g_low_group_count;
 if(low_group_record_index<8U){
  g_low_group_phase[low_group_record_index]=phase;g_low_group_axes[low_group_record_index]=axes;
  g_low_group_count++;
 }
 low_group_sequence=command.sequence;g_low_phase=phase;
 low_phase_tick=(uint32_t)xTaskGetTickCount();return 1U;
#endif
}
static void Low_PublishLift(uint32_t at_observation_or_place)
{
#if !APP_MOTOR_OFFLINE_ONLY
 if(!(vsp_group.command.axes_mask&SPF_LIFT) || !vsp_axis[1].position_cycle)return;
 g_lift_ruler_controller.position_after_raw=(int64_t)(vsp_axis[1].current*
  (float)g_lift_ruler_controller.top_scale+0.5f);
 g_vehicle_lift_current_deg=vsp_axis[1].current;
 g_lift_ruler_controller.operation=at_observation_or_place?3U:2U;
 g_lift_ruler_controller.backoff_valid=at_observation_or_place;
 if(at_observation_or_place)g_lift_ruler_controller.safe_position_raw=g_lift_ruler_controller.position_after_raw;
 g_lift_ruler_controller.samples++;
#else
 (void)at_observation_or_place;
#endif
}
static uint32_t Low_EmptyApproach(void)
{
 g_low_pick_overlap_axes=SPF_EXT|SPF_LIFT|(g_low_region<2U?SPF_DM:0U);
 return Low_Group(4U,g_low_pick_overlap_axes,low_yaw,low_extension,low_pick_depth);
}

/* A bound mission passes the piece ordinal separately from the color-mapped slot.
 * Ordinal 0 retains the stand-alone low-pick slot convention. */
static uint32_t Low_TryStartForPiece(uint32_t now,uint32_t ordinal)
{
 float depth;(void)now;
 if(ordinal>3U || !g_low_sequence || g_low_sequence==g_low_accepted_sequence || Low_Busy() || g_low_state==9U ||
  g_region_state==1U || g_it_auto_state==1U || g_it_auto_state==9U || g_pzq_near_state==1U ||
  g_it_state!=2U || g_it_request || g_it_cause || g_it_rejected || !it_platform() || !it_dm_fresh() ||
  !SingleTeach_ReferencesReady() || !g_low_wheel_hold_ready || SourceBodyStep_Busy() ||
  SourceBodyStep_WheelWindow() || VehicleArm_Busy() || Stage2Fast_IsActive() || SourceStage2_IsActive() ||
  g_lift_ruler_request || g_lift_ruler_stop_pending || !g_low_pose_version || !g_low_scene_id ||
  g_low_slot<1U || g_low_slot>3U || g_low_region>2U || g_low_color<1U || g_low_color>6U ||
  !(g_low_target_yaw>=0.0f && g_low_target_yaw<=270.0f) ||
  !(g_low_target_extension_mm>=0.0f && g_low_target_extension_mm<=170.0f) ||
  (g_low_region==2U && (g_low_target_yaw!=180.0f || g_low_target_extension_mm!=0.0f)) ||
  g_low_confirm_pose_sequence!=g_low_sequence || !g_low_confirm_frame_cycle || fabsf(g_dm_can2_logical_position_deg-180.0f)>1.0f)return 0U;
 /* The host has already latched the sole two-frame decision. No new image can revoke it. */
 if(!Low_ReadDepth(&depth) || !it_at_top() || !Low_ExtensionAt(65.0f))return 0U;
 low_slot=g_low_slot;low_yaw=g_low_target_yaw;low_extension=g_low_target_extension_mm;
 low_last_piece=ordinal?ordinal==3U:low_slot==3U;low_mission_piece=ordinal!=0U;
 low_pick_depth=Low_CommandPickDepth(g_low_region);
 g_low_accepted_sequence=g_low_sequence;g_low_state=1U;g_low_commit_frame_cycle=g_low_confirm_frame_cycle;
 g_low_grip_command_ms=0U;g_low_fault=g_low_elapsed_ms=g_low_clear_ms=g_low_clear_valid=g_low_release_ms=0U;
 g_low_release_command_ms=g_low_release_pulse_tick=g_low_release_wait_ms=0U;
 g_low_actual_pick_depth_pulses=g_low_actual_rise_mm=0.0f;g_it_lead5_grip_done=0U;
 g_low_group_count=0U;
 memset((void*)g_low_group_phase,0,sizeof(g_low_group_phase));memset((void*)g_low_group_axes,0,sizeof(g_low_group_axes));
 memset((void*)g_low_group_skew_ms,0,sizeof(g_low_group_skew_ms));memset((void*)g_low_group_started_ms,0,sizeof(g_low_group_started_ms));
 memset((void*)g_low_group_done_ms,0,sizeof(g_low_group_done_ms));
 low_begin=(uint32_t)xTaskGetTickCount();low_release_applied=0U;
 low_top_done_sequence=low_top_done_piece=0U;
 low_departure_clear_piece=g_low_departure_clear_ms=0U;
 g_it_state=1U;g_it_action=17U;g_it_dm_deg=low_yaw;g_it_extension_mm=low_extension;
 g_it_carrier_auto_deg=(float)(low_slot-1U)*120.0f;
 if(jiazhua(g_low_region<2U?SOURCE_PREGRIP_DEG:170.0f)!=1U || CarrierPwm_Write((uint32_t)(500.0f+g_it_carrier_auto_deg*2000.0f/270.0f+0.5f))!=1U){Low_Fail(7U);return 1U;}
 low_tray_tick=(uint32_t)xTaskGetTickCount();g_it_carrier_auto_done=1U;
 if(!Low_EmptyApproach())Low_Fail(8U);
 return 1U;
}

static uint32_t Low_TryStart(uint32_t now)
{return Low_TryStartForPiece(now,0U);}

#include "low_readonly_diag.inc"
static void Low_Service(uint32_t now)
{
 uint32_t request=g_low_request;g_low_request=0U;
 Low_ReadonlyDiag();WheelHold_Service();
 if(request==9U && Low_Busy()){Low_Fail(2U);return;}
 if(request==1U && !Low_TryStart(now))g_low_rejected++;
 /* The obsolete second visual-confirm request cannot change this transaction. */
 if(!Low_Busy())return;
 now=(uint32_t)xTaskGetTickCount();g_low_elapsed_ms=now-low_begin;g_it_elapsed_ms=g_low_elapsed_ms;
 if(!it_platform() || g_lift_ruler_cause || g_startup_axes_cause || g_dm_jog_first_fault ||
  g_it_cause || g_it_rejected){Low_Fail(g_it_cause?g_it_cause:12U);return;}
#if !APP_MOTOR_OFFLINE_ONLY
 Low_ReleaseLoadedTransfer(now);
 now=(uint32_t)xTaskGetTickCount();
 /* Existing carrier transfer height is the physical retreat boundary, not
  * a new delay or a top/home requirement. Keep servicing Z during departure. */
 if(low_last_piece && g_low_phase==7U && low_release_applied &&
    g_low_release_wait_ms>=SOURCE_RELEASE_DWELL_MS &&
    vsp_group.command.sequence==low_group_sequence &&
    vsp_group.command.kind==SPF_CMD_TRANSFER &&
    vsp_group.command.goal.lift_pulses==(float)SOURCE_OBSERVE_PULSES &&
    vsp_axis[1].position_cycle && (uint32_t)(now-vsp_axis[1].position_tick)<=150U &&
    fabsf(vsp_axis[1].current-(SingleTeach_TopDeg()+(float)SOURCE_RETURN_PULSES*(360.0f/3200.0f)))<=SOURCE_TOP_TOLERANCE_DEG){
  if(!low_departure_clear_piece)g_low_departure_clear_ms=now-low_begin;
  low_departure_clear_piece=g_low_accepted_sequence;
 }
 if(Low_Loaded() && (vsp_group.command.axes_mask&SPF_LIFT) && vsp_axis[1].position_cycle)
  (void)Low_TransferGate(now,(vsp_axis[1].current-SingleTeach_TopDeg())*(3200.0f/360.0f),
   (uint32_t)(now-vsp_axis[1].position_tick)<=150U);
 /* v1.11ï¼šå·¦å³ä¿æŒå¼ çˆªï¼›Zè¿›å…¥åŽŸ20mmçª—å£ä¸”X/4310å®žé™…åˆ°ä½åŽæ‰å‘å¤¹ç´§ã€‚ */
 if(g_low_phase==4U && !g_it_lead5_grip_done && g_source_parallel_state!=4U &&
    (g_low_region>=2U ||
     (vsp_axis[0].position_cycle && (uint32_t)(now-vsp_axis[0].position_tick)<=150U &&
      fabsf(vsp_axis[0].current-vsp_axis[0].goal)<=SOURCE_EXTENSION_DONE_DEG &&
      it_dm_fresh() && fabsf(g_dm_can2_logical_position_deg-low_yaw)<=1.0f)) &&
    vsp_group.command.sequence==low_group_sequence &&
    (vsp_group.command.axes_mask&SPF_LIFT) && (vsp_group.started&SPF_LIFT) &&
    vsp_axis[1].position_cycle && (uint32_t)(now-vsp_axis[1].position_tick)<=150U &&
    (vsp_axis[1].current-SingleTeach_TopDeg())*(3200.0f/360.0f)>=low_pick_depth-SOURCE_GRIP_LEAD_MM*SOURCE_LIFT_PULSES_PER_MM){
  if(jiazhua(140.0f)!=1U){Low_Fail(7U);return;}
  g_low_grip_command_ms=(uint32_t)xTaskGetTickCount()-low_begin;g_it_lead5_grip_done=1U;
 }
#endif
 if(g_low_phase==6U){
  if(!low_release_applied){
   if(!Gripper_PulseApplied())return;
   now=(uint32_t)xTaskGetTickCount();
   low_release_applied=1U;low_pulse_tick=now;g_low_release_pulse_tick=now;g_low_release_ms=now-low_begin;
  }
  g_low_release_wait_ms=now-low_pulse_tick;
  if(g_low_release_wait_ms<SOURCE_RELEASE_DWELL_MS)return;
#if !APP_MOTOR_OFFLINE_ONLY
  if(low_last_piece && vsp_group.command.sequence==low_group_sequence &&
     vsp_group.state==3U && g_source_parallel_done_mask==vsp_group.command.axes_mask &&
     vsp_axis[1].position_cycle && it_at_top()){
   if(!low_departure_clear_piece)g_low_departure_clear_ms=now-low_begin;
   low_departure_clear_piece=g_low_accepted_sequence;
  }
#endif
  /* The completed loaded return already reached observation Z. No extra Z move. */
  g_low_return_overlap_axes=0U;g_low_phase=7U;
  return;
 }
#if !APP_MOTOR_OFFLINE_ONLY
 if(g_source_parallel_state==4U){Low_Fail(8U);return;}
 if(VehicleSourceParallel_Owns() || g_source_parallel_state!=3U ||
    vsp_group.command.sequence!=low_group_sequence ||
    g_source_parallel_done_mask!=vsp_group.command.axes_mask)return;
 if(low_group_record_index<8U && !g_low_group_done_ms[low_group_record_index]){
  g_low_group_skew_ms[low_group_record_index]=vsp_group.last_start_ms-vsp_group.first_start_ms;
  g_low_group_started_ms[low_group_record_index]=vsp_group.first_start_ms-low_begin;
  g_low_group_done_ms[low_group_record_index]=now-low_begin;
 }
 switch(g_low_phase){
 case 1U:if(!Low_EmptyApproach())Low_Fail(8U);
  break;
 case 4U:
  if(!vsp_axis[1].position_cycle){Low_Fail(14U);return;}
  g_low_actual_pick_depth_pulses=(vsp_axis[1].current-SingleTeach_TopDeg())*(3200.0f/360.0f);
  Low_PublishLift(0U);
  if(!g_it_lead5_grip_done){
   if(jiazhua(140.0f)!=1U){Low_Fail(7U);return;}
   g_low_grip_command_ms=(uint32_t)xTaskGetTickCount()-low_begin;g_it_lead5_grip_done=1U;
  }
  g_low_lift_retract_axes=SPF_DM|SPF_LIFT|(low_extension>0.0f?SPF_EXT:0U);
  if(!Low_Group(5U,g_low_lift_retract_axes,36.5f,
     0.0f,(float)SOURCE_RETURN_PULSES))Low_Fail(8U);
  break;
 case 5U:
  Low_PublishLift(1U); /* Exact loaded group DONE at the pre-existing observation height. */
  g_low_phase=12U;
  /* fall through: release at tray yaw, without the 10mm lowering group. */
 case 11U:
  Low_PublishLift(1U); /* Legacy label; actual completed return is already at observation Z. */
  g_low_phase=12U;
  /* fall through */
 case 12U:
  if((uint32_t)(now-low_tray_tick)<700U)return;
  if(jiazhua(160.0f)!=1U){Low_Fail(7U);return;}
  g_low_release_command_ms=(uint32_t)xTaskGetTickCount()-low_begin;
  low_release_applied=0U;g_low_phase=6U;low_phase_tick=(uint32_t)xTaskGetTickCount();break;
 case 7U:
  if(!vsp_axis[1].position_cycle || (uint32_t)(now-vsp_axis[1].position_tick)>150U){
   /* Completed SPF axes stop polling; this CAN owner refreshes Z while
    * waiting for actual chassis motion, rather than reusing an old sample. */
   if(!VSP_Position(5U,&vsp_axis[1])){Low_Fail(14U);return;}
   now=(uint32_t)xTaskGetTickCount();
  }
  if(vsp_group.command.kind!=SPF_CMD_TRANSFER ||
     vsp_group.command.goal.lift_pulses!=(float)SOURCE_OBSERVE_PULSES ||
     vsp_group.command.axes_mask!=g_low_lift_retract_axes || !vsp_axis[1].position_cycle ||
     (uint32_t)(now-vsp_axis[1].position_tick)>150U ||
     fabsf((vsp_axis[1].current-SingleTeach_TopDeg())*(3200.0f/360.0f)-
       (float)SOURCE_OBSERVE_PULSES)>SOURCE_TOP_TOLERANCE_DEG*(3200.0f/360.0f)){
   Low_Fail(14U);return;
  }
  if(low_last_piece && low_mission_piece){
   ChassisZone2CanStatus body;uint32_t i,moving=0U;
   /* The removed Z move used to provide this handoff window. Keep phase7
    * until the same mission commits departure and actual wheels start. */
   if(!RegionDispatch_SourceDepartureCommitted())return;
   ChassisAuto_Zone2Read(&body);
   if(body.state!=CHASSIS_AUTO_RUN || !body.feedback_ready)return;
   for(i=0U;i<4U;i++)if(body.wheel_decirpm[i])moving=1U;
   if(!moving)return; /* 4310 returns only after actual chassis motion feedback. */
  }
  low_top_done_sequence=low_group_sequence;low_top_done_piece=g_low_accepted_sequence;
  Low_PublishLift(1U);
  /* Release PWM+200ms already completed. Open during the empty return,
   * rather than only after reaching the next observation pose. */
  if(!low_last_piece && jiazhua(210.0f)!=1U){Low_Fail(7U);return;}
  if(!Low_Group(8U,SPF_DM|SPF_EXT,low_last_piece?0.0f:180.0f,
     low_last_piece?0.0f:65.0f,(float)SOURCE_OBSERVE_PULSES))Low_Fail(8U);
  break;
 case 8U:
  g_low_state=2U;g_low_phase=10U;
  g_low_done_sequence=g_low_accepted_sequence;g_it_state=2U;g_it_lift_permit=0U;break;
 default:Low_Fail(13U);break;
 }
#endif
}
