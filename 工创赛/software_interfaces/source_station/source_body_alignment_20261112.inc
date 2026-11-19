/* Supervised alignment. Each axis/turn needs a fresh explicit request.
 * 1 legacy left10mm,2 axis1..20mm at15mm/s,3 signed5..10deg turn,
 * 7 finish a completed alignment chain,9 stop. Never rearms a fault.
 */
volatile uint32_t g_source_body_step_request,g_source_body_step_state;
volatile uint32_t g_source_body_step_reason,g_source_body_step_session;
volatile uint32_t g_source_body_step_elapsed_ms,g_source_body_step_submitted;
volatile uint32_t g_source_align_direction=1U;
volatile uint32_t g_source_align_command_seq,g_source_align_rejected;
volatile float g_source_align_distance_mm=10.0f,g_source_align_turn_deg=-5.0f;
static uint32_t s_body_step_begin,s_body_step_lift_samples,s_body_step_it_sequence;
static uint32_t s_align_chain,s_align_kind,s_align_direction;
static float s_align_distance,s_align_turn;
extern volatile uint32_t g_it_auto_state,g_lift_compare_backoff_valid;

uint32_t SourceBodyStep_KeepsPower(void){return g_source_body_step_state!=0U;}
uint32_t SourceBodyStep_WheelWindow(void){return g_source_body_step_state!=0U && g_source_body_step_state!=3U;}
uint32_t SourceBodyStep_Busy(void){return g_source_body_step_state==1U || g_source_body_step_state==2U || g_source_body_step_state==8U;}
uint32_t SourceBodyStep_SmallDistanceWindow(void){return s_align_chain && (g_source_body_step_state==1U || g_source_body_step_state==2U);}

static uint32_t SBS_Mechanics(void)
{
 float safe;
 if(!StartupAxes_Holding() || !g_lift_compare_top_valid || !g_lift_compare_backoff_valid ||
    !g_vehicle_lift_reference_valid || !g_vehicle_lift_feedback_valid ||
    !g_lift_ruler_controller.top_scale || g_lift_ruler_state!=LRC_DONE || g_lift_ruler_request ||
    g_lift_ruler_cause || g_lift_ruler_stop_pending || VehicleArm_Busy() || g_vehicle_arm_stop_pending ||
    g_vehicle_arm_cause || g_it_state!=2U || g_it_request || g_it_cause || g_it_rejected ||
    g_it_auto_state==1U || g_it_auto_state==9U || Stage2Body_Owns())return 0U;
 safe=(float)g_lift_ruler_controller.safe_position_raw/(float)g_lift_ruler_controller.top_scale;
 return fabsf(g_vehicle_lift_current_deg-safe)<=1.0f;
}
static void SBS_Fail(uint32_t reason)
{
 if(!g_source_body_step_reason)g_source_body_step_reason=reason;
 if(g_source_body_step_submitted){MotionStop();g_source_body_step_state=8U;}
 else g_source_body_step_state=9U;
}
static void SourceBodyStep_Service(uint32_t now)
{
 uint32_t request=g_source_body_step_request,continuation;
 MotionStatus status;ChassisAutoStatus body;
 g_source_body_step_request=0U;
 if(request==9U && SourceBodyStep_Busy()){SBS_Fail(9U);return;}
 if(request>=1U && request<=3U && SourceBodyStep_Busy()){g_source_align_rejected++;request=0U;}
 if(request==7U && g_source_body_step_state==3U && s_align_chain){
  if(!MotionFinish()){SBS_Fail(8U);return;}s_align_chain=0U;return;
 }
 if(request>=1U && request<=3U){
  continuation=g_source_body_step_state==3U && s_align_chain && request!=1U;
  if((g_source_body_step_state && !continuation) || !SBS_Mechanics() || g_combined_test.state ||
     (!continuation && (s_motion_session_open || s_motion_has_action)) ||
     (continuation && (!s_motion_session_open || !s_motion_has_action)) || g_chassis_auto_active ||
     g_safety_fault || g_rtos_fault || g_can_ctrl_fault_latched || !g_safety_ready ||
     g_can_test_result!=1U || !App_HeadingRuntimeSource(now) || !App_HeadingRuntimeExclusive()){
   if(!g_source_body_step_reason)g_source_body_step_reason=1U;return;
  }
  if((request==2U && (g_source_align_direction>3U || !(g_source_align_distance_mm>=1.0f && g_source_align_distance_mm<=20.0f))) ||
     (request==3U && !(fabsf(g_source_align_turn_deg)>=5.0f && fabsf(g_source_align_turn_deg)<=10.0f))){
   SBS_Fail(10U);return;
  }
  s_align_kind=request;s_align_direction=request==1U?MOVE_LEFT:g_source_align_direction;
  s_align_distance=request==1U?10.0f:g_source_align_distance_mm;s_align_turn=g_source_align_turn_deg;
  if(request!=1U)s_align_chain=1U;
  g_source_align_command_seq++;if(!g_source_align_command_seq)g_source_align_command_seq=1U;
  g_source_body_step_state=1U;g_source_body_step_submitted=g_source_body_step_session=0U;
  s_body_step_begin=now;s_body_step_lift_samples=g_lift_ruler_samples;s_body_step_it_sequence=g_it_sequence;
 }
 if(!SourceBodyStep_Busy())return;
 g_source_body_step_elapsed_ms=now-s_body_step_begin;(void)ChassisAuto_ReadStatus(&body);
 if(g_source_body_step_state==8U){if(!body.active && body.rotation_stop_mask==15U)g_source_body_step_state=9U;return;}
 if(!SBS_Mechanics() || g_lift_ruler_samples!=s_body_step_lift_samples || g_it_sequence!=s_body_step_it_sequence ||
    g_safety_fault || g_rtos_fault || g_can_ctrl_fault_latched || !App_HeadingRuntimeSource(now)){
  SBS_Fail(2U);return;
 }
 if(g_source_body_step_elapsed_ms>=12000U){SBS_Fail(3U);return;}
 if(g_source_body_step_state==1U){
  MotionProfile profile;uint32_t rpm,run_ms,accepted;
  if(!g_can_guard_ready)return;
  RouteHeading_Reset(&s_route_heading_reference);
  if(!RouteHeading_Ensure(&s_route_heading_reference,s_heading_source.yaw_deg,1U) ||
     !RouteHeading_Target(&s_route_heading_reference,s_align_kind==3U?s_align_turn:0.0f,&s_route_heading_target)){
   SBS_Fail(4U);return;
  }
  s_route_heading_turn_active=s_align_kind==3U;
  if(s_align_kind!=3U && !MotionRoute_Build(s_align_direction,s_align_distance,15.0f,&profile,&rpm,&run_ms)){SBS_Fail(4U);return;}
  s_motion_list_submit=1U;
  if(s_align_kind==3U)accepted=TurnStart(s_align_turn>0?TURN_CCW:TURN_CW,fabsf(s_align_turn),3U);
  else accepted=App_MoveSubmit(s_align_direction,rpm,run_ms,s_align_distance,15.0f);
  s_motion_list_submit=0U;
  if(!accepted){SBS_Fail(5U);return;}
  g_source_body_step_submitted=1U;g_source_body_step_state=2U;return;
 }
 if(!MotionReadStatus(&status)){SBS_Fail(6U);return;}
 if(status.backend_active && !g_source_body_step_session)g_source_body_step_session=body.session;
 if(status.state==MOTION_FAULT || status.state==MOTION_CANCELLED || status.backend_cause){SBS_Fail(6U);return;}
 if(status.state==MOTION_DONE){
  if(!g_source_body_step_session || body.session!=g_source_body_step_session || status.backend_active ||
     status.backend_result!=1U || status.stop_mask!=15U ||
     (s_align_kind==3U && (!s_combined_turn || fabsf(g_heading_exec_relative_deg-s_align_turn)>.001f)) ||
     (s_align_kind!=3U && (!status.distance_mode || !status.distance_reached ||
       status.distance_direction!=s_align_direction || fabsf(status.distance_target_cm-s_align_distance/10.0f)>.001f))){
   SBS_Fail(7U);return;
  }
  if(!s_align_chain && !MotionFinish()){SBS_Fail(8U);return;}
  g_source_body_step_state=3U;
 }
}
