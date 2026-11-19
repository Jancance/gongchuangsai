/* 仅原TaskA提交；真实速度/位置/STOP仍由原唯一CAN任务执行。
 * state0空闲1等真实wheel guard2运行3正常完成8停止中9失败。
 * 一次上电只试一次。成功不清backend DONE，失败不清原故障。
 */
volatile uint32_t g_source_body_step_request,g_source_body_step_state;
volatile uint32_t g_source_body_step_reason,g_source_body_step_session;
volatile uint32_t g_source_body_step_elapsed_ms,g_source_body_step_submitted;
static uint32_t s_body_step_begin,s_body_step_lift_samples,s_body_step_it_sequence;
extern volatile uint32_t g_it_auto_state,g_lift_compare_backoff_valid;

uint32_t SourceBodyStep_KeepsPower(void)
/* 这是动作所有权发出的VM保持请求，不是供电健康证明。 */
{return g_source_body_step_state!=0U;}
uint32_t SourceBodyStep_WheelWindow(void)
{return g_source_body_step_state!=0U && g_source_body_step_state!=3U;}
uint32_t SourceBodyStep_Busy(void)
{return g_source_body_step_state==1U || g_source_body_step_state==2U || g_source_body_step_state==8U;}

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
 uint32_t request=g_source_body_step_request;MotionStatus status;ChassisAutoStatus body;
 g_source_body_step_request=0U;
 if(request==9U && SourceBodyStep_Busy()){SBS_Fail(9U);return;}
 if(request==1U){
  if(g_source_body_step_state || !SBS_Mechanics() || g_combined_test.state ||
     s_motion_session_open || s_motion_has_action || g_chassis_auto_active ||
     g_safety_fault || g_rtos_fault || g_can_ctrl_fault_latched || !g_safety_ready ||
     g_can_test_result!=1U || !App_HeadingRuntimeSource(now) || !App_HeadingRuntimeExclusive()){
   if(!g_source_body_step_reason)g_source_body_step_reason=1U;return;
  }
  g_source_body_step_state=1U;s_body_step_begin=now;
  s_body_step_lift_samples=g_lift_ruler_samples;s_body_step_it_sequence=g_it_sequence;
 }
 if(!SourceBodyStep_Busy())return;
 g_source_body_step_elapsed_ms=now-s_body_step_begin;
 (void)ChassisAuto_ReadStatus(&body);
 if(g_source_body_step_state==8U){
  if(!body.active && body.rotation_stop_mask==15U){g_source_body_step_state=9U;}
  return;
 }
 if(!SBS_Mechanics() || g_lift_ruler_samples!=s_body_step_lift_samples || g_it_sequence!=s_body_step_it_sequence ||
    g_safety_fault || g_rtos_fault || g_can_ctrl_fault_latched || !App_HeadingRuntimeSource(now)){
  SBS_Fail(2U);return;
 }
 if(g_source_body_step_elapsed_ms>=12000U){SBS_Fail(3U);return;}
 if(g_source_body_step_state==1U){
  MotionProfile profile;uint32_t rpm,run_ms,accepted;
  if(!g_can_guard_ready)return;
  if(!MotionRoute_Build(MOVE_LEFT,10.0f,15.0f,&profile,&rpm,&run_ms)){SBS_Fail(4U);return;}
  /* 取本次新鲜HWT航向为保持目标，不伪造首段路线完成或航向参考。 */
  if(!RouteHeading_Ensure(&s_route_heading_reference,s_heading_source.yaw_deg,1U) ||
     !RouteHeading_Target(&s_route_heading_reference,0.0f,&s_route_heading_target)){SBS_Fail(4U);return;}
  s_motion_list_submit=1U;
  accepted=App_MoveSubmit(MOVE_LEFT,rpm,run_ms,10.0f,15.0f);
  s_motion_list_submit=0U;
  if(!accepted){SBS_Fail(5U);return;}
  g_source_body_step_submitted=1U;g_source_body_step_state=2U;return;
 }
 if(!MotionReadStatus(&status)){SBS_Fail(6U);return;}
 if(status.backend_active && status.distance_mode && !g_source_body_step_session)
  g_source_body_step_session=body.session;
 if(status.state==MOTION_FAULT || status.state==MOTION_CANCELLED || status.backend_cause){SBS_Fail(6U);return;}
 if(status.state==MOTION_DONE){
  if(!g_source_body_step_session || body.session!=g_source_body_step_session ||
     !status.distance_reached || status.distance_direction!=MOVE_LEFT ||
     fabsf(status.distance_target_cm-1.0f)>.001f || status.backend_active ||
     status.backend_result!=1U || status.stop_mask!=15U){SBS_Fail(7U);return;}
  /* 原接口检查同一会话停稳。保持机构电源，不写软件到位/视觉成功位。 */
  if(!MotionFinish()){SBS_Fail(8U);return;}
  g_source_body_step_state=3U;
 }
}
