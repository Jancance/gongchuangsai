/* Sole CAN owner; no TaskA/ISR writes into the motion planner. */
static SourceStationTarget ss_target,ss_selected_target;
static uint32_t ss_target_valid,ss_used_instances[3];
uint32_t SourceStation_OfferTarget(const SourceStationTarget *target,uint32_t now)
{
 if(!target || !SourceStation_PickAllowed(target->run) ||
  (g_it_auto_state==1U && g_it_auto_phase!=7U) ||
  target->pose_version!=SOURCE_STATION_POSE_VERSION || !target->pose_version ||
  !target->material_cycle || !target->instance_id || target->capture_verified!=1U ||
  target->stationary!=1U || target->sweep_verified!=1U ||
  (int32_t)(now-target->captured_ms)<0 || now-target->captured_ms>SOURCE_STATION_FRAME_MAX_AGE_MS ||
  !target->exit_budget_ms || (int32_t)(target->usable_until_ms-now)<=0 ||
  target->usable_until_ms-now<target->exit_budget_ms)return 0U;
 ss_target=*target;ss_target_valid=1U;return 1U;
}
static uint32_t it_auto_take_vision(uint32_t now)
{
 uint32_t i,region;
 static const float yaw[3]={195.0f,165.0f,180.0f};
 static const float ext[3]={156.0f,160.0f,0.0f};
 if(!SourceStation_PickAllowed(g_vis_task_run_id) || g_vis_task_item!=g_it_auto_item ||
  g_vis_pending_seq || !ss_target_valid || ss_target.run!=g_vis_task_run_id ||
  ss_target.material_cycle!=g_vis_mat_cycle || now-ss_target.captured_ms>SOURCE_STATION_FRAME_MAX_AGE_MS ||
  (int32_t)(ss_target.usable_until_ms-now)<=0 ||
  ss_target.usable_until_ms-now<ss_target.exit_budget_ms)return 0U;
 for(i=0U;i<g_it_auto_item;i++)if(ss_used_instances[i]==ss_target.instance_id)return 0U;
 /* Region lookup is the SAME taught mechanics; never bind color to direction. */
 g_vis_taught_mask=7U;
 for(i=0U;i<3U;i++){g_vis_region_deg[i]=yaw[i];g_vis_region_ext_mm[i]=ext[i];}
 if(!Vision_TakeTaskTarget(now))return 0U;
 region=g_vis_pick_region;
 if(region>2U){(void)Vision_RejectUnstarted(g_vis_task_run_id,g_vis_pending_seq);return 0U;}
 ss_used_instances[g_it_auto_item]=ss_target.instance_id;ss_target_valid=0U;
 ss_selected_target=ss_target;
 g_it_auto_region=region;g_it_dm_deg=yaw[region];g_it_extension_mm=0.0f;g_it_axes=SPF_DM;
 g_it_carrier_auto_deg=(float)g_it_auto_item*120.0f;
 it_auto_sequence=g_it_sequence;g_it_request=3U;g_it_auto_phase=1U;it_auto_phase_begin=now;
 return 1U;
}
static uint32_t it_auto_target_usable(uint32_t now)
{
 return ss_selected_target.run==g_vis_task_run_id &&
  (int32_t)(ss_selected_target.usable_until_ms-now)>0 &&
  ss_selected_target.usable_until_ms-now>=ss_selected_target.exit_budget_ms;
}
static void SourceStation_VisionService(uint32_t now)
{
 static uint32_t started_run;
 if(g_it_auto_vision_mode && g_it_auto_state==1U && !SourceStation_PickAllowed(g_vis_task_run_id)){
  g_it_auto_state=9U;g_it_auto_fault=14U;g_it_request=0U;it_fail(14U);return;
 }
 if(!SOURCE_STATION_ENABLE || !SourceStation_PickAllowed(g_vis_task_run_id) ||
  started_run==g_vis_task_run_id || !g_vis_task_valid || g_vis_task_item ||
  g_it_auto_state==1U || g_it_auto_state==9U)return;
 g_it_auto_vision_mode=1U;ss_target_valid=0U;memset(ss_used_instances,0,sizeof(ss_used_instances));
 if(it_auto_start(now))started_run=g_vis_task_run_id;
 else g_it_auto_vision_mode=0U;
}
