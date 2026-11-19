/* Included after integration10_complete.inc by vehicle_arm.c. No new task. */
#include "source_stage2_backend.h"
#include "source_stage2_owner.h"
#include "source_stage2_vision_bridge.h"
typedef struct {
 S2_BackendHooks hooks;S2_ServoTiming timing;SPF_Config config;SPF_Command active;
 S2_Event observed;
 uint32_t configured,have_active,begin_ms,mechanical,route,have_observation;
 uint32_t target_instance,target_color,target_slot;
} S2B_Context;
static S2B_Context s2b;

static uint32_t S2B_MoveKind(uint32_t kind)
{
 return kind==SPF_CMD_DEPLOY || kind==SPF_CMD_PICK || kind==SPF_CMD_TRANSFER ||
  kind==SPF_CMD_PLACE || kind==SPF_CMD_NEXT || kind==SPF_CMD_ROUTE_AND_STOW;
}

static uint32_t S2B_Begin(void *unused,const SPF_Command *c,uint32_t now)
{
 SPF_Command arm;uint32_t pulse;(void)unused;
 if(!c || !s2b.configured || c->run_id!=s2b.config.run_id ||
    c->pose_version!=s2b.config.pose_version || !it_platform())return 2U;
 if(s2b.have_active && c->sequence==s2b.active.sequence)return 1U;
 if(S2B_MoveKind(c->kind)){
  if(!s2b.hooks.path_verified(s2b.hooks.context,c))return 2U;
  if(VehicleArm_Busy() || VehicleSourceParallel_Owns())return 0U;
 }
 s2b.active=*c;s2b.begin_ms=now;s2b.have_active=1U;
 s2b.have_observation=s2b.mechanical=s2b.route=0U;
 if(c->kind==SPF_CMD_ALIGN){
  uint32_t result=s2b.hooks.alignment_begin(s2b.hooks.context,c,now);
  if(!result)s2b.have_active=0U;
  return result;
 }
 if(c->kind==SPF_CMD_CLOSE || c->kind==SPF_CMD_RELEASE){
  return jiazhua(c->gripper_deg)==1U?1U:2U;
 }
 if(!S2B_MoveKind(c->kind))return 2U;
 arm=*c;arm.axes_mask&=7U;
 /* The known real parallel backend controls only the three mechanism axes. */
 if(!VehicleArm_BeginSourceParallel(&arm,1U,c->kind==SPF_CMD_ROUTE_AND_STOW?1U:0U))return 2U;
 s2b.mechanical=1U;
 if(c->kind==SPF_CMD_DEPLOY && jiazhua(c->gripper_deg)!=1U)return 2U;
 if(c->kind==SPF_CMD_PICK){
  /* First item slot0deg, then120/240. This output is not tray feedback. */
  if(c->item>=3U)return 2U;
  pulse=500U+(uint32_t)((float)c->item*120.0f*2000.0f/270.0f+0.5f);
  if(!CarrierPwm_Write(pulse))return 2U;
 }
 if(c->kind==SPF_CMD_ROUTE_AND_STOW){
  /* One publication, no waiting for arm arrival before route submission. */
  if(s2b.hooks.route_begin(s2b.hooks.context,c,now)!=1U)return 2U;
  s2b.route=1U;
 }
 return 1U;
}

static uint32_t S2B_Poll(void *unused,const SPF_Command *c,S2_Event *event,uint32_t now)
{
 uint32_t route_result;(void)unused;
 if(!c || !event || !s2b.have_active || c->run_id!=s2b.active.run_id ||
    c->sequence!=s2b.active.sequence || !it_platform())return 2U;
 if(c->kind==SPF_CMD_ALIGN)return s2b.hooks.alignment_poll(s2b.hooks.context,c,event,now);
 memset(event,0,sizeof(*event));event->run_id=c->run_id;event->sequence=c->sequence;
 event->observed_ms=now;
 if(s2b.mechanical){
  if(g_source_parallel_cause || g_vehicle_arm_cause || g_source_parallel_state==4U)return 2U;
  if(g_source_parallel_state==3U && !VehicleSourceParallel_Owns()){
   event->completed_mask=c->kind==SPF_CMD_ROUTE_AND_STOW?SPF_STOW:(c->axes_mask&7U);
  }
 }
 if(c->kind==SPF_CMD_ROUTE_AND_STOW){
  route_result=s2b.hooks.route_poll(s2b.hooks.context,c,now);
  if(route_result==2U)return 2U;
  if(route_result==1U)event->completed_mask|=SPF_ROUTE;
  return 1U;
 }
 if(c->kind==SPF_CMD_PICK && now-s2b.begin_ms<s2b.timing.carrier_settle_ms)return 0U;
 if(c->kind==SPF_CMD_CLOSE || c->kind==SPF_CMD_RELEASE){
  if(now-s2b.begin_ms<s2b.timing.gripper_settle_ms || !s2b.have_observation)return 0U;
  *event=s2b.observed;return 1U;
 }
 return 1U;
}

static uint32_t S2B_Stop(void *unused,uint32_t mask,uint32_t now)
{
 uint32_t done=0U;(void)unused;
 if(VehicleSourceParallel_Owns())VehicleSourceParallel_Cancel();
 if(!VehicleSourceParallel_Owns() && !g_source_parallel_stop_mask &&
    !g_vehicle_arm_stop_pending && !g_lift_ruler_stop_pending)done|=mask&(7U|SPF_STOW);
 if(mask&SPF_ROUTE){
  if(s2b.hooks.route_stop(s2b.hooks.context,now)==1U)done|=SPF_ROUTE;
 }
 /* Preserve VM and gripper state, particularly while a material may be held. */
 return done;
}

uint32_t SourceStage2Backend_Start(const SPF_Config *cfg,const S2_BackendHooks *hooks,
 const S2_ServoTiming *timing,uint32_t now)
{
 S2_IO io={0,S2B_Begin,S2B_Poll,S2B_Stop,S2_VisionGrasp,S2_VisionPlaced};uint32_t i;
 if(!cfg || !hooks || !timing || SourceStage2_IsActive() ||
    !hooks->alignment_begin || !hooks->alignment_poll || !hooks->route_begin ||
    !hooks->route_poll || !hooks->route_stop || !hooks->path_verified ||
    !timing->carrier_settle_ms || timing->carrier_settle_ms>2000U ||
    !timing->gripper_settle_ms || timing->gripper_settle_ms>2000U ||
    !it_platform() || !SingleTeach_ReferencesReady() || g_it_request || g_it_state==1U ||
    !g_vis_task_valid || g_vis_task_run_id!=cfg->run_id || g_vis_task_item ||
    g_vis_pending_seq || VehicleArm_Busy() || VehicleSourceParallel_Owns())return 0U;
 for(i=0U;i<3U;i++)if(cfg->requested_colors[i]!=g_vis_task_colors[i])return 0U;
 if(!SourceStage2_Start(cfg,&io,now))return 0U;
 memset(&s2b,0,sizeof(s2b));s2b.config=*cfg;s2b.hooks=*hooks;s2b.timing=*timing;s2b.configured=1U;
 return 1U;
}

uint32_t SourceStage2Backend_Target(const SPF_Target *target,uint32_t slot,uint32_t now)
{
 if(!target || !s2b.configured || g_vis_task_item>=3U || slot<1U || slot>3U || target->run_id!=g_vis_task_run_id ||
    !g_vis_pending_seq || target->sequence!=g_vis_pending_seq || slot!=g_vis_pending_slot ||
    slot!=g_vis_task_item+1U || target->color_id!=g_vis_task_colors[g_vis_task_item] ||
    fabsf(target->pick.dm_deg-g_src_pick_deg)>0.001f ||
    fabsf(target->pick.extension_mm-g_src_extension_mm)>0.001f)return 0U;
 if(!SourceStage2_Target(target,slot,now))return 0U;
 s2b.target_instance=target->instance_id;s2b.target_color=target->color_id;s2b.target_slot=slot;
 return 1U;
}

uint32_t SourceStage2Backend_Observe(const S2_Event *event,uint32_t now)
{
 if(!event || !s2b.have_active || event->run_id!=s2b.active.run_id ||
    event->sequence!=s2b.active.sequence || event->instance_id!=s2b.target_instance ||
    event->color_id!=s2b.target_color || event->slot!=s2b.target_slot ||
    (int32_t)(event->observed_ms-s2b.begin_ms)<0 || (uint32_t)(now-event->observed_ms)>1000U ||
    (s2b.active.kind!=SPF_CMD_CLOSE && s2b.active.kind!=SPF_CMD_RELEASE))return 0U;
 s2b.observed=*event;s2b.have_observation=1U;return 1U;
}

uint32_t SourceStage2Backend_Select(const S2_SourceObservation *observation,uint32_t now)
{
 SPF_Target target;
 if(!observation || !s2b.configured || g_source_stage2_state!=SPF_WAIT_TARGET ||
    observation->run_id!=s2b.config.run_id || observation->pose_version!=s2b.config.pose_version ||
    !observation->instance_id || observation->capture_verified!=1U ||
    observation->stationary!=1U || observation->sweep_valid!=1U ||
    (int32_t)(now-observation->captured_ms)<0 || now-observation->captured_ms>150U ||
    (int32_t)(observation->usable_until_ms-now)<=0 || !observation->safe_exit_budget_ms ||
    observation->usable_until_ms-now<observation->safe_exit_budget_ms)return 0U;
 if(!Vision_TakeTaskTarget(now))return 0U;
 memset(&target,0,sizeof(target));target.run_id=observation->run_id;
 target.pose_version=observation->pose_version;target.sequence=g_vis_pending_seq;
 target.observed_ms=observation->captured_ms;target.usable_until_ms=observation->usable_until_ms;
 target.instance_id=observation->instance_id;target.color_id=g_vis_task_colors[g_vis_task_item];
 target.stationary=observation->stationary;target.sweep_valid=observation->sweep_valid;
 target.safe_exit_budget_ms=observation->safe_exit_budget_ms;
 target.pick.dm_deg=g_src_pick_deg;target.pick.extension_mm=g_src_extension_mm;
 target.pick.lift_pulses=(float)g_lift_compare_target_pulses;
 if(SourceStage2Backend_Target(&target,g_vis_pending_slot,now))return 1U;
 (void)Vision_RejectUnstarted(target.run_id,target.sequence);return 0U;
}

uint32_t SourceStage2Backend_ReadActive(SPF_Command *out)
{
 if(!out || !s2b.have_active)return 0U;*out=s2b.active;return 1U;
}
