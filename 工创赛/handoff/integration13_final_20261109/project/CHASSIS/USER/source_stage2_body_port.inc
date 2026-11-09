/* Task A owns route preparation/publication. CAN task only submits/polls. */
#include "source_stage2_body_port.h"
#include "source_observer.h"
#include <stdlib.h>
enum { S2BP_IDLE,S2BP_QUEUED,S2BP_OBSERVE,S2BP_MOVE,S2BP_ROUTE,
       S2BP_DONE,S2BP_FAULT,S2BP_STOPPING,S2BP_STOPPED };
enum { S2BP_OK,S2BP_UNCONFIGURED,S2BP_CONTEXT,S2BP_OBSERVATION,
       S2BP_SMALL_STEP,S2BP_RANGE,S2BP_QUEUE,S2BP_SESSION,S2BP_BACKEND,
       S2BP_TIMEOUT,S2BP_CANCELLED };
static struct {
 MetricAlignmentProfile profile;S2_BodyLimits limits;SPF_Command command;
 S2_Event event;volatile uint32_t configured,state,reason,stop;
 uint32_t submitted_ms;
 uint32_t started,session,session_segment,move_begin_ms,corrections;
} s2bp;
volatile uint32_t g_stage2_body_state,g_stage2_body_reason,g_stage2_body_session;

static void S2BP_Publish(void)
{g_stage2_body_state=s2bp.state;g_stage2_body_reason=s2bp.reason;g_stage2_body_session=s2bp.session;}
uint32_t Stage2Body_Owns(void)
{
 uint32_t owns;taskENTER_CRITICAL();
 owns=s2bp.configured && s2bp.state>=S2BP_QUEUED && s2bp.state<=S2BP_STOPPING &&
  s2bp.state!=S2BP_FAULT && (s2bp.state!=S2BP_DONE ||
   (s2bp.command.kind==SPF_CMD_ROUTE_AND_STOW && !g_source_stage2_completed));
 taskEXIT_CRITICAL();return owns;
}
uint32_t Stage2Body_KeepsPower(void)
{return Stage2Body_Owns() && StartupAxes_Holding() && !g_safety_fault && !g_rtos_fault && !g_can_ctrl_fault_latched;}
uint32_t Stage2Body_PositionEnabled(void){return s2bp.configured;}
uint32_t Stage2Body_ObservationWindow(void)
{return s2bp.configured && s2bp.state==S2BP_OBSERVE && !s2bp.stop;}

uint32_t Stage2Body_Configure(const MetricAlignmentProfile *profile,const S2_BodyLimits *limits)
{
 static const volatile uint32_t isolated_profile=VEHICLE_LIFT_RULER_ONLY,offline_profile=APP_MOTOR_OFFLINE_ONLY;
 if(isolated_profile || offline_profile)return 0U;
 if(!profile || !limits || Stage2Body_Owns() || SourceStage2_IsActive() || !MetricAlignment_ProfileValid(profile) ||
    limits->body_frame_verified!=1U || limits->route3_profile_verified!=1U ||
    !limits->tolerance_milli_mm || limits->tolerance_milli_mm>20000U ||
    !limits->yaw_tolerance_mdeg || limits->yaw_tolerance_mdeg>(uint32_t)profile->max_yaw_mdeg ||
    !limits->max_corrections || limits->max_corrections>3U ||
    !(limits->correction_speed_mm_s>=12.1f && limits->correction_speed_mm_s<=100.0f))return 0U;
 memset(&s2bp,0,sizeof(s2bp));s2bp.profile=*profile;s2bp.limits=*limits;
 s2bp.configured=1U;SourceObserver_Init(profile);S2BP_Publish();return 1U;
}

static uint32_t S2BP_Submit(const SPF_Command *command,uint32_t now)
{
 uint32_t accepted=0U;
 if(!command)return 2U;
 taskENTER_CRITICAL();
 if(!s2bp.configured){taskEXIT_CRITICAL();return 2U;}
 if(s2bp.command.run_id==command->run_id && s2bp.command.sequence==command->sequence &&
    s2bp.command.kind==command->kind && s2bp.state!=S2BP_IDLE){
  accepted=s2bp.state==S2BP_FAULT || s2bp.state==S2BP_STOPPED?2U:1U;
 }else if(s2bp.state==S2BP_IDLE || s2bp.state==S2BP_DONE){
  s2bp.command=*command;s2bp.submitted_ms=now;s2bp.stop=0U;
  s2bp.started=s2bp.session=s2bp.session_segment=s2bp.corrections=0U;
  s2bp.state=S2BP_QUEUED;s2bp.reason=0U;memset(&s2bp.event,0,sizeof(s2bp.event));accepted=1U;
 }
 taskEXIT_CRITICAL();return accepted;
}
static uint32_t S2BP_AlignBegin(void *unused,const SPF_Command *command,uint32_t now)
{(void)unused;return command && command->kind==SPF_CMD_ALIGN?S2BP_Submit(command,now):2U;}
static uint32_t S2BP_RouteBegin(void *unused,const SPF_Command *command,uint32_t now)
{(void)unused;return command && command->kind==SPF_CMD_ROUTE_AND_STOW && command->route_stage==3U?S2BP_Submit(command,now):2U;}
static uint32_t S2BP_Poll(const SPF_Command *command,S2_Event *event)
{
 uint32_t result=0U;taskENTER_CRITICAL();
 if(!command || command->run_id!=s2bp.command.run_id || command->sequence!=s2bp.command.sequence || command->kind!=s2bp.command.kind)result=2U;
 else if(s2bp.state==S2BP_DONE){if(event)*event=s2bp.event;result=1U;}
 else if(s2bp.reason || s2bp.state==S2BP_FAULT || s2bp.state==S2BP_STOPPED)result=2U;
 taskEXIT_CRITICAL();return result;
}
static uint32_t S2BP_AlignPoll(void *unused,const SPF_Command *command,S2_Event *event,uint32_t now)
{(void)unused;(void)now;return S2BP_Poll(command,event);}
static uint32_t S2BP_RoutePoll(void *unused,const SPF_Command *command,uint32_t now)
{(void)unused;(void)now;return S2BP_Poll(command,0);}
void Stage2Body_Cancel(void)
{taskENTER_CRITICAL();if(s2bp.configured && s2bp.state!=S2BP_IDLE && s2bp.state!=S2BP_STOPPED)s2bp.stop=1U;taskEXIT_CRITICAL();}
static uint32_t S2BP_Stop(void *unused,uint32_t now)
{
 uint32_t done;(void)unused;(void)now;
 taskENTER_CRITICAL();done=!s2bp.configured || s2bp.state==S2BP_IDLE || s2bp.state==S2BP_STOPPED;
 if(!done)s2bp.stop=1U;taskEXIT_CRITICAL();return done;
}
uint32_t Stage2Body_GetHooks(S2_BackendHooks *out,void *context,uint32_t (*path)(void *,const SPF_Command *))
{
 if(!out || !path || !s2bp.configured)return 0U;
 out->context=context;out->alignment_begin=S2BP_AlignBegin;out->alignment_poll=S2BP_AlignPoll;
 out->route_begin=S2BP_RouteBegin;out->route_poll=S2BP_RoutePoll;out->route_stop=S2BP_Stop;
 out->path_verified=path;return 1U;
}

static void S2BP_Fail(uint32_t reason)
{if(!s2bp.reason)s2bp.reason=reason;SourceObserver_Invalidate();
 if(s2bp.started)s2bp.stop=1U;s2bp.state=S2BP_FAULT;S2BP_Publish();}
static uint32_t S2BP_Context(void)
{
 return s2bp.command.run_id==g_vehicle_run_id && g_vehicle_run_id &&
  (g_vehicle_stage==2U || (s2bp.command.kind==SPF_CMD_ROUTE_AND_STOW && g_vehicle_stage==3U)) &&
  !g_vehicle_cause && !g_safety_fault && !g_rtos_fault && !g_can_ctrl_fault_latched;
}
static uint32_t S2BP_QueueCorrection(const MetricAlignmentObservation *observation,uint32_t now)
{
 float dx=(float)observation->dx_milli_mm/1000.0f,dy=(float)observation->dy_milli_mm/1000.0f;
 float amount;uint32_t direction;RouteRow row;FieldPoint end=s_vehicle_handoff;
 if(fabsf(dx)>=fabsf(dy)){direction=dx<0?1U:2U;amount=fabsf(dx);end.x_mm+=dx;}
 else{direction=dy<0?3U:0U;amount=fabsf(dy);end.y_mm+=dy;}
 if(amount<VEHICLE_MIN_ROUTE_CM*10.0f){S2BP_Fail(S2BP_SMALL_STEP);return 0U;}
 if(amount>20.0f || !s_vehicle_handoff_valid ||
    !FieldMap_AxisClear(s_vehicle_handoff,end,FIELD_BODY_HALF_MM)){S2BP_Fail(S2BP_RANGE);return 0U;}
 row=(RouteRow){MOTION_FORWARD_DISTANCE+direction,amount/10.0f,s2bp.limits.correction_speed_mm_s/10.0f};
 if(!VehicleMission_Queue(2U,&row,1U)){S2BP_Fail(S2BP_QUEUE);return 0U;}
 /* This is a correction transaction, not another source-arrival/deploy stage. */
 g_vehicle_state=8U;s2bp.started=1U;s2bp.state=S2BP_MOVE;s2bp.session=0U;
 s2bp.move_begin_ms=now;s2bp.corrections++;SourceObserver_Invalidate();return 1U;
}

uint32_t Stage2Body_Service(uint32_t now)
{
 ChassisAutoStatus backend;MetricAlignmentObservation observation;uint32_t colors[3],session;
 if(!s2bp.configured || s2bp.state==S2BP_IDLE || s2bp.state==S2BP_STOPPED)return 0U;
 (void)ChassisAuto_ReadStatus(&backend);
 if(s2bp.stop){
  if(s2bp.state!=S2BP_STOPPING){
   s_vehicle_queue_pending=0U;s_serial_start_permit=0U;MotionStop();
   s2bp.state=S2BP_STOPPING;SourceObserver_Invalidate();
  }
  if(!backend.active && ((!s2bp.started && !s_motion_session_open) || backend.rotation_stop_mask==15U)){
   s2bp.state=S2BP_STOPPED;if(!s2bp.reason)s2bp.reason=S2BP_CANCELLED;
  }
  S2BP_Publish();return 1U;
 }
 if(s2bp.state==S2BP_FAULT){S2BP_Publish();return 1U;}
 if(!S2BP_Context()){S2BP_Fail(S2BP_CONTEXT);return 1U;}
 if(s2bp.state==S2BP_DONE){S2BP_Publish();return s2bp.command.kind==SPF_CMD_ROUTE_AND_STOW && !g_source_stage2_completed;}
 if(now-s2bp.submitted_ms>=VEHICLE_STAGE_TIMEOUT_MS){S2BP_Fail(S2BP_TIMEOUT);return 1U;}
 if(s2bp.state==S2BP_QUEUED){
  if(g_vehicle_stage!=2U || g_vehicle_state!=8U || !VehicleMission_Stopped()){
   S2BP_Fail(S2BP_CONTEXT);return 1U;
  }
  if(s2bp.command.kind==SPF_CMD_ALIGN){
   g_vehicle_alignment_state=1U;SourceObserver_Invalidate();s2bp.state=S2BP_OBSERVE;
  }else{
   taskENTER_CRITICAL();session=Vision_ReadLoadedBatch(g_vehicle_run_id,1U,colors);taskEXIT_CRITICAL();
   if(!session || !VehicleMission_Load(3U)){S2BP_Fail(S2BP_QUEUE);return 1U;}
   g_vehicle_stage_mask|=1UL<<1;s2bp.started=1U;s2bp.state=S2BP_ROUTE;s2bp.session=0U;
   s2bp.move_begin_ms=now;
  }
 }
 if(s_vehicle_queue_pending){
  g_vehicle_queue_wait_ms=now-s_vehicle_queue_begin;
  if(g_vehicle_queue_wait_ms>=VEHICLE_QUEUE_ADMISSION_TIMEOUT_MS){S2BP_Fail(S2BP_QUEUE);return 1U;}
  g_vehicle_queue_attempts++;g_vehicle_queue_last_mask=App_SerialIdleBlocked(now);
  if(g_vehicle_queue_last_mask&~(32U|256U)){S2BP_Fail(S2BP_QUEUE);return 1U;}
  if(!g_vehicle_queue_last_mask)(void)VehicleMission_PublishQueued(s_vehicle_queue_pending);
  if(s2bp.state==S2BP_MOVE)g_vehicle_state=8U;
  S2BP_Publish();return 1U;
 }
 if(s2bp.state==S2BP_MOVE || s2bp.state==S2BP_ROUTE){
  if(g_combined_test.state>=4U || backend.cause){S2BP_Fail(S2BP_BACKEND);return 1U;}
  if(g_combined_test.state==1U && backend.active){
   if(!s2bp.session || s2bp.session_segment!=g_combined_test.index){
    s2bp.session=backend.session;s2bp.session_segment=g_combined_test.index;
   }else if(backend.session!=s2bp.session){S2BP_Fail(S2BP_SESSION);return 1U;}
  }
  if(!VehicleMission_Stopped()){S2BP_Publish();return 1U;}
  if(!s2bp.session || backend.session!=s2bp.session){S2BP_Fail(S2BP_SESSION);return 1U;}
  if(s2bp.state==S2BP_MOVE){
   if(!VehicleMission_CaptureHandoff()){S2BP_Fail(S2BP_BACKEND);return 1U;}
   g_vehicle_state=8U;s2bp.state=S2BP_OBSERVE;SourceObserver_Invalidate();
  }else{
   taskENTER_CRITICAL();
   memset(&s2bp.event,0,sizeof(s2bp.event));s2bp.event.run_id=s2bp.command.run_id;
   s2bp.event.sequence=s2bp.command.sequence;s2bp.event.completed_mask=SPF_ROUTE;
   s2bp.event.observed_ms=now;s2bp.state=S2BP_DONE;taskEXIT_CRITICAL();
  }
 }
 if(s2bp.state==S2BP_OBSERVE){
  SourceObserver_Service(now);
  if(SourceObserver_Read(now,&observation)==MA_OK){
   if(abs(observation.yaw_mdeg)>(int32_t)s2bp.limits.yaw_tolerance_mdeg){S2BP_Fail(S2BP_RANGE);return 1U;}
   if(abs(observation.dx_milli_mm)<=(int32_t)s2bp.limits.tolerance_milli_mm &&
      abs(observation.dy_milli_mm)<=(int32_t)s2bp.limits.tolerance_milli_mm){
    taskENTER_CRITICAL();
    memset(&s2bp.event,0,sizeof(s2bp.event));s2bp.event.run_id=s2bp.command.run_id;
    s2bp.event.sequence=s2bp.command.sequence;s2bp.event.evidence_flags=S2_ALIGNED;
    s2bp.event.observed_ms=now;s2bp.state=S2BP_DONE;taskEXIT_CRITICAL();
   }else if(s2bp.corrections>=s2bp.limits.max_corrections)S2BP_Fail(S2BP_RANGE);
   else (void)S2BP_QueueCorrection(&observation,now);
  }
 }
 S2BP_Publish();return 1U;
}
#include "stage2_site_activate.inc"
