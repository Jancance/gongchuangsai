#include "source_stage2_runtime.h"
#include <string.h>

static uint32_t required(const SPF_Command *c)
{
 if(c->kind==SPF_CMD_ROUTE_AND_STOW)return SPF_ROUTE|SPF_STOW;
 return c->axes_mask&7U;
}

uint32_t S2_Begin(S2_Runtime *r,const SPF_Config *cfg,const S2_IO *io,uint32_t now)
{
 uint32_t i,j;
 if(!r || !cfg || !io || !io->begin || !io->poll || !io->stop || !io->observe_grasp || !io->commit_loaded ||
    (r->flow.state!=SPF_IDLE && r->flow.state!=SPF_DONE) || r->required_stop_mask)return 0U;
 for(i=0U;i<3U;i++){
  if(cfg->requested_colors[i]<1U || cfg->requested_colors[i]>6U)return 0U;
  for(j=0U;j<i;j++)if(cfg->requested_colors[i]==cfg->requested_colors[j])return 0U;
 }
 /* Validate before replacing caller state. */
 {SPF_Flow check;memset(&check,0,sizeof(check));if(!SPF_Begin(&check,cfg,now))return 0U;}
 memset(r,0,sizeof(*r));r->io=*io;
 if(!SPF_Begin(&r->flow,cfg,now))return 0U;
 r->configuration_loaded=1U;return 1U;
}

void S2_Cancel(S2_Runtime *r,uint32_t reason)
{
 if(!r || !r->configuration_loaded)return;
 SPF_Cancel(&r->flow,reason);
 /* Include every resource a preparation/uncertain start could own. */
 r->required_stop_mask|=7U|SPF_ROUTE|SPF_STOW;
}

uint32_t S2_TargetReady(S2_Runtime *r,const SPF_Target *target,uint32_t slot,uint32_t now)
{
 if(!r || !target || !r->configuration_loaded || r->required_stop_mask ||
    slot!=r->flow.item+1U || slot>3U || r->have_command)return 0U;
 if(!SPF_TargetReady(&r->flow,target,now))return 0U;
 r->target_sequence=target->sequence;r->instance_id=target->instance_id;
 r->color_id=target->color_id;r->slot=slot;
 r->grasp_confirmed=0U;return 1U;
}

static uint32_t evidence(const S2_Runtime *r,const S2_Event *event,uint32_t now)
{
 uint32_t flags=0U;
 if(r->active.kind==SPF_CMD_ALIGN)flags=S2_ALIGNED;
 if(r->active.kind==SPF_CMD_CLOSE)flags=S2_SOURCE_CLEAR|S2_HELD;
 if(r->active.kind==SPF_CMD_RELEASE)flags=S2_RELEASED|S2_PRESENT|S2_STABLE;
 if(!flags)return 1U;
 if((event->evidence_flags&flags)!=flags || (int32_t)(event->observed_ms-r->command_ms)<0 ||
    (int32_t)(now-event->observed_ms)<0 || now-event->observed_ms>1000U)return 0U;
 if(r->active.kind==SPF_CMD_CLOSE || r->active.kind==SPF_CMD_RELEASE){
  if(event->instance_id!=r->instance_id || event->color_id!=r->color_id ||
     event->slot!=r->slot)return 0U;
 }
 return 1U;
}

void S2_Service(S2_Runtime *r,uint32_t now)
{
 uint32_t result,mask;S2_Event event;
 if(!r || !r->configuration_loaded)return;
 SPF_Service(&r->flow,now);
 if(r->flow.state==SPF_FAULT){
  if(r->flow.stop_pending && !r->required_stop_mask)r->required_stop_mask=7U|SPF_ROUTE|SPF_STOW;
  if(r->required_stop_mask){
   mask=r->io.stop(r->io.context,r->required_stop_mask,now)&r->required_stop_mask;
   r->stop_confirmed_mask|=mask;r->required_stop_mask&=~mask;
   if(!r->required_stop_mask)r->flow.stop_pending=0U;
  }
  return;
 }
 if(r->flow.state==SPF_IDLE || r->flow.state==SPF_DONE || r->flow.state==SPF_WAIT_TARGET)return;
 if(!r->have_command){
  if(!SPF_ReadCommand(&r->flow,&r->active,now))return;
  r->have_command=1U;r->started=r->done_mask=0U;r->command_ms=now;
 }
 if(!r->started){
  result=r->io.begin(r->io.context,&r->active,now);
  if(result==2U){S2_Cancel(r,4U);return;}
  if(result!=1U)return;
  r->started=1U;return;
 }
 memset(&event,0,sizeof(event));result=r->io.poll(r->io.context,&r->active,&event,now);
 if(result==2U){S2_Cancel(r,5U);return;}
 if(result!=1U)return;
 if(event.run_id!=r->active.run_id || event.sequence!=r->active.sequence ||
    (event.completed_mask&~(7U|SPF_ROUTE|SPF_STOW))){r->rejected_events++;return;}
 r->done_mask|=event.completed_mask;
 mask=required(&r->active);
 if((r->done_mask&mask)!=mask || !evidence(r,&event,now)){r->rejected_events++;return;}
 if(r->active.kind==SPF_CMD_CLOSE){
  if(!r->io.observe_grasp(r->io.context,r->active.run_id,r->target_sequence,&event,now)){
   S2_Cancel(r,6U);return;
  }
  r->grasp_confirmed=1U;
 }
 if(r->active.kind==SPF_CMD_RELEASE){
  if(!r->grasp_confirmed){S2_Cancel(r,6U);return;}
  if(!r->io.commit_loaded(r->io.context,r->active.run_id,r->target_sequence,&event,now)){
   S2_Cancel(r,7U);return;
  }
  r->loaded_mask|=1U<<(r->slot-1U);r->committed++;
 }
 if(!SPF_Report(&r->flow,r->active.sequence,1U,r->done_mask,now)){S2_Cancel(r,8U);return;}
 r->have_command=r->started=r->done_mask=0U;
}

uint32_t S2_Finished(const S2_Runtime *r)
{
 return r && r->configuration_loaded && r->flow.state==SPF_DONE &&
  r->loaded_mask==7U && r->committed==3U && !r->required_stop_mask && !r->flow.fault;
}
