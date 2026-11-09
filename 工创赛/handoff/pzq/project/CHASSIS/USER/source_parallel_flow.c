#include "source_parallel_flow.h"
#include <string.h>
static uint32_t pose_valid(const SPF_Pose *p)
{return p && p->dm_deg>=0 && p->dm_deg<=270 && p->extension_mm>=0 &&
 p->extension_mm<=170 && p->lift_pulses>=0 && p->lift_pulses<=SPF_MAX_LIFT_PULSES;}
static uint32_t due(uint32_t now,uint32_t deadline)
{return (int32_t)(now-deadline)>=0;}
static void stage(SPF_Flow *f,uint32_t state,uint32_t now)
{f->state=state;f->issued=0;f->last_progress_ms=now;}
uint32_t SPF_Begin(SPF_Flow *f,const SPF_Config *cfg,uint32_t now)
{
 if(!f || !cfg || (f->state!=SPF_IDLE && f->state!=SPF_DONE) ||
  !cfg->run_id || !cfg->pose_version || cfg->live_references_valid!=1 ||
  cfg->deploy_sweep_valid!=1 || !pose_valid(&cfg->deploy) ||
  cfg->gripper_open_deg<0 || !(cfg->gripper_open_deg<=270) ||
  cfg->transaction_timeout_ms<1000 || cfg->transaction_timeout_ms>15000)return 0;
 memset(f,0,sizeof(*f));f->config=*cfg;stage(f,SPF_DEPLOY,now);return 1;
}
void SPF_Cancel(SPF_Flow *f,uint32_t reason)
{if(f && f->state!=SPF_FAULT){f->fault=reason?reason:1;f->state=SPF_FAULT;f->stop_pending=1;f->issued=0;}}
static void command_base(SPF_Flow *f,SPF_Command *c)
{
 memset(c,0,sizeof(*c));c->run_id=f->config.run_id;c->pose_version=f->config.pose_version;
 c->sequence=++f->sequence;c->item=f->item;c->dm_deg_s=SPF_DM_DEG_S;
 c->extension_rpm=SPF_EXT_RPM;c->lift_rpm=SPF_LIFT_RPM;
 c->stepper_accel_rpm_s=SPF_STEPPER_ACCEL;c->keep_vm_on=1;
}
uint32_t SPF_ReadCommand(SPF_Flow *f,SPF_Command *out,uint32_t now)
{
 SPF_Command c;if(!f || !out || f->issued)return 0;
 if(f->state==SPF_IDLE || f->state==SPF_DONE || f->state==SPF_WAIT_TARGET)return 0;
 if(f->state==SPF_FAULT && !f->stop_pending)return 0;
 command_base(f,&c);
 switch(f->state){
 case SPF_DEPLOY:c.kind=SPF_CMD_DEPLOY;c.axes_mask=SPF_DM|SPF_EXT;
  c.goal=f->config.deploy;c.gripper_deg=f->config.gripper_open_deg;break;
 case SPF_ALIGN:c.kind=SPF_CMD_ALIGN;break; /* A fit and real body correction */
 case SPF_PICK:c.kind=SPF_CMD_PICK;c.axes_mask=SPF_DM|SPF_EXT|SPF_LIFT;c.goal=f->command.goal;break;
 case SPF_CLOSE:c.kind=SPF_CMD_CLOSE;c.gripper_deg=f->config.gripper_close_deg;break;
 case SPF_TRANSFER:c.kind=SPF_CMD_TRANSFER;c.axes_mask=7;c.goal=f->config.transfer;break;
 case SPF_PLACE:c.kind=SPF_CMD_PLACE;c.axes_mask=7;c.goal=f->config.place;break;
 case SPF_RELEASE:c.kind=SPF_CMD_RELEASE;c.gripper_deg=f->config.gripper_open_deg;break;
 case SPF_NEXT:c.kind=SPF_CMD_NEXT;c.axes_mask=7;c.goal=f->config.deploy;break;
 case SPF_ROUTE_AND_STOW:c.kind=SPF_CMD_ROUTE_AND_STOW;c.axes_mask=7|SPF_ROUTE|SPF_STOW;
  c.goal=f->config.initial;c.gripper_deg=f->config.initial_gripper_deg;
  c.route_stage=3;c.return_to_top_after_all_three=1;break;
 default:c.kind=SPF_CMD_STOP;c.axes_mask=7|SPF_ROUTE|SPF_STOW;break;
 }
 f->command=c;f->issued=1;f->last_progress_ms=now;*out=c;return 1;
}
uint32_t SPF_Report(SPF_Flow *f,uint32_t sequence,uint32_t success,uint32_t mask,uint32_t now)
{
 if(!f || !f->issued || sequence!=f->command.sequence)return 0;
 if(!success){SPF_Cancel(f,2);return 1;}
 if(f->state==SPF_FAULT){f->stop_pending=0;return 1;}
 switch(f->state){
 case SPF_DEPLOY:stage(f,SPF_ALIGN,now);break;
 case SPF_ALIGN:stage(f,SPF_WAIT_TARGET,now);break;
 case SPF_PICK:stage(f,SPF_CLOSE,now);break;
 case SPF_CLOSE:stage(f,SPF_TRANSFER,now);break;
 case SPF_TRANSFER:stage(f,SPF_PLACE,now);break;
 case SPF_PLACE:stage(f,SPF_RELEASE,now);break;
 case SPF_RELEASE:
  /* This report means backend-confirmed placed-and-release completion,
   * never mere command ACK. Each item is counted once. */
  f->placed++;if(f->placed==3){f->branch_done_mask=0;stage(f,SPF_ROUTE_AND_STOW,now);}
  else {f->item++;stage(f,SPF_NEXT,now);}break;
 case SPF_NEXT:stage(f,SPF_WAIT_TARGET,now);break;
 case SPF_ROUTE_AND_STOW:
  if(mask&~(SPF_ROUTE|SPF_STOW))return 0;
  f->branch_done_mask|=mask;
  if(f->branch_done_mask==(SPF_ROUTE|SPF_STOW))stage(f,SPF_DONE,now);
  break;
 default:return 0;
 }return 1;
}
uint32_t SPF_TargetReady(SPF_Flow *f,const SPF_Target *t,uint32_t now)
{
 uint32_t i;
 if(!f || !t || f->state!=SPF_WAIT_TARGET || !f->config.carrier_height_valid ||
  !f->config.transfer_sweep_valid || !f->config.stow_while_driving_valid ||
  !pose_valid(&f->config.transfer) || !pose_valid(&f->config.place) || !pose_valid(&f->config.initial) ||
  !(f->config.gripper_close_deg>=0 && f->config.gripper_close_deg<=270) ||
  !(f->config.initial_gripper_deg>=0 && f->config.initial_gripper_deg<=270) ||
  t->run_id!=f->config.run_id || t->pose_version!=f->config.pose_version ||
  t->sequence<=f->last_target_sequence || !t->color_id || t->color_id>6 ||
  f->item>=3 || t->color_id!=f->config.requested_colors[f->item] || !t->instance_id ||
  t->stationary!=1 || t->sweep_valid!=1 || !pose_valid(&t->pick) ||
  (int32_t)(now-t->observed_ms)<0 || now-t->observed_ms>150 ||
  due(now,t->usable_until_ms) || !t->safe_exit_budget_ms ||
  t->usable_until_ms-now<t->safe_exit_budget_ms)return 0;
 for(i=0;i<f->item;i++)if(f->used_instances[i]==t->instance_id)return 0;
 f->used_instances[f->item]=t->instance_id;
 f->last_target_sequence=t->sequence;f->last_color=t->color_id;
 f->command.goal=t->pick;stage(f,SPF_PICK,now);return 1;
}
void SPF_Service(SPF_Flow *f,uint32_t now)
{
 /* Real route has its own existing route timeout; do not kill a healthy
  * stage3 route with the short arm transaction deadline. */
 if(f && f->state!=SPF_IDLE && f->state!=SPF_DONE && f->state!=SPF_FAULT &&
  f->state!=SPF_ROUTE_AND_STOW &&
  now-f->last_progress_ms>=f->config.transaction_timeout_ms)SPF_Cancel(f,3);
}
uint32_t SPF_GroupBegin(SPF_Group *g,const SPF_Command *c,const SPF_GroupIO *io,
 uint32_t verified,uint32_t skew,uint32_t timeout)
{
 if(!g || !c || !io || (g->state && g->state!=3) || verified!=1 ||
  !io->prepare || !io->start || !io->poll || !io->stop || !io->clock_ms ||
  !(c->axes_mask&7) || (c->axes_mask&~7U) || !pose_valid(&c->goal) ||
  c->dm_deg_s!=SPF_DM_DEG_S || c->extension_rpm!=SPF_EXT_RPM ||
  c->lift_rpm!=SPF_LIFT_RPM || c->stepper_accel_rpm_s!=SPF_STEPPER_ACCEL ||
  !c->keep_vm_on || skew<1 || skew>100 || timeout<100 || timeout>15000)return 0;
 memset(g,0,sizeof(*g));g->command=*c;g->io=*io;g->max_start_skew_ms=skew;
 g->timeout_ms=timeout;g->begin_ms=io->clock_ms(io->context);g->state=1;return 1;
}
void SPF_GroupCancel(SPF_Group *g,uint32_t reason)
{
 if(g && g->state!=4){g->state=4;g->fault=reason?reason:1;
  /* Even prepare may enable/queue an axis; cancel all owned axes. */
  g->stop_mask=g->command.axes_mask&7;}
}
void SPF_GroupService(SPF_Group *g)
{
 uint32_t bit,result,now;if(!g || !g->state || g->state==3)return;
 if(g->state==4){
  for(bit=1;bit<=4;bit<<=1)if((g->stop_mask&bit) && g->io.stop(g->io.context,bit))g->stop_mask&=~bit;
  return;
 }
 now=g->io.clock_ms(g->io.context);
 if(now-g->begin_ms>=g->timeout_ms){SPF_GroupCancel(g,3);return;}

 if(g->delayed_mask){
  /* Prepare only released axes: preparing DM can queue a retarget. */
  for(bit=1U;bit<=4U;bit<<=1)if((g->command.axes_mask&bit) && !(g->started&bit) &&
     (g->release_mask&bit)){
   if(!(g->prepared&bit)){
    result=g->io.prepare(g->io.context,bit,&g->command);
    if(result==2U){SPF_GroupCancel(g,4U);return;}
    if(result==1U)g->prepared|=bit;
    break;
   }
   if(!g->started)g->first_start_ms=now;
   g->started|=bit;
   if(g->io.start(g->io.context,bit,&g->command)!=1U){SPF_GroupCancel(g,6U);return;}
   g->last_start_ms=g->io.clock_ms(g->io.context);g->state=2U;break;
  }
  /* Keep servicing moving axes while a later release waits for height. */
  for(bit=1U;bit<=4U;bit<<=1)if((g->started&bit) && !(g->done&bit)){
   result=g->io.poll(g->io.context,bit);
   if(result==2U){SPF_GroupCancel(g,7U);return;}
   if(result==1U)g->done|=bit;
  }
  if(g->done==g->command.axes_mask)g->state=3U;
  return;
 }
 if(g->state==1){
  for(bit=1;bit<=4;bit<<=1)if((g->command.axes_mask&bit) && !(g->prepared&bit)){
   result=g->io.prepare(g->io.context,bit,&g->command);
   if(result==2){SPF_GroupCancel(g,4);return;}
   if(result==1)g->prepared|=bit;return;
  }
  if(g->prepared!=g->command.axes_mask)return;
  /* No completion wait between starts: every prepared axis is released
   * in one owner service. Cross-bus physical simultaneity is not claimed. */
  for(bit=1;bit<=4;bit<<=1)if(g->command.axes_mask&bit){
   now=g->io.clock_ms(g->io.context);if(!g->started)g->first_start_ms=now;
   if(now-g->first_start_ms>g->max_start_skew_ms){SPF_GroupCancel(g,5);return;}
   g->started|=bit; /* uncertain start failure still needs STOP */
   if(g->io.start(g->io.context,bit,&g->command)!=1){SPF_GroupCancel(g,6);return;}
   g->last_start_ms=g->io.clock_ms(g->io.context);
   if(g->last_start_ms-g->first_start_ms>g->max_start_skew_ms){SPF_GroupCancel(g,5);return;}
  }
  g->state=2;return;
 }
 for(bit=1;bit<=4;bit<<=1)if((g->command.axes_mask&bit) && !(g->done&bit)){
  result=g->io.poll(g->io.context,bit);
  if(result==2){SPF_GroupCancel(g,7);return;}
  if(result==1)g->done|=bit;
 }
 if(g->done==g->command.axes_mask)g->state=3;
}
