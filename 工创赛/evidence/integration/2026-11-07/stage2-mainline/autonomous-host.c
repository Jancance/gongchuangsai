
#include "source_stage2_runtime.h"
#include <stdio.h>
#include <string.h>
static unsigned checks,failures;
#define CK(x) do{checks++;if(!(x)){failures++;printf("FAIL %d scenario%u\n",__LINE__,f->scenario);}}while(0)
typedef struct {
 unsigned scenario,now,begin_calls[11],poll_calls[11],physical_starts[11];
 unsigned commits,stop_calls,route_publications,route_started_before_stow;
 unsigned malformed_sent,gate_observed,commit_slots;
} Fake;
static S2_Runtime *active;
static SPF_Config config(void){
 SPF_Config c;memset(&c,0,sizeof(c));c.run_id=10;c.pose_version=7;
 c.live_references_valid=c.deploy_sweep_valid=c.carrier_height_valid=1;
 c.transfer_sweep_valid=c.stow_while_driving_valid=1;c.transaction_timeout_ms=1000;
 c.requested_colors[0]=1;c.requested_colors[1]=4;c.requested_colors[2]=6;
 c.deploy=(SPF_Pose){180,48,0};c.transfer=(SPF_Pose){35.758f,0,7200};
 c.place=c.transfer;c.initial=(SPF_Pose){0,0,0};c.gripper_open_deg=210;c.gripper_close_deg=140;c.initial_gripper_deg=145;return c;
}
static SPF_Target target(unsigned item,unsigned now){
 SPF_Target t;memset(&t,0,sizeof(t));t.run_id=10;t.pose_version=7;t.sequence=item+1;
 t.observed_ms=now;t.usable_until_ms=now+3000;t.color_id=item==0?1:item==1?4:6;
 t.instance_id=100+item;t.stationary=t.sweep_valid=1;t.safe_exit_budget_ms=2400;
 t.pick=(SPF_Pose){item==0?190:item==1?165:200,48+item*10.0f,118800};return t;
}
static uint32_t begin(void*p,const SPF_Command*c,uint32_t now){Fake*f=p;(void)now;
 f->begin_calls[c->kind]++;
 if(f->scenario==2 && c->kind==SPF_CMD_DEPLOY && f->begin_calls[c->kind]<3)return 0;
 if(f->scenario==3 && c->kind==SPF_CMD_PICK)return 2;
 f->physical_starts[c->kind]++;
 if(c->kind==SPF_CMD_ROUTE_AND_STOW){
  CK(active->committed==3 && active->loaded_mask==7);
  CK(c->route_stage==3 && c->axes_mask==(7|SPF_ROUTE|SPF_STOW));
  f->route_publications++;f->route_started_before_stow=1;
 }
 return 1;
}
static uint32_t poll(void*p,const SPF_Command*c,S2_Event*e,uint32_t now){Fake*f=p;unsigned n=++f->poll_calls[c->kind];
 memset(e,0,sizeof(*e));e->run_id=c->run_id;e->sequence=c->sequence;e->observed_ms=now;
 e->completed_mask=c->axes_mask&7;e->instance_id=active->instance_id;
 e->color_id=active->color_id;e->slot=active->slot;
 if(c->kind==SPF_CMD_ALIGN)e->evidence_flags=S2_ALIGNED;
 if(c->kind==SPF_CMD_CLOSE)e->evidence_flags=S2_SOURCE_CLEAR|S2_HELD;
 if(c->kind==SPF_CMD_RELEASE)e->evidence_flags=S2_RELEASED|S2_PRESENT|S2_STABLE;
 if(c->kind==SPF_CMD_ROUTE_AND_STOW){
  e->completed_mask=n==1?SPF_ROUTE:SPF_STOW;
  if(n==1){CK(!S2_Finished(active));CK(f->route_started_before_stow);}
 }
 if(f->scenario==4 && c->kind==SPF_CMD_TRANSFER)return 2;
 if(f->scenario==5 && c->kind==SPF_CMD_CLOSE){e->evidence_flags=0;f->gate_observed=active->flow.placed==0;}
 if(f->scenario==6 && c->kind==SPF_CMD_RELEASE){e->evidence_flags=S2_RELEASED|S2_PRESENT;f->gate_observed=active->committed==0;}
 if(f->scenario==7 && c->kind==SPF_CMD_ALIGN){e->evidence_flags=0;f->gate_observed=active->flow.state==SPF_ALIGN;}
 if(f->scenario==8 && c->kind==SPF_CMD_RELEASE){e->instance_id++;}
 if(f->scenario==9 && c->kind==SPF_CMD_CLOSE){e->observed_ms=active->command_ms-1;}
 if(f->scenario==10 && c->kind==SPF_CMD_DEPLOY){e->run_id++;}
 if(f->scenario==11 && c->kind==SPF_CMD_PICK){e->sequence++;}
 if(f->scenario==12 && c->kind==SPF_CMD_DEPLOY && !f->malformed_sent){
  e->sequence++;f->malformed_sent=1;
 }
 if(f->scenario==13 && c->kind==SPF_CMD_PICK){e->completed_mask=(n%3)==1?1:(n%3)==2?2:4;}
 return 1;
}
static uint32_t stop(void*p,uint32_t mask,uint32_t now){Fake*f=p;(void)now;f->stop_calls++;
 return f->stop_calls<3?0:mask;
}
static uint32_t grasp(void*p,uint32_t run,uint32_t sequence,const S2_Event*e,uint32_t now){
 Fake*f=p;(void)now;CK(run==10 && sequence==e->slot && e->instance_id==99+e->slot);
 return 1;
}
static uint32_t commit(void*p,uint32_t run,uint32_t sequence,const S2_Event*e,uint32_t now){Fake*f=p;
 unsigned slot=e->slot;(void)now;
 CK(run==10 && sequence==slot && e->instance_id==100+slot-1 && e->color_id==(slot==1?1:slot==2?4:6));
 if(f->scenario==14)return 0;
 CK(!(f->commit_slots&(1U<<(slot-1))));f->commit_slots|=1U<<(slot-1);f->commits++;return 1;
}
int main(void){unsigned scenario;Fake fixture,*f=&fixture;S2_Runtime r;SPF_Config cfg;S2_IO io;SPF_Target t;
 for(scenario=0;scenario<15;scenario++){
  memset(f,0,sizeof(*f));memset(&r,0,sizeof(r));f->scenario=scenario;cfg=config();active=&r;
  io=(S2_IO){f,begin,poll,stop,grasp,commit};CK(S2_Begin(&r,&cfg,&io,0));
  for(f->now=10;f->now<16000;f->now+=10){
   if(r.flow.state==SPF_WAIT_TARGET){
    t=target(r.flow.item,f->now);
    if(scenario==1)t.stationary=0;
    if(scenario==1){CK(!S2_TargetReady(&r,&t,r.flow.item+1,f->now));}
    else{CK(S2_TargetReady(&r,&t,r.flow.item+1,f->now));}
   }
   S2_Service(&r,f->now);
   if(r.flow.state==SPF_FAULT && f->stop_calls==1){
    CK(r.required_stop_mask!=0 && !S2_Begin(&r,&cfg,&io,f->now));
   }
   if(S2_Finished(&r) || (r.flow.state==SPF_FAULT && !r.flow.stop_pending && !r.required_stop_mask))break;
  }
  if(scenario==0||scenario==2||scenario==12||scenario==13){
   CK(S2_Finished(&r) && f->commits==3 && f->commit_slots==7 && r.flow.placed==3);
   CK(f->route_publications==1 && f->physical_starts[SPF_CMD_ROUTE_AND_STOW]==1);
   CK(f->physical_starts[SPF_CMD_PICK]==3 && f->physical_starts[SPF_CMD_RELEASE]==3);
   S2_Service(&r,f->now+10);CK(f->commits==3 && f->route_publications==1);
   if(scenario==12)CK(r.rejected_events>=1);
  }else{
   CK(r.flow.state==SPF_FAULT && !S2_Finished(&r));
   CK(f->commits==0 && !f->route_publications && f->stop_calls==3 && !r.required_stop_mask);
   CK(!S2_Begin(&r,&cfg,&io,f->now));
   if(scenario==5||scenario==6||scenario==7)CK(f->gate_observed);
  }
 }
 memset(f,0,sizeof(*f));memset(&r,0,sizeof(r));cfg=config();active=&r;io=(S2_IO){f,begin,poll,stop,grasp,commit};
 cfg.requested_colors[1]=1;CK(!S2_Begin(&r,&cfg,&io,0));cfg=config();io.poll=0;CK(!S2_Begin(&r,&cfg,&io,0));
 printf("stage2 autonomous scenarios=15 checks=%u failures=%u\n",checks,failures);return failures?1:0;
}
