
#include "source_stage2_owner.h"
#include <stdio.h>
#include <string.h>
static unsigned checks,stops,starts;
#define CK(x) do{checks++;if(!(x)){printf("owner FAIL %d\n",__LINE__);return 1;}}while(0)
static uint32_t begin(void*p,const SPF_Command*c,uint32_t now){(void)p;(void)c;(void)now;starts++;return 1;}
static uint32_t poll(void*p,const SPF_Command*c,S2_Event*e,uint32_t now){
 (void)p;memset(e,0,sizeof(*e));e->run_id=c->run_id;e->sequence=c->sequence;
 e->completed_mask=c->axes_mask&7U;e->evidence_flags=S2_ALIGNED;e->observed_ms=now;return 1;}
static uint32_t stop(void*p,uint32_t mask,uint32_t now){(void)p;(void)now;return ++stops<2?0:mask;}
static uint32_t accept(void*p,uint32_t run,uint32_t seq,const S2_Event*e,uint32_t now){(void)p;(void)run;(void)seq;(void)e;(void)now;return 1;}
int main(void){SPF_Config c;SPF_Target t;S2_IO io;unsigned now;
 SourceStage2_Service(0);CK(!SourceStage2_IsActive() && starts==0 && g_source_stage2_state==SPF_IDLE);
 CK(!SourceStage2_Start(0,0,0));memset(&c,0,sizeof(c));c.run_id=c.pose_version=c.live_references_valid=c.deploy_sweep_valid=1;
 c.carrier_height_valid=c.transfer_sweep_valid=c.stow_while_driving_valid=1;c.transaction_timeout_ms=1000;
 c.requested_colors[0]=1;c.requested_colors[1]=2;c.requested_colors[2]=3;c.gripper_open_deg=210;
 io=(S2_IO){0,begin,poll,stop,accept,accept};CK(SourceStage2_Start(&c,&io,0));CK(SourceStage2_IsActive());
 for(now=10;now<=40;now+=10)SourceStage2_Service(now);
 CK(g_source_stage2_state==SPF_WAIT_TARGET && g_source_stage2_loaded_mask==0 && starts==2);
 memset(&t,0,sizeof(t));t.run_id=t.pose_version=t.sequence=t.instance_id=t.color_id=t.stationary=t.sweep_valid=1;
 t.observed_ms=50;t.usable_until_ms=3050;t.safe_exit_budget_ms=1000;
 CK(SourceStage2_Target(&t,1,50));SourceStage2_Cancel(9);CK(SourceStage2_IsActive() && g_source_stage2_fault==9);
 SourceStage2_Service(60);CK(SourceStage2_IsActive());SourceStage2_Service(70);
 CK(!SourceStage2_IsActive() && !g_source_stage2_completed && g_source_stage2_state==SPF_FAULT);
 CK(!SourceStage2_Start(&c,&io,80));printf("stage2 owner checks=%u failures=0\n",checks);return 0;}
