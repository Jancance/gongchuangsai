
#include "source_parallel_flow.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
static unsigned checks,failures;
#define CK(v) do{checks++;if(!(v)){failures++;printf("FAIL %d %s\n",__LINE__,#v);}}while(0)
static SPF_Config config(void){
 SPF_Config c;memset(&c,0,sizeof(c));c.run_id=42;c.pose_version=7;
 c.live_references_valid=c.deploy_sweep_valid=c.carrier_height_valid=1;
 c.transfer_sweep_valid=c.stow_while_driving_valid=1;c.transaction_timeout_ms=12000;
 c.deploy=(SPF_Pose){180,48,0};c.transfer=(SPF_Pose){35.1f,0,60000};
 c.place=(SPF_Pose){35.1f,0,70000};c.initial=(SPF_Pose){0,0,0};
 c.gripper_open_deg=210;c.gripper_close_deg=140;c.initial_gripper_deg=145;
 c.requested_colors[0]=1;c.requested_colors[1]=4;c.requested_colors[2]=2;return c;
}
static SPF_Target target(unsigned item,unsigned now){
 SPF_Target t;memset(&t,0,sizeof(t));t.run_id=42;t.pose_version=7;t.sequence=item+1;
 t.observed_ms=now;t.usable_until_ms=now+3000;t.color_id=item==0?1:item==1?4:2;
 t.instance_id=100+item;t.stationary=t.sweep_valid=1;t.safe_exit_budget_ms=2400;
 t.pick=(SPF_Pose){193.45f,164.07f,108400};return t;
}
typedef struct{unsigned now,starts,prepared,fail_prepare,fail_start,fail_poll,
 stop_retry,start_cost,start_calls[3],finish[3],stop_calls[3],stop_mask;} Fake;
static unsigned idx(unsigned bit){return bit==1?0:bit==2?1:2;}
static uint32_t prepare(void *p,uint32_t bit,const SPF_Command *c){Fake *x=p;(void)c;
 if(bit==x->fail_prepare)return 2;x->prepared|=bit;return 1;}
static uint32_t start(void *p,uint32_t bit,const SPF_Command *c){Fake *x=p;unsigned i=idx(bit);
 CK(x->prepared==c->axes_mask);x->start_calls[i]++;x->starts|=bit;x->now+=x->start_cost;
 x->finish[i]=x->now+(bit==1?300:bit==2?800:1741);return bit==x->fail_start?0:1;}
static uint32_t poll(void *p,uint32_t bit){Fake *x=p;
 if(bit==x->fail_poll)return 2;return x->now>=x->finish[idx(bit)];}
static uint32_t stop(void *p,uint32_t bit){Fake *x=p;unsigned i=idx(bit);x->stop_calls[i]++;
 if(x->stop_calls[i]<=x->stop_retry)return 0;x->stop_mask|=bit;return 1;}
static uint32_t clock_ms(void *p){return ((Fake *)p)->now;}
static SPF_GroupIO io(Fake *p){SPF_GroupIO v={p,prepare,start,poll,stop,clock_ms};return v;}
int main(void){
 SPF_Config cfg=config();SPF_Flow f={0};SPF_Command c,again;SPF_Target t;unsigned now=100;
 CK(SPF_Begin(&f,&cfg,now));CK(SPF_ReadCommand(&f,&c,now));
 CK(c.kind==SPF_CMD_DEPLOY && c.axes_mask==3 && c.dm_deg_s==300 && c.extension_rpm==106);
 CK(!SPF_ReadCommand(&f,&again,now));CK(!SPF_Report(&f,c.sequence+1,1,3,now));
 CK(SPF_Report(&f,c.sequence,1,3,now));CK(SPF_ReadCommand(&f,&c,now));CK(c.kind==SPF_CMD_ALIGN);
 CK(SPF_Report(&f,c.sequence,1,0,now));CK(f.state==SPF_WAIT_TARGET);
 for(unsigned item=0;item<3;item++){
  t=target(item,now);t.pose_version=6;CK(!SPF_TargetReady(&f,&t,now));t.pose_version=7;
  t.stationary=0;CK(!SPF_TargetReady(&f,&t,now));t.stationary=1;
  t.safe_exit_budget_ms=3001;CK(!SPF_TargetReady(&f,&t,now));t.safe_exit_budget_ms=2400;
  t.observed_ms=now-151;CK(!SPF_TargetReady(&f,&t,now));t.observed_ms=now;
  if(item){t.instance_id=100;CK(!SPF_TargetReady(&f,&t,now));t.instance_id=100+item;}
  CK(SPF_TargetReady(&f,&t,now));
  for(unsigned action=SPF_CMD_PICK;action<=SPF_CMD_RELEASE;action++){
   CK(SPF_ReadCommand(&f,&c,now));CK(c.kind==action && c.item==item);
   CK(c.dm_deg_s==300 && c.extension_rpm==106 && c.lift_rpm==1846.4f && c.keep_vm_on);
   if(action==SPF_CMD_PICK)CK(c.axes_mask==7 && c.goal.dm_deg>180 && c.goal.extension_mm>50);
   CK(SPF_Report(&f,c.sequence,1,c.axes_mask,now));now+=10;
  }
  CK(f.placed==item+1);
  if(item<2){CK(SPF_ReadCommand(&f,&c,now));CK(c.kind==SPF_CMD_NEXT && !c.return_to_top_after_all_three);
   CK(SPF_Report(&f,c.sequence,1,7,now));}
 }
 CK(f.state==SPF_ROUTE_AND_STOW);CK(SPF_ReadCommand(&f,&c,now));
 CK(c.kind==SPF_CMD_ROUTE_AND_STOW && c.route_stage==3 && c.axes_mask==(7|8|16));
 CK(c.goal.lift_pulses==0 && c.gripper_deg==145 && c.return_to_top_after_all_three);
 CK(SPF_Report(&f,c.sequence,1,SPF_ROUTE,now));CK(f.state==SPF_ROUTE_AND_STOW);
 SPF_Service(&f,now+13000);CK(f.state==SPF_ROUTE_AND_STOW); /* route not arm-timeout blocked */
 CK(SPF_Report(&f,c.sequence,1,SPF_STOW,now));CK(f.state==SPF_DONE);
 CK(!SPF_Report(&f,c.sequence,1,16,now));
 /* Missing measurements do not forbid initial preparation, but block pickup. */
 f=(SPF_Flow){0};cfg=config();cfg.carrier_height_valid=0;CK(SPF_Begin(&f,&cfg,now));
 CK(SPF_ReadCommand(&f,&c,now));CK(SPF_Report(&f,c.sequence,1,3,now));
 CK(SPF_ReadCommand(&f,&c,now));CK(SPF_Report(&f,c.sequence,1,0,now));
 t=target(0,now);CK(!SPF_TargetReady(&f,&t,now));
 SPF_Service(&f,now+12000);CK(f.state==SPF_FAULT);CK(SPF_ReadCommand(&f,&c,now+12000));CK(c.kind==SPF_CMD_STOP);
 /* Parallel hardware transaction simulation: failures never leave an axis un-stopped. */
 for(unsigned scenario=0;scenario<7;scenario++){
  SPF_Group g={0};Fake x={0};SPF_GroupIO ops=io(&x);x.start_cost=2;
  c=(SPF_Command){0};c.axes_mask=7;c.goal=(SPF_Pose){193,164,108400};
  c.dm_deg_s=300;c.extension_rpm=106;c.lift_rpm=1846.4f;c.stepper_accel_rpm_s=45055;c.keep_vm_on=1;
  if(scenario==1)x.fail_prepare=2;if(scenario==2)x.fail_start=2;
  if(scenario==3)x.fail_poll=4;if(scenario==4)x.start_cost=11;
  if(scenario==5)x.stop_retry=3;
  CK(!SPF_GroupBegin(&g,&c,&ops,0,20,5000));
  CK(SPF_GroupBegin(&g,&c,&ops,1,20,scenario==6?100:5000));
  for(unsigned step=0;step<600 && g.state!=3;step++){
   x.now+=10;SPF_GroupService(&g);
   if(scenario==5 && g.state==2)SPF_GroupCancel(&g,9);
   if(g.state==4 && !g.stop_mask)break;
  }
  if(scenario==0){CK(g.state==3 && g.done==7);CK(g.last_start_ms-g.first_start_ms==6);
   CK(x.now<1900);CK(x.start_calls[0]==1 && x.start_calls[1]==1 && x.start_calls[2]==1);}
  else{CK(g.state==4 && !g.stop_mask && x.stop_mask==7);CK(g.fault!=0);}
 }
 printf("SOURCE_PARALLEL_FLOW checks%u failures%u\n",checks,failures);return failures?1:0;
}
