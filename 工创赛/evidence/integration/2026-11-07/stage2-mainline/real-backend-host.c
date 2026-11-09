
#include "source_stage2_backend.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
static uint32_t owned,busy,ready=1,path_ok=1,start_ok=1,route_ok=1,route_done,route_stopped=1;
static uint32_t g_source_parallel_cause,g_vehicle_arm_cause,g_source_parallel_state;
static uint32_t g_source_parallel_stop_mask,g_vehicle_arm_stop_pending,g_lift_ruler_stop_pending;
static uint32_t g_it_request,g_it_state,g_vis_task_valid=1,g_vis_task_run_id=10,g_vis_task_item;
static uint32_t g_vis_pending_seq,g_vis_pending_slot,g_vis_task_colors[3]={1,4,6};
static uint32_t g_lift_compare_target_pulses=118800,target_ok=1,rejected_targets;
volatile uint32_t g_source_stage2_state=SPF_WAIT_TARGET;
static float g_src_pick_deg=193,g_src_extension_mm=48;
static unsigned arm_calls,grip_calls,carrier_calls,route_calls,cancels,last_pwm,checks,failures;
static SPF_Command arm_command;static SPF_Target selected_target;static S2_IO captured;
static uint32_t it_platform(void){return ready;}
static uint32_t SingleTeach_ReferencesReady(void){return ready;}
static uint32_t VehicleArm_Busy(void){return busy;}
static uint32_t VehicleSourceParallel_Owns(void){return owned;}
static void VehicleSourceParallel_Cancel(void){cancels++;}
static uint32_t VehicleArm_BeginSourceParallel(const SPF_Command*c,uint32_t verified,uint32_t driving){
 (void)verified;(void)driving;arm_calls++;arm_command=*c;if(start_ok){owned=1;g_source_parallel_state=1;}return start_ok;}
static uint32_t jiazhua(float deg){grip_calls++;return deg>=0 && deg<=270;}
static uint32_t CarrierPwm_Write(uint32_t pwm){carrier_calls++;last_pwm=pwm;return 1;}
uint32_t SourceStage2_Start(const SPF_Config*c,const S2_IO*io,uint32_t now){(void)c;(void)now;captured=*io;return 1;}
uint32_t SourceStage2_Target(const SPF_Target*t,uint32_t slot,uint32_t now){(void)slot;(void)now;selected_target=*t;return target_ok;}
uint32_t SourceStage2_IsActive(void){return 0;}
static uint32_t Vision_TakeTaskTarget(uint32_t now){(void)now;if(g_vis_pending_seq)return 0;g_vis_pending_seq=45;g_vis_pending_slot=g_vis_task_item+1;return 1;}
uint32_t Vision_RejectUnstarted(uint32_t run,uint32_t seq){if(run!=g_vis_task_run_id || seq!=g_vis_pending_seq)return 0;g_vis_pending_seq=g_vis_pending_slot=0;rejected_targets++;return 1;}
uint32_t S2_VisionGrasp(void*p,uint32_t run,uint32_t seq,const S2_Event*e,uint32_t now){(void)p;(void)run;(void)seq;(void)e;(void)now;return 1;}
uint32_t S2_VisionPlaced(void*p,uint32_t run,uint32_t seq,const S2_Event*e,uint32_t now){(void)p;(void)run;(void)seq;(void)e;(void)now;return 1;}
#include "source_stage2_backend.inc"
#define CK(x) do{checks++;if(!(x)){failures++;printf("FAIL %d\n",__LINE__);}}while(0)
static uint32_t ab(void*p,const SPF_Command*c,uint32_t now){(void)p;(void)c;(void)now;return 1;}
static uint32_t ap(void*p,const SPF_Command*c,S2_Event*e,uint32_t now){(void)p;memset(e,0,sizeof(*e));e->run_id=c->run_id;e->sequence=c->sequence;e->observed_ms=now;e->evidence_flags=S2_ALIGNED;return 1;}
static uint32_t rb(void*p,const SPF_Command*c,uint32_t now){(void)p;(void)c;(void)now;route_calls++;CK(owned==1);return route_ok;}
static uint32_t rp(void*p,const SPF_Command*c,uint32_t now){(void)p;(void)c;(void)now;return route_done;}
static uint32_t rs(void*p,uint32_t now){(void)p;(void)now;return route_stopped;}
static uint32_t path(void*p,const SPF_Command*c){(void)p;(void)c;return path_ok;}
static SPF_Command cmd(unsigned kind,unsigned seq,unsigned item,unsigned axes){
 SPF_Command c;memset(&c,0,sizeof(c));c.run_id=10;c.pose_version=7;c.kind=kind;c.sequence=seq;c.item=item;c.axes_mask=axes;c.gripper_deg=kind==SPF_CMD_CLOSE?140:210;return c;}
int main(void){SPF_Config cfg;S2_BackendHooks hooks={0,ab,ap,rb,rp,rs,path};S2_ServoTiming timing={700,400};SPF_Command c,out;SPF_Target t;S2_Event e,physical;S2_SourceObservation observation;unsigned n;
 memset(&cfg,0,sizeof(cfg));cfg.run_id=10;cfg.pose_version=7;cfg.requested_colors[0]=1;cfg.requested_colors[1]=4;cfg.requested_colors[2]=6;
 CK(!SourceStage2Backend_Start(0,&hooks,&timing,0));g_it_state=1;CK(!SourceStage2Backend_Start(&cfg,&hooks,&timing,0));g_it_state=0;
 CK(SourceStage2Backend_Start(&cfg,&hooks,&timing,0));CK(captured.begin && captured.poll && captured.stop && captured.observe_grasp && captured.commit_loaded);
 c=cmd(SPF_CMD_DEPLOY,1,0,3);CK(captured.begin(0,&c,10)==1 && arm_calls==1 && grip_calls==1 && arm_command.axes_mask==3);
 n=arm_calls;CK(captured.begin(0,&c,20)==1 && arm_calls==n);
 CK(captured.poll(0,&c,&e,30)==1 && !e.completed_mask);owned=0;g_source_parallel_state=3;
 CK(captured.poll(0,&c,&e,40)==1 && e.completed_mask==3);
 c=cmd(SPF_CMD_ALIGN,2,0,0);CK(captured.begin(0,&c,50)==1);CK(captured.poll(0,&c,&e,60)==1 && (e.evidence_flags&S2_ALIGNED));
 g_vis_task_item=1;g_vis_pending_seq=23;g_vis_pending_slot=2;
 memset(&t,0,sizeof(t));t.run_id=10;t.sequence=23;t.instance_id=99;t.color_id=4;t.pick=(SPF_Pose){193,48,118800};
 CK(SourceStage2Backend_Target(&t,2,70));t.pick.extension_mm=49;CK(!SourceStage2Backend_Target(&t,2,70));t.pick.extension_mm=48;
 c=cmd(SPF_CMD_PICK,3,1,7);CK(captured.begin(0,&c,100)==1 && carrier_calls==1 && last_pwm==1389);
 owned=0;g_source_parallel_state=3;CK(captured.poll(0,&c,&e,799)==0);CK(captured.poll(0,&c,&e,800)==1 && e.completed_mask==7);
 c=cmd(SPF_CMD_CLOSE,4,1,0);CK(captured.begin(0,&c,900)==1);CK(captured.poll(0,&c,&e,1400)==0);
 memset(&physical,0,sizeof(physical));physical.run_id=10;physical.sequence=4;physical.observed_ms=1000;physical.instance_id=99;physical.color_id=4;physical.slot=2;physical.evidence_flags=S2_SOURCE_CLEAR|S2_HELD;
 CK(SourceStage2Backend_Observe(&physical,1001));CK(captured.poll(0,&c,&e,1299)==0);CK(captured.poll(0,&c,&e,1300)==1 && e.evidence_flags==physical.evidence_flags && e.observed_ms==1000);
 physical.run_id=11;CK(!SourceStage2Backend_Observe(&physical,1300));physical.run_id=10;physical.sequence=3;CK(!SourceStage2Backend_Observe(&physical,1300));physical.sequence=4;
 CK(!SourceStage2Backend_Observe(&physical,2101));CK(SourceStage2Backend_ReadActive(&out) && out.sequence==4);
 c=cmd(SPF_CMD_ROUTE_AND_STOW,5,2,31);CK(captured.begin(0,&c,1500)==1 && arm_command.axes_mask==7 && route_calls==1);
 CK(captured.poll(0,&c,&e,1600)==1 && e.completed_mask==0);owned=0;g_source_parallel_state=3;
 CK(captured.poll(0,&c,&e,1700)==1 && e.completed_mask==SPF_STOW);
 route_done=1;CK(captured.poll(0,&c,&e,1800)==1 && e.completed_mask==(SPF_ROUTE|SPF_STOW));
 c=cmd(SPF_CMD_ROUTE_AND_STOW,6,2,31);route_ok=0;CK(captured.begin(0,&c,2000)==2 && owned==1);
 route_stopped=0;CK(captured.stop(0,31,2010)==0 && cancels==1);owned=0;g_source_parallel_stop_mask=0;route_stopped=1;
 CK(captured.stop(0,31,2020)==31);
 c=cmd(SPF_CMD_TRANSFER,7,1,7);path_ok=0;n=arm_calls;CK(captured.begin(0,&c,2100)==2 && arm_calls==n);path_ok=1;
 busy=1;CK(captured.begin(0,&c,2200)==0 && arm_calls==n);busy=0;
 g_vis_task_item=3;CK(!SourceStage2Backend_Target(&t,4,2200));
 g_vis_task_item=0;g_vis_pending_seq=0;memset(&observation,0,sizeof(observation));
 observation.run_id=10;observation.pose_version=7;observation.instance_id=700;observation.captured_ms=2300;
 observation.usable_until_ms=5300;observation.stationary=observation.sweep_valid=1;observation.safe_exit_budget_ms=2400;
 CK(!SourceStage2Backend_Select(&observation,2310) && !g_vis_pending_seq);
 observation.capture_verified=1;CK(!SourceStage2Backend_Select(&observation,2451) && !g_vis_pending_seq);
 observation.stationary=0;CK(!SourceStage2Backend_Select(&observation,2310));observation.stationary=1;
 CK(SourceStage2Backend_Select(&observation,2310));
 CK(selected_target.sequence==45 && selected_target.instance_id==700 && selected_target.color_id==1 && selected_target.observed_ms==2300);
 CK(selected_target.pick.dm_deg==193 && selected_target.pick.extension_mm==48 && selected_target.pick.lift_pulses==118800);
 g_vis_pending_seq=0;target_ok=0;CK(!SourceStage2Backend_Select(&observation,2320) && !g_vis_pending_seq && rejected_targets==1);
 printf("stage2 real-backend checks=%u failures=%u\n",checks,failures);return failures?1:0;}
