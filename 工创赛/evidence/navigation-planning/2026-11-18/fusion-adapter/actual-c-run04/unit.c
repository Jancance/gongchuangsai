#include "fusion_adapter.h"
#include <math.h>
#include <string.h>
volatile uint32_t review_assertions;
volatile uint32_t size_adapter_state=sizeof(FusionAdapterState),size_adapter_input=sizeof(FusionAdapterInput),size_adapter_report=sizeof(FusionAdapterReport);
static FusionInput input;static FusionPlan plan,original_plan;static FusionCheck check;
static FusionAdapterState state,saved;static FusionAdapterInput in,backup,original_input;
static FusionObservation obs,old,zero;static FusionAdapterReport report;
static FusionState reference,saved_reference;static FusionOutput desired,old_desired;
static uint32_t mutated;
#define CK(x) do{review_assertions++;if(!(x))return __LINE__;}while(0)
#define CLOSE(a,b,e) CK(fabsf((a)-(b))<(e))
static void time_at(uint32_t ms){uint32_t i;in.map.pose.sample_ms=in.now_ms=in.map.now_ms=ms;in.map.pose.age_ms=0;for(i=0;i<4;i++)in.map.wheel_ms[i]=ms;in.map.angle_ms=ms;}
static void tick(uint32_t dt){time_at(in.map.pose.sample_ms+dt);in.map.pose.updates++;}
static uint32_t setup(void)
{
 FusionConfig *c=&input.config;memset(&input,0,sizeof(input));memset(&in,0,sizeof(in));
 input.model_only=1;input.count=2;input.identity.run_id=input.identity.start_id=input.identity.map_version=input.identity.layout_id=input.identity.path_id=1;
 c->body_length_mm=270;c->body_width_mm=250;c->heading_budget_deg=3;c->position_tolerance_mm=5;c->max_error_mm=40;
 c->stop_speed_mm_s=2;c->stop_omega_deg_s=.5f;c->acceleration_hint_mm_s2=200;c->cross_gain_s=.5f;c->lateral_cap_mm_s=20;
 c->preview_s=.1f;c->wheel_limit_rpm=80;c->heading_reserve_rpm=5;c->speed_state_margin_mm_s=2;c->max_age_ms=100;
 c->geometry.wheel_diameter_mm=77;c->geometry.track_width_mm=212;c->geometry.wheelbase_mm=206;
 input.points[0].x_mm=300;input.points[0].y_mm=300;input.points[1].x_mm=1200;input.points[1].y_mm=300;input.speed_mm_s[0]=200;
 in.identity=input.identity;in.map.run_id=in.map.start_id=in.map.map_version=1;in.map.begin_ms=900;in.map.based=1;in.map.state=APP_MAP_TRACKING;
 in.map.raw_x_mm=in.map.pose.x_mm=300;in.map.raw_y_mm=in.map.pose.y_mm=300;in.map.pose.valid=1;in.map.pose.updates=1;in.map.source=APP_MAP_RAW_WHEEL_HWT;
 time_at(1000);FusionAdapter_Reset(&state);FusionReference_Reset(&reference);return FusionReference_Prepare(&plan,&input,&check);
}
static uint32_t observe(void)
{
 uint32_t status;original_plan=plan;original_input=in;
 status=FusionAdapter_Observe(&plan,&state,&in,&obs,&report);
 if(memcmp(&plan,&original_plan,sizeof(plan))||memcmp(&in,&original_input,sizeof(in)))mutated=1;
 return status;
}
static uint32_t unavailable(void){return memcmp(&obs,&zero,sizeof(obs))==0;}
static uint32_t advance_reference(void){return FusionReference_Step(&plan,&reference,&obs,&desired);}
__attribute__((section("i.test_entry"))) uint32_t test_entry(void)
{
 uint32_t i,j,k,status,*member[6];float *number[7];union{uint32_t u;float f;} bits;float invalid[3],v;
 CK(setup());CK(observe()==FUSION_ADAPTER_WAIT&&unavailable());tick(20);CK(observe()==FUSION_ADAPTER_READY&&obs.valid&&!obs.stopped&&report.estimated_stationary);
 old=obs;saved=state;in.now_ms++;in.map.now_ms++;in.map.pose.age_ms++;
 in.map.correction_x_mm=50;in.map.source=2;in.map.calibration_scope=3;in.map.nav_applied=1;in.map.action_epoch=10;in.map.calibration_flags=7;in.map.calibration_direction=3;in.map.angle_ms++;for(i=0;i<4;i++)in.map.wheel_ms[i]++;
 CK(observe()==1);old.now_ms=in.now_ms;CK(memcmp(&obs,&old,sizeof(obs))==0&&memcmp(&state,&saved,sizeof(state))==0);
 CK(report.correction_x_mm==50&&report.source==2&&report.calibration_flags==7&&report.calibration_direction==3&&report.window_ms==20);
 /* Six caller fields and three actual map fields reject without state change. */
 for(i=0;i<9;i++){
  CK(setup());observe();tick(20);observe();saved=state;
  member[0]=&in.identity.run_id;member[1]=&in.identity.start_id;member[2]=&in.identity.map_version;member[3]=&in.identity.layout_id;member[4]=&in.identity.path_id;member[5]=&in.identity.direction;
  if(i<6)(*member[i])++;else if(i==6)in.map.run_id++;else if(i==7)in.map.start_id=2;else in.map.map_version++;
  CK(observe()==2&&unavailable()&&memcmp(&state,&saved,sizeof(state))==0);
 }
 /* Bad prepared shell/domain never mutates an already-ready state. */
 for(i=0;i<21;i++){
  CK(setup());observe();tick(20);observe();saved=state;
  switch(i){case 0:plan.valid=0;break;case 1:plan.signature=0;break;case 2:plan.input.count=1;break;case 3:plan.input.count=33;break;case 4:plan.input.box_count=33;break;case 5:plan.input.model_only=0;break;
  case 6:plan.input.identity.run_id=0;break;case 7:plan.input.identity.start_id=3;break;case 8:plan.input.identity.map_version=0;break;case 9:plan.input.identity.layout_id=0;break;case 10:plan.input.identity.path_id=0;break;case 11:plan.input.identity.direction=2;break;
  case 12:plan.input.config.max_age_ms=99;break;case 13:plan.input.config.max_age_ms=1001;break;case 14:plan.input.config.stop_speed_mm_s=-1;break;case 15:plan.input.config.stop_speed_mm_s=5.01f;break;case 16:plan.input.config.stop_omega_deg_s=-1;break;case 17:plan.input.config.stop_omega_deg_s=2.01f;break;
  case 18:bits.u=0x7fc00000U;plan.input.config.stop_speed_mm_s=bits.f;break;case 19:bits.u=0x7f800000U;plan.input.config.stop_omega_deg_s=bits.f;break;default:plan.input.config.max_age_ms=0;}
  CK(observe()==2&&unavailable()&&report.reason==FUSION_ADAPTER_BAD_PLAN&&memcmp(&state,&saved,sizeof(state))==0);
 }
 CK(setup());observe();tick(20);observe();plan.signature++;CK(observe()==0&&unavailable()&&!state.ready);
 CK(setup());observe();input.identity.direction=1;CK(FusionReference_Prepare(&plan,&input,&check));in.identity=input.identity;CK(observe()==0&&state.identity.direction==1);
 /* Short-window anchor is distinct from latest accepted sample. */
 CK(setup());observe();for(i=1;i<20;i++){tick(1);in.map.raw_x_mm+=.4f;in.map.pose.x_mm=in.map.raw_x_mm;CK(observe()==0&&unavailable()&&state.anchor_ms==1000);saved=state;CK(observe()==0&&memcmp(&saved,&state,sizeof(state))==0);}
 saved=state;in.map.raw_x_mm++;in.map.pose.updates=0;CK(observe()==2&&unavailable()&&memcmp(&saved,&state,sizeof(state))==0);in.map.raw_x_mm--;in.map.pose.updates=saved.updates;tick(1);in.map.raw_x_mm+=.4f;in.map.pose.x_mm=in.map.raw_x_mm;CK(observe()==1);CLOSE(obs.vx_mm_s,400,.03f);
 tick(1);CK(observe()==0&&!state.ready&&state.anchor_ms==1020);saved=state;in.map.pose.sample_ms--;in.map.pose.updates=0;CK(observe()==2&&memcmp(&saved,&state,sizeof(state))==0);
 CK(setup());observe();tick(20);observe();in.map.pose.updates=0;CK(observe()==0&&state.anchor_ms==1020);tick(20);CK(observe()==1);in.map.begin_ms=1030;CK(observe()==0&&!state.ready);
 tick(101);CK(observe()==0&&state.anchor_ms==1141);tick(20);CK(observe()==1);
 /* Map health loss clears the window; every recovery needs two new samples. */
 for(i=0;i<8;i++){
  CK(setup());observe();tick(20);observe();
  switch(i){case 0:in.map.based=0;break;case 1:in.map.state=APP_MAP_GAP;break;case 2:in.map.pose.valid=0;break;case 3:in.map.pose.reasons=1;break;case 4:in.map.pose.continuity_lost=1;break;case 5:in.map.pose.local_origin=1;break;case 6:in.now_ms+=101;in.map.angle_ms=in.now_ms;for(j=0;j<4;j++)in.map.wheel_ms[j]=in.now_ms;break;default:in.now_ms--;}
  CK(observe()==2&&unavailable()&&!state.bound);in.map.based=in.map.pose.valid=1;in.map.state=APP_MAP_TRACKING;in.map.pose.reasons=in.map.pose.continuity_lost=in.map.pose.local_origin=0;time_at(1200);in.map.pose.updates++;
  CK(observe()==0&&unavailable());tick(20);CK(observe()==1);
 }
 for(i=0;i<4;i++)for(j=0;j<3;j++){
  CK(setup());in.map.begin_ms=500;time_at(1400);observe();tick(20);in.map.wheel_ms[i]=in.now_ms-399;CK(observe()==1);saved=state;
  if(j==0)in.map.wheel_ms[i]=in.now_ms-400;else if(j==1)in.map.wheel_ms[i]=in.now_ms+1;else in.map.wheel_ms[i]=499;
  CK(observe()==2&&unavailable()&&!state.bound);
 }
 for(j=0;j<3;j++){
  CK(setup());in.map.begin_ms=500;observe();tick(20);in.map.angle_ms=in.now_ms-99;CK(observe()==1);
  if(j==0)in.map.angle_ms=in.now_ms-100;else if(j==1)in.map.angle_ms=in.now_ms+1;else in.map.angle_ms=499;CK(observe()==2&&unavailable()&&!state.bound);
 }
 /* Five core floating fields conflict; correction is audit metadata only. */
 for(i=0;i<5;i++){
  CK(setup());observe();tick(20);observe();saved=state;
  number[0]=&in.map.raw_x_mm;number[1]=&in.map.raw_y_mm;number[2]=&in.map.pose.x_mm;number[3]=&in.map.pose.y_mm;number[4]=&in.map.pose.heading_deg;
  *number[i]+=1;CK(observe()==2&&unavailable()&&memcmp(&state,&saved,sizeof(state))==0);
 }
 /* All consumed floating fields reject NaN/Inf and clear source health. */
 bits.u=0x7fc00000U;invalid[0]=bits.f;bits.u=0x7f800000U;invalid[1]=bits.f;bits.u=0xff800000U;invalid[2]=bits.f;
 for(i=0;i<7;i++)for(j=0;j<3;j++){
  CK(setup());observe();tick(20);number[0]=&in.map.raw_x_mm;number[1]=&in.map.raw_y_mm;number[2]=&in.map.pose.x_mm;number[3]=&in.map.pose.y_mm;number[4]=&in.map.correction_x_mm;number[5]=&in.map.correction_y_mm;number[6]=&in.map.pose.heading_deg;
  *number[i]=invalid[j];CK(observe()==2&&unavailable()&&!state.bound);
 }
 CK(setup());in.map.raw_x_mm=3e38f;observe();saved=state;tick(20);in.map.raw_x_mm=-3e38f;CK(observe()==2&&unavailable()&&memcmp(&state,&saved,sizeof(state))==0);
 /* uint32 source/stop/begin rollover and negative heading wrap. */
 CK(setup());in.map.begin_ms=0xffffff00U;time_at(0xfffffff0U);in.map.pose.heading_deg=-179;observe();tick(16);CK(observe()==0&&state.latest.sample_ms==0);tick(4);in.map.pose.heading_deg=179;in.stop_confirmed=1;in.stop_run_id=1;in.stop_sample_ms=0;CK(observe()==1&&obs.sample_ms==4&&obs.stopped);CLOSE(obs.omega_deg_s,-100,.01f);
 saved=state;in.map.pose.sample_ms--;in.map.pose.updates=0;CK(observe()==2&&unavailable()&&memcmp(&state,&saved,sizeof(state))==0);
 CK(setup());observe();saved=state;time_at(1000+0x80000000U);CK(observe()==2&&unavailable()&&memcmp(&state,&saved,sizeof(state))==0);
 /* Stop is a separate, bounded fact; asynchronous updates cannot rewrite cache. */
 CK(setup());observe();tick(20);observe();saved=state;in.stop_confirmed=1;in.stop_run_id=1;in.stop_sample_ms=in.now_ms;CK(observe()==1&&!obs.stopped&&report.stop_pending&&memcmp(&state,&saved,sizeof(state))==0);
 tick(20);CK(observe()==1&&obs.stopped);saved=state;old=obs;
 for(i=0;i<6;i++){
  backup=in;in.stop_sample_ms=in.now_ms;
  if(i==0)in.stop_confirmed=0;else if(i==1)in.stop_run_id=2;else if(i==2)in.stop_sample_ms=in.now_ms+1;else if(i==3)in.stop_sample_ms=899;else if(i==4)in.stop_sample_ms=in.now_ms-101;else in.stop_sample_ms=in.now_ms+0x80000000U;
  CK(observe()==2&&unavailable()&&memcmp(&state,&saved,sizeof(state))==0);in=backup;
 }
 in.now_ms=1121;in.map.angle_ms=1121;for(i=0;i<4;i++)in.map.wheel_ms[i]=1121;in.stop_sample_ms=1121;CK(observe()==2&&unavailable()&&memcmp(&state,&saved,sizeof(state))==0);
 for(i=0;i<2;i++){
  CK(setup());in.map.begin_ms=800;observe();tick(20);in.stop_confirmed=1;in.stop_run_id=1;in.stop_sample_ms=in.now_ms-(100+i);CK(observe()==1&&obs.stopped==(i==0));
 }
 CK(setup());observe();tick(20);in.map.raw_x_mm+=8;in.map.pose.x_mm+=8;in.stop_confirmed=1;in.stop_run_id=1;in.stop_sample_ms=in.now_ms;CK(observe()==1&&obs.stopped&&!report.estimated_stationary);CLOSE(obs.vx_mm_s,400,.01f);
 /* Component bounds are those of Reference, rather than a new norm limit. */
 CK(setup());observe();tick(20);in.map.raw_x_mm+=200;in.map.raw_y_mm+=200;CK(observe()==1);CLOSE(obs.vx_mm_s,10000,.01f);CLOSE(obs.vy_mm_s,10000,.01f);
 for(i=0;i<2;i++){
  CK(setup());observe();saved=state;tick(20);v=i? -10000.1f:10000.1f;in.map.raw_x_mm+=v*.02f;CK(observe()==2&&unavailable()&&memcmp(&state,&saved,sizeof(state))==0);
 }
 /* All legal integer windows retain the existing component boundary. */
 for(i=20;i<=100;i++)for(j=0;j<2;j++){
  CK(setup());observe();tick(i);v=j?-10000.0f:10000.0f;in.map.raw_x_mm+=v*(float)i/1000.0f;CK(observe()==1);CLOSE(obs.vx_mm_s,v,.001f);
 }
 for(i=0;i<3;i++)for(j=0;j<3;j++){
  CK(setup());v=i==0?1e8f:(i==1?-1e8f:3e38f);in.map.raw_x_mm=v;observe();tick(20);in.map.raw_x_mm+=j==0?-200.0f:(j==1?0.0f:200.0f);CK(observe()==1);
 }
 /* All legal integer windows retain the existing component boundary. */
 for(i=20;i<=100;i++)for(j=0;j<2;j++){
  CK(setup());observe();tick(i);v=j?-10000.0f:10000.0f;in.map.raw_x_mm+=v*(float)i/1000.0f;CK(observe()==1);CLOSE(obs.vx_mm_s,v,.01f);
 }
 for(i=0;i<3;i++)for(j=0;j<3;j++){
  CK(setup());v=i==0?1e8f:(i==1?-1e8f:3e38f);in.map.raw_x_mm=v;observe();tick(20);in.map.raw_x_mm+=j==0?-200.0f:(j==1?0.0f:200.0f);CK(observe()==1);
 }
 /* NULL contracts and optional report: no invalid dereference or stale output. */
 CK(setup());observe();tick(20);observe();saved=state;memset(&obs,0x5a,sizeof(obs));CK(FusionAdapter_Observe(0,&state,&in,&obs,&report)==2&&unavailable()&&memcmp(&state,&saved,sizeof(state))==0);
 CK(FusionAdapter_Observe(&plan,0,&in,&obs,&report)==2&&unavailable());CK(FusionAdapter_Observe(&plan,&state,0,&obs,&report)==2&&unavailable());CK(FusionAdapter_Observe(&plan,&state,&in,0,&report)==2&&memcmp(&state,&saved,sizeof(state))==0);
 CK(FusionAdapter_Observe(&plan,&state,&in,&obs,0)==1);FusionAdapter_Reset(0);FusionAdapter_Reset(&state);CK(!state.bound);
 /* Actual Reference integration: first/duplicate/stop/recovery are not invented. */
 CK(setup());CK(observe()==0);CK(advance_reference()==FUSION_INVALID&&!reference.seen);
 tick(20);CK(observe()==1&&!obs.stopped);CK(advance_reference()==FUSION_TURN_START&&reference.phase==0);saved_reference=reference;old_desired=desired;
 in.now_ms++;CK(observe()==1);CK(advance_reference()==FUSION_TURN_START&&memcmp(&reference,&saved_reference,sizeof(reference))==0&&memcmp(&desired,&old_desired,sizeof(desired))==0);
 in.stop_confirmed=1;in.stop_run_id=1;in.stop_sample_ms=in.now_ms;CK(observe()==1&&!obs.stopped);CK(advance_reference()==FUSION_TURN_START&&reference.phase==0);
 tick(20);CK(observe()==1&&obs.stopped);CK(advance_reference()==FUSION_TURN_START&&reference.phase==1);
 tick(20);CK(observe()==1);CK(advance_reference()==FUSION_FOLLOW);saved_reference=reference;old_desired=desired;CK(observe()==1);CK(advance_reference()==FUSION_FOLLOW&&memcmp(&reference,&saved_reference,sizeof(reference))==0&&memcmp(&desired,&old_desired,sizeof(desired))==0);
 in.map.state=APP_MAP_GAP;CK(observe()==2);CK(advance_reference()==FUSION_INVALID&&memcmp(&reference,&saved_reference,sizeof(reference))==0);in.map.state=APP_MAP_TRACKING;in.map.raw_x_mm=1000;in.map.pose.x_mm=300;tick(20);CK(observe()==0);tick(20);CK(observe()==1);CLOSE(obs.vx_mm_s,0,.01f);CK(advance_reference()==FUSION_FOLLOW);
 /* Current high speed + valid stop cannot advance initial Reference phase. */
 CK(setup());observe();tick(20);in.map.raw_x_mm+=8;in.stop_confirmed=1;in.stop_run_id=1;in.stop_sample_ms=in.now_ms;CK(observe()==1&&obs.stopped);CK(advance_reference()==FUSION_TURN_START&&reference.phase==0);
 /* At the endpoint, zero difference without stop cannot yield DONE. */
 CK(setup());observe();tick(20);in.stop_confirmed=1;in.stop_run_id=1;in.stop_sample_ms=in.now_ms;observe();advance_reference();tick(20);observe();advance_reference();
 in.stop_confirmed=0;in.map.pose.x_mm=1200;tick(20);CK(observe()==1&&!obs.stopped);CK(advance_reference()==FUSION_FOLLOW&&reference.phase==1);tick(20);observe();CK(advance_reference()!=FUSION_DONE);
 /* MATLAB v8 numerical oracle: raw speed, corrected pose, world map heading. */
#include "adapter_vectors.inc"
 CK(!mutated);CK(size_adapter_state>0&&size_adapter_input>0&&size_adapter_report>0);return 0;
}
