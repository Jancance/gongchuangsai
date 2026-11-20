#include "fusion_adapter.h"
#include <float.h>
#include <math.h>
#include <string.h>
static uint32_t finite_f(float x){return x>=-FLT_MAX&&x<=FLT_MAX;}
static float abs_f(float x){return x<0.0f?-x:x;}
static float wrap(float h){float x=fmodf(h+180.0f,360.0f);if(x<0.0f)x+=360.0f;return x-180.0f;}
static uint32_t not_before(uint32_t a,uint32_t b){return a-b<0x80000000U;}
static uint32_t identity(const FusionIdentity *a,const FusionIdentity *b)
{return a->run_id==b->run_id&&a->start_id==b->start_id&&a->map_version==b->map_version&&a->layout_id==b->layout_id&&a->path_id==b->path_id&&a->direction==b->direction;}
static uint32_t plan_ok(const FusionPlan *p)
{
 const FusionIdentity *i;const FusionConfig *c;
 if(!p||p->valid!=1U||!p->signature||p->input.model_only!=1U||p->input.count<2U||p->input.count>FUSION_POINTS||p->input.box_count>FUSION_BOXES)return 0U;
 i=&p->input.identity;c=&p->input.config;
 return i->run_id&&(i->start_id==1U||i->start_id==2U)&&i->map_version&&i->layout_id&&i->path_id&&i->direction<=1U&&
 c->max_age_ms>=100U&&c->max_age_ms<=1000U&&finite_f(c->stop_speed_mm_s)&&c->stop_speed_mm_s>=0&&c->stop_speed_mm_s<=5&&
 finite_f(c->stop_omega_deg_s)&&c->stop_omega_deg_s>=0&&c->stop_omega_deg_s<=2;
}
static FusionAdapterCore core(const AppMapPose *m)
{
 FusionAdapterCore c;c.sample_ms=m->pose.sample_ms;c.begin_ms=m->begin_ms;c.raw_x_mm=m->raw_x_mm;c.raw_y_mm=m->raw_y_mm;
 c.x_mm=m->pose.x_mm;c.y_mm=m->pose.y_mm;c.heading_deg=m->pose.heading_deg;return c;
}
static uint32_t same_core(const FusionAdapterCore *a,const FusionAdapterCore *b)
{return a->sample_ms==b->sample_ms&&a->begin_ms==b->begin_ms&&a->raw_x_mm==b->raw_x_mm&&a->raw_y_mm==b->raw_y_mm&&a->x_mm==b->x_mm&&a->y_mm==b->y_mm&&a->heading_deg==b->heading_deg;}
void FusionAdapter_Reset(FusionAdapterState *s){if(s)memset(s,0,sizeof(*s));}
static void prime(FusionAdapterState *s,const FusionPlan *p,const FusionAdapterInput *in)
{
 const AppMapPose *m=&in->map;FusionAdapter_Reset(s);s->bound=1U;s->signature=p->signature;s->identity=p->input.identity;
 s->anchor_ms=m->pose.sample_ms;s->anchor_raw_x_mm=m->raw_x_mm;s->anchor_raw_y_mm=m->raw_y_mm;s->anchor_heading_deg=m->pose.heading_deg;
 s->latest=core(m);s->updates=m->pose.updates;
}
static uint32_t stop_ok(const FusionPlan *p,const FusionAdapterInput *in,uint32_t ms,uint32_t run)
{return run==p->input.identity.run_id&&not_before(in->now_ms,ms)&&not_before(ms,in->map.begin_ms)&&in->now_ms-ms<=p->input.config.max_age_ms;}
static void audit(FusionAdapterReport *r,const AppMapPose *m)
{
 uint32_t i;r->source=m->source;r->calibration_scope=m->calibration_scope;r->nav_applied=m->nav_applied;r->action_epoch=m->action_epoch;
 r->calibration_flags=m->calibration_flags;r->calibration_direction=m->calibration_direction;
 for(i=0;i<4U;i++)r->wheel_ms[i]=m->wheel_ms[i];r->angle_ms=m->angle_ms;r->map_now_ms=m->now_ms;r->pose_age_ms=m->pose.age_ms;
 r->raw_x_mm=m->raw_x_mm;r->raw_y_mm=m->raw_y_mm;r->correction_x_mm=m->correction_x_mm;r->correction_y_mm=m->correction_y_mm;
}
static uint32_t result(FusionAdapterReport *r,uint32_t status,uint32_t reason)
{r->status=status;r->reason=reason;return status;}
static uint32_t stationary(const FusionConfig *c,const FusionObservation *o)
{return sqrtf(o->vx_mm_s*o->vx_mm_s+o->vy_mm_s*o->vy_mm_s)<=c->stop_speed_mm_s&&abs_f(o->omega_deg_s)<=c->stop_omega_deg_s;}
uint32_t FusionAdapter_Observe(const FusionPlan *p,FusionAdapterState *s,const FusionAdapterInput *in,FusionObservation *o,FusionAdapterReport *r)
{
 FusionAdapterReport local;FusionAdapterCore current;FusionAdapterState next;const AppMapPose *m;const FusionConfig *c;
 uint32_t i,bound,stop,dt;float vx,vy,omega;
 if(!r)r=&local;memset(r,0,sizeof(*r));r->feedback_source=FUSION_ESTIMATED_FEEDBACK;
 if(o)memset(o,0,sizeof(*o));
 if(!o||!s||!in||!plan_ok(p))return result(r,FUSION_ADAPTER_UNAVAILABLE,FUSION_ADAPTER_BAD_PLAN);
 m=&in->map;c=&p->input.config;audit(r,m);
 if(!identity(&in->identity,&p->input.identity)||m->run_id!=p->input.identity.run_id||m->start_id!=p->input.identity.start_id||m->map_version!=p->input.identity.map_version)
  return result(r,FUSION_ADAPTER_UNAVAILABLE,FUSION_ADAPTER_IDENTITY);
 bound=s->bound&&s->signature==p->signature&&identity(&s->identity,&p->input.identity)&&s->latest.begin_ms==m->begin_ms;
 if(bound&&!not_before(m->pose.sample_ms,s->latest.sample_ms))return result(r,FUSION_ADAPTER_UNAVAILABLE,FUSION_ADAPTER_OLD_SAMPLE);
 if(!not_before(in->now_ms,m->pose.sample_ms)||!not_before(m->pose.sample_ms,m->begin_ms)||
 !not_before(in->now_ms,m->angle_ms)||!not_before(m->angle_ms,m->begin_ms))goto bad_map;
 for(i=0;i<4U;i++)if(!not_before(in->now_ms,m->wheel_ms[i])||!not_before(m->wheel_ms[i],m->begin_ms)||in->now_ms-m->wheel_ms[i]>=400U)goto bad_map;
 if(m->based!=1U||m->state!=APP_MAP_TRACKING||m->pose.valid!=1U||m->pose.reasons||m->pose.continuity_lost||m->pose.local_origin||
 in->now_ms-m->pose.sample_ms>c->max_age_ms||in->now_ms-m->angle_ms>=100U||
 !finite_f(m->raw_x_mm)||!finite_f(m->raw_y_mm)||!finite_f(m->pose.x_mm)||!finite_f(m->pose.y_mm)||
 !finite_f(m->correction_x_mm)||!finite_f(m->correction_y_mm)||!finite_f(m->pose.heading_deg)||abs_f(m->pose.heading_deg)>3600.0f)goto bad_map;
 stop=in->stop_confirmed==1U&&stop_ok(p,in,in->stop_sample_ms,in->stop_run_id);
 if(!bound){prime(s,p,in);return result(r,FUSION_ADAPTER_WAIT,FUSION_ADAPTER_PRIME);}
 current=core(m);
 if(m->pose.sample_ms==s->latest.sample_ms&&!same_core(&current,&s->latest))return result(r,FUSION_ADAPTER_UNAVAILABLE,FUSION_ADAPTER_CORE_CONFLICT);
 if(m->pose.updates<s->updates){prime(s,p,in);return result(r,FUSION_ADAPTER_WAIT,FUSION_ADAPTER_PRIME);}
 if(m->pose.sample_ms==s->latest.sample_ms){
  r->stop_pending=stop!=s->cached.stopped||(stop&&in->stop_sample_ms!=s->stop_sample_ms);
  if(!s->ready)return result(r,FUSION_ADAPTER_WAIT,FUSION_ADAPTER_SHORT_WINDOW);
  if(s->cached.stopped&&(!stop||!stop_ok(p,in,s->stop_sample_ms,s->stop_run_id)))return result(r,FUSION_ADAPTER_UNAVAILABLE,FUSION_ADAPTER_STOP_INVALID);
  *o=s->cached;o->now_ms=in->now_ms;r->window_ms=s->window_ms;r->estimated_stationary=stationary(c,o);return result(r,FUSION_ADAPTER_READY,0U);
 }
 dt=m->pose.sample_ms-s->anchor_ms;r->window_ms=dt;
 if(dt>100U){prime(s,p,in);return result(r,FUSION_ADAPTER_WAIT,FUSION_ADAPTER_LONG_WINDOW);}
 next=*s;next.latest=current;next.updates=m->pose.updates;next.ready=0U;
 if(dt<20U){*s=next;return result(r,FUSION_ADAPTER_WAIT,FUSION_ADAPTER_SHORT_WINDOW);}
 vx=(m->raw_x_mm-s->anchor_raw_x_mm)*(1000.0f/(float)dt);vy=(m->raw_y_mm-s->anchor_raw_y_mm)*(1000.0f/(float)dt);
 omega=wrap(m->pose.heading_deg-s->anchor_heading_deg)*(1000.0f/(float)dt);
 if(!finite_f(vx)||!finite_f(vy)||!finite_f(omega)||abs_f(vx)>10000.0f||abs_f(vy)>10000.0f||abs_f(omega)>3600.0f)return result(r,FUSION_ADAPTER_UNAVAILABLE,FUSION_ADAPTER_NUMERIC);
 o->identity=p->input.identity;o->position.x_mm=m->pose.x_mm;o->position.y_mm=m->pose.y_mm;o->heading_deg=m->pose.heading_deg;
 o->vx_mm_s=vx;o->vy_mm_s=vy;o->omega_deg_s=omega;o->sample_ms=m->pose.sample_ms;o->now_ms=in->now_ms;o->valid=1U;o->feedback_source=FUSION_ESTIMATED_FEEDBACK;o->stopped=stop;
 next.anchor_ms=m->pose.sample_ms;next.anchor_raw_x_mm=m->raw_x_mm;next.anchor_raw_y_mm=m->raw_y_mm;next.anchor_heading_deg=m->pose.heading_deg;
 next.ready=1U;next.window_ms=dt;next.cached=*o;next.stop_sample_ms=in->stop_sample_ms;next.stop_run_id=in->stop_run_id;*s=next;
 r->estimated_stationary=stationary(c,o);return result(r,FUSION_ADAPTER_READY,0U);
bad_map:
 FusionAdapter_Reset(s);return result(r,FUSION_ADAPTER_UNAVAILABLE,FUSION_ADAPTER_MAP_INVALID);
}
