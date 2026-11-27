#include "fusion_reference.h"
#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>
#define F_PI 3.14159265358979323846f
static uint32_t finite_f(float x){return x>=-FLT_MAX && x<=FLT_MAX;}
static float abs_f(float x){return x<0.0f?-x:x;}
static float min_f(float a,float b){return a<b?a:b;}
static float max_f(float a,float b){return a>b?a:b;}
static float clamp(float x,float a,float b){return min_f(b,max_f(a,x));}
static float norm(float x,float y){return sqrtf(x*x+y*y);}
static float wrap(float h){float x=fmodf(h+180.0f,360.0f);if(x<0.0f)x+=360.0f;return x-180.0f;}
static uint32_t identity(const FusionIdentity *a,const FusionIdentity *b)
{return a->run_id==b->run_id && a->start_id==b->start_id && a->map_version==b->map_version && a->layout_id==b->layout_id && a->path_id==b->path_id && a->direction==b->direction;}
static uint32_t point_ok(FusionPoint p){return finite_f(p.x_mm)&&finite_f(p.y_mm)&&p.x_mm>=0.0f&&p.x_mm<=2400.0f&&p.y_mm>=0.0f&&p.y_mm<=2400.0f;}
static uint32_t box_ok(FusionBox b){return finite_f(b.min_x)&&finite_f(b.min_y)&&finite_f(b.max_x)&&finite_f(b.max_y)&&b.min_x>=0.0f&&b.min_y>=0.0f&&b.max_x<=2400.0f&&b.max_y<=2400.0f&&b.min_x<=b.max_x&&b.min_y<=b.max_y;}
static uint32_t range(float x,float lo,float hi){return finite_f(x)&&x>=lo&&x<=hi;}
static uint32_t config_ok(const FusionConfig *c)
{
 return range(c->body_length_mm,1,500)&&range(c->body_width_mm,1,500)&&
 range(c->position_budget_mm,0,100)&&range(c->heading_budget_deg,0.01f,5)&&
 range(c->position_tolerance_mm,0.01f,10)&&range(c->max_error_mm,c->position_tolerance_mm,100)&&
 range(c->stop_speed_mm_s,0,5)&&range(c->stop_omega_deg_s,0,2)&&
 range(c->start_heading_deg,-3600,3600)&&range(c->goal_heading_deg,-3600,3600)&&
 range(c->acceleration_hint_mm_s2,0.01f,3000)&&range(c->cross_gain_s,0,10)&&
 range(c->lateral_cap_mm_s,0,100)&&range(c->preview_s,0.01f,1)&&
 range(c->wheel_limit_rpm,0.01f,1000)&&range(c->heading_reserve_rpm,0,c->wheel_limit_rpm)&&
 c->heading_reserve_rpm<c->wheel_limit_rpm&&range(c->speed_state_margin_mm_s,0,50)&&
 c->max_age_ms>0U&&c->max_age_ms<=1000U&&
 range(c->geometry.wheel_diameter_mm,1,1000)&&range(c->geometry.track_width_mm,1,1000)&&range(c->geometry.wheelbase_mm,1,1000);
}
/* Exact support extrema over the heading interval, then conservative AABB sweep.
 * This may reject feasible paths. It cannot certify a changing-heading translation. */
static void extent_at(float h,float l,float w,float *ex,float *ey)
{float r=h*F_PI/180.0f,c=abs_f(cosf(r)),s=abs_f(sinf(r));*ex=max_f(*ex,c*l+s*w);*ey=max_f(*ey,s*l+c*w);}
static void extents(const FusionConfig *c,float heading,float *ex,float *ey)
{
 float lo=heading-c->heading_budget_deg,hi=heading+c->heading_budget_deg;
 float l=c->body_length_mm*.5f,w=c->body_width_mm*.5f,a=atan2f(w,l)*180.0f/F_PI,b=atan2f(l,w)*180.0f/F_PI;
 float candidates[4];int32_t k;uint32_t j;*ex=*ey=0.0f;
 extent_at(lo,l,w,ex,ey);extent_at(hi,l,w,ex,ey);
 for(k=-22;k<=22;k++){
  candidates[0]=a+180.0f*k;candidates[1]=-a+180.0f*k;candidates[2]=b+180.0f*k;candidates[3]=-b+180.0f*k;
  for(j=0;j<4U;j++)if(candidates[j]>=lo&&candidates[j]<=hi)extent_at(candidates[j],l,w,ex,ey);
 }
 *ex+=c->position_budget_mm;*ey+=c->position_budget_mm;
}
static uint32_t hits_box(FusionPoint a,FusionPoint b,FusionBox r,float ex,float ey)
{
 float lo=0.0f,hi=1.0f,p[2],v[2],mn[2],mx[2],u,w,z;uint32_t i;
 p[0]=a.x_mm;p[1]=a.y_mm;v[0]=b.x_mm-a.x_mm;v[1]=b.y_mm-a.y_mm;
 mn[0]=r.min_x-ex;mn[1]=r.min_y-ey;mx[0]=r.max_x+ex;mx[1]=r.max_y+ey;
 for(i=0;i<2U;i++){
  if(abs_f(v[i])<.000001f){if(p[i]<mn[i]||p[i]>mx[i])return 0U;}
  else{u=(mn[i]-p[i])/v[i];w=(mx[i]-p[i])/v[i];if(u>w){z=u;u=w;w=z;}lo=max_f(lo,u);hi=min_f(hi,w);if(lo>hi)return 0U;}
 }
 return 1U;
}
static uint32_t clear_extents(const FusionInput *in,FusionPoint a,FusionPoint b,float ex,float ey)
{
 static const FusionBox yellow[4]={{550,550,1000,1000},{550,1400,1000,1850},{1400,550,1850,1000},{1400,1400,1850,1850}};
 uint32_t i;
 if(min_f(a.x_mm,b.x_mm)-ex<0||min_f(a.y_mm,b.y_mm)-ey<0||max_f(a.x_mm,b.x_mm)+ex>2400||max_f(a.y_mm,b.y_mm)+ey>2400)return 0U;
 for(i=0;i<4U;i++)if(hits_box(a,b,yellow[i],ex,ey))return 0U;
 for(i=0;i<in->box_count;i++)if(hits_box(a,b,in->boxes[i],ex,ey))return 0U;
 return 1U;
}
static uint32_t clear_line(const FusionInput *in,FusionPoint a,FusionPoint b,float h)
{float ex,ey;extents(&in->config,h,&ex,&ey);return clear_extents(in,a,b,ex,ey);}
/* Conservative complete rotation disk enclosed by a box. CW/CCW may both reject;
 * no claim of precise directional rotation sweep or measured transport outline. */
static uint32_t clear_turn(const FusionInput *in,FusionPoint p)
{float r=.5f*norm(in->config.body_length_mm,in->config.body_width_mm)+in->config.position_budget_mm;return clear_extents(in,p,p,r,r);}
static uint32_t collinear(const FusionInput *in,uint32_t i)
{
 float ax=in->points[i+1U].x_mm-in->points[i].x_mm,ay=in->points[i+1U].y_mm-in->points[i].y_mm;
 float bx=in->points[i+2U].x_mm-in->points[i+1U].x_mm,by=in->points[i+2U].y_mm-in->points[i+1U].y_mm;
 return ax*bx+ay*by>0.0f&&abs_f(ax*by-ay*bx)<=.000001f*norm(ax,ay)*norm(bx,by);
}
static uint32_t hash_bytes(const void *value,uint32_t size)
{const unsigned char *p=(const unsigned char *)value;uint32_t i,h=2166136261U;for(i=0;i<size;i++){h^=p[i];h*=16777619U;}return h?h:1U;}
static uint32_t signature(const FusionInput *in){return hash_bytes(in,sizeof(*in));}
static uint32_t observation_signature(const FusionObservation *o)
{FusionObservation copy=*o;copy.now_ms=0U;return hash_bytes(&copy,sizeof(copy));}
uint32_t FusionReference_Prepare(FusionPlan *p,const FusionInput *in,FusionCheck *check)
{
 uint32_t i,n;float d,h,exit;FusionCheck local;
 if(!check)check=&local;check->reason=FUSION_BAD_PLAN;check->edge=0U;
 if(!p)return 0U;
 if(!in||in==&p->input){p->valid=0U;return 0U;}
 memset(p,0,sizeof(*p));
 if(in->model_only!=1U||in->count<2U||in->count>FUSION_POINTS||in->box_count>FUSION_BOXES||
 !in->identity.run_id||(in->identity.start_id!=1U&&in->identity.start_id!=2U)||!in->identity.map_version||!in->identity.layout_id||!in->identity.path_id||in->identity.direction>1U||!config_ok(&in->config))return 0U;
 for(i=0;i<in->box_count;i++)if(!box_ok(in->boxes[i]))return 0U;
 for(i=0;i<in->count;i++)if(!point_ok(in->points[i]))return 0U;
 p->input=*in;n=in->count-1U;d=wrap(in->config.goal_heading_deg-in->config.start_heading_deg);h=wrap(in->config.goal_heading_deg);
 if(abs_f(d)<.001f)h=wrap(in->config.start_heading_deg);
 else if(abs_f(abs_f(d)-90.0f)<.001f){if(!clear_turn(in,in->points[0])){check->reason=FUSION_GEOMETRY;return 0U;}}
 else if(abs_f(abs_f(d)-180.0f)<.001f){
  /* Disk geometry is direction-independent; CW is the preferred feasible choice. */
  if(!clear_turn(in,in->points[0])||!clear_turn(in,in->points[n])){check->reason=FUSION_GEOMETRY;return 0U;}
  p->turn_direction=-1;h=wrap(in->config.start_heading_deg-90.0f);
 }else return 0U;
 p->hold_heading_deg=h;
 for(i=0;i<n;i++){
  check->edge=i;
  p->lengths[i]=norm(in->points[i+1].x_mm-in->points[i].x_mm,in->points[i+1].y_mm-in->points[i].y_mm);
  if(!range(p->lengths[i],.01f,4000)||!range(in->speed_mm_s[i],.01f,2000))return 0U;
  p->prefix[i+1]=p->prefix[i]+p->lengths[i];
  if(!clear_line(in,in->points[i],in->points[i+1],h)){check->reason=FUSION_GEOMETRY;return 0U;}
 }
 p->node_cap[n]=0.0f;
 for(i=n;i>0U;i--){
  exit=(i<n&&collinear(in,i-1U))?p->node_cap[i]:0.0f;
  p->node_cap[i-1U]=min_f(in->speed_mm_s[i-1U],sqrtf(exit*exit+2*in->config.acceleration_hint_mm_s2*p->lengths[i-1U]));
 }
 p->signature=signature(in);p->valid=1U;check->reason=0U;check->edge=0U;return 1U;
}
void FusionReference_Reset(FusionState *s){if(s)memset(s,0,sizeof(*s));}
static uint32_t stopped(const FusionConfig *c,const FusionObservation *o)
{return o->stopped==1U&&norm(o->vx_mm_s,o->vy_mm_s)<=c->stop_speed_mm_s&&abs_f(o->omega_deg_s)<=c->stop_omega_deg_s;}
static uint32_t reject(FusionOutput *o,uint32_t status,uint32_t reason)
{if(status==FUSION_INVALID)memset(o,0,sizeof(*o));o->world_vx_mm_s=o->world_vy_mm_s=o->speed_cap_mm_s=o->wheel_scale=0;memset(&o->body,0,sizeof(o->body));memset(&o->wheels,0,sizeof(o->wheels));o->status=status;o->reason=reason;o->model_only=1U;return status;}
uint32_t FusionReference_Step(const FusionPlan *p,FusionState *s,const FusionObservation *o,FusionOutput *out)
{
 FusionState next;const FusionInput *in;const FusionConfig *c;FusionPoint a,b,projection,preview;
 float h,error,dx,dy,len,tx,ty,nx,ny,q,e,residual,exit,cap,lat,speed,r,scale,mag;uint32_t fresh,i,n,is_stop,obs_signature;
 if(!out)return FUSION_INVALID;
 memset(out,0,sizeof(*out));out->model_only=1U;
 if(!p||!s||!o||p->valid!=1U||!p->signature||p->input.count<2U||p->input.count>FUSION_POINTS||p->input.box_count>FUSION_BOXES)return reject(out,FUSION_INVALID,FUSION_BAD_PLAN);
 in=&p->input;c=&in->config;n=in->count-1U;
 if(s->edge>=n||s->phase>3U||(s->signature&&s->signature!=p->signature))return reject(out,FUSION_INVALID,FUSION_BAD_PLAN);
 if(!identity(&in->identity,&o->identity))return reject(out,FUSION_INVALID,FUSION_IDENTITY);
 if(o->valid!=1U||o->continuity_lost||o->reasons||
 (o->feedback_source!=FUSION_ESTIMATED_FEEDBACK&&o->feedback_source!=FUSION_MEASURED_FEEDBACK)||o->stopped>1U||
 !finite_f(o->position.x_mm)||!finite_f(o->position.y_mm)||!range(o->heading_deg,-3600,3600)||
 !range(o->vx_mm_s,-10000,10000)||!range(o->vy_mm_s,-10000,10000)||!range(o->omega_deg_s,-3600,3600)||
 (int32_t)(o->now_ms-o->sample_ms)<0||o->now_ms-o->sample_ms>c->max_age_ms||
 (s->seen&&(int32_t)(o->sample_ms-s->last_sample_ms)<0))return reject(out,FUSION_INVALID,FUSION_BAD_OBSERVATION);
 obs_signature=observation_signature(o);
 if(s->seen&&o->sample_ms==s->last_sample_ms){
  if(obs_signature!=s->observation_signature)return reject(out,FUSION_INVALID,FUSION_BAD_OBSERVATION);
  *out=s->cached;return out->status;
 }
 next=*s;next.signature=p->signature;fresh=!s->seen||o->sample_ms!=s->last_sample_ms;is_stop=stopped(c,o);
 h=(s->phase>=2U)?wrap(c->goal_heading_deg):p->hold_heading_deg;error=wrap(h-o->heading_deg);
 out->heading_target_deg=h;out->heading_error_deg=error;out->edge=s->edge;
 if(!clear_line(in,o->position,o->position,o->heading_deg))return reject(out,FUSION_RECOVERY,FUSION_GEOMETRY);
 if(s->phase==0U){
  if(norm(o->position.x_mm-in->points[0].x_mm,o->position.y_mm-in->points[0].y_mm)>c->max_error_mm)return reject(out,FUSION_RECOVERY,FUSION_OUTSIDE_BUDGET);
  if(abs_f(wrap(p->hold_heading_deg-c->start_heading_deg))>.001f || abs_f(error)>c->heading_budget_deg)
   if(!clear_turn(in,o->position))return reject(out,FUSION_RECOVERY,FUSION_GEOMETRY);
  out->status=FUSION_TURN_START;
  if(abs_f(error)<=c->heading_budget_deg&&is_stop&&fresh)next.phase=1U;
 }else if(s->phase>=2U){
  if(norm(o->position.x_mm-in->points[n].x_mm,o->position.y_mm-in->points[n].y_mm)>c->position_tolerance_mm)return reject(out,FUSION_RECOVERY,FUSION_OUTSIDE_BUDGET);
  if(abs_f(wrap(c->goal_heading_deg-p->hold_heading_deg))>.001f&&!clear_turn(in,o->position))return reject(out,FUSION_RECOVERY,FUSION_GEOMETRY);
  if(s->phase==3U){if(abs_f(error)>c->heading_budget_deg||!is_stop)return reject(out,FUSION_RECOVERY,FUSION_SPEED_STATE);out->status=FUSION_DONE;}
  else{out->status=FUSION_TURN_END;if(abs_f(error)<=c->heading_budget_deg&&is_stop&&fresh){next.phase=3U;out->status=FUSION_DONE;}}
 }else{
  i=s->edge;a=in->points[i];b=in->points[i+1U];len=p->lengths[i];tx=(b.x_mm-a.x_mm)/len;ty=(b.y_mm-a.y_mm)/len;nx=-ty;ny=tx;
  dx=o->position.x_mm-a.x_mm;dy=o->position.y_mm-a.y_mm;q=dx*tx+dy*ty;e=dx*nx+dy*ny;residual=len-q;
  if(abs_f(e)>c->max_error_mm||q< -c->max_error_mm||abs_f(error)>c->heading_budget_deg)return reject(out,FUSION_RECOVERY,FUSION_OUTSIDE_BUDGET);
  /* Bounded by plan capacity, same-direction neighbors only; never nearest-path search. */
  while(i+1U<n&&collinear(in,i)&&q>=len){
   i++;a=in->points[i];b=in->points[i+1U];len=p->lengths[i];tx=(b.x_mm-a.x_mm)/len;ty=(b.y_mm-a.y_mm)/len;nx=-ty;ny=tx;
   dx=o->position.x_mm-a.x_mm;dy=o->position.y_mm-a.y_mm;q=dx*tx+dy*ty;e=dx*nx+dy*ny;residual=len-q;
  }
  next.edge=i;out->edge=i;
  out->signed_s_mm=p->prefix[i]+q;out->cross_mm=e;out->endpoint_residual_mm=residual;
  out->remaining_mm=len-min_f(q,len)+p->prefix[n]-p->prefix[i+1U];
  if(residual< -c->position_tolerance_mm)return reject(out,FUSION_RECOVERY,FUSION_OUTSIDE_BUDGET);
  exit=(i+1U<n&&collinear(in,i))?p->node_cap[i+1U]:0.0f;
  cap=min_f(in->speed_mm_s[i],sqrtf(exit*exit+2*c->acceleration_hint_mm_s2*max_f(0,residual)));
  speed=norm(o->vx_mm_s,o->vy_mm_s);
  if(o->vx_mm_s*tx+o->vy_mm_s*ty>cap+c->speed_state_margin_mm_s||o->vx_mm_s*tx+o->vy_mm_s*ty< -c->stop_speed_mm_s||abs_f(o->vx_mm_s*nx+o->vy_mm_s*ny)>c->lateral_cap_mm_s+c->speed_state_margin_mm_s||speed>2000.0f+c->speed_state_margin_mm_s)return reject(out,FUSION_RECOVERY,FUSION_SPEED_STATE);
  projection.x_mm=a.x_mm+clamp(q,0,len)*tx;projection.y_mm=a.y_mm+clamp(q,0,len)*ty;
  if(!clear_line(in,o->position,projection,o->heading_deg))return reject(out,FUSION_RECOVERY,FUSION_GEOMETRY);
  out->status=FUSION_FOLLOW;out->speed_cap_mm_s=cap;
  lat=clamp(-c->cross_gain_s*e,-c->lateral_cap_mm_s,c->lateral_cap_mm_s);
  out->world_vx_mm_s=cap*tx+lat*nx;out->world_vy_mm_s=cap*ty+lat*ny;
  mag=norm(out->world_vx_mm_s,out->world_vy_mm_s);if(mag>600){out->world_vx_mm_s*=600/mag;out->world_vy_mm_s*=600/mag;}
  r=o->heading_deg*F_PI/180.0f;
  out->body.vx_mm_s=cosf(r)*out->world_vx_mm_s+sinf(r)*out->world_vy_mm_s;
  out->body.vy_mm_s=-sinf(r)*out->world_vx_mm_s+cosf(r)*out->world_vy_mm_s;
  if(!Chassis_Inverse(&c->geometry,&out->body,c->wheel_limit_rpm-c->heading_reserve_rpm,&out->wheels,&scale))return reject(out,FUSION_INVALID,FUSION_BAD_PLAN);
  out->wheel_scale=scale;out->body.vx_mm_s*=scale;out->body.vy_mm_s*=scale;
  out->world_vx_mm_s*=scale;out->world_vy_mm_s*=scale;out->speed_cap_mm_s*=scale;
  out->mixed=abs_f(out->body.vx_mm_s)>.001f&&abs_f(out->body.vy_mm_s)>.001f;
  preview.x_mm=o->position.x_mm+c->preview_s*out->world_vx_mm_s;preview.y_mm=o->position.y_mm+c->preview_s*out->world_vy_mm_s;
  if(!clear_line(in,o->position,preview,o->heading_deg))return reject(out,FUSION_RECOVERY,FUSION_GEOMETRY);
  if(q>=len-c->position_tolerance_mm&&norm(o->position.x_mm-b.x_mm,o->position.y_mm-b.y_mm)<=c->position_tolerance_mm&&fresh){
   if(i+1U==n){if(is_stop){next.phase=2U;out->status=FUSION_TURN_END;out->heading_target_deg=wrap(c->goal_heading_deg);out->heading_error_deg=wrap(out->heading_target_deg-o->heading_deg);}}
   else if((collinear(in,i)&&q>=len)||is_stop)next.edge=i+1U;
   /* No residual desired velocity on a stopped corner or completed edge. */
   if(next.phase!=s->phase||next.edge>i){out->world_vx_mm_s=out->world_vy_mm_s=0;memset(&out->body,0,sizeof(out->body));memset(&out->wheels,0,sizeof(out->wheels));}
  }
 }
 next.seen=1U;next.last_sample_ms=o->sample_ms;next.observation_signature=obs_signature;next.cached=*out;*s=next;return out->status;
}
