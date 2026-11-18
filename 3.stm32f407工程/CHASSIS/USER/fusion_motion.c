#include "fusion_motion.h"
#include <float.h>
#include <math.h>
#include <string.h>

#define MOTION_GRID_MS 20U
#define MOTION_MAX_MS 8000U
#define MOTION_MAX_DT_MS 200U
#define MOTION_PI 3.14159265358979323846f

const FusionMotionLimits g_fusion_motion_limits = {
    400.0f,150.0f,300.0f,MOTION_PI/2.0f,MOTION_PI,8.0f*MOTION_PI,
    300.0f,250.0f,1400.0f
};

typedef struct { ChassisWheelSpeeds x,y,yaw; } MotionWheelMap;

static uint32_t finite(float v)
{
    return v>=-FLT_MAX && v<=FLT_MAX;
}

static uint32_t vector_valid(FusionMotionVector v)
{
    return finite(v.x) && finite(v.y) && finite(v.yaw);
}

static uint32_t in_range(float v,float upper)
{
    return v>0.0f && v<=upper;
}

static uint32_t limits_valid(const FusionMotionLimits *l)
{
    const FusionMotionLimits *cap=&g_fusion_motion_limits;
    return l && in_range(l->component_mm_s,cap->component_mm_s) &&
        in_range(l->translation_a,cap->translation_a) && in_range(l->translation_j,cap->translation_j) &&
        in_range(l->yaw_v,cap->yaw_v) && in_range(l->yaw_a,cap->yaw_a) && in_range(l->yaw_j,cap->yaw_j) &&
        in_range(l->wheel_rpm,cap->wheel_rpm) && in_range(l->wheel_a,cap->wheel_a) &&
        in_range(l->wheel_j,cap->wheel_j);
}

static float norm_xy(FusionMotionVector v)
{
    return sqrtf(v.x*v.x+v.y*v.y);
}

static FusionMotionVector difference(FusionMotionVector a,FusionMotionVector b,float multiplier)
{
    FusionMotionVector out={(a.x-b.x)*multiplier,(a.y-b.y)*multiplier,(a.yaw-b.yaw)*multiplier};
    return out;
}

static FusionMotionVector interpolate(FusionMotionVector a,FusionMotionVector b,float t)
{
    FusionMotionVector out={(1.0f-t)*a.x+t*b.x,(1.0f-t)*a.y+t*b.y,(1.0f-t)*a.yaw+t*b.yaw};
    return out;
}

static uint32_t wheel_map(MotionWheelMap *map)
{
    const ChassisBodyVelocity x={1.0f,0.0f,0.0f},y={0.0f,1.0f,0.0f},yaw={0.0f,0.0f,1.0f};
    return Chassis_Inverse(&g_chassis_geometry,&x,FLT_MAX,&map->x,0) &&
        Chassis_Inverse(&g_chassis_geometry,&y,FLT_MAX,&map->y,0) &&
        Chassis_Inverse(&g_chassis_geometry,&yaw,FLT_MAX,&map->yaw,0);
}

static float wheel_component(FusionMotionVector v,const MotionWheelMap *map,uint32_t i)
{
    return v.x*map->x.rpm[i]+v.y*map->y.rpm[i]+v.yaw*map->yaw.rpm[i];
}

static float wheel_peak(FusionMotionVector v,const MotionWheelMap *map)
{
    float peak=0.0f;
    uint32_t i;
    for(i=0U;i<CHASSIS_WHEEL_COUNT;i++) peak=fmaxf(peak,fabsf(wheel_component(v,map,i)));
    return peak;
}

static uint32_t speed_valid(FusionMotionVector v,const MotionWheelMap *map,const FusionMotionLimits *l)
{
    return fabsf(v.x)<=l->component_mm_s && fabsf(v.y)<=l->component_mm_s &&
        fabsf(v.yaw)<=l->yaw_v && wheel_peak(v,map)<=l->wheel_rpm;
}

static uint32_t wheel_speeds(FusionMotionVector v,ChassisWheelSpeeds *wheels)
{
    const ChassisBodyVelocity body={v.x,v.y,v.yaw};
    return Chassis_Inverse(&g_chassis_geometry,&body,FLT_MAX,wheels,0);
}

static uint32_t outward(float speed,float acceleration,float cap)
{
    return (speed>=cap && acceleration>0.0f) || (speed<=-cap && acceleration<0.0f);
}

static uint32_t endpoints_valid(FusionMotionVector q,FusionMotionVector a,FusionMotionVector target,
    const FusionMotionLimits *l)
{
    ChassisWheelSpeeds start,end,rate;
    uint32_t i;
    if(fabsf(q.x)>l->component_mm_s || fabsf(q.y)>l->component_mm_s || fabsf(q.yaw)>l->yaw_v ||
       fabsf(target.x)>l->component_mm_s || fabsf(target.y)>l->component_mm_s || fabsf(target.yaw)>l->yaw_v ||
       outward(q.x,a.x,l->component_mm_s) || outward(q.y,a.y,l->component_mm_s) ||
       outward(q.yaw,a.yaw,l->yaw_v) || !wheel_speeds(q,&start) || !wheel_speeds(target,&end) ||
       !wheel_speeds(a,&rate)) return 0U;
    for(i=0U;i<CHASSIS_WHEEL_COUNT;i++) {
        if(fabsf(start.rpm[i])>l->wheel_rpm || fabsf(end.rpm[i])>l->wheel_rpm ||
           outward(start.rpm[i],rate.rpm[i],l->wheel_rpm)) return 0U;
    }
    return 1U;
}

static uint32_t derivative_valid(FusionMotionVector v,const MotionWheelMap *map,float xy,float yaw,float wheel)
{
    /* Allow single-precision roundoff at a mathematical derivative limit. */
    const float tolerance=1.000002f;
    return norm_xy(v)<=xy*tolerance && fabsf(v.yaw)<=yaw*tolerance &&
        wheel_peak(v,map)<=wheel*tolerance;
}

static void split(const FusionMotionVector p[4],float t,FusionMotionVector left[4],FusionMotionVector right[4])
{
    FusionMotionVector a=interpolate(p[0],p[1],t),b=interpolate(p[1],p[2],t),c=interpolate(p[2],p[3],t);
    left[0]=p[0];left[1]=a;left[2]=interpolate(a,b,t);
    right[3]=p[3];right[2]=c;right[1]=interpolate(b,c,t);
    left[3]=right[0]=interpolate(left[2],right[1],t);
}

static uint32_t root_within(const float p[4],float t,float cap)
{
    float s,a,b,c;
    if(!(t>0.0f && t<1.0f)) return 1U;
    s=1.0f-t;
    a=s*p[0]+t*p[1];b=s*p[1]+t*p[2];c=s*p[2]+t*p[3];
    return fabsf(s*(s*a+t*b)+t*(s*b+t*c))<=cap;
}

static uint32_t scalar_speed_valid(const float p[4],float cap)
{
    float d0,d1,d2,a,b,disc,k;
    if(fabsf(p[1])<=cap && fabsf(p[2])<=cap) return 1U;
    d0=p[1]-p[0];d1=p[2]-p[1];d2=p[3]-p[2];
    a=d0-2.0f*d1+d2;b=2.0f*(d1-d0);
    if(a==0.0f) return b==0.0f || root_within(p,-d0/b,cap);
    disc=b*b-4.0f*a*d0;
    if(disc<0.0f) return 1U;
    k=-0.5f*(b+copysignf(sqrtf(disc),b));
    if(k==0.0f) return root_within(p,-b/(2.0f*a),cap);
    return root_within(p,k/a,cap) && root_within(p,d0/k,cap);
}

static uint32_t speed_controls_valid(const FusionMotionVector p[4],const MotionWheelMap *map,const FusionMotionLimits *l)
{
    float scalar[4];
    uint32_t i,j;
    if(speed_valid(p[1],map,l) && speed_valid(p[2],map,l)) return 1U;
    for(i=0U;i<4U;i++) scalar[i]=p[i].x;
    if(!scalar_speed_valid(scalar,l->component_mm_s)) return 0U;
    for(i=0U;i<4U;i++) scalar[i]=p[i].y;
    if(!scalar_speed_valid(scalar,l->component_mm_s)) return 0U;
    for(i=0U;i<4U;i++) scalar[i]=p[i].yaw;
    if(!scalar_speed_valid(scalar,l->yaw_v)) return 0U;
    for(j=0U;j<CHASSIS_WHEEL_COUNT;j++) {
        for(i=0U;i<4U;i++) scalar[i]=wheel_component(p[i],map,j);
        if(!scalar_speed_valid(scalar,l->wheel_rpm)) return 0U;
    }
    return 1U;
}

static void acceleration_controls(FusionMotionVector q,FusionMotionVector a,FusionMotionVector target,
    float t,FusionMotionVector d[3])
{
    const FusionMotionVector zero={0.0f,0.0f,0.0f};
    d[0]=a;
    d[1]=difference(difference(target,q,3.0f/t),a,1.0f);
    d[2]=zero;
}

static void split_acceleration(const FusionMotionVector d[3],FusionMotionVector left[3],FusionMotionVector right[3])
{
    left[0]=d[0];left[1]=interpolate(d[0],d[1],0.5f);
    right[2]=d[2];right[1]=interpolate(d[1],d[2],0.5f);
    left[2]=right[0]=interpolate(left[1],right[1],0.5f);
}

static uint32_t derivatives_valid(const FusionMotionVector d[3],const MotionWheelMap *map,
    const FusionMotionLimits *l,float t)
{
    FusionMotionVector halves[2][3],left[3],right[3];
    uint32_t i,j;
    if(!derivative_valid(difference(d[1],d[0],2.0f/t),map,l->translation_j,l->yaw_j,l->wheel_j) ||
       !derivative_valid(difference(d[2],d[1],2.0f/t),map,l->translation_j,l->yaw_j,l->wheel_j)) return 0U;
    split_acceleration(d,halves[0],halves[1]);
    for(i=0U;i<2U;i++) {
        split_acceleration(halves[i],left,right);
        for(j=0U;j<3U;j++) {
            if(!derivative_valid(left[j],map,l->translation_a,l->yaw_a,l->wheel_a) ||
               !derivative_valid(right[j],map,l->translation_a,l->yaw_a,l->wheel_a)) return 0U;
        }
    }
    return 1U;
}

static void add_bounds(const FusionMotionVector p[4],float dt,FusionMotionCurve *curve)
{
    uint32_t i;
    /* Each of the four Bernstein basis functions has integral dt/4. */
    for(i=0U;i<4U;i++) {
        curve->distance_mm+=dt*0.25f*norm_xy(p[i]);
        curve->rotation_rad+=dt*0.25f*fabsf(p[i].yaw);
    }
}

uint32_t FusionMotion_Prepare(FusionMotionVector q,FusionMotionVector a,FusionMotionVector target,
    uint32_t dt_ms,const FusionMotionLimits *l,FusionMotionCurve *curve)
{
    MotionWheelMap map;
    FusionMotionVector p[4],halves[2][4],left[4],right[4],d[3],delta;
    float lower_ms,t,h;
    uint32_t duration,i;
    if(!curve) return 0U;
    memset(curve,0,sizeof(*curve));
    if(!limits_valid(l) || !vector_valid(q) || !vector_valid(a) || !vector_valid(target) ||
       !dt_ms || dt_ms>MOTION_MAX_DT_MS || !wheel_map(&map) ||
       !endpoints_valid(q,a,target,l) ||
       !derivative_valid(a,&map,l->translation_a,l->yaw_a,l->wheel_a)) return 0U;
    delta=difference(target,q,1.0f);
    lower_ms=1000.0f*fmaxf(fmaxf(norm_xy(delta)/l->translation_a,fabsf(delta.yaw)/l->yaw_a),
        wheel_peak(delta,&map)/l->wheel_a);
    if(lower_ms>(float)MOTION_MAX_MS+0.001f) return 0U;
    duration=MOTION_GRID_MS*(uint32_t)ceilf(fmaxf((float)dt_ms,lower_ms-0.001f)/(float)MOTION_GRID_MS);
    p[0]=q;p[2]=p[3]=target;
    for(;duration<=MOTION_MAX_MS;duration+=MOTION_GRID_MS) {
        t=0.001f*(float)duration;h=t*0.25f;
        p[1].x=q.x+t/3.0f*a.x;p[1].y=q.y+t/3.0f*a.y;p[1].yaw=q.yaw+t/3.0f*a.yaw;
        if(!speed_controls_valid(p,&map,l)) continue;
        acceleration_controls(q,a,target,t,d);
        if(!derivatives_valid(d,&map,l,t)) continue;
        split(p,0.5f,halves[0],halves[1]);
        for(i=0U;i<2U;i++) {
            split(halves[i],0.5f,left,right);
            add_bounds(left,h,curve);add_bounds(right,h,curve);
        }
        memcpy(curve->control,p,sizeof(p));
        curve->initial_a=a;curve->duration_ms=duration;
        return 1U;
    }
    return 0U;
}

uint32_t FusionMotion_Sample(const FusionMotionCurve *curve,uint32_t elapsed_ms,FusionMotionSample *sample)
{
    const FusionMotionVector *p;
    FusionMotionVector d[3],j[2],delta;
    float s,t,u,blend,initial;
    if(!sample) return 0U;
    memset(sample,0,sizeof(*sample));
    if(!curve || !curve->duration_ms || elapsed_ms>curve->duration_ms) return 0U;
    p=curve->control;t=0.001f*(float)curve->duration_ms;s=(float)elapsed_ms/(float)curve->duration_ms;
    acceleration_controls(p[0],curve->initial_a,p[3],t,d);
    j[0]=difference(d[1],d[0],2.0f/t);j[1]=difference(d[2],d[1],2.0f/t);
    u=1.0f-s;
    initial=t*s*u*u;
    if(s<=0.5f) {
        blend=s*s*(3.0f-2.0f*s);
        delta=difference(p[3],p[0],blend);
        sample->q=p[0];
    } else {
        blend=u*u*(3.0f-2.0f*u);
        delta=difference(p[0],p[3],blend);
        sample->q=p[3];
    }
    sample->q.x+=delta.x+initial*curve->initial_a.x;
    sample->q.y+=delta.y+initial*curve->initial_a.y;
    sample->q.yaw+=delta.yaw+initial*curve->initial_a.yaw;
    sample->a=interpolate(interpolate(d[0],d[1],s),interpolate(d[1],d[2],s),s);
    sample->j=interpolate(j[0],j[1],s);
    return 1U;
}

static uint32_t nonnegative(float v)
{
    return finite(v) && v>=0.0f;
}

static uint32_t request_valid(const FusionMotionRequest *r)
{
    float norm;
    if(!r || !r->dt_ms || r->dt_ms>MOTION_MAX_DT_MS || r->max_age_ms>=0x80000000U ||
       r->turn_sign<-1 || r->turn_sign>1 || r->mixed_lost>1U ||
       !finite(r->tangent_x) || !finite(r->tangent_y)) return 0U;
    norm=r->tangent_x*r->tangent_x+r->tangent_y*r->tangent_y;
    return fabsf(norm-1.0f)<=0.000001f && nonnegative(r->remaining_mm) && nonnegative(r->back_mm) &&
        nonnegative(r->lateral_mm) && nonnegative(r->angle_rad) && nonnegative(r->feedback_xy_mm_s) &&
        nonnegative(r->feedback_yaw_rad_s) && nonnegative(r->mixed_mm) && nonnegative(r->mixed_rad);
}

static float forward_of(FusionMotionVector v,const FusionMotionRequest *r)
{
    return v.x*r->tangent_x+v.y*r->tangent_y;
}

static float lateral_of(FusionMotionVector v,const FusionMotionRequest *r)
{
    return v.y*r->tangent_x-v.x*r->tangent_y;
}

static uint32_t turn_valid(const FusionMotionVector *p,uint32_t n,int32_t sign)
{
    uint32_t i;
    if(!sign) return 1U;
    for(i=0U;i<n;i++) if((float)sign*p[i].yaw < -1e-9f) return 0U;
    return 1U;
}

static FusionMotionReason reserve(FusionMotionVector q,FusionMotionVector a,
    const FusionMotionRequest *r,const FusionMotionLimits *limits,FusionMotionReservation *out)
{
    const FusionMotionVector zero={0.0f,0.0f,0.0f};
    uint32_t i,age=0U;
    uint64_t held_ms;
    float t,held,turn,mm_error,arc,curve_lat=0.0f,curve_back=0.0f;
    if(!request_valid(r) || !limits_valid(limits) || !vector_valid(q) || !vector_valid(a))
        return FUSION_MOTION_INPUT;
    for(i=0U;i<5U;i++) {
        uint32_t one_age=r->now_ms-r->source_ms[i];
        if(one_age>r->max_age_ms) return FUSION_MOTION_STALE;
        if(one_age>age) age=one_age;
    }
    if(r->mixed_lost) return FUSION_MOTION_PARTIAL;
    held_ms=(uint64_t)age+r->compute_ms+r->queue_ms+r->batch_ms;
    if(held_ms>=0x80000000U) return FUSION_MOTION_INPUT;
    if(!FusionMotion_Prepare(q,a,zero,r->dt_ms,limits,&out->curve)) return FUSION_MOTION_NO_BRAKE;
    if(!turn_valid(out->curve.control,2U,r->turn_sign)) return FUSION_MOTION_REVERSE;
    out->held_ms=(uint32_t)held_ms;
    held=0.001f*(float)out->held_ms;t=0.001f*(float)out->curve.duration_ms;
    turn=fminf(2.0f,held*(fabsf(q.yaw)+r->feedback_yaw_rad_s)+out->curve.rotation_rad+
        r->feedback_yaw_rad_s*t+r->mixed_rad);
    mm_error=r->feedback_xy_mm_s*(held+t)+r->mixed_mm;
    arc=out->curve.distance_mm+held*norm_xy(q);
    for(i=0U;i<4U;i++) {
        curve_lat+=fabsf(lateral_of(out->curve.control[i],r));
        curve_back+=fmaxf(0.0f,-forward_of(out->curve.control[i],r));
    }
    out->distance_mm=arc+mm_error;
    out->back_mm=t*0.25f*curve_back+held*fmaxf(0.0f,-forward_of(q,r))+arc*turn+mm_error;
    out->lateral_mm=t*0.25f*curve_lat+held*fabsf(lateral_of(q,r))+arc*turn+mm_error;
    out->rotation_rad=out->curve.rotation_rad+held*fabsf(q.yaw)+
        r->feedback_yaw_rad_s*(held+t)+r->mixed_rad;
    if(out->distance_mm>r->remaining_mm) return FUSION_MOTION_DISTANCE;
    if(out->back_mm>r->back_mm) return FUSION_MOTION_BACK;
    if(out->lateral_mm>r->lateral_mm) return FUSION_MOTION_LATERAL;
    if(out->rotation_rad>r->angle_rad) return FUSION_MOTION_ANGLE;
    return FUSION_MOTION_OK;
}

FusionMotionReason FusionMotion_Reserve(FusionMotionVector q,FusionMotionVector a,
    const FusionMotionRequest *request,const FusionMotionLimits *limits,FusionMotionReservation *out)
{
    if(!out) return FUSION_MOTION_INPUT;
    memset(out,0,sizeof(*out));
    out->reason=reserve(q,a,request,limits,out);
    return out->reason;
}

FusionMotionDecision FusionMotion_Propose(FusionMotionVector q,FusionMotionVector a,FusionMotionVector target,
    const FusionMotionRequest *r,const FusionMotionLimits *l,FusionMotionProposal *out)
{
    FusionMotionRequest refined;
    FusionMotionVector prefix[4],right[4];
    float dt,rotation=0.0f,back=0.0f,lateral=0.0f,turn,prefix_margin;
    uint32_t i;
    if(!out) return FUSION_MOTION_FAULT;
    memset(out,0,sizeof(*out));
    out->reason=FusionMotion_Reserve(q,a,r,l,&out->before);
    if(out->reason!=FUSION_MOTION_OK) return FUSION_MOTION_FAULT;
    if(!FusionMotion_Prepare(q,a,target,r->dt_ms,l,&out->candidate)) {
        out->reason=FUSION_MOTION_NO_CANDIDATE;return FUSION_MOTION_BRAKE;
    }
    if(!turn_valid(out->candidate.control,4U,r->turn_sign)) {
        out->reason=FUSION_MOTION_REVERSE;return FUSION_MOTION_BRAKE;
    }
    (void)FusionMotion_Sample(&out->candidate,r->dt_ms,&out->prefix);
    split(out->candidate.control,(float)r->dt_ms/(float)out->candidate.duration_ms,prefix,right);
    dt=0.001f*(float)r->dt_ms;
    for(i=0U;i<4U;i++) {
        out->prefix_mm+=dt*0.25f*norm_xy(prefix[i]);
        rotation+=dt*0.25f*fabsf(prefix[i].yaw);
        back+=dt*0.25f*fmaxf(0.0f,-forward_of(prefix[i],r));
        lateral+=dt*0.25f*fabsf(lateral_of(prefix[i],r));
    }
    rotation+=dt*r->feedback_yaw_rad_s;
    turn=fminf(2.0f,rotation);
    prefix_margin=out->prefix_mm*turn+dt*r->feedback_xy_mm_s;
    out->prefix_mm+=dt*r->feedback_xy_mm_s;
    refined=*r;
    refined.remaining_mm=fmaxf(0.0f,r->remaining_mm-out->prefix_mm);
    refined.angle_rad=fmaxf(0.0f,r->angle_rad-rotation);
    out->reason=FusionMotion_Reserve(out->prefix.q,out->prefix.a,&refined,l,&out->after);
    if(out->reason!=FUSION_MOTION_OK) return FUSION_MOTION_BRAKE;
    prefix_margin+=out->after.distance_mm*turn;
    if(out->prefix_mm+out->after.distance_mm>r->remaining_mm ||
       back+out->after.back_mm+prefix_margin>r->back_mm ||
       lateral+out->after.lateral_mm+prefix_margin>r->lateral_mm ||
       rotation+out->after.rotation_rad>r->angle_rad) {
        out->reason=FUSION_MOTION_PREFIX_SPACE;return FUSION_MOTION_BRAKE;
    }
    return FUSION_MOTION_ADVANCE;
}
