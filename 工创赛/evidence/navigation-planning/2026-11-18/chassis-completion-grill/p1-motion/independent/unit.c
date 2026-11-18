#include "fusion_motion.h"
#include <string.h>
typedef struct {
    FusionMotionVector q,a,target;
    FusionMotionLimits limits;
    FusionMotionRequest request;
    uint32_t mode,dt,sample_time;
} ReviewInput;
typedef struct {
    uint32_t decision, sampled, inverse, encoded;
    FusionMotionCurve curve;
    FusionMotionSample sample;
    ChassisWheelSpeeds wheels;
    float scale;
    ChassisMotorCommand commands[4];
    FusionMotionProposal proposal;
    FusionMotionReservation reservation;
    float extrema[4];
    uint32_t first_overspeed_ms;
} ReviewOutput;
ReviewInput review_input;
ReviewOutput review_output;
int test_entry(void)
{
    ReviewInput *i=&review_input;
    ReviewOutput *o=&review_output;
    ChassisBodyVelocity b;
    uint32_t t;
    float val;
    memset(o,0,sizeof(*o));
    if (i->mode==1u) {
        o->decision=FusionMotion_Propose(i->q,i->a,i->target,&i->request,&i->limits,&o->proposal);
        return 0;
    }
    if (i->mode==2u) {
        o->decision=FusionMotion_Reserve(i->q,i->a,&i->request,&i->limits,&o->reservation);
        return 0;
    }
    o->decision=FusionMotion_Prepare(i->q,i->a,i->target,i->dt,&i->limits,&o->curve);
    if (!o->decision) return 0;
    o->sampled=FusionMotion_Sample(&o->curve,i->sample_time,&o->sample);
    if (!o->sampled) return 0;
    b.vx_mm_s=o->sample.q.x;
    b.vy_mm_s=o->sample.q.y;
    b.wz_rad_s=o->sample.q.yaw;
    o->inverse=Chassis_Inverse(&g_chassis_geometry,&b,i->limits.wheel_rpm,&o->wheels,&o->scale);
    o->encoded=Chassis_EncodeEmm(&o->wheels,300,o->commands);
    if (i->mode!=3u) return 0;
    for (t=0;t<=o->curve.duration_ms;t++) {
        FusionMotionSample s;
        FusionMotion_Sample(&o->curve,t,&s);
        val=s.q.x>0?s.q.x:-s.q.x;
        if(val>o->extrema[0]) o->extrema[0]=val;
        if(val>i->limits.component_mm_s&&!o->first_overspeed_ms)
            o->first_overspeed_ms=t;
        val=s.a.x>0?s.a.x:-s.a.x;
        if(val>o->extrema[1]) o->extrema[1]=val;
        val=s.j.x>0?s.j.x:-s.j.x;
        if(val>o->extrema[2]) o->extrema[2]=val;
    }
    return 0;
}

