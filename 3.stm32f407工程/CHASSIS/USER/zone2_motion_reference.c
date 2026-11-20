#include "zone2_motion_reference.h"
#include "vehicle_tuning.h"
#include <math.h>
#include <string.h>

#define DEG_TO_RAD 0.01745329251994329577f
#define REF_BATCH_MS FUSION_MOTION_FEEDBACK_PERIOD_MS
#define REF_BATCH_S 0.08f
#define REF_SPEED 250.0f
#define REF_ACCEL 120.0f
#define REF_JERK 240.0f
#define LONG_LINE_SPEED 400.0f
#define LONG_LINE_MIN_MM 780.0f
#define TRACK_MM 2.0f
#define EDGE_TRACK_MM 1.0f
#define EDGE_LENGTH_MM 5.0f
#define CORNER_MIN_EXIT_MM (12.0f*(REF_BATCH_S+36.0f*REF_BATCH_S/2.0f)+TRACK_MM)

enum { SPAN_LINE, SPAN_BLEND, SPAN_TURN, SPAN_S_LINE };
typedef struct { float trim,speed; uint32_t duration_ms; } Corner;

static FusionPoint add_scaled(FusionPoint p,FusionPoint d,float s)
{
    return (FusionPoint){p.x_mm+d.x_mm*s,p.y_mm+d.y_mm*s};
}

static FusionPoint delta(FusionPoint from,FusionPoint to)
{
    return (FusionPoint){to.x_mm-from.x_mm,to.y_mm-from.y_mm};
}

static float norm(FusionPoint p)
{
    return sqrtf(p.x_mm*p.x_mm+p.y_mm*p.y_mm);
}

static FusionPoint direction(const FusionPlan *p,uint32_t edge)
{
    FusionPoint d=delta(p->input.points[edge],p->input.points[edge+1U]);
    return add_scaled((FusionPoint){0,0},d,1.0f/p->lengths[edge]);
}

static float wraps_deg(float heading)
{
    float a=fmodf(heading+180.0f,360.0f);
    return a+(a<0.0f?180.0f:-180.0f);
}

static float support(float lo,float hi,float a,float b)
{
    float peak=fmaxf(a*fabsf(cosf(lo))+b*fabsf(sinf(lo)),
                     a*fabsf(cosf(hi))+b*fabsf(sinf(hi)));
    float angle=atan2f(b,a);
    int32_t k;
    for(k=-2;k<=2;k++) {
        float peak_angle=(float)k*180.0f*DEG_TO_RAD;
        if((lo<=peak_angle+angle && hi>=peak_angle+angle) ||
           (lo<=peak_angle-angle && hi>=peak_angle-angle))
            return sqrtf(a*a+b*b);
    }
    return peak;
}

static FusionPoint body_extent(const FusionConfig *c,float heading)
{
    float l=c->body_length_mm*.5f,w=c->body_width_mm*.5f;
    float lo=(heading-c->heading_budget_deg)*DEG_TO_RAD;
    float hi=(heading+c->heading_budget_deg)*DEG_TO_RAD;
    return (FusionPoint){support(lo,hi,l,w)+c->position_budget_mm,
                         support(lo,hi,w,l)+c->position_budget_mm};
}

static uint32_t box_overlap(FusionBox a,FusionBox b)
{
    return a.min_x<=b.max_x && b.min_x<=a.max_x &&
           a.min_y<=b.max_y && b.min_y<=a.max_y;
}

static uint32_t clear_hull(const FusionInput *in,const FusionPoint *p,
    uint32_t count,FusionPoint extent,float tracking)
{
    static const FusionBox yellow[4]={{550,550,1000,1000},{550,1400,1000,1850},
        {1400,550,1850,1000},{1400,1400,1850,1850}};
    FusionBox hull={p[0].x_mm,p[0].y_mm,p[0].x_mm,p[0].y_mm};
    uint32_t i;
    for(i=1U;i<count;i++) {
        hull.min_x=fminf(hull.min_x,p[i].x_mm);
        hull.min_y=fminf(hull.min_y,p[i].y_mm);
        hull.max_x=fmaxf(hull.max_x,p[i].x_mm);
        hull.max_y=fmaxf(hull.max_y,p[i].y_mm);
    }
    hull.min_x-=extent.x_mm+tracking;
    hull.min_y-=extent.y_mm+tracking;
    hull.max_x+=extent.x_mm+tracking;
    hull.max_y+=extent.y_mm+tracking;
    if(hull.min_x<=0.0f || hull.min_y<=0.0f ||
       hull.max_x>=FIELD_SIZE_MM || hull.max_y>=FIELD_SIZE_MM) return 0U;
    for(i=0U;i<4U;i++) if(box_overlap(hull,yellow[i])) return 0U;
    for(i=0U;i<in->box_count;i++) if(box_overlap(hull,in->boxes[i])) return 0U;
    return 1U;
}

static uint32_t sweep(const FusionInput *in,const FusionPoint *p,uint32_t count,
    FusionPoint extent,float tracking,uint32_t depth)
{
    FusionPoint work[6],left[6],right[6];
    uint32_t i,n;
    if(clear_hull(in,p,count,extent,tracking)) return 1U;
    if(!depth) return 0U;
    memcpy(work,p,count*sizeof(*p));
    for(n=count;n>0U;n--) {
        left[count-n]=work[0];
        right[n-1U]=work[n-1U];
        for(i=0U;i+1U<n;i++) work[i]=add_scaled(work[i],delta(work[i],work[i+1U]),.5f);
    }
    return sweep(in,left,count,extent,tracking,depth-1U) &&
           sweep(in,right,count,extent,tracking,depth-1U);
}

static uint32_t clear_line(const FusionPlan *p,uint32_t i,FusionPoint ext)
{
    FusionPoint pair[2]={p->input.points[i],p->input.points[i+1U]};
    FusionPoint unit=direction(p,i),a=pair[0],b=pair[1];
    float len=p->lengths[i];
    uint32_t last=i+2U==p->input.count;
    if(i==0U) {
        float near=fminf(EDGE_LENGTH_MM,len);
        pair[1]=add_scaled(a,unit,near);
        if(!sweep(&p->input,pair,2U,ext,EDGE_TRACK_MM,7U)) return 0U;
        a=pair[1];
        if(near==len) return 1U;
    }
    if(last) {
        float near=fminf(EDGE_LENGTH_MM,norm(delta(a,b)));
        pair[0]=add_scaled(b,unit,-near);
        pair[1]=b;
        if(!sweep(&p->input,pair,2U,ext,EDGE_TRACK_MM,7U)) return 0U;
        b=pair[0];
        if(near>=norm(delta(a,pair[1]))) return 1U;
    }
    pair[0]=a;
    pair[1]=b;
    return sweep(&p->input,pair,2U,ext,TRACK_MM,7U);
}

static uint32_t fit_line(float length,float initial,float final,float cap,Zone2MotionSpan *span)
{
    static const uint32_t ramp_counts[]={5U,8U,10U,15U,20U,25U,30U,40U};
    uint32_t best=0U,i,n;
    float lo=fmaxf(initial,final),base=fminf(initial,final);
    for(i=0U;i<sizeof(ramp_counts)/sizeof(ramp_counts[0]);i++) {
        uint32_t ramp=ramp_counts[i];
        float b=(float)ramp*REF_BATCH_S;
        for(n=0U;n<=200U;n++) {
            uint32_t total=n+2U*ramp;
            float cruise=(length-b*(initial+final)*.5f)/((float)n*REF_BATCH_S+b);
            float change=cruise-base;
            if(cruise+0.0001f<lo || (best && total>=best)) break;
            if(cruise<=cap && 1.5f*change<=REF_ACCEL*b && 6.0f*change<=REF_JERK*b*b) {
                best=total;
                span->duration_ms=best*REF_BATCH_MS;
                span->ramp_ms=ramp*REF_BATCH_MS;
                span->kind=SPAN_LINE;
                span->data.line.entry=initial;
                span->data.line.cruise=cruise;
                span->data.line.exit=final;
                break;
            }
        }
    }
    return best!=0U;
}

static float s_ramp_time(float change)
{
    const float limit=REF_ACCEL/REF_JERK;
    if(change<=REF_ACCEL*limit) return 2.0f*sqrtf(change/REF_JERK);
    return change/REF_ACCEL+limit;
}

static float s_ramp_distance(float entry,float exit,float peak,float *rise,float *fall)
{
    *rise=s_ramp_time(peak-entry);
    *fall=s_ramp_time(peak-exit);
    return .5f*((entry+peak)*(*rise)+(exit+peak)*(*fall));
}

static void fit_long_line(FusionPoint d,float length,float cap,Zone2MotionSpan *span)
{
    const float entry=span->data.line.entry,exit=span->data.line.exit;
    const float minimum=fmaxf(entry,exit);
    float lo=minimum,hi=fminf(cap,LONG_LINE_SPEED),peak,rise,fall,dist,seconds;
    uint32_t i,batches;
    if(length<LONG_LINE_MIN_MM || fminf(fabsf(d.x_mm),fabsf(d.y_mm))>1.0e-6f ||
       hi<=minimum || s_ramp_distance(entry,exit,lo,&rise,&fall)>length) return;

    /* Find the fastest feasible peak, then refit it to a whole number of batches. */
    if(s_ramp_distance(entry,exit,hi,&rise,&fall)<=length) lo=hi;
    else for(i=0U;i<24U;i++) {
        peak=.5f*(lo+hi);
        if(s_ramp_distance(entry,exit,peak,&rise,&fall)<=length) lo=peak;
        else hi=peak;
    }
    dist=s_ramp_distance(entry,exit,lo,&rise,&fall);
    if(lo<=0.0f) return;
    batches=(uint32_t)ceilf((rise+fall+(length-dist)/lo)/REF_BATCH_S);
    if(batches*REF_BATCH_MS>=span->duration_ms) return;
    seconds=(float)(batches*REF_BATCH_MS)*.001f;
    hi=lo;
    lo=minimum;
    for(i=0U;i<24U;i++) {
        peak=.5f*(lo+hi);
        dist=s_ramp_distance(entry,exit,peak,&rise,&fall);
        if(dist+peak*(seconds-rise-fall)<length) lo=peak;
        else hi=peak;
    }
    peak=.5f*(lo+hi);
    dist=s_ramp_distance(entry,exit,peak,&rise,&fall);
    if(seconds<rise+fall || fabsf(dist+peak*(seconds-rise-fall)-length)>.005f) return;
    span->kind=SPAN_S_LINE;
    span->duration_ms=batches*REF_BATCH_MS;
    span->data.line.cruise=peak;
    span->data.line.rise_s=rise;
    span->data.line.fall_s=fall;
}

static void corner_controls(const FusionPlan *p,uint32_t node,float trim,FusionPoint control[6])
{
    FusionPoint c=p->input.points[node],a=direction(p,node-1U),b=direction(p,node);
    uint32_t i;
    for(i=0U;i<3U;i++) {
        control[i]=add_scaled(c,a,-trim*(float)(3U-i)/3.0f);
        control[3U+i]=add_scaled(c,b,trim*(float)(i+1U)/3.0f);
    }
}

static float max_difference(FusionPoint *points,uint32_t count)
{
    float peak=0.0f;
    uint32_t i;
    for(i=0U;i+1U<count;i++) {
        FusionPoint d=delta(points[i],points[i+1U]);
        points[i]=add_scaled((FusionPoint){0,0},d,(float)(count-1U));
        peak=fmaxf(peak,norm(points[i]));
    }
    return peak;
}

static void select_corner(const FusionPlan *p,uint32_t node,FusionPoint ext,Corner *out)
{
    FusionPoint a=direction(p,node-1U),b=direction(p,node);
    FusionPoint control[6];
    Zone2MotionSpan scratch;
    float before=p->lengths[node-1U],after=p->lengths[node];
    float turn=atan2f(a.x_mm*b.y_mm-a.y_mm*b.x_mm,a.x_mm*b.x_mm+a.y_mm*b.y_mm);
    float trim=fminf(160.0f,.4f*fminf(before,after));
    float exit=after-trim-(node+2U<p->input.count?.4f*after:0.0f);
    float cap_a=fminf(REF_SPEED,p->input.speed_mm_s[node-1U]);
    float cap_b=fminf(REF_SPEED,p->input.speed_mm_s[node]);
    float cap=fminf(cap_a,cap_b);
    FusionPoint bisector=add_scaled(a,b,1.0f);
    float speed,accel,jerk,join,t,projection;
    uint32_t ticks,i;
    if(fabsf(turn)<1.0e-6f*DEG_TO_RAD || fabsf(turn)>150.0f*DEG_TO_RAD ||
       trim<8.0f || exit<CORNER_MIN_EXIT_MM || cap<12.0f) return;
    corner_controls(p,node,trim,control);
    if(!sweep(&p->input,control,6U,ext,TRACK_MM,7U)) return;
    speed=max_difference(control,6U);
    join=norm(control[0]);
    projection=control[0].x_mm*bisector.x_mm+control[0].y_mm*bisector.y_mm;
    for(i=1U;i<5U;i++)
        projection=fminf(projection,control[i].x_mm*bisector.x_mm+control[i].y_mm*bisector.y_mm);
    projection/=norm(bisector);
    accel=max_difference(control,5U);
    jerk=max_difference(control,4U);
    t=fmaxf(fmaxf(speed/cap,sqrtf(accel/REF_ACCEL)),
           fmaxf(powf(jerk/REF_JERK,1.0f/3.0f),join/90.0f));
    ticks=(uint32_t)ceilf(t/REF_BATCH_S);
    for(;;ticks++) {
        float v=join/((float)ticks*REF_BATCH_S);
        if(projection/((float)ticks*REF_BATCH_S)<12.0f) return;
        if(fit_line(before-trim,0.0f,v,cap_a,&scratch) &&
           fit_line(after-trim,v,0.0f,cap_b,&scratch)) {
            out->trim=trim;
            out->speed=v;
            out->duration_ms=ticks*REF_BATCH_MS;
            return;
        }
    }
}

static void push_span(Zone2MotionReference *r,const Zone2MotionSpan *span)
{
    Zone2MotionSection *section=&r->sections[r->section_count];
    r->spans[r->span_count++]=*span;
    section->span_count++;
    section->duration_ms+=span->duration_ms;
    r->duration_ms+=span->duration_ms;
}

static void finish_section(Zone2MotionReference *r)
{
    r->section_count++;
    if(r->section_count<ZONE2_MOTION_SECTION_COUNT)
        r->sections[r->section_count].first_span=r->span_count;
}

static uint32_t add_turn(const FusionPlan *p,Zone2MotionReference *r,FusionPoint point,
    float heading,float turn)
{
    Zone2MotionSpan span={0};
    const FusionConfig *c=&p->input.config;
    float radius=.5f*norm((FusionPoint){c->body_length_mm,c->body_width_mm})+c->position_budget_mm;
    if(fabsf(turn)<.001f) return 1U;
    if(fabsf(fabsf(turn)-90.0f)>=.001f ||
       !clear_hull(&p->input,&point,1U,(FusionPoint){radius,radius},TRACK_MM+.5f)) return 0U;
    span.kind=SPAN_TURN;
    span.duration_ms=2080U;
    span.data.turn.center=point;
    span.data.turn.start_rad=heading*DEG_TO_RAD;
    span.data.turn.delta_rad=turn*DEG_TO_RAD;
    push_span(r,&span);
    finish_section(r);
    return 1U;
}

uint32_t Zone2MotionReference_Prepare(const FusionPlan *p,Zone2MotionReference *r)
{
    Corner corners[FUSION_POINTS]={0};
    Zone2MotionSpan span={0};
    FusionPoint ext;
    const FusionInput *in;
    uint32_t i,n,drop;
    float start,hold,goal;
    if(!r) return 0U;
    r->section_count=0U;
    if(!p || p->valid!=1U || p->input.count<2U || p->input.count>FUSION_POINTS ||
       p->input.box_count>FUSION_BOXES) return 0U;
    in=&p->input;
    n=in->count-1U;
    start=wraps_deg(in->config.start_heading_deg);
    hold=wraps_deg(p->hold_heading_deg);
    goal=wraps_deg(in->config.goal_heading_deg);
    ext=body_extent(&in->config,hold);
    for(i=0U;i<n;i++)
        if(!(p->lengths[i]>=.01f && p->lengths[i]<=4000.0f) || !clear_line(p,i,ext)) return 0U;
    for(i=1U;i<n;i++) select_corner(p,i,ext,&corners[i]);
    for(;;) {
        memset(r,0,sizeof(*r));
        r->hold_rad=hold*DEG_TO_RAD;
        if(!add_turn(p,r,in->points[0],start,wraps_deg(hold-start))) return 0U;
        drop=0U;
        for(i=0U;i<n;i++) {
            FusionPoint u=direction(p,i);
            FusionPoint a=add_scaled(in->points[i],u,corners[i].trim);
            FusionPoint b=add_scaled(in->points[i+1U],u,-corners[i+1U].trim);
            FusionPoint d=delta(a,b);
            float length=norm(d),cap=fminf(in->speed_mm_s[i],REF_SPEED);
            if(!fit_line(length,corners[i].speed,corners[i+1U].speed,cap,&span)) {
                drop=corners[i+1U].duration_ms?i+1U:i;
                if(!corners[drop].duration_ms) {
                    r->section_count=0U;
                    return 0U;
                }
                break;
            }
            fit_long_line(d,length,in->speed_mm_s[i],&span);
            span.data.line.from=a;
            span.data.line.to=b;
            push_span(r,&span);
            if(corners[i+1U].duration_ms) {
                span.kind=SPAN_BLEND;
                span.duration_ms=corners[i+1U].duration_ms;
                corner_controls(p,i+1U,corners[i+1U].trim,span.data.control);
                push_span(r,&span);
            } else {
                finish_section(r);
                if(i+1U<n) r->duration_ms+=2U*REF_BATCH_MS;
            }
        }
        if(!drop) break;
        corners[drop]=(Corner){0};
    }
    if(!add_turn(p,r,in->points[n],hold,wraps_deg(goal-hold))) {
        r->section_count=0U;
        return 0U;
    }
    r->sections[r->section_count-1U].terminal=1U;
    return 1U;
}

static float ramp_area(float u)
{
    return u*u*u*(1.0f-.5f*u);
}

static float s_ramp_area(float time,float duration,float from,float to)
{
    const float t=fminf(fmaxf(time,0.0f),duration);
    const float b=fminf(REF_ACCEL/REF_JERK,.5f*duration);
    const float x=fmaxf(0.0f,t-b),y=fmaxf(0.0f,t-(duration-b));
    const float jerk=copysignf(REF_JERK,to-from);
    return from*t+jerk*(t*t*t-x*x*x-y*y*y)/6.0f;
}

static FusionPoint sample_span(const Zone2MotionSpan *span,float t,float *heading)
{
    float end=(float)span->duration_ms*.001f;
    float u=t/end,v=1.0f-u;
    uint32_t i;
    if(span->kind==SPAN_TURN) {
        *heading=span->data.turn.start_rad+span->data.turn.delta_rad*u*u*u*(10.0f+u*(6.0f*u-15.0f));
        return span->data.turn.center;
    }
    if(span->kind==SPAN_BLEND) {
        const float basis[6]={v*v*v*v*v,5.0f*u*v*v*v*v,10.0f*u*u*v*v*v,
            10.0f*u*u*u*v*v,5.0f*u*u*u*u*v,u*u*u*u*u};
        FusionPoint point={0};
        for(i=0U;i<6U;i++) point=add_scaled(point,span->data.control[i],basis[i]);
        return point;
    }
    if(t<=0.0f) return span->data.line.from;
    if(t>=end) return span->data.line.to;
    if(span->kind==SPAN_S_LINE) {
        const float entry=span->data.line.entry,cruise=span->data.line.cruise,exit=span->data.line.exit;
        const float rise=span->data.line.rise_s,fall=span->data.line.fall_s;
        const float cruise_s=end-rise-fall;
        FusionPoint d=delta(span->data.line.from,span->data.line.to);
        float s=s_ramp_area(t,rise,entry,cruise)+cruise*fminf(cruise_s,fmaxf(0.0f,t-rise))
            +s_ramp_area(t-rise-cruise_s,fall,cruise,exit);
        return add_scaled(span->data.line.from,d,s/norm(d));
    }
    {
        FusionPoint d=delta(span->data.line.from,span->data.line.to);
        float b=(float)span->ramp_ms*.001f,f=end-2.0f*b;
        float entry=span->data.line.entry,cruise=span->data.line.cruise,exit=span->data.line.exit;
        float x=fminf(1.0f,t/b),y=fmaxf(0.0f,fminf(1.0f,(t-b-f)/b));
        float s=b*(entry*x+(cruise-entry)*ramp_area(x)+cruise*y+(exit-cruise)*ramp_area(y))
            +cruise*fmaxf(0.0f,fminf(f,t-b));
        return add_scaled(span->data.line.from,d,s/norm(d));
    }
}

uint32_t Zone2MotionReference_Sample(const Zone2MotionReference *r,uint32_t section,
    int32_t elapsed_ms,FusionMotionVector *pose)
{
    const Zone2MotionSection *s;
    const Zone2MotionSpan *span;
    FusionPoint point;
    uint32_t i,t;
    float heading;
    if(!r || !pose || section>=r->section_count) return 0U;
    s=&r->sections[section];
    t=elapsed_ms<0?0U:(uint32_t)elapsed_ms;
    if(t>s->duration_ms) t=s->duration_ms;
    for(i=0U;i+1U<s->span_count;i++) {
        span=&r->spans[s->first_span+i];
        if(t<span->duration_ms) break;
        t-=span->duration_ms;
    }
    span=&r->spans[s->first_span+i];
    heading=r->hold_rad;
    point=sample_span(span,(float)t*.001f,&heading);
    *pose=(FusionMotionVector){point.x_mm,point.y_mm,heading};
    return 1U;
}
