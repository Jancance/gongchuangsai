#include "app_config.h"
#include "zone2_nav_schedule.h"
#include <math.h>
#include <string.h>

#if APP_ZONE2_POSE_TRACE_ENABLE
#include <stddef.h>

#define ZONE2_ROUTE_TRACE_MAGIC 0x3152325AU
#define ZONE2_ROUTE_TRACE_CAPTURE 1U
#define ZONE2_ROUTE_TRACE_FROZEN 2U
#define ZONE2_ROUTE_TRACE_OBSERVED 1U
#define ZONE2_ROUTE_TRACE_BEGUN 2U
#define ZONE2_ROUTE_TRACE_ARRIVED 4U

typedef struct {
    uint32_t observed_elapsed_ms,begin_elapsed_ms,arrive_elapsed_ms,stop_source_ms,session;
    uint16_t sequence;
    uint8_t status,start,goal,count,source_leg,flags;
    uint8_t ids[24U];
} Zone2RouteTraceLeg;

typedef struct {
    uint32_t magic,state,fault,path_error,run_start_ms,final_elapsed_ms,grid_valid,occupied_mask;
    uint32_t run_id,map_generation,layout_id,link_generation,scan_sequence,completed_legs;
    uint8_t start_id,node_id,zone,final_phase;
    Zone2RouteTraceLeg legs[ZONE2_NAV_LEGS];
} Zone2RouteTrace;

typedef char Zone2RouteTraceLayout[
    sizeof(Zone2RouteTraceLeg)==52U && offsetof(Zone2RouteTraceLeg,sequence)==20U &&
    offsetof(Zone2RouteTraceLeg,ids)==28U && offsetof(Zone2RouteTrace,start_id)==56U &&
    offsetof(Zone2RouteTrace,legs)==60U && sizeof(Zone2RouteTrace)==476U ? 1 : -1];

volatile Zone2RouteTrace g_zone2_route_trace __attribute__((section(".bss.zone2_route_trace"),zero_init));

void Zone2NavSchedule_TraceStart(uint32_t now_ms)
{
    if(g_zone2_route_trace.state) return;
    g_zone2_route_trace.magic=ZONE2_ROUTE_TRACE_MAGIC;
    g_zone2_route_trace.run_start_ms=now_ms;
    g_zone2_route_trace.state=ZONE2_ROUTE_TRACE_CAPTURE;
}

void Zone2NavSchedule_TraceFreeze(const Zone2NavSchedule *s,uint32_t now_ms,
    uint32_t fault,uint32_t path_error,uint8_t final_phase)
{
    if(g_zone2_route_trace.state!=ZONE2_ROUTE_TRACE_CAPTURE) return;
    g_zone2_route_trace.fault=fault;
    g_zone2_route_trace.path_error=path_error;
    g_zone2_route_trace.final_elapsed_ms=now_ms-g_zone2_route_trace.run_start_ms;
    g_zone2_route_trace.completed_legs=s->leg;
    g_zone2_route_trace.final_phase=final_phase;
    g_zone2_route_trace.state=ZONE2_ROUTE_TRACE_FROZEN;
}

static void trace_map(const Zone2NavMap *map)
{
    if(g_zone2_route_trace.state!=ZONE2_ROUTE_TRACE_CAPTURE) return;
    g_zone2_route_trace.run_id=map->context.run_id;
    g_zone2_route_trace.map_generation=map->context.map_generation;
    g_zone2_route_trace.layout_id=map->context.layout_id;
    g_zone2_route_trace.start_id=map->context.start_id;
    g_zone2_route_trace.grid_valid=map->valid;
}
static void trace_local(uint32_t index,const Zone2NavLeg *leg,uint32_t count,uint32_t now_ms)
{
    volatile Zone2RouteTraceLeg *record;
    if(g_zone2_route_trace.state!=ZONE2_ROUTE_TRACE_CAPTURE) return;
    record=&g_zone2_route_trace.legs[index];
    record->observed_elapsed_ms=now_ms-g_zone2_route_trace.run_start_ms;
    record->start=leg->start;record->goal=leg->goal;
    record->count=(uint8_t)count;record->source_leg=(uint8_t)(index+1U);
    record->flags|=ZONE2_ROUTE_TRACE_OBSERVED;
}
#endif

static bool same_identity(const FusionIdentity *a,const FusionIdentity *b)
{
    return a && a->run_id==b->run_id && a->start_id==b->start_id && a->map_version==b->map_version &&
        a->layout_id==b->layout_id && a->path_id==b->path_id && a->direction==b->direction;
}

static Zone2SchedulePhase fail(Zone2NavSchedule *s,Zone2NavPathResult error)
{
    s->phase=ZONE2_SCHEDULE_FAILED;s->error=error;s->plan.valid=0U;s->map.valid=0U;
    return s->phase;
}

bool Zone2NavSchedule_MapCurrent(const Zone2NavSchedule *s)
{
    return s && s->map.valid==1U && s->map.context.run_id==s->run_id &&
        s->map.context.layout_id==s->layout_id && s->map.context.map_generation==s->layout_id &&
        s->map.context.start_id==s->start_id && (s->start_id==1U || s->start_id==2U);
}

static bool fresh_pose(const Zone2NavSchedule *s,const FusionObservation *pose,uint32_t now_ms)
{
    return pose && pose->valid==1U && !pose->continuity_lost && !pose->reasons &&
        pose->feedback_source==FUSION_MEASURED_FEEDBACK &&
        pose->identity.run_id==s->run_id && pose->identity.start_id==s->start_id &&
        pose->identity.layout_id==s->layout_id &&
        (uint32_t)(now_ms-pose->sample_ms)<=s->geometry.config.max_age_ms;
}

bool Zone2NavSchedule_Start(Zone2NavSchedule *s,uint32_t run_id,uint8_t start_id,
    uint32_t layout_id,bool stowed,const FusionConfig *config,float speed_mm_s)
{
    if(!s || (s->phase!=ZONE2_SCHEDULE_IDLE && s->phase!=ZONE2_SCHEDULE_DONE) ||
       !run_id || run_id==s->run_id || (start_id!=1U && start_id!=2U) || !layout_id || !stowed || !config ||
       !config->max_age_ms || config->max_age_ms>=0x80000000U || !(speed_mm_s>0.0f && speed_mm_s<=600.0f))
        return false;
    s->run_id=run_id;s->layout_id=layout_id;s->start_id=start_id;s->leg=0U;s->session=0U;s->error=ZONE2_NAV_OK;
    s->geometry.config=*config;s->geometry.speed_mm_s=speed_mm_s;
    s->map=(Zone2NavMap){{run_id,layout_id,layout_id,start_id},1U};
    s->plan.valid=0U;s->phase=ZONE2_SCHEDULE_POSE;
#if APP_ZONE2_POSE_TRACE_ENABLE
    trace_map(&s->map);
#endif
    return true;
}

Zone2SchedulePhase Zone2NavSchedule_Poll(Zone2NavSchedule *s,const FusionObservation *pose,uint32_t now_ms)
{
    Zone2NavLeg leg;
    Zone2NavPathResult result;
    if(s->phase==ZONE2_SCHEDULE_IDLE || s->phase>=ZONE2_SCHEDULE_DONE) return s->phase;
    if(!Zone2NavSchedule_MapCurrent(s)) return fail(s,ZONE2_NAV_STALE_MAP);
    if(s->phase==ZONE2_SCHEDULE_ACTIVE) return s->phase;
    if(s->phase==ZONE2_SCHEDULE_READY) {
        if((uint32_t)(now_ms-s->prepared_sample_ms)<=s->geometry.config.max_age_ms) return s->phase;
        s->plan.valid=0U;s->phase=ZONE2_SCHEDULE_POSE;
    }
    if(!Zone2NavPath_Leg(s->start_id,s->leg,&leg)) return fail(s,ZONE2_NAV_INPUT);
    if(!fresh_pose(s,pose,now_ms)) return s->phase;
    if(!(fabsf(remainderf(pose->heading_deg-leg.start_heading_deg,360.0f))<=
        s->geometry.config.heading_budget_deg)) return s->phase;
    if(!Zone2NavPath_Station(leg.goal,&s->geometry.terminal)) return fail(s,ZONE2_NAV_TERMINAL);
#if APP_ZONE2_MISSION_ENABLE
    if(leg.goal==10U) s->geometry.terminal.x_mm+=50.0f;
#endif
    s->geometry.actual=pose->position;
    /* Plan the nominal turn and swept envelope from the stage heading.
     * Keep the measured position and check its heading against that envelope. */
    s->geometry.config.start_heading_deg=leg.start_heading_deg;
    s->geometry.config.goal_heading_deg=leg.goal_heading_deg;
    result=Zone2NavPath_PrepareLocal(&s->map,s->leg,&s->geometry,&s->route,&s->plan);
    if(result!=ZONE2_NAV_OK) return fail(s,result);
    if(!Zone2NavSchedule_MapCurrent(s)) return fail(s,ZONE2_NAV_STALE_MAP);
#if APP_ZONE2_POSE_TRACE_ENABLE
    trace_local(s->leg,&leg,s->route.reference.count,now_ms);
#endif
    s->prepared_sample_ms=pose->sample_ms;s->phase=ZONE2_SCHEDULE_READY;
    return s->phase;
}

bool Zone2NavSchedule_BeginLeg(Zone2NavSchedule *s,const FusionIdentity *identity,uint32_t session,uint32_t now_ms)
{
    if(s->phase!=ZONE2_SCHEDULE_READY || !session || session==s->session ||
       !same_identity(identity,&s->plan.input.identity) ||
       (uint32_t)(now_ms-s->prepared_sample_ms)>s->geometry.config.max_age_ms) return false;
    if(!Zone2NavSchedule_MapCurrent(s)) {
        fail(s,ZONE2_NAV_STALE_MAP);
        return false;
    }
    s->session=session;s->phase=ZONE2_SCHEDULE_ACTIVE;
#if APP_ZONE2_POSE_TRACE_ENABLE
    if(g_zone2_route_trace.state==ZONE2_ROUTE_TRACE_CAPTURE) {
        volatile Zone2RouteTraceLeg *record=&g_zone2_route_trace.legs[s->leg];
        record->begin_elapsed_ms=now_ms-g_zone2_route_trace.run_start_ms;
        record->session=session;
        record->flags|=ZONE2_ROUTE_TRACE_BEGUN;
    }
#endif
    return true;
}

bool Zone2NavSchedule_Arrive(Zone2NavSchedule *s,uint32_t session,bool station_arrived,
    const FusionObservation *pose,uint32_t now_ms)
{
    if(s->phase!=ZONE2_SCHEDULE_ACTIVE) return false;
    if(!Zone2NavSchedule_MapCurrent(s)) {
        fail(s,ZONE2_NAV_STALE_MAP);
        return false;
    }
    if(session!=s->session || !station_arrived || !fresh_pose(s,pose,now_ms) || pose->stopped!=1U ||
       !same_identity(&pose->identity,&s->plan.input.identity)) return false;
    if(s->leg+1U==ZONE2_NAV_LEGS &&
       !Zone2NavPath_InsideReturn(s->start_id,pose->position,pose->heading_deg,s->geometry.config.position_budget_mm,
           s->geometry.config.heading_budget_deg,pose->stopped)) return false;
#if APP_ZONE2_POSE_TRACE_ENABLE
    if(g_zone2_route_trace.state==ZONE2_ROUTE_TRACE_CAPTURE) {
        volatile Zone2RouteTraceLeg *record=&g_zone2_route_trace.legs[s->leg];
        record->arrive_elapsed_ms=now_ms-g_zone2_route_trace.run_start_ms;
        record->stop_source_ms=pose->sample_ms;
        record->flags|=ZONE2_ROUTE_TRACE_ARRIVED;
    }
#endif
    s->plan.valid=0U;
    s->phase=++s->leg==ZONE2_NAV_LEGS?ZONE2_SCHEDULE_DONE:ZONE2_SCHEDULE_POSE;
    return true;
}

void Zone2NavSchedule_Abort(Zone2NavSchedule *s)
{
    if(s->phase>ZONE2_SCHEDULE_IDLE && s->phase<ZONE2_SCHEDULE_DONE) (void)fail(s,ZONE2_NAV_STATUS);
}
