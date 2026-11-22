#include "app_config.h"
#include "zone2_nav_schedule.h"
#include "radar_zone_config.h"
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
    uint8_t ids[RADAR_CLIENT_MAX_WAYPOINTS];
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

static void trace_map(const RadarObserverMap *map)
{
    if(g_zone2_route_trace.state!=ZONE2_ROUTE_TRACE_CAPTURE) return;
    g_zone2_route_trace.occupied_mask=map->occupied_mask;
    g_zone2_route_trace.run_id=map->identity.run_id;
    g_zone2_route_trace.map_generation=map->identity.map_generation;
    g_zone2_route_trace.layout_id=map->identity.layout_id;
    g_zone2_route_trace.link_generation=map->identity.link_generation;
    g_zone2_route_trace.scan_sequence=map->identity.scan_sequence;
    g_zone2_route_trace.start_id=map->identity.start_id;
    g_zone2_route_trace.node_id=map->identity.node_id;
    g_zone2_route_trace.zone=map->identity.zone;
    g_zone2_route_trace.grid_valid=1U;
}

static void trace_reply(uint32_t leg_index,const radar_client_path_result_t *path,
    uint8_t source_leg,uint32_t now_ms)
{
    volatile Zone2RouteTraceLeg *record;
    uint32_t i;
    if(g_zone2_route_trace.state!=ZONE2_ROUTE_TRACE_CAPTURE) return;
    record=&g_zone2_route_trace.legs[leg_index];
    record->observed_elapsed_ms=now_ms-g_zone2_route_trace.run_start_ms;
    record->sequence=path->request_sequence;
    record->status=path->status;
    record->start=path->start;
    record->goal=path->goal;
    record->count=path->waypoint_count;
    record->source_leg=source_leg;
    for(i=0U;i<RADAR_CLIENT_MAX_WAYPOINTS;++i) record->ids[i]=path->waypoints[i];
    record->flags|=ZONE2_ROUTE_TRACE_OBSERVED;
}
#endif

static Zone2NavContext context_of(const RadarObserverMapIdentity *id)
{
    return (Zone2NavContext){id->run_id,id->map_generation,id->layout_id,id->link_generation,
        id->scan_sequence,id->start_id,id->node_id,id->zone};
}

static bool same_identity(const FusionIdentity *a,const FusionIdentity *b)
{
    return a && a->run_id==b->run_id && a->start_id==b->start_id && a->map_version==b->map_version &&
        a->layout_id==b->layout_id && a->path_id==b->path_id && a->direction==b->direction;
}

static Zone2SchedulePhase fail(Zone2NavSchedule *s,Zone2NavPathResult error)
{
    s->phase=ZONE2_SCHEDULE_FAILED;s->error=error;s->plan.valid=0U;s->map.valid=0U;
    memset(s->cache,0,sizeof(s->cache));
    RadarObserver_Cancel();
    return s->phase;
}

static bool current_map(const Zone2NavSchedule *s)
{
    RadarObserverMap map;
    const RadarObserverMapIdentity *id=&s->map_identity;
    return RadarObserver_GetMap(s->run_id,&map) && map.valid==1U && map.occupied_mask==s->map.occupied_mask &&
        map.identity.run_id==id->run_id && map.identity.map_generation==id->map_generation &&
        map.identity.layout_id==id->layout_id && map.identity.link_generation==id->link_generation &&
        map.identity.scan_sequence==id->scan_sequence && map.identity.start_id==id->start_id &&
        map.identity.node_id==id->node_id && map.identity.zone==id->zone;
}

static bool fresh_pose(const Zone2NavSchedule *s,const FusionObservation *pose,uint32_t now_ms)
{
    return pose && pose->valid==1U && !pose->continuity_lost && !pose->reasons &&
        pose->feedback_source==FUSION_MEASURED_FEEDBACK &&
        pose->identity.run_id==s->run_id && pose->identity.start_id==s->map_identity.start_id &&
        pose->identity.layout_id==s->layout_id &&
        (uint32_t)(now_ms-pose->sample_ms)<=s->geometry.config.max_age_ms;
}

bool Zone2NavSchedule_Start(Zone2NavSchedule *s,uint32_t run_id,uint8_t start_id,
    uint32_t layout_id,bool stowed,const FusionConfig *config,float speed_mm_s)
{
    RadarObserverStatus before;
    if(!s || (s->phase!=ZONE2_SCHEDULE_IDLE && s->phase!=ZONE2_SCHEDULE_DONE) ||
       !run_id || run_id==s->run_id || (start_id!=1U && start_id!=2U) || !layout_id || !stowed || !config ||
       !config->max_age_ms || config->max_age_ms>=0x80000000U || !(speed_mm_s>0.0f && speed_mm_s<=400.0f))
        return false;
    RadarObserver_GetStatus(&before);
    if(!RadarObserver_StartScan(run_id,start_id,layout_id)) return false;
    s->run_id=run_id;s->layout_id=layout_id;s->start_id=start_id;s->leg=0U;s->session=0U;s->error=ZONE2_NAV_OK;
    s->scan_status_sequence=before.sequence;
    s->geometry.config=*config;s->geometry.speed_mm_s=speed_mm_s;
    memset(s->cache,0,sizeof(s->cache));
    s->map.valid=0U;s->plan.valid=0U;s->phase=ZONE2_SCHEDULE_SCAN;
    return true;
}

Zone2SchedulePhase Zone2NavSchedule_Poll(Zone2NavSchedule *s,const FusionObservation *pose,uint32_t now_ms)
{
    Zone2NavLeg leg;
    Zone2NavRequest request;
    Zone2NavReply *reply=&s->reply;
    Zone2NavPathResult result;
    bool cached;
    if(s->phase==ZONE2_SCHEDULE_IDLE || s->phase>=ZONE2_SCHEDULE_DONE) return s->phase;
    if(s->phase==ZONE2_SCHEDULE_SCAN) {
        RadarObserverMap map;
        if(!RadarObserver_GetMap(s->run_id,&map)) {
            RadarObserverStatus status;
            RadarObserver_GetStatus(&status);
            if(status.sequence!=s->scan_status_sequence && status.error) return fail(s,ZONE2_NAV_STATUS);
            return s->phase;
        }
        if(map.valid!=1U || !map.status.map_valid || map.status.stage<5U || map.status.stage>8U || map.status.error ||
           map.identity.run_id!=s->run_id || map.identity.layout_id!=s->layout_id ||
           !map.identity.map_generation || !map.identity.link_generation ||
           map.identity.start_id!=s->start_id || map.identity.node_id!=RADAR_NODE_FOR_ZONE(s->start_id) ||
           map.identity.zone!=s->start_id)
            return fail(s,ZONE2_NAV_STALE_MAP);
        s->map_identity=map.identity;
        s->map=(Zone2NavMap){context_of(&map.identity),map.occupied_mask,1U};
#if APP_ZONE2_POSE_TRACE_ENABLE
        trace_map(&map);
#endif
        s->phase=ZONE2_SCHEDULE_REQUEST;
    }
    if(!current_map(s)) return fail(s,ZONE2_NAV_STALE_MAP);
    if(s->phase==ZONE2_SCHEDULE_ACTIVE) return s->phase;
    if(s->phase==ZONE2_SCHEDULE_READY) {
        if((uint32_t)(now_ms-s->prepared_sample_ms)<=s->geometry.config.max_age_ms) return s->phase;
        s->plan.valid=0U;s->phase=ZONE2_SCHEDULE_POSE;
    }
    if(!Zone2NavPath_Leg(s->start_id,s->leg,&leg)) return fail(s,ZONE2_NAV_INPUT);
    cached=s->leg==5U || s->leg==6U;
    if(cached) reply=&s->cache[s->leg-5U];
    if(s->phase==ZONE2_SCHEDULE_REQUEST) {
        if(cached) {
#if APP_ZONE2_POSE_TRACE_ENABLE
            /* Legs 6 and 7 select the saved replies from legs 3 and 4. */
            trace_reply(s->leg,&reply->path,(uint8_t)(s->leg-2U),now_ms);
#endif
            s->phase=ZONE2_SCHEDULE_POSE;
        } else {
            /* A busy observer can accept the same request on a later poll. */
            if(!RadarObserver_RequestPath(&s->map_identity,leg.start,leg.goal)) return s->phase;
            s->phase=ZONE2_SCHEDULE_REPLY;
            return s->phase;
        }
    }
    if(s->phase==ZONE2_SCHEDULE_REPLY) {
        RadarObserverPathReply incoming;
        if(!RadarObserver_GetPathReply(&s->map_identity,leg.start,leg.goal,&incoming)) return s->phase;
#if APP_ZONE2_POSE_TRACE_ENABLE
        trace_reply(s->leg,&incoming.path,(uint8_t)(s->leg+1U),now_ms);
#endif
        if(!incoming.status.path_valid || incoming.status.stage<5U ||
           incoming.status.stage>8U || incoming.status.error) return fail(s,ZONE2_NAV_STATUS);
        s->reply=(Zone2NavReply){context_of(&incoming.identity),incoming.path};
        s->phase=ZONE2_SCHEDULE_POSE;
    }
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
    /* The observer publishes only the accepted path transaction, including its wire sequence. */
    request=(Zone2NavRequest){s->map.context,s->leg+1U,reply->path.request_sequence,
        leg.start,leg.goal,(uint8_t)cached};
    result=Zone2NavPath_Prepare(&s->map,&request,reply,&s->geometry,&s->route,&s->plan);
    if(result!=ZONE2_NAV_OK) return fail(s,result);
    if(!current_map(s)) return fail(s,ZONE2_NAV_STALE_MAP);
    if(s->leg==2U || s->leg==3U) s->cache[s->leg-2U]=s->reply;
    s->prepared_sample_ms=pose->sample_ms;s->phase=ZONE2_SCHEDULE_READY;
    return s->phase;
}

bool Zone2NavSchedule_BeginLeg(Zone2NavSchedule *s,const FusionIdentity *identity,uint32_t session,uint32_t now_ms)
{
    if(s->phase!=ZONE2_SCHEDULE_READY || !session || session==s->session ||
       !same_identity(identity,&s->plan.input.identity) ||
       (uint32_t)(now_ms-s->prepared_sample_ms)>s->geometry.config.max_age_ms) return false;
    if(!current_map(s)) {
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
    if(!current_map(s)) {
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
    s->phase=++s->leg==ZONE2_NAV_LEGS?ZONE2_SCHEDULE_DONE:ZONE2_SCHEDULE_REQUEST;
    return true;
}

void Zone2NavSchedule_Abort(Zone2NavSchedule *s)
{
    if(s->phase>ZONE2_SCHEDULE_IDLE && s->phase<ZONE2_SCHEDULE_DONE) (void)fail(s,ZONE2_NAV_STATUS);
}
