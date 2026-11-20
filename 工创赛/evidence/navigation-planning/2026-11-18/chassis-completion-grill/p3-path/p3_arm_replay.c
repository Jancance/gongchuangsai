#include "zone2_nav_path.h"
#include "field_map.h"
#include <math.h>
#include <string.h>

typedef struct {
    Zone2NavLeg leg;
    uint16_t sequence;
    uint8_t cache,count,waypoints[RADAR_CLIENT_MAX_WAYPOINTS];
    uint32_t visited,point_count;
    FusionPoint points[FUSION_POINTS];
    Zone2NavTurns cw,ccw;
    uint32_t alternate;
} LegVector;

typedef struct { float xy,yaw; uint32_t accepted; } ReturnVector;
#include "p3_frozen.inc"

volatile unsigned replay_assertions,replay_case,replay_detail;
volatile float replay_actual,replay_expected;
#define CHECK(expr) do { ++replay_assertions; if(!(expr)) return __LINE__; } while(0)
#define SAME(actual,expected) do { replay_actual=(actual);replay_expected=(expected);CHECK(replay_actual==replay_expected); } while(0)

static const Zone2NavContext context={23,4,2,8,700,2,1,2};
static Zone2NavMap map;
static Zone2NavRequest request;
static Zone2NavReply reply;
static Zone2NavGeometry geometry;
static Zone2NavRoute route;
static FusionPlan plan;

static void set_leg(const LegVector *v,uint32_t leg)
{
    request=(Zone2NavRequest){context,leg+1U,v->sequence,v->leg.start,v->leg.goal,v->cache};
    reply=(Zone2NavReply){0};
    reply.context=context;
    reply.path.request_sequence=v->sequence;
    reply.path.start=v->leg.start;
    reply.path.goal=v->leg.goal;
    reply.path.waypoint_count=v->count;
    memcpy(reply.path.waypoints,v->waypoints,v->count);
    geometry=(Zone2NavGeometry){0};
    geometry.config.position_budget_mm=5;
    geometry.config.heading_budget_deg=1;
    geometry.config.position_tolerance_mm=3;
    geometry.config.max_error_mm=5;
    geometry.config.stop_speed_mm_s=1;
    geometry.config.stop_omega_deg_s=1;
    geometry.config.start_heading_deg=v->leg.start_heading_deg;
    geometry.config.goal_heading_deg=v->leg.goal_heading_deg;
    geometry.config.acceleration_hint_mm_s2=150;
    geometry.config.cross_gain_s=1;
    geometry.config.lateral_cap_mm_s=40;
    geometry.config.preview_s=0.05f;
    geometry.config.wheel_limit_rpm=300;
    geometry.config.heading_reserve_rpm=15;
    geometry.config.speed_state_margin_mm_s=5;
    geometry.config.max_age_ms=100;
    geometry.actual=v->points[0];
    geometry.terminal=v->points[v->point_count-1U];
    geometry.speed_mm_s=400;
}

static unsigned check_turns(const Zone2NavTurns *got,const Zone2NavTurns *want)
{
    SAME(got->travel_heading_deg,want->travel_heading_deg);
    SAME(got->first_deg,want->first_deg);
    SAME(got->second_deg,want->second_deg);
    return 0;
}

static unsigned check_path(const LegVector *v)
{
    uint32_t i,err;
    SAME(Zone2NavPath_Prepare(&map,&request,&reply,&geometry,&route,&plan),ZONE2_NAV_OK);
    CHECK(plan.valid);
    CHECK(plan.input.identity.path_id==request.path_id && plan.input.identity.run_id==context.run_id);
    CHECK(plan.input.identity.map_version==context.map_generation && plan.input.identity.layout_id==context.layout_id);
    SAME(plan.input.config.body_length_mm,283);
    SAME(plan.input.config.body_width_mm,254);
    CHECK(route.visited_mask==v->visited);
    SAME(plan.input.count,v->point_count);
    for(i=0U;i<v->point_count;i++) {
        replay_detail=i;
        SAME(plan.input.points[i].x_mm,v->points[i].x_mm);
        SAME(plan.input.points[i].y_mm,v->points[i].y_mm);
    }
    err=check_turns(&route.preferred,&v->cw);
    if(err) return err;
    SAME(route.has_alternate,v->alternate);
    if(v->alternate) return check_turns(&route.alternate,&v->ccw);
    return 0;
}

static unsigned check_cases(void)
{
    radar_client_grid_result_t grid={700,0,{0}};
    Zone2NavLeg leg;
    FusionPoint terminal;
    unsigned i,err;
    CHECK(Zone2NavPath_SetMap(&context,&grid,&map));
    CHECK(map.valid && !map.occupied_mask);
    for(i=0U;i<ZONE2_NAV_LEGS;i++) {
        const LegVector *v=&legs[i];
        replay_case=i;
        CHECK(Zone2NavPath_Leg(i,&leg));
        CHECK(leg.start==v->leg.start && leg.goal==v->leg.goal);
        SAME(leg.start_heading_deg,v->leg.start_heading_deg);
        CHECK(leg.goal_heading_deg==v->leg.goal_heading_deg ||
              fabsf(leg.goal_heading_deg-v->leg.goal_heading_deg)==360.0f);
        CHECK(Zone2NavPath_Station(leg.goal,&terminal));
        SAME(terminal.x_mm,v->points[v->point_count-1U].x_mm);
        SAME(terminal.y_mm,v->points[v->point_count-1U].y_mm);
        set_leg(v,i);
        if(v->cache) request.request_sequence=2;
        err=check_path(v);
        if(err) return err;
    }
    CHECK(!Zone2NavPath_Leg(ZONE2_NAV_LEGS,&leg));
    CHECK(!Zone2NavPath_Station(24,&terminal));
    replay_case=100;
    grid.count=1; grid.grid_ids[0]=12;
    CHECK(Zone2NavPath_SetMap(&context,&grid,&map));
    set_leg(&legs[2],2);
    SAME(Zone2NavPath_Prepare(&map,&request,&reply,&geometry,&route,&plan),ZONE2_NAV_OCCUPIED);
    CHECK(!plan.valid);
    set_leg(&detour,8);
    err=check_path(&detour);
    if(err) return err;
    CHECK(plan.input.box_count==1 && plan.input.boxes[0].min_x==960 && plan.input.boxes[0].max_y==1440);
    return 0;
}

static unsigned check_rejections(void)
{
    radar_client_grid_result_t grid={700,0,{0}};
    Zone2NavReply bad;
    unsigned i;
    replay_case=101;
    CHECK(Zone2NavPath_SetMap(&context,&grid,&map));
    set_leg(&legs[2],2);
    bad=reply; bad.context.run_id++;
    CHECK(!Zone2NavPath_Matches(&request,&bad));
    bad=reply; bad.context.layout_id++;
    CHECK(!Zone2NavPath_Matches(&request,&bad));
    bad=reply; bad.context.scan_sequence++;
    CHECK(!Zone2NavPath_Matches(&request,&bad));
    bad=reply; bad.context.link_generation++;
    CHECK(!Zone2NavPath_Matches(&request,&bad));
    bad=reply; bad.path.start=10; bad.path.goal=14;
    CHECK(!Zone2NavPath_Matches(&request,&bad));
    bad=reply; bad.path.request_sequence++;
    SAME(Zone2NavPath_Prepare(&map,&request,&bad,&geometry,&route,&plan),ZONE2_NAV_IDENTITY);
    bad=reply; bad.path.status=1;
    SAME(Zone2NavPath_Prepare(&map,&request,&bad,&geometry,&route,&plan),ZONE2_NAV_STATUS);
    bad=reply; bad.path.waypoints[0]=25;
    SAME(Zone2NavPath_Prepare(&map,&request,&bad,&geometry,&route,&plan),ZONE2_NAV_FORMAT);
    bad.path.waypoints[0]=11;
    SAME(Zone2NavPath_Prepare(&map,&request,&bad,&geometry,&route,&plan),ZONE2_NAV_TERMINAL);
    bad=reply; bad.path.waypoint_count=25;
    SAME(Zone2NavPath_Prepare(&map,&request,&bad,&geometry,&route,&plan),ZONE2_NAV_FORMAT);
    bad.path.waypoint_count=2; bad.path.waypoints[0]=4; bad.path.waypoints[1]=10;
    SAME(Zone2NavPath_Prepare(&map,&request,&bad,&geometry,&route,&plan),ZONE2_NAV_NON_AXIS);
    bad.path.waypoints[0]=14;
    SAME(Zone2NavPath_Prepare(&map,&request,&bad,&geometry,&route,&plan),ZONE2_NAV_REPEAT);
    bad.path.waypoint_count=3; bad.path.waypoints[0]=12; bad.path.waypoints[1]=14; bad.path.waypoints[2]=10;
    SAME(Zone2NavPath_Prepare(&map,&request,&bad,&geometry,&route,&plan),ZONE2_NAV_REPEAT);
    geometry.actual=(FusionPoint){1900,1600};
    SAME(Zone2NavPath_Prepare(&map,&request,&reply,&geometry,&route,&plan),ZONE2_NAV_POSE_CELL);
    set_leg(&legs[2],2);
    grid.scan_sequence++;
    CHECK(!Zone2NavPath_SetMap(&context,&grid,&map));
    CHECK(!map.valid);
    SAME(Zone2NavPath_Prepare(&map,&request,&reply,&geometry,&route,&plan),ZONE2_NAV_STALE_MAP);
    grid.scan_sequence=700;
    grid.count=1; grid.grid_ids[0]=255;
    CHECK(!Zone2NavPath_SetMap(&context,&grid,&map));
    grid.count=3; grid.grid_ids[0]=4; grid.grid_ids[1]=4; grid.grid_ids[2]=24;
    CHECK(Zone2NavPath_SetMap(&context,&grid,&map));
    CHECK(map.occupied_mask==((1UL<<4)|(1UL<<24)));

    replay_case=102;
    set_leg(&legs[0],0);
    grid.count=0;
    CHECK(Zone2NavPath_SetMap(&context,&grid,&map));
    request.goal=reply.path.goal=0; reply.path.waypoint_count=0;
    geometry.terminal=(FusionPoint){160,152};
    SAME(Zone2NavPath_Prepare(&map,&request,&reply,&geometry,&route,&plan),ZONE2_NAV_OK);
    SAME(plan.input.count,2);
    geometry.terminal=geometry.actual;
    SAME(Zone2NavPath_Prepare(&map,&request,&reply,&geometry,&route,&plan),ZONE2_NAV_COINCIDENT);
    CHECK(!plan.valid);
    reply.path.waypoint_count=1; reply.path.waypoints[0]=0;
    SAME(Zone2NavPath_Prepare(&map,&request,&reply,&geometry,&route,&plan),ZONE2_NAV_TERMINAL);
    request.goal=reply.path.goal=24; reply.path.waypoint_count=24;
    geometry.terminal=(FusionPoint){2150,2155};
    for(i=0U;i<24U;i++) reply.path.waypoints[i]=i+1U;
    SAME(Zone2NavPath_Prepare(&map,&request,&reply,&geometry,&route,&plan),ZONE2_NAV_GEOMETRY);
    SAME(route.reference.count,27);
    CHECK(route.visited_mask==0x01ffffff);
    request.goal=reply.path.goal=18; reply.path.waypoint_count=2;
    reply.path.waypoints[0]=1; reply.path.waypoints[1]=18;
    geometry.terminal=(FusionPoint){720,1680};
    SAME(Zone2NavPath_Prepare(&map,&request,&reply,&geometry,&route,&plan),ZONE2_NAV_GEOMETRY);
    CHECK(!plan.valid);

    /* Free center cell with an under-body overlap into an occupied neighbor. */
    request.start=reply.path.start=request.goal=reply.path.goal=12;
    reply.path.waypoint_count=0;
    geometry.actual=(FusionPoint){1200,1000};
    geometry.terminal=(FusionPoint){1200,1199};
    grid.count=1; grid.grid_ids[0]=7;
    CHECK(Zone2NavPath_SetMap(&context,&grid,&map));
    SAME(Zone2NavPath_Prepare(&map,&request,&reply,&geometry,&route,&plan),ZONE2_NAV_GEOMETRY);
    return 0;
}

unsigned test_entry(void)
{
    const FusionPoint center={150,150};
    unsigned result=check_cases(),i;
    if(result) return result;
    result=check_rejections();
    if(result) return result;
    replay_case=103;
    for(i=0U;i<sizeof(return_vectors)/sizeof(return_vectors[0]);i++) {
        const ReturnVector *v=&return_vectors[i];
        replay_detail=i;
        SAME(Zone2NavPath_InsideReturn(center,90,v->xy,v->yaw,1),v->accepted);
    }
    CHECK(Zone2NavPath_InsideReturn((FusionPoint){153,147},90,2,1,1));
    CHECK(!Zone2NavPath_InsideReturn((FusionPoint){150,145},90,2,1,1));
    CHECK(!Zone2NavPath_InsideReturn(center,90,2,1,0));
    CHECK(!Zone2NavPath_InsideReturn(center,-90,2,1,1));
    CHECK(!Zone2NavPath_InsideReturn(center,80,2,1,1));
    return 0;
}
