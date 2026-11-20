#include "zone2_nav_path.h"
#include <string.h>

volatile unsigned replay_assertions,replay_case,replay_detail;
volatile float replay_actual,replay_expected;
#define CHECK(x) do { ++replay_assertions; if(!(x)) return __LINE__; } while(0)
#define EQUAL(a,b) do { replay_actual=(float)(a); replay_expected=(float)(b); CHECK(replay_actual==replay_expected); } while(0)

static const Zone2NavContext context={41,2,3,4,511,2,1,2};
static Zone2NavMap map;
static Zone2NavRequest request;
static Zone2NavReply reply;
static Zone2NavGeometry geometry;
static Zone2NavRoute route;
static FusionPlan plan;

static void setup(void)
{
    request=(Zone2NavRequest){context,100,421,0,24,0};
    reply=(Zone2NavReply){context,{0}};
    reply.path.request_sequence=421;
    reply.path.start=0;
    reply.path.goal=24;
    reply.path.waypoint_count=2;
    reply.path.waypoints[0]=20;
    reply.path.waypoints[1]=24;
    geometry=(Zone2NavGeometry){0};
    geometry.config.position_budget_mm=5;
    geometry.config.heading_budget_deg=1;
    geometry.config.position_tolerance_mm=3;
    geometry.config.max_error_mm=5;
    geometry.config.stop_speed_mm_s=1;
    geometry.config.stop_omega_deg_s=1;
    geometry.config.start_heading_deg=90;
    geometry.config.goal_heading_deg=90;
    geometry.config.acceleration_hint_mm_s2=150;
    geometry.config.cross_gain_s=1;
    geometry.config.lateral_cap_mm_s=40;
    geometry.config.preview_s=0.05f;
    geometry.config.wheel_limit_rpm=300;
    geometry.config.heading_reserve_rpm=15;
    geometry.config.speed_state_margin_mm_s=5;
    geometry.config.max_age_ms=100;
    geometry.actual=(FusionPoint){150,150};
    geometry.terminal=(FusionPoint){2160,2160};
    geometry.speed_mm_s=400;
}

static unsigned expect(Zone2NavPathResult result)
{
    plan.valid=1U;
    EQUAL(Zone2NavPath_Prepare(&map,&request,&reply,&geometry,&route,&plan),result);
    EQUAL(plan.valid,result==ZONE2_NAV_OK);
    return 0;
}

#define EXPECT(result) do { unsigned failed=expect(result); if(failed) return failed; } while (0)

static unsigned tests(void)
{
    radar_client_grid_result_t grid={511,2,{4,12}};
    setup();
    replay_case=1;
    CHECK(Zone2NavPath_SetMap(&context,&grid,&map));
    CHECK(map.occupied_mask==((1UL<<4)|(1UL<<12)));
    EXPECT(ZONE2_NAV_OK);
    CHECK(route.visited_mask==((1UL<<0)|(1UL<<9)|(1UL<<10)|(1UL<<19)|
        (1UL<<20)|(1UL<<21)|(1UL<<22)|(1UL<<23)|(1UL<<24)));
    EQUAL(plan.input.count,4);
    EQUAL(plan.input.points[0].x_mm,150);
    EQUAL(plan.input.points[1].x_mm,240);
    EQUAL(plan.input.points[1].y_mm,240);
    EQUAL(plan.input.points[2].x_mm,240);
    EQUAL(plan.input.points[2].y_mm,2160);
    EQUAL(plan.input.points[3].x_mm,2160);
    EQUAL(plan.input.config.body_length_mm,283);
    EQUAL(plan.input.config.body_width_mm,254);
    EQUAL(plan.input.box_count,2);

    replay_case=2;
    request.request_sequence=422;
    EXPECT(ZONE2_NAV_IDENTITY);
    request.from_cache=1;
    request.path_id=101;
    geometry.actual=(FusionPoint){151,153};
    EXPECT(ZONE2_NAV_OK);
    EQUAL(plan.input.identity.path_id,101);
    EQUAL(plan.input.points[0].x_mm,151);
    EQUAL(plan.input.points[0].y_mm,153);
    reply.context.link_generation++;
    EXPECT(ZONE2_NAV_IDENTITY);
    reply.context=context;
    reply.context.scan_sequence++;
    EXPECT(ZONE2_NAV_IDENTITY);
    reply.context=context;
    reply.context.layout_id++;
    EXPECT(ZONE2_NAV_IDENTITY);
    reply.context=context;
    reply.path.start=24;
    reply.path.goal=0;
    EXPECT(ZONE2_NAV_IDENTITY);
    reply.path.start=0;
    reply.path.goal=24;
    map.context.map_generation++;
    EXPECT(ZONE2_NAV_STALE_MAP);
    map.context=context;
    reply.path.status=4;
    EXPECT(ZONE2_NAV_STATUS);

    replay_case=3;
    setup();
    grid.grid_ids[0]=19;
    CHECK(Zone2NavPath_SetMap(&context,&grid,&map));
    EXPECT(ZONE2_NAV_OCCUPIED);
    grid.grid_ids[0]=0;
    CHECK(Zone2NavPath_SetMap(&context,&grid,&map));
    EXPECT(ZONE2_NAV_OCCUPIED);
    grid.grid_ids[0]=24;
    CHECK(Zone2NavPath_SetMap(&context,&grid,&map));
    EXPECT(ZONE2_NAV_OCCUPIED);
    grid.grid_ids[0]=4;
    CHECK(Zone2NavPath_SetMap(&context,&grid,&map));
    reply.path.waypoints[0]=24;
    reply.path.waypoint_count=1;
    EXPECT(ZONE2_NAV_NON_AXIS);
    reply.path.waypoints[0]=20;
    EXPECT(ZONE2_NAV_TERMINAL);
    reply.path.waypoints[0]=25;
    EXPECT(ZONE2_NAV_FORMAT);
    reply.path.waypoints[0]=0;
    reply.path.waypoints[1]=24;
    reply.path.waypoint_count=2;
    EXPECT(ZONE2_NAV_REPEAT);

    replay_case=4;
    /* A second status-zero diagonal, rejected before the footprint check. */
    grid.count=0;
    CHECK(Zone2NavPath_SetMap(&context,&grid,&map));
    setup();
    request.goal=8;
    reply.path.goal=8;
    reply.path.waypoint_count=1;
    reply.path.waypoints[0]=8;
    geometry.actual=(FusionPoint){240,240};
    geometry.terminal=(FusionPoint){720,720};
    EXPECT(ZONE2_NAV_NON_AXIS);

    replay_case=5;
    setup();
    geometry.actual=(FusionPoint){130,150};
    EXPECT(ZONE2_NAV_GEOMETRY);
    geometry.actual=(FusionPoint){480,240};
    EXPECT(ZONE2_NAV_POSE_CELL);
    geometry.actual=(FusionPoint){150,150};
    geometry.terminal=(FusionPoint){2400,2160};
    EXPECT(ZONE2_NAV_POSE_CELL);
    geometry.terminal=(FusionPoint){2160,2160};
    geometry.config.position_budget_mm=7;
    EXPECT(ZONE2_NAV_GEOMETRY);
    geometry.config.position_budget_mm=5;
    EXPECT(ZONE2_NAV_OK);

    replay_case=6;
    request.goal=0;
    reply.path.goal=0;
    reply.path.waypoint_count=0;
    geometry.actual=(FusionPoint){479,240};
    geometry.terminal=(FusionPoint){240,240};
    grid.count=1;
    grid.grid_ids[0]=1;
    CHECK(Zone2NavPath_SetMap(&context,&grid,&map));
    EXPECT(ZONE2_NAV_GEOMETRY);
    geometry.terminal=geometry.actual;
    EXPECT(ZONE2_NAV_COINCIDENT);
    CHECK(!Zone2NavPath_InsideReturn(geometry.actual,90,5,1,1));
    CHECK(Zone2NavPath_InsideReturn((FusionPoint){150,150},90,5,1,1));
    CHECK(!Zone2NavPath_InsideReturn((FusionPoint){150,150},90,5,1,0));
    CHECK(!Zone2NavPath_InsideReturn((FusionPoint){150,150},91,5,1,1));
    CHECK(!Zone2NavPath_InsideReturn((FusionPoint){150,150},90,8,1,1));
    CHECK(!Zone2NavPath_InsideReturn((FusionPoint){150,150},90,5,2,1));
    return 0;
}

unsigned test_entry(void)
{
    replay_detail=0U;
    return tests();
}
