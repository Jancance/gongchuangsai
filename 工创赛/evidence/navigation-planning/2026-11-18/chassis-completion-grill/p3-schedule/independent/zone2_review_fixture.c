#include "zone2_nav_schedule.h"
#include <string.h>

volatile unsigned replay_assertions,replay_case,replay_detail;
volatile float replay_actual,replay_expected;
static Zone2NavSchedule schedule;
static RadarObserverMap map;
static RadarObserverPathReply response;
static unsigned request_count,cancel_count,map_available,reply_available;

#define CHECK(x) do { ++replay_assertions; if(!(x)) return __LINE__; } while(0)
#define EQUAL(a,b) do { replay_actual=(float)(a); replay_expected=(float)(b); CHECK(replay_actual==replay_expected); } while(0)

void RadarObserver_GetStatus(RadarObserverStatus *status) { *status=map.status; }
bool RadarObserver_GetMap(uint32_t id,RadarObserverMap *out)
{
    if(!map_available || id!=map.identity.run_id) return false;
    *out=map;
    return true;
}
bool RadarObserver_StartScan(uint32_t id,uint8_t zone,uint32_t layout)
{
    return id==21U && zone==2U && layout==4U;
}
bool RadarObserver_RequestPath(const RadarObserverMapIdentity *id,uint8_t start,uint8_t goal)
{
    if(id->run_id!=21U || id->layout_id!=4U || start!=0U || goal!=2U) return false;
    ++request_count;
    return true;
}
bool RadarObserver_GetPathReply(const RadarObserverMapIdentity *id,uint8_t start,uint8_t goal,RadarObserverPathReply *out)
{
    if(!reply_available || id->run_id!=21U || start!=0U || goal!=2U) return false;
    *out=response;
    return true;
}
void RadarObserver_Cancel(void) { ++cancel_count; map_available=0U; }

static unsigned check_schedule(void)
{
    FusionConfig config={0};
    FusionObservation pose={0};
    config.position_budget_mm=5;
    config.heading_budget_deg=1;
    config.position_tolerance_mm=3;
    config.max_error_mm=5;
    config.stop_speed_mm_s=1;
    config.stop_omega_deg_s=1;
    config.acceleration_hint_mm_s2=150;
    config.cross_gain_s=1;
    config.lateral_cap_mm_s=40;
    config.preview_s=0.05f;
    config.wheel_limit_rpm=300;
    config.heading_reserve_rpm=15;
    config.speed_state_margin_mm_s=5;
    config.max_age_ms=100;
    map=(RadarObserverMap){{21U,71U,4U,8U,24U,2U,1U,2U},0U,1U,{5U,8U,0U,1U,0U,0U}};
    response.identity=map.identity;
    response.status=(RadarObserverStatus){6U,8U,0U,1U,1U,0U};
    response.path.start=0U;
    response.path.goal=2U;
    response.path.request_sequence=13U;
    response.path.waypoint_count=1U;
    response.path.waypoints[0]=2U;
    replay_case=1U;
    CHECK(!Zone2NavSchedule_Start(&schedule,21U,1U,4U,true,&config,400.0f));
    CHECK(!Zone2NavSchedule_Start(&schedule,21U,2U,4U,false,&config,400.0f));
    CHECK(Zone2NavSchedule_Start(&schedule,21U,2U,4U,true,&config,400.0f));
    CHECK(!Zone2NavSchedule_Start(&schedule,21U,2U,4U,true,&config,400.0f));
    EQUAL(Zone2NavSchedule_Poll(&schedule,0,0),ZONE2_SCHEDULE_SCAN);
    map_available=1U;
    EQUAL(Zone2NavSchedule_Poll(&schedule,0,0),ZONE2_SCHEDULE_REPLY);
    CHECK(request_count==1U);
    EQUAL(Zone2NavSchedule_Poll(&schedule,0,0),ZONE2_SCHEDULE_REPLY);
    reply_available=1U;
    EQUAL(Zone2NavSchedule_Poll(&schedule,0,0),ZONE2_SCHEDULE_POSE);
    CHECK(request_count==1U && !schedule.plan.valid);

    pose.identity=(FusionIdentity){21U,2U,90U,4U,0U,0U};
    pose.position=(FusionPoint){150.0f,150.0f};
    pose.heading_deg=90.0f;
    pose.valid=1U;
    pose.feedback_source=FUSION_MEASURED_FEEDBACK;
    pose.sample_ms=100U;
    pose.stopped=1U;

    replay_case=2U;
    EQUAL(Zone2NavSchedule_Poll(&schedule,&pose,201U),ZONE2_SCHEDULE_POSE);
    pose.feedback_source=FUSION_ESTIMATED_FEEDBACK;
    EQUAL(Zone2NavSchedule_Poll(&schedule,&pose,101U),ZONE2_SCHEDULE_POSE);
    pose.feedback_source=FUSION_MEASURED_FEEDBACK;
    pose.identity.start_id=1U;
    EQUAL(Zone2NavSchedule_Poll(&schedule,&pose,101U),ZONE2_SCHEDULE_POSE);
    pose.identity.start_id=2U;

    replay_case=3U;
    pose.identity.layout_id=99U;
    EQUAL(Zone2NavSchedule_Poll(&schedule,&pose,101U),ZONE2_SCHEDULE_POSE);
    CHECK(!schedule.plan.valid);
    pose.identity.layout_id=4U;

    replay_case=4U;
    pose.sample_ms=0xFFFFFFFEU;
    EQUAL(Zone2NavSchedule_Poll(&schedule,&pose,1U),ZONE2_SCHEDULE_READY);
    CHECK(schedule.plan.valid && schedule.plan.input.identity.path_id==1U);
    CHECK(schedule.plan.input.identity.map_version==71U);
    CHECK(schedule.plan.input.points[0].x_mm==150.0f);
    replay_case=5U;
    CHECK(!Zone2NavSchedule_BeginLeg(&schedule,&schedule.plan.input.identity,9U,99U));
    CHECK(schedule.phase==ZONE2_SCHEDULE_READY && schedule.session==0U);
    EQUAL(Zone2NavSchedule_Poll(&schedule,0,99U),ZONE2_SCHEDULE_POSE);
    CHECK(!schedule.plan.valid && request_count==1U);
    pose.sample_ms=99U;
    pose.position.x_mm=151.0f;
    EQUAL(Zone2NavSchedule_Poll(&schedule,&pose,99U),ZONE2_SCHEDULE_READY);
    CHECK(schedule.plan.valid && schedule.plan.input.points[0].x_mm==151.0f && request_count==1U);
    CHECK(!Zone2NavSchedule_BeginLeg(&schedule,&pose.identity,9U,100U));
    CHECK(Zone2NavSchedule_BeginLeg(&schedule,&schedule.plan.input.identity,9U,100U));
    CHECK(!Zone2NavSchedule_Arrive(&schedule,9U,true,&pose,100U));
    pose.identity=schedule.plan.input.identity;
    CHECK(!Zone2NavSchedule_Arrive(&schedule,8U,true,&pose,100U));
    CHECK(!Zone2NavSchedule_Arrive(&schedule,9U,false,&pose,100U));
    pose.stopped=0U;
    CHECK(!Zone2NavSchedule_Arrive(&schedule,9U,true,&pose,100U));
    CHECK(schedule.phase==ZONE2_SCHEDULE_ACTIVE && schedule.session==9U);
    replay_case=6U;
    map.identity.link_generation++;
    EQUAL(Zone2NavSchedule_Poll(&schedule,&pose,101U),ZONE2_SCHEDULE_FAILED);
    CHECK(schedule.error==ZONE2_NAV_STALE_MAP && cancel_count==1U && !schedule.plan.valid);
    CHECK(schedule.cache[0].path.waypoint_count==0U && schedule.cache[1].path.waypoint_count==0U);
    EQUAL(Zone2NavSchedule_Poll(&schedule,&pose,3U),ZONE2_SCHEDULE_FAILED);
    CHECK(cancel_count==1U);
    return 0U;
}

unsigned test_entry(void) { replay_detail=0U;return check_schedule(); }
