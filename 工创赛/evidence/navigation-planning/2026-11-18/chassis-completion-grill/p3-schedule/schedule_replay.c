#define test_entry observer_test_entry
#include "observer_fixture.c"
#undef test_entry
#include "zone2_nav_schedule.h"

volatile unsigned replay_assertions,replay_case,replay_detail;
volatile float replay_actual,replay_expected;
static Zone2NavSchedule schedule;
static const FusionConfig nav_config={
    .position_budget_mm=5,.heading_budget_deg=1,.position_tolerance_mm=5,.max_error_mm=5,
    .stop_speed_mm_s=1,.stop_omega_deg_s=1,.acceleration_hint_mm_s2=150,.cross_gain_s=1,
    .lateral_cap_mm_s=40,.preview_s=0.05f,.wheel_limit_rpm=300,.heading_reserve_rpm=15,
    .speed_state_margin_mm_s=5,.max_age_ms=100
};
typedef struct {uint8_t start,goal,count,nodes[2];float heading;} PeerPath;
static const PeerPath paths[]={
    {0,2,1,{2},90},{2,14,2,{4,14},90},{14,10,1,{10},-90},{10,22,2,{12,22},180},
    {22,14,2,{12,14},90},{14,10,1,{10},-90},{10,22,2,{12,22},180},{22,0,2,{2,0},90}
};

static void nav_frame(uint8_t type,const uint8_t *data,uint16_t size)
{
    queue(1,type,data,size);
    RadarObserver_Service();
}

static void nav_ack(uint8_t command)
{
    const uint8_t ack_bytes[2]={command,0};
    nav_frame(LINK_EVT_ACK,ack_bytes,sizeof(ack_bytes));
}

static void nav_ready(void)
{
    const RadarZone2Config nav_radar={1,0,8,5,5,5,1000,1};
    const uint8_t hello0[13]={1,36,6,0},hello2[13]={1,36,6,2},persisted[33]={11};
    CHECK(RadarObserver_Configure(&nav_radar));
    RadarObserver_Service();
    nav_frame(LINK_EVT_HELLO,hello0,sizeof(hello0));
    RadarObserver_Service();
    nav_ack(LINK_CMD_SELECT_ZONE);
    nav_frame(LINK_EVT_HELLO,hello2,sizeof(hello2));
    RadarObserver_Service();
    nav_frame(LINK_EVT_PERSISTED_STATE,persisted,sizeof(persisted));
    nav_ack(LINK_CMD_GET_PERSISTED_STATE);
    CHECK(RadarObserver_IsScanReady());
}

static void scan(uint32_t run,uint8_t grid_sequence)
{
    const uint8_t grid[5]={grid_sequence,0,0,0,0};
    CHECK(Zone2NavSchedule_Start(&schedule,run,2,123,true,&nav_config,400));
    CHECK(!Zone2NavSchedule_Start(&schedule,run,2,123,true,&nav_config,400));
    CHECK(Zone2NavSchedule_Poll(&schedule,0,tick)==ZONE2_SCHEDULE_SCAN);
    RadarObserver_Service();
    tick+=5;
    RadarObserver_Service();
    leg_complete();
    tick+=5;
    RadarObserver_Service();
    RadarObserver_Service();
    nav_ack(LINK_CMD_SINGLE_MEASURE);
    nav_frame(LINK_EVT_OBSTACLE_GRID,grid,sizeof(grid));
    nav_frame(LINK_EVT_STATUS,locked,sizeof(locked));
    CHECK(Zone2NavSchedule_Poll(&schedule,0,tick)==ZONE2_SCHEDULE_SCAN);
    leg_complete();
    CHECK(Zone2NavSchedule_Poll(&schedule,0,tick)==ZONE2_SCHEDULE_SCAN);
    tick+=5;
    RadarObserver_Service();
    CHECK(stage()==8 && servo_angle==180 && !lift_claim && !g_radar_lift_enable);
    CHECK(Zone2NavSchedule_Poll(&schedule,0,tick)==ZONE2_SCHEDULE_REPLY);
}

static void path_result(const PeerPath *path)
{
    uint16_t seq,stale;
    uint8_t result[8]={0,0,0,path->start,path->goal,path->count,path->nodes[0],path->nodes[1]};
    RadarObserver_Service();
    RadarObserver_Service();
    seq=command_sequence[LINK_CMD_PLAN_PATH];
    stale=seq-1U;
    result[0]=(uint8_t)stale;result[1]=(uint8_t)(stale>>8);
    nav_frame(LINK_EVT_PATH_RESULT,result,6U+path->count);
    CHECK(Zone2NavSchedule_Poll(&schedule,0,tick)==ZONE2_SCHEDULE_REPLY);
    result[3]=path->goal;result[4]=path->start;
    result[6]=path->start;result[5]=1U;
    nav_frame(LINK_EVT_PATH_RESULT,result,7U);
    CHECK(Zone2NavSchedule_Poll(&schedule,0,tick)==ZONE2_SCHEDULE_REPLY);
    result[0]=(uint8_t)seq;result[1]=(uint8_t)(seq>>8);
    result[3]=path->start;result[4]=path->goal;
    result[5]=path->count;result[6]=path->nodes[0];
    nav_frame(LINK_EVT_PATH_RESULT,result,6U+path->count);
    CHECK(Zone2NavSchedule_Poll(&schedule,0,tick)==ZONE2_SCHEDULE_REPLY);
    nav_ack(LINK_CMD_PLAN_PATH);
    CHECK(Zone2NavSchedule_Poll(&schedule,0,tick)==ZONE2_SCHEDULE_POSE);
}

unsigned test_entry(void)
{
    uint32_t i,requests=0U;
    uint16_t pending;
    uint8_t reverse[7]={0,0,0,2,0,1,0};
    FusionObservation pose={0};
    nav_ready();
    CHECK(!Zone2NavSchedule_Start(&schedule,1,1,123,true,&nav_config,400));
    CHECK(!Zone2NavSchedule_Start(&schedule,1,2,123,false,&nav_config,400));
    scan(1,44);
    pose.identity.run_id=1;pose.identity.start_id=2;pose.identity.layout_id=123;
    pose.position=(FusionPoint){150,150};pose.heading_deg=90;
    pose.valid=1U;pose.feedback_source=FUSION_MEASURED_FEEDBACK;
    for(i=0U;i<8U;i++) {
        FusionIdentity wrong;
        const PeerPath *path=&paths[i];
        const uint32_t session=i+100U;
        if(i!=5U && i!=6U) {
            CHECK(Zone2NavSchedule_Poll(&schedule,0,tick)==ZONE2_SCHEDULE_REPLY);
            path_result(path);requests++;
        }
        pose.sample_ms=tick-nav_config.max_age_ms-1U;
        CHECK(Zone2NavSchedule_Poll(&schedule,&pose,tick)==ZONE2_SCHEDULE_POSE);
        pose.sample_ms=tick;
        pose.identity.layout_id++;
        CHECK(Zone2NavSchedule_Poll(&schedule,&pose,tick)==ZONE2_SCHEDULE_POSE);
        pose.identity.layout_id--;
        if(i==5) pose.position=(FusionPoint){2163,1196};
        CHECK(Zone2NavSchedule_Poll(&schedule,&pose,tick)==ZONE2_SCHEDULE_READY);
        CHECK(schedule.plan.valid && schedule.plan.input.identity.path_id==i+1U);
        CHECK(schedule.plan.input.points[0].x_mm==pose.position.x_mm);
        CHECK(schedule.plan.input.points[0].y_mm==pose.position.y_mm);
        CHECK(command_count[LINK_CMD_PLAN_PATH]==requests);
        wrong=schedule.plan.input.identity;
        wrong.path_id--;
        CHECK(!Zone2NavSchedule_BeginLeg(&schedule,&wrong,session,tick));
        CHECK(!Zone2NavSchedule_BeginLeg(&schedule,&schedule.plan.input.identity,session,tick+101U));
        CHECK(Zone2NavSchedule_BeginLeg(&schedule,&schedule.plan.input.identity,session,tick));
        CHECK(!Zone2NavSchedule_BeginLeg(&schedule,&schedule.plan.input.identity,session,tick));
        CHECK(Zone2NavPath_Station(path->goal,&pose.position));
        pose.heading_deg=path->heading;
        pose.identity=wrong;
        pose.stopped=1U;
        CHECK(!Zone2NavSchedule_Arrive(&schedule,session,true,&pose,tick));
        pose.identity=schedule.plan.input.identity;
        CHECK(!Zone2NavSchedule_Arrive(&schedule,session-1U,true,&pose,tick));
        CHECK(!Zone2NavSchedule_Arrive(&schedule,session,false,&pose,tick));
        pose.stopped=0U;
        CHECK(!Zone2NavSchedule_Arrive(&schedule,session,true,&pose,tick));
        pose.stopped=1U;
        if(i==7U) {
            pose.position.x_mm=166;
            CHECK(!Zone2NavSchedule_Arrive(&schedule,session,true,&pose,tick));
            pose.position.x_mm=150;
        }
        CHECK(Zone2NavSchedule_Arrive(&schedule,session,true,&pose,tick));
        CHECK(!schedule.plan.valid && schedule.leg==i+1U);
    }
    CHECK(schedule.phase==ZONE2_SCHEDULE_DONE && requests==6U);
    CHECK(command_count[LINK_CMD_SINGLE_MEASURE]==1U);
    scan(2,45);
    CHECK(schedule.cache[0].context.run_id==0 && schedule.cache[1].context.run_id==0);
    RadarObserver_Service();
    RadarObserver_Service();
    pending=command_sequence[LINK_CMD_PLAN_PATH];
    reverse[0]=(uint8_t)pending;reverse[1]=(uint8_t)(pending>>8);
    nav_frame(LINK_EVT_PATH_RESULT,reverse,sizeof(reverse));
    CHECK(Zone2NavSchedule_Poll(&schedule,&pose,tick)==ZONE2_SCHEDULE_FAILED);
    CHECK(!schedule.plan.valid && !schedule.map.valid && schedule.error==ZONE2_NAV_STALE_MAP);
    nav_ack(LINK_CMD_PLAN_PATH);
    CHECK(Zone2NavSchedule_Poll(&schedule,&pose,tick)==ZONE2_SCHEDULE_FAILED);
    replay_assertions=audit_checks;
    replay_case=requests;
    replay_detail=schedule.phase;
    replay_actual=(float)requests;
    replay_expected=6.0f;
    return audit_failed_lines[0];
}
