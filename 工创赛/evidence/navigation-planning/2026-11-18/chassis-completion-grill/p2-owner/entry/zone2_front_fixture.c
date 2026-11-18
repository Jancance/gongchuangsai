#include "app_zone2.c"
#include <string.h>

volatile uint32_t g_can_test_result, g_radar_zone_selected;
volatile uint32_t g_radar_servo_state, g_radar_lift_request, g_radar_lift_enable;
volatile uint32_t g_radar_lift_phase_bits, g_radar_lift_direction;
volatile float g_radar_servo_angle_deg[2];

static uint32_t scan_requests,map_published,request_pending,reply_published,map_run,locked_zone;
static uint32_t freeze_pose,freeze_heading,pose_tick,hwt_tick,heading_invalid;
static float map_heading;
volatile uint32_t front_report_phase, front_report_fault, front_report_reference, front_report_path;
volatile uint32_t front_report_sends, front_report_stops, front_report_backend;

uint32_t RadarZoneSwitch_Lock(void) { return locked_zone=g_radar_zone_selected; }
void RadarZoneSwitch_Unlock(void) { locked_zone=0U; }
void RadarLift_GetSnapshot(RadarLiftSnapshot *s) { *s=(RadarLiftSnapshot){0}; }
bool RadarObserver_IsScanReady(void) { return true; }
bool RadarObserver_StartScan(uint32_t run,uint8_t start,uint32_t layout)
{
    CHECK(run==1U && start==2U && layout==VEHICLE_MAP_VERSION && locked_zone==2U);
    CHECK(!g_vm_test_request);
    scan_requests++;
    return true;
}
void RadarObserver_Cancel(void) { map_published=0U; }
void RadarObserver_GetStatus(RadarObserverStatus *s) { *s=(RadarObserverStatus){0}; }
bool RadarObserver_GetMap(uint32_t run,RadarObserverMap *m)
{
    CHECK(run==1U);
    if (!map_published) return false;
    *m=(RadarObserverMap){0};
    m->valid=1U;
    m->status.stage=8U;
    m->status.map_valid=1U;
    m->identity=(RadarObserverMapIdentity){1U,10U,VEHICLE_MAP_VERSION,2U,3U,2U,1U,2U};
    return true;
}
bool RadarObserver_RequestPath(const RadarObserverMapIdentity *i,uint8_t start,uint8_t goal)
{
    CHECK(i->map_generation==10U && i->link_generation==2U && start==0U && goal==2U);
    CHECK(!request_pending);
    request_pending=1U;
    return true;
}
bool RadarObserver_GetPathReply(const RadarObserverMapIdentity *i,uint8_t start,uint8_t goal,
    RadarObserverPathReply *r)
{
    CHECK(request_pending && start==0U && goal==2U);
    if (!reply_published) return false;
    *r=(RadarObserverPathReply){0};
    r->identity=*i;
    r->status.stage=8U;
    r->path.start=start;
    r->path.goal=goal;
    r->path.waypoint_count=1U;
    r->path.waypoints[0]=goal;
    r->path.request_sequence=30U;
    return true;
}
void AppOdometry_RequestMapRun(uint32_t run,uint32_t start)
{
    CHECK(run==1U && start==2U && g_vm_test_request && !g_chassis_auto_active);
    map_run=run;
}
void AppOdometry_ReadMap(AppMapPose *m)
{
    ChassisZone2CanStatus s;
    ChassisAuto_Zone2Read(&s);
    *m=(AppMapPose){0};
    m->run_id=map_run;
    if (!map_run || !s.feedback_ready) return;
    m->start_id=2U;
    m->map_version=VEHICLE_MAP_VERSION;
    m->based=1U;
    m->state=APP_MAP_TRACKING;
    if (!freeze_pose) pose_tick=test_tick-8U;
    m->pose=(ChassisOdometryStatus){150.0f,151.5f,map_heading,pose_tick,test_tick-pose_tick,5U,1U,0U,0U,0U};
}
uint32_t App_HeadingSourceRead(AppHeadingSource *h)
{
    if (!freeze_heading) hwt_tick=test_tick-4U;
    *h=(AppHeadingSource){1U,hwt_tick,hwt_tick,hwt_tick,!heading_invalid,3U,0.0f,0.0f};
    return 1U;
}

static void live_step(void)
{
    test_tick=slot;
    ChassisAuto_SafetyUpdate(g_vm_test_request);
    if (!(phase&1U) && (g_chassis_auto_state==CHASSIS_AUTO_START ||
        g_chassis_auto_state==CHASSIS_AUTO_RUN) && !AppZone2_CanPermit(slot))
        (void)ChassisAuto_RequestStop();
    ChassisAuto_Zone2CanStep(slot,phase);
    ChassisAuto_Zone2SafetyReport();
    AppZone2_Step();
    CHECK(!test_depth);
    slot+=10U;
    phase=(phase+1U)%8U;
}

static void reset_front(uint32_t tick,float heading)
{
    ChassisAuto_Init();
    g_vm_test_request=0U;
    g_safety_ready=g_can_guard_ready=g_can_test_result=1U;
    g_rtos_fault=g_safety_fault=g_can_ctrl_fault_latched=0U;
    enable_mask=normal=warm_reads=sends=stops=stop_delay=drift=drift_samples=0U;
    fault_mode=fault_address=status_reads=feedback_reads=0U;
    fail_status=fail_status_address=status_protect=attempted_status=0U;
    epoch=1U; phase=0U; slot=test_tick=tick;
    g_zone2_phase=ZONE2_IDLE;
    g_zone2_fault=ZONE2_FAULT_NONE;
    g_zone2_reference_status=g_zone2_stop_pose_ms=0U;
    s_start_request=s_begin_ms=s_session=s_power_epoch=s_primed=s_zone_locked=0U;
    memset(&s_schedule,0,sizeof(s_schedule));
    FusionReference_Reset(&s_reference);
    scan_requests=map_published=request_pending=reply_published=map_run=locked_zone=0U;
    g_radar_servo_state=1U;
    g_radar_servo_angle_deg[1]=180.0f;
    g_radar_zone_selected=1U;
    freeze_pose=freeze_heading=heading_invalid=0U;
    map_heading=heading;
}

static void start_front(void)
{
    uint32_t i;
    AppZone2_RequestStart();
    AppZone2_RequestStart();
    for (i=0U;i<5U;i++) live_step();
    CHECK(g_zone2_phase==ZONE2_WAIT_READY && !scan_requests && !g_vm_test_request);
    g_radar_zone_selected=2U;
    g_radar_servo_angle_deg[1]=45.0f;
    live_step();
    CHECK(!scan_requests && !locked_zone);
    g_radar_servo_angle_deg[1]=180.0f;
    live_step();
    CHECK(g_zone2_phase==ZONE2_SCAN && scan_requests==1U && locked_zone==2U);
    for (i=0U;i<6U;i++) live_step();
    CHECK(!g_vm_test_request && !map_run);
    map_published=reply_published=1U;
    live_step();
    CHECK(g_zone2_phase==ZONE2_PREPARE && request_pending && map_run==1U && !g_chassis_auto_active);
}

static void good_path(uint32_t tick,float heading)
{
    uint32_t i;
    ChassisZone2CanStatus can;
    reset_front(tick,heading);
    start_front();
    for (i=0U;i<460U && g_zone2_phase!=ZONE2_SHADOW_DONE;i++) live_step();
    ChassisAuto_Zone2Read(&can);
    if (!front_report_phase && g_zone2_phase!=ZONE2_SHADOW_DONE) {
        front_report_phase=g_zone2_phase;
        front_report_fault=g_zone2_fault;
        front_report_reference=g_zone2_reference_status;
        front_report_path=s_schedule.phase;
        front_report_sends=sends;
        front_report_stops=stops;
        front_report_backend=can.state;
    }
    CHECK(g_zone2_phase==ZONE2_SHADOW_DONE && !g_zone2_fault);
    CHECK(g_zone2_reference_status==FUSION_TURN_START && s_schedule.phase==ZONE2_SCHEDULE_ACTIVE);
    CHECK(s_schedule.geometry.config.start_heading_deg==90.0f && s_schedule.geometry.actual.y_mm==151.5f);
    CHECK(scan_requests==1U && can.ack_generation>=2U && sends==4U*can.ack_generation && stops==4U);
    CHECK(newer(g_zone2_stop_pose_ms,can.stop_ack_ms));
    CHECK(!g_vm_test_request && !locked_zone);
    AppZone2_RequestStart();
    live_step();
    CHECK(scan_requests==1U && sends==4U*can.ack_generation);
}

static void externally_requested_stop(uint32_t tick,uint32_t lose_map,uint32_t recover)
{
    ChassisZone2CanStatus can;
    uint32_t i, prior_sends;
    reset_front(tick,90.0f);
    start_front();
    for (i=0U;i<350U;i++) {
        ChassisAuto_Zone2Read(&can);
        if (g_zone2_phase==ZONE2_ZERO && can.ack_generation && !(phase&1U)) break;
        live_step();
    }
    CHECK(g_zone2_phase==ZONE2_ZERO && can.ack_generation && !(phase&1U));
    prior_sends=sends;
    if (lose_map) map_published=0U;
    else heading_invalid=1U;
    live_step();
    if (lose_map) {
        CHECK(g_zone2_phase==ZONE2_FAILED && g_zone2_fault==ZONE2_FAULT_PATH &&
            g_zone2_path_error==ZONE2_NAV_STALE_MAP && !g_zone2_stop_pose_ms);
        map_published=1U;
        for (i=0U;i<140U;i++) live_step();
        ChassisAuto_Zone2Read(&can);
        CHECK(can.state==CHASSIS_AUTO_DONE && can.result==1U && g_zone2_phase==ZONE2_FAILED);
        CHECK(stops==4U && sends==prior_sends && !g_vm_test_request && !locked_zone);
        return;
    }
    CHECK(g_zone2_phase==ZONE2_STOP && !g_zone2_stop_pose_ms && sends==prior_sends);
    for (i=0U;i<140U;i++) live_step();
    ChassisAuto_Zone2Read(&can);
    CHECK(g_zone2_phase==ZONE2_STOP && can.state==CHASSIS_AUTO_DONE && can.result==1U &&
        can.wheel_quiet && !g_zone2_stop_pose_ms && g_vm_test_request && locked_zone==2U);
    if (recover) {
        heading_invalid=0U;
        live_step();
        CHECK(g_zone2_phase==ZONE2_SHADOW_DONE && g_zone2_fault==ZONE2_FAULT_NONE &&
            newer(g_zone2_stop_pose_ms,can.stop_ack_ms));
    } else {
        for (i=0U;i<120U && g_zone2_phase==ZONE2_STOP;i++) live_step();
        CHECK(g_zone2_phase==ZONE2_FAILED && g_zone2_fault==ZONE2_FAULT_DEADLINE &&
            !g_zone2_stop_pose_ms);
    }
    CHECK(stops==4U && sends==prior_sends && !g_vm_test_request && !locked_zone);
}

void test_entry(void)
{
    uint32_t i;
    good_path(1000U,90.875f);
    good_path(UINT32_MAX-500U,449.125f);
    reset_front(5000U,91.125f);
    start_front();
    for (i=0U;i<180U;i++) live_step();
    CHECK(g_zone2_phase==ZONE2_PREPARE && s_schedule.phase==ZONE2_SCHEDULE_POSE && !sends);
    reset_front(8000U,90.0f);
    start_front();
    for (i=0U;i<150U && g_zone2_phase!=ZONE2_STOP;i++) live_step();
    CHECK(g_zone2_phase==ZONE2_STOP);
    heading_invalid=1U;
    freeze_pose=1U;
    for (i=0U;i<140U;i++) live_step();
    CHECK(g_zone2_phase==ZONE2_STOP && !g_zone2_stop_pose_ms);
    freeze_pose=0U; heading_invalid=0U;
    live_step();
    CHECK(g_zone2_phase==ZONE2_SHADOW_DONE);
    externally_requested_stop(12000U,0U,1U);
    externally_requested_stop(UINT32_MAX-1000U,0U,0U);
    externally_requested_stop(18000U,1U,0U);
}
