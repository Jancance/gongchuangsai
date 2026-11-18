#include "chassis_auto_test.c"

uint32_t test_tick, test_depth;
volatile uint32_t g_vm_test_request, g_rtos_fault, g_safety_fault, g_safety_ready, g_can_guard_ready;
volatile uint32_t g_motion_diag_fault_latched, g_motion_auto_stop_state, g_motion_auto_stop_cut_vm;
volatile uint32_t g_wheel_test_active, g_wheel_test_cut_vm, g_can_ctrl_fault_latched;
volatile uint32_t g_can_poll_result[4], g_can_poll_flags[4], g_can_poll_count[4];
volatile TickType_t g_can_poll_tick[4];
volatile uint32_t checks, failures, first_failure;

static uint32_t enable_mask, normal, warm_reads, sends, stops, stop_delay, drift, drift_samples, epoch;
static uint32_t fault_mode, fault_address, status_reads, feedback_reads;
static uint32_t fail_status, fail_status_address, status_protect, attempted_status;
static uint32_t slot, phase;

static void check(uint32_t condition,uint32_t line)
{
    checks++;
    if (!condition) { failures++; if (!first_failure) first_failure=line; }
}
#define CHECK(condition) check((condition),__LINE__)

static uint32_t mock_timed_power,mock_power_seen,mock_enable_tick;
void Power_ReadCommandStatus(PowerCommandStatus *s)
{ *s=(PowerCommandStatus){g_vm_test_request,1U,mock_enable_tick,epoch}; }
uint32_t StartupAxes_Holding(void) { return 0U; }
uint32_t CAN1_EmmSettingsConfirmed(void) { return 1U; }

uint32_t CAN1_ReadStatus(uint8_t address,CanRxMsg *reply)
{
    CHECK(!test_depth && address==1U);
    normal=1U; warm_reads++;
    reply->Data[1]=0U;
    test_tick+=2U;
    return 1U;
}
uint32_t CAN1_ReadStatusBounded(uint8_t address,uint32_t expected,CanRxMsg *reply,uint32_t *ms)
{
    CHECK(normal && !test_depth && phase==7U && expected==slot && address>=1U && address<=4U);
    status_reads++;
    attempted_status=address-1U;
    if (fail_status && (!fail_status_address || address==fail_status_address)) {
        fail_status=0U; test_tick+=7U; return CAN1_TIMING_WINDOW;
    }
    reply->Data[1]=(enable_mask>>(address-1U))&1U;
    if (status_protect) reply->Data[1]|=4U;
    *ms=(test_tick+=2U);
    return 1U;
}
uint32_t CAN1_ReadEmmSystemFeedbackBounded(uint8_t address,uint32_t expected,CanRxMsg *reply,
    CAN1_EmmSystemFeedback *f)
{
    (void)reply;
    CHECK(normal && !test_depth && phase<=5U && (phase&1U) && expected==slot);
    feedback_reads++;
    *f=(CAN1_EmmSystemFeedback){test_tick,test_tick+2U,600U+address,0,0U,0U,0U,address,1U};
    test_tick+=2U;
    if (drift && address==3U && g_chassis_auto_state==CHASSIS_AUTO_SETTLE) {
        uint32_t j=drift_samples++;
        if (drift==1U) f->position_counts+=j;
        else if (drift==2U) {
            f->position_counts=(j%3U)!=0U;
            f->position_sign=(j%3U)==2U;
        } else f->speed_decirpm=10;
    }
    return 1U;
}
uint32_t CAN1_ReadEmmSystemFeedback(uint8_t address,CanRxMsg *reply,CAN1_EmmSystemFeedback *f)
{ (void)address; (void)reply; (void)f; CHECK(0U); return 0U; }
uint32_t CAN1_ReadPosition(uint8_t address,CanRxMsg *reply,uint8_t *sign,uint32_t *position)
{ (void)address; (void)reply; (void)sign; (void)position; CHECK(0U); return 0U; }
uint32_t CAN1_SetEmmSpeedRaw(uint8_t address,uint8_t dir,uint16_t raw,uint16_t accel,CanRxMsg *reply)
{ (void)address; (void)dir; (void)raw; (void)accel; (void)reply; CHECK(0U); return 0U; }
uint32_t CAN1_SetMotorEnable(uint8_t address,uint8_t on,CanRxMsg *reply)
{
    (void)reply;
    CHECK(normal && !test_depth && !(phase&1U));
    if (on) enable_mask|=1UL<<(address-1U);
    else enable_mask&=~(1UL<<(address-1U));
    test_tick+=2U;
    return 1U;
}
uint32_t CAN1_SetEmmSpeedRawBounded(uint8_t address,uint8_t dir,uint16_t raw,uint16_t accel,
    uint32_t expected,CanRxMsg *reply,CAN1_EmmSpeedReceipt *r)
{
    (void)dir; (void)raw; (void)accel; (void)reply;
    CHECK(normal && !test_depth && address==phase/2U+1U && !(phase&1U) && expected==slot);
    CHECK(raw==0U); sends++;
    *r=(CAN1_EmmSpeedReceipt){test_tick,test_tick+2U,1U,1U,1U};
    test_tick+=2U;
    if (address==fault_address) {
        if (fault_mode==1U) { test_tick+=5U; return CAN1_TIMING_WINDOW; }
        if (fault_mode==2U) CHECK(ChassisAuto_RequestStop());
        if (fault_mode==3U) epoch++;
        if (fault_mode==4U) { g_can_ctrl_fault_latched=1U; return 8U; }
    }
    return 1U;
}
uint32_t CAN1_StopMotor(uint8_t address,CanRxMsg *reply)
{
    (void)reply;
    CHECK(normal && !test_depth && address==stops+1U);
    stops++;
    test_tick+=stops==4U && stop_delay ? stop_delay : 2U;
    return 1U;
}
void Safety_CanReport(uint32_t a,uint32_t b,uint32_t c,const volatile TickType_t ticks[4])
{ (void)a; (void)b; (void)c; (void)ticks; }
void Safety_MotionFeedbackReport(uint32_t a,uint32_t b,uint32_t c,
    const volatile TickType_t speed[4],const volatile TickType_t position[4])
{ (void)a; (void)b; (void)c; (void)speed; (void)position; }

static void step(void)
{
    test_tick=slot;
    ChassisAuto_SafetyUpdate(g_vm_test_request);
    ChassisAuto_Zone2CanStep(slot,phase);
    ChassisAuto_Zone2SafetyReport();
    CHECK(!test_depth);
    slot+=10U; phase=(phase+1U)%8U;
}
static void setup(uint32_t initial)
{
    ChassisZone2CanStatus s;
    uint32_t n;
    ChassisAuto_Init();
    enable_mask=normal=warm_reads=sends=stops=stop_delay=drift=drift_samples=0U;
    fault_mode=fault_address=status_reads=feedback_reads=0U;
    fail_status=fail_status_address=status_protect=attempted_status=g_can_ctrl_fault_latched=0U;
    epoch=1U; phase=0U; slot=test_tick=initial;
    g_vm_test_request=0U;
    g_safety_ready=g_can_guard_ready=1U;
    CHECK(ChassisAuto_Zone2Prime(3U));
    for (n=0U;n<45U;n++) step();
    ChassisAuto_Zone2Read(&s);
    CHECK(warm_reads==1U && status_reads>=4U && feedback_reads>=4U);
    CHECK(s.feedback_ready && !s.ready && s.state==CHASSIS_AUTO_IDLE);
    CHECK(ChassisAuto_Zone2Begin(3U,s.session));
    for (n=0U;n<155U && g_chassis_auto_state!=CHASSIS_AUTO_START;n++) step();
    ChassisAuto_Zone2Read(&s);
    CHECK(s.feedback_ready && s.ready && s.session==1U && s.power_epoch==1U && !s.ack_generation);
    CHECK(enable_mask==15U && !sends && !stops);
}
static void command(uint32_t generation,uint16_t raw,uint32_t passes)
{
    ChassisZone2CanStatus s;
    ChassisMotorCommand c[4]={{1U,0U,raw},{2U,1U,raw},{3U,0U,raw},{4U,1U,raw}};
    while (phase) step();
    CHECK(ChassisAuto_Zone2Submit(3U,1U,generation,c)==passes);
    if (passes) { uint32_t i; for (i=0U;i<7U;i++) step(); }
    ChassisAuto_Zone2Read(&s);
    CHECK(!passes || fault_mode || (s.result==0U && s.ack_generation==generation &&
        s.possible_mask==15U && s.ack_mask==15U && s.tx_complete_mask==15U));
}

void owner_test_entry(void)
{
    uint32_t n, missed;
    ChassisZone2CanStatus s, previous;
    ChassisMotorCommand zero[4]={{1U,0U,0U},{2U,1U,0U},{3U,0U,0U},{4U,1U,0U}};
    setup(1100U);
    command(1U,200U,0U);
    command(1U,0U,1U);
    CHECK(ChassisAuto_RequestStop());
    drift=2U;
    for (n=0U;n<190U && g_chassis_auto_active;n++) step();
    ChassisAuto_Zone2Read(&s);
    CHECK(stops==4U && s.wheel_quiet && s.result==1U && s.stop_ack_ms);
    CHECK(ChassisAuto_ReleaseCompleted(1U) && ChassisAuto_Zone2Begin(3U,1U));

    setup(2000U);
    command(1U,0U,1U);
    CHECK(ChassisAuto_RequestStop());
    drift=1U;
    for (n=0U;n<225U && g_chassis_auto_active;n++) step();
    ChassisAuto_Zone2Read(&s);
    CHECK(stops==4U && !s.wheel_quiet && s.result==2U && s.cause==7U);

    setup(3000U);
    command(1U,0U,1U);
    fault_mode=1U; fault_address=3U;
    command(2U,200U,1U);
    for (n=0U;n<20U && g_chassis_auto_active;n++) step();
    ChassisAuto_Zone2Read(&s);
    CHECK(s.ack_generation==1U && s.ack_mask==3U && s.possible_mask==7U && s.result==2U);
    CHECK(s.cause==4U && sends==7U && stops==4U && !s.wheel_quiet);

    setup(4000U);
    command(1U,0U,1U);
    fault_mode=2U; fault_address=4U;
    command(2U,200U,1U);
    for (n=0U;n<20U && g_chassis_auto_active;n++) step();
    ChassisAuto_Zone2Read(&s);
    CHECK(s.ack_generation==1U && s.ack_mask==7U && s.possible_mask==15U);
    CHECK(s.result==2U && sends==8U && stops==4U && !s.wheel_quiet);

    setup(5000U);
    command(1U,0U,1U);
    fault_mode=3U; fault_address=4U;
    command(2U,200U,1U);
    step();
    ChassisAuto_Zone2Read(&s);
    CHECK(s.ack_generation==1U && s.ack_mask==7U && s.possible_mask==15U && s.result==2U);
    CHECK(!s.wheel_quiet && !s.feedback_ready && s.cause==2U);

    setup(6000U);
    command(1U,0U,1U);
    fault_mode=4U; fault_address=2U;
    command(2U,200U,1U);
    for (n=0U;n<20U && g_chassis_auto_active;n++) step();
    ChassisAuto_Zone2Read(&s);
    CHECK(s.ack_generation==1U && s.ack_mask==1U && s.possible_mask==3U && s.result==2U);
    CHECK(s.cause==4U && stops==4U && g_can_ctrl_fault_latched);

    setup(7000U);
    command(1U,0U,1U);
    ChassisAuto_Zone2Read(&previous);
    fail_status=1U;
    step();
    ChassisAuto_Zone2Read(&s);
    CHECK(s.state==CHASSIS_AUTO_RUN && !s.cause);
    CHECK(s.status_misses[attempted_status]==1U &&
        s.status_ms[attempted_status]==previous.status_ms[attempted_status]);
    missed=attempted_status;
    for (n=2U;n<=6U;n++) command(n,0U,1U);
    ChassisAuto_Zone2Read(&s);
    CHECK(s.ack_generation==6U && !s.result && !s.cause && s.feedback_ready);
    CHECK(s.status_misses[missed]==1U && !s.status_miss_streak[missed] &&
        Auto_NavNewer(s.status_ms[missed],previous.status_ms[missed]));

    setup(8000U);
    command(1U,0U,1U);
    fail_status=1U;
    step();
    missed=attempted_status;
    fail_status=1U; fail_status_address=missed+1U;
    for (n=2U;n<=5U;n++) command(n,0U,1U);
    fault_mode=5U;
    command(6U,0U,1U);
    for (n=0U;n<16U && g_chassis_auto_active;n++) step();
    ChassisAuto_Zone2Read(&s);
    CHECK(s.ack_generation==5U && s.result==2U && s.cause==3U);
    CHECK(s.status_misses[missed]==2U && s.status_miss_streak[missed]==2U);

    setup(9000U);
    command(1U,0U,1U);
    status_protect=1U;
    step();
    ChassisAuto_Zone2Read(&s);
    CHECK(s.result==2U && s.cause==2U && !s.feedback_ready);

    setup(10000U);
    while (phase) step();
    CHECK(ChassisAuto_Zone2Submit(3U,1U,1U,zero));
    slot+=80U; /* Whole missed cycle, even though the RTOS phase still looks correct. */
    step();
    for (n=0U;n<16U && g_chassis_auto_active;n++) step();
    ChassisAuto_Zone2Read(&s);
    CHECK(s.ack_generation==0U && s.possible_mask==0U && !sends);
    CHECK(s.result==2U && s.cause==6U && stops==4U);

    setup(11000U);
    command(1U,0U,1U);
    CHECK(ChassisAuto_RequestStop());
    drift=3U;
    for (n=0U;n<225U && g_chassis_auto_active;n++) step();
    ChassisAuto_Zone2Read(&s);
    CHECK(stops==4U && !s.wheel_quiet && s.result==2U && s.cause==7U);
}

#include "app_zone2.c"
#include <string.h>

volatile uint32_t g_can_test_result, g_radar_zone_selected;
volatile uint32_t g_radar_servo_state, g_radar_lift_request, g_radar_lift_enable;
volatile uint32_t g_radar_lift_phase_bits, g_radar_lift_direction;
volatile float g_radar_servo_angle_deg[2];

static uint32_t scan_requests,map_published,request_pending,reply_published,map_run,locked_zone;
static uint32_t freeze_pose,freeze_heading,pose_tick,hwt_tick,heading_invalid;
static float map_heading;
static uint32_t change_map_generation, map_gap, rate_invalid;
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
    m->identity=(RadarObserverMapIdentity){1U,10U+change_map_generation,VEHICLE_MAP_VERSION,2U,3U,2U,1U,2U};
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
    m->state=map_gap ? APP_MAP_GAP : APP_MAP_TRACKING;
    if (!freeze_pose) pose_tick=test_tick-8U;
    m->pose=(ChassisOdometryStatus){150.0f,151.5f,map_heading,pose_tick,test_tick-pose_tick,5U,1U,0U,0U,0U};
}
uint32_t App_HeadingSourceRead(AppHeadingSource *h)
{
    if (!freeze_heading) hwt_tick=test_tick-4U;
    *h=(AppHeadingSource){1U,hwt_tick,hwt_tick,rate_invalid ? test_tick-100U : hwt_tick,!heading_invalid,3U,0.0f,0.0f};
    return 1U;
}

void AppMotion_StepOdometry(void) {}
void AppChassis_Zone2CanStep(uint32_t slot_ms,uint32_t phase)
{
    if (g_can_test_result==1U) {
        if (!(phase&1U) && (g_chassis_auto_state==CHASSIS_AUTO_START ||
            g_chassis_auto_state==CHASSIS_AUTO_RUN) && !AppZone2_CanPermit(slot_ms))
            (void)ChassisAuto_RequestStop();
        ChassisAuto_Zone2CanStep(slot_ms,phase);
    }
    ChassisAuto_Zone2SafetyReport();
    AppMotion_StepOdometry();
    AppZone2_Step();
}

static void live_step(void)
{
    test_tick=slot;
    ChassisAuto_SafetyUpdate(g_vm_test_request);
    AppChassis_Zone2CanStep(slot,phase);
    if (mock_timed_power && g_vm_test_request && !mock_power_seen) {
        mock_enable_tick=test_tick;
        mock_power_seen=1U;
    }
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

static void original_test_entry(void)
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
}


static void after_first_zero(uint32_t tick)
{
    uint32_t i;
    ChassisZone2CanStatus can;
    change_map_generation=map_gap=rate_invalid=0U;
    mock_timed_power=tick>UINT32_MAX-4000U;
    mock_enable_tick=mock_power_seen=0U;
    reset_front(tick,90.0f);
    start_front();
    for (i=0U;i<240U;i++) {
        live_step();
        ChassisAuto_Zone2Read(&can);
        if (can.ack_generation==1U && phase==0U) break;
    }
    CHECK(i<240U && can.ack_generation==1U && phase==0U);
    CHECK(g_zone2_phase==ZONE2_ZERO && g_vm_test_request && can.session==1U && can.power_epoch==1U);
}

static void wait_failed(uint32_t limit,uint32_t cause)
{
    uint32_t i;
    for (i=0U;i<limit && g_zone2_phase!=ZONE2_FAILED;i++) live_step();
    CHECK(g_zone2_phase==ZONE2_FAILED && g_zone2_fault==cause);
    CHECK(!locked_zone && !g_zone2_stop_pose_ms);
    for (i=0U;i<150U && g_vm_test_request;i++) live_step();
    CHECK(!g_vm_test_request && !g_chassis_auto_active);
}

void test_entry(void)
{
    uint32_t i;
    ChassisZone2CanStatus can;
    original_test_entry();

    /* Recoverable observation loss with the TRACKING pose still available. */
    after_first_zero(14000U);
    rate_invalid=1U;
    live_step();
    CHECK(g_zone2_phase==ZONE2_STOP && !g_zone2_fault && !g_zone2_stop_pose_ms);
    for (i=0U;i<140U;i++) live_step();
    ChassisAuto_Zone2Read(&can);
    CHECK(g_zone2_phase==ZONE2_STOP && can.state==CHASSIS_AUTO_DONE && can.wheel_quiet);
    CHECK(!g_zone2_stop_pose_ms && g_vm_test_request);
    rate_invalid=0U;
    for (i=0U;i<160U && g_zone2_phase!=ZONE2_SHADOW_DONE;i++) live_step();
    ChassisAuto_Zone2Read(&can);
    CHECK(g_zone2_phase==ZONE2_SHADOW_DONE && can.result==1U && !g_vm_test_request && stops==4U);
    CHECK(sends==4U && newer(g_zone2_stop_pose_ms,can.stop_ack_ms));

    after_first_zero(UINT32_MAX-3200U);
    rate_invalid=1U;
    live_step();
    CHECK(g_zone2_phase==ZONE2_STOP);
    CHECK(s_stop_ms>UINT32_MAX-2500U);
    while (slot-s_stop_ms+2U<2500U) {
        live_step();
        CHECK(g_zone2_phase==ZONE2_STOP && !g_zone2_stop_pose_ms);
    }
    wait_failed(2U,ZONE2_FAULT_DEADLINE);
    CHECK(stops==4U && sends==4U && test_tick-s_stop_ms>=2500U && test_tick-s_stop_ms<=2520U);

    after_first_zero(19000U);
    change_map_generation=1U;
    wait_failed(1U,ZONE2_FAULT_PATH);
    CHECK(stops==4U);

    after_first_zero(22000U);
    map_gap=1U;
    wait_failed(253U,ZONE2_FAULT_DEADLINE);
    CHECK(stops==4U);

    after_first_zero(25000U);
    fault_mode=3U;
    fault_address=2U;
    for (i=0U;i<100U && g_zone2_phase!=ZONE2_FAILED;i++) live_step();
    CHECK(g_zone2_phase==ZONE2_FAILED && g_zone2_fault!=ZONE2_FAULT_NONE);
    for (i=0U;i<20U && g_vm_test_request;i++) live_step();
    CHECK(!g_vm_test_request && !g_chassis_auto_active && !g_zone2_stop_pose_ms);
    CHECK(epoch==2U && g_chassis_auto_cut_vm && !locked_zone);
    front_report_phase=g_zone2_phase;
    front_report_fault=g_zone2_fault;
    front_report_backend=g_chassis_auto_state;
    front_report_sends=sends;
    front_report_stops=stops;
}
