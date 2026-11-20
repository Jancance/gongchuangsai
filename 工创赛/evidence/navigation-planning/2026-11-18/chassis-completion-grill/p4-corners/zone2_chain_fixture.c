#include "owner_peer.c"
#undef CHECK
#include "radar_peer.c"
#undef CHECK
#include "app_zone2.c"
#include "app_chassis_service.h"

#define CHECK(condition) check((condition),__LINE__)
#define CHAIN_SCORING_BUDGET_MS 180000U
#define CHAIN_MAX_SLOTS 21000U
#define CHAIN_OCCUPIED_GRID (CHAIN_OCCUPIED_DETOUR || CHAIN_OCCUPIED_BAD_PATH)

volatile unsigned replay_assertions,replay_case,replay_detail;
volatile float replay_actual,replay_expected;
volatile uint32_t chain_stop_legs,chain_path_requests,chain_reuses,chain_fault;
volatile uint32_t chain_cause,chain_reference,chain_backend,chain_sends,chain_nonzero;
volatile uint32_t chain_shadow,chain_map_state,chain_map_reasons,chain_map_updates;
volatile uint32_t chain_path_error,chain_stop_frames,chain_runtime_ms;
volatile uint32_t chain_start_ms,chain_scoring_budget_end_ms,chain_outcome_ms;
volatile uint32_t chain_post_budget_nonzero;
volatile uint32_t chain_command_hash=2166136261U;
volatile uint32_t chain_scan_stage_ms[8],chain_scan_tx_ms,chain_scan_result_ms,chain_scan_ready_ms,chain_lift_leg_ms;
volatile uint32_t chain_arrival_ms[8],chain_plan_ms[8],chain_plan_sections[8];
volatile uint32_t chain_track_ms[8],chain_tracking_checks,chain_edge_checks;
volatile uint32_t chain_worst_leg,chain_worst_section,chain_worst_span,chain_worst_ms;
volatile uint32_t chain_recovery_ms,chain_recovery_stops,chain_recovery_quiet;
volatile uint32_t chain_truth_samples,chain_truth_contacts,chain_first_contact_ms;
volatile uint32_t chain_robust_contacts;
volatile uint32_t chain_one_miss,chain_one_miss_ms,chain_owner_miss;
volatile uint32_t chain_partial,chain_f6_possible,chain_f6_tx,chain_f6_ack;
volatile uint32_t chain_occupied_intervals,chain_occupied_contacts,chain_detour_witness;
volatile uint32_t chain_occupied_reject_nonzero_after_first_stop;
volatile float chain_x,chain_y,chain_heading;
volatile float chain_max_error_mm,chain_max_heading_deg,chain_edge_error_mm;
volatile float chain_worst_ref_x,chain_worst_ref_y;
volatile float chain_truth_x,chain_truth_y,chain_truth_yaw,chain_truth_clearance;
volatile float chain_robust_clearance;
volatile float chain_occupied_clearance,chain_detour_witness_x,chain_detour_witness_y;
volatile float g_radar_servo_angle_deg[2]={0.0f,180.0f};
volatile uint32_t g_radar_servo_state=1U,g_radar_zone_selected=2U;
volatile uint32_t g_can_test_result=1U,g_radar_lift_request,g_radar_lift_phase_bits,g_radar_lift_direction;
static uint32_t locked_zone, handled_measure, handled_path, handled_hello;
static uint32_t next_radar_ms,scan_reply_pending;
static AppHeadingSource hwt;
static float plant_yaw,plant_rate;
static double truth_x=150.0,truth_y=150.0,truth_heading=1.57079632679489661923;
static double driver_counts[4]={1000.0,1000.0,1000.0,1000.0};
static int32_t driver_decirpm[4];
static uint32_t plant_ms;
static const uint32_t idle_motion;
static const RouteHeadingReference route_heading;
static const AppOdometryInputs odometry_inputs={&g_vm_test_request,&idle_motion,&idle_motion,
    &idle_motion,&route_heading,&idle_motion};

typedef struct { uint8_t start,goal,count,nodes[2]; } PeerPath;
static const PeerPath wire_paths[]={
    {0,2,1,{2}}, {2,14,2,{4,14}}, {14,10,1,{10}}, {10,22,2,{12,22}},
    {22,14,2,{12,14}}, {22,0,2,{2,0}}
};
static const PeerPath detour_path={2,14,2,{12,14}};
static const double yellow_center[]={775.0,1625.0};

uint32_t RadarZoneSwitch_Lock(void) { return locked_zone=g_radar_zone_selected; }
void RadarZoneSwitch_Unlock(void) { locked_zone=0U; }
uint32_t App_HeadingSourceRead(AppHeadingSource *out) { *out=hwt; return 1U; }
void AppCanDiagnostics_ReadPositionSamples(ChassisOdometrySamples *samples)
{ *samples=(ChassisOdometrySamples){0}; }
void AppMotion_StepOdometry(void) { AppOdometry_Step(&odometry_inputs); }
#include "can_service_entry.c"

/* Distance along at least one separating axis for an oriented chassis and a square. */
static double box_clearance(double dx,double dy,double c,double s,double half_l,double half_w,double box_half)
{
    const double ac=fabs(c),as=fabs(s);
    double field_axes=fmax(fabs(dx)-(half_l*ac+half_w*as+box_half),
        fabs(dy)-(half_l*as+half_w*ac+box_half));
    double body_axes=fmax(fabs(c*dx+s*dy)-(half_l+box_half*(ac+as)),
        fabs(-s*dx+c*dy)-(half_w+box_half*(ac+as)));
    return fmax(field_axes,body_axes);
}

#if CHAIN_INTERVAL_CLEARANCE
enum { WALL_X_MIN, WALL_X_MAX, WALL_Y_MIN, WALL_Y_MAX, YELLOW_FIRST, CELL_5=8, FEATURE_COUNT };

volatile struct {
    double minimum_mm[FEATURE_COUNT];
    double final_return_mm;
    uint32_t count,nonpositive,worst_feature,start_ms,end_ms,phase,leg_index;
} chain_interval_clearance;

static const double half_pi=1.57079632679489661923;
static const double rad_per_degree=0.01745329251994329577;
static const double clearance_roundoff_mm=1e-9;

static double interval_support(double angle,double half_angle,double h_l,double h_w)
{
    const double peak=atan2(h_w,h_l);
    const double pi=2.0*half_pi;
    if (fabs(remainder(angle-peak,pi))<=half_angle || fabs(remainder(angle+peak,pi))<=half_angle)
        return hypot(h_l,h_w);
    const double lo=angle-half_angle,hi=angle+half_angle;
    return fmax(h_l*fabs(cos(lo))+h_w*fabs(sin(lo)),h_l*fabs(cos(hi))+h_w*fabs(sin(hi)));
}

/* Max gap on four fixed separating axes, with the same error bound on each axis. */
static double interval_box_gap(double dx,double dy,double c,double s,double half_side,const double ext[4])
{
    const double body_box_extent=half_side*(fabs(c)+fabs(s));
    return fmax(fmax(fabs(dx)-half_side-ext[0],fabs(dy)-half_side-ext[1]),
        fmax(fabs(c*dx+s*dy)-body_box_extent-ext[2],fabs(-s*dx+c*dy)-body_box_extent-ext[3]));
}

static void interval_clearance_step(uint32_t now,uint32_t dt,double x,double y,double heading,
    double c,double s,double center_travel,double yaw_travel,double h_l,double h_w)
{
    const double angle=yaw_travel+rad_per_degree*(double)s_config.heading_budget_deg;
    const double center=center_travel+(double)s_config.position_budget_mm+clearance_roundoff_mm;
    const double ext[4]={interval_support(heading,angle,h_l,h_w)+center,
        interval_support(heading-half_pi,angle,h_l,h_w)+center,
        interval_support(0.0,angle,h_l,h_w)+center,interval_support(-half_pi,angle,h_l,h_w)+center};
    const uint32_t features=CHAIN_OCCUPIED_GRID ? FEATURE_COUNT : CELL_5;
    double gap[FEATURE_COUNT];
    uint32_t i,j,worst=0U;
    gap[WALL_X_MIN]=x-ext[0];
    gap[WALL_X_MAX]=2400.0-x-ext[0];
    gap[WALL_Y_MIN]=y-ext[1];
    gap[WALL_Y_MAX]=2400.0-y-ext[1];
    for (i=0U;i<2U;i++) for (j=0U;j<2U;j++)
        gap[YELLOW_FIRST+2U*i+j]=interval_box_gap(x-yellow_center[i],y-yellow_center[j],c,s,225.0,ext);
    if (CHAIN_OCCUPIED_GRID) gap[CELL_5]=interval_box_gap(x-2160.0,y-720.0,c,s,240.0,ext);
    for (i=1U;i<features;i++) if (gap[i]<gap[worst]) worst=i;
    if (!chain_interval_clearance.count ||
        gap[worst]<chain_interval_clearance.minimum_mm[chain_interval_clearance.worst_feature]) {
        chain_interval_clearance.worst_feature=worst;
        chain_interval_clearance.start_ms=now-dt-1000U;
        chain_interval_clearance.end_ms=now-1000U;
        chain_interval_clearance.phase=g_zone2_phase;
        chain_interval_clearance.leg_index=s_schedule.leg;
    }
    for (i=0U;i<features;i++)
        if (!chain_interval_clearance.count || gap[i]<chain_interval_clearance.minimum_mm[i])
            chain_interval_clearance.minimum_mm[i]=gap[i];
    chain_interval_clearance.nonpositive+=gap[worst]<=0.0;
    chain_interval_clearance.count++;
}

static double final_return_clearance(void)
{
    const double angle=rad_per_degree*(double)s_config.heading_budget_deg;
    const double center=(double)s_config.position_budget_mm+clearance_roundoff_mm;
    const double h_l=0.5*(double)VEHICLE_BODY_LENGTH_MM,h_w=0.5*(double)VEHICLE_BODY_WIDTH_MM;
    const double ext_x=interval_support(truth_heading,angle,h_l,h_w)+center;
    const double ext_y=interval_support(truth_heading-half_pi,angle,h_l,h_w)+center;
    return fmin(fmin(truth_x,300.0-truth_x)-ext_x,fmin(truth_y,300.0-truth_y)-ext_y);
}
#endif

static void truth_step(uint32_t now,uint32_t dt)
{
    const double conversion=3.14159265358979323846*(double)g_chassis_geometry.wheel_diameter_mm/2400.0;
    const double a=(double)driver_decirpm[0],b=-(double)driver_decirpm[1];
    const double c=(double)driver_decirpm[2],d=-(double)driver_decirpm[3];
    const double forward=(a+b+c+d)*conversion*(double)CHASSIS_FORWARD_DISTANCE_SCALE;
    const double left=(a-b-c+d)*conversion;
    const double arm=((double)g_chassis_geometry.track_width_mm+g_chassis_geometry.wheelbase_mm)*0.5;
    const double omega=(-a+b-c+d)*conversion/arm;
    const double span=0.001*(double)dt;
    const double half=0.5*omega*span;
    const double mid=truth_heading+half;
    const double half_span=0.5*span;
    const double vx=forward*cos(mid)-left*sin(mid);
    const double vy=forward*sin(mid)+left*cos(mid);
    const double h_l=0.5*(double)VEHICLE_BODY_LENGTH_MM,h_w=0.5*(double)VEHICLE_BODY_WIDTH_MM;
    const double bound=(hypot(forward,left)+hypot(h_l,h_w)*fabs(omega))*half_span;
    const double quarter=0.5*half;
    const double sinc_half=fabs(quarter)<1e-9 ? 1.0 : sin(quarter)/quarter;
    const double middle_x=truth_x+(forward*cos(mid-quarter)-left*sin(mid-quarter))*half_span*sinc_half;
    const double middle_y=truth_y+(forward*sin(mid-quarter)+left*cos(mid-quarter))*half_span*sinc_half;
    const double cs=cos(mid),sn=sin(mid),ac=fabs(cs),as=fabs(sn);
    const double ext_x=ac*h_l+as*h_w,ext_y=as*h_l+ac*h_w;
    const double sinc=fabs(half)<1e-9 ? 1.0 : sin(half)/half;
    double gap=fmin(fmin(middle_x,2400.0-middle_x)-ext_x,
        fmin(middle_y,2400.0-middle_y)-ext_y);
    uint32_t i,j;
    for (i=0U;i<2U;i++) for (j=0U;j<2U;j++)
        gap=fmin(gap,box_clearance(middle_x-yellow_center[i],middle_y-yellow_center[j],cs,sn,h_l,h_w,225.0));
    if (CHAIN_OCCUPIED_GRID) {
        const double occupied_gap=box_clearance(middle_x-2160.0,middle_y-720.0,cs,sn,h_l,h_w,240.0)-bound;
        if (!chain_occupied_intervals || occupied_gap<chain_occupied_clearance)
            chain_occupied_clearance=(float)occupied_gap;
        chain_occupied_intervals++;
        chain_occupied_contacts+=occupied_gap<=0.0;
        gap=fmin(gap,occupied_gap+bound);
    }
    if (CHAIN_OCCUPIED_DETOUR && s_schedule.leg==1U && fabs(middle_x-1200.0)<15.0 &&
        fabs(middle_y-900.0)<10.0 && (!chain_detour_witness ||
        fabs(middle_y-900.0)<fabs((double)chain_detour_witness_y-900.0))) {
        chain_detour_witness=1U;
        chain_detour_witness_x=(float)middle_x;
        chain_detour_witness_y=(float)middle_y;
    }
    gap-=bound;
    if (!chain_truth_samples || gap<chain_truth_clearance) chain_truth_clearance=(float)gap;
    if (gap<=0.0) {
        if (!chain_truth_contacts) chain_first_contact_ms=now-1000U;
        chain_truth_contacts++;
    }
    gap-=(double)s_config.position_budget_mm+0.01745329251994329577*
        (double)s_config.heading_budget_deg*hypot(h_l,h_w);
    if (!chain_truth_samples || gap<chain_robust_clearance) chain_robust_clearance=(float)gap;
    chain_robust_contacts+=gap<=0.0;
#if CHAIN_INTERVAL_CLEARANCE
    interval_clearance_step(now,dt,middle_x,middle_y,mid,cs,sn,
        hypot(forward,left)*half_span,fabs(half),h_l,h_w);
#endif
    chain_truth_samples++;
    truth_x+=vx*span*sinc;
    truth_y+=vy*span*sinc;
    truth_heading+=2.0*half;
}

/* Ideal wheel encoders and simulated HWT; no target position enters the plant. */
static void advance_plant(uint32_t now)
{
    ChassisWheelSpeeds logical;
    ChassisBodyVelocity body;
    float rpm[4];
    const uint32_t dt=now-plant_ms;
    uint32_t i;
    if (!dt) return;
    truth_step(now,dt);
    for (i=0U;i<4U;i++) {
        rpm[i]=0.1f*(float)driver_decirpm[i];
        driver_counts[i]+=(double)driver_decirpm[i]*(double)dt*(65536.0/600000.0);
    }
    CHECK(Chassis_DriverFeedbackToLogical(rpm,15U,&logical));
    CHECK(Chassis_Forward(&g_chassis_geometry,&logical,&body));
    plant_rate=body.wz_rad_s*57.2957795f;
    plant_yaw+=plant_rate*0.001f*(float)dt;
    if (plant_yaw>180.0f) plant_yaw-=360.0f;
    if (plant_yaw<=-180.0f) plant_yaw+=360.0f;
    plant_ms=now;
}

static uint32_t plant_command(uint8_t address,uint8_t dir,uint16_t raw)
{
    advance_plant(test_tick);
    driver_decirpm[address-1U]=dir ? -(int32_t)raw : (int32_t)raw;
    chain_command_hash=(chain_command_hash^test_tick)*16777619U;
    chain_command_hash=(chain_command_hash^((uint32_t)address<<24)^((uint32_t)dir<<16)^raw)*16777619U;
    chain_nonzero+=raw!=0U;
    chain_post_budget_nonzero+=raw!=0U && test_tick-s_begin_ms>CHAIN_SCORING_BUDGET_MS;
    return 1U;
}

static void plant_feedback(uint8_t address,CAN1_EmmSystemFeedback *f)
{
    double n;
    advance_plant(test_tick);
    n=driver_counts[address-1U];
    f->position_sign=n<0.0;
    f->position_counts=(uint32_t)(fabs(n)+0.5);
    f->speed_decirpm=driver_decirpm[address-1U];
}

static uint32_t drop_one_reply(uint8_t address,uint32_t command)
{
    if (CHAIN_MISS_COMMAND!=command || chain_one_miss || !chain_nonzero || address!=3U) return 0U;
    chain_one_miss=1U;
    chain_one_miss_ms=test_tick-1000U;
    return 1U;
}

static uint32_t drop_moving_ack(uint8_t address,uint8_t dir,uint16_t raw,CAN1_EmmSpeedReceipt *receipt)
{
    if (!CHAIN_DROP_F6 || chain_partial || address!=3U || !raw) return 0U;
    chain_partial=1U;
    receipt->ack_valid=0U;
    (void)plant_command(address,dir,raw);
    return 1U;
}

static void drive_radar_peer(void)
{
    uint32_t before=command_count[LINK_CMD_PLAN_PATH];
    const uint8_t grid[6]={44,0,0,0,CHAIN_OCCUPIED_GRID ? 1U : 0U,5};
    if (CHAIN_STARTUP_TIMING && lift.state==1U && tick-lift_started_ms>=chain_lift_leg_ms) {
        lift.state=2U;
        lift.done=config.steps;
    }
    RadarObserver_Service();
    if (CHAIN_STARTUP_TIMING) {
        const uint32_t current=stage();
        if (current>=1U && current<=8U && !chain_scan_stage_ms[current-1U])
            chain_scan_stage_ms[current-1U]=tick-1000U;
        if (current==8U && !chain_scan_ready_ms) chain_scan_ready_ms=tick-1000U;
    }
    if (!CHAIN_DROP_LINK && command_count[LINK_CMD_HELLO]!=handled_hello) {
        handled_hello=command_count[LINK_CMD_HELLO];
        enqueue_hello(2U);
    }
    if (!CHAIN_STARTUP_TIMING && lift.state==1U) leg_complete();
    if (command_count[LINK_CMD_SINGLE_MEASURE]!=handled_measure) {
        handled_measure++;
        enqueue_ack(LINK_CMD_SINGLE_MEASURE);
        if (CHAIN_STARTUP_TIMING) chain_scan_tx_ms=tick-1000U;
        scan_reply_pending=1U;
    }
    if (scan_reply_pending && (!CHAIN_STARTUP_TIMING ||
        tick-1000U-chain_scan_tx_ms>=CHAIN_SCAN_DELAY_MS)) {
        queue(1U,LINK_EVT_OBSTACLE_GRID,grid,5U+grid[4]);
        queue(1U,LINK_EVT_STATUS,locked,sizeof(locked));
        if (CHAIN_STARTUP_TIMING) chain_scan_result_ms=tick-1000U;
        scan_reply_pending=0U;
    }
    if (command_count[LINK_CMD_PLAN_PATH]!=handled_path) {
        uint16_t seq=command_sequence[LINK_CMD_PLAN_PATH];
        uint8_t result[8]={0};
        if (handled_path>=sizeof(wire_paths)/sizeof(wire_paths[0])) {
            CHECK(0U);
            return;
        }
        const PeerPath *p=CHAIN_OCCUPIED_DETOUR && handled_path==1U ? &detour_path : &wire_paths[handled_path];
        handled_path++;
        CHECK(path_start==p->start && path_goal==p->goal);
        result[0]=(uint8_t)seq; result[1]=(uint8_t)(seq>>8);
        result[3]=p->start; result[4]=p->goal; result[5]=p->count;
        result[6]=p->nodes[0]; result[7]=p->nodes[1];
        enqueue_ack(LINK_CMD_PLAN_PATH);
        queue(1U,LINK_EVT_PATH_RESULT,result,6U+p->count);
    }
    CHECK(command_count[LINK_CMD_PLAN_PATH]<=before+1U);
    g_radar_servo_angle_deg[1]=(float)servo_angle;
}

static void run_slot(void)
{
    if (CHAIN_STARTUP_TIMING) {
        /* Equal ticks give CAN precedence. UART transmissions and IRQs are simulated. */
        for (;next_radar_ms<slot;next_radar_ms+=CHAIN_RADAR_PERIOD_MS) {
            tick=test_tick=next_radar_ms;
            drive_radar_peer();
        }
    }
    test_tick=slot;
    tick=slot;
    advance_plant(slot);
    hwt.sequence++;
    hwt.sample_ms=hwt.angle_ms=hwt.rate_ms=slot;
    hwt.yaw_deg=plant_yaw; hwt.rate_dps=plant_rate;
    hwt.healthy=1U; hwt.valid_mask=3U;
    if (!CHAIN_STARTUP_TIMING) drive_radar_peer();
    ChassisAuto_SafetyUpdate(g_vm_test_request);
    AppChassis_Zone2CanStep(slot,phase);
    if (CHAIN_MISS_COMMAND && chain_one_miss && !chain_owner_miss) {
        ChassisZone2CanStatus report;
        ChassisAuto_Zone2Read(&report);
        chain_owner_miss=CHAIN_MISS_COMMAND==0x43U ? report.wheel_misses[2] : report.status_misses[2];
    }
    CHECK(!test_depth);
    slot+=10U;
    phase=(phase+1U)%8U;
}

#if APP_ZONE2_NAV_MOTION_ENABLE
static void observe_track(void)
{
    AppMapPose map;
    FusionMotionVector ref;
    float err,heading,edge;
    const uint32_t leg=s_schedule.leg;
    uint32_t span_index,elapsed;
    chain_plan_ms[leg]=s_trajectory.duration_ms;
    chain_plan_sections[leg]=s_trajectory.section_count;
    if (g_zone2_phase!=ZONE2_TRACK && g_zone2_phase!=ZONE2_SECTION_ZERO) return;
    if (!chain_track_ms[leg]) chain_track_ms[leg]=s_section_ms-1000U;
    AppOdometry_ReadMap(&map);
    if (!map.pose.valid || !Zone2MotionReference_Sample(&s_trajectory,s_section,
        (int32_t)(map.pose.sample_ms-s_section_ms),&ref)) return;
    err=hypotf(map.pose.x_mm-ref.x,map.pose.y_mm-ref.y);
    heading=fabsf(remainderf(map.pose.heading_deg-ref.yaw*57.2957795f,360.0f));
    edge=fminf(fminf(ref.x,FIELD_SIZE_MM-ref.x),fminf(ref.y,FIELD_SIZE_MM-ref.y));
    chain_tracking_checks++;
    if (err>chain_max_error_mm) {
        const Zone2MotionSection *section=&s_trajectory.sections[s_section];
        chain_max_error_mm=err;
        chain_worst_leg=leg+1U;
        chain_worst_section=s_section;
        chain_worst_ms=map.pose.sample_ms-1000U;
        chain_worst_ref_x=ref.x; chain_worst_ref_y=ref.y;
        elapsed=map.pose.sample_ms-s_section_ms;
        if ((int32_t)elapsed<0) elapsed=0U;
        for (span_index=0U;span_index+1U<section->span_count;span_index++) {
            const Zone2MotionSpan *span=&s_trajectory.spans[section->first_span+span_index];
            if (elapsed<span->duration_ms) break;
            elapsed-=span->duration_ms;
        }
        chain_worst_span=s_trajectory.spans[section->first_span+span_index].kind;
    }
    chain_max_heading_deg=fmaxf(chain_max_heading_deg,heading);
    if (edge<=155.0f) {
        chain_edge_checks++;
        chain_edge_error_mm=fmaxf(chain_edge_error_mm,err);
    }
}
#endif

unsigned test_entry(void)
{
    AppMapPose map;
    ChassisZone2CanStatus can;
    uint32_t i,last_leg=0U,last_path=0U,pose_captured=0U;
    uint32_t nonzero_at_first_stop=0U;
    ChassisAuto_Init();
    AppOdometry_Init();
    g_safety_ready=g_can_guard_ready=1U;
    epoch=1U;
    slot=test_tick=tick=plant_ms=1000U;
    if (CHAIN_STARTUP_TIMING) {
        next_radar_ms=slot;
        chain_lift_leg_ms=(config.steps*RADAR_LIFT_INTERVAL_US+999U)/1000U;
    }
    servo_angle=180U;
    CHECK(RadarObserver_Configure(&config));
    RadarObserver_Service();
    handshake();
    handled_hello=command_count[LINK_CMD_HELLO];
    AppZone2_RequestStart();
    for (i=0U;i<CHAIN_MAX_SLOTS && g_zone2_phase!=ZONE2_FAILED;i++) {
        run_slot();
#if APP_ZONE2_NAV_MOTION_ENABLE
        if (s_schedule.leg<8U && s_schedule.phase==ZONE2_SCHEDULE_ACTIVE) observe_track();
#endif
        if (s_schedule.leg!=last_leg) {
            CHECK(s_schedule.leg==last_leg+1U);
            CHECK(stops==4U*s_schedule.leg);
            chain_reuses+=(handled_path==last_path);
            last_path=handled_path;
            last_leg=s_schedule.leg;
            if (CHAIN_OCCUPIED_BAD_PATH && last_leg==1U) nonzero_at_first_stop=chain_nonzero;
            if (last_leg<=8U) chain_arrival_ms[last_leg-1U]=slot-1000U;
            AppOdometry_ReadMap(&map);
            pose_captured=1U;
        }
        if (s_schedule.phase==ZONE2_SCHEDULE_DONE) break;
#if !APP_ZONE2_NAV_MOTION_ENABLE
        if (g_zone2_phase==ZONE2_SHADOW_DONE) break;
#endif
    }
    if (!pose_captured || s_schedule.phase!=ZONE2_SCHEDULE_DONE) AppOdometry_ReadMap(&map);
    ChassisAuto_Zone2Read(&can);
    chain_stop_legs=s_schedule.leg;
    chain_path_requests=handled_path;
    chain_fault=g_zone2_fault;
    chain_cause=can.cause;
    chain_reference=g_zone2_reference_status;
    chain_path_error=g_zone2_path_error;
    chain_f6_possible=can.possible_mask;chain_f6_tx=can.tx_complete_mask;chain_f6_ack=can.ack_mask;
    chain_stop_frames=stops;
    chain_runtime_ms=slot-1000U;
    chain_start_ms=s_begin_ms-1000U;
    chain_scoring_budget_end_ms=chain_start_ms+CHAIN_SCORING_BUDGET_MS;
    chain_outcome_ms=test_tick-1000U;
    chain_backend=can.state;
    chain_sends=sends;
    chain_shadow=(s_schedule.phase!=ZONE2_SCHEDULE_DONE && g_zone2_fault==0U);
    chain_map_state=map.state;
    chain_map_reasons=map.pose.reasons;
    chain_map_updates=map.pose.updates;
    chain_x=map.pose.x_mm;chain_y=map.pose.y_mm;chain_heading=map.pose.heading_deg;
    chain_truth_x=(float)truth_x;chain_truth_y=(float)truth_y;
    chain_truth_yaw=(float)(truth_heading*57.2957795130823208768);
#if CHAIN_INTERVAL_CLEARANCE
    if (s_schedule.phase==ZONE2_SCHEDULE_DONE && s_schedule.leg==8U && !g_zone2_fault)
        chain_interval_clearance.final_return_mm=final_return_clearance();
#endif
    if (CHAIN_DROP_LINK || CHAIN_DROP_F6) {
        uint32_t old_sends=sends;
        CHECK(g_zone2_phase==ZONE2_FAILED && s_schedule.leg==0U && handled_path==1U && chain_nonzero);
        if (CHAIN_DROP_LINK) {
            CHECK(g_zone2_fault==ZONE2_FAULT_PATH && g_zone2_path_error==ZONE2_NAV_STALE_MAP);
        } else {
            CHECK(chain_partial && g_zone2_fault==ZONE2_FAULT_BACKEND && can.cause==4U &&
                can.possible_mask==7U && can.tx_complete_mask==7U && can.ack_mask==3U);
        }
        for (i=0U;i<220U;i++) {
            run_slot();
            ChassisAuto_Zone2Read(&can);
            if (can.wheel_quiet) chain_recovery_quiet=1U;
            if (!g_chassis_auto_active && !g_vm_test_request) break;
        }
        chain_recovery_ms=slot-1000U;
        chain_recovery_stops=stops;
        CHECK(stops==4U && sends==old_sends &&
            !g_chassis_auto_active && !g_vm_test_request && !locked_zone);
    }
#if CHAIN_OCCUPIED_BAD_PATH
    CHECK(g_zone2_phase==ZONE2_FAILED && g_zone2_fault==ZONE2_FAULT_PATH &&
        g_zone2_path_error==ZONE2_NAV_OCCUPIED && chain_stop_legs==1U &&
        handled_path==2U && stops==4U && nonzero_at_first_stop);
    for (i=0U;i<220U;i++) run_slot();
    chain_occupied_reject_nonzero_after_first_stop=chain_nonzero-nonzero_at_first_stop;
    CHECK(!chain_occupied_reject_nonzero_after_first_stop && stops==4U &&
        !g_chassis_auto_active && !g_vm_test_request && !locked_zone);
#endif
    CHECK(!audit_failures);
    CHECK(command_count[LINK_CMD_SINGLE_MEASURE]==1U);
    CHECK(chain_one_miss==(CHAIN_MISS_COMMAND!=0U) && chain_owner_miss==(CHAIN_MISS_COMMAND!=0U));
#if CHAIN_OCCUPIED_GRID
    CHECK(chain_occupied_intervals && !chain_occupied_contacts && chain_occupied_clearance>0.0f);
#endif
#if CHAIN_OCCUPIED_DETOUR
    CHECK(chain_detour_witness);
#endif
#if !CHAIN_DROP_LINK && !CHAIN_DROP_F6 && !CHAIN_OCCUPIED_BAD_PATH
#if CHAIN_STARTUP_TIMING
    CHECK(chain_lift_leg_ms>=3079U && chain_scan_stage_ms[0] && chain_scan_tx_ms &&
        chain_scan_result_ms-chain_scan_tx_ms>=CHAIN_SCAN_DELAY_MS &&
        chain_scan_ready_ms>chain_scan_result_ms);
#endif
    CHECK(!g_zone2_fault && !g_vm_test_request && !locked_zone);
#if APP_ZONE2_NAV_MOTION_ENABLE
    CHECK(s_schedule.phase==ZONE2_SCHEDULE_DONE);
    CHECK(chain_stop_legs==8U && handled_path==6U && chain_reuses==2U && stops==32U);
    CHECK(chain_nonzero && map.pose.updates>100U && fabsf(chain_x-150.0f)<=15.0f &&
        fabsf(chain_y-150.0f)<=15.0f && fabsf(chain_heading-90.0f)<=1.0f);
    CHECK(chain_truth_samples>100U && !chain_truth_contacts && chain_truth_clearance>0.0f);
    CHECK(Zone2NavPath_InsideReturn((FusionPoint){chain_truth_x,chain_truth_y},chain_truth_yaw,
        s_config.position_budget_mm,s_config.heading_budget_deg,1U));
#else
    CHECK(g_zone2_phase==ZONE2_SHADOW_DONE && g_zone2_stop_pose_ms);
    CHECK(handled_path==1U && stops==4U && !chain_nonzero && map.pose.updates);
#endif
#if CHAIN_STARTUP_TIMING
    CHECK(g_zone2_phase==ZONE2_DONE);
#endif
#if CHAIN_REQUIRE_POST_BUDGET_MOTION
    CHECK(chain_outcome_ms>chain_scoring_budget_end_ms && chain_post_budget_nonzero);
#endif
#endif
    replay_assertions=checks+audit_checks;
    replay_case=g_zone2_phase;
    replay_detail=s_schedule.phase;
    replay_actual=(float)g_zone2_fault;
    replay_expected=(CHAIN_DROP_LINK || CHAIN_OCCUPIED_BAD_PATH) ? (float)ZONE2_FAULT_PATH :
        (CHAIN_DROP_F6 ? (float)ZONE2_FAULT_BACKEND : 0.0f);
    return first_failure;
}
