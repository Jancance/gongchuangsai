#include "app_config.h"

#if APP_ZONE2_NAV_ENABLE
#include "app_zone2.h"
#include "app_can_diagnostics.h"
#include "app_odometry.h"
#include "can.h"
#include "chassis_auto_test.h"
#include "freertos_hooks.h"
#include "heading_source.h"
#include "radar_lift.h"
#include "radar_servo.h"
#include "radar_zone_switch.h"
#include "safety.h"
#include "vehicle_tuning.h"
#include "zone2_nav_schedule.h"
#if APP_ZONE2_NAV_MOTION_ENABLE
#include "zone2_motion_reference.h"
#endif
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

#define ZONE2_RUN_ID 1U
#define ZONE2_START_ID 2U
#define ZONE2_WAIT_READY_MS 15000U
#define ZONE2_POSE_MS 160U
#define ZONE2_DEG_TO_RAD 0.01745329251994329577f
#define ZONE2_SECTION_SETTLE_MS 2000U

typedef enum {
    ZONE2_IDLE, ZONE2_WAIT_READY, ZONE2_SCAN, ZONE2_PREPARE,
    ZONE2_ZERO, ZONE2_STOP, ZONE2_SHADOW_DONE, ZONE2_FAILED,
    ZONE2_TRACK, ZONE2_SECTION_ZERO, ZONE2_DONE
} Zone2Phase;

typedef enum {
    ZONE2_FAULT_NONE, ZONE2_FAULT_HEALTH, ZONE2_FAULT_DEADLINE,
    ZONE2_FAULT_PATH, ZONE2_FAULT_BACKEND, ZONE2_FAULT_SOURCE, ZONE2_FAULT_REFERENCE
} Zone2Fault;

static const FusionConfig s_config = {
    .position_budget_mm=5.0f, .heading_budget_deg=1.0f,
    .position_tolerance_mm=5.0f, .max_error_mm=5.0f,
    .stop_speed_mm_s=1.0f, .stop_omega_deg_s=1.0f,
    .acceleration_hint_mm_s2=150.0f, .cross_gain_s=1.0f,
    .lateral_cap_mm_s=40.0f, .preview_s=0.05f,
    .wheel_limit_rpm=300.0f, .heading_reserve_rpm=15.0f,
    .speed_state_margin_mm_s=5.0f, .max_age_ms=ZONE2_POSE_MS
};
static const ChassisMotorCommand s_zero[4] = {
    {1U,0U,0U}, {2U,1U,0U}, {3U,0U,0U}, {4U,1U,0U}
};

static volatile uint32_t s_start_request;
static Zone2NavSchedule s_schedule;
#if APP_ZONE2_NAV_MOTION_ENABLE
static Zone2MotionReference s_trajectory;
static FusionMotionVector s_correction, s_proposed;
static uint32_t s_section, s_section_ms, s_generation, s_station_stop, s_prepare_ms;
volatile uint32_t g_zone2_completed_legs;
#else
static FusionState s_reference;
#endif
static uint32_t s_begin_ms, s_session, s_power_epoch, s_primed, s_zone_locked;
static uint32_t s_stop_ms;
volatile Zone2Phase g_zone2_phase;
volatile Zone2Fault g_zone2_fault;
volatile Zone2NavPathResult g_zone2_path_error;
volatile uint32_t g_zone2_reference_status, g_zone2_stop_pose_ms;

static uint32_t now_ms(void)
{
    return (uint32_t)xTaskGetTickCount()*portTICK_PERIOD_MS;
}

static uint32_t newer(uint32_t stamp,uint32_t anchor)
{
    return (int32_t)(stamp-anchor)>0;
}

static uint32_t healthy(void)
{
    return !g_safety_fault && !g_rtos_fault && !g_can_ctrl_fault_latched &&
        g_can_test_result<=1U && !g_chassis_auto_cut_vm;
}

/* PWM command and released coils only. The lift has no stowed position sensor. */
static uint32_t stowed_command(void)
{
    RadarLiftSnapshot lift;
    RadarLift_GetSnapshot(&lift);
    return g_radar_servo_state==1U && g_radar_servo_angle_deg[1]==180.0f &&
        !g_radar_lift_request && !g_radar_lift_phase_bits && !g_radar_lift_enable &&
        (lift.state==0U || (lift.state==2U && lift.result==1U && g_radar_lift_direction==0U));
}

static void fail(Zone2Fault reason)
{
    g_zone2_fault=reason;
    g_zone2_path_error=s_schedule.error;
    Zone2NavSchedule_Abort(&s_schedule);
    g_zone2_phase=ZONE2_FAILED;
    if (s_primed) (void)ChassisAuto_Zone2Disarm();
    if (s_zone_locked) RadarZoneSwitch_Unlock();
}

static uint32_t current_map(void)
{
    RadarObserverMap map;
    const RadarObserverMapIdentity *expected=&s_schedule.map_identity;
    return RadarObserver_GetMap(ZONE2_RUN_ID,&map) && map.valid==1U &&
        map.occupied_mask==s_schedule.map.occupied_mask &&
        map.identity.run_id==expected->run_id &&
        map.identity.start_id==expected->start_id &&
        map.identity.layout_id==expected->layout_id &&
        map.identity.map_generation==expected->map_generation &&
        map.identity.link_generation==expected->link_generation &&
        map.identity.scan_sequence==expected->scan_sequence &&
        map.identity.zone==expected->zone && map.identity.node_id==expected->node_id;
}

static uint32_t sources(const ChassisZone2CanStatus *can,uint32_t *now,
    AppMapPose *map,AppHeadingSource *heading)
{
    AppOdometry_ReadMap(map);
    if (!App_HeadingSourceRead(heading)) return 0U;
    *now=now_ms();
    return can->run_id==ZONE2_RUN_ID && can->power_epoch && can->feedback_ready &&
        (!s_power_epoch || can->power_epoch==s_power_epoch) &&
        map->run_id==ZONE2_RUN_ID && map->start_id==ZONE2_START_ID &&
        map->map_version==VEHICLE_MAP_VERSION && map->state==APP_MAP_TRACKING &&
        map->based && map->pose.valid && !map->pose.local_origin &&
        !map->pose.continuity_lost && !map->pose.reasons &&
        *now-map->pose.sample_ms<=ZONE2_POSE_MS && heading->healthy &&
        App_HeadingFrameAge(heading,*now,1U)<ODOM_YAW_MAX_AGE_MS &&
        App_HeadingFrameAge(heading,*now,2U)<ODOM_YAW_MAX_AGE_MS &&
        fabsf(heading->rate_dps)<=360.0f;
}

static uint32_t observation(const ChassisZone2CanStatus *can,uint32_t now,
    FusionObservation *o,AppHeadingSource *heading)
{
    AppMapPose map;
    ChassisWheelSpeeds wheels;
    ChassisBodyVelocity body;
    float driver_rpm[4], c, s;
    uint32_t i;

    if (!sources(can,&now,&map,heading)) return 0U;
    for (i=0U;i<4U;i++) driver_rpm[i]=0.1f*(float)can->wheel_decirpm[i];
    if (!Chassis_DriverFeedbackToLogical(driver_rpm,can->position_mask,&wheels) ||
        !Chassis_Forward(&g_chassis_geometry,&wheels,&body)) return 0U;
    c=cosf(map.pose.heading_deg*ZONE2_DEG_TO_RAD);
    s=sinf(map.pose.heading_deg*ZONE2_DEG_TO_RAD);
    *o=(FusionObservation){0};
    if (s_schedule.plan.valid) o->identity=s_schedule.plan.input.identity;
    else o->identity=(FusionIdentity){ZONE2_RUN_ID,ZONE2_START_ID,
        s_schedule.map_identity.map_generation,VEHICLE_MAP_VERSION,0U,0U};
    o->position=(FusionPoint){map.pose.x_mm,map.pose.y_mm};
    o->heading_deg=map.pose.heading_deg;
    o->vx_mm_s=c*body.vx_mm_s-s*body.vy_mm_s;
    o->vy_mm_s=s*body.vx_mm_s+c*body.vy_mm_s;
    o->omega_deg_s=heading->rate_dps;
    o->sample_ms=map.pose.sample_ms;
    o->now_ms=now;
    o->valid=1U;
    o->feedback_source=FUSION_MEASURED_FEEDBACK;
    return 1U;
}

static uint32_t quiet_since(const FusionObservation *pose,const AppHeadingSource *heading,
    uint32_t anchor)
{
    return newer(pose->sample_ms,anchor) && newer(heading->angle_ms,anchor) &&
        newer(heading->rate_ms,anchor) && fabsf(pose->omega_deg_s)<=s_config.stop_omega_deg_s &&
        hypotf(pose->vx_mm_s,pose->vy_mm_s)<=s_config.stop_speed_mm_s;
}

#if APP_ZONE2_NAV_MOTION_ENABLE
static void start_section(uint32_t batch_ms)
{
    s_section_ms=batch_ms;
    s_generation=0U;
    s_correction=(FusionMotionVector){0};
    g_zone2_phase=ZONE2_TRACK;
}

static uint32_t submit_zero(const ChassisZone2CanStatus *can)
{
    return ChassisAuto_Zone2SubmitAt(ZONE2_RUN_ID,s_session,can->ack_generation+1U,
        can->next_batch_ms,s_zero);
}

static void motion_batch(const ChassisZone2CanStatus *can,const FusionObservation *pose)
{
    FusionMotionFeedbackInput in={0};
    FusionMotionFeedbackBatch out;
    int32_t sample=(int32_t)(pose->sample_ms-s_section_ms);
    int32_t next=(int32_t)(can->next_batch_ms-s_section_ms);
    int32_t end=(int32_t)s_trajectory.sections[s_section].duration_ms;
    uint32_t i;

    if (s_generation) {
        if (can->ack_generation!=s_generation || can->ack_mask!=0x0FU ||
            can->tx_complete_mask!=0x0FU) {
            fail(ZONE2_FAULT_BACKEND);
            return;
        }
        s_correction=s_proposed;
        s_generation=0U;
    }
    if (next>end+(int32_t)ZONE2_SECTION_SETTLE_MS ||
        !Zone2MotionReference_Sample(&s_trajectory,s_section,sample,&in.reference)) {
        fail(ZONE2_FAULT_REFERENCE);
        return;
    }
    in.measured=(FusionMotionVector){pose->position.x_mm,pose->position.y_mm,pose->heading_deg*ZONE2_DEG_TO_RAD};
    in.committed=s_correction;
    in.reference_complete=(int32_t)(pose->now_ms-s_section_ms)>=end;
    in.capture_complete=sample>=end;
    for (i=0U;i<CHASSIS_WHEEL_COUNT;i++) {
        FusionMotionVector from,to;
        int32_t slot=next+(int32_t)i*20;
        if (!Zone2MotionReference_Sample(&s_trajectory,s_section,slot,&from) ||
            !Zone2MotionReference_Sample(&s_trajectory,s_section,slot+80,&to)) {
            fail(ZONE2_FAULT_REFERENCE);
            return;
        }
        in.slot_from[i]=(FusionMotionPoint){from.x,from.y};
        in.slot_to[i]=(FusionMotionPoint){to.x,to.y};
        in.slot_yaw_delta[i]=remainderf(to.yaw-from.yaw,6.28318530717958648f);
    }
    if (!FusionMotion_FeedbackBatch(&in,&out)) {
        fail(ZONE2_FAULT_REFERENCE);
        return;
    }
    if (!ChassisAuto_Zone2SubmitAt(ZONE2_RUN_ID,s_session,can->ack_generation+1U,
        can->next_batch_ms,out.commands)) {
        fail(ZONE2_FAULT_BACKEND);
        return;
    }
    s_generation=can->ack_generation+1U;
    s_proposed=out.correction;
    g_zone2_reference_status=FUSION_FOLLOW;
    if (out.finish_requested) g_zone2_phase=ZONE2_SECTION_ZERO;
}

static void advance_after_stop(ChassisZone2CanStatus *can,FusionObservation *pose,uint32_t now)
{
    FusionMotionVector endpoint;
    if (!s_station_stop) {
        fail(ZONE2_FAULT_SOURCE);
        return;
    }
    if (!Zone2MotionReference_Sample(&s_trajectory,s_section,
        (int32_t)s_trajectory.sections[s_section].duration_ms,&endpoint)) {
        fail(ZONE2_FAULT_REFERENCE);
        return;
    }
    if (!(hypotf(pose->position.x_mm-endpoint.x,pose->position.y_mm-endpoint.y)<=
        s_config.position_tolerance_mm) ||
        !(fabsf(remainderf(pose->heading_deg-endpoint.yaw/ZONE2_DEG_TO_RAD,360.0f))<=
        s_config.heading_budget_deg)) {
        fail(ZONE2_FAULT_REFERENCE);
        return;
    }
    pose->stopped=1U;
    if (!Zone2NavSchedule_Arrive(&s_schedule,s_session,true,pose,now)) {
        fail(ZONE2_FAULT_PATH);
        return;
    }
    g_zone2_completed_legs=s_schedule.leg;
    g_zone2_stop_pose_ms=pose->sample_ms;
    if (s_schedule.phase==ZONE2_SCHEDULE_DONE) {
        RadarObserver_Cancel();
        (void)ChassisAuto_Zone2Disarm();
        s_primed=0U;
        RadarZoneSwitch_Unlock();
        s_zone_locked=0U;
        g_zone2_phase=ZONE2_DONE;
        g_zone2_reference_status=FUSION_DONE;
    } else if (ChassisAuto_Zone2ReleaseCompleted(ZONE2_RUN_ID,can->session)) {
        s_prepare_ms=now;
        g_zone2_phase=ZONE2_PREPARE;
    } else {
        fail(ZONE2_FAULT_BACKEND);
    }
}
#endif

void AppZone2_RequestStart(void)
{
    taskENTER_CRITICAL();
    if (g_zone2_phase==ZONE2_IDLE) s_start_request=1U;
    taskEXIT_CRITICAL();
}

uint32_t AppZone2_CanPermit(uint32_t slot_ms)
{
    ChassisZone2CanStatus can;
    AppHeadingSource heading;
    AppMapPose map;
    uint32_t now=now_ms();
    (void)slot_ms;
    if ((g_zone2_phase!=ZONE2_ZERO && g_zone2_phase!=ZONE2_TRACK &&
        g_zone2_phase!=ZONE2_SECTION_ZERO) || !healthy() || !current_map()) return 0U;
    ChassisAuto_Zone2Read(&can);
    return can.session==s_session && sources(&can,&now,&map,&heading);
}

void AppZone2_Step(void)
{
    ChassisZone2CanStatus can;
    ChassisAutoStatus backend;
    FusionObservation pose;
#if !APP_ZONE2_NAV_MOTION_ENABLE
    FusionOutput desired;
#endif
    AppHeadingSource heading;
    Zone2SchedulePhase schedule_phase;
    uint32_t now=now_ms(), observed;

    if (g_zone2_phase==ZONE2_IDLE) {
        taskENTER_CRITICAL();
        if (s_start_request) {
            s_start_request=0U;
            s_begin_ms=now;
            g_zone2_phase=ZONE2_WAIT_READY;
        }
        taskEXIT_CRITICAL();
    }
    if (g_zone2_phase==ZONE2_FAILED) {
        if (s_primed && !g_chassis_auto_active) {
            (void)ChassisAuto_Zone2Disarm();
            s_primed=0U;
        }
        return;
    }
    if (g_zone2_phase==ZONE2_IDLE || g_zone2_phase==ZONE2_SHADOW_DONE || g_zone2_phase==ZONE2_DONE) return;
    if (!healthy()) {
        fail(ZONE2_FAULT_HEALTH);
        return;
    }
    if (g_zone2_phase==ZONE2_WAIT_READY) {
        if (g_can_test_result==1U && g_safety_ready && g_radar_zone_selected==ZONE2_START_ID &&
            stowed_command() && RadarObserver_IsScanReady()) {
            s_zone_locked=RadarZoneSwitch_Lock();
            if (s_zone_locked==ZONE2_START_ID && Zone2NavSchedule_Start(&s_schedule,
                ZONE2_RUN_ID,ZONE2_START_ID,VEHICLE_MAP_VERSION,true,&s_config,400.0f)) {
                g_zone2_phase=ZONE2_SCAN;
                return;
            }
            if (s_zone_locked) RadarZoneSwitch_Unlock();
            s_zone_locked=0U;
        }
        if (now-s_begin_ms>=ZONE2_WAIT_READY_MS) fail(ZONE2_FAULT_DEADLINE);
        return;
    }
    ChassisAuto_Zone2Read(&can);
    observed=observation(&can,now,&pose,&heading);
    if (observed) now=pose.now_ms;
    schedule_phase=Zone2NavSchedule_Poll(&s_schedule,observed?&pose:0,now);
    if (schedule_phase==ZONE2_SCHEDULE_FAILED) {
        fail(ZONE2_FAULT_PATH);
        return;
    }
    if (g_zone2_phase==ZONE2_SCAN && schedule_phase!=ZONE2_SCHEDULE_SCAN) {
        if (!stowed_command() || !ChassisAuto_Zone2Prime(ZONE2_RUN_ID)) {
            fail(ZONE2_FAULT_BACKEND);
            return;
        }
        s_primed=1U;
        AppOdometry_RequestMapRun(ZONE2_RUN_ID,ZONE2_START_ID);
#if APP_ZONE2_NAV_MOTION_ENABLE
        s_prepare_ms=now;
#endif
        g_zone2_phase=ZONE2_PREPARE;
        return;
    }
#if APP_ZONE2_NAV_MOTION_ENABLE
    if (g_zone2_phase==ZONE2_PREPARE && now-s_prepare_ms>=ZONE2_WAIT_READY_MS) {
        fail(ZONE2_FAULT_DEADLINE);
        return;
    }
#endif
    if (g_zone2_phase==ZONE2_PREPARE && observed && schedule_phase==ZONE2_SCHEDULE_READY) {
        pose.identity=s_schedule.plan.input.identity;
#if APP_ZONE2_NAV_MOTION_ENABLE
        if (!Zone2MotionReference_Prepare(&s_schedule.plan,&s_trajectory)) {
            fail(ZONE2_FAULT_REFERENCE);
            return;
        }
        s_section=0U;
        s_station_stop=0U;
        g_zone2_reference_status=FUSION_TURN_START;
#else
        g_zone2_reference_status=FusionReference_Step(&s_schedule.plan,&s_reference,&pose,&desired);
        if (g_zone2_reference_status!=FUSION_TURN_START) {
            fail(ZONE2_FAULT_REFERENCE);
            return;
        }
#endif
        if (!ChassisAuto_ReadStatus(&backend) || backend.active ||
            !ChassisAuto_Zone2Begin(ZONE2_RUN_ID,backend.session)) {
            fail(ZONE2_FAULT_BACKEND);
            return;
        }
        ChassisAuto_Zone2Read(&can);
        s_session=can.session;
        s_power_epoch=can.power_epoch;
        if (!Zone2NavSchedule_BeginLeg(&s_schedule,&pose.identity,s_session,now)) {
            fail(ZONE2_FAULT_PATH);
            return;
        }
        g_zone2_phase=ZONE2_ZERO;
        return;
    }
    if (g_zone2_phase!=ZONE2_ZERO && g_zone2_phase!=ZONE2_STOP &&
        g_zone2_phase!=ZONE2_TRACK && g_zone2_phase!=ZONE2_SECTION_ZERO) return;
    if (can.session!=s_session || can.power_epoch!=s_power_epoch ||
        can.state==CHASSIS_AUTO_FAULT || can.state==CHASSIS_AUTO_CANCELLED || can.cause) {
        fail(ZONE2_FAULT_BACKEND);
        return;
    }
    /* A CAN-slot source veto can request STOP before the front end sees it. */
    if (g_zone2_phase!=ZONE2_STOP && (can.state==CHASSIS_AUTO_STOP ||
        can.state==CHASSIS_AUTO_SETTLE || can.state==CHASSIS_AUTO_DONE)) {
        s_stop_ms=now;
        g_zone2_phase=ZONE2_STOP;
    }
#if APP_ZONE2_NAV_MOTION_ENABLE
    if (can.ready && (g_zone2_phase==ZONE2_ZERO || g_zone2_phase==ZONE2_TRACK ||
        g_zone2_phase==ZONE2_SECTION_ZERO)) {
        if (!observed) {
            fail(ZONE2_FAULT_SOURCE);
            return;
        }
        if (g_zone2_phase!=ZONE2_TRACK && can.ack_generation && can.ack_mask==0x0FU &&
            can.zero_confirmed && quiet_since(&pose,&heading,can.zero_ack_ms)) {
            if (g_zone2_phase==ZONE2_SECTION_ZERO) {
                FusionMotionVector endpoint;
                if (can.ack_generation!=s_generation) {
                    fail(ZONE2_FAULT_BACKEND);
                    return;
                }
                if (!Zone2MotionReference_Sample(&s_trajectory,s_section,
                    (int32_t)s_trajectory.sections[s_section].duration_ms,&endpoint)) {
                    fail(ZONE2_FAULT_REFERENCE);
                    return;
                }
                if (hypotf(pose.position.x_mm-endpoint.x,pose.position.y_mm-endpoint.y)>2.5f ||
                    fabsf(remainderf(pose.heading_deg-endpoint.yaw/ZONE2_DEG_TO_RAD,360.0f))>.10f) {
                    s_generation=0U;
                    s_correction=(FusionMotionVector){0};
                    g_zone2_phase=ZONE2_TRACK;
                    motion_batch(&can,&pose);
                    return;
                }
                if (s_trajectory.sections[s_section].terminal) {
                    if (!ChassisAuto_RequestStop()) {
                        fail(ZONE2_FAULT_BACKEND);
                        return;
                    }
                    s_station_stop=1U;
                    s_stop_ms=now;
                    g_zone2_phase=ZONE2_STOP;
                    return;
                }
                s_section++;
            }
            start_section(can.next_batch_ms);
        }
        if (g_zone2_phase==ZONE2_TRACK) {
            motion_batch(&can,&pose);
        } else if (g_zone2_phase==ZONE2_SECTION_ZERO &&
            now-s_section_ms>s_trajectory.sections[s_section].duration_ms+ZONE2_SECTION_SETTLE_MS) {
            fail(ZONE2_FAULT_DEADLINE);
        } else if (!submit_zero(&can)) {
            fail(ZONE2_FAULT_BACKEND);
        } else if (g_zone2_phase==ZONE2_SECTION_ZERO) {
            s_generation=can.ack_generation+1U;
        }
    }
#else
    if (g_zone2_phase==ZONE2_ZERO && can.ready) {
        if (!observed) {
            fail(ZONE2_FAULT_SOURCE);
            return;
        }
        if (can.ack_generation && can.ack_mask==0x0FU && can.zero_confirmed &&
            quiet_since(&pose,&heading,can.zero_ack_ms)) {
            if (!ChassisAuto_RequestStop()) {
                fail(ZONE2_FAULT_BACKEND);
                return;
            }
            s_stop_ms=now;
            g_zone2_phase=ZONE2_STOP;
        } else if (!ChassisAuto_Zone2Submit(ZONE2_RUN_ID,s_session,can.ack_generation+1U,s_zero)) {
            fail(ZONE2_FAULT_BACKEND);
            return;
        }
    }
#endif
    if (g_zone2_phase==ZONE2_STOP) {
        if (now-s_stop_ms>=CHASSIS_AUTO_STOP_MS+CHASSIS_AUTO_SETTLE_MS) {
            fail(ZONE2_FAULT_DEADLINE);
        } else if (can.state==CHASSIS_AUTO_DONE && can.result==1U &&
            can.ack_generation && can.wheel_quiet && current_map() &&
            observed && quiet_since(&pose,&heading,can.stop_ack_ms)) {
#if APP_ZONE2_NAV_MOTION_ENABLE
            advance_after_stop(&can,&pose,now);
#else
            g_zone2_stop_pose_ms=pose.sample_ms;
            RadarObserver_Cancel();
            (void)ChassisAuto_Zone2Disarm();
            RadarZoneSwitch_Unlock();
            g_zone2_phase=ZONE2_SHADOW_DONE;
#endif
        }
    }
}
#endif
