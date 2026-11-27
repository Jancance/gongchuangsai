#include "app_odometry.h"
#include "vehicle_tuning.h"
#include "FreeRTOS.h"
#include "task.h"
#include "freertos_hooks.h"
#include "heading_source.h"
#include "chassis_auto_test.h"
#include "power.h"
#include "safety.h"
#include "can.h"
#include <string.h>
#include <math.h>
#if APP_CONTINUOUS_MAP_ENABLE
#include "app_can_diagnostics.h"
#endif

#if APP_NAV_RESIDUAL_ACTIVE || APP_MAP_NAV_RESIDUAL_ENABLE
#include "app_motion_runtime.h"
#include "motion_control.h"
#include "app_entry.h"
#include <math.h>
#endif
#if APP_CONTINUOUS_MAP_ENABLE
static uint32_t s_map_run;
#endif
#if APP_MAP_NAV_RESIDUAL_ENABLE
typedef struct {uint32_t epoch,run,begin,direction,scope;} MapAction;
typedef struct {
    uint32_t epoch,based,active;
    float x,y,cs,sn,history_x,history_y;
} MapNavigation;
static MapAction s_map_action __attribute__((section(".bss.map_navigation"),zero_init));
static MapNavigation s_map_navigation __attribute__((section(".bss.map_navigation"),zero_init));
#endif
void AppOdometry_MapActionAccepted(uint32_t direction,uint32_t rpm,float distance_mm,float route_speed)
{
#if APP_MAP_NAV_RESIDUAL_ENABLE
    uint32_t scope=APP_MAP_CAL_EXTRAPOLATED;
    if(APP_BASELINE40_ACTIVE && !APP_CONTROL_DISTANCE_RATIO_ENABLE && distance_mm==2100.0f && route_speed==0.0f &&
       AppDebugPlan40Allowed(g_debug_plan_id,direction,rpm,distance_mm) &&
       ((direction==1U && APP_LEFT40_DRIFT_TRIM_ENABLE) ||
        (direction==2U && APP_RIGHT40_FORWARD_TRIM_ENABLE) || direction==0U || direction==3U))
        scope=APP_MAP_CAL_REFERENCE;
    taskENTER_CRITICAL();
    if(++s_map_action.epoch==0U) ++s_map_action.epoch;
    s_map_action.begin=(uint32_t)xTaskGetTickCount()*portTICK_PERIOD_MS;
    s_map_action.run=s_map_run;
    s_map_action.direction=(distance_mm>0.0f && direction<4U)?direction:4U;
    s_map_action.scope=scope;
    taskEXIT_CRITICAL();
#else
    (void)direction;(void)rpm;(void)distance_mm;(void)route_speed;
#endif
}
#if APP_CONTINUOUS_MAP_ENABLE
static ChassisOdometry s_map_odometry;
static AppMapPose s_map_pose,s_map_published __attribute__((section(".bss.map_pose"),zero_init));
static uint32_t s_map_start,s_map_begin,s_map_gap_run;
static uint32_t s_map_sign[4],s_map_raw[4],s_map_power_epoch,s_map_sample_begin;
static AppMapStationAnchor s_map_anchor;
static void AppOdometry_MapGap(uint32_t reason)
{
    s_map_pose.state=APP_MAP_GAP;
    s_map_odometry.status.valid=0U;
    s_map_odometry.status.continuity_lost=1U;
    s_map_odometry.status.reasons|=ODOM_GAP|reason;
}
#endif
uint32_t AppOdometry_RequestStationAnchor(uint32_t run,uint32_t start,uint32_t version,
 uint32_t power_epoch,uint32_t ticket,uint32_t cell)
{
 uint32_t sequence=0U;
#if APP_CONTINUOUS_MAP_ENABLE
 taskENTER_CRITICAL();
 if(run && run==s_map_run && start==s_map_start && version==VEHICLE_MAP_VERSION &&
    power_epoch && power_epoch==s_map_power_epoch && ticket && (cell==10U || cell==22U) &&
    s_map_gap_run!=run && s_map_published.run_id==run && s_map_published.start_id==start &&
    s_map_published.map_version==version && s_map_published.based &&
    s_map_published.state==APP_MAP_TRACKING && s_map_published.pose.valid &&
    !s_map_published.pose.local_origin && !s_map_published.pose.reasons &&
    !s_map_published.pose.continuity_lost &&
    s_map_anchor.state!=1U && s_map_anchor.state!=2U &&
    !(s_map_anchor.run_id==run && s_map_anchor.ticket>=ticket)) {
  sequence=s_map_anchor.sequence+1U;if(!sequence)sequence=1U;
  memset(&s_map_anchor,0,sizeof(s_map_anchor));s_map_anchor.sequence=sequence;
  s_map_anchor.ticket=ticket;s_map_anchor.run_id=run;s_map_anchor.start_id=start;
  s_map_anchor.map_version=version;s_map_anchor.power_epoch=power_epoch;s_map_anchor.cell=cell;
  s_map_anchor.request_ms=(uint32_t)xTaskGetTickCount()*portTICK_PERIOD_MS;s_map_anchor.state=1U;
 }
 taskEXIT_CRITICAL();
#else
 (void)run;(void)start;(void)version;(void)power_epoch;(void)ticket;(void)cell;
#endif
 return sequence;
}
void AppOdometry_ReadStationAnchor(AppMapStationAnchor *snapshot)
{
 if(!snapshot)return;
#if APP_CONTINUOUS_MAP_ENABLE
 taskENTER_CRITICAL();*snapshot=s_map_anchor;taskEXIT_CRITICAL();
#else
 memset(snapshot,0,sizeof(*snapshot));
#endif
}
void AppOdometry_RequestMapRun(uint32_t run_id,uint32_t start_id)
{
#if APP_CONTINUOUS_MAP_ENABLE
    taskENTER_CRITICAL();
    if(run_id && (!s_map_run || (int32_t)(run_id-s_map_run)>0)) {
        s_map_run=run_id;s_map_start=start_id;s_map_gap_run=0U;
        s_map_begin=(uint32_t)xTaskGetTickCount()*portTICK_PERIOD_MS;
    }
    taskEXIT_CRITICAL();
#else
    (void)run_id;(void)start_id;
#endif
}
/* Small atomic identity read avoids placing a full map snapshot on motion call stacks. */
uint32_t AppOdometry_MapTrackingRun(void)
{
    uint32_t run=0U;
#if APP_CONTINUOUS_MAP_ENABLE
    uint32_t i,now;
    taskENTER_CRITICAL();
    now=(uint32_t)xTaskGetTickCount()*portTICK_PERIOD_MS;
    if(s_map_run==s_map_published.run_id && s_map_published.state==APP_MAP_TRACKING &&
       s_map_published.pose.valid && s_map_gap_run!=s_map_run) {
        uint32_t stale=now-s_map_published.angle_ms>=ODOM_YAW_MAX_AGE_MS;
        for(i=0U;i<4U;i++) if(now-s_map_published.wheel_ms[i]>=ODOM_WHEEL_MAX_AGE_MS) stale=1U;
        if(stale) s_map_gap_run=s_map_run;else run=s_map_run;
    }
    taskEXIT_CRITICAL();
#endif
    return run;
}
void AppOdometry_ReadMap(AppMapPose *snapshot)
{
    uint32_t now=(uint32_t)xTaskGetTickCount()*portTICK_PERIOD_MS;
#if APP_CONTINUOUS_MAP_ENABLE
    uint32_t i;
    taskENTER_CRITICAL();
    now=(uint32_t)xTaskGetTickCount()*portTICK_PERIOD_MS;
    *snapshot=s_map_published;
    if(s_map_run!=snapshot->run_id) {
        memset(snapshot,0,sizeof(*snapshot));
        snapshot->run_id=s_map_run;snapshot->start_id=s_map_start;
        snapshot->begin_ms=s_map_begin;snapshot->pose.age_ms=0xFFFFFFFFU;
        snapshot->state=(s_map_start==1U || s_map_start==2U)?APP_MAP_WAIT_BASELINE:APP_MAP_NO_START;
        snapshot->pose.reasons=(snapshot->state==APP_MAP_NO_START)?APP_MAP_NO_START_REASON:ODOM_WAITING;
    }
    snapshot->map_version=VEHICLE_MAP_VERSION;
    if(snapshot->based) snapshot->pose.age_ms=now-snapshot->pose.sample_ms;
    if(snapshot->state==APP_MAP_TRACKING) {
        uint32_t stale=now-snapshot->angle_ms>=ODOM_YAW_MAX_AGE_MS;
        for(i=0U;i<4U;i++) if(now-snapshot->wheel_ms[i]>=ODOM_WHEEL_MAX_AGE_MS) stale=1U;
        if(stale || s_map_gap_run==snapshot->run_id) {
            s_map_gap_run=snapshot->run_id;
            snapshot->state=APP_MAP_GAP;snapshot->pose.valid=0U;
            snapshot->pose.continuity_lost=1U;snapshot->pose.reasons|=ODOM_GAP|ODOM_STALE;
        }
    }
    taskEXIT_CRITICAL();
#else
    memset(snapshot,0,sizeof(*snapshot));snapshot->pose.age_ms=0xFFFFFFFFU;
    snapshot->pose.reasons=ODOM_WAITING;
#endif
    snapshot->now_ms=now;
}
#if APP_CONTINUOUS_MAP_ENABLE
#if APP_MAP_NAV_RESIDUAL_ENABLE
static void AppOdometry_MapNavigationStep(void)
{
    static const float cross[4]={APP_FORWARD_NAV_RESIDUAL_COEFFICIENT,APP_LEFT_NAV_RESIDUAL_COEFFICIENT,
        APP_RIGHT_NAV_RESIDUAL_COEFFICIENT,APP_BACKWARD_NAV_RESIDUAL_COEFFICIENT};
    static const float main[4]={APP_FORWARD_MAIN_NAV_COEFFICIENT,APP_LEFT_MAIN_NAV_COEFFICIENT,
        APP_RIGHT_MAIN_NAV_COEFFICIENT,APP_BACKWARD_MAIN_NAV_COEFFICIENT};
    MapAction action;uint32_t busy,d;float f,l,q,cf,cl,angle;
    taskENTER_CRITICAL();action=s_map_action;busy=MotionGetState()==MOTION_BUSY;taskEXIT_CRITICAL();
    if(action.run!=s_map_pose.run_id || !action.epoch) return;
    if(action.epoch!=s_map_navigation.epoch) {
        s_map_navigation.epoch=action.epoch;s_map_navigation.based=0U;s_map_navigation.active=busy;
        s_map_navigation.history_x=s_map_pose.correction_x_mm;
        s_map_navigation.history_y=s_map_pose.correction_y_mm;
        s_map_pose.action_epoch=action.epoch;s_map_pose.calibration_direction=action.direction;
        s_map_pose.calibration_flags|=APP_MAP_CAL_BOUNDARY;
    }
    if(!busy) {s_map_navigation.active=0U;s_map_pose.calibration_flags|=APP_MAP_CAL_TAIL;}
    d=action.direction;
    if(d>=4U) {s_map_pose.calibration_flags|=APP_MAP_CAL_UNSUPPORTED;return;}
    if(!s_map_navigation.active || (int32_t)(s_map_odometry.status.sample_ms-action.begin)<0) return;
    if(!s_map_navigation.based) {
        s_map_navigation.x=s_map_odometry.status.x_mm;s_map_navigation.y=s_map_odometry.status.y_mm;
        angle=s_map_odometry.status.heading_deg*0.01745329251994329577f;
        s_map_navigation.cs=cosf(angle);s_map_navigation.sn=sinf(angle);s_map_navigation.based=1U;return;
    }
    f=s_map_navigation.cs*(s_map_odometry.status.x_mm-s_map_navigation.x)+
        s_map_navigation.sn*(s_map_odometry.status.y_mm-s_map_navigation.y);
    l=-s_map_navigation.sn*(s_map_odometry.status.x_mm-s_map_navigation.x)+
        s_map_navigation.cs*(s_map_odometry.status.y_mm-s_map_navigation.y);
    q=d==0U?f:(d==1U?l:(d==2U?-l:-f));if(q<0.0f) q=0.0f;
    cf=APP_LEFT_NAV_RESIDUAL_WEIGHT*q*((d==1U || d==2U)?cross[d]:(d==0U?main[d]:-main[d]));
    cl=APP_LEFT_NAV_RESIDUAL_WEIGHT*q*((d==0U || d==3U)?cross[d]:(d==1U?main[d]:-main[d]));
    s_map_pose.correction_x_mm=s_map_navigation.history_x+s_map_navigation.cs*cf-s_map_navigation.sn*cl;
    s_map_pose.correction_y_mm=s_map_navigation.history_y+s_map_navigation.sn*cf+s_map_navigation.cs*cl;
    if(q>0.0f) {s_map_pose.nav_applied=1U;s_map_pose.calibration_scope|=action.scope;}
}
#endif
static uint32_t AppOdometry_AnchorQuiet(const AppMapStationAnchor *a,
 const AppHeadingSource *source,uint32_t now,uint32_t after)
{
 ChassisZone2CanStatus can;uint32_t i;
 ChassisAuto_Zone2Read(&can);
 if(!source->healthy || App_HeadingFrameAge(source,now,1U)>=ODOM_YAW_MAX_AGE_MS ||
    App_HeadingFrameAge(source,now,2U)>=ODOM_YAW_MAX_AGE_MS || fabsf(source->rate_dps)>1.0f ||
    (int32_t)(source->angle_ms-after)<=0 || (int32_t)(source->rate_ms-after)<=0 ||
    can.run_id!=a->run_id || can.power_epoch!=a->power_epoch || !can.feedback_ready ||
    can.state!=CHASSIS_AUTO_IDLE || g_chassis_auto_active || g_wheel_test_active)return 0U;
 for(i=0U;i<4U;i++)if(can.wheel_decirpm[i] || (int32_t)(can.wheel_ms[i]-after)<=0 ||
    now-can.wheel_ms[i]>=ODOM_WHEEL_MAX_AGE_MS)return 0U;
 return 1U;
}
static void AppOdometry_MapAnchorStep(const AppHeadingSource *source,
 const PowerCommandStatus *power,uint32_t now)
{
 AppMapStationAnchor a;float x,y,heading;
 AppOdometry_ReadStationAnchor(&a);
 if(a.state!=1U && a.state!=2U)return;
 if(a.run_id!=s_map_run || a.start_id!=s_map_start || a.map_version!=VEHICLE_MAP_VERSION ||
    a.power_epoch!=s_map_power_epoch || !power->active || power->enable_count!=a.power_epoch ||
    s_map_gap_run==a.run_id || s_map_pose.state==APP_MAP_GAP ||
    g_safety_fault || g_rtos_fault || g_can_ctrl_fault_latched ||
    now-a.request_ms>=2500U) {
  taskENTER_CRITICAL();s_map_anchor.state=4U;taskEXIT_CRITICAL();return;
 }
 if(a.state!=1U || !AppOdometry_AnchorQuiet(&a,source,now,a.request_ms))return;
 if(s_map_pose.state!=APP_MAP_TRACKING || !s_map_odometry.status.valid ||
    s_map_odometry.status.continuity_lost || s_map_odometry.status.reasons)return;
 x=a.cell==10U?FIELD_COARSE_X_MM+50.0f:FIELD_STORAGE_X_MM;
 y=a.cell==10U?FIELD_COARSE_Y_MM:FIELD_STORAGE_Y_MM;
 heading=remainderf(source->yaw_deg+s_map_odometry.heading_offset,360.0f);
 if(!ChassisOdometry_SetPose(&s_map_odometry,x,y,heading,source->yaw_deg)) {
  taskENTER_CRITICAL();s_map_anchor.state=4U;taskEXIT_CRITICAL();return;
 }
 /* Never splice a pre-cutover wheel/yaw into the fresh native baseline. */
 s_map_sample_begin=now+1U;
 memset(s_map_sign,0,sizeof(s_map_sign));memset(s_map_raw,0,sizeof(s_map_raw));
 memset(s_map_pose.wheel_ms,0,sizeof(s_map_pose.wheel_ms));s_map_pose.angle_ms=0U;
 s_map_pose.correction_x_mm=s_map_pose.correction_y_mm=0.0f;
 s_map_pose.nav_applied=s_map_pose.calibration_scope=s_map_pose.calibration_flags=s_map_pose.action_epoch=0U;
 s_map_pose.state=APP_MAP_WAIT_BASELINE;s_map_pose.based=0U;
#if APP_MAP_NAV_RESIDUAL_ENABLE
 taskENTER_CRITICAL();memset(&s_map_action,0,sizeof(s_map_action));taskEXIT_CRITICAL();
 memset(&s_map_navigation,0,sizeof(s_map_navigation));
#endif
 taskENTER_CRITICAL();s_map_anchor.applied_ms=now;s_map_anchor.state=2U;taskEXIT_CRITICAL();
}
static void AppOdometry_MapAnchorAck(const AppHeadingSource *source,uint32_t now)
{
 AppMapStationAnchor a;uint32_t i;
 AppOdometry_ReadStationAnchor(&a);
 if(a.state!=2U || s_map_pose.state!=APP_MAP_TRACKING || !s_map_odometry.based ||
    !s_map_odometry.status.valid || s_map_odometry.status.continuity_lost ||
    s_map_odometry.status.reasons || (int32_t)(s_map_odometry.status.sample_ms-a.applied_ms)<=0 ||
    !AppOdometry_AnchorQuiet(&a,source,now,a.applied_ms))return;
 for(i=0U;i<4U;i++)if((int32_t)(s_map_pose.wheel_ms[i]-a.applied_ms)<=0)return;
 taskENTER_CRITICAL();s_map_anchor.ack_ms=now;s_map_anchor.state=3U;taskEXIT_CRITICAL();
}
static void AppOdometry_MapStep(const AppHeadingSource *source,
    const ChassisOdometrySamples *backend,uint32_t now)
{
    ChassisOdometrySamples diagnostic;
    PowerCommandStatus power;
    uint32_t i,run,start,begin,gap;
    taskENTER_CRITICAL();run=s_map_run;start=s_map_start;begin=s_map_begin;gap=s_map_gap_run;taskEXIT_CRITICAL();
    if(run!=s_map_pose.run_id) {
        ChassisOdometry_Init(&s_map_odometry);
        memset(&s_map_pose,0,sizeof(s_map_pose));
        memset(s_map_sign,0,sizeof(s_map_sign));memset(s_map_raw,0,sizeof(s_map_raw));
        s_map_power_epoch=0U;s_map_sample_begin=begin;
#if APP_MAP_NAV_RESIDUAL_ENABLE
        memset(&s_map_navigation,0,sizeof(s_map_navigation));
#endif
        s_map_pose.run_id=run;s_map_pose.start_id=start;s_map_pose.begin_ms=begin;
        s_map_pose.state=(start==1U || start==2U)?APP_MAP_WAIT_BASELINE:APP_MAP_NO_START;
        if(s_map_pose.state==APP_MAP_NO_START) s_map_odometry.status.reasons=APP_MAP_NO_START_REASON;
    }
    if(run && gap==run) AppOdometry_MapGap(ODOM_STALE);
    if(!run || s_map_pose.state==APP_MAP_NO_START || s_map_pose.state==APP_MAP_GAP) goto publish;
    Power_ReadCommandStatus(&power);
    if(g_safety_fault || g_rtos_fault || g_can_ctrl_fault_latched ||
       (s_map_odometry.based && (!power.active || power.enable_count!=s_map_power_epoch))) {
        AppOdometry_MapGap(APP_MAP_FAULT);goto publish;
    }
    AppOdometry_MapAnchorStep(source,&power,now);
    begin=s_map_sample_begin;
    if(!s_map_odometry.based && (g_chassis_auto_active || g_wheel_test_active)) {
        AppOdometry_MapGap(APP_MAP_NO_BASELINE);goto publish;
    }
    if(!s_map_odometry.configured) {
        if(!power.active || !source->healthy || (int32_t)(source->angle_ms-begin)<0 ||
           App_HeadingFrameAge(source,now,1U)>=ODOM_YAW_MAX_AGE_MS) goto publish;
        if(!ChassisOdometry_SetPose(&s_map_odometry,
            start==1U?FIELD_START1_X_MM:FIELD_START2_X_MM,
            start==1U?FIELD_START1_Y_MM:FIELD_START2_Y_MM,90.0f,source->yaw_deg)) goto publish;
        s_map_power_epoch=power.enable_count;
    }
    AppCanDiagnostics_ReadPositionSamples(&diagnostic);
    for(i=0U;i<4U;i++) {
        const ChassisOdometrySamples *chosen=0;
        uint32_t j;
        for(j=0U;j<2U;j++) {
            const ChassisOdometrySamples *candidate=j?&diagnostic:backend;
            uint32_t ms=candidate->ms[i];
            if(!(candidate->mask&(1UL<<i)) || (int32_t)(ms-begin)<0) continue;
            if((s_map_odometry.raw_mask&(1UL<<i)) && (int32_t)(ms-s_map_pose.wheel_ms[i])<0) continue;
            if(candidate->sign[i]>1U || (int32_t)(ms-now)>0) {
                AppOdometry_MapGap(APP_MAP_BAD_SOURCE);goto publish;
            }
            if(!chosen || (int32_t)(ms-chosen->ms[i])>0) chosen=candidate;
            else if(ms==chosen->ms[i] && (candidate->sign[i]!=chosen->sign[i] || candidate->raw[i]!=chosen->raw[i])) {
                AppOdometry_MapGap(APP_MAP_BAD_SOURCE);goto publish;
            }
        }
        if(chosen) {
            uint32_t ms=chosen->ms[i];
            if(s_map_odometry.raw_mask&(1UL<<i)) {
                if((int32_t)(ms-s_map_pose.wheel_ms[i])<0) continue;
                if(ms==s_map_pose.wheel_ms[i] && (chosen->sign[i]!=s_map_sign[i] || chosen->raw[i]!=s_map_raw[i])) {
                    AppOdometry_MapGap(APP_MAP_BAD_SOURCE);goto publish;
                }
            }
            ChassisOdometry_Wheel(&s_map_odometry,i,chosen->sign[i],chosen->raw[i],ms);
            s_map_pose.wheel_ms[i]=ms;s_map_sign[i]=chosen->sign[i];s_map_raw[i]=chosen->raw[i];
        }
    }
    if(source->healthy && (int32_t)(source->angle_ms-begin)>=0 &&
       App_HeadingFrameAge(source,now,1U)<ODOM_YAW_MAX_AGE_MS) {
        if(!s_map_odometry.yaw.count || (int32_t)(source->angle_ms-s_map_pose.angle_ms)>=0) {
            ChassisOdometry_Yaw(&s_map_odometry,source->yaw_deg,source->angle_ms);
            s_map_pose.angle_ms=source->angle_ms;
        }
    } else if(s_map_odometry.based) AppOdometry_MapGap(ODOM_STALE);
    if(s_map_pose.state!=APP_MAP_GAP) {
        ChassisOdometry_Update(&s_map_odometry,now);
        if(s_map_odometry.status.continuity_lost || s_map_odometry.rejected) AppOdometry_MapGap(0U);
        else if(s_map_odometry.based && s_map_odometry.status.valid) s_map_pose.state=APP_MAP_TRACKING;
    }
publish:
    taskENTER_CRITICAL();
    if(run && s_map_gap_run==run) AppOdometry_MapGap(ODOM_STALE);
    if(s_map_pose.state==APP_MAP_GAP && s_map_published.run_id==run) {
        uint32_t reasons=s_map_odometry.status.reasons;
        s_map_pose=s_map_published;s_map_odometry.status=s_map_published.pose;
        s_map_odometry.status.x_mm=s_map_published.raw_x_mm;
        s_map_odometry.status.y_mm=s_map_published.raw_y_mm;
        AppOdometry_MapGap(reasons);
    }
#if APP_MAP_NAV_RESIDUAL_ENABLE
    if(s_map_pose.state==APP_MAP_TRACKING && s_map_odometry.status.valid) AppOdometry_MapNavigationStep();
#endif
    s_map_pose.pose=s_map_odometry.status;s_map_pose.based=s_map_odometry.based;
    s_map_pose.raw_x_mm=s_map_odometry.status.x_mm;s_map_pose.raw_y_mm=s_map_odometry.status.y_mm;
    s_map_pose.pose.x_mm+=s_map_pose.correction_x_mm;s_map_pose.pose.y_mm+=s_map_pose.correction_y_mm;
    s_map_pose.pose.age_ms=s_map_pose.based?now-s_map_pose.pose.sample_ms:0xFFFFFFFFU;
    s_map_pose.source=s_map_pose.nav_applied?APP_MAP_CALIBRATED_WHEEL_HWT:APP_MAP_RAW_WHEEL_HWT;
    s_map_pose.map_version=VEHICLE_MAP_VERSION;
    s_map_pose.now_ms=now;AppOdometry_MapAnchorAck(source,now);s_map_published=s_map_pose;
    taskEXIT_CRITICAL();
}
#endif
#if APP_BASELINE40_ACTIVE
static AppOdometryBaseline40 s_baseline40;
#if APP_NAV_RESIDUAL_ACTIVE
static AppOdometryBaseline40 s_navigation40;
static uint32_t s_navigation_left_scope;
static uint32_t AppOdometry_LeftNavigationScope(const AppOdometryInputs *inputs)
{
    const AppMotionObservation *motion=AppMotion_Live();
    const MotionAction *action=motion->actions;
    uint32_t id=g_debug_plan_id;
    if ((*inputs->runtime_route_id) || (*motion->action_count)!=1U ||
        action->run_ms || action->reference_mm_s!=0.0f || action->relative_deg!=0.0f) return 0U;
#if APP_LEFT_NAV_RESIDUAL_ENABLE
    if ((id==DEBUG_PLAN_LEFT210_40 || id==DEBUG_PLAN_LEFT150_40) &&
        action->kind==MOTION_LEFT_DISTANCE && action->base_rpm==APP_BASELINE40_LEFT_RPM &&
        AppDebugPlan40Allowed(id,1U,action->base_rpm,action->distance_mm)) return 1U;
#endif
#if APP_DIRECTION_NAV_RESIDUAL_ENABLE
    if (action->distance_mm!=2100.0f) return 0U;
    if (APP_RIGHT40_FORWARD_TRIM_ENABLE && id==DEBUG_PLAN_RIGHT210_40 &&
        action->kind==MOTION_RIGHT_DISTANCE && action->base_rpm==APP_BASELINE40_LEFT_RPM &&
        AppDebugPlan40Allowed(id,2U,action->base_rpm,action->distance_mm)) return 2U;
    if (action->base_rpm==APP_BASELINE40_RPM) {
        if (id==DEBUG_PLAN_FORWARD210_40 && action->kind==MOTION_FORWARD_DISTANCE &&
            AppDebugPlan40Allowed(id,0U,action->base_rpm,action->distance_mm)) return 3U;
        if (id==DEBUG_PLAN_BACKWARD210_40 && action->kind==MOTION_BACKWARD_DISTANCE &&
            AppDebugPlan40Allowed(id,3U,action->base_rpm,action->distance_mm)) return 4U;
    }
#endif
    return 0U;
}
#endif
void AppOdometry_ReadBaseline40(AppOdometryBaseline40 *snapshot)
{
    taskENTER_CRITICAL();
    *snapshot=s_baseline40;
    taskEXIT_CRITICAL();
}
void AppOdometry_ReadNavigation40(AppOdometryBaseline40 *snapshot)
{
#if APP_NAV_RESIDUAL_ACTIVE
    taskENTER_CRITICAL();
    *snapshot=s_navigation40;
    taskEXIT_CRITICAL();
#else
    AppOdometry_ReadBaseline40(snapshot);
#endif
}
#endif

#if APP_TEST_MODE == 3U
volatile PerimeterRecord g_perimeter_records[9];
static uint32_t s_perimeter_odom_started,s_perimeter_completed_seen;
void AppOdometry_ResetRecords(void)
{
#if APP_BASELINE40_ACTIVE
    s_baseline40.anchor_valid=0U;
#if APP_NAV_RESIDUAL_ACTIVE
    s_navigation40.anchor_valid=0U;
#endif
#endif
    s_perimeter_odom_started=s_perimeter_completed_seen=0U;
    memset((void*)g_perimeter_records,0,sizeof(g_perimeter_records));
}
#endif

#if APP_POSITION_DIAGNOSTIC && APP_HEADING_SESSION_TEST
/* CAN任务独占计算状态；TaskA只复制小型只读结果。 */
static ChassisOdometry s_odometry;
volatile ChassisOdometryStatus g_chassis_odometry;
/* 调试Watch：先填三个目标，最后写request=1；仅VM关闭且无运动时受理。
 * result:1接受，2条件不足；CAN任务消费一次，不等待条件后自动执行。 */
#if APP_IS_DEBUG
volatile uint32_t g_odometry_pose_request=0U, g_odometry_pose_result=0U;
volatile float g_odometry_pose_x_mm=0.0f, g_odometry_pose_y_mm=0.0f;
volatile float g_odometry_pose_heading_deg=0.0f;
#endif
void AppOdometry_Init(void)
{
#if APP_BASELINE40_ACTIVE
    memset(&s_baseline40,0,sizeof(s_baseline40));
#if APP_NAV_RESIDUAL_ACTIVE
    memset(&s_navigation40,0,sizeof(s_navigation40));
    s_navigation_left_scope=0U;
#endif
#endif
    ChassisOdometry_Init(&s_odometry);
#if APP_CONTINUOUS_MAP_ENABLE
    ChassisOdometry_Init(&s_map_odometry);memset(&s_map_pose,0,sizeof(s_map_pose));
    s_map_pose.pose=s_map_odometry.status;s_map_published=s_map_pose;
    s_map_run=s_map_start=s_map_begin=s_map_gap_run=s_map_sample_begin=0U;
    memset(&s_map_anchor,0,sizeof(s_map_anchor));
#if APP_MAP_NAV_RESIDUAL_ENABLE
    memset(&s_map_action,0,sizeof(s_map_action));memset(&s_map_navigation,0,sizeof(s_map_navigation));
#endif
#endif
}

void AppOdometry_Step(const AppOdometryInputs *inputs)
{
    AppHeadingSource source;
    ChassisOdometrySamples samples;
#if APP_NAV_RESIDUAL_ACTIVE
    float navigation_x,navigation_y;
#endif
    uint32_t i,now=(uint32_t)xTaskGetTickCount()*portTICK_PERIOD_MS;
    (void)App_HeadingSourceRead(&source);
#if APP_IS_DEBUG
    if(g_odometry_pose_request==1U) {
        PowerCommandStatus power;
        float x,y,heading;
        taskENTER_CRITICAL();
        x=g_odometry_pose_x_mm; y=g_odometry_pose_y_mm;
        heading=g_odometry_pose_heading_deg; g_odometry_pose_request=0U;
        taskEXIT_CRITICAL();
        Power_ReadCommandStatus(&power);
        g_odometry_pose_result=2U;
#if APP_BASELINE40_ACTIVE
        s_baseline40.anchor_valid=0U;
#if APP_NAV_RESIDUAL_ACTIVE
        s_navigation40.anchor_valid=0U;
#endif
#endif
        if(!(*inputs->vm_request) && !power.active && !g_chassis_auto_active &&
           !g_wheel_test_active && !(*inputs->jog_preparing) && source.healthy &&
           App_HeadingFrameAge(&source,now,1U)<ODOM_YAW_MAX_AGE_MS &&
           !g_safety_fault && !g_rtos_fault && !g_can_ctrl_fault_latched)
            if(ChassisOdometry_SetPose(&s_odometry,x,y,heading,source.yaw_deg))
            {
                g_odometry_pose_result=1U;
#if APP_NAV_RESIDUAL_ACTIVE
                s_navigation_left_scope=0U;
#endif
            }
    }
#endif
    if(!s_perimeter_odom_started && (*inputs->combined_state)==1U && source.healthy) {
        if(inputs->route_heading_reference->valid && ChassisOdometry_SetPose(&s_odometry,
            (VEHICLE_STAGE1_ENABLE || (*inputs->runtime_route_id))?0.0f:2100.0f,(VEHICLE_STAGE1_ENABLE || (*inputs->runtime_route_id))?0.0f:2100.0f,
            (VEHICLE_STAGE1_ENABLE || (*inputs->runtime_route_id))?0.0f:-90.0f,inputs->route_heading_reference->origin_deg))
        {
            s_perimeter_odom_started=1U;
#if APP_BASELINE40_ACTIVE
            s_baseline40.origin_x_mm=s_odometry.status.x_mm;
            s_baseline40.origin_y_mm=s_odometry.status.y_mm;
            s_baseline40.path_heading_deg=s_odometry.status.heading_deg;
            s_baseline40.heading_offset_deg=s_odometry.heading_offset;
            s_baseline40.reference_set_ms=now;
            s_baseline40.anchor_valid=1U;
#if APP_NAV_RESIDUAL_ACTIVE
            s_navigation_left_scope=AppOdometry_LeftNavigationScope(inputs);
#endif
#endif
        }
    }
    ChassisAuto_ReadOdometrySamples(&samples);
#if APP_CONTINUOUS_MAP_ENABLE
    AppOdometry_MapStep(&source,&samples,now);
#endif
    if(source.healthy && App_HeadingFrameAge(&source,now,1U)<ODOM_YAW_MAX_AGE_MS)
        ChassisOdometry_Yaw(&s_odometry,source.yaw_deg,source.angle_ms);
    else ChassisOdometry_Invalidate(&s_odometry);
    for(i=0U;i<4U;i++)
        if(samples.mask & (1UL<<i))
            ChassisOdometry_Wheel(&s_odometry,i,samples.sign[i],samples.raw[i],samples.ms[i]);
    if(samples.mask!=15U || g_safety_fault || g_rtos_fault || g_can_ctrl_fault_latched)
        ChassisOdometry_Invalidate(&s_odometry);
    ChassisOdometry_Update(&s_odometry,now);
#if APP_NAV_RESIDUAL_ACTIVE
    navigation_x=s_odometry.status.x_mm;navigation_y=s_odometry.status.y_mm;
    if(s_navigation_left_scope)
    {
        float angle=s_baseline40.path_heading_deg*0.01745329251994329577f;
        float cs=cosf(angle),sn=sinf(angle);
        float left=-sn*(navigation_x-s_baseline40.origin_x_mm)+
            cs*(navigation_y-s_baseline40.origin_y_mm);
        float forward=cs*(navigation_x-s_baseline40.origin_x_mm)+
            sn*(navigation_y-s_baseline40.origin_y_mm);
        float distance=left,coefficient=APP_LEFT_NAV_RESIDUAL_COEFFICIENT;
        float correction;
        if(s_navigation_left_scope==2U) {
            distance=-left;coefficient=APP_RIGHT_NAV_RESIDUAL_COEFFICIENT;
        } else if(s_navigation_left_scope==3U) {
            distance=forward;coefficient=APP_FORWARD_NAV_RESIDUAL_COEFFICIENT;
        } else if(s_navigation_left_scope==4U) {
            distance=-forward;coefficient=APP_BACKWARD_NAV_RESIDUAL_COEFFICIENT;
        }
        correction=APP_LEFT_NAV_RESIDUAL_WEIGHT*coefficient*((distance>0.0f)?distance:0.0f);
        if(s_navigation_left_scope<=2U) {
            navigation_x+=cs*correction;navigation_y+=sn*correction;
        } else {
            navigation_x-=sn*correction;navigation_y+=cs*correction;
        }
#if APP_MAIN_NAV_RESIDUAL_ENABLE
        {
            float main_coefficient=APP_LEFT_MAIN_NAV_COEFFICIENT;
            if(s_navigation_left_scope==2U) main_coefficient=APP_RIGHT_MAIN_NAV_COEFFICIENT;
            else if(s_navigation_left_scope==3U) main_coefficient=APP_FORWARD_MAIN_NAV_COEFFICIENT;
            else if(s_navigation_left_scope==4U) main_coefficient=APP_BACKWARD_MAIN_NAV_COEFFICIENT;
            correction=APP_LEFT_NAV_RESIDUAL_WEIGHT*main_coefficient*((distance>0.0f)?distance:0.0f);
            if(s_navigation_left_scope==1U) {
                navigation_x-=sn*correction;navigation_y+=cs*correction;
            } else if(s_navigation_left_scope==2U) {
                navigation_x+=sn*correction;navigation_y-=cs*correction;
            } else if(s_navigation_left_scope==3U) {
                navigation_x+=cs*correction;navigation_y+=sn*correction;
            } else {
                navigation_x-=cs*correction;navigation_y-=sn*correction;
            }
        }
#endif
    }
#endif
    taskENTER_CRITICAL();
#if APP_BASELINE40_ACTIVE
    s_baseline40.pose=s_odometry.status;
#endif
#if APP_NAV_RESIDUAL_ACTIVE
    s_navigation40=s_baseline40;
    s_navigation40.pose.x_mm=navigation_x;s_navigation40.pose.y_mm=navigation_y;
    g_chassis_odometry=s_navigation40.pose;
#else
    g_chassis_odometry=s_odometry.status;
#endif
    if(s_perimeter_odom_started && !g_perimeter_records[0].captured && s_odometry.status.valid &&
       (*inputs->combined_completed)==0U) {
        g_perimeter_records[0].captured=1U;g_perimeter_records[0].tick_ms=now;
        g_perimeter_records[0].pose=s_odometry.status;
    }
    if((*inputs->combined_completed)>s_perimeter_completed_seen && (*inputs->combined_completed)<=8U) {
        uint32_t slot=(*inputs->combined_completed);
        g_perimeter_records[slot].captured=1U;g_perimeter_records[slot].tick_ms=now;
        g_perimeter_records[slot].pose=s_odometry.status;
        s_perimeter_completed_seen=slot;
    }
    taskEXIT_CRITICAL();
}
#endif
