#include "app_config.h"

#if APP_ZONE2_NAV_ENABLE
#include "app_zone2.h"
#include "app_can_diagnostics.h"
#include "app_odometry.h"
#include "boot_ready_link.h"
#include "buzzer.h"
#include "can.h"
#include "chassis_auto_test.h"
#include "freertos_hooks.h"
#include "heading_source.h"
#include "radar_lift.h"
#include "radar_servo.h"
#include "radar_servo_config.h"
#include "radar_zone_switch.h"
#include "safety.h"
#include "startup_axes.h"
#include "usart.h"
#include "vehicle_tuning.h"
#include "zone2_nav_schedule.h"
#if APP_ZONE2_NAV_MOTION_ENABLE
#include "zone2_motion_reference.h"
#endif
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <float.h>
#include <stdio.h>
#include <string.h>
#if APP_ZONE2_MISSION_ENABLE
#include "field_map.h"
#include "screen.h"
#include "source_body_step.h"
#include <string.h>
#endif
#if APP_ZONE2_POSE_TRACE_ENABLE
#include <stddef.h>
#endif

#define ZONE2_RUN_ID 1U
#define ZONE2_START_ID 2U
#define ZONE2_WAIT_READY_MS 15000U
#define ZONE2_POSE_MS 160U
#define ZONE2_DEG_TO_RAD 0.01745329251994329577f
#define ZONE2_SECTION_PROGRESS_MS 2000U
#define ZONE2_SECTION_PROGRESS_STEP 0.2f
#define ZONE2_SECTION_POSITION_MM 2.5f
#define ZONE2_SECTION_HEADING_DEG (FUSION_MOTION_HEADING_TOLERANCE_DEG+FUSION_MOTION_HEADING_ROUNDOFF_DEG)
#if APP_ZONE2_MISSION_ENABLE
#define ZONE2_QR_DWELL_MS 1000U
#define ZONE2_QR_SHIFT_MM 50.0f
#define ZONE2_QR_SPEED_MM_S 30.0f
#define ZONE2_QR_PATH_ID 9U
#define STATION_STEP_MM 30.0f
#define STATION_STEP_DEG 10.0f
#define STATION_STOP_MM 0.20f
#define STATION_STOP_DEG 0.15f
#define STATION_SWEEP_RESERVE_MM 10.0f
#endif

typedef enum {
    ZONE2_IDLE, ZONE2_WAIT_READY, ZONE2_SCAN, ZONE2_PREPARE,
    ZONE2_ZERO, ZONE2_STOP, ZONE2_SHADOW_DONE, ZONE2_FAILED,
    ZONE2_TRACK, ZONE2_SECTION_ZERO, ZONE2_DONE
#if APP_ZONE2_MISSION_ENABLE
    , ZONE2_QR_WAIT, ZONE2_STATION_WAIT, ZONE2_MISSION_DONE
#endif
} Zone2Phase;

typedef enum {
    ZONE2_FAULT_NONE, ZONE2_FAULT_HEALTH, ZONE2_FAULT_DEADLINE,
    ZONE2_FAULT_PATH, ZONE2_FAULT_BACKEND, ZONE2_FAULT_SOURCE, ZONE2_FAULT_REFERENCE
#if APP_ZONE2_MISSION_ENABLE
    , ZONE2_FAULT_QR, ZONE2_FAULT_STATION
#endif
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
static uint32_t s_section, s_section_ms, s_generation, s_station_stop;
#if !APP_ZONE2_MISSION_ENABLE
static uint32_t s_prepare_ms;
static uint32_t s_section_progress_started, s_section_progress_ms, s_section_sample_ms;
static float s_section_checkpoint;
#endif
volatile uint32_t g_zone2_completed_legs;
#else
static FusionState s_reference;
#endif
static uint32_t s_begin_ms, s_session, s_power_epoch, s_primed, s_zone_locked;
static uint32_t s_stop_ms;
static uint32_t s_startup_axes_started,s_scan_beep_started;
volatile uint32_t g_boot_main_init_done=0U,g_boot_app_init_done=0U,g_boot_app_selftests_skipped=0U;
static uint32_t s_boot_last_tx,s_boot_sequence,s_boot_last_sequence,s_boot_beeped;
static uint32_t s_boot_ready_zone,s_boot_zone_epoch;
static char s_boot_rx[48];
static uint32_t s_boot_rx_length;

static uint16_t boot_crc16(const uint8_t *data,uint32_t length)
{
    uint16_t crc=0xffffU;uint32_t i;uint8_t bit;
    for(i=0U;i<length;i++) {
        crc^=(uint16_t)data[i]<<8;
        for(bit=0U;bit<8U;bit++)crc=(uint16_t)((crc&0x8000U)?((crc<<1)^0x1021U):(crc<<1));
    }
    return crc;
}

static void boot_parse_line(void)
{
    char *comma=strrchr(s_boot_rx,',');unsigned long zone,epoch,sequence;unsigned int received_crc;
    uint32_t prefix_length;
    if(!comma)return;
    prefix_length=(uint32_t)(comma-s_boot_rx)+1U;
    if(sscanf(s_boot_rx,"BA1,%lu,%lu,%lu,%4x",&zone,&epoch,&sequence,&received_crc)!=4)return;
    if(zone!=s_boot_ready_zone || epoch!=s_boot_zone_epoch || sequence!=s_boot_last_sequence ||
       received_crc!=boot_crc16((const uint8_t*)s_boot_rx,prefix_length))return;
    g_boot_app_init_done=1U;
}

uint32_t BootReady_IsReady(void) { return g_boot_app_init_done==1U; }

void BootReady_LinkPoll(uint32_t now)
{
    uint8_t byte;char frame[48];int n;uint16_t crc;uint32_t zone=g_radar_zone_selected;
    if(!g_boot_main_init_done || g_boot_app_init_done || !g_safety_ready || g_can_test_result!=1U ||
       !g_radar_zone_ready || zone<1U || zone>2U || !RadarObserver_IsReadyForZone(zone) ||
       g_radar_servo_state!=1U)return;
    if((uint32_t)(now-s_boot_last_tx)>=250U) {
        s_boot_last_tx=now;s_boot_last_sequence=++s_boot_sequence;
        n=snprintf(frame,sizeof(frame),"BR1,%lu,%lu,%lu,",(unsigned long)zone,
            (unsigned long)s_boot_zone_epoch,(unsigned long)s_boot_last_sequence);
        if(n>0 && (uint32_t)n<sizeof(frame)-7U) {
            crc=boot_crc16((const uint8_t*)frame,(uint32_t)n);
            n+=snprintf(frame+n,sizeof(frame)-(uint32_t)n,"%04X\r\n",crc);
            if(n>0)(void)USART6_VisionSend((const uint8_t*)frame,(uint16_t)n);
        }
    }
    while(USART6_VisionReadByte(&byte)) {
        if(byte=='\r')continue;
        if(byte=='\n') {
            s_boot_rx[s_boot_rx_length]='\0';boot_parse_line();s_boot_rx_length=0U;
            if(g_boot_app_init_done)break;
        } else if(s_boot_rx_length<sizeof(s_boot_rx)-1U)s_boot_rx[s_boot_rx_length++]=(char)byte;
        else s_boot_rx_length=0U;
    }
}

uint32_t BootReady_SharedByte(uint8_t byte,uint32_t now,uint32_t binary_active)
{
    (void)now;
    if(binary_active || g_boot_app_init_done)return 0U;
    if(byte=='\r')return 1U;
    if(byte=='\n') {s_boot_rx[s_boot_rx_length]='\0';boot_parse_line();s_boot_rx_length=0U;return 1U;}
    if(s_boot_rx_length<sizeof(s_boot_rx)-1U)s_boot_rx[s_boot_rx_length++]=(char)byte;
    else s_boot_rx_length=0U;
    return 1U;
}

void BootReady_BeepStep(void)
{
    uint32_t selected=g_radar_zone_ready?g_radar_zone_selected:0U;
    g_boot_main_init_done=1U;
    if(selected!=s_boot_ready_zone) {
        s_boot_ready_zone=selected;s_boot_zone_epoch++;
        if(!s_boot_zone_epoch)s_boot_zone_epoch=1U;
        g_boot_app_init_done=0U;s_boot_beeped=0U;
        s_boot_last_sequence=0U;s_boot_rx_length=0U;
    }
    BootReady_LinkPoll((uint32_t)xTaskGetTickCount());
    if(g_boot_app_init_done && !s_boot_beeped){Buzzer_Start(1U);s_boot_beeped=1U;}
}
volatile Zone2Phase g_zone2_phase;
volatile Zone2Fault g_zone2_fault;
volatile Zone2NavPathResult g_zone2_path_error;
volatile uint32_t g_zone2_reference_status, g_zone2_stop_pose_ms;
volatile uint32_t g_zone2_scan_obstacle_count;

#if APP_ZONE2_MISSION_ENABLE
typedef struct {
    uint32_t start_ms,end_ms;
} MissionInterval;

volatile struct {
    uint32_t key_ms,scan_start_ms,scan_end_ms,return_ms,page1_ms,failure_ms;
    MissionInterval navigation[ZONE2_NAV_LEGS],station[ZONE2_NAV_LEGS-2U];
    uint32_t receipt_ms[ZONE2_NAV_LEGS-2U][12];
    uint32_t picks,places,unverified_storage_actions;
    uint8_t storage_action_state[ZONE2_NAV_LEGS-2U][3U];
} g_zone2_mission_trace;

static struct {
    char task_code[16];
    uint32_t scan_open, scan_after_ms, display_pending;
    uint32_t scan_wait_ms, search_count, search_active, search_interrupted;
    uint32_t station_result, station_result_ms, page1_pending;
    AppZone2Station station;
    uint8_t receipt_count, slots[3], base_color[4], planned_base_color[4];
    float scan_y_mm;
} s_mission;
volatile uint32_t g_zone2_task_display_result;
volatile uint32_t g_source_body_step_request,g_source_body_step_state;
volatile uint32_t g_source_body_step_reason,g_source_body_step_session,g_source_align_rejected;
volatile uint32_t g_source_body_step_elapsed_ms,g_source_body_step_submitted;
volatile uint32_t g_source_align_direction,g_source_align_command_seq,g_source_align_chain_active;
volatile float g_source_align_distance_mm,g_source_align_turn_deg;

typedef enum { STATION_BODY_IDLE,STATION_BODY_ZERO,STATION_BODY_TRACK,STATION_BODY_STOP } StationBodyPhase;
static struct {
    StationBodyPhase phase;
    FusionPoint anchor,target;
    float nominal_yaw,target_yaw;
    uint32_t restoring,started_ms,stop_ms,ready_ms;
} s_station_body;

uint32_t SourceBodyStep_KeepsPower(void)
{
    return s_station_body.phase!=STATION_BODY_IDLE;
}

uint32_t SourceBodyStep_WheelWindow(void)
{
    return s_station_body.phase!=STATION_BODY_IDLE;
}

uint32_t SourceBodyStep_Busy(void)
{
    return SourceBodyStep_WheelWindow();
}

uint32_t SourceBodyStep_SmallDistanceWindow(void)
{
    return SourceBodyStep_WheelWindow();
}

uint32_t SourceBodyStep_FastAlignmentWindow(void)
{
    return SourceBodyStep_WheelWindow();
}
#endif

#if APP_ZONE2_POSE_TRACE_ENABLE
#define ZONE2_TRACE_CAPACITY 2048U
#define ZONE2_TRACE_PERIOD_MS 100U
#define ZONE2_TRACE_MAGIC 0x3154325AU
#define ZONE2_TRACE_CAPTURE 1U
#define ZONE2_TRACE_FROZEN 2U
#define ZONE2_TRACE_DROPPED 1U
#define ZONE2_TRACE_VALID 0x10U
#define ZONE2_TRACE_TERMINAL 0x20U

typedef struct {
    uint32_t elapsed_ms, source_ms;
    int16_t x_decimm, y_decimm, yaw_centideg;
    uint8_t phase, meta;
} Zone2PoseTraceRecord;

typedef struct {
    uint32_t magic, count, state, fault, flags;
    Zone2PoseTraceRecord records[ZONE2_TRACE_CAPACITY];
} Zone2PoseTrace;

typedef char Zone2PoseTraceLayout[
    sizeof(Zone2PoseTraceRecord)==16U && offsetof(Zone2PoseTrace,records)==20U &&
    sizeof(Zone2PoseTrace)==32788U ? 1 : -1];

volatile Zone2PoseTrace g_zone2_pose_trace __attribute__((section(".bss.zone2_pose_trace"),zero_init));

static struct {
    uint32_t source_ms, valid;
    float x_mm, y_mm, heading_deg;
} s_trace_pose;
static uint32_t s_trace_last_ms;

static void trace_capture(uint32_t now,uint32_t terminal)
{
    Zone2PoseTraceRecord record={0};
    uint32_t count=g_zone2_pose_trace.count;
    float x, y, yaw;

    if (!terminal) {
        if (count && now-s_trace_last_ms<ZONE2_TRACE_PERIOD_MS) return;
        /* Reserve the final record for DONE or FAILED. */
        if (count>=ZONE2_TRACE_CAPACITY-1U) {
            g_zone2_pose_trace.flags|=ZONE2_TRACE_DROPPED;
            return;
        }
        s_trace_last_ms=now;
    }
    record.elapsed_ms=now-s_begin_ms;
    record.source_ms=s_trace_pose.source_ms;
    record.phase=(uint8_t)g_zone2_phase;
    record.meta=(uint8_t)(g_zone2_completed_legs&0x0FU);
    if (s_trace_pose.valid && isfinite(s_trace_pose.x_mm) &&
        isfinite(s_trace_pose.y_mm) && isfinite(s_trace_pose.heading_deg)) {
        x=roundf(s_trace_pose.x_mm*10.0f);
        y=roundf(s_trace_pose.y_mm*10.0f);
        yaw=roundf(remainderf(s_trace_pose.heading_deg,360.0f)*100.0f);
        if (x>=-32768.0f && x<=32767.0f && y>=-32768.0f && y<=32767.0f &&
            yaw>=-18000.0f && yaw<=18000.0f) {
            record.x_decimm=(int16_t)x;
            record.y_decimm=(int16_t)y;
            record.yaw_centideg=(int16_t)yaw;
            record.meta|=ZONE2_TRACE_VALID;
        }
    }
    if (terminal) record.meta|=ZONE2_TRACE_TERMINAL;
    g_zone2_pose_trace.records[count]=record;
    g_zone2_pose_trace.count=count+1U;
    if (terminal) {
        g_zone2_pose_trace.fault=(uint32_t)g_zone2_fault;
        g_zone2_pose_trace.state=ZONE2_TRACE_FROZEN;
    }
}
#endif

static uint32_t now_ms(void)
{
    return (uint32_t)xTaskGetTickCount()*portTICK_PERIOD_MS;
}

static uint32_t newer(uint32_t stamp,uint32_t anchor)
{
    return (int32_t)(stamp-anchor)>0;
}


#if APP_ZONE2_MISSION_ENABLE
void AppZone2_OfferTaskCode(const char *code,uint32_t first_ms,uint32_t last_ms)
{
    uint32_t now=now_ms();
    if (!FieldMap_TaskCodeValid(code) || now-last_ms>VEHICLE_STAGE1_SCAN_FRESH_MS ||
        (int32_t)(last_ms-first_ms)<0) return;
    taskENTER_CRITICAL();
    if (s_mission.scan_open && newer(first_ms,s_mission.scan_after_ms) && !s_mission.task_code[0]) {
        memcpy(s_mission.task_code,code,sizeof(s_mission.task_code));
        s_mission.display_pending=2U;
        s_mission.scan_open=0U;
    }
    taskEXIT_CRITICAL();
}

void AppZone2_DisplayTaskCode(void)
{
    char code[16]={0};
    uint32_t result_pending,picks,places;
    taskENTER_CRITICAL();
    result_pending=s_mission.page1_pending;
    picks=g_zone2_mission_trace.picks;
    places=g_zone2_mission_trace.places;
    if (!result_pending && s_mission.display_pending) {
        memcpy(code,s_mission.task_code,sizeof(code));
        s_mission.display_pending--;
    }
    taskEXIT_CRITICAL();
    if (result_pending) {
        if (Screen_ShowResult(picks,places)==SCREEN_OK) {
            g_zone2_mission_trace.page1_ms=now_ms();
            s_mission.page1_pending=0U;
        }
        return;
    }
    if (!code[0]) return;
    g_zone2_task_display_result=Screen_ShowTaskCode(code);
    if (g_zone2_task_display_result==SCREEN_OK) s_mission.display_pending=0U;
}

uint32_t AppZone2_CopyTaskCode(char code[16])
{
    uint32_t ready;
    if (!code) return 0U;
    taskENTER_CRITICAL();
    ready=s_mission.task_code[0]!=0;
    if (ready) memcpy(code,s_mission.task_code,sizeof(s_mission.task_code));
    taskEXIT_CRITICAL();
    return ready;
}

uint32_t AppZone2_StationRead(AppZone2Station *station)
{
    uint32_t ready=0U;
    if (!station) return 0U;
    taskENTER_CRITICAL();
    if (g_zone2_phase==ZONE2_STATION_WAIT && !s_mission.station_result) {
        *station=s_mission.station;
        ready=1U;
    }
    taskEXIT_CRITICAL();
    return ready;
}

static uint8_t station_receipts(uint8_t kind)
{
    return kind==APP_ZONE2_ROUGH?12U:6U;
}

static uint8_t storage_ring_for_color(uint8_t color)
{
    uint8_t ring;
    for (ring=1U;ring<=3U;ring++) {
#if APP_ZONE2_UNVERIFIED_TRIAL_ENABLE
        if (s_mission.planned_base_color[ring]==color) return ring;
#else
        if (s_mission.base_color[ring]==color) return ring;
#endif
    }
    return 0U;
}

static uint32_t confirm_transfer(uint8_t item,AppZone2Action action,uint8_t color,uint8_t ring)
{
    switch (action) {
    case APP_ZONE2_SOURCE_GRIP:
    case APP_ZONE2_ROUGH_GRIP:
        g_zone2_mission_trace.picks++;
        break;
    case APP_ZONE2_SOURCE_SLOT:
    case APP_ZONE2_ROUGH_SLOT:
        if (s_mission.slots[item]) return 0U;
        s_mission.slots[item]=color;
        break;
    case APP_ZONE2_ROUGH_SLOT_GRIP:
    case APP_ZONE2_STORAGE_SLOT_GRIP:
        if (s_mission.slots[item]!=color) return 0U;
        s_mission.slots[item]=0U;
        break;
    case APP_ZONE2_STORAGE_PLACE:
#if APP_ZONE2_UNVERIFIED_TRIAL_ENABLE
        break;
#else
        if (s_mission.station.kind==APP_ZONE2_STORAGE_BASE) {
            if (s_mission.base_color[ring]) return 0U;
            s_mission.base_color[ring]=color;
        } else if (s_mission.base_color[ring]!=color) return 0U;
        g_zone2_mission_trace.storage_action_state[s_mission.station.leg-2U][item]=
            APP_ZONE2_STATION_VERIFIED;
        /* Fall through to count confirmed field placements. */
#endif
    case APP_ZONE2_ROUGH_PLACE:
        g_zone2_mission_trace.places++;
        break;
    default:
        return 0U;
    }
    return 1U;
}

uint32_t AppZone2_StationConfirm(uint32_t ticket,uint8_t item,AppZone2Action action,
    uint8_t color,uint8_t ring)
{
    static const AppZone2Action source[]={APP_ZONE2_SOURCE_GRIP,APP_ZONE2_SOURCE_SLOT};
    static const AppZone2Action rough[]={APP_ZONE2_ROUGH_SLOT_GRIP,APP_ZONE2_ROUGH_PLACE,
        APP_ZONE2_ROUGH_GRIP,APP_ZONE2_ROUGH_SLOT};
    static const AppZone2Action storage[]={APP_ZONE2_STORAGE_SLOT_GRIP,APP_ZONE2_STORAGE_PLACE};
    const AppZone2Station *station=&s_mission.station;
    const AppZone2Action *expected;
    uint8_t count;
    uint32_t accepted=0U;

    taskENTER_CRITICAL();
    count=s_mission.receipt_count;
    expected=station->kind==APP_ZONE2_SOURCE?source:
        station->kind==APP_ZONE2_ROUGH?rough:storage;
    if (g_zone2_phase==ZONE2_STATION_WAIT && ticket && ticket==station->ticket &&
        !s_mission.station_result && count<station_receipts(station->kind) &&
        item==(count/2U)%3U && action==expected[(count>=6U?2U:0U)+count%2U] &&
        color==station->colors[item] && ring==station->rings[item] &&
        confirm_transfer(item,action,color,ring)) {
        g_zone2_mission_trace.receipt_ms[station->leg-2U][count]=now_ms();
        s_mission.receipt_count++;
        accepted=1U;
    }
    taskEXIT_CRITICAL();
    return accepted;
}

uint32_t AppZone2_StationComplete(uint32_t ticket,uint32_t success)
{
    uint32_t accepted=0U;
    taskENTER_CRITICAL();
    if (g_zone2_phase==ZONE2_STATION_WAIT && ticket && ticket==s_mission.station.ticket &&
        !s_mission.station_result && success<=1U &&
#if APP_ZONE2_UNVERIFIED_TRIAL_ENABLE
        !(success && (s_mission.station.kind==APP_ZONE2_STORAGE_BASE ||
                     s_mission.station.kind==APP_ZONE2_STORAGE_STACK)) &&
#endif
        (!success || (s_mission.receipt_count==station_receipts(s_mission.station.kind) &&
            !SourceBodyStep_Busy() && !g_source_body_step_request))) {
        s_mission.station_result=success?1U:2U;
        s_mission.station_result_ms=now_ms();
        accepted=1U;
    }
    taskEXIT_CRITICAL();
    return accepted;
}

uint32_t AppZone2_StationCompleteUnverified(uint32_t ticket)
{
    uint32_t accepted=0U;
#if APP_ZONE2_UNVERIFIED_TRIAL_ENABLE
    taskENTER_CRITICAL();
    if (g_zone2_phase==ZONE2_STATION_WAIT && ticket && ticket==s_mission.station.ticket &&
        !s_mission.station_result &&
        (s_mission.station.kind==APP_ZONE2_STORAGE_BASE ||
         s_mission.station.kind==APP_ZONE2_STORAGE_STACK) &&
        s_mission.receipt_count==station_receipts(s_mission.station.kind) &&
        !SourceBodyStep_Busy() && !g_source_body_step_request) {
        uint8_t item;
        for (item=0U;item<3U;item++)
            g_zone2_mission_trace.storage_action_state[s_mission.station.leg-2U][item]=
                APP_ZONE2_STATION_ACTION_COMPLETE_UNVERIFIED;
        g_zone2_mission_trace.unverified_storage_actions+=3U;
        s_mission.station_result=APP_ZONE2_STATION_ACTION_COMPLETE_UNVERIFIED;
        s_mission.station_result_ms=now_ms();
        accepted=1U;
    }
    taskEXIT_CRITICAL();
#else
    (void)ticket;
#endif
    return accepted;
}

static uint32_t scan_has_code(void)
{
    uint32_t ready;
    taskENTER_CRITICAL();
    ready=s_mission.task_code[0]!=0;
    taskEXIT_CRITICAL();
    return ready;
}
#endif

static uint32_t healthy(void)
{
    return !g_safety_fault && !g_rtos_fault && !g_can_ctrl_fault_latched &&
        g_can_test_result<=1U && !g_chassis_auto_cut_vm;
}

/* PWM command and released coils only. The lift has no stowed position sensor. */
static uint32_t stowed_command(void)
{
    RadarLiftSnapshot lift;
    uint32_t zone=s_zone_locked;
    if(zone!=1U && zone!=2U) return 0U;
    RadarLift_GetSnapshot(&lift);
    return g_radar_servo_state==1U &&
        g_radar_servo_angle_deg[zone-1U]==(zone==1U?RADAR_SERVO1_INIT_DEG:RADAR_SERVO2_INIT_DEG) &&
        !g_radar_lift_request && !g_radar_lift_phase_bits && !g_radar_lift_enable &&
        (lift.state==0U || (lift.state==2U && lift.result==1U &&
         g_radar_lift_direction==(zone==1U?(RADAR_ZONE1_DOWN_DIRECTION^1U):0U)));
}

static void fail(Zone2Fault reason)
{
#if APP_ZONE2_MISSION_ENABLE
    taskENTER_CRITICAL();
    s_mission.scan_open=0U;
    if (s_station_body.phase!=STATION_BODY_IDLE || g_source_body_step_request) {
        g_source_body_step_state=9U;
        g_source_body_step_reason=(uint32_t)reason;
        s_station_body.phase=STATION_BODY_IDLE;
        g_source_body_step_request=0U;
        g_source_align_chain_active=0U;
    }
    g_zone2_mission_trace.failure_ms=now_ms();
    g_zone2_phase=ZONE2_FAILED;
    taskEXIT_CRITICAL();
#endif
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
        map->run_id==ZONE2_RUN_ID && map->start_id==s_zone_locked &&
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
    uint32_t i, source_ready;

    source_ready=sources(can,&now,&map,heading);
#if APP_ZONE2_POSE_TRACE_ENABLE
    s_trace_pose.source_ms=map.pose.sample_ms;
#endif
    if (!source_ready) return 0U;
    for (i=0U;i<4U;i++) driver_rpm[i]=0.1f*(float)can->wheel_decirpm[i];
    if (!Chassis_DriverFeedbackToLogical(driver_rpm,can->position_mask,&wheels) ||
        !Chassis_Forward(&g_chassis_geometry,&wheels,&body)) return 0U;
    c=cosf(map.pose.heading_deg*ZONE2_DEG_TO_RAD);
    s=sinf(map.pose.heading_deg*ZONE2_DEG_TO_RAD);
    *o=(FusionObservation){0};
    if (s_schedule.plan.valid) o->identity=s_schedule.plan.input.identity;
    else o->identity=(FusionIdentity){ZONE2_RUN_ID,s_zone_locked,
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
#if APP_ZONE2_POSE_TRACE_ENABLE
    s_trace_pose.x_mm=o->position.x_mm;
    s_trace_pose.y_mm=o->position.y_mm;
    s_trace_pose.heading_deg=o->heading_deg;
    s_trace_pose.valid=1U;
#endif
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
#if !APP_ZONE2_MISSION_ENABLE
    s_section_progress_started=0U;
#endif
    s_generation=0U;
    s_correction=(FusionMotionVector){0};
    g_zone2_phase=ZONE2_TRACK;
}

static uint32_t section_error(const FusionObservation *pose,const FusionMotionVector *endpoint,float *error)
{
    float position=hypotf(pose->position.x_mm-endpoint->x,pose->position.y_mm-endpoint->y);
    float heading=pose->heading_deg-endpoint->yaw/ZONE2_DEG_TO_RAD;
    float yaw;
    if (!(position<=FLT_MAX && heading>=-FLT_MAX && heading<=FLT_MAX)) return 0U;
    yaw=fabsf(remainderf(heading,360.0f));
    *error=fmaxf(position/ZONE2_SECTION_POSITION_MM,yaw/ZONE2_SECTION_HEADING_DEG);
    return 1U;
}

#if !APP_ZONE2_MISSION_ENABLE
static uint32_t section_stalled(const FusionObservation *pose,float error)
{
    if (!s_section_progress_started || newer(pose->sample_ms,s_section_sample_ms)) {
        if (!s_section_progress_started || s_section_checkpoint-error>=ZONE2_SECTION_PROGRESS_STEP) {
            s_section_checkpoint=error;
            s_section_progress_ms=pose->now_ms;
        }
        s_section_sample_ms=pose->sample_ms;
        s_section_progress_started=1U;
    }
    return pose->now_ms-s_section_progress_ms>=ZONE2_SECTION_PROGRESS_MS;
}
#endif

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
    if (!Zone2MotionReference_Sample(&s_trajectory,s_section,sample,&in.reference)) {
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

static void finish_navigation(void)
{
    RadarObserver_Cancel();
    (void)ChassisAuto_Zone2Disarm();
    s_primed=0U;
    RadarZoneSwitch_Unlock();
    s_zone_locked=0U;
#if APP_ZONE2_MISSION_ENABLE
    g_zone2_mission_trace.return_ms=now_ms();
    taskENTER_CRITICAL();
    s_mission.page1_pending=1U;
    g_zone2_phase=ZONE2_MISSION_DONE;
    taskEXIT_CRITICAL();
#else
    g_zone2_phase=ZONE2_DONE;
#endif
    g_zone2_reference_status=FUSION_DONE;
}

#if APP_ZONE2_MISSION_ENABLE
static uint32_t start_scan_shift(ChassisZone2CanStatus *can,const FusionObservation *pose)
{
    ChassisAutoStatus backend;
    Zone2NavGeometry geometry=s_schedule.geometry;
    Zone2NavRequest request={s_schedule.map.context,ZONE2_QR_PATH_ID+s_mission.search_count,0U,2U,2U,0U};
    Zone2NavReply reply={0};

    if (!(fabsf(remainderf(pose->heading_deg-90.0f,360.0f))<=s_config.heading_budget_deg)) return 0U;
    geometry.actual=pose->position;
    geometry.terminal=(FusionPoint){
        FIELD_SCAN_X_MM+(s_mission.search_count==0U?ZONE2_QR_SHIFT_MM:-ZONE2_QR_SHIFT_MM),
        s_mission.scan_y_mm};
    geometry.config.start_heading_deg=90.0f;
    geometry.config.goal_heading_deg=90.0f;
    geometry.speed_mm_s=ZONE2_QR_SPEED_MM_S;
    reply.context=s_schedule.map.context;
    reply.path.start=reply.path.goal=2U;
    s_schedule.error=Zone2NavPath_Prepare(&s_schedule.map,&request,&reply,&geometry,
        &s_schedule.route,&s_schedule.plan);
    if (s_schedule.error!=ZONE2_NAV_OK ||
        !Zone2MotionReference_Prepare(&s_schedule.plan,&s_trajectory)) return 0U;
    if (!ChassisAuto_Zone2ReleaseCompleted(ZONE2_RUN_ID,can->session) ||
        !ChassisAuto_ReadStatus(&backend) || backend.active ||
        !ChassisAuto_Zone2Begin(ZONE2_RUN_ID,backend.session)) return 0U;
    ChassisAuto_Zone2Read(can);
    s_session=can->session;
    s_power_epoch=can->power_epoch;
    s_section=0U;
    s_station_stop=0U;
    s_mission.search_count++;
    s_mission.search_active=1U;
    s_mission.search_interrupted=0U;
    g_zone2_reference_status=FUSION_TURN_START;
    g_zone2_phase=ZONE2_ZERO;
    return 1U;
}

static void wait_at_scan(ChassisZone2CanStatus *can,const FusionObservation *pose,
    const AppHeadingSource *heading,uint32_t observed,uint32_t now)
{
    uint32_t ready=scan_has_code();
    if (!ready && (s_mission.search_count>=2U ||
        now-s_mission.scan_wait_ms<ZONE2_QR_DWELL_MS)) return;
    if (!observed || !can->wheel_quiet || !quiet_since(pose,heading,can->stop_ack_ms)) {
        fail(ZONE2_FAULT_SOURCE);
        return;
    }
    if (ready) {
        if (!ChassisAuto_Zone2ReleaseCompleted(ZONE2_RUN_ID,can->session)) {
            fail(ZONE2_FAULT_BACKEND);
            return;
        }
        g_zone2_mission_trace.scan_end_ms=now;
        g_zone2_phase=ZONE2_PREPARE;
    } else if (!start_scan_shift(can,pose)) {
        fail(ZONE2_FAULT_QR);
    }
}

static uint32_t station_step_clear(FusionPoint from,FusionPoint target)
{
    static const FusionBox yellow[]={{550,550,1000,1000},{550,1400,1000,1850},
        {1400,550,1850,1000},{1400,1400,1850,1850}};
    const float radius=0.5f*hypotf(VEHICLE_BODY_LENGTH_MM,VEHICLE_BODY_WIDTH_MM)+STATION_SWEEP_RESERVE_MM;
    FusionBox hull={fminf(from.x_mm,target.x_mm)-radius,fminf(from.y_mm,target.y_mm)-radius,
        fmaxf(from.x_mm,target.x_mm)+radius,fmaxf(from.y_mm,target.y_mm)+radius};
    FieldPoint center;
    uint32_t i;
    if (!(hull.min_x>0.0f && hull.min_y>0.0f &&
          hull.max_x<FIELD_SIZE_MM && hull.max_y<FIELD_SIZE_MM)) return 0U;
    for (i=0U;i<sizeof(yellow)/sizeof(yellow[0]);i++) {
        if (hull.min_x<=yellow[i].max_x && hull.max_x>=yellow[i].min_x &&
            hull.min_y<=yellow[i].max_y && hull.max_y>=yellow[i].min_y) return 0U;
    }
    for (i=0U;i<25U;i++) {
        const float half=FIELD_GRID_SIZE_MM*0.5f;
        if ((s_schedule.map.occupied_mask&(1UL<<i)) && FieldMap_CellCenter(i,&center) &&
            hull.min_x<=center.x_mm+half && hull.max_x>=center.x_mm-half &&
            hull.min_y<=center.y_mm+half && hull.max_y>=center.y_mm-half) return 0U;
    }
    return 1U;
}

static uint32_t station_step_target(const FusionObservation *pose,uint32_t request,
    uint32_t direction,float amount)
{
    static const float offset[]={0.0f,90.0f,-90.0f,180.0f};
    float angle;
    s_station_body.target=pose->position;
    s_station_body.target_yaw=pose->heading_deg;
    if (request==2U) {
        if (direction>3U || !(amount>=0.25f && amount<=STATION_STEP_MM)) return 0U;
        angle=(pose->heading_deg+offset[direction])*ZONE2_DEG_TO_RAD;
        s_station_body.target.x_mm+=amount*cosf(angle);
        s_station_body.target.y_mm+=amount*sinf(angle);
    } else if (request==3U) {
        if (!(fabsf(amount)>STATION_STOP_DEG && fabsf(amount)<=STATION_STEP_DEG+0.5f)) return 0U;
        s_station_body.target_yaw+=amount;
    } else return 0U;
    if (!(hypotf(s_station_body.target.x_mm-s_station_body.anchor.x_mm,
                 s_station_body.target.y_mm-s_station_body.anchor.y_mm)<=STATION_STEP_MM+0.01f) ||
        !(fabsf(remainderf(s_station_body.target_yaw-s_station_body.nominal_yaw,360.0f))<=STATION_STEP_DEG+0.01f))
        return 0U;
    return station_step_clear(pose->position,s_station_body.target);
}

static uint32_t station_step_begin(const ChassisZone2CanStatus *can,uint32_t restoring,uint32_t now)
{
    ChassisAutoStatus backend;
    ChassisZone2CanStatus started;
    if (can->state!=CHASSIS_AUTO_IDLE || !ChassisAuto_ReadStatus(&backend) || backend.active ||
        !ChassisAuto_Zone2Begin(ZONE2_RUN_ID,backend.session)) return 0U;
    ChassisAuto_Zone2Read(&started);
    s_session=started.session;
    s_power_epoch=started.power_epoch;
    s_station_body.started_ms=now;
    s_station_body.phase=STATION_BODY_ZERO;
    s_station_body.restoring=restoring;
    g_source_body_step_state=1U;
    g_source_body_step_session=s_session;
    g_source_body_step_submitted=0U;
    g_source_align_chain_active=!restoring;
    return 1U;
}

static uint32_t station_step_commands(const FusionObservation *pose,ChassisMotorCommand commands[4],
    uint32_t *at_target)
{
    const float dx=s_station_body.target.x_mm-pose->position.x_mm;
    const float dy=s_station_body.target.y_mm-pose->position.y_mm;
    const float distance=hypotf(dx,dy);
    const float angle=remainderf(s_station_body.target_yaw-pose->heading_deg,360.0f);
    const float heading=pose->heading_deg*ZONE2_DEG_TO_RAD;
    const float c=cosf(heading),s=sinf(heading);
    const float gain=distance>STATION_STOP_MM?fminf(2.0f,15.0f/distance):0.0f;
    ChassisBodyVelocity body={gain*(dx*c+dy*s),gain*(dy*c-dx*s),
        fabsf(angle)>STATION_STOP_DEG?fmaxf(-10.0f,fminf(10.0f,2.0f*angle))*ZONE2_DEG_TO_RAD:0.0f};
    ChassisWheelSpeeds wheels;
    if (!(distance<=STATION_STEP_MM+STATION_SWEEP_RESERVE_MM && fabsf(angle)<=2.0f*STATION_STEP_DEG) ||
        !station_step_clear(pose->position,s_station_body.target)) return 0U;
    *at_target=distance<=STATION_STOP_MM && fabsf(angle)<=STATION_STOP_DEG;
    return Chassis_Inverse(&g_chassis_geometry,&body,20.0f,&wheels,0) &&
        Chassis_EncodeEmm(&wheels,200U,commands);
}

static void station_body_service(const ChassisZone2CanStatus *can,const FusionObservation *pose,
    const AppHeadingSource *heading,uint32_t observed,uint32_t now)
{
    ChassisMotorCommand commands[4];
    uint32_t request=g_source_body_step_request,at_target;
    uint32_t i;
    float amount;

    if (request==9U) {
        fail(ZONE2_FAULT_STATION);
        return;
    }
    if (s_station_body.phase==STATION_BODY_IDLE) {
        if (!request || s_mission.station_result) return;
        if (request==7U) {
            g_source_align_chain_active=0U;
            g_source_body_step_request=0U;
            return;
        }
        if (!observed || can->state!=CHASSIS_AUTO_IDLE ||
            !quiet_since(pose,heading,s_station_body.ready_ms)) return;
        for (i=0U;i<4U;i++) {
            if (can->wheel_decirpm[i] || !newer(can->wheel_ms[i],s_station_body.ready_ms)) return;
        }
        amount=request==2U?g_source_align_distance_mm:g_source_align_turn_deg;
        if (!station_step_target(pose,request,g_source_align_direction,amount) ||
            !station_step_begin(can,0U,now)) {
            g_source_align_rejected++;
            fail(ZONE2_FAULT_STATION);
            return;
        }
        g_source_body_step_request=0U;
        return;
    }
    g_source_body_step_elapsed_ms=now-s_station_body.started_ms;
    if (can->session!=s_session || can->power_epoch!=s_power_epoch || can->cause ||
        can->state==CHASSIS_AUTO_FAULT || can->state==CHASSIS_AUTO_CANCELLED) {
        fail(ZONE2_FAULT_BACKEND);
        return;
    }
    if (s_station_body.phase==STATION_BODY_STOP) {
        if (can->state==CHASSIS_AUTO_DONE && can->result==1U &&
            can->wheel_quiet && observed && quiet_since(pose,heading,can->stop_ack_ms)) {
            if (hypotf(pose->position.x_mm-s_station_body.target.x_mm,
                       pose->position.y_mm-s_station_body.target.y_mm)>1.0f ||
                fabsf(remainderf(pose->heading_deg-s_station_body.target_yaw,360.0f))>0.5f ||
                !ChassisAuto_Zone2ReleaseCompleted(ZONE2_RUN_ID,can->session)) {
                fail(ZONE2_FAULT_STATION);
                return;
            }
            s_station_body.ready_ms=now;
            s_station_body.phase=STATION_BODY_IDLE;
            g_source_body_step_state=3U;
            if (!s_station_body.restoring) g_source_align_command_seq++;
        } else if (now-s_station_body.stop_ms>=CHASSIS_AUTO_STOP_MS+CHASSIS_AUTO_SETTLE_MS) {
            fail(ZONE2_FAULT_BACKEND);
        }
        return;
    }
    if (!observed || can->state==CHASSIS_AUTO_STOP ||
        can->state==CHASSIS_AUTO_SETTLE || can->state==CHASSIS_AUTO_DONE) {
        fail(ZONE2_FAULT_SOURCE);
        return;
    }
    if (!can->ready) return;
    if (s_station_body.phase==STATION_BODY_ZERO) {
        if (!can->zero_confirmed || !quiet_since(pose,heading,can->zero_ack_ms)) {
            if (!submit_zero(can)) fail(ZONE2_FAULT_BACKEND);
            return;
        }
        s_station_body.phase=STATION_BODY_TRACK;
    }
    if (!station_step_commands(pose,commands,&at_target)) {
        fail(ZONE2_FAULT_REFERENCE);
    } else if (at_target) {
        if (!ChassisAuto_RequestStop()) {
            fail(ZONE2_FAULT_BACKEND);
            return;
        }
        s_station_body.stop_ms=now;
        s_station_body.phase=STATION_BODY_STOP;
    } else if (!ChassisAuto_Zone2SubmitAt(ZONE2_RUN_ID,s_session,can->ack_generation+1U,
        can->next_batch_ms,commands)) {
        fail(ZONE2_FAULT_BACKEND);
    } else {
        g_source_body_step_submitted++;
    }
}

static void wait_at_station(const ChassisZone2CanStatus *can,const FusionObservation *pose,
    const AppHeadingSource *heading,uint32_t observed,uint32_t now)
{
    uint32_t result,stamp,i;
    float heading_error;
    taskENTER_CRITICAL();
    result=s_mission.station_result;
    stamp=s_mission.station_result_ms;
    taskEXIT_CRITICAL();
    if (!result || SourceBodyStep_Busy()) {
        station_body_service(can,pose,heading,observed,now);
        return;
    }
    if (result==2U) {
        fail(ZONE2_FAULT_STATION);
        return;
    }
    if (result!=1U && result!=APP_ZONE2_STATION_ACTION_COMPLETE_UNVERIFIED) return;
    if (newer(s_station_body.ready_ms,stamp)) stamp=s_station_body.ready_ms;
    if (!observed || !quiet_since(pose,heading,stamp) || !can->feedback_ready ||
        can->state!=CHASSIS_AUTO_IDLE) {
        if (now-stamp>=CHASSIS_AUTO_STOP_MS+CHASSIS_AUTO_SETTLE_MS) fail(ZONE2_FAULT_SOURCE);
        return;
    }
    for (i=0U;i<4U;i++) {
        if (can->wheel_decirpm[i] || !newer(can->wheel_ms[i],stamp)) {
            if (now-stamp>=CHASSIS_AUTO_STOP_MS+CHASSIS_AUTO_SETTLE_MS) fail(ZONE2_FAULT_SOURCE);
            return;
        }
    }
    heading_error=remainderf(s_station_body.nominal_yaw-pose->heading_deg,360.0f);
    if (fabsf(heading_error)>s_config.heading_budget_deg) {
        if (!station_step_target(pose,3U,0U,heading_error) || !station_step_begin(can,1U,now))
            fail(ZONE2_FAULT_STATION);
        return;
    }
    g_zone2_mission_trace.station[s_mission.station.leg-2U].end_ms=now;
    g_zone2_phase=ZONE2_PREPARE;
}

static uint32_t offer_station(uint32_t now,const FusionObservation *pose)
{
    static const uint8_t kinds[6]={APP_ZONE2_SOURCE,APP_ZONE2_ROUGH,APP_ZONE2_STORAGE_BASE,
        APP_ZONE2_SOURCE,APP_ZONE2_ROUGH,APP_ZONE2_STORAGE_STACK};
    AppZone2Station station={0};
    Zone2NavLeg leg;
    uint8_t i,offset;

    if (s_schedule.leg<2U || s_schedule.leg>=ZONE2_NAV_LEGS ||
        !Zone2NavPath_Leg(s_schedule.start_id,s_schedule.leg-1U,&leg) || !scan_has_code()) return 0U;
    station.leg=(uint8_t)s_schedule.leg;
    station.ticket=s_mission.station.ticket+1U;
    station.run_id=s_schedule.run_id;
    station.cell=leg.goal;
    station.kind=kinds[station.leg-2U];
    station.batch=station.leg<5U?1U:2U;
    offset=station.batch==1U?0U:8U;
    memcpy(station.task_code,s_mission.task_code,sizeof(station.task_code));
    for (i=0U;i<3U;i++) {
        station.colors[i]=(uint8_t)(station.task_code[offset+i]-'0');
        if (station.kind==APP_ZONE2_SOURCE) {
            if (s_mission.slots[i]) return 0U;
            continue;
        }
        if (s_mission.slots[i]!=station.colors[i]) return 0U;
        if (station.kind!=APP_ZONE2_STORAGE_STACK) {
            station.rings[i]=(uint8_t)(station.task_code[offset+4U+i]-'0');
        } else {
            station.rings[i]=storage_ring_for_color(station.colors[i]);
            if (!station.rings[i]) return 0U;
        }
    }
#if APP_ZONE2_UNVERIFIED_TRIAL_ENABLE
    if (station.kind==APP_ZONE2_STORAGE_BASE) {
        for (i=0U;i<3U;i++)
            s_mission.planned_base_color[station.rings[i]]=station.colors[i];
    }
#endif
    taskENTER_CRITICAL();
    s_mission.station=station;
    s_mission.receipt_count=0U;
    s_mission.station_result=0U;
    s_station_body.anchor=pose->position;
    s_station_body.nominal_yaw=leg.goal_heading_deg;
    s_station_body.ready_ms=pose->sample_ms;
    g_zone2_mission_trace.station[station.leg-2U].start_ms=now;
    g_zone2_phase=ZONE2_STATION_WAIT;
    taskEXIT_CRITICAL();
    return 1U;
}
#endif

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
    if (
#if APP_ZONE2_MISSION_ENABLE
        !s_mission.search_interrupted &&
#endif
        (!(hypotf(pose->position.x_mm-endpoint.x,pose->position.y_mm-endpoint.y)<=
        s_config.position_tolerance_mm) ||
        !(fabsf(remainderf(pose->heading_deg-endpoint.yaw/ZONE2_DEG_TO_RAD,360.0f))<=
        s_config.heading_budget_deg))) {
        fail(ZONE2_FAULT_REFERENCE);
        return;
    }
    pose->stopped=1U;
#if APP_ZONE2_MISSION_ENABLE
    if (s_mission.search_active) {
        s_schedule.plan.valid=0U;
        s_mission.search_active=0U;
        s_mission.search_interrupted=0U;
        s_mission.scan_wait_ms=now;
        g_zone2_stop_pose_ms=pose->sample_ms;
        g_zone2_phase=ZONE2_QR_WAIT;
        return;
    }
#endif
    if (!Zone2NavSchedule_Arrive(&s_schedule,s_session,true,pose,now)) {
        fail(ZONE2_FAULT_PATH);
        return;
    }
    g_zone2_completed_legs=s_schedule.leg;
    g_zone2_stop_pose_ms=pose->sample_ms;
#if APP_ZONE2_MISSION_ENABLE
    g_zone2_mission_trace.navigation[s_schedule.leg-1U].end_ms=now;
    if (s_schedule.leg==1U) {
        s_mission.scan_y_mm=pose->position.y_mm;
        s_mission.scan_wait_ms=now;
        g_zone2_mission_trace.scan_start_ms=now;
        taskENTER_CRITICAL();
        s_mission.scan_after_ms=now;
        s_mission.scan_open=1U;
        g_zone2_phase=ZONE2_QR_WAIT;
        taskEXIT_CRITICAL();
    } else if (s_schedule.phase==ZONE2_SCHEDULE_DONE) {
        finish_navigation();
    } else if (!ChassisAuto_Zone2ReleaseCompleted(ZONE2_RUN_ID,can->session)) {
        fail(ZONE2_FAULT_BACKEND);
    } else if (!offer_station(now,pose)) {
        fail(ZONE2_FAULT_STATION);
    }
#else
    if (s_schedule.phase==ZONE2_SCHEDULE_DONE) {
        finish_navigation();
    } else if (ChassisAuto_Zone2ReleaseCompleted(ZONE2_RUN_ID,can->session)) {
        s_prepare_ms=now;
        g_zone2_phase=ZONE2_PREPARE;
    } else {
        fail(ZONE2_FAULT_BACKEND);
    }
#endif
}
#endif

void AppZone2_RequestStart(void)
{
    taskENTER_CRITICAL();
    if (g_boot_app_init_done==1U && !Buzzer_IsBusy() && g_zone2_phase==ZONE2_IDLE && !s_start_request) {
        uint32_t zone=RadarZoneSwitch_Lock();
        if (zone==ZONE2_START_ID || (zone==1U && APP_ZONE1_RADAR_LIFT_CONFIRMED)) {
            s_zone_locked=zone;
#if APP_ZONE2_MISSION_ENABLE
            g_zone2_mission_trace.key_ms=now_ms();
#endif
            s_start_request=1U;
        } else if (zone) RadarZoneSwitch_Unlock();
    }
    taskEXIT_CRITICAL();
}

uint32_t AppZone2_CanPermit(uint32_t slot_ms)
{
    ChassisZone2CanStatus can;
    AppHeadingSource heading;
    AppMapPose map;
    uint32_t now=now_ms();
    (void)slot_ms;
    if (!healthy() || !current_map()) return 0U;
    if (g_zone2_phase!=ZONE2_ZERO && g_zone2_phase!=ZONE2_TRACK &&
        g_zone2_phase!=ZONE2_SECTION_ZERO
#if APP_ZONE2_MISSION_ENABLE
        && !(g_zone2_phase==ZONE2_STATION_WAIT && SourceBodyStep_WheelWindow())
#endif
        ) return 0U;
    ChassisAuto_Zone2Read(&can);
    return can.session==s_session && sources(&can,&now,&map,&heading);
}

#if APP_ZONE2_POSE_TRACE_ENABLE
static void zone2_step(void)
#else
void AppZone2_Step(void)
#endif
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

    BootReady_BeepStep();

    if (g_zone2_phase==ZONE2_IDLE) {
        taskENTER_CRITICAL();
        if (s_start_request) {
            s_start_request=0U;
            s_begin_ms=now;
            g_zone2_phase=ZONE2_WAIT_READY;
        }
        taskEXIT_CRITICAL();
#if APP_ZONE2_POSE_TRACE_ENABLE
        if (g_zone2_phase==ZONE2_WAIT_READY && !g_zone2_pose_trace.state) {
            g_zone2_pose_trace.magic=ZONE2_TRACE_MAGIC;
            Zone2NavSchedule_TraceStart(s_begin_ms);
            trace_capture(s_begin_ms,0U);
            g_zone2_pose_trace.state=ZONE2_TRACE_CAPTURE;
        }
#endif
    }
    if (g_zone2_phase==ZONE2_FAILED) {
        if (s_primed && !g_chassis_auto_active) {
            (void)ChassisAuto_Zone2Disarm();
            s_primed=0U;
        }
        return;
    }
#if APP_ZONE2_MISSION_ENABLE
    if (g_zone2_phase==ZONE2_MISSION_DONE) return;
#endif
    if (g_zone2_phase==ZONE2_IDLE || g_zone2_phase==ZONE2_SHADOW_DONE || g_zone2_phase==ZONE2_DONE) return;
    if (!healthy()) {
        fail(ZONE2_FAULT_HEALTH);
        return;
    }
    if (g_zone2_phase==ZONE2_WAIT_READY) {
        if (g_can_test_result==1U && g_safety_ready && s_zone_locked &&
            stowed_command() && RadarObserver_IsScanReady() &&
            !g_rtos_fault && !g_safety_fault && !g_can_ctrl_fault_latched) {
            if (!s_startup_axes_started) {
                if (!StartupAxes_Begin()) { fail(ZONE2_FAULT_HEALTH); return; }
                s_startup_axes_started=1U;
            }
            if (!Zone2NavSchedule_Start(&s_schedule,ZONE2_RUN_ID,(uint8_t)s_zone_locked,
                VEHICLE_MAP_VERSION,true,&s_config,400.0f)) {
                StartupAxes_Cancel();
                fail(ZONE2_FAULT_PATH);
                return;
            }
            g_zone2_phase=ZONE2_SCAN;
            return;
        }
#if !APP_ZONE2_MISSION_ENABLE
        if (now-s_begin_ms>=ZONE2_WAIT_READY_MS) fail(ZONE2_FAULT_DEADLINE);
#endif
        return;
    }
    ChassisAuto_Zone2Read(&can);
    observed=observation(&can,now,&pose,&heading);
    if (observed) now=pose.now_ms;
#if APP_ZONE2_MISSION_ENABLE
    if (g_zone2_phase==ZONE2_QR_WAIT || g_zone2_phase==ZONE2_STATION_WAIT || s_mission.search_active) {
        if (!current_map()) {
            fail(ZONE2_FAULT_PATH);
            return;
        }
        if (g_zone2_phase==ZONE2_QR_WAIT) {
            wait_at_scan(&can,&pose,&heading,observed,now);
            return;
        }
        if (g_zone2_phase==ZONE2_STATION_WAIT) {
            wait_at_station(&can,&pose,&heading,observed,now);
            return;
        }
    }
    schedule_phase=s_mission.search_active?ZONE2_SCHEDULE_ACTIVE:
        Zone2NavSchedule_Poll(&s_schedule,observed?&pose:0,now);
#else
    schedule_phase=Zone2NavSchedule_Poll(&s_schedule,observed?&pose:0,now);
#endif
    if (schedule_phase==ZONE2_SCHEDULE_FAILED) {
        fail(ZONE2_FAULT_PATH);
        return;
    }
    if (g_zone2_phase==ZONE2_SCAN && schedule_phase!=ZONE2_SCHEDULE_SCAN) {
        RadarObserverStatus radar_status;
        if (!s_scan_beep_started) {
            RadarObserver_GetStatus(&radar_status);
            g_zone2_scan_obstacle_count=radar_status.obstacle_count;
            Buzzer_Start(g_zone2_scan_obstacle_count);
            s_scan_beep_started=1U;
        }
        if (!StartupAxes_Holding() || Buzzer_IsBusy()) return;
        if (!ChassisAuto_Zone2Prime(ZONE2_RUN_ID)) {
            fail(ZONE2_FAULT_BACKEND);
            return;
        }
        s_primed=1U;
        AppOdometry_RequestMapRun(ZONE2_RUN_ID,s_zone_locked);
#if APP_ZONE2_NAV_MOTION_ENABLE && !APP_ZONE2_MISSION_ENABLE
        s_prepare_ms=now;
#endif
        g_zone2_phase=ZONE2_PREPARE;
        return;
    }
    if (g_zone2_phase==ZONE2_PREPARE && observed && schedule_phase==ZONE2_SCHEDULE_READY &&
        g_can_guard_ready==1U) {
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
#if APP_ZONE2_MISSION_ENABLE
        g_zone2_mission_trace.navigation[s_schedule.leg].start_ms=now;
#endif
        g_zone2_phase=ZONE2_ZERO;
        return;
    }
#if APP_ZONE2_NAV_MOTION_ENABLE && !APP_ZONE2_MISSION_ENABLE
    if (g_zone2_phase==ZONE2_PREPARE && now-s_prepare_ms>=ZONE2_WAIT_READY_MS) {
        fail(ZONE2_FAULT_DEADLINE);
        return;
    }
#endif
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
        float endpoint_error=0.0f;
        uint32_t endpoint_ready=0U,station_ready=0U;
#if !APP_ZONE2_MISSION_ENABLE
        uint32_t stalled=0U;
#endif
        if (!observed) {
            fail(ZONE2_FAULT_SOURCE);
            return;
        }
#if APP_ZONE2_MISSION_ENABLE
        if (s_mission.search_active && scan_has_code()) {
            if (!s_mission.search_interrupted) {
                s_mission.search_interrupted=1U;
                s_stop_ms=now;
            }
            if (can.ack_generation && can.ack_mask==0x0FU && can.tx_complete_mask==0x0FU &&
                can.zero_confirmed && quiet_since(&pose,&heading,can.zero_ack_ms)) {
                if (!ChassisAuto_RequestStop()) {
                    fail(ZONE2_FAULT_BACKEND);
                    return;
                }
                s_station_stop=1U;
                s_stop_ms=now;
                g_zone2_phase=ZONE2_STOP;
            } else if (now-s_stop_ms>=CHASSIS_AUTO_STOP_MS+CHASSIS_AUTO_SETTLE_MS) {
                fail(ZONE2_FAULT_SOURCE);
            } else if (!submit_zero(&can)) {
                fail(ZONE2_FAULT_BACKEND);
            }
            return;
        }
#endif
        if (g_zone2_phase!=ZONE2_ZERO) {
            int32_t duration=(int32_t)s_trajectory.sections[s_section].duration_ms;
            if ((int32_t)(pose.sample_ms-s_section_ms)>=duration) {
                FusionMotionVector endpoint;
                if (!Zone2MotionReference_Sample(&s_trajectory,s_section,duration,&endpoint) ||
                    !section_error(&pose,&endpoint,&endpoint_error)) {
                    fail(ZONE2_FAULT_REFERENCE);
                    return;
                }
                endpoint_ready=1U;
                station_ready=g_zone2_phase==ZONE2_SECTION_ZERO &&
                    s_trajectory.sections[s_section].terminal && endpoint_error<=1.0f;
#if !APP_ZONE2_MISSION_ENABLE
                /* Inside the endpoint band, wait at zero for standstill evidence. */
                if (endpoint_error<=1.0f) s_section_progress_started=0U;
                else stalled=section_stalled(&pose,endpoint_error);
            } else if (s_section_progress_started) {
                stalled=pose.now_ms-s_section_progress_ms>=ZONE2_SECTION_PROGRESS_MS;
#endif
            }
        }
        /* A station STOP has its own measured standstill check after the STOP ACK. */
        if ((g_zone2_phase==ZONE2_ZERO || endpoint_ready) && g_zone2_phase!=ZONE2_TRACK &&
             can.ack_generation && can.ack_mask==0x0FU && can.tx_complete_mask==0x0FU &&
             (station_ready || (can.zero_confirmed && quiet_since(&pose,&heading,can.zero_ack_ms)))) {
            if (g_zone2_phase==ZONE2_SECTION_ZERO) {
                if (can.ack_generation!=s_generation) {
                    fail(ZONE2_FAULT_BACKEND);
                    return;
                }
                if (endpoint_error>1.0f) {
#if !APP_ZONE2_MISSION_ENABLE
                    if (stalled) {
                        fail(ZONE2_FAULT_DEADLINE);
                        return;
                    }
#endif
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
#if !APP_ZONE2_MISSION_ENABLE
            stalled=0U;
#endif
        }
#if !APP_ZONE2_MISSION_ENABLE
        if (stalled) {
            fail(ZONE2_FAULT_DEADLINE);
        } else
#endif
        if (g_zone2_phase==ZONE2_TRACK) {
            motion_batch(&can,&pose);
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
        if (can.state==CHASSIS_AUTO_DONE && can.result==1U &&
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
        } else if (now-s_stop_ms>=CHASSIS_AUTO_STOP_MS+CHASSIS_AUTO_SETTLE_MS) {
            fail(ZONE2_FAULT_DEADLINE);
        }
    }
}

#if APP_ZONE2_POSE_TRACE_ENABLE
void AppZone2_Step(void)
{
    uint32_t now,terminal;

    s_trace_pose.source_ms=0U;
    s_trace_pose.valid=0U;
    zone2_step();
    if (g_zone2_pose_trace.state==ZONE2_TRACE_CAPTURE) {
        terminal=g_zone2_phase==ZONE2_DONE || g_zone2_phase==ZONE2_FAILED;
#if APP_ZONE2_MISSION_ENABLE
        terminal|=g_zone2_phase==ZONE2_MISSION_DONE;
#endif
        now=now_ms();
        trace_capture(now,terminal);
        if (terminal) Zone2NavSchedule_TraceFreeze(&s_schedule,now,(uint32_t)g_zone2_fault,
            (uint32_t)g_zone2_path_error,(uint8_t)g_zone2_phase);
    }
}
#endif
#endif
