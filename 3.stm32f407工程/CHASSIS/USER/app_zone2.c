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
#include "heading_controller.h"
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
#include "region_dispatch.h"
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
#define STATION_CORRECTION_MAX_RPM 7U
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
    .acceleration_hint_mm_s2=2500.0f, .cross_gain_s=1.0f,
    .lateral_cap_mm_s=40.0f, .preview_s=0.05f,
    .wheel_limit_rpm=640.0f, .heading_reserve_rpm=20.0f,
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
static HeadingHoldController s_zone2_heading_controller;
#if !APP_ZONE2_MISSION_ENABLE
static uint32_t s_prepare_ms;
static uint32_t s_section_progress_started, s_section_progress_ms, s_section_sample_ms;
static float s_section_checkpoint;
#endif
volatile uint32_t g_zone2_completed_legs;
volatile uint32_t g_station_turn_started_ms,g_station_prepare_publish_ms;
volatile float g_station_prepare_publish_remaining_mm;
#else
static FusionState s_reference;
#endif
static uint32_t s_begin_ms, s_session, s_power_epoch, s_primed, s_zone_locked;
static uint32_t s_startup_axes_started;
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
       !g_radar_zone_ready || zone<1U || zone>2U)return;
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
    MissionInterval navigation[ZONE2_NAV_LEGS],station[ZONE2_STATION_COUNT];
    uint32_t receipt_ms[ZONE2_STATION_COUNT][12];
    uint32_t picks,places,unverified_storage_actions;
    uint8_t storage_action_state[ZONE2_STATION_COUNT][3U];
} g_zone2_mission_trace;

static struct {
    char task_code[16];
    uint32_t display_pending;
    uint32_t station_result, station_result_ms, page1_pending;
    AppZone2Station station;
    uint8_t receipt_count, slots[3], base_color[4], planned_base_color[4];
} s_mission;
volatile uint32_t g_zone2_task_display_result;
volatile uint32_t g_source_body_step_request,g_source_body_step_state;
volatile uint32_t g_source_body_step_reason,g_source_body_step_session,g_source_align_rejected;
volatile uint32_t g_source_body_step_elapsed_ms,g_source_body_step_submitted;
volatile uint32_t g_source_align_direction,g_source_align_command_seq,g_source_align_chain_active;
volatile float g_source_align_distance_mm,g_source_align_turn_deg;

typedef enum { STATION_BODY_IDLE,STATION_BODY_ZERO,STATION_BODY_TRACK,STATION_BODY_STOP,STATION_BODY_FINAL_ZERO } StationBodyPhase;
static struct {
    StationBodyPhase phase;
    FusionPoint anchor,target;
    float nominal_yaw,target_yaw;
    uint32_t restoring,started_ms,stop_ms,ready_ms,zero_generation;
    uint32_t stream,closing,observed_ms;
    int16_t last_raw[4];
    uint32_t turning; /* Actual rotation requests retain the full rotation-disk check. */
} s_station_body;
static struct {
 uint32_t ticket,vision_run,vision_pose,vision_sequence,observation_ms,translation_epoch;
 uint32_t anchor_sequence,request_ms;FusionPoint aligned;
} s_station_fixed;
static uint32_t s_station_translation_epoch;
static uint32_t s_station_incoming,s_nav_generation;
volatile uint32_t g_zone2_preparation_ms;
volatile float g_zone2_station_remaining_mm;
static int16_t s_nav_last_raw[4];
volatile uint32_t g_zone2_approach_ms,g_zone2_vision_handoff_ms,g_zone2_vision_handoff_session;
static uint32_t offer_station(uint32_t now,const FusionObservation *pose,uint32_t index,uint32_t early);
static uint32_t approach_service(const ChassisZone2CanStatus *can,const FusionObservation *pose,uint32_t now);
typedef struct {
    uint32_t pose,frame,observed_ms,request,direction,pending;
    float amount;
} StationVision;
static StationVision s_station_vision;
/* First failure in a station-body session. Validity is written last for SWD. */
enum { VISION_START_POSE=1, VISION_START_STOPPED, VISION_START_FRAME_AGE,
       VISION_START_BACKEND, VISION_START_ZERO_SUBMIT };
volatile struct {
    uint32_t reason,tick_ms,ticket,phase,session,observed,vision_ms,vision_age_ms;
    uint32_t vision_pose,vision_frame,vision_request,vision_pending,speed_batches;
    uint32_t can_state,can_result,can_cause,can_ready,can_feedback_ready;
    uint32_t ack_generation,attempt_generation,ack_mask,tx_mask;
    uint32_t zero_confirmed,zero_ack_ms,stop_ack_ms,next_batch_ms;
} g_zone2_vision_first_fault;

static void station_vision_fault(uint32_t reason,const ChassisZone2CanStatus *can,
                                 uint32_t observed,uint32_t now)
{
    taskENTER_CRITICAL();
    if (s_station_body.stream && !g_zone2_vision_first_fault.reason) {
        g_zone2_vision_first_fault.tick_ms=now;
        g_zone2_vision_first_fault.ticket=s_mission.station.ticket;
        g_zone2_vision_first_fault.phase=s_station_body.phase;
        g_zone2_vision_first_fault.session=s_session;
        g_zone2_vision_first_fault.observed=observed;
        g_zone2_vision_first_fault.vision_ms=s_station_body.observed_ms;
        g_zone2_vision_first_fault.vision_age_ms=now-s_station_body.observed_ms;
        g_zone2_vision_first_fault.vision_pose=s_station_vision.pose;
        g_zone2_vision_first_fault.vision_frame=s_station_vision.frame;
        g_zone2_vision_first_fault.vision_request=s_station_vision.request;
        g_zone2_vision_first_fault.vision_pending=s_station_vision.pending;
        g_zone2_vision_first_fault.speed_batches=g_source_body_step_submitted;
        g_zone2_vision_first_fault.can_state=can->state;
        g_zone2_vision_first_fault.can_result=can->result;
        g_zone2_vision_first_fault.can_cause=can->cause;
        g_zone2_vision_first_fault.can_ready=can->ready;
        g_zone2_vision_first_fault.can_feedback_ready=can->feedback_ready;
        g_zone2_vision_first_fault.ack_generation=can->ack_generation;
        g_zone2_vision_first_fault.attempt_generation=can->attempt_generation;
        g_zone2_vision_first_fault.ack_mask=can->ack_mask;
        g_zone2_vision_first_fault.tx_mask=can->tx_complete_mask;
        g_zone2_vision_first_fault.zero_confirmed=can->zero_confirmed;
        g_zone2_vision_first_fault.zero_ack_ms=can->zero_ack_ms;
        g_zone2_vision_first_fault.stop_ack_ms=can->stop_ack_ms;
        g_zone2_vision_first_fault.next_batch_ms=can->next_batch_ms;
        g_zone2_vision_first_fault.reason=reason;
    }
    taskEXIT_CRITICAL();
}

uint32_t SourceBodyStep_Stream(uint32_t ticket,uint32_t pose,uint32_t frame,
    uint32_t observed_ms,uint32_t request,uint32_t direction,float amount)
{
    uint32_t accepted=0U;
    taskENTER_CRITICAL();
    if (ticket && ticket==s_mission.station.ticket &&
        (g_zone2_phase==ZONE2_STATION_WAIT || AppZone2_StationApproaching()) &&
        !s_mission.station_result && pose && frame &&
        (!s_station_body.closing || s_station_body.phase==STATION_BODY_TRACK) &&
        (request==2U || request==3U || request==7U) &&
        (pose!=s_station_vision.pose || (int32_t)(frame-s_station_vision.frame)>0)) {
        const uint32_t idle_close=request==7U && s_station_body.phase==STATION_BODY_IDLE &&
            !s_station_incoming && (int32_t)(observed_ms-s_station_body.ready_ms)>0;
        s_station_vision=(StationVision){pose,frame,observed_ms,request,direction,!idle_close,amount};
        accepted=idle_close?2U:1U;
    }
    taskEXIT_CRITICAL();
    return accepted;
}

static StationVision station_vision_take(void)
{
    StationVision v;
    taskENTER_CRITICAL();
    v=s_station_vision;s_station_vision.pending=0U;
    taskEXIT_CRITICAL();
    return v;
}

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
extern TaskHandle_t g_zone2_display_task;

void AppZone2_OfferTaskCode(const char *code,uint32_t first_ms,uint32_t last_ms)
{
    uint32_t now=now_ms();
    uint32_t queued=0U;
    if (!FieldMap_TaskCodeValid(code) || now-last_ms>VEHICLE_STAGE1_SCAN_FRESH_MS ||
        (int32_t)(last_ms-first_ms)<0) return;
    taskENTER_CRITICAL();
    /* First valid receipt belongs to this boot; later packages cannot replace it. */
    if (!s_mission.task_code[0]) {
        memcpy(s_mission.task_code,code,sizeof(s_mission.task_code));
        s_mission.display_pending=1U;
        queued=1U;
    }
    taskEXIT_CRITICAL();
    if (queued && g_zone2_display_task) xTaskNotifyGive(g_zone2_display_task);
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
        s_mission.display_pending=0U;
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
    if ((g_zone2_phase==ZONE2_STATION_WAIT || AppZone2_StationApproaching()) &&
        !s_mission.station_result) {
        *station=s_mission.station;
        ready=1U;
    }
    taskEXIT_CRITICAL();
    return ready;
}

uint32_t AppZone2_StationApproaching(void)
{
    return s_station_incoming && s_schedule.phase==ZONE2_SCHEDULE_ACTIVE &&
        (g_zone2_phase==ZONE2_TRACK || g_zone2_phase==ZONE2_SECTION_ZERO ||
#if APP_ZONE2_NAV_MOTION_ENABLE
         /* 正常到站停车仍持有工位票据并继续机构准备；异常停车不延续。 */
         (g_zone2_phase==ZONE2_STOP && s_station_stop) ||
#endif
         g_zone2_phase==ZONE2_STATION_WAIT);
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

uint32_t AppZone2_StationVisualAligned(uint32_t ticket,uint32_t vision_run,
 uint32_t vision_pose,uint32_t vision_sequence,uint32_t observation_ms)
{
 AppMapPose map;uint32_t accepted=0U,now=now_ms();
 AppOdometry_ReadMap(&map);
 taskENTER_CRITICAL();
 if(g_zone2_phase==ZONE2_STATION_WAIT && ticket && ticket==s_mission.station.ticket &&
    s_mission.station.kind!=APP_ZONE2_SOURCE && !s_mission.station_result &&
    !s_station_incoming && vision_run && vision_pose && vision_sequence &&
    now-observation_ms<=500U && (int32_t)(observation_ms-s_station_body.ready_ms)>0 &&
    !SourceBodyStep_Busy() && !g_source_body_step_request &&
    map.run_id==s_schedule.run_id && map.start_id==s_schedule.start_id &&
    map.state==APP_MAP_TRACKING && map.based && map.pose.valid && !map.pose.local_origin &&
    !map.pose.continuity_lost && !map.pose.reasons && now-map.pose.sample_ms<=ZONE2_POSE_MS) {
  s_station_fixed.ticket=ticket;s_station_fixed.vision_run=vision_run;
  s_station_fixed.vision_pose=vision_pose;s_station_fixed.vision_sequence=vision_sequence;
  s_station_fixed.observation_ms=observation_ms;s_station_fixed.translation_epoch=s_station_translation_epoch;
  s_station_fixed.aligned=(FusionPoint){map.pose.x_mm,map.pose.y_mm};accepted=1U;
 }
 taskEXIT_CRITICAL();return accepted;
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
        g_can_test_result<=1U && !g_chassis_auto_cut_vm
#if APP_ZONE2_MISSION_ENABLE
        && !RegionDispatch_ParallelFaulted()
#endif
        ;
}

static void fail(Zone2Fault reason)
{
#if APP_ZONE2_MISSION_ENABLE
    taskENTER_CRITICAL();
    if (s_station_body.phase!=STATION_BODY_IDLE || g_source_body_step_request) {
        g_source_body_step_state=9U;
        g_source_body_step_reason=(uint32_t)reason;
        s_station_body.phase=STATION_BODY_IDLE;
        g_source_body_step_request=0U;
        g_source_align_chain_active=0U;
    }
    (void)RegionDispatch_StopActive();
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
    return Zone2NavSchedule_MapCurrent(&s_schedule);
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
        s_schedule.map.context.map_generation,VEHICLE_MAP_VERSION,0U,0U};
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
    s_correction=(FusionMotionVector){0};s_zone2_heading_controller=(HeadingHoldController){0};
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
    uint32_t ok=ChassisAuto_Zone2SubmitAt(ZONE2_RUN_ID,s_session,can->ack_generation+1U,
        can->next_batch_ms,s_zero);
#if APP_ZONE2_MISSION_ENABLE
    if(ok){memset(s_nav_last_raw,0,sizeof(s_nav_last_raw));s_nav_generation=can->ack_generation+1U;}
#endif
    return ok;
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
    if(fabsf(in.slot_yaw_delta[0])<1.0e-5f && fabsf(in.slot_yaw_delta[1])<1.0e-5f &&
       fabsf(in.slot_yaw_delta[2])<1.0e-5f && fabsf(in.slot_yaw_delta[3])<1.0e-5f) {
        float angle=in.reference.yaw; /* Entry prefix can hold a different heading. */
        float dx=(in.slot_to[0].x-in.slot_from[0].x)/0.08f;
        float dy=(in.slot_to[0].y-in.slot_from[0].y)/0.08f;
        float body_forward=dx*cosf(angle)+dy*sinf(angle);
        float body_lateral=-dx*sinf(angle)+dy*cosf(angle);
        float kp,kd,correction,error;
        if(fabsf(in.reference.yaw*(180.0f/3.14159265358979323846f)-pose->heading_deg)<=10.0f) {
            if(fabsf(body_forward)>1.0f && fabsf(body_lateral)>1.0f) {
                float longitudinal_kp=(body_forward>=0.0f)?APP_SPEED_TRIAL160_FORWARD_HEADING_KP_RPM_PER_DEG:APP_SPEED_TRIAL160_HEADING_KP_RPM_PER_DEG;
                float longitudinal_kd=(body_forward>=0.0f)?APP_SPEED_TRIAL160_FORWARD_HEADING_KD_RPM_PER_DPS:APP_SPEED_TRIAL160_HEADING_KD_RPM_PER_DPS;
                float lateral_kp=(body_lateral>=0.0f)?APP_SPEED_TRIAL160_LEFT_HEADING_KP_RPM_PER_DEG:APP_SPEED_TRIAL160_RIGHT_HEADING_KP_RPM_PER_DEG;
                float lateral_kd=(body_lateral>=0.0f)?APP_SPEED_TRIAL160_LEFT_HEADING_KD_RPM_PER_DPS:APP_SPEED_TRIAL160_RIGHT_HEADING_KD_RPM_PER_DPS;
                kp=fmaxf(longitudinal_kp,lateral_kp);kd=fmaxf(longitudinal_kd,lateral_kd);
            } else if(fabsf(body_forward)>=fabsf(body_lateral)) {
                kp=(body_forward>=0.0f)?APP_SPEED_TRIAL160_FORWARD_HEADING_KP_RPM_PER_DEG:APP_SPEED_TRIAL160_HEADING_KP_RPM_PER_DEG;
                kd=(body_forward>=0.0f)?APP_SPEED_TRIAL160_FORWARD_HEADING_KD_RPM_PER_DPS:APP_SPEED_TRIAL160_HEADING_KD_RPM_PER_DPS;
            } else {
                kp=(body_lateral>=0.0f)?APP_SPEED_TRIAL160_LEFT_HEADING_KP_RPM_PER_DEG:APP_SPEED_TRIAL160_RIGHT_HEADING_KP_RPM_PER_DEG;
                kd=(body_lateral>=0.0f)?APP_SPEED_TRIAL160_LEFT_HEADING_KD_RPM_PER_DPS:APP_SPEED_TRIAL160_RIGHT_HEADING_KD_RPM_PER_DPS;
            }
            if(HeadingController_HoldStepRoute(&s_zone2_heading_controller,
                in.reference.yaw*(180.0f/3.14159265358979323846f),pose->heading_deg,
                pose->omega_deg_s,pose->now_ms,kp,kd,(float)APP_SPEED_TRIAL160_HEADING_MAX_RPM,
                &correction,&error)==1U) {
                in.heading_correction_rpm=correction;
                in.heading_pd_enabled=1U;
            }
        }
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
#if APP_ZONE2_MISSION_ENABLE
    for(i=0U;i<4U;i++)s_nav_last_raw[i]=(int16_t)
        (out.commands[i].direction==g_chassis_forward_direction[i]?
         (int32_t)out.commands[i].speed_raw:-(int32_t)out.commands[i].speed_raw);
    s_nav_generation=can->ack_generation+1U;
#endif
    s_generation=can->ack_generation+1U;
    s_proposed=out.correction;
    g_zone2_reference_status=FUSION_FOLLOW;
    if (out.finish_requested) g_zone2_phase=ZONE2_SECTION_ZERO;
}

static void finish_navigation(void)
{
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
static void wait_at_scan(ChassisZone2CanStatus *can,const FusionObservation *pose,
    const AppHeadingSource *heading,uint32_t observed,uint32_t now)
{
    if (!scan_has_code()) return;
    if (!observed || !can->wheel_quiet || !quiet_since(pose,heading,can->stop_ack_ms)) {
        fail(ZONE2_FAULT_SOURCE);
        return;
    }
    if (!ChassisAuto_Zone2ReleaseCompleted(ZONE2_RUN_ID,can->session)) {
        fail(ZONE2_FAULT_BACKEND);
        return;
    }
    g_zone2_mission_trace.scan_end_ms=now;
    g_zone2_phase=ZONE2_PREPARE;
}

/* Exact rectangular support over the current-to-held heading interval. */
static float station_translation_support(float lo,float hi,float a,float b)
{
    float peak=fmaxf(a*fabsf(cosf(lo))+b*fabsf(sinf(lo)),
                     a*fabsf(cosf(hi))+b*fabsf(sinf(hi)));
    const float angle=atan2f(b,a);
    int32_t k;
    for(k=-2;k<=2;k++) {
        const float base=(float)k*180.0f*ZONE2_DEG_TO_RAD;
        if((lo<=base+angle && hi>=base+angle) ||
           (lo<=base-angle && hi>=base-angle)) return hypotf(a,b);
    }
    return peak;
}
static uint32_t station_step_clear(FusionPoint from,FusionPoint target,
    float from_yaw,float target_yaw,uint32_t turning)
{
    static const FusionBox yellow[]={{550,550,1000,1000},{550,1400,1000,1850},
        {1400,550,1850,1000},{1400,1400,1850,1850}};
    const float radius=0.5f*hypotf(VEHICLE_BODY_LENGTH_MM,VEHICLE_BODY_WIDTH_MM)+STATION_SWEEP_RESERVE_MM;
    float ex=radius,ey=radius;
    FusionBox hull;
    uint32_t i;
    if(!turning) {
        const float h=remainderf(from_yaw,360.0f);
        const float end=h+remainderf(target_yaw-from_yaw,360.0f);
        const float lo=(fminf(h,end)-s_config.heading_budget_deg)*ZONE2_DEG_TO_RAD;
        const float hi=(fmaxf(h,end)+s_config.heading_budget_deg)*ZONE2_DEG_TO_RAD;
        const float reserve=STATION_SWEEP_RESERVE_MM+s_config.position_budget_mm;
        const float l=VEHICLE_BODY_LENGTH_MM*0.5f,w=VEHICLE_BODY_WIDTH_MM*0.5f;
        ex=station_translation_support(lo,hi,l,w)+reserve;
        ey=station_translation_support(lo,hi,w,l)+reserve;
    }
    hull=(FusionBox){fminf(from.x_mm,target.x_mm)-ex,fminf(from.y_mm,target.y_mm)-ey,
        fmaxf(from.x_mm,target.x_mm)+ex,fmaxf(from.y_mm,target.y_mm)+ey};
    if (!(hull.min_x>0.0f && hull.min_y>0.0f &&
          hull.max_x<FIELD_SIZE_MM && hull.max_y<FIELD_SIZE_MM)) return 0U;
    for (i=0U;i<sizeof(yellow)/sizeof(yellow[0]);i++) {
        if (hull.min_x<=yellow[i].max_x && hull.max_x>=yellow[i].min_x &&
            hull.min_y<=yellow[i].max_y && hull.max_y>=yellow[i].min_y) return 0U;
    }
    return 1U;
}

/* Once a terminal rotation is reached, choose a nearby real turn center
 * instead of correcting position back to a millimetre-perfect waypoint.
 * This changes the remaining reference only after checking the same full
 * rotation envelope at the current measured pose. */
static uint32_t station_nearby_turn(const FusionObservation *pose)
{
    Zone2MotionSection *next;
    Zone2MotionSpan *turn;
    FusionMotionVector end;
    if(s_schedule.leg<1U || s_schedule.leg>6U || s_section+1U>=s_trajectory.section_count)return 0U;
    next=&s_trajectory.sections[s_section+1U];
    if(!next->terminal || next->span_count!=1U)return 0U;
    turn=&s_trajectory.spans[next->first_span];
    if(turn->kind!=2U || !Zone2MotionReference_Sample(&s_trajectory,s_section,
       (int32_t)s_trajectory.sections[s_section].duration_ms,&end))return 0U;
    if(hypotf(pose->position.x_mm-end.x,pose->position.y_mm-end.y)>STATION_STEP_MM ||
       fabsf(remainderf(pose->heading_deg-end.yaw/ZONE2_DEG_TO_RAD,360.0f))>s_config.heading_budget_deg ||
       !station_step_clear(pose->position,pose->position,pose->heading_deg,
           pose->heading_deg+turn->data.turn.delta_rad/ZONE2_DEG_TO_RAD,1U))return 0U;
    turn->data.turn.center=pose->position;
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
    s_station_body.turning=(request==3U);
    /* Alignment is a chain of explicit bounded requests, each from its fresh measured pose. */
    if (!(hypotf(s_station_body.target.x_mm-pose->position.x_mm,
                 s_station_body.target.y_mm-pose->position.y_mm)<=STATION_STEP_MM+0.01f) ||
        !(fabsf(remainderf(s_station_body.target_yaw-pose->heading_deg,360.0f))<=STATION_STEP_DEG+0.5f))
        return 0U;
    return station_step_clear(pose->position,s_station_body.target,
            pose->heading_deg,s_station_body.target_yaw,s_station_body.turning);
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
    s_station_body.stream=s_station_body.closing=0U;
    memset(s_station_body.last_raw,0,sizeof(s_station_body.last_raw));
    g_source_body_step_state=1U;
    g_source_body_step_session=s_session;
    g_source_body_step_submitted=0U;
    g_source_align_chain_active=!restoring;
    return 1U;
}

static uint32_t station_step_commands(const FusionObservation *pose,ChassisMotorCommand commands[4],
    uint32_t *at_target,uint16_t limit,uint16_t accel)
{
    const float dx=s_station_body.target.x_mm-pose->position.x_mm;
    const float dy=s_station_body.target.y_mm-pose->position.y_mm;
    const float distance=hypotf(dx,dy);
    const float angle=remainderf(s_station_body.target_yaw-pose->heading_deg,360.0f);
    const float heading=pose->heading_deg*ZONE2_DEG_TO_RAD;
    const float c=cosf(heading),s=sinf(heading);
    const float mm_s_per_rpm=g_chassis_geometry.wheel_diameter_mm*(180.0f*ZONE2_DEG_TO_RAD/60.0f);
    const float brake=accel?sqrtf(2.0f*(float)accel*mm_s_per_rpm*distance):
        (float)limit*mm_s_per_rpm;
    const float gain=distance>STATION_STOP_MM?
        fminf(2.0f,fminf((float)limit*mm_s_per_rpm,brake)/distance):0.0f;
    ChassisBodyVelocity body={gain*(dx*c+dy*s),gain*(dy*c-dx*s),
        fabsf(angle)>STATION_STOP_DEG?fmaxf(-10.0f,fminf(10.0f,2.0f*angle))*ZONE2_DEG_TO_RAD:0.0f};
    ChassisWheelSpeeds wheels;
    if (!(distance<=STATION_STEP_MM+STATION_SWEEP_RESERVE_MM && fabsf(angle)<=2.0f*STATION_STEP_DEG) ||
        !station_step_clear(pose->position,s_station_body.target,
            pose->heading_deg,s_station_body.target_yaw,s_station_body.turning)) return 0U;
    *at_target=distance<=STATION_STOP_MM && fabsf(angle)<=STATION_STOP_DEG;
    return Chassis_Inverse(&g_chassis_geometry,&body,(float)limit,&wheels,0) &&
        Chassis_EncodeEmm(&wheels,limit,commands);
}

static void station_body_service(const ChassisZone2CanStatus *can,const FusionObservation *pose,
    const AppHeadingSource *heading,uint32_t observed,uint32_t now)
{
    ChassisMotorCommand commands[4];
    uint32_t request=g_source_body_step_request,at_target;
    uint32_t i;
    float amount;
    StationVision vision;
    const uint32_t source=s_mission.station.kind==APP_ZONE2_SOURCE;
    const uint16_t limit=source?110U:90U,accel=source?70U:50U;
    const int32_t delta=source?6:4; /* 80 ms scheduled four-wheel batch, 0.1 RPM units. */

    if (request==9U) {
        fail(ZONE2_FAULT_STATION);
        return;
    }
    if (s_station_body.phase==STATION_BODY_IDLE) {
        if (s_mission.station_result) return;
        if (!request) {
            if (!observed || can->state!=CHASSIS_AUTO_IDLE ||
                !quiet_since(pose,heading,s_station_body.ready_ms)) return;
            for (i=0U;i<4U;i++) {
                if (can->wheel_decirpm[i] || !newer(can->wheel_ms[i],s_station_body.ready_ms)) return;
            }
            vision=station_vision_take();
            if (!vision.pending || vision.request==7U || now_ms()-vision.observed_ms>500U ||
                (int32_t)(vision.observed_ms-s_station_body.ready_ms)<=0) return;
            if (!station_step_target(pose,vision.request,vision.direction,vision.amount) ||
                !station_step_begin(can,0U,now)) {
                g_source_align_rejected++;fail(ZONE2_FAULT_STATION);return;
            }
            s_station_body.stream=1U;
            s_station_body.observed_ms=vision.observed_ms;
            if (vision.request==2U && ++s_station_translation_epoch==0U) s_station_translation_epoch=1U;
            return;
        }
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
        if(request==2U){if(++s_station_translation_epoch==0U)s_station_translation_epoch=1U;}
        g_source_body_step_request=0U;
        return;
    }
    g_source_body_step_elapsed_ms=now-s_station_body.started_ms;
    if (can->run_id!=ZONE2_RUN_ID || can->session!=s_session ||
        can->power_epoch!=s_power_epoch || can->cause ||
        can->state==CHASSIS_AUTO_FAULT || can->state==CHASSIS_AUTO_CANCELLED) {
        station_vision_fault(VISION_START_BACKEND,can,observed,now_ms());
        fail(ZONE2_FAULT_BACKEND);
        return;
    }
    if (s_station_body.phase==STATION_BODY_STOP) {
        if (can->state==CHASSIS_AUTO_DONE && can->result==1U &&
            can->wheel_quiet && observed && quiet_since(pose,heading,can->stop_ack_ms)) {
            if ((!s_station_body.stream &&
                 (hypotf(pose->position.x_mm-s_station_body.target.x_mm,
                         pose->position.y_mm-s_station_body.target.y_mm)>1.0f ||
                  fabsf(remainderf(pose->heading_deg-s_station_body.target_yaw,360.0f))>0.5f)) ||
                !ChassisAuto_Zone2ReleaseCompleted(ZONE2_RUN_ID,can->session)) {
                fail(ZONE2_FAULT_STATION);
                return;
            }
            if(s_station_incoming) {
                FusionObservation arrived=*pose;arrived.stopped=1U;
                if(!Zone2NavSchedule_Arrive(&s_schedule,s_session,true,&arrived,now)) {
                    fail(ZONE2_FAULT_PATH);return;
                }
                g_zone2_completed_legs=s_schedule.leg;
                g_zone2_stop_pose_ms=pose->sample_ms;
                g_zone2_mission_trace.navigation[s_schedule.leg-1U].end_ms=now;
                s_station_incoming=0U;
                RegionDispatch_StationStopped(s_mission.station.ticket);
            }
            taskENTER_CRITICAL();
            if (s_station_body.stream) {
                s_station_vision.pending=0U;
                g_source_align_chain_active=0U;
            }
            s_station_body.closing=0U;
            s_station_body.ready_ms=now;
            s_station_body.phase=STATION_BODY_IDLE;
            g_source_body_step_state=3U;
            if (!s_station_body.restoring) g_source_align_command_seq++;
            taskEXIT_CRITICAL();
        } /* Otherwise keep waiting for actual stop evidence. */
        return;
    }
    if (!observed || can->state==CHASSIS_AUTO_STOP ||
        can->state==CHASSIS_AUTO_SETTLE || can->state==CHASSIS_AUTO_DONE) {
        station_vision_fault(observed?VISION_START_STOPPED:VISION_START_POSE,
                             can,observed,now_ms());
        fail(ZONE2_FAULT_SOURCE);
        return;
    }
    if (!can->ready) return;
    if (s_station_body.phase==STATION_BODY_FINAL_ZERO) {
        if (can->ack_generation==s_station_body.zero_generation &&
            can->ack_mask==0x0FU && can->tx_complete_mask==0x0FU &&
            can->zero_confirmed && quiet_since(pose,heading,can->zero_ack_ms)) {
            if (!ChassisAuto_RequestStop()) {
                fail(ZONE2_FAULT_BACKEND);
                return;
            }
            s_station_body.stop_ms=now;
            s_station_body.phase=STATION_BODY_STOP;
        } else if (!submit_zero(can)) {
            fail(ZONE2_FAULT_BACKEND);
        } else {
            /* Keep the existing batch heartbeat while genuine quiet feedback arrives. */
            s_station_body.zero_generation=can->ack_generation+1U;
        }
        return;
    }
    if (s_station_body.phase==STATION_BODY_ZERO) {
        /* 新会话按底层协议先发一次零速首包，ACK后即可纠偏，不等待停稳。
         * 行驶中视觉接入沿用导航会话，不另发零速首包。 */
        if (!can->ack_generation) {
            if (!can->attempt_generation && !submit_zero(can)) {
                station_vision_fault(VISION_START_ZERO_SUBMIT,can,observed,now_ms());
                fail(ZONE2_FAULT_BACKEND);
            }
            return;
        }
        if(can->ack_mask!=0x0FU || can->tx_complete_mask!=0x0FU)return;
        s_station_body.phase=STATION_BODY_TRACK;
    }
    if (s_station_body.stream) {
        { /* Keep accepting fresh corrections while ramping toward zero. */
            vision=station_vision_take();
            if (vision.pending && now_ms()-vision.observed_ms<=500U) {
                s_station_body.observed_ms=vision.observed_ms;
                if (vision.request==7U) s_station_body.closing=1U;
                else {
                    if (!station_step_target(pose,vision.request,vision.direction,vision.amount)) {
                        fail(ZONE2_FAULT_REFERENCE);return;
                    }
                    s_station_body.closing=0U;
                    if (vision.request==2U && ++s_station_translation_epoch==0U)
                        s_station_translation_epoch=1U;
                }
            }
            if (!s_station_body.closing && now_ms()-s_station_body.observed_ms>500U) {
                station_vision_fault(VISION_START_FRAME_AGE,can,observed,now_ms());
                fail(ZONE2_FAULT_SOURCE);return;
            }
        }
        if (!s_station_body.closing && !station_step_commands(pose,commands,&at_target,limit,accel)) {
            fail(ZONE2_FAULT_REFERENCE);return;
        }
        at_target=1U;
        for (i=0U;i<4U;i++) {
            int32_t previous=s_station_body.last_raw[i],target=0,change;
            if (!s_station_body.closing) {
                target=(int32_t)commands[i].speed_raw;
                if (commands[i].direction!=g_chassis_forward_direction[i]) target=-target;
            }
            if ((target<0 && previous>0) || (target>0 && previous<0)) target=0;
            change=target-previous;
            if (change>delta) change=delta;
            if (change< -delta) change= -delta;
            target=previous+change;
            commands[i].address=(uint8_t)(i+1U);
            commands[i].direction=(uint8_t)(g_chassis_forward_direction[i]^(target<0));
            commands[i].speed_raw=(uint16_t)(target<0?-target:target);
            s_station_body.last_raw[i]=(int16_t)target;
            if (target) at_target=0U;
        }
        if (!ChassisAuto_Zone2SubmitAt(ZONE2_RUN_ID,s_session,can->ack_generation+1U,
                                      can->next_batch_ms,commands)) fail(ZONE2_FAULT_BACKEND);
        else if (s_station_body.closing && at_target) {
            s_station_body.zero_generation=can->ack_generation+1U;
            s_station_body.phase=STATION_BODY_FINAL_ZERO;
        } else g_source_body_step_submitted++;
        return;
    }
    if (!station_step_commands(pose,commands,&at_target,STATION_CORRECTION_MAX_RPM,0U)) {
        fail(ZONE2_FAULT_REFERENCE);
    } else if (at_target) {
        /* Formal STOP requires a fully acknowledged zero-speed batch first. */
        if (!submit_zero(can)) {
            fail(ZONE2_FAULT_BACKEND);
            return;
        }
        s_station_body.zero_generation=can->ack_generation+1U;
        s_station_body.phase=STATION_BODY_FINAL_ZERO;
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
    if(s_station_fixed.anchor_sequence) {
        AppMapStationAnchor a;AppOdometry_ReadStationAnchor(&a);
        if(a.sequence!=s_station_fixed.anchor_sequence || a.ticket!=s_mission.station.ticket ||
           a.run_id!=s_schedule.run_id || a.start_id!=s_schedule.start_id ||
           a.power_epoch!=can->power_epoch || a.map_version!=VEHICLE_MAP_VERSION ||
           a.cell!=s_mission.station.cell || a.state==4U || now-s_station_fixed.request_ms>=2500U) {
            fail(ZONE2_FAULT_SOURCE);return;
        }
        if(a.state!=3U || !observed || !quiet_since(pose,heading,a.applied_ms))return;
    }
    if (newer(s_station_body.ready_ms,stamp)) stamp=s_station_body.ready_ms;
    if (!observed || !quiet_since(pose,heading,stamp) || !can->feedback_ready ||
        can->state!=CHASSIS_AUTO_IDLE) {
        /* No total wait deadline; fresh standstill evidence is still required. */
        return;
    }
    for (i=0U;i<4U;i++) {
        if (can->wheel_decirpm[i] || !newer(can->wheel_ms[i],stamp)) {
            /* No total wait deadline; fresh standstill evidence is still required. */
            return;
        }
    }
    heading_error=remainderf(s_station_body.nominal_yaw-pose->heading_deg,360.0f);
    if (fabsf(heading_error)>s_config.heading_budget_deg) {
        float turn=fmaxf(-STATION_STEP_DEG,fminf(STATION_STEP_DEG,heading_error));
        if (!station_step_target(pose,3U,0U,turn) || !station_step_begin(can,1U,now))
            fail(ZONE2_FAULT_STATION);
        return;
    }
    if(s_mission.station.kind!=APP_ZONE2_SOURCE && !s_station_fixed.anchor_sequence) {
        uint32_t sequence;
        if(s_station_fixed.ticket!=s_mission.station.ticket || !s_station_fixed.vision_run ||
           !s_station_fixed.vision_pose || !s_station_fixed.vision_sequence ||
           s_station_fixed.translation_epoch!=s_station_translation_epoch ||
           hypotf(pose->position.x_mm-s_station_fixed.aligned.x_mm,
                  pose->position.y_mm-s_station_fixed.aligned.y_mm)>s_config.position_tolerance_mm) {
            fail(ZONE2_FAULT_STATION);return;
        }
        sequence=AppOdometry_RequestStationAnchor(s_schedule.run_id,s_schedule.start_id,
            VEHICLE_MAP_VERSION,can->power_epoch,s_mission.station.ticket,s_mission.station.cell);
        if(!sequence){fail(ZONE2_FAULT_SOURCE);return;}
        s_station_fixed.anchor_sequence=sequence;s_station_fixed.request_ms=now;return;
    }
    g_zone2_mission_trace.station[s_mission.station.leg-2U].end_ms=now;
    g_zone2_phase=ZONE2_PREPARE;
}

/* v1.9 transfers directly at a fresh fixed-view observation; existing
 * wheel slew limits and final real STOP/quiet confirmation remain. */

/* 剩余距离达到集中配置值时仅发布一次工位票据，此处不放行抓取。 */
static uint32_t approach_service(const ChassisZone2CanStatus *can,const FusionObservation *pose,uint32_t now)
{
    FusionMotionVector endpoint;
    StationVision v;
    uint32_t i,stamp=0U,last_turn=0xffffffffU;
    float remaining;
    const uint32_t last=s_trajectory.section_count?s_trajectory.section_count-1U:0U;
    const int32_t cap=s_schedule.leg==1U || s_schedule.leg==4U?110:90;
    if(s_schedule.leg<1U || s_schedule.leg>6U || !s_trajectory.section_count)return 0U;
    if(!Zone2MotionReference_Sample(&s_trajectory,last,
         (int32_t)s_trajectory.sections[last].duration_ms,&endpoint))return 0U;
    remaining=hypotf(pose->position.x_mm-endpoint.x,pose->position.y_mm-endpoint.y);
    g_zone2_station_remaining_mm=remaining;
    for(i=0U;i<s_trajectory.section_count;i++)
        if(s_trajectory.spans[s_trajectory.sections[i].first_span].kind==2U)last_turn=i;
    if(last_turn!=0xffffffffU && s_section==last_turn && !g_station_turn_started_ms) {
        const Zone2MotionSpan *turn=&s_trajectory.spans[s_trajectory.sections[last_turn].first_span];
        if(fabsf(remainderf(pose->heading_deg-turn->data.turn.start_rad/ZONE2_DEG_TO_RAD,360.0f))>0.1f)
            g_station_turn_started_ms=now;
    }
    if(!s_station_incoming) {
        /* 到导航终点剩余30cm时准备；按真实反馈接入视觉，不以定时替代到位。 */
        if(remaining>((s_schedule.leg==1U || s_schedule.leg==4U)?
            VEHICLE_SOURCE_PREPARE_DISTANCE_MM:VEHICLE_STATION_PREPARE_DISTANCE_MM))return 0U;
        if(last_turn!=0xffffffffU && (s_section<last_turn ||
           (s_section==last_turn && !g_station_turn_started_ms)))return 0U;
        if(!offer_station(now,pose,s_schedule.leg,1U)){fail(ZONE2_FAULT_STATION);return 1U;}
        g_zone2_preparation_ms=g_zone2_approach_ms=g_station_prepare_publish_ms=now;
        g_station_prepare_publish_remaining_mm=remaining;
    }
    if(last_turn!=0xffffffffU && s_section<=last_turn)return 0U;
    if(!RegionDispatch_ObservationReady(s_mission.station.ticket,&stamp) ||
       fabsf(remainderf(pose->heading_deg-s_station_body.nominal_yaw,360.0f))>s_config.heading_budget_deg ||
       !s_nav_generation || can->ack_generation!=s_nav_generation ||
       can->ack_mask!=0x0FU || can->tx_complete_mask!=0x0FU)return 0U;
    for(i=0U;i<4U;i++)if(s_nav_last_raw[i]>cap || s_nav_last_raw[i]< -cap ||
        can->wheel_decirpm[i]>cap || can->wheel_decirpm[i]< -cap)return 0U;
    taskENTER_CRITICAL();v=s_station_vision;taskEXIT_CRITICAL();
    if(!v.pending || now_ms()-v.observed_ms>500U || (int32_t)(v.observed_ms-stamp)<=0)return 0U;
    v=station_vision_take();
    if(v.request!=7U && !station_step_target(pose,v.request,v.direction,v.amount)) {
        fail(ZONE2_FAULT_REFERENCE);return 1U;
    }
    s_station_body.started_ms=now;s_station_body.phase=STATION_BODY_TRACK;
    s_station_body.restoring=0U;s_station_body.stream=1U;s_station_body.closing=v.request==7U;
    s_station_body.observed_ms=v.observed_ms;
    memcpy(s_station_body.last_raw,s_nav_last_raw,sizeof(s_nav_last_raw));
    if(v.request==2U && ++s_station_translation_epoch==0U)s_station_translation_epoch=1U;
    g_source_body_step_state=1U;g_source_body_step_session=s_session;
    g_source_body_step_submitted=0U;g_source_align_chain_active=1U;
    g_zone2_vision_handoff_ms=now;g_zone2_vision_handoff_session=s_session;
    g_zone2_phase=ZONE2_STATION_WAIT;
    return 1U;
}

static uint32_t offer_station(uint32_t now,const FusionObservation *pose,uint32_t index,uint32_t early)
{
    static const uint8_t kinds[6]={APP_ZONE2_SOURCE,APP_ZONE2_ROUGH,APP_ZONE2_STORAGE_BASE,
        APP_ZONE2_SOURCE,APP_ZONE2_ROUGH,APP_ZONE2_STORAGE_STACK};
    AppZone2Station station={0};
    Zone2NavLeg leg;
    uint8_t i,offset;

    if (index<1U || index>6U ||
        !Zone2NavPath_Leg(s_schedule.start_id,index,&leg) || !scan_has_code()) return 0U;
    station.leg=(uint8_t)(index+1U);
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
    memset(&s_station_vision,0,sizeof(s_station_vision));
    memset(&s_station_fixed,0,sizeof(s_station_fixed));
    s_mission.receipt_count=0U;
    s_mission.station_result=0U;
    s_station_body.anchor=pose->position;
    s_station_body.nominal_yaw=leg.goal_heading_deg;
    s_station_body.ready_ms=pose->sample_ms;
    g_zone2_mission_trace.station[station.leg-2U].start_ms=now;
    s_station_incoming=early;
    if(!early)g_zone2_phase=ZONE2_STATION_WAIT;
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
        (!(hypotf(pose->position.x_mm-endpoint.x,pose->position.y_mm-endpoint.y)<=
        s_config.position_tolerance_mm) ||
        !(fabsf(remainderf(pose->heading_deg-endpoint.yaw/ZONE2_DEG_TO_RAD,360.0f))<=
        s_config.heading_budget_deg))) {
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
#if APP_ZONE2_MISSION_ENABLE
    g_zone2_mission_trace.navigation[s_schedule.leg-1U].end_ms=now;
    if (s_schedule.leg==1U) {
        taskENTER_CRITICAL();
        g_zone2_mission_trace.scan_start_ms=now;
        g_zone2_phase=ZONE2_QR_WAIT;
        taskEXIT_CRITICAL();
    } else if (s_schedule.phase==ZONE2_SCHEDULE_DONE) {
        finish_navigation();
    } else if (!ChassisAuto_Zone2ReleaseCompleted(ZONE2_RUN_ID,can->session)) {
        fail(ZONE2_FAULT_BACKEND);
    } else if(s_station_incoming && s_mission.station.leg==s_schedule.leg) {
        s_station_incoming=0U;s_station_body.ready_ms=now;
        memset(&s_station_vision,0,sizeof(s_station_vision));
        g_zone2_phase=ZONE2_STATION_WAIT;
        RegionDispatch_StationStopped(s_mission.station.ticket);
    } else if (!offer_station(now,pose,s_schedule.leg-1U,0U)) {
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
    if (g_boot_app_init_done==1U && g_zone2_phase==ZONE2_IDLE && !s_start_request) {
        uint32_t zone=RadarZoneSwitch_Lock();
        if (zone==1U || zone==2U) {
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
        if (s_startup_axes_started && g_startup_axes_state>=5U) {
            fail(ZONE2_FAULT_HEALTH); return;
        }
        if (g_can_test_result==1U && g_safety_ready && s_zone_locked &&
            !g_rtos_fault && !g_safety_fault && !g_can_ctrl_fault_latched) {
            /* Prime opens real wheel feedback while inactive; motion begins only after homing. */
            if (!s_primed) {
                if (!ChassisAuto_Zone2Prime(ZONE2_RUN_ID)) {
                    fail(ZONE2_FAULT_BACKEND);
                    return;
                }
                s_primed=1U;
            }
            if (!s_startup_axes_started) {
                if (!StartupAxes_Begin()) { fail(ZONE2_FAULT_HEALTH); return; }
                s_startup_axes_started=1U;
            }
            if (!StartupAxes_Holding()) return;
#if APP_ZONE2_MISSION_ENABLE
            {
                uint32_t ready=RegionDispatch_StartupReferencesReady();
                if(ready==2U){fail(ZONE2_FAULT_HEALTH);return;}
                if(ready!=1U)return;
            }
#endif
            if (!Zone2NavSchedule_Start(&s_schedule,ZONE2_RUN_ID,(uint8_t)s_zone_locked,
                VEHICLE_MAP_VERSION,true,&s_config,2000.0f)) {
                StartupAxes_Cancel();
                fail(ZONE2_FAULT_PATH);
                return;
            }
            AppOdometry_RequestMapRun(ZONE2_RUN_ID,s_zone_locked);
#if APP_ZONE2_NAV_MOTION_ENABLE && !APP_ZONE2_MISSION_ENABLE
            s_prepare_ms=now;
#endif
            g_zone2_phase=ZONE2_PREPARE;
            return;
        }
#if !APP_ZONE2_MISSION_ENABLE
        /* Startup readiness has no business total-duration deadline. */
#endif
        return;
    }
    ChassisAuto_Zone2Read(&can);
    observed=observation(&can,now,&pose,&heading);
    if (observed) now=pose.now_ms;
#if APP_ZONE2_MISSION_ENABLE
    if (g_zone2_phase==ZONE2_QR_WAIT) {
        if (!current_map()) { fail(ZONE2_FAULT_PATH); return; }
        wait_at_scan(&can,&pose,&heading,observed,now);
        return;
    }
    if (g_zone2_phase==ZONE2_STATION_WAIT) {
        if (!current_map()) { fail(ZONE2_FAULT_PATH); return; }
        wait_at_station(&can,&pose,&heading,observed,now);
        return;
    }
#endif
    schedule_phase=Zone2NavSchedule_Poll(&s_schedule,observed?&pose:0,now);
    if (schedule_phase==ZONE2_SCHEDULE_FAILED) {
        fail(ZONE2_FAULT_PATH);
        return;
    }
    if (g_zone2_phase==ZONE2_PREPARE && observed && schedule_phase==ZONE2_SCHEDULE_READY &&
        g_can_guard_ready==1U) {
        pose.identity=s_schedule.plan.input.identity;
#if APP_ZONE2_NAV_MOTION_ENABLE
        uint32_t expected_sections,attempt,reference_ready=0U;
        /* A valid raw direct candidate gets the complete motion check first.
         * Reuse existing storage for one bounded configured-return retry. */
        for(attempt=0U;attempt<2U;attempt++) {
            expected_sections=1U+
                (fabsf(remainderf(s_schedule.plan.input.config.start_heading_deg-
                    s_schedule.plan.hold_heading_deg,360.0f))>=0.001f)+
                (fabsf(remainderf(s_schedule.plan.input.config.goal_heading_deg-
                    s_schedule.plan.hold_heading_deg,360.0f))>=0.001f);
            if (!Zone2NavPath_ReturnStopValid(&s_schedule.route,&s_schedule.plan)) {
                fail(ZONE2_FAULT_REFERENCE);
                return;
            }
            if(s_schedule.route.return_stop_node) expected_sections++;
            if(s_schedule.route.entry_needed) expected_sections++;
            if (Zone2MotionReference_PrepareRoute(&s_schedule.plan,&s_trajectory,&s_schedule.route) &&
                s_trajectory.section_count==expected_sections) {
                reference_ready=1U;
                break;
            }
            if(attempt || !Zone2NavPath_RawDirectValid(&s_schedule.route,&s_schedule.plan) ||
               (s_schedule.leg!=2U && s_schedule.leg!=5U) ||
               s_schedule.plan.input.identity.path_id!=s_schedule.leg+1U) break;
            {
                Zone2NavPathResult retry=Zone2NavPath_PrepareReturn(&s_schedule.map,
                    s_schedule.leg,&s_schedule.geometry,&s_schedule.route,&s_schedule.plan);
                if(retry!=ZONE2_NAV_OK) {
                    s_schedule.error=retry;
                    fail(ZONE2_FAULT_PATH);
                    return;
                }
            }
        }
        if(!reference_ready) {
            fail(ZONE2_FAULT_REFERENCE);
            return;
        }
        s_section=0U;g_station_turn_started_ms=0U;
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
    /* Preparation readiness has no business total-duration deadline. */
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
        if(g_zone2_phase!=ZONE2_ZERO && approach_service(&can,&pose,now)) {
            if(g_zone2_phase==ZONE2_STATION_WAIT)station_body_service(&can,&pose,&heading,observed,now);
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
#if APP_ZONE2_MISSION_ENABLE
                if(station_nearby_turn(&pose)) {
                    endpoint_error=0.0f;
                    if(g_zone2_phase==ZONE2_TRACK)g_zone2_phase=ZONE2_SECTION_ZERO;
                }
#endif
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
                    s_correction=(FusionMotionVector){0};s_zone2_heading_controller=(HeadingHoldController){0};
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
                    (void)ChassisAuto_Zone2Disarm();
            RadarZoneSwitch_Unlock();
            g_zone2_phase=ZONE2_SHADOW_DONE;
#endif
        } /* Otherwise keep waiting for actual stop evidence. */
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
