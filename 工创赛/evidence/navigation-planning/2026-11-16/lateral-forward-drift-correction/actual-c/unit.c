
#include <stdint.h>
#include <string.h>
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_config.h"
#include "app_entry.h"
#include "motion_distance_limits.h"
#include "app_fusion_capture.h"
#include "app_odometry.h"
#include "chassis_auto_test.h"
#include "heading_source.h"
#include "power.h"
static uint32_t clock_ms;
static ChassisAutoStatus motion;
static ChassisWheelLog wheel;
static ChassisOdometrySamples position;
#if APP_BASELINE40_ACTIVE
static AppOdometryBaseline40 od;
#endif
static AppHeadingSource heading;
volatile uint32_t g_vm_test_request,g_safety_fault,g_rtos_fault,g_can_ctrl_fault_latched;
uint32_t SystemCoreClock=168000000U;
TickType_t xTaskGetTickCount(void){return clock_ms;}
uint32_t ChassisAuto_ReadStatus(ChassisAutoStatus *v){*v=motion;return 1U;}
uint32_t ChassisAuto_ReadWheelLog(ChassisWheelLog *v){*v=wheel;return 1U;}
void ChassisAuto_ReadOdometrySamples(ChassisOdometrySamples *v){*v=position;}
#if APP_BASELINE40_ACTIVE
void AppOdometry_ReadBaseline40(AppOdometryBaseline40 *v){*v=od;}
#endif
uint32_t App_HeadingSourceRead(AppHeadingSource *v){*v=heading;return 1U;}
#include "app_fusion_capture.h"
#if APP_BASELINE40_ACTIVE && APP_FUSION_CAPTURE_ENABLE
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_odometry.h"
#include "chassis_auto_test.h"
#include "heading_source.h"
#include "safety.h"
#include "freertos_hooks.h"
#include "can.h"
#include "power.h"
extern volatile uint32_t g_vm_test_request;
#if APP_FUSION_CAPTURE_STATIC_TEST
volatile uint32_t g_fusion_static_request;
#endif

/* Fixed schema v1. All times are receiver/service proxies, never ADC time. */
typedef struct {
    uint32_t ms, edge_cycle;
    int16_t accel[3], gyro[3], temperature;
    uint16_t edge_delta;
} CaptureRaw;
typedef struct { uint32_t now_ms, source_ms, value, meta; } CaptureWheel;
typedef struct {
    uint32_t now_ms, angle_ms, rate_ms;
    float yaw_deg, rate_dps;
    uint32_t flags;
} CaptureHwt;
typedef struct {
    uint32_t now_ms, sample_ms;
    float x_mm, y_mm, heading_deg;
    uint32_t updates, reasons, continuity_lost, motion, faults;
} CaptureOd;
typedef struct {
    uint32_t magic, version, size_a, size_b, state, reason, session;
    uint32_t trigger_ms, stop_ms, freeze_ms, pre_count, pre_head, post_count;
    uint32_t raw_count, wheel_count, hwt_count, od_count;
    uint32_t wheel_dropped, hwt_dropped, od_dropped, quality;
    uint32_t first_sample_sequence, last_sample_sequence;
    uint32_t first_edge_sequence, last_edge_sequence;
    uint32_t trigger_skipped, final_skipped, trigger_races, final_races;
    uint32_t trigger_errors, final_errors, accel_config, gyro_config, whoami;
    uint32_t max_read_lag_cycles, max_read_duration_cycles, max_step_cycles;
    uint32_t reference_set_ms;
    float origin_x_mm, origin_y_mm, path_heading_deg, heading_offset_deg;
    uint32_t first_ms, last_ms, last_raw_state, last_raw_error;
    uint32_t system_core_clock, reserved[17];
} CaptureHeader;
typedef struct {
    union { ChassisAutoStatus motion; ChassisWheelLog wheel; } shared;
    ChassisOdometrySamples position;
    AppOdometryBaseline40 od;
    AppHeadingSource hwt;
    uint32_t source_ms[8], source_meta[8], source_value[8], source_seen;
    uint32_t hwt_ms, od_ms, tail_seen, seen_raw;
} CaptureScratch;
typedef struct {
    CaptureHeader h;
    CaptureRaw raw[1050];
    CaptureWheel wheel[512];
    CaptureScratch scratch;
} CaptureBankA;
typedef struct {
    CaptureRaw raw[1050];
    CaptureHwt hwt[210];
    CaptureOd od[84];
} CaptureBankB;
typedef char CaptureLayoutCheck[(sizeof(CaptureRaw)==24U && sizeof(CaptureWheel)==16U &&
    sizeof(CaptureHwt)==24U && sizeof(CaptureOd)==40U && sizeof(CaptureHeader)==256U &&
    sizeof(CaptureHeader)+sizeof(CaptureScratch)<=768U &&
    sizeof(CaptureBankA)<=35840U && sizeof(CaptureBankB)<=35840U) ? 1 : -1];
/* Existing RAM regions remain untouched. Link-map validation is mandatory. */
#if defined(CAPTURE_UNIT_TEST)
CaptureBankA g_fusion_capture_a;
CaptureBankB g_fusion_capture_b;
#else
CaptureBankA g_fusion_capture_a __attribute__((at(0x20017300),zero_init));
CaptureBankB g_fusion_capture_b __attribute__((at(0x10007300),zero_init));
#endif

static CaptureRaw *Capture_At(uint32_t index)
{
    return index<1050U ? &g_fusion_capture_a.raw[index] :
        &g_fusion_capture_b.raw[index-1050U];
}
static void Capture_Freeze(uint32_t reason,uint32_t now)
{
    CaptureHeader *h=&g_fusion_capture_a.h;
    h->reason=reason;h->freeze_ms=now;
    __DMB();h->state=2U;__DMB();
}
static void Capture_Sources(uint32_t now)
{
    CaptureHeader *h=&g_fusion_capture_a.h;
    CaptureScratch *s=&g_fusion_capture_a.scratch;
    uint32_t i,j,ms,meta,value;
    uint32_t motion=(s->shared.motion.state&255U) | ((s->shared.motion.result&255U)<<8) |
        ((s->shared.motion.cause&255U)<<16) | ((s->shared.motion.active&1U)<<24);
    ChassisAuto_ReadOdometrySamples(&s->position);
    (void)ChassisAuto_ReadWheelLog(&s->shared.wheel);
    for(i=0U;i<8U;i++) {
        j=i&3U;
        if(i<4U) {
            ms=s->position.ms[j];value=s->position.raw[j];
            meta=j | (1UL<<8) | ((s->position.sign[j]&1U)<<16) |
                (((s->position.mask>>j)&1U)<<17);
        } else {
            ms=s->shared.wheel.sample_ms[j];value=(uint32_t)s->shared.wheel.measured_decirpm[j];
            meta=j | (2UL<<8) | (((s->shared.wheel.speed_valid_mask>>j)&1U)<<17);
        }
        if(!(s->source_seen&(1UL<<i)) || ms!=s->source_ms[i] || meta!=s->source_meta[i] || value!=s->source_value[i]) {
            s->source_seen|=1UL<<i;s->source_ms[i]=ms;s->source_meta[i]=meta;s->source_value[i]=value;
            if(h->wheel_count<512U) {
                CaptureWheel *v=&g_fusion_capture_a.wheel[h->wheel_count++];
                v->now_ms=now;v->source_ms=ms;v->value=value;v->meta=meta;
            } else h->wheel_dropped++;
        }
    }
    if(h->hwt_count==0U || now-s->hwt_ms>=50U) {
        s->hwt_ms=now;(void)App_HeadingSourceRead(&s->hwt);
        if(h->hwt_count<210U) {
            CaptureHwt *v=&g_fusion_capture_b.hwt[h->hwt_count++];
            v->now_ms=now;v->angle_ms=s->hwt.angle_ms;v->rate_ms=s->hwt.rate_ms;
            v->yaw_deg=s->hwt.yaw_deg;v->rate_dps=s->hwt.rate_dps;
            v->flags=(s->hwt.healthy&1U) | ((s->hwt.valid_mask&3U)<<8);
        } else h->hwt_dropped++;
    }
    if(h->od_count==0U || now-s->od_ms>=125U) {
        s->od_ms=now;AppOdometry_ReadBaseline40(&s->od);
        if(s->od.anchor_valid && !h->reserved[0]) {
            h->reserved[0]=1U;h->reference_set_ms=s->od.reference_set_ms;
            h->origin_x_mm=s->od.origin_x_mm;h->origin_y_mm=s->od.origin_y_mm;
            h->path_heading_deg=s->od.path_heading_deg;h->heading_offset_deg=s->od.heading_offset_deg;
        } else if(h->reserved[0] && s->od.reference_set_ms!=h->reference_set_ms)h->quality|=8U;
        if(h->od_count<84U) {
            CaptureOd *v=&g_fusion_capture_b.od[h->od_count++];
            v->now_ms=now;v->sample_ms=s->od.pose.sample_ms;
            v->x_mm=s->od.pose.x_mm;v->y_mm=s->od.pose.y_mm;
            v->heading_deg=s->od.pose.heading_deg;v->updates=s->od.pose.updates;
            v->reasons=s->od.pose.reasons;v->continuity_lost=s->od.pose.continuity_lost;
            v->motion=motion | ((s->od.pose.valid&1U)<<25) | ((g_vm_test_request&1U)<<26);
            v->faults=(g_safety_fault?1U:0U) | (g_rtos_fault?2U:0U) |
                (g_can_ctrl_fault_latched?4U:0U);
        } else h->od_dropped++;
    }
}
void AppFusionCapture_Step(const IcmRawStatus *raw)
{
    CaptureHeader *h=&g_fusion_capture_a.h;
    CaptureScratch *s=&g_fusion_capture_a.scratch;
    CaptureRaw *v;uint32_t now,started,delta,index,i,lag,duration;
    if(h->state==2U)return;
    started=DWT->CYCCNT;now=(uint32_t)xTaskGetTickCount()*portTICK_PERIOD_MS;
    if(h->magic==0U) {
        h->magic=0x31434246U;h->version=1U;
        h->size_a=sizeof(g_fusion_capture_a);h->size_b=sizeof(g_fusion_capture_b);
        h->system_core_clock=SystemCoreClock;
#if APP_FUSION_CAPTURE_STATIC_TEST
        h->reserved[1]=0x53544154U;h->reserved[3]=8000U;
#endif
    }
    (void)ChassisAuto_ReadStatus(&s->shared.motion);
#if APP_FUSION_CAPTURE_STATIC_TEST
    if(h->state==0U && g_fusion_static_request==0x53544154U) {
        g_fusion_static_request=0U;
        if(s->shared.motion.active || s->shared.motion.state==CHASSIS_AUTO_START) {
            h->reserved[2]++;goto static_wait;
        }
#else
    if(h->state==0U && s->shared.motion.state==CHASSIS_AUTO_START) {
#endif
        h->state=1U;h->session=s->shared.motion.session;h->trigger_ms=now;
        h->trigger_skipped=raw->skipped_edges;h->trigger_races=raw->read_races;
        h->trigger_errors=raw->transfer_errors;
        if(h->pre_count) {
            index=(h->pre_head+100U-h->pre_count)%100U;
            h->first_ms=Capture_At(index)->ms;
            h->first_sample_sequence=h->last_sample_sequence-h->pre_count+1U;
            h->first_edge_sequence=h->last_edge_sequence;
            for(i=1U;i<h->pre_count;i++)
                h->first_edge_sequence-=Capture_At((index+i)%100U)->edge_delta;
        }
        AppOdometry_ReadBaseline40(&s->od);
        h->reserved[0]=s->od.anchor_valid;
        h->reference_set_ms=s->od.reference_set_ms;h->origin_x_mm=s->od.origin_x_mm;
        h->origin_y_mm=s->od.origin_y_mm;h->path_heading_deg=s->od.path_heading_deg;
        h->heading_offset_deg=s->od.heading_offset_deg;
    }
#if APP_FUSION_CAPTURE_STATIC_TEST
static_wait:
#endif
    if(h->state==1U) {
        if(s->shared.motion.session!=h->session){Capture_Freeze(3U,now);return;}
#if APP_FUSION_CAPTURE_STATIC_TEST
        if(s->shared.motion.active || s->shared.motion.state==CHASSIS_AUTO_START) {
            Capture_Freeze(5U,now);return;
        }
        if(now-h->trigger_ms>=8000U){Capture_Freeze(4U,now);return;}
#else
        if(!s->tail_seen && (s->shared.motion.state==CHASSIS_AUTO_STOP ||
            s->shared.motion.state==CHASSIS_AUTO_FAULT || s->shared.motion.state==CHASSIS_AUTO_CANCELLED ||
            s->shared.motion.state==CHASSIS_AUTO_DONE)) {
            s->tail_seen=1U;h->stop_ms=now;
        }
        /* reserved[4]: terminal seen, reserved[5]: first terminal receiver ms. STOP time is unchanged. */
        if(!h->reserved[4] && (s->shared.motion.state==CHASSIS_AUTO_FAULT ||
            s->shared.motion.state==CHASSIS_AUTO_CANCELLED || s->shared.motion.state==CHASSIS_AUTO_DONE)) {
            h->reserved[4]=1U;h->reserved[5]=now;
        }
        if(h->reserved[4] && now-h->reserved[5]>=500U){Capture_Freeze(1U,now);return;}
        if(now-h->trigger_ms>=10000U){Capture_Freeze(2U,now);return;}
#endif
    }
    h->last_raw_state=raw->state;h->last_raw_error=raw->last_error;
    h->final_skipped=raw->skipped_edges;h->final_races=raw->read_races;
    h->final_errors=raw->transfer_errors;
    if(raw->state!=ICM_RAW_READY || raw->last_error!=ICM_RAW_OK || !raw->fresh ||
        !raw->sample_sequence) {if(h->state==1U)h->quality|=1U;goto sources;}
    if(s->seen_raw && raw->sample_sequence==h->last_sample_sequence)goto sources;
    delta=s->seen_raw?raw->edge_sequence-h->last_edge_sequence:0U;
    if(s->seen_raw && (raw->sample_sequence-h->last_sample_sequence!=1U ||
        delta!=1U || raw->sample_tick_ms-h->last_ms<1U ||
        raw->sample_tick_ms-h->last_ms>20U))h->quality|=2U;
    if(delta>65535U){delta=65535U;h->quality|=4U;}
    if(h->state==0U) {
        index=h->pre_head;h->pre_head=(h->pre_head+1U)%100U;
        if(h->pre_count<100U)h->pre_count++;
    } else {
        if(h->post_count>=2000U){Capture_Freeze(2U,now);return;}
        index=100U+h->post_count++;
    }
    v=Capture_At(index);v->ms=raw->sample_tick_ms;v->edge_cycle=raw->edge_cycle;
    for(i=0U;i<3U;i++){v->accel[i]=raw->raw_accel[i];v->gyro[i]=raw->raw_gyro[i];}
    v->temperature=raw->raw_temperature;v->edge_delta=(uint16_t)delta;
    if(!s->seen_raw) {
        h->first_sample_sequence=raw->sample_sequence;h->first_edge_sequence=raw->edge_sequence;
        h->first_ms=raw->sample_tick_ms;
    }
    s->seen_raw=1U;h->last_sample_sequence=raw->sample_sequence;
    h->last_edge_sequence=raw->edge_sequence;h->last_ms=raw->sample_tick_ms;
    h->raw_count=h->pre_count+h->post_count;
    h->accel_config=raw->accel_config;h->gyro_config=raw->gyro_config;h->whoami=raw->whoami;
    lag=raw->read_start_cycle-raw->edge_cycle;duration=raw->read_end_cycle-raw->read_start_cycle;
    if(lag>h->max_read_lag_cycles)h->max_read_lag_cycles=lag;
    if(duration>h->max_read_duration_cycles)h->max_read_duration_cycles=duration;
sources:
    if(h->state==1U)Capture_Sources(now);
    duration=DWT->CYCCNT-started;
    if(duration>h->max_step_cycles)h->max_step_cycles=duration;
}
#endif

volatile uint32_t review_assertions;
#define CK(x) do{review_assertions++;if(!(x))return __LINE__;}while(0)
#if APP_BASELINE40_ACTIVE
static IcmRawStatus rr;
static void sample(uint32_t ms){
 clock_ms=ms;DWT->CYCCNT=ms*168000U;rr.state=ICM_RAW_READY;rr.last_error=ICM_RAW_OK;rr.fresh=1;
 rr.sample_sequence++;rr.edge_sequence++;rr.sample_tick_ms=ms;rr.edge_cycle=ms*168000U;
 rr.read_start_cycle=rr.edge_cycle+20;rr.read_end_cycle=rr.edge_cycle+100;
 AppFusionCapture_Step(&rr);
}
static void reset_capture(void){
 memset(&g_fusion_capture_a,0,sizeof(g_fusion_capture_a));memset(&g_fusion_capture_b,0,sizeof(g_fusion_capture_b));
 memset(&rr,0,sizeof(rr));memset(&motion,0,sizeof(motion));motion.session=1;
}
static uint32_t capture_check(uint32_t terminal,uint32_t kind,uint32_t expected){
 uint32_t ms;reset_capture();
 for(ms=0;ms<=14000;ms+=5){
  motion.state=ms<500 ? CHASSIS_AUTO_IDLE : (ms==500 ? CHASSIS_AUTO_START : CHASSIS_AUTO_RUN);
  if(ms>=8550)motion.state=CHASSIS_AUTO_STOP;
  if(ms>=terminal)motion.state=kind;
  sample(ms);if(g_fusion_capture_a.h.state==2)break;
 }
 CK(g_fusion_capture_a.h.reason==expected);
 CK(g_fusion_capture_a.h.raw_count<=2100 && g_fusion_capture_a.h.post_count<=2000);
 if(expected==1){
  CK(g_fusion_capture_a.h.reserved[4]==1 && g_fusion_capture_a.h.reserved[5]==terminal);
  CK(g_fusion_capture_a.h.freeze_ms==terminal+500);
 }else{CK(g_fusion_capture_a.h.freeze_ms==10500 && g_fusion_capture_a.h.post_count==2000);}
 if(terminal==9700){CK(g_fusion_capture_a.h.stop_ms==8550 && g_fusion_capture_a.h.post_count==1940);}
 return 0;
}
#endif
__attribute__((section("i.test_entry"))) uint32_t test_entry(void){
 uint32_t id,d,r,n,b,i,z;MotionAction a[8];ChassisMotorCommand cmd[4];
 for(id=8;id<=9;id++)for(d=0;d<4;d++)for(r=99;r<=103;r++){
  CK(AppDebugPlan40Allowed(id,d,r,2100)==(APP_BASELINE40_ACTIVE && ((id==8&&d==0&&r==100)||(id==9&&d==1&&r==102))));
 }
 CK(!AppDebugPlan40Allowed(9,1,102,2099));CK(!AppDebugPlan40Allowed(9,1,102,2101));
 CK(AppDebugPlan_Load(8,a,&n)==APP_BASELINE40_ACTIVE);
 if(APP_BASELINE40_ACTIVE){CK(n==1 && a[0].kind==4 && a[0].base_rpm==100);}
 CK(AppDebugPlan_Load(9,a,&n)==APP_BASELINE40_ACTIVE);
 if(APP_BASELINE40_ACTIVE){CK(n==1 && a[0].kind==5 && a[0].base_rpm==102 && a[0].distance_mm==2100);}
#if APP_BASELINE40_ACTIVE
 CK(MOTION_PROFILE_BASE_CEILING==102 && MotionDistanceFollowRpm(102)==2 && MotionDistanceFollowKp(102)==1);
 CK(MotionDistanceReferenceSpeed(3.14159265358979323846f*77/60,102)==400);
 CK(MotionDistanceAcceleration(1,102,2100)==(APP_LEFT40_DRIFT_TRIM_ENABLE ? 150.0f/1.04f : 150.0f));
 CK(MotionDistanceJerk(1,102,2100)==(APP_LEFT40_DRIFT_TRIM_ENABLE ? 300.0f/1.04f : 300.0f));
 CK(MotionDistanceAcceleration(0,100,2100)==150 && MotionDistanceJerk(0,100,2100)==300);
 CK(MotionDistanceFeedbackLimit(1,102,2100)==130 && MotionDistanceRunMsFor(1,102,2100)==12779);
 CK(MotionDistanceJumpCountsPerMs(1,102,2100)==175);
 for(b=0;b<=102;b++)for(z=0;z<3;z++){
  float correction=(float)((int32_t)z*2-2);float eff=correction;
  CK(Chassis_TranslationProfileCommands(1,b,correction,cmd));
  for(i=0;i<4;i++)CK(cmd[i].speed_raw<=1040);
  if(b>2){CK(cmd[0].direction==0 && cmd[1].direction==0 && cmd[2].direction==1 && cmd[3].direction==1);}
  if(b==0){for(i=0;i<4;i++)CK(cmd[i].speed_raw==0);}
 }

 for(b=0;b<=102;b++)for(z=0;z<3;z++){
  ChassisMotorCommand old[4];int32_t logical[4];float h=(float)((int32_t)z*2-2);uint32_t trim=(uint32_t)(.04f*b+.5f);
  CK(Chassis_Left40DriftCommands(b,h,cmd));
  CK(Chassis_TranslationProfileCommands(1,b,h,old));
  for(i=0;i<4;i++){
   logical[i]=cmd[i].direction ? -(int32_t)cmd[i].speed_raw : (int32_t)cmd[i].speed_raw;
   if(i==1 || i==3)logical[i]=-logical[i];
   CK(cmd[i].speed_raw<=1080);
  }
  CK(logical[0]+logical[1]+logical[2]+logical[3]==-(int32_t)trim*40);
  CK(logical[0]-logical[1]-logical[2]+logical[3]==(int32_t)b*40);
  if(!trim)CK(!memcmp(cmd,old,sizeof(cmd)));
 }
 CK(!Chassis_Left40DriftCommands(103,0,cmd));
 CK(!Chassis_Left40DriftCommands(102,3,cmd));
 CK(!Chassis_Left40DriftCommands(102,0,0));
 CK(!Chassis_TranslationProfileCommands(1,103,0,cmd));
 CK(Chassis_Baseline40Commands(100,0,0,cmd));
 CK(!Chassis_Baseline40Commands(102,0,0,cmd));
 z=capture_check(9700,CHASSIS_AUTO_DONE,1);if(z)return z;
 z=capture_check(13500,CHASSIS_AUTO_DONE,2);if(z)return z;
 z=capture_check(1500,CHASSIS_AUTO_FAULT,1);if(z)return z;
 z=capture_check(1500,CHASSIS_AUTO_CANCELLED,1);if(z)return z;
#else
 CK(!MotionTrial40Allowed(1,102,2100));
#endif
 return 0;
}
