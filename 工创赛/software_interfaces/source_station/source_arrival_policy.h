#ifndef SOURCE_ARRIVAL_POLICY_H
#define SOURCE_ARRIVAL_POLICY_H
#include <stdint.h>
/* Supervised policy, not a fabricated vehicle arrival/QR parser. The owner
 * supplies a NEW real scanner frame, explicit arrival event and measured stop.
 * Physical encoder/route states are never overwritten to simulate arrival. */
enum { SAP_IDLE,SAP_WAIT_QR,SAP_WAIT_ARRIVAL,SAP_PREPARE,SAP_ALIGN,SAP_READY,SAP_FAULT=9 };
typedef struct {uint32_t state,run,qr_revision,phase_ms,reason;} SourceArrivalPolicy;
typedef struct {int32_t delta_milli_mm;uint32_t speed_mm_s,reason;} SourceCorrectionStep;
static uint32_t SourceArrival_Begin(SourceArrivalPolicy *p,uint32_t run,uint32_t now)
{
 if(!p || !run || (p->state!=SAP_IDLE && p->state!=SAP_READY))return 0U;
 p->state=SAP_WAIT_QR;p->run=run;p->qr_revision=0U;p->phase_ms=now;p->reason=0U;return 1U;
}
static uint32_t SourceArrival_Scanned(SourceArrivalPolicy *p,uint32_t run,uint32_t revision,
 uint32_t actual_code_valid,uint32_t now)
{
 if(!p || p->state!=SAP_WAIT_QR || run!=p->run || !revision || actual_code_valid!=1U)return 0U;
 p->qr_revision=revision;p->state=SAP_WAIT_ARRIVAL;p->phase_ms=now;return 1U;
}
static uint32_t SourceArrival_Arrived(SourceArrivalPolicy *p,uint32_t run,
 uint32_t explicit_event,uint32_t actual_stopped,uint32_t references_ready,uint32_t now)
{
 if(!p || p->state!=SAP_WAIT_ARRIVAL || run!=p->run || explicit_event!=1U ||
  actual_stopped!=1U || references_ready!=1U)return 0U;
 p->state=SAP_PREPARE;p->phase_ms=now;return 1U;
}
static uint32_t SourceArrival_ViewReady(SourceArrivalPolicy *p,uint32_t run,
 uint32_t actual_pose_complete,uint32_t current_camera_pose_verified,uint32_t now)
{
 if(!p || p->state!=SAP_PREPARE || run!=p->run || actual_pose_complete!=1U ||
  current_camera_pose_verified!=1U)return 0U;
 p->state=SAP_ALIGN;p->phase_ms=now;return 1U;
}
static uint32_t SourceArrival_Aligned(SourceArrivalPolicy *p,uint32_t run,
 uint32_t fresh_metric_confirmed,uint32_t actual_stopped,uint32_t now)
{
 if(!p || p->state!=SAP_ALIGN || run!=p->run || fresh_metric_confirmed!=1U || actual_stopped!=1U)return 0U;
 p->state=SAP_READY;p->phase_ms=now;return 1U;
}
/* reason0=executable,1=inside accepted tolerance,2=sub-minimum unsupported,
 *3=outside bounded observation envelope,4=invalid. No fake small-step success. */
static SourceCorrectionStep SourceCorrection_Plan(int32_t delta,uint32_t tolerance)
{
 SourceCorrectionStep s={0,0U,4U};int64_t magnitude=delta<0?-(int64_t)delta:(int64_t)delta;
 uint32_t step;
 if(!tolerance || tolerance>20000U)return s;
 if(magnitude<=tolerance){s.reason=1U;return s;}
 if(magnitude>100000){s.reason=3U;return s;}
 if(magnitude<10000){s.reason=2U;return s;}
 if(magnitude>40000){step=40000U;s.speed_mm_s=60U;}
 else if(magnitude>20000){step=20000U;s.speed_mm_s=35U;}
 else {step=10000U;s.speed_mm_s=15U;}
 s.delta_milli_mm=delta<0?-(int32_t)step:(int32_t)step;s.reason=0U;return s;
}
#endif
