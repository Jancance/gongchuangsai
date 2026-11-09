#include "source_observer.h"
#include "metric_alignment_bridge.h"
#include <stdio.h>
#include <string.h>
#ifndef SOURCE_OBSERVER_HOST
#include "FreeRTOS.h"
#include "task.h"
#define LOCK() taskENTER_CRITICAL()
#define UNLOCK() taskEXIT_CRITICAL()
#else
#define LOCK() ((void)0)
#define UNLOCK() ((void)0)
#endif
volatile uint32_t g_source_observer_reason,g_source_observer_accepted;
volatile uint32_t g_source_observer_requests,g_source_observer_frames,g_source_observer_dropped;
volatile int32_t g_source_observer_dx_milli_mm,g_source_observer_dy_milli_mm;
static MetricAlignmentInbox inbox;
static MetricAlignmentProfile profile;
static char incoming[256],queued[256],request_wire[256];
static uint32_t used,invalid,last_byte,request_size,sequence;
static volatile uint32_t queued_size;
static uint16_t crc16(const char*p,uint32_t n)
{
 uint16_t crc=0xffffU;uint32_t i,j;
 for(i=0;i<n;i++){crc^=(uint16_t)(uint8_t)p[i]<<8;
  for(j=0;j<8;j++)crc=(uint16_t)((crc&0x8000U)?((uint32_t)crc<<1)^0x1021U:(uint32_t)crc<<1);}
 return crc;
}
void SourceObserver_Init(const MetricAlignmentProfile*p)
{
 memset(&inbox,0,sizeof(inbox));memset(&profile,0,sizeof(profile));if(p)profile=*p;
 used=invalid=last_byte=queued_size=request_size=sequence=0;
 g_source_observer_reason=g_source_observer_accepted=g_source_observer_requests=0;
 g_source_observer_frames=g_source_observer_dropped=0;
 g_source_observer_dx_milli_mm=g_source_observer_dy_milli_mm=0;
}
void SourceObserver_Byte(uint8_t byte,uint32_t now)
{
 if((uint32_t)(now-last_byte)>=200U){used=invalid=0;}
 last_byte=now;if(byte=='\r')return;
 if(byte=='\n'){
  if(!invalid && used>=4U && !memcmp(incoming,"MA1,",4U)){
   LOCK();
   if(!queued_size){memcpy(queued,incoming,used);queued[used]='\r';queued[used+1]='\n';queued_size=used+2U;}
   else g_source_observer_dropped++;
   UNLOCK();
  }
  used=invalid=0;return;
 }
 if(invalid)return;
 if(byte<32U || byte>126U || used>=sizeof(incoming)-2U){invalid=1;g_source_observer_dropped++;return;}
 incoming[used++]=(char)byte;
}
static void publish_request(void)
{
 const MetricAlignmentRequest*r=&inbox.request;const MetricAlignmentProfile*p=&r->profile;int n;
 n=snprintf(request_wire,sizeof(request_wire),
  "MR1,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%ld,%ld,%ld,%ld,",
  (unsigned long)r->request_id,(unsigned long)r->run_id,(unsigned long)r->stage,
  (unsigned long)r->start_id,(unsigned long)r->map_version,(unsigned long)r->goal_cell,
  (unsigned long)r->timeout_ms,(unsigned long)p->camera_id,(unsigned long)p->calibration_id,
  (unsigned long)p->mount_id,(unsigned long)p->height_id,(unsigned long)p->target_id,
  (unsigned long)p->max_frame_age_ms,(unsigned long)p->min_confidence_permille,
  (long)p->max_delta_milli_mm,(long)p->max_yaw_mdeg,
  (long)r->nominal_x_milli_mm,(long)r->nominal_y_milli_mm);
 if(n<=0 || (uint32_t)n>=sizeof(request_wire)-7U){request_size=0;return;}
 snprintf(request_wire+n,sizeof(request_wire)-(uint32_t)n,"%04X\r\n",crc16(request_wire,(uint32_t)n));
 request_size=(uint32_t)n+6U;
}
void SourceObserver_Service(uint32_t now)
{
 char frame[256];uint32_t length;MetricAlignmentObservation o;
 LOCK();length=queued_size;if(length){memcpy(frame,queued,length);queued_size=0;}UNLOCK();
 if(!MetricAlignment_ProfileValid(&profile)){
  MetricAlignment_Invalidate(&inbox);request_size=0;
  g_source_observer_accepted=0;g_source_observer_reason=MA_UNCALIBRATED;return;
 }
 if(!inbox.pending || !MetricAlignmentBridge_Current(&inbox) || now-inbox.request.issued_ms>=inbox.request.timeout_ms){
  if(++sequence==0U)sequence=1U;
  g_source_observer_reason=MetricAlignmentBridge_Open(&inbox,&profile,sequence,now);
  g_source_observer_accepted=0;
  if(g_source_observer_reason==MA_OK){g_source_observer_requests++;publish_request();}
  else request_size=0;
 }
 if(length){
  g_source_observer_frames++;
  g_source_observer_reason=MetricAlignmentBridge_Submit(&inbox,frame,length,now);
 }
 if(MetricAlignmentBridge_Read(&inbox,now,&o)==MA_OK){
  g_source_observer_accepted=1;
  g_source_observer_dx_milli_mm=o.dx_milli_mm;g_source_observer_dy_milli_mm=o.dy_milli_mm;
 }else{
  g_source_observer_accepted=0;
  g_source_observer_dx_milli_mm=g_source_observer_dy_milli_mm=0;
 }
}
uint32_t SourceObserver_Request(char*out,uint32_t capacity)
{
 uint32_t n=request_size;if(!out || !n || capacity<=n)return 0;
 memcpy(out,request_wire,n);out[n]='\0';request_size=0;return n;
}
