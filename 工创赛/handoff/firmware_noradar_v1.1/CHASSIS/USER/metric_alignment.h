#ifndef STAGE2_METRIC_ALIGNMENT_H
#define STAGE2_METRIC_ALIGNMENT_H
#include <stdint.h>
#include <string.h>
#include <limits.h>
/* 只保存观测；没有电机、UART、到位DONE或自动放行接口。距离0.001mm，角度0.001deg。 */
enum { MA_OK=0,MA_BAD_ARGUMENT=1,MA_NOT_STOPPED=2,MA_UNCALIBRATED=3,
 MA_NO_REQUEST=4,MA_EXPIRED=5,MA_SESSION=6,MA_PROFILE=7,MA_UNITS=8,
 MA_OLD_FRAME=9,MA_CONFIDENCE=10,MA_RANGE=11,MA_DUPLICATE=12 };
typedef struct {
 uint32_t calibrated,camera_id,calibration_id,mount_id,height_id,target_id;
 uint32_t max_frame_age_ms,min_confidence_permille;
 int32_t max_delta_milli_mm,max_yaw_mdeg;
} MetricAlignmentProfile;
typedef struct {
 uint32_t request_id,run_id,stage,start_id,map_version,goal_cell,issued_ms,timeout_ms;
 int32_t nominal_x_milli_mm,nominal_y_milli_mm;
 MetricAlignmentProfile profile;
} MetricAlignmentRequest;
typedef struct {
 uint32_t request_id,run_id,stage,start_id,map_version,goal_cell;
 uint32_t camera_id,calibration_id,mount_id,height_id,target_id;
 uint32_t frame_sequence,capture_after_request_ms,frame_age_ms,confidence_permille,units;
 int32_t dx_milli_mm,dy_milli_mm,yaw_mdeg;
} MetricAlignmentObservation;
typedef struct {
 uint32_t pending,accepted,last_reason,last_sequence,have_sequence,accepted_ms;
 MetricAlignmentRequest request;
 MetricAlignmentObservation observation;
} MetricAlignmentInbox;
static uint32_t MetricAlignment_ProfileValid(const MetricAlignmentProfile *p)
{
 return p && p->calibrated==1 && p->camera_id && p->calibration_id && p->mount_id &&
  p->height_id && p->target_id && p->max_frame_age_ms>=1 && p->max_frame_age_ms<=200 &&
  p->min_confidence_permille>=1 && p->min_confidence_permille<=1000 &&
  p->max_delta_milli_mm>0 && p->max_delta_milli_mm<=100000 &&
  p->max_yaw_mdeg>0 && p->max_yaw_mdeg<=10000;
}
static uint32_t MetricAlignment_Begin(MetricAlignmentInbox *c,
 const MetricAlignmentRequest *r,uint32_t stopped_and_healthy)
{
 if(!c)return MA_BAD_ARGUMENT;
 /* 失败的新请求也必须撤销旧请求，不能跨轮次保留旧许可。 */
 memset(c,0,sizeof(*c));
 if(!r || !r->request_id || !r->run_id || r->stage!=2 || r->start_id<1 || r->start_id>2 ||
    !r->map_version || r->goal_cell!=14 || r->timeout_ms<100 || r->timeout_ms>1000 ||
    r->nominal_x_milli_mm<0 || r->nominal_x_milli_mm>=2400000 ||
    r->nominal_y_milli_mm<0 || r->nominal_y_milli_mm>=2400000)
  return c->last_reason=MA_BAD_ARGUMENT;
 if(!stopped_and_healthy)return c->last_reason=MA_NOT_STOPPED;
 if(!MetricAlignment_ProfileValid(&r->profile))return c->last_reason=MA_UNCALIBRATED;
 c->request=*r;c->pending=1;return MA_OK;
}
static uint32_t MetricAlignment_Invalidate(MetricAlignmentInbox *c)
{
 if(!c)return MA_BAD_ARGUMENT;
 memset(c,0,sizeof(*c));c->last_reason=MA_NO_REQUEST;return MA_OK;
}
static uint32_t MetricAlignment_Accept(MetricAlignmentInbox *c,
 const MetricAlignmentObservation *o,uint32_t now_ms,uint32_t stopped_and_healthy)
{
 const MetricAlignmentRequest *r;const MetricAlignmentProfile *p;uint32_t elapsed,reason=MA_OK;
 if(!c || !o)return MA_BAD_ARGUMENT;
 if(!c->pending)return c->last_reason=MA_NO_REQUEST;
 r=&c->request;p=&r->profile;elapsed=now_ms-r->issued_ms;
 if(!stopped_and_healthy){c->pending=c->accepted=0;reason=MA_NOT_STOPPED;}
 else if(elapsed>=r->timeout_ms){c->pending=c->accepted=0;reason=MA_EXPIRED;}
 else if(o->request_id!=r->request_id || o->run_id!=r->run_id || o->stage!=2 ||
  o->start_id!=r->start_id || o->map_version!=r->map_version || o->goal_cell!=r->goal_cell)reason=MA_SESSION;
 else if(o->camera_id!=p->camera_id || o->calibration_id!=p->calibration_id ||
  o->mount_id!=p->mount_id || o->height_id!=p->height_id || o->target_id!=p->target_id)reason=MA_PROFILE;
 else if(o->units!=1)reason=MA_UNITS;
 /* SBC和MCU时钟不直接相减：总往返由MCU截止约束，SBC报告请求接收后的采集时间。 */
 else if(!o->frame_sequence || o->frame_age_ms>=p->max_frame_age_ms ||
  o->capture_after_request_ms>elapsed || o->capture_after_request_ms>=r->timeout_ms ||
  elapsed-o->capture_after_request_ms>=p->max_frame_age_ms)reason=MA_OLD_FRAME;
 else if(o->confidence_permille<p->min_confidence_permille || o->confidence_permille>1000)reason=MA_CONFIDENCE;
 else if(o->dx_milli_mm < -p->max_delta_milli_mm || o->dx_milli_mm>p->max_delta_milli_mm ||
  o->dy_milli_mm < -p->max_delta_milli_mm || o->dy_milli_mm>p->max_delta_milli_mm ||
  o->yaw_mdeg < -p->max_yaw_mdeg || o->yaw_mdeg>p->max_yaw_mdeg)reason=MA_RANGE;
 else if(c->have_sequence && o->frame_sequence<=c->last_sequence)reason=MA_DUPLICATE;
 if(reason)return c->last_reason=reason;
 c->observation=*o;c->accepted=1;c->accepted_ms=now_ms;c->last_sequence=o->frame_sequence;c->have_sequence=1;
 /* 请求传输延时未知，用往返时间减SBC请求后采集偏移，保守上界覆盖传输积压。 */
 if(c->observation.frame_age_ms<elapsed-o->capture_after_request_ms)
  c->observation.frame_age_ms=elapsed-o->capture_after_request_ms;
 return c->last_reason=MA_OK;
}
/* 读取也复核时效和停车状态；accepted是观测通过，绝不表示已对准。 */
static uint32_t MetricAlignment_Read(MetricAlignmentInbox *c,uint32_t now_ms,
 uint32_t stopped_and_healthy,MetricAlignmentObservation *out)
{
 if(!c || !out)return MA_BAD_ARGUMENT;
 if(!stopped_and_healthy){MetricAlignment_Invalidate(c);return MA_NOT_STOPPED;}
 if(!c->pending || !c->accepted)return MA_NO_REQUEST;
 if(now_ms-c->request.issued_ms>=c->request.timeout_ms ||
    now_ms-c->accepted_ms>=c->request.profile.max_frame_age_ms-c->observation.frame_age_ms){
  c->accepted=0;return c->last_reason=MA_EXPIRED;
 }
 *out=c->observation;return MA_OK;
}
/* 有界ASCII帧解码，CRC16/CCITT-FALSE；仅完整帧，不承担UART流分帧。 */
static uint32_t MetricAlignment_Decode(const char *wire,uint32_t length,MetricAlignmentObservation *out)
{
 uint32_t i,pos=4U,j;uint16_t crc=0xffffU,expected=0U;int64_t values[19];
 if(!wire || !out || length<12U || length>256U || memcmp(wire,"MA1,",4) ||
    wire[length-2]!='\r' || wire[length-1]!='\n')return MA_BAD_ARGUMENT;
 for(i=0;i<length-6;i++){
  crc^=(uint16_t)(uint8_t)wire[i]<<8;
  for(j=0;j<8;j++)crc=(uint16_t)((crc&0x8000U)?((uint32_t)crc<<1)^0x1021U:(uint32_t)crc<<1);
 }
 for(i=length-6;i<length-2;i++){
  uint32_t v=(uint8_t)wire[i];
  if(v>='0' && v<='9')v-='0';else if(v>='A' && v<='F')v=v-'A'+10U;else return MA_BAD_ARGUMENT;
  expected=(uint16_t)((expected<<4)|v);
 }
 if(expected!=crc)return MA_BAD_ARGUMENT;
 for(i=0;i<19;i++){
  int64_t v=0;uint32_t digits=0,negative=0;
  if(pos<length-6 && wire[pos]=='-'){if(i<16)return MA_BAD_ARGUMENT;negative=1;pos++;}
  while(pos<length-6 && wire[pos]>='0' && wire[pos]<='9'){
   if(++digits>10)return MA_BAD_ARGUMENT;
   v=v*10+(wire[pos++]-'0');
  }
  if(!digits || pos>=length-6 || wire[pos++]!=',')return MA_BAD_ARGUMENT;
  v=negative?-v:v;
  if((i<16 && v>4294967295LL) || (i>=16 && (v<INT32_MIN || v>INT32_MAX)))return MA_BAD_ARGUMENT;
  values[i]=v;
 }
 if(pos!=length-6)return MA_BAD_ARGUMENT;
 {MetricAlignmentObservation o;
  o.request_id=(uint32_t)values[0];o.run_id=(uint32_t)values[1];o.stage=(uint32_t)values[2];
  o.start_id=(uint32_t)values[3];o.map_version=(uint32_t)values[4];o.goal_cell=(uint32_t)values[5];
  o.camera_id=(uint32_t)values[6];o.calibration_id=(uint32_t)values[7];o.mount_id=(uint32_t)values[8];
  o.height_id=(uint32_t)values[9];o.target_id=(uint32_t)values[10];o.frame_sequence=(uint32_t)values[11];
  o.capture_after_request_ms=(uint32_t)values[12];o.frame_age_ms=(uint32_t)values[13];
  o.confidence_permille=(uint32_t)values[14];o.units=(uint32_t)values[15];
  o.dx_milli_mm=(int32_t)values[16];o.dy_milli_mm=(int32_t)values[17];o.yaw_mdeg=(int32_t)values[18];
  *out=o;
 }
 return MA_OK;
}
#endif
