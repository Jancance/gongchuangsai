
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <float.h>
#include "app_icm_observer.h"
#include "app_odometry.h"
#include "chassis_auto_test.h"
#include "heading_source.h"
#include "power.h"
static uint32_t locks,clock_ms,tx_busy,tx_calls,s_serial_tx_tick,s_serial_tx_valid=1U;
#define taskENTER_CRITICAL() (++locks)
#define taskEXIT_CRITICAL() (--locks)
#define APP_SERIAL_STATUS_PERIOD_MS 100U
static AppOdometryBaseline40 tw;
static ChassisAutoStatus tm;
static PowerCommandStatus tp;
static AppHeadingSource ty;
static char wire[4][384];static uint32_t wire_count;
static uint32_t xTaskGetTickCount(void){return clock_ms;}
void AppOdometry_ReadBaseline40(AppOdometryBaseline40 *v){*v=tw;}
uint32_t ChassisAuto_ReadStatus(ChassisAutoStatus *v){*v=tm;return 1U;}
void Power_ReadCommandStatus(PowerCommandStatus *v){*v=tp;}
uint32_t App_HeadingSourceRead(AppHeadingSource *v){*v=ty;return 1U;}
static uint32_t App_DebugSendString(const char *v){tx_calls++;if(tx_busy)return 0U;if(wire_count<4U)strcpy(wire[wire_count++],v);return 1U;}


#if APP_BASELINE40_ACTIVE









#define OBS_CAPACITY 128U
#define OBS_G 9806.65f
#define OBS_RAD 0.0174532925199433f
typedef struct { uint32_t ms; float a[2]; } ObsPoint;
static ObsPoint s_history[OBS_CAPACITY];
static uint32_t s_count, s_head, s_anchors, s_anchor_ms[2];
static float s_anchor_p[2][2];
static float s_gravity[3], s_accel_bias[3], s_gyro_bias[3];
static int32_t s_accel_sum[3], s_gyro_sum[3];
static uint32_t s_cal_count, s_cal_ready, s_cal_blocked, s_ever_active;
static uint32_t s_seen, s_last_seq, s_last_edge, s_last_ms;
static uint32_t s_last_skipped, s_last_races, s_last_errors;
static uint32_t s_session, s_reference, s_continuity, s_frame_seen;
static uint32_t s_last_predict, s_predicted, s_gaps, s_gravity_gap;
static AppHeadingSource s_yaw;
static AppIcmObserverStatus s_value, s_published;

static uint32_t Obs_Finite(float x) { return x>=-3.4028234e38f && x<=3.4028234e38f; }
static void Obs_Clear(void) { s_count=s_head=s_anchors=0U; }
static void Obs_CalClear(void)
{
    s_cal_count=0U;
    memset(s_accel_sum,0,sizeof(s_accel_sum));
    memset(s_gyro_sum,0,sizeof(s_gyro_sum));
}
static void Obs_Cross(const float *a,const float *b,float *c)
{
    c[0]=a[1]*b[2]-a[2]*b[1]; c[1]=a[2]*b[0]-a[0]*b[2];
    c[2]=a[0]*b[1]-a[1]*b[0];
}
static uint32_t Obs_Normalize(float *v)
{
    float n=sqrtf(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]); uint32_t i;
    if (!(n>0.0f) || !Obs_Finite(n)) return 0U;
    for(i=0U;i<3U;i++) v[i]*=OBS_G/n;
    return 1U;
}
static void Obs_Publish(uint32_t reasons)
{
    s_value.reasons=reasons; s_value.valid=(reasons==0U);
    s_value.flags=(s_cal_ready?ICM_OBS_CAL:0U) | ICM_OBS_NO_LEVER |
        (s_gravity_gap?ICM_OBS_GRAVITY_GAP:0U) |
        (s_cal_blocked?ICM_OBS_CAL_BLOCKED:0U) |
        (s_value.flags&(ICM_OBS_WHEEL|ICM_OBS_ACCEL));
    s_value.cal_samples=s_cal_count; s_value.history_samples=s_count;
    s_value.gaps=s_gaps; s_value.sequence++;
    taskENTER_CRITICAL(); s_published=s_value; taskEXIT_CRITICAL();
}
void AppIcmObserver_Read(AppIcmObserverStatus *snapshot)
{
    taskENTER_CRITICAL(); *snapshot=s_published; taskEXIT_CRITICAL();
}
static const ObsPoint *Obs_At(uint32_t index)
{ return &s_history[(s_head+OBS_CAPACITY-s_count+index)%OBS_CAPACITY]; }
static void Obs_Append(uint32_t ms,float f,float r)
{
    s_history[s_head].ms=ms; s_history[s_head].a[0]=f; s_history[s_head].a[1]=r;
    s_head=(s_head+1U)%OBS_CAPACITY; if(s_count<OBS_CAPACITY)s_count++;
}
/* Integrate only local time, clipping/interpolating acceleration at each bound. */
static uint32_t Obs_Integrals(uint32_t start,uint32_t finish,uint32_t origin,
    uint32_t now,float *iv,float *im)
{
    uint32_t i,j,span,lo,hi,left,right; float x,y,d,al,ar,A,B;
    const ObsPoint *p,*q;
    iv[0]=iv[1]=im[0]=im[1]=0.0f;
    if(start==finish)return 1U;
    for(i=1U;i<s_count;i++) {
        p=Obs_At(i-1U);q=Obs_At(i);span=q->ms-p->ms;
        left=start-p->ms;right=finish-p->ms;
        if((int32_t)(finish-p->ms)<=0 || (int32_t)(start-q->ms)>=0)continue;
        lo=((int32_t)left>0)?left:0U; hi=(right<span)?right:span;
        if(hi<=lo)continue;
        x=(float)lo/(float)span;y=(float)hi/(float)span;d=(float)(hi-lo)*0.001f;
        for(j=0U;j<2U;j++) {
            al=p->a[j]+(q->a[j]-p->a[j])*x;ar=p->a[j]+(q->a[j]-p->a[j])*y;
            A=d*(al+ar)*0.5f;B=d*d*(al+2.0f*ar)/6.0f;
            iv[j]+=A;
            if(origin==now)im[j]+=(float)(now-(p->ms+lo))*0.001f*A-B;
            else im[j]+=(float)((p->ms+lo)-origin)*0.001f*A+B;
        }
    }
    return 1U;
}
static uint32_t Obs_Predict(uint32_t now)
{
    uint32_t j,delta,age; float first[2],weighted[2],after[2],position[2],u,C,h;
    if(s_anchors<2U)return ICM_OBS_BAD_WHEEL;
    delta=s_anchor_ms[1]-s_anchor_ms[0];age=now-s_anchor_ms[1];
    if(delta<1U || delta>250U || age>200U)return ICM_OBS_WINDOW;
    if(s_count<2U || (uint32_t)(now-Obs_At(0U)->ms)<(uint32_t)(now-s_anchor_ms[0]) ||
       Obs_At(s_count-1U)->ms!=now)return ICM_OBS_HISTORY;
    (void)Obs_Integrals(s_anchor_ms[0],s_anchor_ms[1],s_anchor_ms[0],now,first,weighted);
    (void)Obs_Integrals(s_anchor_ms[1],now,now,now,after,position);
    h=(float)age*0.001f;
    for(j=0U;j<2U;j++) {
        u=(s_anchor_p[1][j]-s_anchor_p[0][j])/((float)delta*0.001f);
        C=weighted[j]/((float)delta*0.001f);
        first[j]=u+C+after[j]; position[j]=s_anchor_p[1][j]+u*h+h*C+position[j];
        if(!Obs_Finite(first[j]) || !Obs_Finite(position[j]))return ICM_OBS_NUMERIC;
    }
    s_value.position_f_mm=position[0];s_value.position_r_mm=position[1];
    s_value.velocity_f_mm_s=first[0];s_value.velocity_r_mm_s=first[1];
    return 0U;
}
void AppIcmObserver_Step(const IcmRawStatus *raw)
{
    AppOdometryBaseline40 wheel; ChassisAutoStatus motion; PowerCommandStatus power;
    AppHeadingSource yaw; uint32_t i,dt=0U,gap=0U,new_sample,reason=0U,ready;
    float a[3],w[3],c[3],cc[3],g[3],step,angle,cs,sn,dx,dy,p[2];
    ready=ChassisAuto_ReadStatus(&motion);Power_ReadCommandStatus(&power);
    if(ready && motion.active)s_ever_active=1U;
    if(!s_cal_ready && s_ever_active)s_cal_blocked=1U;
    new_sample=!s_seen || raw->sample_sequence!=s_last_seq;
    if(s_seen) {
        if(raw->skipped_edges!=s_last_skipped || raw->read_races!=s_last_races ||
           raw->transfer_errors!=s_last_errors)gap=1U;
        if(new_sample) {
            dt=raw->sample_tick_ms-s_last_ms;
            if(raw->sample_sequence-s_last_seq!=1U || raw->edge_sequence-s_last_edge!=1U ||
               dt<1U || dt>20U)gap=1U;
        }
    }
    s_last_skipped=raw->skipped_edges;s_last_races=raw->read_races;s_last_errors=raw->transfer_errors;
    if(gap) {
        Obs_Clear();s_gaps++;if(s_cal_ready)s_gravity_gap=1U;else Obs_CalClear();
    }
    if(raw->state!=ICM_RAW_READY || raw->last_error!=ICM_RAW_OK || !raw->fresh ||
       !raw->sample_sequence || raw->accel_config!=0x38U || raw->gyro_config!=0x28U) {
        Obs_Clear();s_value.flags=0U;if(!s_cal_ready)Obs_CalClear();
        Obs_Publish(ICM_OBS_RAW | (s_cal_ready?0U:ICM_OBS_WAIT_CAL));return;
    }
    if(!new_sample) {
        if(gap){s_value.flags=0U;Obs_Publish(ICM_OBS_GAP);}
        return;
    }
    s_seen=1U;s_last_seq=raw->sample_sequence;s_last_edge=raw->edge_sequence;
    s_last_ms=raw->sample_tick_ms;
    s_value.sample_sequence=raw->sample_sequence;s_value.estimate_ms=raw->sample_tick_ms;
    s_value.flags=0U;s_value.hwt_age_ms=s_value.wheel_age_ms=0xFFFFFFFFU;
    a[0]=-(float)raw->raw_accel[1]*(OBS_G/8192.0f);
    a[1]=-(float)raw->raw_accel[0]*(OBS_G/8192.0f);
    a[2]=-(float)raw->raw_accel[2]*(OBS_G/8192.0f);
    w[0]=-(float)raw->raw_gyro[1]*(1000.0f/32768.0f)*OBS_RAD;
    w[1]=-(float)raw->raw_gyro[0]*(1000.0f/32768.0f)*OBS_RAD;
    w[2]=-(float)raw->raw_gyro[2]*(1000.0f/32768.0f)*OBS_RAD;
    if(!s_cal_ready) {
        if(!ready || motion.active || power.active || s_cal_blocked)Obs_CalClear();
        else {
            s_accel_sum[0]-=raw->raw_accel[1];s_accel_sum[1]-=raw->raw_accel[0];s_accel_sum[2]-=raw->raw_accel[2];
            s_gyro_sum[0]-=raw->raw_gyro[1];s_gyro_sum[1]-=raw->raw_gyro[0];s_gyro_sum[2]-=raw->raw_gyro[2];
            s_cal_count++;
            if(s_cal_count==200U) {
                for(i=0U;i<3U;i++){s_gravity[i]=(float)s_accel_sum[i]*(OBS_G/1638400.0f);
                    s_gyro_bias[i]=(float)s_gyro_sum[i]*(1000.0f/6553600.0f)*OBS_RAD;}
                if(!Obs_Normalize(s_gravity)){Obs_CalClear();Obs_Publish(ICM_OBS_NUMERIC);return;}
                for(i=0U;i<3U;i++)s_accel_bias[i]=(float)s_accel_sum[i]*(OBS_G/1638400.0f)-s_gravity[i];
                s_cal_ready=1U;dt=0U;
            }
        }
        if(!s_cal_ready){Obs_Publish(ICM_OBS_WAIT_CAL);return;}
    }
    if(dt>=1U && dt<=20U) {
        step=(float)dt*0.001f;
        for(i=0U;i<3U;i++)w[i]-=s_gyro_bias[i];
        Obs_Cross(w,s_gravity,c);Obs_Cross(w,c,cc);
        for(i=0U;i<3U;i++)g[i]=s_gravity[i]-step*c[i]+0.5f*step*step*cc[i];
        if(!Obs_Normalize(g)){Obs_Clear();Obs_Publish(ICM_OBS_NUMERIC);return;}
        memcpy(s_gravity,g,sizeof(g));
    }
    AppOdometry_ReadBaseline40(&wheel);
    s_value.session=motion.session;s_value.reference_ms=wheel.reference_set_ms;
    if(!ready || !wheel.anchor_valid) {
        Obs_Clear();Obs_Publish(ICM_OBS_REFERENCE);return;
    }
    if(!s_frame_seen || motion.session!=s_session || wheel.reference_set_ms!=s_reference ||
       wheel.pose.continuity_lost!=s_continuity) {
        Obs_Clear();s_frame_seen=1U;s_session=motion.session;
        s_reference=wheel.reference_set_ms;s_continuity=wheel.pose.continuity_lost;
        reason|=ICM_OBS_REFERENCE;
    }
    if(!wheel.pose.valid || (int32_t)(wheel.pose.sample_ms-wheel.reference_set_ms)<0) {
        Obs_Clear();Obs_Publish(ICM_OBS_BAD_WHEEL);return;
    }
    if(!App_HeadingSourceRead(&yaw) || !yaw.healthy || !(yaw.valid_mask&1U)) {
        Obs_Clear();Obs_Publish(ICM_OBS_HWT);return;
    }
    if((uint32_t)(raw->sample_tick_ms-yaw.angle_ms)<100U)s_yaw=yaw;
    s_value.hwt_age_ms=raw->sample_tick_ms-s_yaw.angle_ms;
    if(!s_yaw.healthy || !(s_yaw.valid_mask&1U) || s_value.hwt_age_ms>=100U) {
        Obs_Clear();Obs_Publish(ICM_OBS_HWT);return;
    }
    angle=(s_yaw.yaw_deg+wheel.heading_offset_deg-wheel.path_heading_deg)*OBS_RAD;
    cs=cosf(angle);sn=sinf(angle);
    for(i=0U;i<3U;i++)a[i]-=s_accel_bias[i]+s_gravity[i];
    s_value.accel_f_mm_s2=cs*a[0]-sn*a[1];
    s_value.accel_r_mm_s2=-sn*a[0]-cs*a[1];
    angle=wheel.path_heading_deg*OBS_RAD;cs=cosf(angle);sn=sinf(angle);
    dx=wheel.pose.x_mm-wheel.origin_x_mm;dy=wheel.pose.y_mm-wheel.origin_y_mm;
    p[0]=cs*dx+sn*dy;p[1]=sn*dx-cs*dy;
    if(!Obs_Finite(p[0]) || !Obs_Finite(p[1]) || !Obs_Finite(s_value.accel_f_mm_s2) ||
       !Obs_Finite(s_value.accel_r_mm_s2)) {
        Obs_Clear();Obs_Publish(ICM_OBS_NUMERIC);return;
    }
    s_value.flags=ICM_OBS_ACCEL;
    Obs_Append(raw->sample_tick_ms,s_value.accel_f_mm_s2,s_value.accel_r_mm_s2);
    if((int32_t)(raw->sample_tick_ms-wheel.pose.sample_ms)<0)reason=ICM_OBS_BAD_WHEEL;
    else if(!s_anchors || wheel.pose.sample_ms!=s_anchor_ms[s_anchors-1U]) {
        if(s_anchors && (int32_t)(wheel.pose.sample_ms-s_anchor_ms[s_anchors-1U])<=0) {
            Obs_Clear();reason=ICM_OBS_BAD_WHEEL;
        } else {
            if(s_anchors==2U){s_anchor_ms[0]=s_anchor_ms[1];memcpy(s_anchor_p[0],s_anchor_p[1],sizeof(p));}
            else s_anchors++;
            s_anchor_ms[s_anchors-1U]=wheel.pose.sample_ms;
            memcpy(s_anchor_p[s_anchors-1U],p,sizeof(p));
        }
    }
    if(s_anchors) {
        s_value.flags|=ICM_OBS_WHEEL;s_value.t0_ms=s_anchor_ms[0];
        s_value.t1_ms=s_anchor_ms[s_anchors-1U];s_value.wheel_age_ms=raw->sample_tick_ms-s_value.t1_ms;
        s_value.wheel_f_mm=s_anchor_p[s_anchors-1U][0];s_value.wheel_r_mm=s_anchor_p[s_anchors-1U][1];
    }
    if(gap)reason|=ICM_OBS_GAP;
    if(!reason && (!s_predicted || (uint32_t)(raw->sample_tick_ms-s_last_predict)>=20U)) {
        s_predicted=1U;s_last_predict=raw->sample_tick_ms;reason=Obs_Predict(raw->sample_tick_ms);
        Obs_Publish(reason);
    } else if(reason)Obs_Publish(reason);
}
#endif

uint16_t App_SerialCrc(const char *data, uint32_t length)
{
    uint16_t crc = 0xFFFFU;
    uint32_t i, bit;
    for (i = 0U; i < length; i++)
    {
        crc ^= (uint16_t)((uint16_t)(uint8_t)data[i] << 8);
        for (bit = 0U; bit < 8U; bit++)
            crc = (uint16_t)((crc & 0x8000U) ? ((crc << 1) ^ 0x1021U) : (crc << 1));
    }
    return crc;
}
static uint32_t App_SerialStatusDue(uint32_t now)
{
    return !s_serial_tx_valid ||
        (uint32_t)(now - s_serial_tx_tick) >= APP_SERIAL_STATUS_PERIOD_MS;
}
uint32_t App_SerialDiagnosticWindow(uint32_t now, uint32_t earliest_ms)
{
    return s_serial_tx_valid && !App_SerialStatusDue(now) &&
        (uint32_t)(now - s_serial_tx_tick) >= earliest_ms;
}
static int32_t App_ObserverMilli(float value)
{
    if(!(value>=-2000000.0f && value<=2000000.0f))return (-2147483647L-1L);
    return (int32_t)(value*1000.0f);
}
static void App_IcmObserverTelemetry(void)
{
    static char frame[384];static uint32_t last,sent,sequence,dropped;
    AppIcmObserverStatus o; int32_t values[27];
    uint32_t now=(uint32_t)xTaskGetTickCount(),phase=now-s_serial_tx_tick,i,used=4U;
    int n;uint16_t crc;
    if((sent && (uint32_t)(now-last)<500U) || !App_SerialDiagnosticWindow(now,40U) || phase>55U)return;
    last=now;sent=1U; /* Attempt budget, including a busy transmitter. */
    AppIcmObserver_Read(&o);
    values[0]=(int32_t)sequence++;values[1]=(int32_t)now;values[2]=1;
    values[3]=(int32_t)o.sequence;values[4]=(int32_t)o.session;
    values[5]=(int32_t)o.valid;values[6]=(int32_t)o.reasons;values[7]=(int32_t)o.flags;
    values[8]=(int32_t)o.estimate_ms;values[9]=(int32_t)o.sample_sequence;
    values[10]=(int32_t)o.reference_ms;values[11]=(int32_t)o.t0_ms;values[12]=(int32_t)o.t1_ms;
    values[13]=(int32_t)o.wheel_age_ms;values[14]=(int32_t)o.hwt_age_ms;
    values[15]=(int32_t)o.cal_samples;values[16]=(int32_t)o.history_samples;
    values[17]=(int32_t)o.gaps;
    for(i=18U;i<26U;i++)values[i]=(-2147483647L-1L);
    if(o.flags&ICM_OBS_WHEEL){values[18]=App_ObserverMilli(o.wheel_f_mm);values[19]=App_ObserverMilli(o.wheel_r_mm);}
    if(o.valid){values[20]=App_ObserverMilli(o.position_f_mm);values[21]=App_ObserverMilli(o.position_r_mm);
        values[22]=App_ObserverMilli(o.velocity_f_mm_s);values[23]=App_ObserverMilli(o.velocity_r_mm_s);}
    if(o.flags&ICM_OBS_ACCEL){values[24]=App_ObserverMilli(o.accel_f_mm_s2);values[25]=App_ObserverMilli(o.accel_r_mm_s2);}
    values[26]=(int32_t)dropped;memcpy(frame,"IO1,",4U);
    for(i=0U;i<27U;i++) {
        if(i<18U || i==26U)n=snprintf(frame+used,sizeof(frame)-used,"%lu,",(unsigned long)(uint32_t)values[i]);
        else n=snprintf(frame+used,sizeof(frame)-used,"%ld,",(long)values[i]);
        if(n<0 || (uint32_t)n>=sizeof(frame)-used){dropped++;return;}used+=(uint32_t)n;
    }
    crc=App_SerialCrc(frame,used);n=snprintf(frame+used,sizeof(frame)-used,"%04X\r\n",crc);
    if(n<0 || (uint32_t)n>=sizeof(frame)-used || !App_DebugSendString(frame)){dropped++;return;}
}

volatile uint32_t cases,assertions;
#define CHECK(x) do{assertions++;if(!(x))return __LINE__;}while(0)
#define ZERO(x) memset(&(x),0,sizeof(x))
static void reset_observer(void){
 ZERO(s_history);s_count=s_head=s_anchors=0U;ZERO(s_anchor_ms);ZERO(s_anchor_p);
 ZERO(s_gravity);ZERO(s_accel_bias);ZERO(s_gyro_bias);ZERO(s_accel_sum);ZERO(s_gyro_sum);
 s_cal_count=s_cal_ready=s_cal_blocked=s_ever_active=0U;
 s_seen=s_last_seq=s_last_edge=s_last_ms=s_last_skipped=s_last_races=s_last_errors=0U;
 s_session=s_reference=s_continuity=s_frame_seen=s_last_predict=s_predicted=s_gaps=s_gravity_gap=0U;
 ZERO(s_yaw);ZERO(s_value);ZERO(s_published);ZERO(tw);ZERO(tm);ZERO(tp);ZERO(ty);
 tw.anchor_valid=tw.pose.valid=1U;ty.healthy=1U;ty.valid_mask=3U;
}
static void sample(IcmRawStatus *raw,uint32_t advance){
 raw->sample_sequence+=advance;raw->edge_sequence+=advance;raw->sample_tick_ms+=5U;
 ty.sample_ms=ty.angle_ms=ty.rate_ms=raw->sample_tick_ms;
 tw.pose.sample_ms=raw->sample_tick_ms;
 AppIcmObserver_Step(raw);
}
static IcmRawStatus raw_status(void){
 IcmRawStatus raw;ZERO(raw);raw.state=ICM_RAW_READY;raw.fresh=1U;
 raw.accel_config=0x38U;raw.gyro_config=0x28U;raw.raw_accel[2]=-8192;
 return raw;
}
static float pfunc(float t,float a,float j){return 80.0f*t+0.5f*a*t*t+j*t*t*t/6.0f;}
static float vfunc(float t,float a,float j){return 80.0f+a*t+0.5f*j*t*t;}
static uint32_t kernel_tests(void){
 uint32_t b,d,h,k,j;const uint32_t deltas[4]={1U,22U,100U,200U},ages[3]={0U,53U,150U};
 const uint32_t bases[2]={1000U,0xFFFFFF00U};float aa,jj,t,t0,t1,now,p0,p1,vbound,pbound,expect;
 for(b=0;b<2;b++)for(d=0;d<4;d++)for(h=0;h<3;h++)for(j=0;j<3;j++) {
  reset_observer();aa=(float)((int)j-1)*150.0f;jj=(float)((int)j-1)*20.0f;
  for(k=0;k<128;k++){t=(float)(k*5U)*0.001f;Obs_Append(bases[b]+k*5U,aa+jj*t,-aa-jj*t);}
  s_anchors=2U;s_anchor_ms[1]=bases[b]+635U-ages[h];s_anchor_ms[0]=s_anchor_ms[1]-deltas[d];
  t0=(float)(s_anchor_ms[0]-bases[b])*0.001f;t1=(float)(s_anchor_ms[1]-bases[b])*0.001f;now=0.635f;
  p0=pfunc(t0,aa,jj);p1=pfunc(t1,aa,jj);
  s_anchor_p[0][0]=p0;s_anchor_p[1][0]=p1;
  s_anchor_p[0][1]=-p0;s_anchor_p[1][1]=-p1;
  CHECK(Obs_Predict(bases[b]+635U)==0U);
  vbound=0.0001f+8.0f*FLT_EPSILON*(fabsf(p0)+fabsf(p1))/((float)deltas[d]*0.001f);
  pbound=0.0001f+(float)ages[h]*0.001f*vbound;
  expect=vfunc(now,aa,jj);CHECK(fabsf(s_value.velocity_f_mm_s-expect)<vbound);
  CHECK(fabsf(s_value.velocity_r_mm_s+expect)<vbound);
  expect=pfunc(now,aa,jj);CHECK(fabsf(s_value.position_f_mm-expect)<pbound);
  CHECK(fabsf(s_value.position_r_mm+expect)<pbound);cases++;
 }
 s_anchor_ms[0]=s_anchor_ms[1];CHECK(Obs_Predict(bases[1]+635U)==ICM_OBS_WINDOW);cases++;
 s_anchor_ms[0]=s_anchor_ms[1]-22U;s_count=1U;CHECK(Obs_Predict(bases[1]+635U)==ICM_OBS_HISTORY);cases++;
 reset_observer();for(k=0;k<128;k++)Obs_Append(k*5U,0.0f,0.0f);
 s_anchors=2U;s_anchor_ms[0]=500U;s_anchor_ms[1]=600U;
 CHECK(Obs_Predict(635U)==0U && s_value.position_f_mm==0.0f && s_value.velocity_r_mm_s==0.0f);cases++;
 s_anchor_ms[0]=185U;s_anchor_ms[1]=435U;
 CHECK(Obs_Predict(635U)==0U);cases++;
 s_anchor_ms[0]=184U;CHECK(Obs_Predict(635U)==ICM_OBS_WINDOW);cases++;
 s_anchor_ms[0]=185U;CHECK(Obs_Predict(636U)==ICM_OBS_WINDOW);cases++;
 return 0U;
}
static uint32_t lifecycle_tests(void){
 IcmRawStatus raw;AppIcmObserverStatus o;uint32_t k,seq,head;
 reset_observer();raw=raw_status();raw.sample_tick_ms=0xFFFFFC00U;
 tw.reference_set_ms=raw.sample_tick_ms;
 for(k=0;k<200;k++)sample(&raw,1U);
 CHECK(s_cal_ready && s_cal_count==200U);CHECK(fabsf(s_accel_bias[2])<0.01f);
 for(k=0;k<12;k++)sample(&raw,1U);
 AppIcmObserver_Read(&o);CHECK(o.valid && o.reasons==0U);CHECK(o.position_f_mm==0.0f && o.position_r_mm==0.0f);
 head=s_head;seq=s_published.sequence;AppIcmObserver_Step(&raw);
 CHECK(head==s_head && seq==s_published.sequence);cases++;
 tm.session++;sample(&raw,1U);CHECK(s_count==1U && s_anchors==1U && !s_published.valid);cases++;
 tw.reference_set_ms=raw.sample_tick_ms+5U;sample(&raw,1U);CHECK(s_count==1U && s_anchors==1U);cases++;
 for(k=0;k<8;k++)sample(&raw,1U);
 raw.skipped_edges++;sample(&raw,2U);CHECK(!s_published.valid && (s_published.reasons&ICM_OBS_GAP));
 CHECK(s_count==1U && s_anchors==1U && s_gravity_gap);cases++;
 for(k=0;k<8;k++)sample(&raw,1U);CHECK(s_published.valid && (s_published.flags&ICM_OBS_GRAVITY_GAP));cases++;
 ty.angle_ms=raw.sample_tick_ms+50U;ty.yaw_deg=90.0f;
 raw.sample_tick_ms+=5U;raw.sample_sequence++;raw.edge_sequence++;tw.pose.sample_ms=raw.sample_tick_ms;
 AppIcmObserver_Step(&raw);CHECK(s_yaw.yaw_deg==0.0f);cases++;
 ty.healthy=0U;raw.sample_tick_ms+=5U;raw.sample_sequence++;raw.edge_sequence++;
 AppIcmObserver_Step(&raw);CHECK(!s_published.valid && s_count==0U && s_anchors==0U && s_published.reasons==ICM_OBS_HWT);cases++;
 ty.healthy=1U;for(k=0;k<8;k++)sample(&raw,1U);CHECK(s_published.valid);cases++;
 raw.fresh=0U;AppIcmObserver_Step(&raw);CHECK(!s_published.valid && s_published.reasons&ICM_OBS_RAW);cases++;
 reset_observer();raw=raw_status();for(k=0;k<10;k++)sample(&raw,1U);
 tm.active=1U;sample(&raw,1U);tm.active=0U;for(k=0;k<210;k++)sample(&raw,1U);
 CHECK(s_cal_blocked && !s_cal_ready && !s_cal_count && s_published.reasons==ICM_OBS_WAIT_CAL);cases++;
 reset_observer();raw=raw_status();raw.raw_accel[2]=-9000;
 raw.raw_gyro[0]=11;raw.raw_gyro[1]=-7;raw.raw_gyro[2]=5;
 for(k=0;k<212;k++)sample(&raw,1U);
 CHECK(s_cal_ready && s_published.valid && s_accel_bias[2]>900.0f);
 CHECK(fabsf(s_published.accel_f_mm_s2)<0.001f && fabsf(s_published.accel_r_mm_s2)<0.001f);
 CHECK(fabsf(s_gravity[0])<0.001f && fabsf(s_gravity[1])<0.001f);cases++;
 tw.pose.continuity_lost++;sample(&raw,1U);CHECK(!s_published.valid && s_published.reasons&ICM_OBS_REFERENCE);cases++;
 for(k=0;k<8;k++)sample(&raw,1U);CHECK(s_published.valid);cases++;
 tm.state=CHASSIS_AUTO_DONE;tm.active=0U;
 for(k=0;k<45;k++){
  raw.sample_tick_ms+=5U;raw.sample_sequence++;raw.edge_sequence++;
  ty.angle_ms=raw.sample_tick_ms;AppIcmObserver_Step(&raw);
 }
 CHECK(!s_published.valid && s_published.reasons&ICM_OBS_WINDOW);cases++;
 tw.pose.valid=0U;sample(&raw,1U);CHECK(!s_count && !s_anchors && !s_published.valid);cases++;

 return 0U;
}
static uint32_t wire_tests(void){
 uint32_t i,before;
 ZERO(s_published);s_published.reasons=ICM_OBS_WAIT_CAL;s_published.flags=ICM_OBS_NO_LEVER;
 s_published.wheel_age_ms=s_published.hwt_age_ms=0xFFFFFFFFU;
 s_serial_tx_tick=0U;clock_ms=40U;tx_busy=1U;before=tx_calls;App_IcmObserverTelemetry();
 for(i=41U;i<=55U;i++){clock_ms=i;App_IcmObserverTelemetry();}
 clock_ms=140U;s_serial_tx_tick=100U;App_IcmObserverTelemetry();
 CHECK(tx_calls==before+1U);cases++;
 tx_busy=0U;clock_ms=540U;s_serial_tx_tick=500U;App_IcmObserverTelemetry();
 CHECK(wire_count==1U);cases++;
 s_published.valid=1U;s_published.reasons=0U;s_published.flags=ICM_OBS_CAL|ICM_OBS_WHEEL|ICM_OBS_ACCEL|ICM_OBS_NO_LEVER;
 s_published.estimate_ms=0xFFFFFFF0U;s_published.t1_ms=0xFFFFFFD0U;s_published.t0_ms=0xFFFFFFBAU;
 s_published.wheel_age_ms=32U;s_published.hwt_age_ms=20U;s_published.cal_samples=200U;s_published.history_samples=128U;
 s_published.wheel_f_mm=s_published.position_f_mm=2100.0f;
 clock_ms=0xFFFFFFF0U;s_serial_tx_tick=clock_ms-40U;App_IcmObserverTelemetry();CHECK(wire_count==2U);cases++;
 s_published.estimate_ms=40U;s_published.t1_ms=8U;s_published.t0_ms=0xFFFFFFF2U;
 clock_ms=540U;s_serial_tx_tick=500U;App_IcmObserverTelemetry();CHECK(wire_count==3U);cases++;
 CHECK(App_ObserverMilli(2100.0f)==2100000L);CHECK(App_ObserverMilli(-2100.0f)==-2100000L);
 CHECK(locks==0U);cases++;
 return 0U;
}
__attribute__((section("i.test_entry"))) uint32_t test_entry(void){
 uint32_t r;r=kernel_tests();if(r)return r;r=lifecycle_tests();if(r)return r;
 return wire_tests();
}
