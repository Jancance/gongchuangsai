#include "app_icm_observer.h"
#if APP_BASELINE40_ACTIVE
#include "FreeRTOS.h"
#include "task.h"
#include "app_odometry.h"
#include "chassis_auto_test.h"
#include "heading_source.h"
#include "power.h"
#include <math.h>
#include <string.h>

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
    AppOdometry_ReadNavigation40(&wheel);
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
