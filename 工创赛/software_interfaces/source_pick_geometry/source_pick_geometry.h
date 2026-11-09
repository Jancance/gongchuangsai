#ifndef SOURCE_PICK_GEOMETRY_H
#define SOURCE_PICK_GEOMETRY_H
#include <math.h>
#include <stdint.h>
#include <string.h>
/* MATLAB验证后的离线换算/观测检查候选。无CAN/UART/PWM/启动或到位DONE接口。 */
enum { SPG_OK=0,SPG_ARGUMENT=1,SPG_STROKE=2,SPG_ANGLE=3,SPG_MARGIN=4 };
typedef struct {
 float zero_radius_mm,max_extension_mm,source_reference_deg,min_angle_deg,max_angle_deg,min_stroke_margin_mm;
} SourcePickGeometry;
typedef struct {
 float extension_mm,logical_deg,minimum_stroke_margin_mm;
 uint32_t geometry_in_range,requested_margin_met,motion_allowed;
} SourcePickTarget;
typedef struct {
 float angle_deg,extension_mm,angle_speed_deg_s,extension_speed_mm_s,max_joint_error_mm;
 uint32_t max_feedback_age_ms;
} SourcePickTolerance;
static __inline uint32_t SPG_Finite(float v)
{return v>=-1000000.0f && v<=1000000.0f;}
static __inline float SPG_AngleError(float angle,float desired)
{
 float delta=fmodf(angle-desired+180.0f,360.0f);
 if(delta<0.0f)delta+=360.0f;
 return delta-180.0f;
}
static __inline uint32_t SPG_ValidGeometry(const SourcePickGeometry *g)
{
 return g && SPG_Finite(g->zero_radius_mm) && g->zero_radius_mm>0.0f &&
  SPG_Finite(g->max_extension_mm) && g->max_extension_mm>0.0f &&
  SPG_Finite(g->source_reference_deg) && g->source_reference_deg>=0 && g->source_reference_deg<360 &&
  SPG_Finite(g->min_angle_deg) && SPG_Finite(g->max_angle_deg) &&
  g->min_angle_deg>=0 && g->max_angle_deg<=360 && g->max_angle_deg>g->min_angle_deg &&
  SPG_Finite(g->min_stroke_margin_mm) && g->min_stroke_margin_mm>=0 && g->min_stroke_margin_mm*2<=g->max_extension_mm;
}
static __inline uint32_t SPG_ValidTarget(const SourcePickTarget *t)
{
 return t && t->geometry_in_range==1U && SPG_Finite(t->extension_mm) && t->extension_mm>=0 &&
  SPG_Finite(t->logical_deg) && t->logical_deg>=0 && t->logical_deg<360 && t->motion_allowed==0U;
}
static __inline uint32_t SourcePick_Solve(const SourcePickGeometry *g,
 float target_right_mm,float target_front_mm,float shift_right_mm,float shift_front_mm,
 float chassis_yaw_deg,SourcePickTarget *out)
{
 float x,y,c,s,r,angle;
 if(!out)return SPG_ARGUMENT;
 memset(out,0,sizeof(*out));
 if(!SPG_ValidGeometry(g) ||
  !SPG_Finite(target_right_mm) || !SPG_Finite(target_front_mm) || !SPG_Finite(shift_right_mm) ||
  !SPG_Finite(shift_front_mm) || !SPG_Finite(chassis_yaw_deg))return SPG_ARGUMENT;
 c=cosf(chassis_yaw_deg*0.01745329252f);s=sinf(chassis_yaw_deg*0.01745329252f);
 x=(target_right_mm-shift_right_mm)*c+(target_front_mm-shift_front_mm)*s;
 y=-(target_right_mm-shift_right_mm)*s+(target_front_mm-shift_front_mm)*c;
 r=hypotf(x,y);
 if(!(r>0.0f))return SPG_ARGUMENT;
 angle=fmodf(g->source_reference_deg+atan2f(y,x)*57.29577951f,360.0f);
 if(angle<0.0f)angle+=360.0f;
 out->extension_mm=r-g->zero_radius_mm;out->logical_deg=angle;
 out->minimum_stroke_margin_mm=fminf(out->extension_mm,g->max_extension_mm-out->extension_mm);
 if(!(out->extension_mm>=0.0f && out->extension_mm<=g->max_extension_mm))return SPG_STROKE;
 if(!(angle>=g->min_angle_deg && angle<=g->max_angle_deg))return SPG_ANGLE;
 out->geometry_in_range=1U;
 if(out->minimum_stroke_margin_mm<g->min_stroke_margin_mm)return SPG_MARGIN;
 out->requested_margin_met=1U;
 return SPG_OK;
}
static __inline uint32_t SourcePick_FirmwareSupports(const SourcePickTarget *target,
 float firmware_max_angle_deg,float firmware_max_extension_mm)
{
 return SPG_ValidTarget(target) && SPG_Finite(firmware_max_angle_deg) &&
  SPG_Finite(firmware_max_extension_mm) && firmware_max_angle_deg>0 && firmware_max_angle_deg<=360 &&
  firmware_max_extension_mm>0 && target->logical_deg<=firmware_max_angle_deg &&
  target->extension_mm<=firmware_max_extension_mm;
}
/* 仅检查单个反馈样本；不累计成功、不授予动作许可、不代替事务/接触/载台确认。 */
static __inline uint32_t SourcePick_SampleWithinTolerance(const SourcePickGeometry *g,
 const SourcePickTarget *target,const SourcePickTolerance *tol,
 float angle_deg,float extension_mm,float angle_speed_deg_s,float extension_speed_mm_s,
 uint32_t now_ms,uint32_t dm_tick_ms,uint32_t extension_tick_ms,uint32_t reference_final,
 float *joint_error_upper_mm)
{
 float delta,chord,error;
 if(joint_error_upper_mm)*joint_error_upper_mm=0.0f;
 if(!SPG_ValidGeometry(g) || !SPG_ValidTarget(target) || !tol || !joint_error_upper_mm ||
  target->extension_mm>g->max_extension_mm || target->logical_deg<g->min_angle_deg ||
  target->logical_deg>g->max_angle_deg || reference_final!=1U ||
  !SPG_Finite(angle_deg) || !SPG_Finite(extension_mm) || !SPG_Finite(angle_speed_deg_s) || !SPG_Finite(extension_speed_mm_s) ||
  !SPG_Finite(tol->angle_deg) || !(tol->angle_deg>0 && tol->angle_deg<=3) ||
  !SPG_Finite(tol->extension_mm) || !(tol->extension_mm>0 && tol->extension_mm<=3) ||
  !SPG_Finite(tol->angle_speed_deg_s) || !(tol->angle_speed_deg_s>0) ||
  !SPG_Finite(tol->extension_speed_mm_s) || !(tol->extension_speed_mm_s>0) ||
  !SPG_Finite(tol->max_joint_error_mm) || !(tol->max_joint_error_mm>0) ||
  tol->max_feedback_age_ms<1 || tol->max_feedback_age_ms>100 ||
  (uint32_t)(now_ms-dm_tick_ms)>=tol->max_feedback_age_ms ||
  (uint32_t)(now_ms-extension_tick_ms)>=tol->max_feedback_age_ms ||
  !(extension_mm>=0 && extension_mm<=g->max_extension_mm))return 0U;
 delta=fabsf(SPG_AngleError(angle_deg,target->logical_deg));
 chord=2.0f*(g->zero_radius_mm+target->extension_mm)*sinf(delta*0.00872664626f);
 error=chord+fabsf(extension_mm-target->extension_mm);
 *joint_error_upper_mm=error;
 return delta<=tol->angle_deg && fabsf(extension_mm-target->extension_mm)<=tol->extension_mm &&
  fabsf(angle_speed_deg_s)<=tol->angle_speed_deg_s && fabsf(extension_speed_mm_s)<=tol->extension_speed_mm_s &&
  error<=tol->max_joint_error_mm;
}
#endif
