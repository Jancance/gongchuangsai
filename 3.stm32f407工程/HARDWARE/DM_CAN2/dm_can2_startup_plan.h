#ifndef DM_CAN2_STARTUP_PLAN_H
#define DM_CAN2_STARTUP_PLAN_H
#include <stdint.h>
#include <math.h>
/* 新鲜原始角与既有显示基准换算；不设置电机零点，不恢复多圈机械参考。
 * 人工约180度出发，经逆时针正向半圈到同圈360度（显示0）。
 * 已在零位容差内不额外绕一整圈，其他起始姿态拒绝。 */
static uint32_t DM_CAN2_StartupCcwDelta(float raw_deg,float zero_deg,float *delta_deg)
{
    float work;
    if(!delta_deg || !(raw_deg>=-720.0f && raw_deg<=720.0f) ||
       !(zero_deg>=-720.0f && zero_deg<=720.0f))return 0U;
    work=fmodf(raw_deg-zero_deg,360.0f);if(work<0.0f)work+=360.0f;
    if(work<=3.0f || work>=357.0f){*delta_deg=0.0f;return 1U;}
    if(!(work>=170.0f && work<=190.0f))return 0U;
    *delta_deg=360.0f-work;return 1U;
}
#endif
