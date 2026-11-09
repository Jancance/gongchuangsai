#ifndef DM_CAN2_STARTUP_PLAN_H
#define DM_CAN2_STARTUP_PLAN_H
#include <stdint.h>
#include <math.h>
/* 新鲜原始角与既有显示基准换算；不设置电机零点，不恢复多圈机械参考。
 * 从本次实际手摆收拢位置逆时针正向到同圈360度（显示0）。
 * 不把车左上角猜成固定角度；已近零不绕圈，保留最多190度行程检查。 */
static uint32_t DM_CAN2_StartupCcwDelta(float raw_deg,float zero_deg,float *delta_deg)
{
    float work;
    if(!delta_deg || !(raw_deg>=-720.0f && raw_deg<=720.0f) ||
       !(zero_deg>=-720.0f && zero_deg<=720.0f))return 0U;
    work=fmodf(raw_deg-zero_deg,360.0f);if(work<0.0f)work+=360.0f;
    if(work<=3.0f || work>=357.0f){*delta_deg=0.0f;return 1U;}
    /* 190度沿用原允许最大正向行程；不是机械扫掠已通过的证明。 */
    if(360.0f-work>190.0f)return 0U;
    *delta_deg=360.0f-work;return 1U;
}
#endif
