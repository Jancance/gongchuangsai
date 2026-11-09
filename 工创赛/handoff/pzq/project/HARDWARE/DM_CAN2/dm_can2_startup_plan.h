#ifndef DM_CAN2_STARTUP_PLAN_H
#define DM_CAN2_STARTUP_PLAN_H
#include <stdint.h>
#include <math.h>
#ifndef VEHICLE_DM_LEFTUP_SHORT_RETURN
#define VEHICLE_DM_LEFTUP_SHORT_RETURN 0U
#endif
/* Clockwise return in the established 0..190 degree branch. Optional
 * measured left-up branch is bounded to 330..357 (3..30 degrees to zero). */
static uint32_t DM_CAN2_StartupReturnDelta(float raw_deg,float zero_deg,uint32_t leftup,float *delta_deg)
{
 float work;
 if(!delta_deg || !(raw_deg>=-720 && raw_deg<=720) || !(zero_deg>=-720 && zero_deg<=720))return 0;
 work=fmodf(raw_deg-zero_deg,360);if(work<0)work+=360;
 if(work<=3 || work>=357){*delta_deg=0;return 1;}
 if(work<=190){*delta_deg=-work;return 1;}
 if(leftup && work>=330){*delta_deg=360-work;return 1;}
 return 0;
}
static uint32_t DM_CAN2_StartupCwDelta(float raw_deg,float zero_deg,float *delta_deg)
{return DM_CAN2_StartupReturnDelta(raw_deg,zero_deg,VEHICLE_DM_LEFTUP_SHORT_RETURN,delta_deg);}
#endif
