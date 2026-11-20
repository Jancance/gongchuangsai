#include "app_entry.h"
#include "motion_trial40.h"
uint32_t AppDebugPlan40Allowed(uint32_t id,uint32_t direction,uint32_t rpm,float mm)
{
    return MotionTrial40Allowed(direction,rpm,mm) &&
        ((((id==DEBUG_PLAN_FORWARD210_40 && direction==0U) ||
           (id==DEBUG_PLAN_LEFT210_40 && direction==1U) ||
           (id==DEBUG_PLAN_RIGHT210_40 && direction==2U) ||
           (id==DEBUG_PLAN_BACKWARD210_40 && direction==3U)) && mm==APP_BASELINE40_MM) ||
         (id==DEBUG_PLAN_RIGHT150_40 && direction==2U && mm==1500.0f) ||
         (id==DEBUG_PLAN_RIGHT30_40 && direction==2U && mm==300.0f) ||
         (id==DEBUG_PLAN_FORWARD150_40 && direction==0U && mm==1500.0f) ||
         (id==DEBUG_PLAN_FORWARD30_40 && direction==0U && mm==300.0f) ||
         (id==DEBUG_PLAN_BACKWARD150_40 && direction==3U && mm==1500.0f) ||
         (id==DEBUG_PLAN_BACKWARD30_40 && direction==3U && mm==300.0f) ||
         (id==DEBUG_PLAN_LEFT150_40 && direction==1U && mm==1500.0f) ||
         (id==DEBUG_PLAN_LEFT30_40 && direction==1U && mm==300.0f));
}
