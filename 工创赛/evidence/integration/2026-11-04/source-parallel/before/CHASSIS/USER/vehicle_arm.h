#ifndef VEHICLE_ARM_H
#define VEHICLE_ARM_H
#include "../../USER/vehicle_parameters.h"
extern volatile uint32_t g_vehicle_arm_state,g_vehicle_arm_cause,g_vehicle_arm_dm_owned,g_vehicle_arm_axis_owned;
extern volatile float g_vehicle_lift_current_deg;
extern volatile uint32_t g_vehicle_lift_feedback_valid;
extern volatile uint32_t g_vehicle_arm_stop_pending,g_vehicle_arm_stop_result;
uint32_t VehicleArm_Begin(const VehicleArmRow *row,uint32_t item);
void VehicleArm_Service(void);
void VehicleArm_Cancel(void);
uint32_t VehicleArm_Busy(void);
uint32_t VehicleArm_Exchange(uint8_t address,const uint8_t *q,uint8_t n,uint8_t reply_n,void *out);
#endif
