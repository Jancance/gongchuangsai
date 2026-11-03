#ifndef VEHICLE_MISSION_H
#define VEHICLE_MISSION_H
#include <stdint.h>
/* 由未来真实观测适配器提交，不从Watch或执行器ACK构造物料成功。 */
uint32_t VehicleMission_Observe(uint32_t run,uint32_t stage,uint32_t item,uint32_t row,uint32_t kind);
extern volatile uint32_t g_vehicle_stage,g_vehicle_state,g_vehicle_cause;
#endif
