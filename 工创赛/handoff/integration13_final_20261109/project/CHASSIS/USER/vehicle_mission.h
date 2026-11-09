#ifndef VEHICLE_MISSION_H
#define VEHICLE_MISSION_H
#include <stdint.h>
/* 由未来真实观测适配器提交，不从Watch或执行器ACK构造物料成功。 */
uint32_t VehicleMission_Observe(uint32_t run,uint32_t stage,uint32_t item,uint32_t row,uint32_t kind);
extern volatile uint32_t g_vehicle_stage,g_vehicle_state,g_vehicle_cause;
/* Alignment state: 0 not reached, 1 awaiting calibrated metric vision,
 * 3 invalidated by fault/cancel. No DONE state or position executor yet.
 * Project frame: x right/y forward in mm, zero yaw is nominal +Y.
 */
extern volatile uint32_t g_vehicle_alignment_state;
extern volatile uint32_t g_vehicle_scan_search_state;
typedef struct {
    uint32_t run_id,stage,start_id,layout_version,goal_cell;
    float nominal_target_x_mm,nominal_target_y_mm;
    uint32_t position_executor_enabled;
} VehicleAlignmentRequest;
uint32_t VehicleMission_ReadAlignmentRequest(VehicleAlignmentRequest *request);
/* Read-only latched QR task for source stages 2/5, never Watch defaults. */
uint32_t VehicleMission_ReadSourceTask(char code[16],uint32_t *run,uint32_t *batch);
#endif
