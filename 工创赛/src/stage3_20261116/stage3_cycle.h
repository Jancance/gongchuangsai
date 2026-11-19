#ifndef STAGE3_CYCLE_H
#define STAGE3_CYCLE_H
#include <stdint.h>
/* Bench task object, not a fabricated four-part competition QR code. */
extern volatile uint32_t g_stage3_request,g_stage3_state,g_stage3_fault;
extern volatile uint32_t g_stage3_run,g_stage3_pose,g_stage3_item,g_stage3_phase;
extern volatile uint32_t g_stage3_align_done,g_stage3_config_ready;
extern volatile uint32_t g_stage3_colors[3],g_stage3_positions[3];
extern volatile float g_stage3_carrier_yaw,g_stage3_pick_depth;
extern volatile float g_stage3_target_yaw[3],g_stage3_target_x[3],g_stage3_target_depth[3];
extern volatile uint32_t g_stage3_test_first_item,g_stage3_test_count;
extern volatile uint32_t g_stage3_teach_request,g_stage3_teach_result;
extern volatile float g_stage3_actual_yaw,g_stage3_actual_x,g_stage3_actual_depth,g_stage3_actual_z_mm;
uint32_t Stage3_Owns(void);
uint32_t Stage3_FineTurnAllowed(void);
void Stage3_KeyEvent(void);
void Stage3_Arrive(void);
#endif
