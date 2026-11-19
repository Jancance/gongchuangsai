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
extern volatile uint32_t g_stage3_teach_missing,g_stage3_teach_dm_source,g_stage3_teach_z_source;
extern volatile float g_stage3_teach_measured_x;
extern volatile float g_stage3_actual_yaw,g_stage3_actual_x,g_stage3_actual_depth,g_stage3_actual_z_mm;
uint32_t Stage3_Owns(void);
uint32_t Stage3_FineTurnAllowed(void);
void Stage3_KeyEvent(void);
void Stage3_Arrive(void);
typedef struct {uint32_t run_id,batch_id,colors[3],positions[3];} Stage3_Task;
/* Existing mission/CAN owner only. Bind first, then call Stage3_Arrive. */
uint32_t Stage3_BindMission(const Stage3_Task *task,const uint32_t slot_colors[3]);
extern volatile uint32_t g_stage3_slot_colors[3],g_stage3_task_run,g_stage3_task_batch;
extern volatile uint32_t g_stage3_teach_sequence,g_stage3_teach_saved_sequence;
#endif
