#ifndef STAGE3_CYCLE_H
#define STAGE3_CYCLE_H
#include <stdint.h>
#define S3_STRICT_NORM_UM 2000U
#define S3_STRICT_YAW_MDEG 1000L
#define S3_STRICT_YAW_DEG 1.0f
#define S3_STRICT_TRANSLATE_MIN_MM 0.25f
#define S3_STRICT_TRANSLATE_ARRIVAL_MM 0.1f
#define S3_STRICT_HEADING_ARRIVAL_DEG 0.25f
uint32_t Stage3_FineMotionOwned(uint32_t request);
#define S3_FAST_HANDOFF_OFFSET_PULSES 8000.0f
#define S3_FAST_ROTATE_MAX_DEPTH_PULSES 8000.0f
extern volatile uint32_t g_stage3_fast_align_tick,g_stage3_fast_place_tick,g_stage3_fast_recover_tick;
extern volatile uint32_t g_stage3_fast_timing_mask;
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
extern volatile uint32_t g_stage3_cycle_mode,g_stage3_locked_run;
extern volatile uint32_t g_stage3_first_colors[3],g_stage3_first_positions[3],g_stage3_first_mapping_valid;
extern volatile float g_stage3_recovery_depth[3];
uint32_t Stage3_Owns(void);
uint32_t Stage3_ReplayBodyQueueOwned(void);
uint32_t Stage3_ReplayStopPending(void);
uint32_t Stage3_FineTurnAllowed(void);
void Stage3_KeyEvent(void);
void Stage3_Arrive(void);
typedef struct {uint32_t run_id,batch_id,colors[3],positions[3];} Stage3_Task;
/* Existing mission/CAN owner only. Bind first, then call Stage3_Arrive. */
uint32_t Stage3_BindMission(const Stage3_Task *task,const uint32_t slot_colors[3]);
extern volatile uint32_t g_stage3_slot_colors[3],g_stage3_task_run,g_stage3_task_batch;
extern volatile uint32_t g_stage3_teach_sequence,g_stage3_teach_saved_sequence;
#endif
