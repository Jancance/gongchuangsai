#ifndef SOURCE_PARALLEL_FLOW_H
#define SOURCE_PARALLEL_FLOW_H
#include <stdint.h>
/* Source-stage transaction coordinator. No CAN/UART/PWM access. A backend
 * consumes immutable batches and reports physical-stage outcomes separately.
 * One batch means parallel axes, not a list of single-axis rows. */
/* Aligned to the already measured source commissioning target. */
#define SPF_MAX_LIFT_PULSES 29700U
#define SPF_STAGE3_MAX_LIFT_PULSES 82000U
#define SPF_CARRIER_EXTRA_DOWN_PULSES 4000U /* User carrier release: 10mm at 400p/mm. */
#define SPF_STAGE3_EXT_RPM 106.0f
#define SPF_DM 1U
#define SPF_EXT 2U
#define SPF_LIFT 4U
#define SPF_ROUTE 8U
#define SPF_STOW 16U
#define SPF_DM_DEG_S 300.0f
#define SPF_STAGE3_DM_DEG_S (SPF_DM_DEG_S / 2.0f)
#define SPF_EXT_RPM 150.0f
#define SPF_LIFT_RPM 1846.4f
#define SPF_STAGE3_CARRY_RISE_LIFT_RPM 1000.0f
/* Empty LIFT-only return to the existing observation top; yaw starts only after true DONE. */
#define SPF_SOURCE_EMPTY_OBSERVE_RETURN_LIFT_RPM 600.0f
#define SPF_CARRIER_LOWER_LIFT_RPM 600.0f
/* 2026-11-08: 45055 is not a legal stepper acceleration on this hardware -- it made
 * the extension axis run away (0 -> -670 deg in 0.18 s, overshoot past the 156 mm
 * target). The lift ruler uses 600 for the same drives; SPF_GroupBegin also pins
 * this value, so both must agree. */
#define SPF_STEPPER_ACCEL 600U
enum { SPF_IDLE,SPF_DEPLOY,SPF_ALIGN,SPF_WAIT_TARGET,SPF_PICK,
 SPF_CLOSE,SPF_TRANSFER,SPF_PLACE,SPF_RELEASE,SPF_NEXT,
 SPF_ROUTE_AND_STOW,SPF_DONE,SPF_FAULT };
enum { SPF_CMD_DEPLOY=1,SPF_CMD_ALIGN,SPF_CMD_PICK,SPF_CMD_CLOSE,
 SPF_CMD_TRANSFER,SPF_CMD_PLACE,SPF_CMD_RELEASE,SPF_CMD_NEXT,
 SPF_CMD_ROUTE_AND_STOW,SPF_CMD_STOP,SPF_CMD_STAGE3_TOUCHDOWN,SPF_CMD_STAGE3_FAST_TRANSFER,
 SPF_CMD_STAGE3_LOADED_RETURN,SPF_CMD_STAGE3_EMPTY_HANDOFF,SPF_CMD_STAGE3_CARRY_RISE,SPF_CMD_STAGE3_CARRY_PLACE,SPF_CMD_STAGE3_STORAGE_EXIT,SPF_CMD_SOURCE_EMPTY_OBSERVE_RETURN,SPF_CMD_CARRIER_LOWER };
#define SPF_STAGE3_TOUCHDOWN_RPM 30.0f
#define SPF_STAGE3_TOUCHDOWN_MAX_PULSES 1200U
/* First real VSP poll failure. ARM uint32/float fields are 4 bytes; valid publishes last. */
enum {SPF_POLL_DM_FIRSTFAULT=1,SPF_POLL_DM_STATE8,SPF_POLL_AXIS_DISABLED,
 SPF_POLL_AXIS_PROTECT,SPF_POLL_Z_STALL,SPF_POLL_EXT_STALL_TIMEOUT,SPF_POLL_POSITION_WINDOW};
typedef struct {
 uint32_t valid,sequence,run_id,item,kind,axis,node,reason,phase,flags,options;
 uint32_t position_tick,position_cycle,started_mask,done_mask;
 uint32_t dm_state,dm_result,dm_first_fault,dm_stop_result,now_ms;
 float before,current,goal;
 uint8_t reply[8];
} SPF_PollFirst;
extern volatile SPF_PollFirst g_source_parallel_poll_first;

typedef struct {float dm_deg,extension_mm,lift_pulses;} SPF_Pose;
typedef struct {
 uint32_t run_id,pose_version,live_references_valid,deploy_sweep_valid;
 uint32_t carrier_height_valid,transfer_sweep_valid,stow_while_driving_valid;
 uint32_t transaction_timeout_ms;
 uint32_t requested_colors[3];
 SPF_Pose deploy,transfer,place,initial;
 float gripper_open_deg,gripper_close_deg,initial_gripper_deg;
} SPF_Config;
typedef struct {
 uint32_t run_id,pose_version,sequence,observed_ms,usable_until_ms;
 uint32_t color_id,instance_id,stationary,sweep_valid,safe_exit_budget_ms;
 SPF_Pose pick;
} SPF_Target;
typedef struct {
 uint32_t run_id,sequence,kind,item,axes_mask,route_stage;
 uint32_t pose_version,stepper_accel_rpm_s;
 /* Immutable per-command scope: zero keeps the source profile. */
 uint32_t lift_limit_pulses;
 SPF_Pose goal;
 float dm_deg_s,extension_rpm,lift_rpm,gripper_deg;
 /* Full route+stow is one publication. Body must not wait on arm busy. */
 uint32_t keep_vm_on,return_to_top_after_all_three;
} SPF_Command;
typedef struct {
 SPF_Config config;SPF_Command command;
 uint32_t state,item,placed,sequence,last_target_sequence,last_color;
 uint32_t last_progress_ms,issued,branch_done_mask,fault,stop_pending;
 uint32_t used_instances[3];
} SPF_Flow;
/* Call only from the existing mission owner/task; no concurrent Watch writes.
 * Begin is allowed before carrier height is measured, so deployment/alignment
 * can be prepared independently. Pickup then waits for a complete profile. */
uint32_t SPF_Begin(SPF_Flow *f,const SPF_Config *cfg,uint32_t now);
uint32_t SPF_ReadCommand(SPF_Flow *f,SPF_Command *out,uint32_t now);
uint32_t SPF_Report(SPF_Flow *f,uint32_t sequence,uint32_t success,
 uint32_t completed_mask,uint32_t now);
uint32_t SPF_TargetReady(SPF_Flow *f,const SPF_Target *target,uint32_t now);
void SPF_Service(SPF_Flow *f,uint32_t now);
void SPF_Cancel(SPF_Flow *f,uint32_t reason);
/* All axes must be preflighted before release. Sink prepares/starts hardware
 * within one owner; failures retain STOP on every possibly started axis. */
typedef struct {
 void *context;
 uint32_t (*prepare)(void *,uint32_t,const SPF_Command *);
 uint32_t (*start)(void *,uint32_t,const SPF_Command *);
 uint32_t (*poll)(void *,uint32_t); /* 0 busy,1 fresh done,2 fault */
 uint32_t (*stop)(void *,uint32_t); /* 1 confirmed,0 retry */
 uint32_t (*clock_ms)(void *);
} SPF_GroupIO;
typedef struct {
 SPF_Command command;SPF_GroupIO io;
 uint32_t state,prepared,started,done,stop_mask,fault,begin_ms;
 uint32_t first_start_ms,last_start_ms,max_start_skew_ms,timeout_ms,progress_ms;
 /* Opt-in position-gated release; legacy groups keep both zero. */
 uint32_t delayed_mask,release_mask;
} SPF_Group;
uint32_t SPF_GroupBegin(SPF_Group *g,const SPF_Command *command,
 const SPF_GroupIO *io,uint32_t sweep_and_references_verified,
 uint32_t max_start_skew_ms,uint32_t timeout_ms);
void SPF_GroupService(SPF_Group *g);
void SPF_GroupCancel(SPF_Group *g,uint32_t reason);
uint32_t VehicleArm_BeginStage3CarryPlace(const SPF_Command *rise,const SPF_Command *place);
uint32_t VehicleArm_Stage3LiftDepthFresh(uint32_t sequence,float *depth);
#endif
