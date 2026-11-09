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
uint32_t VehicleArm_BeginSourcePreparation(void);
extern volatile uint32_t g_source_prepare_state,g_source_prepare_cause,g_source_prepare_done_mask;
extern volatile uint32_t g_source_prepare_release_ms,g_source_prepare_elapsed_ms,g_source_prepare_segment;
extern volatile float g_source_prepare_extension_mm,g_source_prepare_grip_deg;
typedef struct {
 uint32_t reason,tick,phase,segment,command,result,rx_id,dlc,raw_hi,raw_lo,bad_count,position_age;
 int32_t origin_mdeg,current_mdeg,goal_mdeg;
} SourcePrepFault;
extern volatile SourcePrepFault g_source_prepare_fault;
typedef struct {uint32_t tick,phase,segment,command,result,raw_hi,raw_lo,options,units;int32_t before_mdeg,goal_mdeg,driver_mdeg;} SourcePrepTrace;
#define SOURCE_PREP_TRACE_CAPACITY 128U
extern volatile SourcePrepTrace g_source_prepare_trace[SOURCE_PREP_TRACE_CAPACITY];
extern volatile uint32_t g_source_prepare_trace_count,g_source_prepare_trace_dropped;
extern volatile uint32_t g_source_prepare_command_units,g_source_prepare_command_options;
extern volatile int32_t g_source_prepare_command_before_mdeg,g_source_prepare_command_goal_mdeg;
extern volatile int32_t g_source_prepare_driver_target_mdeg;
extern volatile uint32_t g_extension_calibration_only;
extern volatile uint32_t g_extension_cal_state,g_extension_cal_cause,g_extension_cal_samples;
extern volatile uint32_t g_extension_cal_options,g_extension_cal_encoder_before,g_extension_cal_encoder_after;
extern volatile int32_t g_extension_cal_position_before,g_extension_cal_position_after,g_extension_cal_target_raw;
uint32_t VehicleArm_BeginExtensionCalibration(void);

#include "source_parallel_flow.h"
extern volatile uint32_t g_vehicle_lift_reference_valid;
extern volatile uint32_t g_it_request,g_it_state,g_it_cause,g_it_rejected,g_it_action,g_it_sequence;
extern volatile uint32_t g_it_axes,g_it_elapsed_ms,g_it_lift_permit;
extern volatile float g_it_dm_deg,g_it_extension_mm,g_it_lift_pulses;
void Integration10_Service(void);
/* Read-only arbitration: prepared source stow may overlap body travel. */
uint32_t VehicleArm_SourceParallelAllowsBody(void);
uint32_t VehicleArm_BeginSourceParallel(const SPF_Command*,uint32_t,uint32_t);
uint32_t VehicleArm_SourceParallelCanAllowed(uint8_t,const uint8_t*,uint32_t);
extern volatile uint32_t g_source_parallel_state,g_source_parallel_cause,g_source_parallel_stop_mask;
#endif
