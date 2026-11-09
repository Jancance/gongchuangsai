#ifndef LIFT_RULER_CALIBRATION_H
#define LIFT_RULER_CALIBRATION_H
#include <stdint.h>
enum {LRC_IDLE=0,LRC_POWER=1,LRC_PREP=2,LRC_MOVING=3,LRC_DONE=4,LRC_CANCELLED=5,LRC_FAULT=6};
enum {LRC_CAUSE_NONE=0,LRC_CAUSE_CANCEL=1,LRC_CAUSE_GUARD=2,LRC_CAUSE_POWER=3,
 LRC_CAUSE_COMM=4,LRC_CAUSE_REPLY=5,LRC_CAUSE_OPTIONS=6,LRC_CAUSE_PREP=7,
 LRC_CAUSE_TARGET=8,LRC_CAUSE_DRIVER=9,LRC_CAUSE_POSITION=10,LRC_CAUSE_SPEED=11,
 LRC_CAUSE_TIMEOUT=12,LRC_CAUSE_SERVICE=13,LRC_CAUSE_ENCODER=14,LRC_CAUSE_HOME_PARAMS=15,
 LRC_CAUSE_HOME_FLAGS=16};
enum {LRC_REQUEST_START=1,LRC_REQUEST_STOP=2};
typedef uint32_t (*LiftRulerExchange)(void*,const uint8_t*,uint32_t,uint32_t,uint8_t*);
typedef struct {
 volatile uint32_t state,vm_request,cause,stop_pending,stop_confirmed,stop_unconfirmed;
 uint32_t phase,begin_ms,last_service_ms,prep_ms,last_position_ms,stop_begin_ms,last_stop_ms;
 uint32_t options,scale,stable,flags,speed_raw,moves_sent,move_units,samples,encoder_before,encoder_after;
 int64_t position_before_raw,command_before_raw,position_after_raw,driver_target_raw,goal_raw;
 uint8_t move_frame[15];
 uint32_t operation,top_valid,top_scale,command_speed,command_accel,home_started,home_seen_running;
 uint32_t home_flags,home_params_valid,home_reported,home_exit_confirmed,expected_encoder_mod;
 uint32_t move_begin_ms,move_elapsed_ms;
 int64_t top_position_raw;
 uint8_t home_parameters[17];
} LiftRulerController;
void LiftRuler_Init(LiftRulerController*);
/* Three explicit operations: initial return, fast down, fast return. No auto-repeat. */
uint32_t LiftRuler_Start(LiftRulerController*,uint32_t now,uint32_t ready);
void LiftRuler_Stop(LiftRulerController*,uint32_t now);
void LiftRuler_Service(LiftRulerController*,uint32_t now,uint32_t ready,uint32_t power,uint32_t healthy,LiftRulerExchange,void*);
uint32_t LiftRuler_CommandAllowed(const LiftRulerController*,const uint8_t*,uint32_t);
#endif
