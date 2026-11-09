#ifndef LIFT_RULER_CALIBRATION_H
#define LIFT_RULER_CALIBRATION_H
#include <stdint.h>
enum {LRC_IDLE=0,LRC_POWER=1,LRC_PREP=2,LRC_MOVING=3,LRC_DONE=4,LRC_CANCELLED=5,LRC_FAULT=6};
enum {LRC_CAUSE_NONE=0,LRC_CAUSE_CANCEL=1,LRC_CAUSE_GUARD=2,LRC_CAUSE_POWER=3,
 LRC_CAUSE_COMM=4,LRC_CAUSE_REPLY=5,LRC_CAUSE_OPTIONS=6,LRC_CAUSE_PREP=7,
 LRC_CAUSE_TARGET=8,LRC_CAUSE_DRIVER=9,LRC_CAUSE_POSITION=10,LRC_CAUSE_SPEED=11,
 LRC_CAUSE_TIMEOUT=12,LRC_CAUSE_SERVICE=13,LRC_CAUSE_ENCODER=14};
enum {LRC_REQUEST_START=1,LRC_REQUEST_STOP=2};
typedef uint32_t (*LiftRulerExchange)(void*,const uint8_t*,uint32_t,uint32_t,uint8_t*);
typedef struct {
 volatile uint32_t state,vm_request,cause,stop_pending,stop_confirmed,stop_unconfirmed;
 uint32_t phase,begin_ms,last_service_ms,prep_ms,last_position_ms,stop_begin_ms,last_stop_ms;
 uint32_t options,scale,stable,flags,speed_raw,moves_sent,move_units,samples,encoder_before,encoder_after;
 int64_t position_before_raw,command_before_raw,position_after_raw,driver_target_raw,goal_raw;
 uint8_t move_frame[15];
} LiftRulerController;
void LiftRuler_Init(LiftRulerController*);
/* START只接单一次；完成/故障后不自动重试相对位移。仅外部显式请求。 */
uint32_t LiftRuler_Start(LiftRulerController*,uint32_t now_ms,uint32_t platform_and_axes_idle);
void LiftRuler_Stop(LiftRulerController*,uint32_t now_ms);
/* exchange适配器固定为CAN地址5。所有访问必须由同一个CAN任务调用。 */
void LiftRuler_Service(LiftRulerController*,uint32_t now_ms,uint32_t platform_and_axes_idle,
 uint32_t power_active,uint32_t can_healthy,LiftRulerExchange,void*);
/* 供实际CAN准入适配；只有本轮已构造的地址5使能/相对小步可以走新调试例外。 */
uint32_t LiftRuler_CommandAllowed(const LiftRulerController*,const uint8_t*,uint32_t);
#endif
