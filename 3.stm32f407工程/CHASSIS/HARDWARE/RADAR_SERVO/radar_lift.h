#ifndef RADAR_LIFT_H
#define RADAR_LIFT_H
#include <stdint.h>
/* First rig only: PD10/PD11/PD12/PB8 -> ULN2003 IN1..4. No homing input.
 * All positions/counts are relative half-step commands, not measured height. */
#define RADAR_LIFT_MAX_STEPS 64U
#define RADAR_LIFT_INTERVAL_US 977U
/* uint16软件计数范围，非机械行程上限；Watch和Begin仍限制64半步。 */
#define RADAR_LIFT_LONG_MAX_STEPS 65535U
extern volatile uint32_t g_radar_lift_interval_us; /* 733..4000us，默认977，单次入队锁存。 */
extern volatile uint32_t g_radar_lift_enable,g_radar_lift_request;
extern volatile uint32_t g_radar_lift_steps,g_radar_lift_direction;
extern volatile uint32_t g_radar_lift_state,g_radar_lift_result,g_radar_lift_sequence;
extern volatile uint32_t g_radar_lift_done,g_radar_lift_phase_bits;
/* enable defaults 0; request1=start,2=cancel and retain holding phase.
 * direction0/1 are opposite sequences; 0=up,1=down confirmed on this rig.
 * state0=untouched,1=moving,2=holding complete,3=cancelled/fault holding.
 * result1=start accepted,2=unsafe/disabled,3=invalid,4=busy,5=cancelled.
 * Completed/cancelled moves keep the last phase energized to avoid dropping.
 * No automatic release, reverse, return-to-origin or chassis command. */
void RadarLift_Service(uint32_t now_ms,uint32_t safe_idle);
typedef struct { uint32_t sequence,state,result,done; } RadarLiftSnapshot;
void RadarLift_GetSnapshot(RadarLiftSnapshot *s);
uint32_t RadarLift_Available(void);
uint32_t RadarLift_Begin(uint32_t steps,uint32_t direction,uint32_t *sequence);
/* 单程预算须>=ceil(steps*锁存us/1000)且<2^31ms；取消/拒绝不保留长程权限。 */
uint32_t RadarLift_BeginBounded(uint32_t steps,uint32_t direction,uint32_t budget_ms,uint32_t *sequence);
uint32_t RadarLift_Claim(void);
void RadarLift_Unclaim(void);
uint32_t RadarLift_Enable(uint32_t now_ms);
/* 仅同序列正常完成且无排队动作允许释放；异常保相。重复释放幂等。 */
uint32_t RadarLift_Release(uint32_t sequence);
void RadarLift_Cancel(void);
uint32_t RadarLift_SafeIdle(uint32_t now_ms);
#endif
