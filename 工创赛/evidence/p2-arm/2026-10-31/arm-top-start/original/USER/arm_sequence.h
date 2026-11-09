#ifndef ARM_SEQUENCE_H
#define ARM_SEQUENCE_H
#include <stdint.h>
/* 调试组：在已人工调好夹取高度、夹爪张开时才允许启动。默认均0。 */
extern volatile uint32_t g_arm_sequence_enable;
extern volatile uint32_t g_arm_grip_height_ready;
/* 1单次夹紧/抬起/放回/松开；2中止保持夹爪；3停稳后清锁存。 */
extern volatile uint32_t g_arm_sequence_request;
/* 夹紧后state2观察确实夹住，再写1；不能提前预填。 */
extern volatile uint32_t g_arm_grip_confirm;
extern volatile uint32_t g_arm_test_lift_pulses;
/* 模式0保留小幅往返；模式1夹紧后顶部回零，再按显式记录的脉冲下降放回。 */
extern volatile uint32_t g_arm_sequence_mode;
/* 顶部到夹取处的已记录脉冲，默认0拒绝；不是毫米或自动测量值。 */
extern volatile uint32_t g_arm_return_down_pulses;
uint32_t ArmSequence_AllowHomeRequest(uint32_t request);
uint32_t ArmSequence_OwnsHome(void);
/* 0空闲，1闭合，2等待夹住确认，3抬升，4短暂停留，5放回，6张开，7软件完成，8中止，9失败。 */
extern volatile uint32_t g_arm_sequence_state;
/* 1仅软件流程结束，2中止，3准入，4配置，5夹爪，6升降，7超时，8服务间隔。 */
extern volatile uint32_t g_arm_sequence_result;
extern volatile uint32_t g_arm_sequence_elapsed_ms;
uint32_t ArmSequence_IsBusy(void);
uint32_t ArmSequence_AllowLiftRequest(uint32_t request);
void ArmSequence_Service(uint32_t ready,uint32_t other_busy,uint32_t now_ms);
#endif
