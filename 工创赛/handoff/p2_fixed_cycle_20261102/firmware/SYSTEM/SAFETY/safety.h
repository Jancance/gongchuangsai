#ifndef SAFETY_H
#define SAFETY_H

#include <stdint.h>
#include "FreeRTOS.h"

/* 当前关键任务编号：不把状态灯任务当成安全证明 */
#define SAFETY_TASK_APP 0U
#define SAFETY_TASK_RX  1U
#define SAFETY_TASK_KEY 2U

/* 应用监督故障，与g_rtos_fault分开记录 */
extern volatile uint32_t g_safety_fault;
extern volatile uint32_t g_safety_ready;
extern volatile uint32_t g_safety_count;
/* CAN健康和首次CAN故障上下文；节点位0～3对应地址1～4 */
extern volatile uint32_t g_can_guard_ready;
extern volatile uint32_t g_can_guard_bad_nodes;
/* 原因：1必要采集关闭，2有效数据缺失/过期，3堵转/保护 */
extern volatile uint32_t g_can_guard_cause;

/* 运动许可第一阶段：仅诊断禁止原因，尚不产生允许输出 */
#define MOTION_BLOCK_VM_OFF         0x01U
#define MOTION_BLOCK_PLATFORM       0x02U
#define MOTION_BLOCK_CAN            0x04U
/* 运动反馈时效、限值、启动授权、停止与重新允许尚未闭环 */
#define MOTION_BLOCK_POLICY_PENDING 0x08U
#define MOTION_BLOCK_SPEED_FEEDBACK 0x10U
#define MOTION_BLOCK_POSITION_FEEDBACK 0x20U
#define MOTION_BLOCK_DIAG_LATCHED   0x40U
/* 本阶段仍是静态诊断阈值，不能据此放行运动 */
#define SAFETY_MOTION_DIAG_FRESH_MS 1000U

/* 仅由Safety任务更新；当前阶段始终为0，不用Watch手动置1 */
extern volatile uint32_t g_motion_allowed;
/* 位掩码，可同时包含多个禁止原因；不等同于g_safety_fault */
extern volatile uint32_t g_motion_block_reason;
/* Safety独立计算的新鲜节点掩码，位0到3对应地址1到4 */
extern volatile uint32_t g_motion_speed_fresh_mask;
extern volatile uint32_t g_motion_position_fresh_mask;
/* 反馈诊断锁存：10速度异常、20位置异常；不是原Safety故障 */
extern volatile uint32_t g_motion_diag_fault_latched;
/* 两类四轮反馈曾完整新鲜后置1，正常撤VM后回0 */
extern volatile uint32_t g_motion_diag_monitor_armed;
/* 单次确认请求默认0；仅在VM请求0且平台健康时接受 */
extern volatile uint32_t g_motion_diag_ack_request;
/* 1确认成功，9条件不满足；拒绝的请求也消费，不延后执行 */
extern volatile uint32_t g_motion_diag_ack_result;

/* 独立CAN任务发布真实采样时间；报告时间不能代替采样时间 */
void Safety_MotionFeedbackReport(uint32_t enabled,
    uint32_t speed_valid_mask, uint32_t position_valid_mask,
    const volatile TickType_t *speed_ticks,
    const volatile TickType_t *position_ticks);

/* 静态自动停止：02只表示收到命令，不等于实际停车 */
#define MOTION_STOP_IDLE 0U
#define MOTION_STOP_PENDING 1U
#define MOTION_STOP_ACKED 2U
#define MOTION_STOP_VM_CUT 3U
#define MOTION_STOP_CANCELLED 4U
/* 从请求发布计时的静态候选，不是实际制动时间保证 */
#define MOTION_STOP_ACK_WAIT_MS 500U
extern volatile uint32_t g_motion_auto_stop_state;
extern volatile uint32_t g_motion_auto_stop_ack_mask;
extern volatile uint32_t g_motion_auto_stop_fail_mask;
extern volatile uint32_t g_motion_auto_stop_result[4];
extern volatile uint32_t g_motion_auto_stop_cut_vm;
extern volatile uint32_t g_motion_auto_stop_session;
/* cause：0无兜底、1应答失败、2总等待超时、3原平台故障 */
extern volatile uint32_t g_motion_auto_stop_cause;
/* 仅CAN任务调用，带会话号拒绝旧回报 */
uint32_t Safety_MotionStopTake(uint8_t *address, uint32_t *session);
void Safety_MotionStopReport(uint8_t address, uint32_t session,
                             uint32_t result);

/* 人工四轮短时测试：固定四台、raw1；不代表正式运动许可 */
#define WHEEL_TEST_RUN_MS 10000U
#define WHEEL_TEST_CUT_MS 10500U
#define WHEEL_TEST_PREP_MS 5000U
#define WHEEL_TEST_IDLE 0U
#define WHEEL_TEST_STARTING 1U
#define WHEEL_TEST_RUNNING 2U
#define WHEEL_TEST_STOPPING 3U
#define WHEEL_TEST_DONE 4U
#define WHEEL_TEST_CUT 5U
#define WHEEL_TEST_CANCELLED 6U
#define WHEEL_TEST_PREPARING 7U
extern volatile uint32_t g_wheel_test_state;
extern volatile uint32_t g_wheel_test_active;
extern volatile uint32_t g_wheel_test_cut_vm;
extern volatile uint32_t g_wheel_test_session;
/* 0正常定时关电，1启动失败，2停止失败，3截止超时，4健康丢失，5人工中止，6准备失败/超时 */
extern volatile uint32_t g_wheel_test_cause;
uint32_t Safety_WheelTestPrepare(void);
void Safety_WheelTestPrepareFailed(void);
uint32_t Safety_WheelTestArm(uint32_t *session);
uint32_t Safety_WheelTestStartAllowed(uint32_t session);
void Safety_WheelTestStartReport(uint32_t session, uint32_t result);
uint32_t Safety_WheelTestTakeStop(uint32_t *session);
void Safety_WheelTestStopReport(uint32_t session, uint32_t result);
void Safety_WheelTestAbort(void);

void Safety_Init(void);
void Safety_Report(uint32_t task_id);
void Safety_Update(uint32_t vm_request, uint32_t can_test_result);

void Safety_CanReport(uint32_t enabled, uint32_t valid_mask,
                      uint32_t protect_mask,
                      const volatile TickType_t *ticks);

#endif
