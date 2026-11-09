#ifndef LIFT_HOME_H
#define LIFT_HOME_H
#include "stm32f4xx.h"
/* Driver menu must match; host does not write parameters or auto-start. */
#define LIFT_HOME_EXPECTED_RPM 60U
/* User-selected 40-second driver limit; finite, separate from unlimited host wait. */
#define LIFT_HOME_EXPECTED_DRIVER_TIMEOUT_MS 40000U
/* After start: no host total timeout. Stop, driver faults and CAN errors remain active. */
/* 请求1手动向上碰撞回零（顶部，参数方向须为1）；2退出回零并停车。默认0不动作。 */
extern volatile uint32_t g_lift_home_request;
/* 0闲置，1~7准备/回零，8驱动报告完成，9失败锁存，10中止。 */
extern volatile uint32_t g_lift_home_state;
/* 1驱动完成，2中止，3启动准备超时，4通信，5驱动保护，6未就绪，7参数不符。 */
extern volatile uint32_t g_lift_home_result;
extern volatile uint32_t g_lift_home_flags;
extern volatile uint32_t g_lift_home_start_result;
extern volatile uint32_t g_lift_home_abort_result;
extern volatile uint32_t g_lift_home_stop_result;
/* 仅驱动报告的原点状态；不等于独立机械验收。掉电/失败/中止清零。 */
extern volatile uint32_t g_lift_home_reported;
uint32_t LiftHome_IsBusy(void);
void LiftHome_Service(uint32_t ready);
#endif
