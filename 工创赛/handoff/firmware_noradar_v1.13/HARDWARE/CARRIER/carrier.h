#ifndef CARRIER_H
#define CARRIER_H
#include <stdint.h>
/* 首次仅脱开盘机构测试：先enable=1，再request=1；2撤销信号/结束会话。 */
extern volatile uint32_t g_carrier_test_enable;
extern volatile uint32_t g_carrier_request;
extern volatile float g_carrier_debug_angle;
extern volatile float g_carrier_latched_angle;
extern volatile uint32_t g_carrier_init_result;
/* 0空闲，1输出窗口，2手动窗口结束待确认，3撤销，4启动0度保持，9失败。 */
extern volatile uint32_t g_carrier_state;
/* 1窗口结束(非到位)，2人工撤销，3非法角度，4未就绪/禁用，
 * 5其他轴互锁，6底层写入失败，8服务间隔超限，9未知请求。
 * 忙时重复请求只增加rejected_count，不重启窗口。 */
extern volatile uint32_t g_carrier_result;
extern volatile uint32_t g_carrier_pulse_us;
extern volatile uint32_t g_carrier_signal_active;
extern volatile uint32_t g_carrier_elapsed_ms;
extern volatile uint32_t g_carrier_rejected_count;
/* 窗口结束仍保留互锁，观察停稳后request2才结束；无位置反馈。 */
uint32_t Carrier_IsBusy(void);
void Carrier_Init(void);
void Carrier_Service(uint32_t ready, uint32_t other_busy, uint32_t now_ms);
#endif
