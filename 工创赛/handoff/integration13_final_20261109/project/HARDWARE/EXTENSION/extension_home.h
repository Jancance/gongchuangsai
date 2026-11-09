#ifndef EXTENSION_HOME_H
#define EXTENSION_HOME_H
#include <stdint.h>

/* 本机2026-10-30实读候选；只匹配，不写驱动参数，不表示碰撞停止已验收。 */
#define EXTENSION_HOME_DIRECTION           0U
#define EXTENSION_HOME_RPM                 30U
#define EXTENSION_HOME_DRIVER_TIMEOUT_MS   10000U
#define EXTENSION_HOME_DETECT_RPM          300U
#define EXTENSION_HOME_DETECT_CURRENT_MA   800U
#define EXTENSION_HOME_DETECT_TIME_MS      60U
#define EXTENSION_HOME_PREP_TIMEOUT_MS     2500U
#define EXTENSION_HOME_HOST_TIMEOUT_MS     11000U
#define EXTENSION_HOME_SERVICE_GAP_MS      500U
#define EXTENSION_HOME_POLL_MS             50U

/* 默认0；1单次向缩回机械止挡碰撞回零，2退出回零并停车；不上电自动触发。 */
extern volatile uint32_t g_extension_home_request;
/* 0闲置，1～7准备/回零，8驱动报告完成，9失败锁存，10人工中止。 */
extern volatile uint32_t g_extension_home_state;
/* 1驱动完成，2中止，3超时，4通信，5驱动状态，6互锁，7参数不符，8服务间隔超限。 */
extern volatile uint32_t g_extension_home_result;
extern volatile uint32_t g_extension_home_flags;
extern volatile uint32_t g_extension_home_start_result;
extern volatile uint32_t g_extension_home_abort_result;
extern volatile uint32_t g_extension_home_stop_result;
/* 驱动本会话报告，不等于已触挡、机械重复性或毫米位置验收。 */
extern volatile uint32_t g_extension_home_reported;
extern volatile uint32_t g_extension_home_elapsed_ms;
/* 运行中或启动后停止未确认时均占用机构，禁止其他轴启动。 */
uint32_t ExtensionHome_IsBusy(void);
void ExtensionHome_Service(uint32_t ready, uint32_t other_busy);
#endif
