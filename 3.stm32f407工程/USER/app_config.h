#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* 初始化提示次数：0静音，1默认，可改为3 */
#define APP_INIT_BEEP_COUNT    2U

/* 单次鸣叫和两次之间的间隔，单位：毫秒 */
#define APP_BEEP_ON_MS         200U
#define APP_BEEP_GAP_MS        200U
/* 启动按键连续稳定时间，单位：毫秒 */
#define APP_START_KEY_DEBOUNCE_MS    20U

/* 开发专用PC13脱机测试；设0恢复原扫码启动，非正式比赛模式 */
#define APP_CHASSIS_KEY_TEST_ENABLE 0U
/* 1为静止校准验证、强制VM0；当前0改用下方受控转动测试。 */
#define APP_HWT_STATIC_TEST 0U
/* 无线Watch显式请求：只运行5RPM单段转动10秒，默认VM关闭。 */
#define APP_HWT_TURN_TEST 0U
/* 当前顺时针对照；0恢复已验证逆时针，1为顺时针。 */
#define APP_HWT_TURN_CLOCKWISE 1U
/* 串口双向专用测试：VM强制关闭，所有运动入口禁用；0恢复原运动配置。 */
/* 测试入口：0手动运动，1无线收发，2串口单次平移。改此处后重新编译下载。 */
/* 3为组合动作测试，默认关闭；不与串口发车同时启用。 */
#define APP_TEST_MODE 3U
#if APP_TEST_MODE > 3U
#error App_test_mode_invalid
#endif
#define APP_UART_LINK_TEST (APP_TEST_MODE == 1U)
/* 串口平移开发模式：CRC遥测、两步启动、1秒通信失效取消。 */
#define APP_SERIAL_MOTION_CONTROL ((APP_TEST_MODE == 2U) || (APP_TEST_MODE == 3U))
#define APP_HEADING_SESSION_TEST 1U
/* 前进航向保持开发模式；0切回已验证的原地转角会话。 */
#define APP_FORWARD_HOLD_TEST 1U
#if APP_SERIAL_MOTION_CONTROL && (APP_UART_LINK_TEST || !APP_HEADING_SESSION_TEST || !APP_FORWARD_HOLD_TEST)
#error Serial_motion_requires_exclusive_translation_runtime
#endif
/* 平移模式：0前进、1车体左移、2车体右移；仅编译选择，不在运动中切换。 */
#define APP_TRANSLATION_HOLD_MODE 2U
#define APP_LATERAL_HOLD_BASE_RPM 15U
#define APP_LATERAL_HOLD_MAX_RPM (APP_LATERAL_HOLD_BASE_RPM + 2U)
#define APP_LATERAL_HOLD_FEEDBACK_LIMIT_RPM 30U
#define APP_LATERAL_HOLD_RUN_MS 8000U
#if (APP_TRANSLATION_HOLD_MODE > 2U)
#error Translation_hold_mode_invalid
#endif
#if (APP_LATERAL_HOLD_BASE_RPM < 3U) || (APP_LATERAL_HOLD_BASE_RPM > 15U)
#error Lateral_hold_base_speed_out_of_trial_bounds
#endif
#if APP_LATERAL_HOLD_FEEDBACK_LIMIT_RPM < APP_LATERAL_HOLD_MAX_RPM
#error Lateral_hold_feedback_limit_below_command_limit
#endif
#if (APP_LATERAL_HOLD_RUN_MS == 0U) || (APP_LATERAL_HOLD_RUN_MS > 10000U)
#error Lateral_hold_run_duration_out_of_trial_bounds
#endif
#define APP_TRANSLATION_HOLD_RUN_MS ((APP_TRANSLATION_HOLD_MODE == 0U) ? APP_FORWARD_HOLD_RUN_MS : APP_LATERAL_HOLD_RUN_MS)
/* 本轮加速候选；纠偏仍限2RPM，最大命令随基础速度同步。 */
#define APP_FORWARD_HOLD_BASE_RPM 15U
#define APP_FORWARD_HOLD_MAX_RPM (APP_FORWARD_HOLD_BASE_RPM + 2U)
/* 前进测试反馈保护阈值，独立于目标和纠偏命令上限。 */
#define APP_FORWARD_HOLD_FEEDBACK_LIMIT_RPM 30U
#if APP_FORWARD_HOLD_FEEDBACK_LIMIT_RPM < APP_FORWARD_HOLD_MAX_RPM
#error Forward_hold_feedback_limit_below_command_limit
#endif
#if (APP_FORWARD_HOLD_BASE_RPM < 3U) || (APP_FORWARD_HOLD_BASE_RPM > 15U)
#error Forward_hold_base_speed_out_of_trial_bounds
#endif
#define APP_FORWARD_HOLD_RUN_MS 8000U
#if (APP_FORWARD_HOLD_RUN_MS == 0U) || (APP_FORWARD_HOLD_RUN_MS > 10000U)
#error Forward_hold_run_duration_out_of_trial_bounds
#endif
#if APP_FORWARD_HOLD_TEST && !APP_HEADING_SESSION_TEST
#error Forward_hold_requires_heading_runtime_backend
#endif
/* 航向调试上电准备试验值，不代表测得VM电压已稳定。 */
#define APP_HEADING_POWER_SETTLE_MS 1000U
/* 30度低速分段减速试验的有限会话截止，包含执行器准备。 */
#define APP_HEADING_RUN_MS ((APP_TEST_MODE == 3U) ? 30000U : 20000U)
/* 当前实车航向会话到位容差；旧软件自检默认配置保持原值。 */
#define APP_HEADING_TOLERANCE_DEG 1.0f
#if (APP_HEADING_RUN_MS <= 1000U) || (APP_HEADING_RUN_MS > 30000U)
#error Heading_run_deadline_out_of_test_bounds
#endif
#if (APP_HEADING_POWER_SETTLE_MS == 0U) || (APP_HEADING_POWER_SETTLE_MS >= 5000U)
#error Heading_power_settle_must_fit_existing_wait_deadline
#endif

#if APP_HWT_STATIC_TEST && APP_HWT_TURN_TEST
#error HWT_test_modes_must_be_exclusive
#endif
#define APP_HWT_BAUD 115200U
#define APP_CHASSIS_KEY_READY_MS 5000U

#if APP_HEADING_SESSION_TEST && (APP_HWT_STATIC_TEST || APP_HWT_TURN_TEST || APP_CHASSIS_KEY_TEST_ENABLE)
#error Heading_session_mode_must_be_exclusive
#endif

#endif
