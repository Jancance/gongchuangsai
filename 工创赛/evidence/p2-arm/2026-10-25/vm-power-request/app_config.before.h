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
#define APP_HEADING_SESSION_TEST 1U

#if APP_HWT_STATIC_TEST && APP_HWT_TURN_TEST
#error HWT_test_modes_must_be_exclusive
#endif
#define APP_HWT_BAUD 115200U
#define APP_CHASSIS_KEY_READY_MS 5000U

#if APP_HEADING_SESSION_TEST && (APP_HWT_STATIC_TEST || APP_HWT_TURN_TEST || APP_CHASSIS_KEY_TEST_ENABLE)
#error Heading_session_mode_must_be_exclusive
#endif

#endif
