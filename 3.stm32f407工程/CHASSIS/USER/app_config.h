#ifndef APP_CONFIG_H
#define APP_CONFIG_H
#include "../../USER/vehicle_parameters.h"
#include "zdt_emm_config.h"
#include "app_entry_config.h"
#if VEHICLE_INTEGRATED_ENABLE && APP_IS_COMPETITION
#error Integrated_field_debug_is_not_verified_competition_profile
#endif
#define APP_PERIMETER_TRIAL_ACTIVE 1U
#define APP_PERIMETER_GROUP_MS 300000U
#include "route_config.h"
/* Explicit single-turn debug candidate. Ordinary rotation remains capped at5RPM. */
#ifndef APP_TURN_BRAKE_TRIAL_ENABLE
#define APP_TURN_BRAKE_TRIAL_ENABLE 1U
#endif
#if APP_TURN_BRAKE_TRIAL_ENABLE > 1U
#error Turn_brake_trial_enable_invalid
#endif
#define APP_TURN_BRAKE_TRIAL_ACTIVE (APP_TURN_BRAKE_TRIAL_ENABLE && APP_ROUTE_USE_CUSTOM && APP_IS_DEBUG && (APP_TEST_MODE == 3U))
#ifndef APP_TURN_BRAKE_TRIAL_DPS
#define APP_TURN_BRAKE_TRIAL_DPS 30U
#endif
#if APP_TURN_BRAKE_TRIAL_DPS != 15U && APP_TURN_BRAKE_TRIAL_DPS != 30U
#error Turn_brake_trial_speed_invalid
#endif
#define APP_TURN_BRAKE_MAX_RPM ((APP_TURN_BRAKE_TRIAL_DPS == 30U) ? 27U : 13U)
#define APP_TURN_BRAKE_FEEDBACK_RPM ((APP_TURN_BRAKE_TRIAL_DPS == 30U) ? 40U : 30U)

/* 低速距离S曲线候选，默认关闭；仅调试mode3，仍不超过15RPM。
 * 下列数值为软件候选，未做载物/刹车实测，不作为提速许可。 */
#ifndef APP_DISTANCE_SCURVE_ENABLE
#define APP_DISTANCE_SCURVE_ENABLE 0U
#endif
#if APP_DISTANCE_SCURVE_ENABLE > 1U
#error Distance_scurve_enable_invalid
#endif
#define APP_DISTANCE_SCURVE_ACTIVE ((APP_DISTANCE_SCURVE_ENABLE || (APP_ROUTE_USE_CUSTOM && APP_ROUTE_SPEED_TRIAL_ENABLE)) && APP_IS_DEBUG && (APP_TEST_MODE == 3U))
#if (APP_ROUTE_SPEED_TRIAL_ENABLE != 0U) && (APP_ROUTE_SPEED_TRIAL_ENABLE != 1U)
#error Route_speed_trial_enable_invalid
#endif
#define APP_ROUTE_SPEED_TRIAL_ACTIVE (APP_ROUTE_SPEED_TRIAL_ENABLE && APP_ROUTE_USE_CUSTOM && APP_DISTANCE_SCURVE_ACTIVE)
/* 左移地面标定候选：默认关闭，仅新路线调试构建启用。 */
#ifndef APP_LEFT_TRIM_ENABLE
#define APP_LEFT_TRIM_ENABLE 0U
#endif
#if APP_LEFT_TRIM_ENABLE > 1U
#error Left_trim_enable_invalid
#endif
#define APP_LEFT_TRIM_ACTIVE (APP_LEFT_TRIM_ENABLE && APP_ROUTE_SPEED_TRIAL_ACTIVE)
#define APP_LEFT_DISTANCE_SCALE 0.99761905f
#define APP_LEFT_BACKWARD_RATIO 0.01670644f
#define APP_DISTANCE_SCURVE_ACCEL_MM_S2 40.0f
#define APP_DISTANCE_SCURVE_JERK_MM_S3 80.0f
#define APP_DISTANCE_SCURVE_TRACK_MM 50.0f
#define APP_DISTANCE_SCURVE_ARRIVAL_MM 2.0f
#define APP_DISTANCE_SCURVE_FINAL_WAIT_MS 2000U
/* 位置跟随候选：P目标经过二阶临界阻尼平滑，单位RPM。
 * 低速预留1RPM及A/jerk余量；20cm/s专用余量见motion_trial20.h。
 * 末端接近为整数1RPM，不能据此宣称实际轮速jerk连续。 */
#define APP_DISTANCE_SCURVE_FOLLOW_KP 0.5f
#define APP_DISTANCE_SCURVE_FOLLOW_RPM 1.0f
#define APP_DISTANCE_SCURVE_FOLLOW_OMEGA 1.0f
#define APP_DISTANCE_SCURVE_APPROACH_RPM 1U
/* 20cm/s单条调试预设，默认关闭；不得用于定时/横移/比赛入口。 */
#ifndef APP_SPEED20_TRIAL_ENABLE
#define APP_SPEED20_TRIAL_ENABLE 0U
#endif
#if APP_SPEED20_TRIAL_ENABLE > 1U
#error Speed20_trial_enable_invalid
#endif
#define APP_SPEED20_TRIAL_ACTIVE (APP_SPEED20_TRIAL_ENABLE && APP_DISTANCE_SCURVE_ACTIVE)
#define APP_SPEED20_TRIAL_RPM 52U
#define APP_SPEED20_TRIAL_MM 2100.0f
#define APP_SPEED20_TRIAL_MM_S 200.0f
#define APP_SPEED20_FEEDBACK_RPM 80U
#define APP_SPEED20_RUN_MS 25000U

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
/* APP_ENTRY / APP_DEBUG_MODE见app_entry_config.h，不再在此修改。 */
#if APP_TEST_MODE > 3U
#error App_test_mode_invalid
#endif
#define APP_UART_LINK_TEST (APP_IS_DEBUG && (APP_TEST_MODE == 1U))
/* 串口平移开发模式：CRC遥测、两步启动、1秒通信失效取消。 */
#define APP_SERIAL_MOTION_CONTROL (APP_IS_DEBUG && ((APP_TEST_MODE == 2U) || (APP_TEST_MODE == 3U)))
/* 位置诊断只读：0可关闭采样/PM1输出，不改变TM1和运动入口。 */
#define APP_POSITION_DIAGNOSTIC (APP_SERIAL_MOTION_CONTROL || APP_IS_COMPETITION)
#if (APP_POSITION_DIAGNOSTIC != 0U) && (APP_POSITION_DIAGNOSTIC != 1U)
#error Position_diagnostic_invalid
#endif
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

/* Offline experiment. Not a vehicle-ready firmware or persistent motor configuration. */
#define APP_FINE_RPM_OFFLINE_ONLY 0U
#define APP_FINE_RPM_VEHICLE 1U
