#ifndef ZDT_EMM_CONFIG_H
#define ZDT_EMM_CONFIG_H
/* 离线工程禁止VM、使能及非零速度，不触碰驱动持久参数。 */
#ifndef APP_MOTOR_OFFLINE_ONLY
#define APP_MOTOR_OFFLINE_ONLY 1U
#endif
/* 四台驱动人工确认FW_Emm及S_Vel_IS=Enable后，再改1并重编译。
 * 未确认可停止/失能及只读查询，禁止使能和非零速度。 */
#ifndef ZDT_EMM_SETTINGS_CONFIRMED
#define ZDT_EMM_SETTINGS_CONFIRMED 0U
#endif
#if ZDT_EMM_SETTINGS_CONFIRMED > 1U
#error Invalid_Emm_settings_confirmation
#endif
#define ZDT_EMM_SPEED_INPUT_DIV10 1U
#if ZDT_EMM_SPEED_INPUT_DIV10 != 1U
#error Emm_requires_div10_speed_input
#endif
/* 保留原底盘300RPM软件上限，不因协议切换自动提速。 */
#define ZDT_EMM_APP_MAX_SPEED_RAW 3000U
#define ZDT_EMM_POSITION_UNITS_PER_REV 65536U
/* Emm加速度为档位，236档按手册公式名义约1000RPM/s。
 * 不可沿用原1000数值或截断成8位；实际带载斜率待验证。 */
#define ZDT_EMM_MOTION_ACCELERATION 236U
#endif
