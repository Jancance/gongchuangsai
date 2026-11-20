#ifndef RADAR_SERVO_CONFIG_H
#define RADAR_SERVO_CONFIG_H
/* 双180度PWM舵机。当前为待实物确认的标称映射，不是机械零点标定。 */
#define RADAR_SERVO_COUNT 2U
/* 用户确认的镜像安装：上电初始位置与出发目标分别配置。 */
#define RADAR_SERVO1_INIT_DEG 0.0f
#define RADAR_SERVO2_INIT_DEG 180.0f
#define RADAR_SERVO1_DEPLOY_DEG 180.0f
#define RADAR_SERVO2_DEPLOY_DEG 0.0f
/* 用户明确要求手动180度命令；仅执行上层显式请求，不自动扫动。 */
#define RADAR_SERVO_TEST_MAX_DEG 180.0f
#define RADAR_SERVO_MAX_DEG 180.0f
#define RADAR_SERVO_PERIOD_US 20000U
#define RADAR_SERVO_ZERO_US 500U
#define RADAR_SERVO_180_US 2500U
#define RADAR_ZONE_DEBOUNCE_MS 50U
/* PA1外部10k上拉：接地档为启停区1，悬空上拉档为启停区2。 */
#define RADAR_ZONE_LOW_ID 1U
#if RADAR_SERVO_ZERO_US < 400U || RADAR_SERVO_180_US > 2600U || RADAR_SERVO_ZERO_US >= RADAR_SERVO_180_US
#error Invalid_radar_servo_pulse_range
#endif
#endif
