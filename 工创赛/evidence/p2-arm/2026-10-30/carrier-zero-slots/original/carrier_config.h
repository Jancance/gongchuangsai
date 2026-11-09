#ifndef CARRIER_CONFIG_H
#define CARRIER_CONFIG_H

/* 用户确认：PB1、5V、与原270度舵机工程同款；原工程标称映射。 */
#define CARRIER_ANGLE_MAX_DEG       270.0f
#define CARRIER_PULSE_MIN_US        500U
#define CARRIER_PULSE_MAX_US        2500U
#define CARRIER_PERIOD_US           20000U
/* 首次须脱开盘机构，以下仅为调试范围，不是实测机械限位/槽位。 */
#define CARRIER_TEST_MIN_DEG        120.0f
#define CARRIER_TEST_MAX_DEG        150.0f
#define CARRIER_TEST_ANGLE_DEG      135.0f
#define CARRIER_TEST_WINDOW_MS      1500U
#define CARRIER_SERVICE_GAP_MS      500U

#endif
