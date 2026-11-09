#ifndef CARRIER_CONFIG_H
#define CARRIER_CONFIG_H

/* PB1、5V、原工程同款270度位置舵机；以下为标称PWM映射。 */
#define CARRIER_ANGLE_MAX_DEG       270.0f
#define CARRIER_PULSE_MIN_US        500U
#define CARRIER_PULSE_MAX_US        2500U
#define CARRIER_PERIOD_US           20000U
/* 用户2026-10-30指定：0度装配1号槽，直连三槽120度等分。
 * 这些是待实物对齐的命令候选，不是已测机械角或到位反馈。
 * 3号回1号须240度反向到0度，不允许按360度循环累加。 */
#define CARRIER_SLOT_1_DEG          0.0f
#define CARRIER_SLOT_2_DEG          120.0f
#define CARRIER_SLOT_3_DEG          240.0f
/* 只有显式enable/request才输出；首次0度摆位须在固定盘之前完成。 */
#define CARRIER_TEST_MIN_DEG        CARRIER_SLOT_1_DEG
#define CARRIER_TEST_MAX_DEG        CARRIER_SLOT_3_DEG
#define CARRIER_TEST_ANGLE_DEG      CARRIER_SLOT_1_DEG
#define CARRIER_TEST_WINDOW_MS      1500U
#define CARRIER_SERVICE_GAP_MS      500U

#endif
