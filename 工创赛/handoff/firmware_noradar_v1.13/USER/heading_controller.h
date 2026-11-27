#ifndef HEADING_CONTROLLER_H
#define HEADING_CONTROLLER_H

#include <stdint.h>

/* 航向闭环当前只输出带符号的逻辑转向轮速，不直接操作CAN或VM。 */
#define HEADING_CTRL_IDLE       0U
#define HEADING_CTRL_TURNING    1U
#define HEADING_CTRL_SETTLING   2U
#define HEADING_CTRL_DONE       3U
#define HEADING_CTRL_FAULT      4U
#define HEADING_CTRL_CANCELLED  5U

#define HEADING_CAUSE_NONE            0U
#define HEADING_CAUSE_INVALID_CONFIG  1U
#define HEADING_CAUSE_SENSOR_INVALID  2U
#define HEADING_CAUSE_TIMEOUT         3U
#define HEADING_CAUSE_CANCELLED       4U

typedef struct
{
    float tolerance_deg;
    float medium_error_deg;
    float coarse_error_deg;
    float settle_rate_dps;
    uint16_t fine_rpm;
    uint16_t medium_rpm;
    uint16_t coarse_rpm;
    uint32_t settle_ms;
    uint32_t timeout_ms;
} HeadingControllerConfig;

typedef struct
{
    HeadingControllerConfig config;
    uint32_t state;
    uint32_t cause;
    uint32_t start_tick;
    uint32_t settle_tick;
    uint32_t settle_active;
    float target_deg;
    float error_deg;
    int32_t command_rpm; /* 正值为俯视逆时针，负值为顺时针。 */
} HeadingController;

typedef struct
{
    uint32_t state;
    uint32_t cause;
    uint32_t stop_required;
    float target_deg;
    float error_deg;
    int32_t command_rpm;
} HeadingControllerOutput;

/* 5/3/1RPM、2度、连续稳定700ms均为待实车调参的工程候选值。 */
extern const HeadingControllerConfig g_heading_controller_default_config;

float HeadingController_WrapDeg(float angle_deg);
float HeadingController_ErrorDeg(float target_deg, float current_deg);
uint32_t HeadingController_ConfigValid(const HeadingControllerConfig *config);
uint32_t HeadingController_Start(HeadingController *controller,
    const HeadingControllerConfig *config, float target_deg,
    float current_deg, uint32_t now_tick);
uint32_t HeadingController_Step(HeadingController *controller,
    uint32_t now_tick, float yaw_deg, float yaw_rate_dps,
    uint32_t sensor_valid, HeadingControllerOutput *output);
void HeadingController_Cancel(HeadingController *controller,
    HeadingControllerOutput *output);

#endif
