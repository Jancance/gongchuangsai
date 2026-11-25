#ifndef HEADING_CONTROLLER_H
#define HEADING_CONTROLLER_H

#include <stdint.h>

/* ����ջ���ǰֻ��������ŵ��߼�ת�����٣���ֱ�Ӳ���CAN��VM�� */
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
    float command_rpm; /* ��ֵΪ������ʱ�룬��ֵΪ˳ʱ�롣 */
} HeadingController;

typedef struct
{
    uint32_t state;
    uint32_t cause;
    uint32_t stop_required;
    float target_deg;
    float error_deg;
    float command_rpm;
} HeadingControllerOutput;

/* 5/3/1RPM��2�ȡ������ȶ�700ms��Ϊ��ʵ�����εĹ��̺�ѡֵ�� */
extern const HeadingControllerConfig g_heading_controller_default_config;

float HeadingController_WrapDeg(float angle_deg);
float HeadingController_ErrorDeg(float target_deg, float current_deg);
/* ǰ��PD��Kp=0.8 RPM/deg��Kd=0.25 RPM/(deg/s)�������޷�2RPM�� */
typedef struct
{
    uint32_t tick, initialized;
    float command_rpm, filtered_rate_dps;
    int32_t quantized_deci_rpm;
} HeadingHoldController;
uint32_t HeadingController_HoldCorrection(float target_deg, float yaw_deg,
    float rate_dps, float *correction_rpm, float *error_deg);
uint32_t HeadingController_HoldStep(HeadingHoldController *hold,
    float target_deg, float yaw_deg, float rate_dps, uint32_t now_ms,
    float *correction_rpm, float *error_deg);

uint32_t HeadingController_HoldStepRoute(HeadingHoldController *hold,
    float target_deg, float yaw_deg, float rate_dps, uint32_t now_ms,
    float kp, float kd, float max_correction_rpm,
    float *correction_rpm, float *error_deg);
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
