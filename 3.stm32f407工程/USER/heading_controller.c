#include "heading_controller.h"
#include <float.h>
#include <stddef.h>

#define HEADING_CONTROLLER_MAX_RPM 10U
#define HEADING_CONTROLLER_INPUT_LIMIT_DEG 3600.0f

const HeadingControllerConfig g_heading_controller_default_config =
{
    2.0f, 8.0f, 25.0f, 3.0f,
    1U, 3U, 5U, 700U, 20000U
};

static uint32_t Heading_Finite(float value)
{
    return ((value >= -FLT_MAX) && (value <= FLT_MAX)) ? 1U : 0U;
}

static float Heading_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

float HeadingController_WrapDeg(float angle_deg)
{
    /* HWT正常范围为±180度；拒绝异常大值，避免归一化循环无界执行。 */
    if (!Heading_Finite(angle_deg) ||
        (Heading_Abs(angle_deg) > HEADING_CONTROLLER_INPUT_LIMIT_DEG)) return 0.0f;
    while (angle_deg >= 180.0f) angle_deg -= 360.0f;
    while (angle_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

float HeadingController_ErrorDeg(float target_deg, float current_deg)
{
    if (!Heading_Finite(target_deg) || !Heading_Finite(current_deg) ||
        (Heading_Abs(target_deg) > HEADING_CONTROLLER_INPUT_LIMIT_DEG) ||
        (Heading_Abs(current_deg) > HEADING_CONTROLLER_INPUT_LIMIT_DEG)) return 0.0f;
    /* 先分别归一化，避免相减后的大角度被误判为零误差 */
    target_deg = HeadingController_WrapDeg(target_deg);
    current_deg = HeadingController_WrapDeg(current_deg);
    return HeadingController_WrapDeg(target_deg - current_deg);
}

uint32_t HeadingController_ConfigValid(const HeadingControllerConfig *config)
{
    if ((config == NULL) || !Heading_Finite(config->tolerance_deg) ||
        !Heading_Finite(config->medium_error_deg) ||
        !Heading_Finite(config->coarse_error_deg) ||
        !Heading_Finite(config->settle_rate_dps)) return 0U;
    if ((config->tolerance_deg <= 0.0f) ||
        (config->medium_error_deg <= config->tolerance_deg) ||
        (config->coarse_error_deg <= config->medium_error_deg) ||
        (config->coarse_error_deg > 180.0f) ||
        (config->settle_rate_dps < 0.0f)) return 0U;
    if ((config->fine_rpm == 0U) ||
        (config->medium_rpm < config->fine_rpm) ||
        (config->coarse_rpm < config->medium_rpm) ||
        (config->coarse_rpm > HEADING_CONTROLLER_MAX_RPM)) return 0U;
    if ((config->settle_ms == 0U) || (config->timeout_ms <= config->settle_ms))
        return 0U;
    return 1U;
}

static void Heading_Output(const HeadingController *controller,
                           HeadingControllerOutput *output)
{
    if (output == NULL) return;
    output->state = controller->state;
    output->cause = controller->cause;
    output->target_deg = controller->target_deg;
    output->error_deg = controller->error_deg;
    output->command_rpm = controller->command_rpm;
    output->stop_required = ((controller->state == HEADING_CTRL_SETTLING) ||
        (controller->state == HEADING_CTRL_DONE) ||
        (controller->state == HEADING_CTRL_FAULT) ||
        (controller->state == HEADING_CTRL_CANCELLED)) ? 1U : 0U;
}

static void Heading_Fault(HeadingController *controller, uint32_t cause,
                          HeadingControllerOutput *output)
{
    controller->state = HEADING_CTRL_FAULT;
    controller->cause = cause;
    controller->command_rpm = 0;
    controller->settle_active = 0U;
    Heading_Output(controller, output);
}

uint32_t HeadingController_Start(HeadingController *controller,
    const HeadingControllerConfig *config, float target_deg,
    float current_deg, uint32_t now_tick)
{
    if (controller == NULL) return 0U;
    controller->state = HEADING_CTRL_FAULT;
    controller->cause = HEADING_CAUSE_INVALID_CONFIG;
    controller->command_rpm = 0;
    controller->settle_active = 0U;
    controller->target_deg = 0.0f;
    controller->error_deg = 0.0f;
    controller->start_tick = now_tick;
    controller->settle_tick = now_tick;
    if (!HeadingController_ConfigValid(config) ||
        !Heading_Finite(target_deg) || !Heading_Finite(current_deg) ||
        (Heading_Abs(target_deg) > HEADING_CONTROLLER_INPUT_LIMIT_DEG) ||
        (Heading_Abs(current_deg) > HEADING_CONTROLLER_INPUT_LIMIT_DEG)) return 0U;
    controller->config = *config;
    controller->target_deg = HeadingController_WrapDeg(target_deg);
    controller->error_deg = HeadingController_ErrorDeg(
        controller->target_deg, current_deg);
    controller->state = HEADING_CTRL_TURNING;
    controller->cause = HEADING_CAUSE_NONE;
    return 1U;
}

uint32_t HeadingController_Step(HeadingController *controller,
    uint32_t now_tick, float yaw_deg, float yaw_rate_dps,
    uint32_t sensor_valid, HeadingControllerOutput *output)
{
    float magnitude;
    uint16_t rpm;
    if ((controller == NULL) || (output == NULL)) return 0U;
    if ((controller->state == HEADING_CTRL_DONE) ||
        (controller->state == HEADING_CTRL_FAULT) ||
        (controller->state == HEADING_CTRL_CANCELLED) ||
        (controller->state == HEADING_CTRL_IDLE))
    {
        Heading_Output(controller, output);
        return 1U;
    }
    if ((sensor_valid != 1U) || !Heading_Finite(yaw_deg) ||
        !Heading_Finite(yaw_rate_dps) ||
        (Heading_Abs(yaw_deg) > HEADING_CONTROLLER_INPUT_LIMIT_DEG) ||
        (Heading_Abs(yaw_rate_dps) > HEADING_CONTROLLER_INPUT_LIMIT_DEG))
    {
        Heading_Fault(controller, HEADING_CAUSE_SENSOR_INVALID, output);
        return 1U;
    }
    if ((uint32_t)(now_tick - controller->start_tick) >=
        controller->config.timeout_ms)
    {
        Heading_Fault(controller, HEADING_CAUSE_TIMEOUT, output);
        return 1U;
    }
    controller->error_deg = HeadingController_ErrorDeg(
        controller->target_deg, yaw_deg);
    magnitude = Heading_Abs(controller->error_deg);
    if (magnitude <= controller->config.tolerance_deg)
    {
        controller->state = HEADING_CTRL_SETTLING;
        controller->command_rpm = 0;
        if (Heading_Abs(yaw_rate_dps) > controller->config.settle_rate_dps)
        {
            controller->settle_active = 0U;
        }
        else
        {
            if (controller->settle_active == 0U)
            {
                controller->settle_active = 1U;
                controller->settle_tick = now_tick;
            }
            if ((uint32_t)(now_tick - controller->settle_tick) >=
                controller->config.settle_ms)
            {
                controller->state = HEADING_CTRL_DONE;
            }
        }
        Heading_Output(controller, output);
        return 1U;
    }
    controller->settle_active = 0U;
    controller->state = HEADING_CTRL_TURNING;
    if (magnitude > controller->config.coarse_error_deg)
        rpm = controller->config.coarse_rpm;
    else if (magnitude > controller->config.medium_error_deg)
        rpm = controller->config.medium_rpm;
    else
        rpm = controller->config.fine_rpm;
    controller->command_rpm = (controller->error_deg > 0.0f) ?
        (int32_t)rpm : -(int32_t)rpm;
    Heading_Output(controller, output);
    return 1U;
}

void HeadingController_Cancel(HeadingController *controller,
    HeadingControllerOutput *output)
{
    if (controller == NULL) return;
    controller->state = HEADING_CTRL_CANCELLED;
    controller->cause = HEADING_CAUSE_CANCELLED;
    controller->command_rpm = 0;
    controller->settle_active = 0U;
    Heading_Output(controller, output);
}

/* 原始角速度与HWT yaw同向，阻尼项取负；只计算，不操作CAN/VM。 */
uint32_t HeadingController_HoldCorrection(float target_deg, float yaw_deg,
    float rate_dps, float *correction_rpm, float *error_deg)
{
    float error, command;
    if ((correction_rpm == NULL) || (error_deg == NULL)) return 0U;
    *correction_rpm = 0.0f;
    *error_deg = 0.0f;
    if (!Heading_Finite(target_deg) || !Heading_Finite(yaw_deg) ||
        !Heading_Finite(rate_dps) || (Heading_Abs(rate_dps) > 2000.0f) ||
        (Heading_Abs(target_deg) > 180.0f) ||
        (Heading_Abs(yaw_deg) > 180.0f)) return 0U;
    error = HeadingController_ErrorDeg(target_deg, yaw_deg);
    *error_deg = error;
    if (Heading_Abs(error) > 10.0f) return 0U;
    command = 0.8f * error - 0.25f * rate_dps;
    if ((Heading_Abs(error) <= 0.3f) && (Heading_Abs(rate_dps) <= 0.3f)) command = 0.0f;
    if (command > 2.0f) command = 2.0f;
    if (command < -2.0f) command = -2.0f;
    *correction_rpm = command;
    return 1U;
}

/* 仅新鲜源帧调用；80ms角速度低通、10RPM/s限斜率、整数命令迟滞。 */
uint32_t HeadingController_HoldStep(HeadingHoldController *hold,
    float target_deg, float yaw_deg, float rate_dps, uint32_t now_ms,
    int32_t *correction_rpm, float *error_deg)
{
    uint32_t elapsed;
    float desired, filtered, step, difference;
    if ((hold == NULL) || (correction_rpm == NULL) || (error_deg == NULL)) return 0U;
    *correction_rpm = 0;
    if (HeadingController_HoldCorrection(target_deg, yaw_deg, rate_dps,
        &desired, error_deg) != 1U) return 0U;
    elapsed = (hold->initialized != 0U) ? now_ms - hold->tick : 20U;
    if (elapsed >= 100U) return 0U;
    if ((hold->initialized != 0U) && (elapsed == 0U))
    { *correction_rpm = hold->quantized_rpm; return 1U; }
    filtered = (hold->initialized != 0U) ? hold->filtered_rate_dps +
        ((float)elapsed / (80.0f + (float)elapsed)) *
        (rate_dps - hold->filtered_rate_dps) : rate_dps;
    if (HeadingController_HoldCorrection(target_deg, yaw_deg, filtered,
        &desired, error_deg) != 1U) return 0U;
    step = 0.01f * (float)elapsed;
    difference = desired - hold->command_rpm;
    if (difference > step) difference = step;
    if (difference < -step) difference = -step;
    hold->command_rpm += difference;
    /* 0.65RPM越级，回落到0.35RPM才返回，减少量化边缘抖动。 */
    if ((hold->command_rpm - (float)hold->quantized_rpm) > 0.65f)
        hold->quantized_rpm++;
    else if ((hold->command_rpm - (float)hold->quantized_rpm) < -0.65f)
        hold->quantized_rpm--;
    hold->filtered_rate_dps = filtered;
    hold->tick = now_ms;
    hold->initialized = 1U;
    *correction_rpm = hold->quantized_rpm;
    return 1U;
}
