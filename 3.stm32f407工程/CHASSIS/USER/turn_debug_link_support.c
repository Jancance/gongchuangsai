#include "app_config.h"
#include "source_body_step.h"
#include "usart.h"
#include "vehicle_mission.h"

#if VEHICLE_FIELD_MODE == VEHICLE_FIELD_MODE_TURN_DEBUG && \
    !APP_ZONE2_NAV_ENABLE && !APP_ZONE2_MISSION_ENABLE && \
    !VEHICLE_INTEGRATED_ENABLE

/* Mission state is absent in this turn-only profile, so the safe state is idle. */
volatile uint32_t g_vehicle_state = 0U;
volatile uint32_t g_vehicle_run_id = 0U;
volatile uint32_t g_vehicle_key = 0U;

/* The source-body path is disabled. A stale call must fail its availability gate. */
volatile uint32_t g_source_body_step_request = 0U;
volatile uint32_t g_source_body_step_state = 0U;
volatile uint32_t g_source_body_step_reason = 0U;
volatile uint32_t g_source_body_step_session = 0U;
volatile uint32_t g_source_align_rejected = 0U;
volatile uint32_t g_source_align_chain_active = 0U;
volatile uint32_t g_source_align_command_seq = 0U;
volatile uint32_t g_source_align_direction = 0U;
volatile float g_source_align_distance_mm = 0.0f;
volatile float g_source_align_turn_deg = 0.0f;

uint32_t SourceBodyStep_WheelWindow(void)
{
    return 1U;
}

uint32_t SourceBodyStep_Busy(void)
{
    return 1U;
}

/* USART6 vision is not initialized in this profile. Reads and sends fail closed. */
volatile uint32_t g_uart6_rx_errors = 0U;

void USART6_VisionConfig(void)
{
}

void USART6_VisionEnableReceive(void)
{
}

uint8_t USART6_VisionReadByte(uint8_t *data)
{
    (void)data;
    return 0U;
}

uint8_t USART6_VisionSend(const uint8_t *data, uint16_t length)
{
    (void)data;
    (void)length;
    return 0U;
}

#endif
