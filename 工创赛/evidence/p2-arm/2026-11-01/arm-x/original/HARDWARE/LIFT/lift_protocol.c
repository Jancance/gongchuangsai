#include "lift_protocol.h"
#include "lift_config.h"

/* Emm位置帧不含节点地址；mode2表示相对当前真实位置，不沿用旧目标。 */
uint8_t LiftProtocol_Move(uint8_t direction, uint32_t pulses, uint8_t out[12])
{
    uint32_t rpm = (direction == 1U) ? LIFT_JOG_RPM : LIFT_DOWN_JOG_RPM;
    if (out == 0 || direction > 1U || pulses == 0U ||
        pulses > (direction == 1U ? LIFT_UP_MAX_PULSES : LIFT_DOWN_MAX_PULSES) || rpm == 0U || rpm > 3000U)
    {
        return 0U;
    }
    out[0] = 0xFDU; out[1] = direction;
    out[2] = (uint8_t)(rpm >> 8);
    out[3] = (uint8_t)rpm;
    out[4] = LIFT_JOG_ACCELERATION;
    out[5] = (uint8_t)(pulses >> 24);
    out[6] = (uint8_t)(pulses >> 16);
    out[7] = (uint8_t)(pulses >> 8);
    out[8] = (uint8_t)pulses;
    out[9] = 2U; out[10] = 0U; out[11] = 0x6BU;
    return 1U;
}
