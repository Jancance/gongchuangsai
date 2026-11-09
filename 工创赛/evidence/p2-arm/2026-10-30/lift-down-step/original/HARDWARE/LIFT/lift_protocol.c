#include "lift_protocol.h"
#include "lift_config.h"

/* Emm位置帧不含节点地址；mode2表示相对当前真实位置，不沿用旧目标。 */
uint8_t LiftProtocol_Move(uint8_t direction, uint32_t pulses, uint8_t out[12])
{
    if (out == 0 || direction > 1U || pulses == 0U ||
        pulses > (direction == 1U ? LIFT_UP_MAX_PULSES : LIFT_JOG_PULSES) || LIFT_JOG_RPM == 0U || LIFT_JOG_RPM > 3000U)
    {
        return 0U;
    }
    out[0] = 0xFDU; out[1] = direction;
    out[2] = (uint8_t)(LIFT_JOG_RPM >> 8);
    out[3] = (uint8_t)LIFT_JOG_RPM;
    out[4] = LIFT_JOG_ACCELERATION;
    out[5] = (uint8_t)(pulses >> 24);
    out[6] = (uint8_t)(pulses >> 16);
    out[7] = (uint8_t)(pulses >> 8);
    out[8] = (uint8_t)pulses;
    out[9] = 2U; out[10] = 0U; out[11] = 0x6BU;
    return 1U;
}
