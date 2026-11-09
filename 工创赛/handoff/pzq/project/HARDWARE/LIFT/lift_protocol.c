#include "lift_protocol.h"
#include "lift_config.h"
#include "can.h"
uint8_t LiftProtocol_Move(uint8_t direction, uint32_t pulses, uint8_t out[15])
{
    return (uint8_t)CAN1_ArmXBuildMove(5U, direction, pulses, LIFT_X_SPEED_DECI_RPM,
                                     LIFT_X_ACCEL_RPM_S, LIFT_X_DECEL_RPM_S, out);
}
