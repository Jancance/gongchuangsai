#ifndef FINE_RPM_TELEMETRY_H
#define FINE_RPM_TELEMETRY_H
#include <stdint.h>
/* TM6/BT6 commands and speed feedback use signed 0.1RPM. */
static __inline int32_t FineRpm_Deci(float rpm)
{
    if (!(rpm>=-300.0f && rpm<=300.0f)) return (-2147483647L-1L);
    return (int32_t)((rpm<0.0f) ? rpm*10.0f-0.5f : rpm*10.0f+0.5f);
}
#endif
