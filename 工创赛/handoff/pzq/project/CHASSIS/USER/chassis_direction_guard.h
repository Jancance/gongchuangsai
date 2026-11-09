#ifndef CHASSIS_DIRECTION_GUARD_H
#define CHASSIS_DIRECTION_GUARD_H
#include <stdint.h>

/* User decision 2026-11-03: no reverse-sign fault or reverse timeout.
 * Keep successful command acknowledgement and sample metadata for telemetry.
 * Missing command acknowledgement remains a separate fault (detail 12).
 * Communication freshness, speed magnitude, position and stop checks live
 * in the backend and remain independent of feedback sign. */
#define DIRECTION_NO_TARGET 12U
typedef struct {
    int32_t target_deci;
    uint32_t valid, ack_ms, direction_ms, sample_ms, sample_seen;
    uint32_t reverse_samples; /* Legacy HF3 field: always zero. */
} ChassisDirectionGuard;

static __inline void DirectionGuard_Reset(ChassisDirectionGuard *s)
{ ChassisDirectionGuard empty = {0}; *s = empty; }

static __inline void DirectionGuard_Ack(ChassisDirectionGuard *s,
    int32_t target_deci, uint32_t now_ms)
{
    if (!s->valid || !s->target_deci || !target_deci ||
        ((s->target_deci < 0) != (target_deci < 0))) {
        s->direction_ms = now_ms;
        s->sample_seen = 0U;
    }
    s->target_deci = target_deci; s->ack_ms = now_ms; s->valid = 1U;
}

static __inline uint32_t DirectionGuard_Check(ChassisDirectionGuard *s,
    uint32_t now_ms, uint32_t sample_ms)
{
    if (!s->valid) return DIRECTION_NO_TARGET;
    if ((uint32_t)(now_ms-sample_ms) >= 0x80000000U ||
        (uint32_t)(sample_ms-s->direction_ms) >= 0x80000000U) return 0U;
    if (s->sample_seen && (uint32_t)(sample_ms-s->sample_ms) >= 0x80000000U) return 0U;
    s->sample_seen=1U; s->sample_ms=sample_ms;
    return 0U;
}
#endif
