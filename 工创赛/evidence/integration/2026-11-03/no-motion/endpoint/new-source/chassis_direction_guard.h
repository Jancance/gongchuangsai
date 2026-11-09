#ifndef CHASSIS_DIRECTION_GUARD_H
#define CHASSIS_DIRECTION_GUARD_H
#include <stdint.h>

/* Physical units: signed 0.1 RPM, uint32 milliseconds. No IO or fault clearing.
 * Only an endpoint/deceleration window may qualify a tiny reverse sample.
 * A second fresh reverse, a large reverse, or a bounded timeout still trips. */
/* Emm reports integer RPM. This is a provisional terminal-only allowance,
 * not a claim that a negative sample is merely quantization. Endpoint gate,
 * low target <=2RPM, second-fresh reverse and 600ms bound stay enforced. */
#define DIRECTION_MICRO_MAX_DECI 20
#define DIRECTION_TARGET_MAX_DECI 20
#define DIRECTION_MICRO_TIMEOUT_MS 600U
#define DIRECTION_REVERSE 10U
#define DIRECTION_PERSISTENT 11U
#define DIRECTION_NO_TARGET 12U
typedef struct {
    int32_t target_deci;
    uint32_t valid, ack_ms, direction_ms, sample_ms, sample_seen;
    uint32_t pending, first_reverse_ms, reverse_samples;
} ChassisDirectionGuard;

static __inline void DirectionGuard_Reset(ChassisDirectionGuard *s)
{ ChassisDirectionGuard empty = {0}; *s = empty; }

/* Call only for an actual successful F6/STOP acknowledgement, never a queued
 * desired value or a cached acknowledgement. Repeated same-sign commands do
 * not renew the pending reverse deadline. */
static __inline void DirectionGuard_Ack(ChassisDirectionGuard *s,
    int32_t target_deci, uint32_t now_ms)
{
    if (!s->valid || !s->target_deci || !target_deci ||
        ((s->target_deci < 0) != (target_deci < 0))) {
        s->direction_ms = now_ms;
        s->sample_seen = s->pending = s->reverse_samples = 0U;
    }
    s->target_deci = target_deci; s->ack_ms = now_ms; s->valid = 1U;
}

static __inline uint32_t DirectionGuard_Expired(const ChassisDirectionGuard *s,
    uint32_t now_ms)
{ return s->pending && (uint32_t)(now_ms-s->first_reverse_ms) >= DIRECTION_MICRO_TIMEOUT_MS; }

static __inline uint32_t DirectionGuard_Check(ChassisDirectionGuard *s,
    uint32_t now_ms, uint32_t sample_ms, int32_t actual_deci, uint32_t endpoint_window)
{
    uint32_t reverse;
    if (!s->valid) return DIRECTION_NO_TARGET;
    if (DirectionGuard_Expired(s,now_ms)) return DIRECTION_PERSISTENT;
    /* Old samples from before an acknowledged direction change are not evidence
     * of reverse motion under the new target. */
    if ((uint32_t)(now_ms-sample_ms) >= 0x80000000U ||
        (uint32_t)(sample_ms-s->direction_ms) >= 0x80000000U) return 0U;
    if (s->sample_seen && (uint32_t)(sample_ms-s->sample_ms) >= 0x80000000U) return 0U;
    reverse = (s->target_deci>0 && actual_deci<0) || (s->target_deci<0 && actual_deci>0);
    if (reverse && (!endpoint_window || s->target_deci < -DIRECTION_TARGET_MAX_DECI ||
        s->target_deci > DIRECTION_TARGET_MAX_DECI ||
        actual_deci < -DIRECTION_MICRO_MAX_DECI || actual_deci > DIRECTION_MICRO_MAX_DECI))
        return DIRECTION_REVERSE;
    /* Repeated cached samples never confirm recovery or count twice. */
    if (s->sample_seen && sample_ms == s->sample_ms) return 0U;
    s->sample_seen=1U; s->sample_ms=sample_ms;
    reverse = (s->target_deci>0 && actual_deci<0) || (s->target_deci<0 && actual_deci>0);
    if (!reverse) { s->pending=s->reverse_samples=0U; return 0U; }
    if (s->pending) { s->reverse_samples++; return DIRECTION_PERSISTENT; }
    s->pending=1U; s->reverse_samples=1U; s->first_reverse_ms=now_ms;
    return 0U;
}
#endif
