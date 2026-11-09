#include "radar_servo_logic.h"
#include <stddef.h>
uint32_t RadarServo_Encode(uint32_t id, float angle_deg, uint16_t *pulse_us)
{
    float pulse;
    if (pulse_us == NULL || id < 1U || id > RADAR_SERVO_COUNT ||
        !(angle_deg >= 0.0f && angle_deg <= RADAR_SERVO_MAX_DEG)) return 0U;
    pulse = (float)RADAR_SERVO_ZERO_US + angle_deg *
        (float)(RADAR_SERVO_180_US-RADAR_SERVO_ZERO_US)/RADAR_SERVO_MAX_DEG;
    *pulse_us = (uint16_t)(pulse + 0.5f);
    return 1U;
}
void RadarZoneFilter_Init(RadarZoneFilter *f)
{
    if (f != NULL) *f = (RadarZoneFilter){0};
}
void RadarZoneFilter_Sample(RadarZoneFilter *f, uint32_t level, uint32_t now_ms)
{
    if (f == NULL || level > 1U) return;
    if (!f->sample_started || level != f->candidate_level) {
        f->sample_started=1U;f->candidate_level=level;f->candidate_ms=now_ms;
        f->ready=0U;return;
    }
    if ((uint32_t)(now_ms-f->candidate_ms) >= RADAR_ZONE_DEBOUNCE_MS) {
        f->zone = level == 0U ? RADAR_ZONE_LOW_ID : 3U-RADAR_ZONE_LOW_ID;
        f->ready=1U;
    }
}
uint32_t RadarZoneFilter_Lock(RadarZoneFilter *f)
{
    if (f == NULL) return 0U;
    if (f->locked) return f->locked_zone;
    if (!f->ready) return 0U;
    f->locked=1U;f->locked_zone=f->zone;return f->locked_zone;
}
void RadarZoneFilter_Unlock(RadarZoneFilter *f)
{
    if (f != NULL) {f->locked=0U;f->locked_zone=0U;}
}
