#ifndef RADAR_SERVO_LOGIC_H
#define RADAR_SERVO_LOGIC_H
#include <stdint.h>
#include "radar_servo_config.h"
typedef struct {
    uint32_t sample_started, candidate_level, candidate_ms, ready, zone;
    uint32_t locked, locked_zone;
} RadarZoneFilter;
uint32_t RadarServo_Encode(uint32_t id, float angle_deg, uint16_t *pulse_us);
void RadarZoneFilter_Init(RadarZoneFilter *filter);
void RadarZoneFilter_Sample(RadarZoneFilter *filter, uint32_t level, uint32_t now_ms);
uint32_t RadarZoneFilter_Lock(RadarZoneFilter *filter);
/* 仅在上层确认本轮结束、机构停稳后解锁，不在运行中跟随开关。 */
void RadarZoneFilter_Unlock(RadarZoneFilter *filter);
#endif
