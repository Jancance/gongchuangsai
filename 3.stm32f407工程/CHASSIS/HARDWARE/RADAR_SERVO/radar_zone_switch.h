#ifndef RADAR_ZONE_SWITCH_H
#define RADAR_ZONE_SWITCH_H
#include <stdint.h>
extern volatile uint32_t g_radar_zone_ready,g_radar_zone_selected,g_radar_zone_locked;
void RadarZoneSwitch_Init(void);
void RadarZoneSwitch_Service(uint32_t now_ms);
uint32_t RadarZoneSwitch_Lock(void);
void RadarZoneSwitch_Unlock(void);
#endif
