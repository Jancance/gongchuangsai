#ifndef HEADING_SOURCE_H
#define HEADING_SOURCE_H
#include <stdint.h>

typedef struct
{
    uint32_t sequence, sample_ms, angle_ms, rate_ms, healthy, valid_mask;
    float yaw_deg, rate_dps;
} AppHeadingSource;

/* One consistent snapshot; return value is publication readiness, not motion permission. */
uint32_t App_HeadingSourceRead(AppHeadingSource *snapshot);
void App_HeadingSourcePublish(void);
void App_HeadingSourceObserve(void);

static __inline uint32_t App_HeadingFrameAge(const AppHeadingSource *s,uint32_t now,uint32_t bit)
{ return (s->valid_mask & bit) ? now-((bit==1U)?s->angle_ms:s->rate_ms) : 0xFFFFFFFFU; }

extern volatile uint32_t g_heading_source_valid, g_heading_source_sequence;
extern volatile uint32_t g_heading_source_age_ms, g_heading_source_angle_age_ms, g_heading_source_rate_age_ms;
extern volatile float g_heading_source_yaw_deg, g_heading_source_rate_dps;
#endif
