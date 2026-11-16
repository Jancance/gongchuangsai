#ifndef APP_FUSION_CAPTURE_H
#define APP_FUSION_CAPTURE_H
#include "app_config.h"
#ifndef APP_FUSION_CAPTURE_ENABLE
#define APP_FUSION_CAPTURE_ENABLE 1
#endif
#ifndef APP_FUSION_CAPTURE_STATIC_TEST
#define APP_FUSION_CAPTURE_STATIC_TEST 0
#endif
#if APP_FUSION_CAPTURE_STATIC_TEST && (!APP_BASELINE40_ACTIVE || !APP_FUSION_CAPTURE_ENABLE)
#error Static_capture_requires_baseline40_capture
#endif
#if APP_BASELINE40_ACTIVE
#include "icm45686_raw.h"
/* One boot, one observation only. No motion requests or sensor transactions. */
#if APP_FUSION_CAPTURE_ENABLE
void AppFusionCapture_Step(const IcmRawStatus *raw);
#if APP_FUSION_CAPTURE_STATIC_TEST
/* Explicit debugger request only: 0x53544154. Does not request motion or VM. */
extern volatile uint32_t g_fusion_static_request;
#endif
#else
#define AppFusionCapture_Step(raw) ((void)0)
#endif
#endif
#endif
