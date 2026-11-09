#ifndef SOURCE_OBSERVER_H
#define SOURCE_OBSERVER_H
#include "metric_alignment.h"
void SourceObserver_Init(const MetricAlignmentProfile*);
/* Task RX producer; service and all inbox access belong to Task A. */
void SourceObserver_Byte(uint8_t byte,uint32_t now_ms);
void SourceObserver_Service(uint32_t now_ms);
/* Latest bounded request, copied out for normal nonblocking UART telemetry. */
uint32_t SourceObserver_Request(char*out,uint32_t capacity);
extern volatile uint32_t g_source_observer_reason,g_source_observer_accepted;
extern volatile uint32_t g_source_observer_requests,g_source_observer_frames,g_source_observer_dropped;
extern volatile int32_t g_source_observer_dx_milli_mm,g_source_observer_dy_milli_mm;
#endif
