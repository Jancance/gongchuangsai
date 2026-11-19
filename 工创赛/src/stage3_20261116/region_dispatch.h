#ifndef REGION_DISPATCH_H
#define REGION_DISPATCH_H
#include <stdint.h>
/* Arrival is supplied by the upper route owner. The bench key is separate. */
extern volatile uint32_t g_region_dispatch_area,g_region_dispatch_rejected;
extern volatile uint32_t g_region_dispatch_test_enable;
uint32_t RegionDispatch_Arrive(uint32_t area);
void RegionDispatch_KeyEvent(void);
void RegionDispatch_ScanService(void);
#endif
