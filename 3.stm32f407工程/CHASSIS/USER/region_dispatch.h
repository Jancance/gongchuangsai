#ifndef REGION_DISPATCH_H
#define REGION_DISPATCH_H
#include <stdint.h>
/* Arrival is supplied by the upper route owner. The bench key is separate. */
extern volatile uint32_t g_region_dispatch_area,g_region_dispatch_rejected;
extern volatile uint32_t g_region_dispatch_test_enable;
uint32_t RegionDispatch_Arrive(uint32_t area);
typedef enum {
 REGION_MODULE_SOURCE=1, REGION_MODULE_ROUGH, REGION_MODULE_STACK, REGION_MODULE_STORAGE_BASE
} RegionModule;
typedef enum { REGION_MODULE_UNKNOWN, REGION_MODULE_BUSY, REGION_MODULE_DONE,
 REGION_MODULE_STOPPING, REGION_MODULE_STOPPED, REGION_MODULE_FAULT } RegionModuleStatus;
typedef struct {
 uint32_t run_id, batch_id;
 RegionModule module;
 uint32_t colors[3], rings[3], slots[3];
} RegionModuleRequest;
/* Task context only. Allocate a fresh increasing run_id per invocation.
 * Begin copies caller inputs. CAN service consumes the request asynchronously. */
uint32_t RegionDispatch_Begin(const RegionModuleRequest *request);
RegionModuleStatus RegionDispatch_Poll(uint32_t run_id);
uint32_t RegionDispatch_Stop(uint32_t run_id);
uint32_t RegionDispatch_StopActive(void);
void RegionDispatch_KeyEvent(void);
void RegionDispatch_ScanService(void);
uint32_t RegionDispatch_WantsService(void);
#if APP_ZONE2_MISSION_ENABLE
void RegionDispatch_MapService(uint32_t now);
#endif
#endif
