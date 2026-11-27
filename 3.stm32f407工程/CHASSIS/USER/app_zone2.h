#ifndef APP_ZONE2_H
#define APP_ZONE2_H

#include <stdint.h>

/* Key task latches one run request. The CAN task owns the navigation runtime. */
void AppZone2_RequestStart(void);
uint32_t AppZone2_CanPermit(uint32_t slot_ms);
void AppZone2_Step(void);

#if APP_ZONE2_MISSION_ENABLE
typedef enum {
    APP_ZONE2_SOURCE=1, APP_ZONE2_ROUGH, APP_ZONE2_STORAGE_BASE, APP_ZONE2_STORAGE_STACK
} AppZone2StationKind;

typedef enum {
    APP_ZONE2_STATION_PENDING=0, APP_ZONE2_STATION_VERIFIED=1,
    APP_ZONE2_STATION_FAILED=2, APP_ZONE2_STATION_ACTION_COMPLETE_UNVERIFIED=3
} AppZone2StationResult;

typedef enum {
    APP_ZONE2_SOURCE_GRIP=1, APP_ZONE2_SOURCE_SLOT, APP_ZONE2_ROUGH_PLACE,
    APP_ZONE2_ROUGH_GRIP, APP_ZONE2_ROUGH_SLOT, APP_ZONE2_STORAGE_PLACE,
    APP_ZONE2_ROUGH_SLOT_GRIP, APP_ZONE2_STORAGE_SLOT_GRIP
} AppZone2Action;

typedef struct {
    uint32_t ticket,run_id;
    uint8_t cell,batch,leg,kind;
    uint8_t colors[3],rings[3];
    char task_code[16];
} AppZone2Station;

/* Scanner and screen tasks communicate with the CAN-owned navigation state. */
void AppZone2_OfferTaskCode(const char *code,uint32_t first_ms,uint32_t last_ms);
void AppZone2_DisplayTaskCode(void);
uint32_t AppZone2_CopyTaskCode(char code[16]);
uint32_t AppZone2_StationRead(AppZone2Station *station);
/* 仅提前发布工位票据；交接前行驶底盘仍由原导航控制。 */
uint32_t AppZone2_StationApproaching(void);
uint32_t AppZone2_StationConfirm(uint32_t ticket,uint8_t item,AppZone2Action action,
    uint8_t color,uint8_t ring);
/* Called only at fresh, CRC/identity-checked O31 visual convergence, before pose token advances. */
uint32_t AppZone2_StationVisualAligned(uint32_t ticket,uint32_t vision_run,
 uint32_t vision_pose,uint32_t vision_sequence,uint32_t observation_ms);
uint32_t AppZone2_StationComplete(uint32_t ticket,uint32_t success);
uint32_t AppZone2_StationCompleteUnverified(uint32_t ticket);
#endif

#endif
