#ifndef ZONE2_NAV_PATH_H
#define ZONE2_NAV_PATH_H

#include "fusion_reference.h"
#define ZONE2_NAV_LEGS 8U
#define ZONE2_STATION_COUNT 6U

typedef struct {
    uint32_t run_id,map_generation,layout_id;
    uint8_t start_id;
} Zone2NavContext;

typedef struct {
    Zone2NavContext context;
    uint8_t valid;
} Zone2NavMap;

typedef struct { uint8_t start,goal; float start_heading_deg,goal_heading_deg; } Zone2NavLeg;

typedef struct {
    FusionConfig config;
    FusionPoint actual,terminal;
    float speed_mm_s;
} Zone2NavGeometry;

typedef struct {
    float travel_heading_deg;
    int32_t first_deg,second_deg;
} Zone2NavTurns;

typedef struct {
    FusionInput reference;
    uint32_t visited_mask;
    Zone2NavTurns preferred,alternate;
    uint8_t has_alternate;
    uint8_t return_stop_node; /* 0=none; 1=configured raw parking return. */
    uint8_t raw_direct; /* Validated raw-departure actual-to-terminal candidate. */
    uint8_t entry_needed; /* Verified translation from actual pose to a turn-clear point. */
    FusionPoint entry_from,entry_to;
} Zone2NavRoute;

typedef enum {
    ZONE2_NAV_OK=0,ZONE2_NAV_INPUT,ZONE2_NAV_STALE_MAP,ZONE2_NAV_IDENTITY,
    ZONE2_NAV_STATUS,ZONE2_NAV_FORMAT,ZONE2_NAV_TERMINAL,ZONE2_NAV_POSE_CELL,
    ZONE2_NAV_OCCUPIED,ZONE2_NAV_REPEAT,ZONE2_NAV_NON_AXIS,ZONE2_NAV_CAPACITY,
    ZONE2_NAV_GEOMETRY,ZONE2_NAV_COINCIDENT
} Zone2NavPathResult;

uint32_t Zone2NavPath_Leg(uint8_t start_id,uint32_t index,Zone2NavLeg *leg);
uint32_t Zone2NavPath_Station(uint8_t grid_id,FusionPoint *point);
/* Build the fixed field route directly from the current measured position.
 * No radar map, health frame or PATH transaction is synthesized. */
Zone2NavPathResult Zone2NavPath_PrepareLocal(const Zone2NavMap *map,uint32_t leg_index,
    const Zone2NavGeometry *geometry,Zone2NavRoute *route,FusionPlan *plan);
/* Rebuild at most once after a valid raw-direct candidate fails motion checks. */
Zone2NavPathResult Zone2NavPath_PrepareReturn(const Zone2NavMap *map,uint32_t leg_index,
    const Zone2NavGeometry *geometry,Zone2NavRoute *route,FusionPlan *plan);
uint32_t Zone2NavPath_RawDirectValid(const Zone2NavRoute *route,const FusionPlan *plan);
/* Only a declared raw-departure node at current configured parking may stop. */
uint32_t Zone2NavPath_ReturnStopValid(const Zone2NavRoute *route,const FusionPlan *plan);
/* stopped must be independent caller evidence. Budgets bound the supplied measured pose. */
uint32_t Zone2NavPath_InsideReturn(uint8_t start_id,FusionPoint pose,float heading_deg,float position_budget_mm,
    float heading_budget_deg,uint32_t stopped);

#endif
