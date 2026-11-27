#ifndef ZONE2_NAV_PATH_H
#define ZONE2_NAV_PATH_H

#include "fusion_reference.h"
#include "radar_client.h"

#define ZONE2_NAV_LEGS 8U

typedef struct {
    uint32_t run_id,map_generation,layout_id,link_generation,scan_sequence;
    uint8_t start_id,node_id,zone;
} Zone2NavContext;

typedef struct {
    Zone2NavContext context;
    uint32_t occupied_mask;
    uint8_t valid;
} Zone2NavMap;

typedef struct {
    Zone2NavContext context;
    uint32_t path_id; /* New local reference identity for each leg, including cache hits. */
    uint16_t request_sequence;
    uint8_t start,goal,from_cache;
} Zone2NavRequest;

typedef struct {
    Zone2NavContext context; /* Context captured when the acknowledged PATH was received. */
    radar_client_path_result_t path;
} Zone2NavReply;

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
} Zone2NavRoute;

typedef enum {
    ZONE2_NAV_OK=0,ZONE2_NAV_INPUT,ZONE2_NAV_STALE_MAP,ZONE2_NAV_IDENTITY,
    ZONE2_NAV_STATUS,ZONE2_NAV_FORMAT,ZONE2_NAV_TERMINAL,ZONE2_NAV_POSE_CELL,
    ZONE2_NAV_OCCUPIED,ZONE2_NAV_REPEAT,ZONE2_NAV_NON_AXIS,ZONE2_NAV_CAPACITY,
    ZONE2_NAV_GEOMETRY,ZONE2_NAV_COINCIDENT
} Zone2NavPathResult;

uint32_t Zone2NavPath_Leg(uint8_t start_id,uint32_t index,Zone2NavLeg *leg);
uint32_t Zone2NavPath_Station(uint8_t grid_id,FusionPoint *point);
/* Call only with a completed GRID_SCAN callback for the supplied context. */
uint32_t Zone2NavPath_SetMap(const Zone2NavContext *context,const radar_client_grid_result_t *grid,Zone2NavMap *map);
/* Matches the directed cache key. The local path_id is per use, not part of the cache key. */
uint32_t Zone2NavPath_Matches(const Zone2NavRequest *request,const Zone2NavReply *reply);
/* Caller owns distinct route scratch and plan output. Only OK yields a valid
 * desired reference. COINCIDENT leaves the plan invalid for separate station evaluation. */
Zone2NavPathResult Zone2NavPath_Prepare(const Zone2NavMap *map,const Zone2NavRequest *request,
    const Zone2NavReply *reply,const Zone2NavGeometry *geometry,Zone2NavRoute *route,FusionPlan *plan);
/* stopped must be independent caller evidence. Budgets bound the supplied measured pose. */
uint32_t Zone2NavPath_InsideReturn(uint8_t start_id,FusionPoint pose,float heading_deg,float position_budget_mm,
    float heading_budget_deg,uint32_t stopped);

#endif
