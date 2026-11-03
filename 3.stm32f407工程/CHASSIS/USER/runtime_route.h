#ifndef RUNTIME_ROUTE_H
#define RUNTIME_ROUTE_H
#include <stdint.h>
#include "motion_action_plan.h"

#define RQ_LINE_MAX 160U
#define RQ_UPLOAD_MS 10000U
#define RQ_BEGIN 1U
#define RQ_ROW 2U
#define RQ_COMMIT 3U
#define RQ_GO 4U
#define RQ_ABORT 5U
#define RQ_OK 0U
#define RQ_DUPLICATE 1U
#define RQ_BUSY 2U
#define RQ_STALE 3U
#define RQ_FORMAT 4U
#define RQ_ID 5U
#define RQ_RANGE 6U
#define RQ_MISSING 7U
#define RQ_DIGEST 8U
#define RQ_EXPIRED 9U
#define RQ_PLAN 10U

/* Translation: amount in 0.1 mm, speed in mm/s. Turn: signed millidegrees,
 * speed in millidegrees/s. Deadline includes preparation and stopping. */
typedef struct {
    uint32_t kind;
    int32_t amount;
    uint32_t speed, timeout_ms;
} RuntimeRouteRow;
typedef struct {
    uint32_t seq,tick,id,op,index,count,digest;
    RuntimeRouteRow row;
} RuntimeRouteMessage;
typedef struct {
    uint32_t id,digest,count,budget_ms;
    RuntimeRouteRow rows[MOTION_ACTION_CAPACITY];
    MotionAction actions[MOTION_ACTION_CAPACITY];
    float local_targets[MOTION_ACTION_CAPACITY+1U][3];
} RuntimeRoutePrepared;
typedef struct {
    uint32_t seen_id,stage_id,stage_count,stage_mask,stage_ms,staging;
    RuntimeRouteRow staged[MOTION_ACTION_CAPACITY];
    uint32_t ready,consumed,last_result;
    RuntimeRoutePrepared committed;
} RuntimeRoute;

uint16_t RuntimeRoute_Crc(const char *data,uint32_t length);
uint32_t RuntimeRoute_Decode(const char *line,RuntimeRouteMessage *message);
uint32_t RuntimeRoute_Digest(uint32_t id,uint32_t count,const RuntimeRouteRow *rows);
void RuntimeRoute_Init(RuntimeRoute *route);
void RuntimeRoute_Tick(RuntimeRoute *route,uint32_t now_ms);
uint32_t RuntimeRoute_Stage(RuntimeRoute *route,const RuntimeRouteMessage *message,uint32_t now_ms);
uint32_t RuntimeRoute_Prepare(const RuntimeRoute *route,const RuntimeRouteMessage *message,
    uint32_t now_ms,RuntimeRoutePrepared *prepared);
void RuntimeRoute_Install(RuntimeRoute *route,const RuntimeRoutePrepared *prepared);
uint32_t RuntimeRoute_Start(RuntimeRoute *route,uint32_t id,uint32_t digest);
#endif
