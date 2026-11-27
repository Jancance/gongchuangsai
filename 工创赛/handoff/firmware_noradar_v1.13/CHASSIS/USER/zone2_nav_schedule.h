#ifndef ZONE2_NAV_SCHEDULE_H
#define ZONE2_NAV_SCHEDULE_H

#include "zone2_nav_path.h"
#include <stdbool.h>

typedef enum {
    ZONE2_SCHEDULE_IDLE=0,ZONE2_SCHEDULE_SCAN,ZONE2_SCHEDULE_REQUEST,ZONE2_SCHEDULE_REPLY,
    ZONE2_SCHEDULE_POSE,ZONE2_SCHEDULE_READY,ZONE2_SCHEDULE_ACTIVE,
    ZONE2_SCHEDULE_DONE,ZONE2_SCHEDULE_FAILED
} Zone2SchedulePhase;

/* Zero-initialize once in static storage. One navigation task owns all calls and
 * observes the phase before consuming the immutable local plan/route.
 * The navigation caller owns the overall run deadline, motor commands and fault stop. */
typedef struct {
    Zone2SchedulePhase phase;
    Zone2NavPathResult error;
    uint32_t run_id,layout_id,leg,session,prepared_sample_ms;
    uint8_t start_id;
    Zone2NavMap map;
    Zone2NavGeometry geometry;
    Zone2NavRoute route;
    FusionPlan plan;
} Zone2NavSchedule;

/* Once per latched PC13 trigger. Supply the locked area and known stowed reference. */
bool Zone2NavSchedule_Start(Zone2NavSchedule *s,uint32_t run_id,uint8_t start_id,
    uint32_t layout_id,bool stowed,const FusionConfig *config,float speed_mm_s);
/* Pass a current corrected wheel/HWT pose with this run's run_id, start_id and
 * layout_id. POSE waits for this sample; READY exposes a new desired plan. */
Zone2SchedulePhase Zone2NavSchedule_Poll(Zone2NavSchedule *s,const FusionObservation *pose,uint32_t now_ms);
/* Bind the new motion session to the ready plan before issuing motor commands. */
bool Zone2NavSchedule_BeginLeg(Zone2NavSchedule *s,const FusionIdentity *identity,uint32_t session,uint32_t now_ms);
/* Caller supplies independently determined station arrival and measured stopped
 * feedback. The observation must carry the active plan identity. */
bool Zone2NavSchedule_Arrive(Zone2NavSchedule *s,uint32_t session,bool station_arrived,
    const FusionObservation *pose,uint32_t now_ms);
bool Zone2NavSchedule_MapCurrent(const Zone2NavSchedule *s);
void Zone2NavSchedule_Abort(Zone2NavSchedule *s);

#if APP_ZONE2_POSE_TRACE_ENABLE
/* The navigation task starts and freezes route and pose diagnostics on the same clock. */
void Zone2NavSchedule_TraceStart(uint32_t now_ms);
void Zone2NavSchedule_TraceFreeze(const Zone2NavSchedule *s,uint32_t now_ms,
    uint32_t fault,uint32_t path_error,uint8_t final_phase);
#endif

#endif
