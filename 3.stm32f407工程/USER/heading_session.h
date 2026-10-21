#ifndef HEADING_SESSION_H
#define HEADING_SESSION_H
#include "heading_controller.h"

/* 纯会话判定模块，无CAN、VM和任务调用；所有时间单位为毫秒。 */
#define HS_IDLE 0U
#define HS_RUNNING 1U
#define HS_STOPPING 2U
#define HS_DONE 3U
#define HS_FAULT 4U
#define HS_CANCELLED 5U
#define HS_ACTION_NONE 0U
#define HS_ACTION_BEGIN 1U
#define HS_ACTION_UPDATE 2U
#define HS_ACTION_STOP 3U
#define HS_ACTION_ABORT 4U
#define HS_ACTION_COMPLETE 5U
#define HS_CAUSE_INPUT 2U
#define HS_CAUSE_TIMEOUT 3U
#define HS_CAUSE_CANCEL 4U
#define HS_CAUSE_EXECUTOR 5U
#define HS_CAUSE_STOP_TIMEOUT 6U
#define HS_CAUSE_REVERSE 7U
#define HS_SOURCE_MS 100U
#define HS_RUN_MS 9000U
#define HS_STOP_MS 2500U

typedef struct
{
    uint32_t now_ms, sample_ms, sequence;
    uint32_t healthy, platform_ok;
    uint32_t backend_active, backend_result, backend_stop_mask;
    float yaw_deg, rate_dps;
} HeadingSessionInput;

typedef struct
{
    uint32_t state, cause, action;
    int32_t command_rpm;
    float target_deg, error_deg;
} HeadingSessionOutput;

typedef struct
{
    HeadingController controller;
    uint32_t state, cause, used;
    uint32_t last_sequence, last_new_ms, stop_ms;
    uint32_t begin_accepted, stop_accepted;
    int32_t initial_sign;
} HeadingSession;

void HeadingSession_Init(HeadingSession *session);
uint32_t HeadingSession_Start(HeadingSession *session,
    const HeadingSessionInput *input, float relative_deg, HeadingSessionOutput *out);
void HeadingSession_Step(HeadingSession *session,
    const HeadingSessionInput *input, HeadingSessionOutput *out);
void HeadingSession_ReportBegin(HeadingSession *session,
    uint32_t accepted, HeadingSessionOutput *out);
void HeadingSession_ReportUpdate(HeadingSession *session,
    uint32_t accepted, HeadingSessionOutput *out);
void HeadingSession_ReportStop(HeadingSession *session,
    uint32_t accepted, HeadingSessionOutput *out);
void HeadingSession_Cancel(HeadingSession *session, HeadingSessionOutput *out);
#endif
