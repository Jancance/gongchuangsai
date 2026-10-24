#include "heading_session.h"
#include <stddef.h>

static void HS_Out(const HeadingSession *s, uint32_t action, HeadingSessionOutput *out)
{
    out->state = s->state;
    out->cause = s->cause;
    out->action = action;
    out->command_rpm = (s->state == HS_RUNNING) ? s->controller.command_rpm : 0;
    out->target_deg = s->controller.target_deg;
    out->error_deg = s->controller.error_deg;
}

static void HS_Fail(HeadingSession *s, uint32_t cause, HeadingSessionOutput *out)
{
    s->state = HS_FAULT;
    s->cause = cause;
    s->controller.command_rpm = 0;
    HS_Out(s, HS_ACTION_ABORT, out);
}

static uint32_t HS_Fresh(const HeadingSessionInput *in)
{
    return ((in->healthy == 1U) && (in->platform_ok == 1U) &&
        ((uint32_t)(in->now_ms - in->sample_ms) < HS_SOURCE_MS)) ? 1U : 0U;
}

void HeadingSession_Init(HeadingSession *s)
{
    HeadingSession empty = {0};
    if (s != NULL) *s = empty;
}

uint32_t HeadingSession_Start(HeadingSession *s, const HeadingSessionInput *in,
    float relative_deg, HeadingSessionOutput *out)
{
    HeadingControllerConfig config = g_heading_controller_default_config;
    HeadingControllerOutput calculated = {0};
    if ((s == NULL) || (in == NULL) || (out == NULL)) return 0U;
    HS_Out(s, HS_ACTION_NONE, out);
    if ((s->used != 0U) || (s->state != HS_IDLE) || !HS_Fresh(in) ||
        (in->backend_active != 0U) || (in->backend_result != 0U) ||
        !(((relative_deg >= 5.0f) && (relative_deg <= ((APP_TEST_MODE == 3U) ? 90.0f : 45.0f))) ||
          ((relative_deg <= -5.0f) && (relative_deg >= ((APP_TEST_MODE == 3U) ? -90.0f : -45.0f))))) return 0U;

    /* 有限会话截止从接受会话开始计时，包含执行器准备耗时。 */
    config.timeout_ms = HS_RUN_MS;
    config.tolerance_deg = APP_HEADING_TOLERANCE_DEG;
    if (HeadingController_Start(&s->controller, &config,
        in->yaw_deg + relative_deg, in->yaw_deg, in->now_ms) != 1U) return 0U;
    (void)HeadingController_Step(&s->controller, in->now_ms,
        in->yaw_deg, in->rate_dps, 1U, &calculated);
    s->used = 1U;
    if (calculated.state == HEADING_CTRL_FAULT)
    {
        HS_Fail(s, HS_CAUSE_INPUT, out);
        return 0U;
    }
    s->state = HS_RUNNING;
    s->last_sequence = in->sequence;
    s->last_new_ms = in->now_ms;
    s->initial_sign = (relative_deg > 0.0f) ? 1 : -1;
    HS_Out(s, HS_ACTION_BEGIN, out);
    return 1U;
}

void HeadingSession_ReportBegin(HeadingSession *s, uint32_t accepted,
    HeadingSessionOutput *out)
{
    if ((s == NULL) || (out == NULL)) return;
    HS_Out(s, HS_ACTION_NONE, out);
    if (s->state != HS_RUNNING) return;
    if (accepted != 1U) HS_Fail(s, HS_CAUSE_EXECUTOR, out);
    else s->begin_accepted = 1U;
}

void HeadingSession_ReportUpdate(HeadingSession *s, uint32_t accepted,
    HeadingSessionOutput *out)
{
    if ((s == NULL) || (out == NULL)) return;
    HS_Out(s, HS_ACTION_NONE, out);
    if ((s->state == HS_RUNNING) && (accepted != 1U))
        HS_Fail(s, HS_CAUSE_EXECUTOR, out);
}

void HeadingSession_ReportStop(HeadingSession *s, uint32_t accepted,
    HeadingSessionOutput *out)
{
    if ((s == NULL) || (out == NULL)) return;
    HS_Out(s, HS_ACTION_NONE, out);
    if (s->state != HS_STOPPING) return;
    if (accepted != 1U) HS_Fail(s, HS_CAUSE_EXECUTOR, out);
    else s->stop_accepted = 1U;
}

void HeadingSession_Cancel(HeadingSession *s, HeadingSessionOutput *out)
{
    HeadingControllerOutput ignored = {0};
    if ((s == NULL) || (out == NULL)) return;
    if ((s->state == HS_RUNNING) || (s->state == HS_STOPPING))
    {
        HeadingController_Cancel(&s->controller, &ignored);
        s->state = HS_CANCELLED;
        s->cause = HS_CAUSE_CANCEL;
    }
    HS_Out(s, ((s->state == HS_CANCELLED) || (s->state == HS_FAULT)) ?
        HS_ACTION_ABORT : HS_ACTION_NONE, out);
}

void HeadingSession_Step(HeadingSession *s, const HeadingSessionInput *in,
    HeadingSessionOutput *out)
{
    HeadingControllerOutput calculated = {0};
    uint32_t delta;
    if ((s == NULL) || (in == NULL) || (out == NULL)) return;
    HS_Out(s, HS_ACTION_NONE, out);
    if ((s->state == HS_FAULT) || (s->state == HS_CANCELLED))
    { HS_Out(s, HS_ACTION_ABORT, out); return; }
    if (s->state == HS_DONE)
    { HS_Out(s, HS_ACTION_COMPLETE, out); return; }
    if (s->state == HS_IDLE) return;

    if (s->state == HS_STOPPING)
    {
        /* backend_result=1必须来自原四轮零速确认后的执行器结果。 */
        if ((s->stop_accepted == 1U) && (in->backend_active == 0U) &&
            (in->backend_result == 1U) && (in->backend_stop_mask == 0x0FU))
        { s->state = HS_DONE; HS_Out(s, HS_ACTION_COMPLETE, out); return; }
        if ((uint32_t)(in->now_ms - s->stop_ms) >= HS_STOP_MS)
        { HS_Fail(s, HS_CAUSE_STOP_TIMEOUT, out); return; }
        if ((in->platform_ok != 1U) ||
            ((in->backend_result != 0U) && (in->backend_result != 1U)))
        { HS_Fail(s, HS_CAUSE_EXECUTOR, out); return; }
        HS_Out(s, s->stop_accepted ? HS_ACTION_NONE : HS_ACTION_STOP, out);
        return;
    }

    if (!HS_Fresh(in) ||
        ((uint32_t)(in->now_ms - s->last_new_ms) >= HS_SOURCE_MS))
    { HS_Fail(s, HS_CAUSE_INPUT, out); return; }
    if (in->backend_result != 0U)
    { HS_Fail(s, HS_CAUSE_EXECUTOR, out); return; }
    if ((uint32_t)(in->now_ms - s->controller.start_tick) >= HS_RUN_MS)
    { HS_Fail(s, HS_CAUSE_TIMEOUT, out); return; }
    delta = in->sequence - s->last_sequence;
    if (delta == 0U)
    {
        /* 同序号不续期、不输出UPDATE，不能靠重读缓存维持运动。 */
        HS_Out(s, s->begin_accepted ? HS_ACTION_NONE : HS_ACTION_BEGIN, out);
        return;
    }
    if (delta > 0x7FFFFFFFU)
    { HS_Fail(s, HS_CAUSE_INPUT, out); return; }
    s->last_sequence = in->sequence;
    s->last_new_ms = in->now_ms;
    (void)HeadingController_Step(&s->controller, in->now_ms,
        in->yaw_deg, in->rate_dps, 1U, &calculated);
    if (calculated.state == HEADING_CTRL_FAULT)
    { HS_Fail(s, HS_CAUSE_INPUT, out); return; }
    if (((s->initial_sign > 0) && (calculated.command_rpm < 0)) ||
        ((s->initial_sign < 0) && (calculated.command_rpm > 0)))
    { HS_Fail(s, HS_CAUSE_REVERSE, out); return; }
    if (calculated.state == HEADING_CTRL_DONE)
    {
        s->state = HS_STOPPING;
        s->stop_ms = in->now_ms;
        HS_Out(s, HS_ACTION_STOP, out);
    }
    else
    {
        /* SETTLING只输出零速UPDATE，不能提前结束整个执行器会话。 */
        HS_Out(s, s->begin_accepted ? HS_ACTION_UPDATE : HS_ACTION_BEGIN, out);
    }
}
