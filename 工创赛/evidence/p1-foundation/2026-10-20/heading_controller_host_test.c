#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "heading_controller.h"

static unsigned checks;
#define CHECK(x) do { checks++; if (!(x)) { \
    fprintf(stderr, "check failed at line %d: %s\n", __LINE__, #x); \
    return 1; } } while (0)
#define CLOSE(a,b,e) (fabs((double)(a) - (double)(b)) <= (e))

int main(void)
{
    HeadingController c;
    HeadingControllerOutput o;
    HeadingControllerConfig cfg = g_heading_controller_default_config;
    uint32_t t0 = 1000U;

    CHECK(CLOSE(HeadingController_WrapDeg(180.0f), -180.0f, 0.001));
    CHECK(CLOSE(HeadingController_WrapDeg(540.0f), -180.0f, 0.001));
    CHECK(CLOSE(HeadingController_WrapDeg(-181.0f), 179.0f, 0.001));
    CHECK(CLOSE(HeadingController_WrapDeg(4000.0f), 0.0f, 0.001));
    CHECK(CLOSE(HeadingController_ErrorDeg(-179.0f, 179.0f), 2.0f, 0.001));
    CHECK(CLOSE(HeadingController_ErrorDeg(179.0f, -179.0f), -2.0f, 0.001));
    CHECK(HeadingController_ConfigValid(&cfg) == 1U);

    CHECK(HeadingController_Start(&c, &cfg, 90.0f, 0.0f, t0) == 1U);
    CHECK(HeadingController_Step(&c, t0, 0.0f, 0.0f, 1U, &o) == 1U);
    CHECK(o.state == HEADING_CTRL_TURNING && o.command_rpm == 5);
    CHECK(o.stop_required == 0U && CLOSE(o.error_deg, 90.0f, 0.001));
    CHECK(HeadingController_Step(&c, t0 + 100U, 70.0f, 0.0f, 1U, &o) == 1U);
    CHECK(o.command_rpm == 3 && CLOSE(o.error_deg, 20.0f, 0.001));
    CHECK(HeadingController_Step(&c, t0 + 200U, 84.0f, 0.0f, 1U, &o) == 1U);
    CHECK(o.command_rpm == 1 && CLOSE(o.error_deg, 6.0f, 0.001));
    CHECK(HeadingController_Step(&c, t0 + 300U, 89.0f, 5.0f, 1U, &o) == 1U);
    CHECK(o.state == HEADING_CTRL_SETTLING && o.command_rpm == 0 && o.stop_required == 1U);
    CHECK(HeadingController_Step(&c, t0 + 500U, 89.0f, 0.0f, 1U, &o) == 1U);
    CHECK(o.state == HEADING_CTRL_SETTLING);
    CHECK(HeadingController_Step(&c, t0 + 1199U, 89.0f, 0.0f, 1U, &o) == 1U);
    CHECK(o.state == HEADING_CTRL_SETTLING);
    CHECK(HeadingController_Step(&c, t0 + 1200U, 89.0f, 0.0f, 1U, &o) == 1U);
    CHECK(o.state == HEADING_CTRL_DONE && o.stop_required == 1U);

    CHECK(HeadingController_Start(&c, &cfg, -90.0f, 0.0f, 0U) == 1U);
    CHECK(HeadingController_Step(&c, 1U, 0.0f, 0.0f, 1U, &o) == 1U);
    CHECK(o.command_rpm == -5);
    CHECK(HeadingController_Step(&c, 2U, -92.5f, 0.0f, 1U, &o) == 1U);
    CHECK(o.command_rpm == 1); /* 超调后反向微调。 */

    CHECK(HeadingController_Start(&c, &cfg, -179.0f, 179.0f, 0U) == 1U);
    CHECK(HeadingController_Step(&c, 1U, 179.0f, 0.0f, 1U, &o) == 1U);
    CHECK(o.command_rpm == 0 && CLOSE(o.error_deg, 2.0f, 0.001));
    CHECK(o.state == HEADING_CTRL_SETTLING);
    CHECK(HeadingController_Start(&c, &cfg, 170.0f, -170.0f, 0U) == 1U);
    CHECK(HeadingController_Step(&c, 1U, -170.0f, 0.0f, 1U, &o) == 1U);
    CHECK(o.command_rpm == -3 && CLOSE(o.error_deg, -20.0f, 0.001));

    CHECK(HeadingController_Start(&c, &cfg, 45.0f, 0.0f, 0U) == 1U);
    CHECK(HeadingController_Step(&c, 1U, 0.0f, 0.0f, 0U, &o) == 1U);
    CHECK(o.state == HEADING_CTRL_FAULT && o.cause == HEADING_CAUSE_SENSOR_INVALID);
    CHECK(o.command_rpm == 0 && o.stop_required == 1U);

    CHECK(HeadingController_Start(&c, &cfg, 4000.0f, 0.0f, 0U) == 0U);
    CHECK(HeadingController_Start(&c, &cfg, 45.0f, 0.0f, 0U) == 1U);
    CHECK(HeadingController_Step(&c, 1U, 4000.0f, 0.0f, 1U, &o) == 1U);
    CHECK(o.state == HEADING_CTRL_FAULT && o.cause == HEADING_CAUSE_SENSOR_INVALID);

    CHECK(HeadingController_Start(&c, &cfg, 45.0f, 0.0f, 0U) == 1U);
    CHECK(HeadingController_Step(&c, cfg.timeout_ms, 0.0f, 0.0f, 1U, &o) == 1U);
    CHECK(o.state == HEADING_CTRL_FAULT && o.cause == HEADING_CAUSE_TIMEOUT);

    CHECK(HeadingController_Start(&c, &cfg, 45.0f, 0.0f, UINT32_MAX - 99U) == 1U);
    CHECK(HeadingController_Step(&c, cfg.timeout_ms - 101U, 0.0f, 0.0f, 1U, &o) == 1U);
    CHECK(o.state == HEADING_CTRL_TURNING);
    CHECK(HeadingController_Step(&c, cfg.timeout_ms - 100U, 0.0f, 0.0f, 1U, &o) == 1U);
    CHECK(o.state == HEADING_CTRL_FAULT && o.cause == HEADING_CAUSE_TIMEOUT);

    CHECK(HeadingController_Start(&c, &cfg, 30.0f, 0.0f, 0U) == 1U);
    HeadingController_Cancel(&c, &o);
    CHECK(o.state == HEADING_CTRL_CANCELLED && o.cause == HEADING_CAUSE_CANCELLED);
    CHECK(o.command_rpm == 0 && o.stop_required == 1U);

    cfg.coarse_rpm = 11U;
    CHECK(HeadingController_ConfigValid(&cfg) == 0U);
    CHECK(HeadingController_Start(&c, &cfg, 0.0f, 0.0f, 0U) == 0U);
    cfg = g_heading_controller_default_config;
    cfg.medium_error_deg = cfg.tolerance_deg;
    CHECK(HeadingController_ConfigValid(&cfg) == 0U);
    cfg = g_heading_controller_default_config;
    cfg.timeout_ms = cfg.settle_ms;
    CHECK(HeadingController_ConfigValid(&cfg) == 0U);

    printf("PASS %u heading controller checks\n", checks);
    return 0;
}
