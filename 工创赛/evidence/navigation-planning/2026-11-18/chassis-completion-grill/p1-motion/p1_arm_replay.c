#include "fusion_motion.h"
#include <math.h>

typedef struct {
    unsigned time_ms;
    float qaj[9];
} TraceRow;

typedef struct {
    FusionMotionVector q, a, target;
    unsigned duration_ms;
    float distance, angle;
    FusionMotionVector control[4];
    TraceRow samples[5];
    unsigned raw[5][4], directions[5][4];
} VectorRow;

#include "p1_arm_frozen.inc"

volatile unsigned replay_assertions, replay_case, replay_detail;
volatile float replay_actual, replay_expected;

#define CHECK(condition) do { ++replay_assertions; if (!(condition)) return __LINE__; } while (0)
#define CLOSE(actual, expected, tolerance) do { \
    replay_actual = (actual); replay_expected = (expected); \
    CHECK(fabsf(replay_actual - replay_expected) <= (tolerance)); \
} while (0)

static const FusionMotionLimits * const limits = &g_fusion_motion_limits;
static const FusionMotionVector zero = {0, 0, 0};

static unsigned sample_matches(const FusionMotionCurve *curve, unsigned elapsed, const TraceRow *row)
{
    FusionMotionSample sample;
    float actual[9];
    unsigned i;
    CHECK(FusionMotion_Sample(curve, elapsed, &sample));
    actual[0] = sample.q.x; actual[1] = sample.q.y; actual[2] = sample.q.yaw;
    actual[3] = sample.a.x; actual[4] = sample.a.y; actual[5] = sample.a.yaw;
    actual[6] = sample.j.x; actual[7] = sample.j.y; actual[8] = sample.j.yaw;
    for (i = 0; i < 9; ++i) {
        CLOSE(actual[i], row->qaj[i], i % 3 == 2 ? 0.00003f : 0.003f);
    }
    return 0;
}

static unsigned run_vectors(void)
{
    FusionMotionCurve curve;
    FusionMotionSample sample;
    ChassisBodyVelocity body;
    ChassisWheelSpeeds wheel;
    ChassisMotorCommand command[4];
    unsigned i, j, k, error;
    for (i = 0; i < sizeof(vectors)/sizeof(vectors[0]); ++i) {
        const VectorRow *v = &vectors[i];
        replay_case = i;
        CHECK(FusionMotion_Prepare(v->q, v->a, v->target, 20, limits, &curve) == !!v->duration_ms);
        CLOSE(curve.duration_ms, v->duration_ms, 0);
        if (!v->duration_ms) continue;
        CLOSE(curve.distance_mm, v->distance, 0.002f);
        CLOSE(curve.rotation_rad, v->angle, 0.00001f);
        for (j = 0; j < 4; ++j) {
            CLOSE(curve.control[j].x, v->control[j].x, 0.0001f);
            CLOSE(curve.control[j].y, v->control[j].y, 0.0001f);
            CLOSE(curve.control[j].yaw, v->control[j].yaw, 0.000001f);
        }
        for (j = 0; j < 5; ++j) {
            replay_detail = j;
            error = sample_matches(&curve, v->samples[j].time_ms, &v->samples[j]);
            if (error) return error;
            CHECK(FusionMotion_Sample(&curve, v->samples[j].time_ms, &sample));
            body.vx_mm_s = sample.q.x; body.vy_mm_s = sample.q.y; body.wz_rad_s = sample.q.yaw;
            CHECK(Chassis_Inverse(&g_chassis_geometry, &body, 300, &wheel, 0));
            CHECK(Chassis_EncodeEmm(&wheel, 300, command));
            for (k = 0; k < 4; ++k) {
                CLOSE(command[k].speed_raw, v->raw[j][k], 0);
                CLOSE(command[k].direction, v->directions[j][k], 0);
            }
        }
        CHECK(!FusionMotion_Sample(&curve, curve.duration_ms + 1, &sample));
    }
    return 0;
}

static const FusionMotionRequest request = {
    35, {0xfffffff1, 3, 14, 12, 18}, 100, 5, 10, 28, 20,
    1, 0, 1000, 60, 25, 3.14159265358979323846f,
    2, 0.001f, 0.2f, 0.001f, 0, 0
};

static unsigned reserve_matches(const FusionMotionReservation *out, const float expected[6])
{
    CLOSE(out->curve.duration_ms, expected[0], 0);
    CLOSE(out->held_ms, expected[1], 0);
    CLOSE(out->distance_mm, expected[2], 0.002f);
    CLOSE(out->back_mm, expected[3], 0.002f);
    CLOSE(out->lateral_mm, expected[4], 0.002f);
    CLOSE(out->rotation_rad, expected[5], 0.00001f);
    return 0;
}

static unsigned run_reservation(void)
{
    const FusionMotionVector q = {350, 0, 0}, a = {90, 0, 0};
    const FusionMotionVector turn = {0, 0, 1.57079632679489661923f};
    FusionMotionRequest r = request;
    FusionMotionReservation out;
    unsigned error;
    replay_case = 100;
    CHECK(FusionMotion_Reserve(q, a, &r, limits, &out) == FUSION_MOTION_OK);
    error = reserve_matches(&out, brake);
    if (error) return error;
    r.remaining_mm = brake[2] - 0.001f;
    CHECK(FusionMotion_Reserve(q, a, &r, limits, &out) == FUSION_MOTION_DISTANCE);
    r.remaining_mm = brake[2] + 0.001f;
    CHECK(FusionMotion_Reserve(q, a, &r, limits, &out) == FUSION_MOTION_OK);
    r.batch_ms = 200;
    CHECK(FusionMotion_Reserve(q, a, &r, limits, &out) == FUSION_MOTION_DISTANCE);
    r = request; r.lateral_mm = 1;
    CHECK(FusionMotion_Reserve(q, a, &r, limits, &out) == FUSION_MOTION_LATERAL);
    r = request; r.source_ms[0] = 0xffffffbe;
    CHECK(FusionMotion_Reserve(q, a, &r, limits, &out) == FUSION_MOTION_STALE);
    r.source_ms[0] = r.now_ms + 1;
    CHECK(FusionMotion_Reserve(q, a, &r, limits, &out) == FUSION_MOTION_STALE);
    r = request; r.mixed_lost = 1;
    CHECK(FusionMotion_Reserve(q, a, &r, limits, &out) == FUSION_MOTION_PARTIAL);
    r = request; r.angle_rad = 0.08f;
    CHECK(FusionMotion_Reserve(turn, zero, &r, limits, &out) == FUSION_MOTION_ANGLE);
    r.angle_rad = turn.yaw;
    CHECK(FusionMotion_Reserve(turn, zero, &r, limits, &out) == FUSION_MOTION_OK);
    r.turn_sign = 1;
    CHECK(FusionMotion_Reserve((FusionMotionVector){0, 0, 0.005f},
          (FusionMotionVector){0, 0, -3.14159265358979323846f}, &r, limits, &out) == FUSION_MOTION_REVERSE);
    r.turn_sign = -1;
    CHECK(FusionMotion_Reserve((FusionMotionVector){0, 0, -turn.yaw}, zero, &r, limits, &out) == FUSION_MOTION_OK);
    r = request;
    CHECK(FusionMotion_Reserve((FusionMotionVector){-200, 0, 0}, zero, &r, limits, &out) == FUSION_MOTION_BACK);
    r.mixed_mm = -0.02f;
    CHECK(FusionMotion_Reserve(q, a, &r, limits, &out) == FUSION_MOTION_INPUT);
    r = request; r.compute_ms = 0xffffffffU;
    CHECK(FusionMotion_Reserve(q, a, &r, limits, &out) == FUSION_MOTION_INPUT);
    return 0;
}

static unsigned run_proposal(void)
{
    const FusionMotionVector q = {180, 0, 0}, a = {10, 0, 0};
    FusionMotionRequest r = request;
    FusionMotionProposal out;
    unsigned error;
    replay_case = 101;
    CHECK(FusionMotion_Propose(q, a, proposal_target, &r, limits, &out) == FUSION_MOTION_ADVANCE);
    error = reserve_matches(&out.before, proposal_before);
    if (error) return error;
    error = reserve_matches(&out.after, proposal_after);
    if (error) return error;
    CLOSE(out.candidate.duration_ms, proposal_duration_ms, 0);
    CLOSE(out.prefix_mm, proposal_prefix_mm, 0.002f);
    CLOSE(out.prefix.q.x, proposal_prefix_q.x, 0.0001f);
    CLOSE(out.prefix.q.yaw, proposal_prefix_q.yaw, 0.000001f);
    CLOSE(out.prefix.a.x, proposal_prefix_a.x, 0.002f);
    CLOSE(out.prefix.a.yaw, proposal_prefix_a.yaw, 0.000001f);
    r.remaining_mm = proposal_before[2] + 0.01f;
    CHECK(FusionMotion_Propose(q, a, proposal_target, &r, limits, &out) == FUSION_MOTION_BRAKE);
    CHECK(out.reason == FUSION_MOTION_DISTANCE && out.before.reason == FUSION_MOTION_OK);
    r.remaining_mm = proposal_before[2] - 0.01f;
    CHECK(FusionMotion_Propose(q, a, proposal_target, &r, limits, &out) == FUSION_MOTION_FAULT);
    r = request;
    CHECK(FusionMotion_Propose(q, a, (FusionMotionVector){401, 0, 0}, &r, limits, &out) == FUSION_MOTION_BRAKE);
    CHECK(out.reason == FUSION_MOTION_NO_CANDIDATE);
    return 0;
}

#ifdef REPLAY_PREFIX_ERRORS
static unsigned run_prefix_errors(void)
{
    const FusionMotionVector target = {20, 0, 0}, turn_target = {20, 0, 0.06f};
    FusionMotionRequest r = request;
    FusionMotionProposal out;
    unsigned i;
    replay_case = 105;
    for (i = 0; i < 5; ++i) r.source_ms[i] = r.now_ms;
    r.compute_ms = r.queue_ms = r.batch_ms = 0;
    r.dt_ms = 200;
    r.mixed_mm = r.mixed_rad = 0;
    r.feedback_xy_mm_s = 10;
    r.feedback_yaw_rad_s = 0;
    CHECK(FusionMotion_Propose(zero, zero, target, &r, limits, &out) == FUSION_MOTION_ADVANCE);
    r.remaining_mm = probe_forward;
    CHECK(FusionMotion_Propose(zero, zero, target, &r, limits, &out) == FUSION_MOTION_BRAKE);
    CHECK(out.reason == FUSION_MOTION_DISTANCE);
    CLOSE(out.prefix_mm, probe_prefix_mm, 0.0001f);
    r.remaining_mm = request.remaining_mm; r.back_mm = probe_back;
    CHECK(FusionMotion_Propose(zero, zero, target, &r, limits, &out) == FUSION_MOTION_BRAKE);
    CHECK(out.reason == FUSION_MOTION_PREFIX_SPACE);
    r.back_mm = request.back_mm; r.lateral_mm = probe_lateral;
    CHECK(FusionMotion_Propose(zero, zero, target, &r, limits, &out) == FUSION_MOTION_BRAKE);
    CHECK(out.reason == FUSION_MOTION_PREFIX_SPACE);
    r.lateral_mm = request.lateral_mm;
    r.feedback_xy_mm_s = 0; r.feedback_yaw_rad_s = 0.2f;
    CHECK(FusionMotion_Propose(zero, zero, turn_target, &r, limits, &out) == FUSION_MOTION_ADVANCE);
    r.angle_rad = probe_angle;
    CHECK(FusionMotion_Propose(zero, zero, turn_target, &r, limits, &out) == FUSION_MOTION_BRAKE);
    CHECK(out.reason == FUSION_MOTION_ANGLE);
    r.angle_rad = request.angle_rad; r.lateral_mm = probe_yaw_tail;
    CHECK(FusionMotion_Propose(zero, zero, turn_target, &r, limits, &out) == FUSION_MOTION_BRAKE);
    CHECK(out.reason == FUSION_MOTION_PREFIX_SPACE);
    return 0;
}
#endif

#ifdef REPLAY_EXACT_CAPS
static unsigned run_exact_caps(void)
{
    FusionMotionLimits lowered = *limits;
    const FusionMotionVector q[4] = {{400, 0, 0}, {0, -400, 0}, {0, 0, limits->yaw_v}, {0, 0, -limits->yaw_v}};
    const FusionMotionVector a[4] = {{0.001f, 0, 0}, {0, -0.001f, 0}, {0, 0, 0.000001f}, {0, 0, -0.000001f}};
    const FusionMotionVector mixed = {240, 50, 0.08f};
    const ChassisBodyVelocity body = {mixed.x, mixed.y, mixed.yaw};
    ChassisWheelSpeeds wheels;
    FusionMotionCurve curve;
    FusionMotionSample sample;
    unsigned i;
    replay_case = 106;
    for (i = 0; i < 4; ++i) {
        FusionMotionVector inward = {-a[i].x, -a[i].y, -a[i].yaw};
        replay_detail = i;
        CHECK(!FusionMotion_Prepare(q[i], a[i], zero, 20, limits, &curve));
        CHECK(FusionMotion_Prepare(q[i], inward, zero, 20, limits, &curve));
    }
    CHECK(Chassis_Inverse(&g_chassis_geometry, &(ChassisBodyVelocity){240, 0, 0}, 300, &wheels, 0));
    lowered.wheel_rpm = wheels.rpm[0];
    CHECK(!FusionMotion_Prepare((FusionMotionVector){240, 0, 0}, a[0], zero, 20, &lowered, &curve));
    CHECK(FusionMotion_Prepare((FusionMotionVector){240, 0, 0},
          (FusionMotionVector){-0.001f, 0, 0}, zero, 20, &lowered, &curve));
    CHECK(FusionMotion_Prepare(q[0], zero, q[0], 20, limits, &curve));
    CHECK(FusionMotion_Prepare(q[0], zero, (FusionMotionVector){400, 60, 0}, 20, limits, &curve));
    for (i = 0; i <= curve.duration_ms; ++i) {
        replay_detail = i;
        CHECK(FusionMotion_Sample(&curve, i, &sample));
        CLOSE(sample.q.x, 400, 0);
    }
    CHECK(Chassis_Inverse(&g_chassis_geometry, &body, 300, &wheels, 0));
    lowered.wheel_rpm = wheels.rpm[CHASSIS_RIGHT_FRONT];
    CHECK(!FusionMotion_Prepare(mixed, a[0], zero, 20, &lowered, &curve));
    CHECK(FusionMotion_Prepare(mixed, (FusionMotionVector){-0.001f, 0, 0}, zero, 20, &lowered, &curve));
    return 0;
}
#endif

#ifdef REPLAY_STABLE_DERIVATIVES
static unsigned run_stable_derivatives(void)
{
    const FusionMotionVector q = {398, 0, 0}, a = {0.001f, 0, 0};
    const FusionMotionVector jerk_a = {-3.5980224609375f, 0, 0}, jerk_target = {397.9721374511719f, 0, 0};
    FusionMotionCurve curve;
    FusionMotionSample sample;
    replay_case = 107;
    CHECK(FusionMotion_Prepare(q, a, q, 20, limits, &curve));
    CLOSE(curve.duration_ms, 20, 0);
    CHECK(FusionMotion_Sample(&curve, 0, &sample));
    CLOSE(sample.a.x, a.x, 0);
    CHECK(FusionMotion_Sample(&curve, 10, &sample));
    CLOSE(sample.a.x, -a.x/4, 0);
    CHECK(FusionMotion_Sample(&curve, curve.duration_ms, &sample));
    CLOSE(sample.a.x, 0, 0);
    CHECK(FusionMotion_Prepare(q, jerk_a, jerk_target, 20, limits, &curve));
    CHECK(curve.duration_ms > 20);
    CHECK(FusionMotion_Sample(&curve, 0, &sample));
    CHECK(fabsf(sample.j.x) <= limits->translation_j);
    CHECK(FusionMotion_Sample(&curve, curve.duration_ms, &sample));
    CHECK(fabsf(sample.j.x) <= limits->translation_j);
    return 0;
}
#endif

static unsigned run_trajectory(unsigned id, const TraceRow *rows, unsigned count, FusionMotionVector cruise)
{
    const FusionMotionVector q = {180, 0, 0}, a = {10, 0, 0};
    FusionMotionCurve first, second;
    unsigned i, error;
    replay_case = 102 + id;
    if (id == 2) {
        CHECK(FusionMotion_Prepare(q, a, zero, 20, limits, &first));
    } else {
        CHECK(FusionMotion_Prepare(zero, zero, cruise, 20, limits, &first));
        CHECK(FusionMotion_Prepare(cruise, zero, zero, 20, limits, &second));
    }
    for (i = 0; i < count; ++i) {
        unsigned elapsed = rows[i].time_ms;
        replay_detail = i;
        error = elapsed <= first.duration_ms ? sample_matches(&first, elapsed, &rows[i]) :
                sample_matches(&second, elapsed - first.duration_ms, &rows[i]);
        if (error) return error;
    }
    return 0;
}

unsigned test_entry(void)
{
    unsigned result = run_vectors();
    if (result) return result;
#ifdef REPLAY_PREFIX_ERRORS
    result = run_prefix_errors();
    if (result) return result;
#endif
#ifdef REPLAY_EXACT_CAPS
    result = run_exact_caps();
    if (result) return result;
#endif
#ifdef REPLAY_STABLE_DERIVATIVES
    result = run_stable_derivatives();
    if (result) return result;
#endif
    result = run_reservation();
    if (result) return result;
    result = run_proposal();
    if (result) return result;
    result = run_trajectory(0, straight, sizeof(straight)/sizeof(straight[0]), (FusionMotionVector){400, 0, 0});
    if (result) return result;
    result = run_trajectory(1, diagonal, sizeof(diagonal)/sizeof(diagonal[0]), (FusionMotionVector){400, 400, 0});
    if (result) return result;
    return run_trajectory(2, short_stop, sizeof(short_stop)/sizeof(short_stop[0]), zero);
}
