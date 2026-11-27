#include "motion_profile.h"
#include <float.h>
#include <math.h>
#include <stddef.h>

static uint32_t Profile_Finite(float value)
{ return value >= -FLT_MAX && value <= FLT_MAX; }
static float Profile_Min(float a, float b) { return a < b ? a : b; }
static float Profile_Abs(float a) { return a < 0.0f ? -a : a; }
static uint32_t Profile_LimitsValid(const MotionProfileLimits *limits)
{
    return limits && Profile_Finite(limits->speed) &&
        Profile_Finite(limits->acceleration) && Profile_Finite(limits->jerk) &&
        limits->speed >= 0.001f && limits->speed <= 10000.0f &&
        limits->acceleration >= 0.001f && limits->acceleration <= 100000.0f &&
        limits->jerk >= 0.001f && limits->jerk <= 1000000.0f;
}

uint32_t MotionProfile_LimitTranslation(const ChassisGeometry *geometry,
    float direction_x, float direction_y, const MotionProfileLimits *requested,
    const MotionProfileWheelLimits *wheel, const MotionProfileWheelLimits *reserve,
    MotionProfileLimits *limited, float *time_scale)
{
    MotionProfileLimits result;
    ChassisBodyVelocity body;
    ChassisWheelSpeeds coefficients;
    float norm, scale = 1.0f, peak = 0.0f, unused, speed, acceleration, jerk;
    uint32_t i;
    if (time_scale) *time_scale = 0.0f;
    if (!limited) return 0U;
    /* 输入输出不得重叠；失败输出保持0，避免沿用先前有效参数。 */
    if (limited == requested) return 0U;
    limited->speed = limited->acceleration = limited->jerk = 0.0f;
    if (!time_scale || !Profile_LimitsValid(requested) || !wheel || !reserve ||
        !Profile_Finite(direction_x) || !Profile_Finite(direction_y) ||
        !Profile_Finite(wheel->speed_rpm) || !Profile_Finite(wheel->acceleration_rpm_s) ||
        !Profile_Finite(wheel->jerk_rpm_s2) || !Profile_Finite(reserve->speed_rpm) ||
        !Profile_Finite(reserve->acceleration_rpm_s) || !Profile_Finite(reserve->jerk_rpm_s2) ||
        reserve->speed_rpm < 0.0f || reserve->acceleration_rpm_s < 0.0f || reserve->jerk_rpm_s2 < 0.0f)
        return 0U;
    norm = sqrtf(direction_x * direction_x + direction_y * direction_y);
    if (!Profile_Finite(norm) || norm < 0.000001f) return 0U;
    speed = wheel->speed_rpm - reserve->speed_rpm;
    acceleration = wheel->acceleration_rpm_s - reserve->acceleration_rpm_s;
    jerk = wheel->jerk_rpm_s2 - reserve->jerk_rpm_s2;
    if (speed <= 0.0f || acceleration <= 0.0f || jerk <= 0.0f) return 0U;
    body.vx_mm_s = direction_x / norm;
    body.vy_mm_s = direction_y / norm;
    body.wz_rad_s = 0.0f;
    if (!Chassis_Inverse(geometry, &body, FLT_MAX, &coefficients, &unused)) return 0U;
    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
        if (Profile_Abs(coefficients.rpm[i]) > peak) peak = Profile_Abs(coefficients.rpm[i]);
    if (!Profile_Finite(peak) || peak <= 0.0f) return 0U;
    scale = Profile_Min(scale, speed / (peak * requested->speed));
    scale = Profile_Min(scale, sqrtf(acceleration / (peak * requested->acceleration)));
    scale = Profile_Min(scale, powf(jerk / (peak * requested->jerk), 1.0f / 3.0f));
    if (!Profile_Finite(scale) || scale <= 0.0f) return 0U;
    result.speed = requested->speed * scale;
    result.acceleration = requested->acceleration * scale * scale;
    result.jerk = requested->jerk * scale * scale * scale;
    if (!Profile_LimitsValid(&result)) return 0U;
    *limited = result; *time_scale = scale;
    return 1U;
}

uint32_t MotionProfile_Prepare(MotionProfile *plan, float distance_mm,
    const MotionProfileLimits *limits)
{
    static const float signs[MOTION_PROFILE_PHASES] = {1,0,-1,0,-1,0,1};
    float tj, ta, tv, peak, minimum_distance, phase_time[MOTION_PROFILE_PHASES];
    double position = 0.0, speed = 0.0, acceleration = 0.0, time = 0.0;
    uint32_t i;
    if (!plan) return 0U;
    plan->valid = 0U;
    if (!Profile_LimitsValid(limits) || !Profile_Finite(distance_mm) ||
        distance_mm < 0.0f || distance_mm > 1000000.0f ||
        (distance_mm > 0.0f && distance_mm < 0.01f)) return 0U;
    plan->distance_mm = distance_mm; plan->limits = *limits;
    plan->duration_s = plan->peak_speed = 0.0f;
    for (i = 0U; i < MOTION_PROFILE_PHASES; i++)
    {
        plan->boundary_s[i] = plan->position[i] = plan->speed[i] = 0.0f;
        plan->acceleration[i] = plan->jerk[i] = 0.0f;
    }
    plan->boundary_s[MOTION_PROFILE_PHASES] = 0.0f;
    if (distance_mm == 0.0f) { plan->valid = 1U; return 1U; }
    peak = limits->speed;
    tj = Profile_Min(limits->acceleration / limits->jerk, sqrtf(peak / limits->jerk));
    ta = peak / (limits->jerk * tj) - tj;
    if (ta < 0.0f) ta = 0.0f;
    minimum_distance = peak * (2.0f * tj + ta);
    tv = 0.0f;
    if (distance_mm >= minimum_distance)
        tv = (distance_mm - minimum_distance) / peak;
    else
    {
        /* 加速与减速对称；短段可能去掉匀速、匀加速和匀减速阶段。 */
        tj = limits->acceleration / limits->jerk;
        if (distance_mm >= 2.0f * limits->acceleration * tj * tj)
            ta = 0.5f * (sqrtf(tj * tj + 4.0f * distance_mm / limits->acceleration) - 3.0f * tj);
        else
        {
            tj = powf(distance_mm / (2.0f * limits->jerk), 1.0f / 3.0f);
            ta = 0.0f;
        }
        peak = limits->jerk * tj * (tj + ta);
    }
    if (!Profile_Finite(tj) || !Profile_Finite(ta) || !Profile_Finite(tv) ||
        tj <= 0.0f || ta < 0.0f || tv < 0.0f || !Profile_Finite(peak) ||
        peak > limits->speed * 1.00001f) return 0U;
    phase_time[0] = phase_time[2] = phase_time[4] = phase_time[6] = tj;
    phase_time[1] = phase_time[5] = ta; phase_time[3] = tv;
    for (i = 0U; i < MOTION_PROFILE_PHASES; i++)
    {
        double dt = phase_time[i], j = signs[i] * limits->jerk;
        plan->boundary_s[i] = (float)time;
        plan->position[i] = (float)position; plan->speed[i] = (float)speed;
        plan->acceleration[i] = (float)acceleration; plan->jerk[i] = (float)j;
        position += speed * dt + 0.5 * acceleration * dt * dt + j * dt * dt * dt / 6.0;
        speed += acceleration * dt + 0.5 * j * dt * dt;
        acceleration += j * dt; time += dt;
    }
    plan->duration_s = plan->boundary_s[MOTION_PROFILE_PHASES] = (float)time;
    if (!Profile_Finite(plan->duration_s) || plan->duration_s <= 0.0f ||
        plan->duration_s > 3600.0f || Profile_Abs((float)position-distance_mm) >
        0.002f + distance_mm * 0.00001f) return 0U;
    plan->peak_speed = peak; plan->valid = 1U;
    return 1U;
}

uint32_t MotionProfile_Sample(const MotionProfile *plan, float elapsed_s,
    MotionProfileSample *sample)
{
    uint32_t i;
    float dt, position, speed, acceleration;
    if (!sample) return MOTION_PROFILE_INVALID;
    sample->position_mm = sample->speed_mm_s = sample->acceleration_mm_s2 = sample->jerk_mm_s3 = 0.0f;
    if (!plan || plan->valid != 1U || !Profile_Finite(elapsed_s) || elapsed_s < 0.0f ||
        !Profile_Finite(plan->duration_s) || plan->duration_s < 0.0f ||
        !Profile_Finite(plan->distance_mm) || plan->distance_mm < 0.0f)
        return MOTION_PROFILE_INVALID;
    if (elapsed_s >= plan->duration_s)
    { sample->position_mm = plan->distance_mm; return MOTION_PROFILE_DONE; }
    for (i = 0U; i < MOTION_PROFILE_PHASES; i++)
        if (elapsed_s < plan->boundary_s[i+1U]) break;
    if (i >= MOTION_PROFILE_PHASES) return MOTION_PROFILE_INVALID;
    dt = elapsed_s - plan->boundary_s[i];
    position = plan->position[i] + plan->speed[i] * dt +
        0.5f * plan->acceleration[i] * dt * dt + plan->jerk[i] * dt * dt * dt / 6.0f;
    speed = plan->speed[i] + plan->acceleration[i] * dt + 0.5f * plan->jerk[i] * dt * dt;
    acceleration = plan->acceleration[i] + plan->jerk[i] * dt;
    if (!Profile_Finite(position) || !Profile_Finite(speed) || !Profile_Finite(acceleration) ||
        !Profile_Finite(plan->jerk[i])) return MOTION_PROFILE_INVALID;
    sample->position_mm = position; sample->speed_mm_s = speed;
    sample->acceleration_mm_s2 = acceleration; sample->jerk_mm_s3 = plan->jerk[i];
    return MOTION_PROFILE_RUNNING;
}
