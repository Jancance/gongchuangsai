#include <float.h>
#include <math.h>
#include <stdio.h>

#include "chassis_kinematics.h"
#include "zdt_emm_protocol.h"

/* Offline reference. Integer RPM columns assume the drive uses 1 RPM per unit. */
enum { SAMPLE_MS = 80, LIMIT_MM = 1920 };

static const struct {
    const char *name;
    int x;
    int y;
} directions[] = {
    {"forward", 1, 0}, {"reverse", -1, 0}, {"left", 0, 1}, {"right", 0, -1}
};

static int emit_sample(const char *direction, unsigned batch, int axis_x, int axis_y,
                       double position, double speed, double acceleration)
{
    ChassisBodyVelocity reference = {(float)(speed * axis_x), (float)(speed * axis_y), 0};
    ChassisBodyVelocity quantized;
    ChassisWheelSpeeds requested, integer;
    ChassisMotorCommand deci[CHASSIS_WHEEL_COUNT];
    unsigned motor_direction[CHASSIS_WHEEL_COUNT];
    unsigned requested_deci[CHASSIS_WHEEL_COUNT];
    const float protocol_limit_rpm = ZDT_EMM_APP_MAX_SPEED_RAW / 10.0f;
    float scale;
    unsigned deci_accepted, wheel;

    if (!Chassis_Inverse(&g_chassis_geometry, &reference, FLT_MAX, &requested, &scale) ||
        scale != 1.0f) return 0;

    deci_accepted = Chassis_EncodeEmm(&requested, (uint16_t)protocol_limit_rpm, deci);
    for (wheel = 0; wheel < CHASSIS_WHEEL_COUNT; ++wheel) {
        const float rpm = requested.rpm[wheel];
        const double magnitude = fabs(rpm);
        unsigned integer_magnitude = (unsigned)(magnitude + 0.5);
        uint8_t payload[7];

        requested_deci[wheel] = (unsigned)(magnitude * 10.0 + 0.5);
        integer.rpm[wheel] = rpm < 0.0f ? -(float)integer_magnitude : (float)integer_magnitude;
        motor_direction[wheel] = g_chassis_forward_direction[wheel] ^
                                 (unsigned)(rpm < 0.0f && integer_magnitude != 0U);
        if (deci_accepted && !ZdtEmm_EncodeSpeed(deci[wheel].direction, deci[wheel].speed_raw,
                                                ZDT_EMM_MOTION_ACCELERATION, 0U, payload)) return 0;
    }
    if (!Chassis_Forward(&g_chassis_geometry, &integer, &quantized)) return 0;

    printf("%s,%u,%u,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%u",
           direction, batch, batch * SAMPLE_MS, position, acceleration, speed,
           reference.vx_mm_s, reference.vy_mm_s, LIMIT_MM - position, deci_accepted);
    for (wheel = 0; wheel < CHASSIS_WHEEL_COUNT; ++wheel) printf(",%.6f", requested.rpm[wheel]);
    for (wheel = 0; wheel < CHASSIS_WHEEL_COUNT; ++wheel) printf(",%u", requested_deci[wheel]);
    for (wheel = 0; wheel < CHASSIS_WHEEL_COUNT; ++wheel) printf(",%.0f", integer.rpm[wheel]);
    for (wheel = 0; wheel < CHASSIS_WHEEL_COUNT; ++wheel) printf(",%u", motor_direction[wheel]);
    printf(",%.6f,%.6f\n", quantized.vx_mm_s, quantized.vy_mm_s);
    return 1;
}

int main(void)
{
    static const unsigned phase_steps[] = {3, 5, 3, 2, 3, 5, 3};
    static const int jerk_sign[] = {1, 0, -1, 0, -1, 0, 1};
    const double dt = SAMPLE_MS / 1000.0;
    const double peak_speed = 1600.0;
    const double peak_acceleration = peak_speed / ((phase_steps[0] + phase_steps[1]) * dt);
    const double jerk_magnitude = peak_acceleration / (phase_steps[0] * dt);
    unsigned direction;

    puts("direction,batch,time_ms,progress_mm,acceleration_mm_s2,speed_mm_s,vx_mm_s,vy_mm_s,"
         "line_remaining_mm,current_deci_accepted,"
         "addr1_logical_rpm,addr2_logical_rpm,addr3_logical_rpm,addr4_logical_rpm,"
         "addr1_requested_deci,addr2_requested_deci,addr3_requested_deci,addr4_requested_deci,"
         "addr1_candidate_integer_rpm,addr2_candidate_integer_rpm,"
         "addr3_candidate_integer_rpm,addr4_candidate_integer_rpm,"
         "addr1_candidate_dir,addr2_candidate_dir,addr3_candidate_dir,addr4_candidate_dir,"
         "candidate_vx_mm_s,candidate_vy_mm_s");
    for (direction = 0; direction < sizeof directions / sizeof directions[0]; ++direction) {
        double position = 0.0, speed = 0.0, acceleration = 0.0;
        unsigned phase, batch = 0;
        const char *name = directions[direction].name;
        const int x = directions[direction].x, y = directions[direction].y;

        if (!emit_sample(name, batch, x, y, position, speed, acceleration)) return 1;
        for (phase = 0; phase < sizeof phase_steps / sizeof phase_steps[0]; ++phase) {
            const double jerk = jerk_magnitude * jerk_sign[phase];
            unsigned step;
            for (step = 0; step < phase_steps[phase]; ++step) {
                position += (speed + (acceleration / 2.0 + jerk * dt / 6.0) * dt) * dt;
                speed += (acceleration + jerk * dt / 2.0) * dt;
                acceleration += jerk * dt;
                if (fabs(speed) < 1e-9) speed = 0.0;
                if (fabs(acceleration) < 1e-9) acceleration = 0.0;
                if (!emit_sample(name, ++batch, x, y, position, speed, acceleration)) return 1;
            }
        }
    }
    return fflush(stdout) == EOF || ferror(stdout);
}
