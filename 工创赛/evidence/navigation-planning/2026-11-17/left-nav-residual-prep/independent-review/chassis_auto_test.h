#ifndef TEST_CHASSIS_AUTO_H
#define TEST_CHASSIS_AUTO_H
#include <stdint.h>
typedef struct
{
    uint32_t state, active, result, cause;
    uint32_t session, segment, rotation_stop_mask;
    uint32_t heading_mode, dry_run, cut_vm, forward_hold_mode;
    uint32_t distance_mode, distance_reached, distance_base_rpm, distance_fault_reason;
    float distance_target_mm, distance_progress_mm;
    /* 候选曲线状态，仅本地快照；不改变TM1字段布局。 */
    uint32_t distance_profile_mode, distance_profile_done;
    float distance_reference_mm, distance_reference_rpm;
    uint32_t translation_mode; /* 0前进、1左移、2右移；只读会话归属。 */
} ChassisAutoStatus;
typedef struct {
    uint32_t mask, sign[4], raw[4], ms[4];
} ChassisOdometrySamples;
extern volatile uint32_t g_chassis_auto_active,g_wheel_test_active;
void ChassisAuto_ReadOdometrySamples(ChassisOdometrySamples *v);
uint32_t ChassisAuto_ReadStatus(ChassisAutoStatus *v);
#endif
