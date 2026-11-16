#ifndef APP_ICM_OBSERVER_H
#define APP_ICM_OBSERVER_H
#include "app_config.h"
#if APP_BASELINE40_ACTIVE
#include "icm45686_raw.h"
#define ICM_OBS_CAL 1U
#define ICM_OBS_WHEEL 2U
#define ICM_OBS_GRAVITY_GAP 4U
#define ICM_OBS_NO_LEVER 8U
#define ICM_OBS_CAL_BLOCKED 16U
#define ICM_OBS_ACCEL 32U
#define ICM_OBS_WAIT_CAL 1U
#define ICM_OBS_RAW 2U
#define ICM_OBS_GAP 4U
#define ICM_OBS_HWT 8U
#define ICM_OBS_REFERENCE 16U
#define ICM_OBS_BAD_WHEEL 32U
#define ICM_OBS_HISTORY 64U
#define ICM_OBS_WINDOW 128U
#define ICM_OBS_NUMERIC 256U
typedef struct {
    uint32_t sequence, session, valid, reasons, flags;
    uint32_t estimate_ms, sample_sequence, reference_ms, t0_ms, t1_ms;
    uint32_t wheel_age_ms, hwt_age_ms, cal_samples, history_samples, gaps;
    float wheel_f_mm, wheel_r_mm, position_f_mm, position_r_mm;
    float velocity_f_mm_s, velocity_r_mm_s, accel_f_mm_s2, accel_r_mm_s2;
} AppIcmObserverStatus;
/* Existing ICM task is the sole writer. These outputs never grant motion. */
void AppIcmObserver_Step(const IcmRawStatus *sample);
void AppIcmObserver_Read(AppIcmObserverStatus *snapshot);
#endif
#endif
