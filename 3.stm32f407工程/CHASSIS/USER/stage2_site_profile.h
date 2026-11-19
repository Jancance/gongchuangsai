#ifndef STAGE2_SITE_PROFILE_H
#define STAGE2_SITE_PROFILE_H
#include "metric_alignment.h"
#include "source_stage2_body_port.h"
#include "source_pick_single_action.h"
#include "source_pick_teach_table.h"
/* SITE DATA TABLE (offline scaffold, values are NOT measured yet).
 * Fill every field from one measured session, then set the validity flags.
 * The system stays disabled while any validity flag is 0 - by design.
 * Units: distance = milli-mm, angle = milli-degree, speed = mm/s.
 */
#define STAGE2_SITE_TABLE_VERSION 0U
static const MetricAlignmentProfile g_stage2_site_profile = {
 /*calibrated*/              0U,
 /*camera_id*/               0U,
 /*calibration_id*/          0U,
 /*mount_id*/                0U,
 /*height_id*/               0U,
 /*target_id*/               0U,
 /*max_frame_age_ms*/        0U,
 /*min_confidence_permille*/ 0U,
 /*max_delta_milli_mm*/      0,
 /*max_yaw_mdeg*/            0
};
static const S2_BodyLimits g_stage2_site_limits = {
 /*body_frame_verified*/     0U,
 /*route3_profile_verified*/ 0U,
 /*tolerance_milli_mm*/      0U,
 /*yaw_tolerance_mdeg*/      0U,
 /*max_corrections*/         0U,
 /*correction_speed_mm_s*/   0.0f
};
static const S2_ServoTiming g_stage2_site_timing = {
 /*carrier_settle_ms*/ 0U,
 /*gripper_settle_ms*/ 0U
};
#endif
