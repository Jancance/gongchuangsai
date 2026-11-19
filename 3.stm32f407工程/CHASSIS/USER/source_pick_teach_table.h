#ifndef SOURCE_PICK_TEACH_TABLE_H
#define SOURCE_PICK_TEACH_TABLE_H
/* Pick teach table: the ONLY three numbers a human must provide.
 *
 * User ownos these: 4310 logical angle and extension length per region.
 * Everything else below is already measured on the vehicle and must not be
 * retuned without a new measurement record.
 *
 * Fill the three rows on site, then rebuild. Keep 0.0 as "not taught yet" -
 * the sequence refuses to start while any row is 0.
 */
#define SRC_TEACH_ARM_UNTAUGHT   0.0f

/* region: down / left / right, in the order the vision reports them */
typedef struct {
    float arm_deg;        /* 4310 logical angle written by the operator   */
    float extension_mm;   /* extension length written by the operator      */
} SrcTeachPoint;

/* ---- site values: operator fills these three rows ---------------------- */
#define SRC_TEACH_DOWN_ARM_DEG    0.0f
#define SRC_TEACH_DOWN_EXT_MM     0.0f
#define SRC_TEACH_LEFT_ARM_DEG    0.0f
#define SRC_TEACH_LEFT_EXT_MM     0.0f
#define SRC_TEACH_RIGHT_ARM_DEG   0.0f
#define SRC_TEACH_RIGHT_EXT_MM    0.0f

/* ---- rotation rule (user 2026-11-08) ----------------------------------
 * Angle frame: 0deg = arm pointing to the LEFT of the vehicle,
 *              180deg = arm pointing at the source disc (front),
 *              270deg = arm pointing to the RIGHT.
 * Therefore a SMALLER angle is a clockwise move.
 * Every pick finishes by rotating CLOCKWISE (decreasing angle) to
 * SRC_ACT_SLOT_ARM_DEG = 35.758deg above the carrier slot. Never wrap past
 * 270deg - there is no need to, clockwise is always the short way from any
 * pick angle inside 0..270.
 * Side mirror: right_deg = 360 - left_deg (symmetric about 180).
 */

/* ---- fixed, already measured: do not touch ---------------------------- */
#define SRC_ACT_LIFT_DEPTH_PULSES 118800U  /* gripper bottom 50mm above disc   */
#define SRC_ACT_GRIP_OPEN_DEG     210.0f   /* recognition pose                 */
#define SRC_ACT_GRIP_PRE_DEG      160.0f   /* pre-close during descent         */
#define SRC_ACT_GRIP_CLOSE_DEG    140.0f   /* clamped                          */
#define SRC_ACT_SLOT_ARM_DEG      35.758f  /* above the on-board carrier slot  */
#define SRC_ACT_ARM_RPM_DEG_S     300.0f
#define SRC_ACT_EXT_RPM           106.0f
#define SRC_ACT_LIFT_RPM          1846.4f

/* ---- timing: from SINGLE_CYCLE_STREAM fin-01/02/03 (5.87-5.96s) ------- */
#define SRC_ACT_DESCENT_MS        2050U
#define SRC_ACT_PRIGRIP_DELAY_MS  420U   /* gripper pre-close starts here     */
#define SRC_ACT_GRIP_MS           400U
#define SRC_ACT_ROTATE_AT_MS      1400U  /* 4310 may move only after this     */
#define SRC_ACT_RISE_MS           3350U
#define SRC_ACT_RELEASE_MS        130U
#define SRC_ACT_TOTAL_MS          (SRC_ACT_DESCENT_MS + SRC_ACT_GRIP_MS + \
                                   SRC_ACT_RISE_MS + SRC_ACT_RELEASE_MS)

/* ---- turntable slot index for piece 1..3 ----------------------------- */
#define SRC_ACT_SLOT_1_DEG        0.0f
#define SRC_ACT_SLOT_2_DEG        120.0f
#define SRC_ACT_SLOT_3_DEG        240.0f

/* ---- guards ----------------------------------------------------------- */
/* arm must stay inside the firmware admission range (integration10 314) */
typedef char src_teach_arm_left_fits[(SRC_TEACH_LEFT_ARM_DEG  >= 0.0f &&
                                      SRC_TEACH_LEFT_ARM_DEG  <= 270.0f) ? 1 : -1];
typedef char src_teach_arm_right_fits[(SRC_TEACH_RIGHT_ARM_DEG >= 0.0f &&
                                       SRC_TEACH_RIGHT_ARM_DEG <= 270.0f) ? 1 : -1];
/* extension must stay inside the recorded stroke (integration10 315) */
typedef char src_teach_ext_fits[(SRC_TEACH_DOWN_EXT_MM  >= 0.0f &&
                                 SRC_TEACH_DOWN_EXT_MM  <= 170.0f &&
                                 SRC_TEACH_LEFT_EXT_MM  >= 0.0f &&
                                 SRC_TEACH_LEFT_EXT_MM  <= 170.0f &&
                                 SRC_TEACH_RIGHT_EXT_MM >= 0.0f &&
                                 SRC_TEACH_RIGHT_EXT_MM <= 170.0f) ? 1 : -1];
/* the 5.9s budget must stay under the turntable dwell if the disc is used */
typedef char src_act_budget_sane[(SRC_ACT_TOTAL_MS <= 7000U) ? 1 : -1];

#endif
