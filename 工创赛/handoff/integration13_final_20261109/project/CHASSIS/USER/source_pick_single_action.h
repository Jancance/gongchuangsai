#ifndef SOURCE_PICK_SINGLE_ACTION_H
#define SOURCE_PICK_SINGLE_ACTION_H
/* Single pick action table (plan C: turntable brings each piece to the
 * down region, so one fixed action serves all three pieces).
 *
 * Measured / recorded inputs (each with a source, do not re-invent):
 *   extension total stroke  170mm  (= 70mm prep + 100mm further)
 *                                  competition_requirements_2027.md:37,
 *                                  integration10_complete.inc:315 (<=170)
 *   retracted radius L0     161mm  (CANDIDATE, not yet measured)
 *                                  SOURCE_PAPER48_RESULT_2026-11-06.md:18
 *   disc 300mm / orbit R=100mm / 120deg   SRC-01, SRC-02
 *
 * Geometry at turntable-axis distance D (user: D=280mm at 75mm clearance):
 *   near piece  D-100        = 180.00mm -> needs  19.00mm extension
 *   side piece  sqrt(D^2+100^2+100D) = 341.17mm -> needs 180.17mm
 *   side pieces exceed the 170mm stroke by 10.17mm at D=280.
 *   They become reachable if D <= 269.5mm (clearance 75 -> 64.5mm),
 *   or if the real L0 turns out >= 171.2mm, or stroke >= 181mm.
 *   Recommended operating window: clearance 55..60mm (D 260..265).
 *   Below 55mm clearance the near piece needs negative extension.
 *
 * So this single action is valid for the NEAR piece as-is. For the side
 * pieces use it only after the D/L0 condition above is confirmed on site.
 * The turntable fallback (let the disc bring the next piece to the near
 * region) needs no extension and no geometry assumption.
 */
#define SRC_PICK_ARM_DEG         180.0f  /* 4310 logical angle while picking      */
#define SRC_PICK_EXT_MM          0.0f    /* extension stays retracted             */
#define SRC_PICK_DEPTH_PULSES    118800U /* lift target below top reference        */
#define SRC_PICK_DEPTH_MM        50.0f   /* measured gripper bottom at that depth  */
#define SRC_GRIP_OPEN_DEG        210.0f  /* recognition pose                       */
#define SRC_GRIP_PRE_DEG         160.0f  /* pre-close during descent               */
#define SRC_GRIP_CLOSE_DEG       140.0f  /* clamped                                */

/* 3 second dwell budget: the disc rotates again 3s after stopping, so the
 * piece must be lifted clear of the disc surface inside that window. */
#define SRC_DESCENT_MS           2000U
#define SRC_GRIP_MS              400U
#define SRC_LIFT_OFF_MS          600U
#define SRC_DWELL_BUDGET_MS      3000U
#define SRC_TRANSFER_MARGIN_MM   5.0f    /* lift clear of the disc rim */

/* compile-time guard: descent + grip + lift-off must fit the dwell */
typedef char src_pick_timing_must_fit_dwell
    [(SRC_DESCENT_MS + SRC_GRIP_MS + SRC_LIFT_OFF_MS <= SRC_DWELL_BUDGET_MS) ? 1 : -1];

/* Turntable: R source says 6..10 s per revolution, 3 s dwell, random
 * direction. Timing below is the design window, not a measurement. */
#define SRC_TURN_MIN_MS          (120U * 10000U / 600U)  /* 120deg @ 60deg/s = 2000ms */
#define SRC_TURN_MAX_MS          (120U * 10000U / 360U)  /* 120deg @ 36deg/s = 3333ms */
#define SRC_TURN_MAX_CYCLE_MS    (SRC_TURN_MAX_MS + SRC_DWELL_BUDGET_MS)

#endif
