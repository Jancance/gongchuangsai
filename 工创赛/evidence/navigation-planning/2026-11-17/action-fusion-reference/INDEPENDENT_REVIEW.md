# Independent review: pure fusion reference

Date: 2026-11-18. Reviewer role: independent high, read-only product review. Baseline HEAD: 9fce01db. Result: no remaining actionable findings within the explicitly limited pure-reference contract. This review does not approve an actuator integration or hardware deployment.

## Frozen sources and preceding MATLAB

- fusion_reference.c SHA256: 07a9439f0435cb8ce35fc045b0a74f17e07e0362549336859760ae5921d02fb2
- fusion_reference.h SHA256: 8e0d855ae91a5ae6bc856aaf6af2300cd24ccdf8d91d251294087466fa0ab961
- Actual existing chassis_kinematics.c SHA256: 9f009f558c6c82607dda9319e8abba96c2b3536343a91d3ffaef5dc2073f76aa
- Final MATLAB SHA256: 23073cb5d4914ac30c3b0064cff64d277043f20d2d0cb9b55150047df2ef4f76; receipt 2026-11-18 00:22:19 to 00:22:26, exit 0, 9034 checks. The receipt binds the preceding product version; the lateral velocity model preceded the corresponding C change.
- Initial 8695 checks only covered the earlier mathematics. Its missing state/budget model was explicitly rejected as sufficient prevalidation. Later 9010 and 9028 checkpoints added callable state sequences, reserve-first scaling, duplicate output caching, and adjacent collinear cursor normalization before their corresponding C changes.

## Findings resolved

1. Duplicate source timestamps could change a zero TURN_START result into a nonzero FOLLOW result after the first call advanced internal phase. The final version validates age/identity first, then returns the cached complete reference for the same immutable observation. Conflicting observations with the same sample timestamp are invalid. Fresh invalid observations do not consume state.
2. Nonzero traversal of collinear nodes depended on landing inside an endpoint tolerance ball, so normal discrete samples could skip that ball and cause recovery. Final cursor normalization follows only directed adjacent collinear edges, may cross multiple short collinear edges, and stops at a direction change. There is no global nearest-edge search.
3. Terminal sideways correction was compared against an along-route speed cap using total actual speed. This rejected normal sideways feedback near zero remaining route distance. Final validation separates actual tangential and normal velocity budgets, retains backward-motion and severe tangential overspeed rejection, and retains the total 400 mm/s mathematical candidate bound.

The third defect was independently reproduced against frozen pre-fix C/H under independent-lateral-before/. Its 1312-assertion PASS intentionally means that the old erroneous FOLLOW-then-RECOVERY behavior was reproduced; it is not a product acceptance result.

## Independent actual-C execution

independent-final/check_fusion_c.py freezes the final product files, compiles them plus the real existing chassis inverse implementation with ARMCC Cortex-M3 soft-float, initializes SHF_ALLOC PROGBITS and NOBITS from the resulting AXF, and executes the machine code in Unicorn. It does not substitute an implementation of the reference module.

Result: PASS, 1328 assertions, including 96 MATLAB CSV budget vectors. See independent-final/actual_c_checks.json. Product-source hashes match the final files above. Static sizes: FusionPlan 1408 B, FusionState 120 B, FusionInput 1012 B; these are object sizes, not live task stack measurements.

Coverage includes the full turn-start/follow/corner-stop/turn-end/done sequence; complete duplicate-reference equality; bad identity, stale/future/nonfinite input and recovery; signed progress and endpoint overshoot; multiple collinear crossings and forbidden corner skips; original-polyline remaining distance and backward speed-cap propagation; actual-heading world/body transforms; reserve-first wheel limits and scale-consistent output frames; actual body-only yellow collision despite a recoverable lateral-error magnitude; preview/correction geometry; all four built-in yellow blocks; and self-intersection without global cursor jumps.

The implementation check uses the separate check_fusion_c.py / actual_c_checks.json and ultimately reports 1364 assertions plus the same 96 budget vectors. The reviewer independently executed its own frozen independent-final/check_fusion_c.py at 1328 assertions. These counts describe different test scripts; both bind identical final product-source hashes. The later implementation-only boundary additions were inspected as evidence and were not claimed as independently replayed. No product source changed after the independent run.

Additional independent assertions exercise the opposite sign of the terminal sideways correction: lateral feedback of 15 mm/s remains FOLLOW, while tangential feedback of 400 mm/s remains RECOVERY and leaves state unchanged. Existing first-failure reports and the old reproduction were preserved.

## Limits and open work

- Input and output remain model_only, deployment_allowed=0, calibration_verified=0. Estimated or measured feedback is supplied by the caller; this pure module does not establish the truth of that source.
- No RTOS scheduling, real source latency, actuator/CAN owner, power, map adapter, radar path/cache adapter, download, reset, physical motion, or vehicle acceptance was tested by this review. No Keil build was run by the reviewer; root owns full-build evidence separately.
- The acceleration hint only gives an ideal distance-based speed cap. Desired references do not guarantee bounded acceleration or jerk, actuator tracking, braking under current acceleration/latency, or physical stopping. Direction changes stop before switching; nonzero corner blending remains pending.
- Rotation uses a conservative complete-rotation disk enclosed by an axis-aligned box. It cannot distinguish a geometrically feasible CW window from a feasible CCW window. The current safe case selects CW; both may be rejected by the conservative envelope. Precise directional fallback and the user-confirmed final parking points remain future work.
- Body dimensions, nominal map and obstacle boxes are model inputs. Actual transport outline, G/F transform, complete same-session obstacle map and endpoint references are not certified. Mixing sideways correction with the existing map 50-percent calibration is not integrated or calibrated here.
- This review does not claim all two-start/eight-stage business routes or a three-minute full task have been verified.

Checked R: FIELD-01/02/06, OBS-01/04, RUN-01/02/07, PWR-01, MECH-01/02. Control and geometry choices remain E under LOC/MOT/CAL. Q FIELD-05, OBS-03 and QR-04 remain pending official technical clarification. Their status was not changed.

## Final project/build evidence read-through

Both targets in the unified gongxun-f407.uvprojx contain exactly one C and one H reference resolving to the active CHASSIS/USER files. Removing these four XML blocks restores HEAD bytes exactly: only 20 ASCII lines were added; original LF and non-ASCII bytes remain. Final project SHA256: 3a288b1113e6547d6f2e22f0ebb647ab2d9d1229ab2ba22208e463aa5aa7f279.

All 12 frozen artifact hashes match build_receipts.json. Both final build logs explicitly compile fusion_reference.c and report 0 errors / 0 warnings; both maps remove unused FusionReference functions, consistent with no runtime caller. Final C/H hashes remain 07a9439f...02fb2 / 8e0d855a...ab961. REPORT.md and final_checks.json consistently limit acceptance to compilation and separate pure ARM execution, preserve the CW/CCW envelope limitation, and do not claim actuator, hardware or A/J acceptance. No new actionable findings. This final step only read existing evidence and updated this report; it did not run Keil, expand tests or change product files.
