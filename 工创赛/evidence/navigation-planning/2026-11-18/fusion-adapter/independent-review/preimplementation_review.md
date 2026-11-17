# Independent preimplementation review

Date: 2026-11-18. Role: independent high review. Baseline HEAD: 70b230c481b03c4fd30f0079c26f9a3f2d003701; reference product baseline d023c37a.

Gate: PASS for writing the narrowly scoped pure copied-value fusion_adapter C/H. This is a MATLAB-contract gate, not acceptance of product C, a runtime caller, physical feedback, or deployment.

## Evidence inspected

- AGENTS.md, current continuation and long-term handoff entries, PLAN.md, PLAN_COMPLETENESS_REVIEW.md, actual AppMapPose/ChassisOdometryStatus and existing FusionReference interfaces and implementation.
- Final verify_fusion_adapter_v8.m, with prior v7 complete-source inspection and v8 ordering/combination change inspected separately.
- matlab_receipt_v8.json and matlab_summary_v8.json: local MATLAB 9.8.0.1323502 (R2020a), 2026-11-18T01:32:22.886335+08:00 to 01:32:29.791944+08:00, exit 0, 740 checks and 120 vectors. Groups M1=37, M2=12, M3=71, M4=27, M5=35, M6=22, M7=536.
- Actual final script SHA256 independently read as 81888abffe3faee2fec80cfca6c5509bf02d361f2f6cf84cb4728522a1dcd356, matching receipt. Receipt states adapter C/H did not exist before and after MATLAB. Reviewer also independently checked both product files absent at this gate.

The reviewer inspected the actual script and execution receipt; the reviewer did not rerun MATLAB and does not relabel the implementation agent's run as an independent execution.

## Necessary coverage and resolved gate findings

M1..M7 now cover legal plan/caller/map identity, prepared-plan fields consumed by the adapter, uint32 modular ordering including zero and rollover, current source age and begin checks, complete immutable Observation caching, latest sample versus short-window anchor, epoch and counter restart, 20..100ms estimation, raw/corrected separation, wrapped map heading, finite/range limits, stop-evidence lifetime and audit metadata. The 20ms quantization result is an engineering model choice; the actual raw deceleration trajectory explicitly demonstrates window-average lag.

Earlier v5 lacked necessary plan-identity and combined cache/time/stop prevalidation. Those gaps were reported before product release and are not hidden by the later check count. v7 additionally had same-timestamp core conflict combined with updates rollback processed as prime. v8 checks core equality before processing same/new-sample updates rollback and adds the exact combined counterexample. Old sample plus lower updates is rejected before any re-prime. These priorities match the final contract.

Cached stopped=true now requires both current complete stop evidence validity and the originally bound stop time remaining fresh. A newer stop timestamp cannot extend old cached evidence. Cached stopped=false can only report new stop evidence as pending until a fresh map sample. Corrected XY is core; correction/source/calibration fields remain finite audited metadata and are not applied twice.

The existing-state rejection of malformed plans is clear in the callable model's early-return ordering. Further byte-for-byte state preservation, NULL behavior, real header layout, actual ARM execution, the 120 numerical vectors and combined Reference state sequences remain mandatory C-stage checks rather than reasons to enlarge this MATLAB gate.

## Independent C review scope remaining

Compile the actual adapter and existing Reference/kinematics sources with the actual app_odometry.h and ChassisOdometryStatus layout. Only platform dependencies may be stubbed and must be identified. Independently create sequences for error-priority combinations, short-window/latest-key preservation, stale/gap recovery, full Observation equality, current versus cached stop time, nonfinite and numerical range boundaries, unchanged plan/map/state on required rejection, and actual Reference no-repeat/no-false-DONE behavior. Bind the result to final product SHA, not just implementation-agent counts.

No product, model, project file, root handoff, Git index or hardware was modified/accessed by this review. Only this stage's independent-review directory was created for evidence. No fetch/push/download/reset/driving was performed.

R checked: FIELD-01/02/06, OBS-01/04, RUN-01/02/03/04/07, PWR-01, MECH-01/02. E: LOC-02/03/05/06/07, MOT-01..04, CAL-02 and estimation/cache parameters. Q FIELD-05/OBS-03/QR-04 remain pending official technical clarification. Runtime/CAN owner, measured wheel/HWT feedback, complete radar bundle, A/J braking, actual stack margin, mixed calibration, transport outline and two-start full-task validation remain open.
