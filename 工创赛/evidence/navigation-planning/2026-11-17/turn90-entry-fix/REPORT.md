# TURN90 entry repair — local coding handoff

2026-11-17. Source checkpoint 66409386. No hardware, serial ports, probe enumeration, reset, download, vehicle run, fetch, commit or push performed by this coding task.

## Product change

Only app_serial_runtime.c and app_motion_runtime.c changed (14 added / 3 removed lines total). Both remain GBK / CRLF.

- Idle PLAN20/21 reports the selected single TURN action before runtime action loading. TURN90 theoretical translation speed is the INT32_MIN not-applicable sentinel.
- List loading admits only one action at index zero, kind TURN, exact plan/direction/82 RPM/±90 tuple, with run_ms/distance_mm/reference_mm_s all zero.
- This single relative turn uses the already supported TurnStart path and sets route_turn_active to zero. Existing default-route PerimeterTurnStart and baseline translation branches remain intact. No ARM/client gate, HeadingSession, TurnBrake or control parameter changes.

## Verification

check_integration.py compiles function text extracted from the current product C sources with the installed ARM compiler and runs it in Unicorn with actual ELF initialized RAM sections. Tested real App_CombinedLoadAction, TurnStart, App_PerimeterTurnStart, real telemetry values[56]/[57] expressions, and the actual heading-runtime Start/StartReference dispatch expression. Real app_entry, HeadingSession, HeadingController, MotionProfile and kinematics objects are linked.

PASS: 10,482 C assertions, 36 session cases: PLAN20/21 × initial 37/170/-170 degrees × preparation yaw drift 0/0.01 degrees × 10/20/40 ms updates. Every case dispatches TurnStart, keeps exact relative ±90 degrees, preserves valid peak-90 S profile, handles warmup with fresh samples, starts reference time on first RUN, and keeps that time through later backend START observations. A stop confirmation mask 7 cannot complete; mask 15 completes. Invalid count/index/kind/direction/RPM/angle and polluted action fields are rejected. PLAN8..19 and default TURN/reference-translation routing remain covered.

Actual C telemetry values, including action_kind=3 and the speed sentinel, are encoded as CRC-protected TMQ, parsed by the unchanged real turn90 Python client, and consumed through PLAN20/21 → ARM → START → software_complete. Exactly one PLAN, ARM and START per direction. HAL/RTOS/CAN acceptance and backend responses are bounded stubs; the test proves entry and lifecycle wiring, not full C serial-parser integration, board stack/timing or real trajectory. Session input follows the reference; it is not a new dynamics simulation and does not replace existing MATLAB evidence.

Red regression: original C fails the idle telemetry assertion. Original motion source with repaired telemetry fails the positive list-load assertion. See red_regression.json and before*.log. Original six dirty files retain their saved hashes; scoped diff check passes using cr-at-eol because source CRLF must remain.

## Remaining

Root owns full Keil build and high independent final review. build_fixed_candidate.py is generated only, not executed here; isolated project path 工创赛/tmp/turn90-entry-fix-build-20261117 and target name turn90_90_entryfix preserve prior flags and source checkpoint 66409386. No deployment or test retry performed here. Real 90-degree accuracy, physical peak speed, rotation-center translation, CAN timing and on-board stack high water remain unverified.

R checked: RUN-01, PWR-01, MECH-01/02. E checked: LOC-02/03/05/07, MOT-02/04, RUN-06. This is debug-entry engineering repair; official Q FIELD-05/OBS-03/QR-04 remain pending technical clarification. Root maintains project handoffs; this worker did not edit them.
