"""ARM replay of the zone-2 application, navigation, CAN owner, radar and map."""
import argparse
import json
import re
import runpy
import shutil
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
COMMON = runpy.run_path(str(HERE.parent / "p1-motion/run_p1_arm_replay.py"))
ROOT, PROJECT, SOURCE, ARM = (COMMON[k] for k in ("ROOT", "PROJECT", "SOURCE", "ARM"))
run, short, sha = (COMMON[k] for k in ("run", "short_path", "sha"))


def replace_once(src, old, new):
    if src.count(old) != 1:
        raise ValueError(f"Expected one fixture anchor: {old!r}")
    return src.replace(old, new)


def build_peers(out, owner, radar, main_source):
    can_source = owner.read_text(encoding="utf-8").split("static void step(void)", 1)[0]
    can_source = replace_once(can_source, "static uint32_t slot, phase;",
        """static uint32_t slot, phase;
static uint32_t plant_command(uint8_t address,uint8_t dir,uint16_t raw);
static void plant_feedback(uint8_t address,CAN1_EmmSystemFeedback *f);
static uint32_t drop_one_reply(uint8_t address,uint32_t command);
static uint32_t drop_moving_ack(uint8_t address,uint8_t dir,uint16_t raw,CAN1_EmmSpeedReceipt *r);""")
    can_source = replace_once(can_source, "    if (drift && address==3U", "    plant_feedback(address,f);\n    if (drift && address==3U")
    can_source = replace_once(can_source, "    feedback_reads++;",
        "    feedback_reads++;\n    if (drop_one_reply(address,0x43U)) return CAN1_TIMING_WINDOW;")
    can_source = replace_once(can_source, "    status_reads++;",
        "    status_reads++;\n    if (drop_one_reply(address,0x3AU)) return CAN1_TIMING_WINDOW;")
    can_source = replace_once(can_source, "        if (fault_mode==4U) { g_can_ctrl_fault_latched=1U; return 8U; }\n    }\n    return 1U;",
        """        if (fault_mode==4U) { g_can_ctrl_fault_latched=1U; return 8U; }
    }
    if (drop_moving_ack(address,dir,raw,r)) return CAN1_TIMING_WINDOW;
    return plant_command(address,dir,raw);""")
    can_source = replace_once(can_source, "address==stops+1U", "address==stops%4U+1U")
    can_source = replace_once(can_source, "    stops++;", "    stops++;\n    (void)plant_command(address,0U,0U);")
    (out / "owner_peer.c").write_text(can_source, encoding="ascii")
    uart_source = radar.read_text(encoding="utf-8").split("static void start_until_grid", 1)[0]
    uart_source = replace_once(uart_source, "static uint32_t command_count[256];",
        "static uint32_t command_count[256];\nstatic uint8_t path_start,path_goal;")
    uart_source = replace_once(uart_source, "    command_sequence[message->type] = message->sequence;",
        """    command_sequence[message->type] = message->sequence;
    if (message->type==LINK_CMD_PLAN_PATH) {
        CHECK(message->payload_length==2U);
        path_start=message->payload[0];
        path_goal=message->payload[1];
    }""")
    first = uart_source.index("uint32_t xTaskGetTickCount")
    last = uart_source.index("uint32_t RadarServo_SetAngle", first)
    uart_source = uart_source[:first] + uart_source[last:]
    radar_task = main_source.read_bytes().split(b"static void App_RadarTask(", 1)[1].split(b"\n}\r", 1)[0]
    config = re.search(rb"const RadarZone2Config cfg\s*=\s*\{([\dU,\s]+)\};", radar_task)
    period = re.search(rb"vTaskDelay\(pdMS_TO_TICKS\((\d+U)\)\);", radar_task)
    if not config or not period:
        raise ValueError("Cannot read production radar task timing")
    uart_source = replace_once(uart_source,
        "static const RadarZone2Config config = {7,0,8,5,5,5,1000,1};",
        f"""#define CHAIN_RADAR_PERIOD_MS {period[1].decode("ascii")}
static const RadarZone2Config config = {{
#if CHAIN_STARTUP_TIMING
    {config[1].decode("ascii").strip()}
#else
    1,0,8,5,5,5,1000,1
#endif
}};
static uint32_t lift_started_ms;""")
    uart_source = replace_once(uart_source, "    CHECK(count == 8 && dir <= 1 && budget > 0);",
        "    CHECK(count == config.steps && dir <= 1 && budget > 0);\n    lift_started_ms=tick;")
    uart_source = uart_source.replace("queue(7,", "queue(1,")
    (out / "radar_peer.c").write_text(uart_source, encoding="ascii")


def main():
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--shadow", action="store_true")
    mode.add_argument("--drop-radar", action="store_true")
    mode.add_argument("--drop-f6", action="store_true")
    mode.add_argument("--occupied-detour", action="store_true")
    mode.add_argument("--occupied-bad-path", action="store_true")
    mode.add_argument("--one-miss", dest="miss_command", action="store_const", const=0x43, default=0)
    mode.add_argument("--one-status-miss", dest="miss_command", action="store_const", const=0x3A)
    parser.add_argument("--startup-timing", type=int, metavar="SCAN_RESULT_DELAY_MS")
    parser.add_argument("--require-post-budget-motion", action="store_true",
                        help="require a nonzero wheel command after the 180-second scoring budget and full completion")
    parser.add_argument("--receipt", type=Path)
    args = parser.parse_args()
    if args.startup_timing is not None and not 0 <= args.startup_timing <= 8000:
        parser.error("startup scan result delay must be 0 through 8000 ms")
    if args.startup_timing is not None and (args.shadow or args.drop_radar or args.drop_f6 or args.occupied_bad_path):
        parser.error("--startup-timing requires a course completion scenario")
    if args.require_post_budget_motion and args.startup_timing is None:
        parser.error("--require-post-budget-motion requires --startup-timing")
    fw = PROJECT / "CHASSIS"
    radar = fw / "HARDWARE/RADAR_V6"
    out = Path(tempfile.mkdtemp(prefix="zone2_chain_", dir=ROOT / "工创赛/tmp"))
    print(out, flush=True)
    fixture = HERE / "zone2_chain_fixture.c"
    owner_peer = HERE.parent / "p2-owner/runtime_owner_fixture.c"
    radar_peer = HERE.parent / "p3-observer/independent_fixture.c"
    radar_registers = radar_peer.with_name("independent_stm32_stub.h")
    units = [SOURCE / f"{name}.c" for name in (
        "motion_profile", "fusion_motion", "fusion_reference", "fusion_adapter",
        "chassis_kinematics", "chassis_odometry", "zone2_nav_schedule", "zone2_nav_path", "field_map",
        "zone2_motion_reference", "app_odometry")]
    units += [radar / f"{name}.c" for name in ("radar_observer", "radar_client", "link_protocol")]
    kernel_runner = HERE.parent / "p1-motion/run_p1_arm_replay.py"
    service = SOURCE / "app_chassis_service.c"
    main_source = SOURCE / "main.c"
    inputs = [Path(__file__), fixture, owner_peer, radar_peer, radar_registers, kernel_runner, service, *units,
              SOURCE / "app_zone2.c", SOURCE / "chassis_auto_test.c", main_source]
    hashes = {p.name: sha(p) for p in inputs}
    build_peers(out, owner_peer, radar_peer, main_source)
    registers = out / "radar_registers"
    registers.mkdir()
    shutil.copyfile(radar_registers, registers / "stm32f4xx.h")
    (out / "unit.c").write_text(fixture.read_text(encoding="utf-8"), encoding="ascii")
    service_txt = service.read_bytes()
    service_signature = b"void AppChassis_Zone2CanStep("
    if service_txt.count(service_signature) != 1:
        raise ValueError("Expected one production Zone2 slot entry point")
    service_func = service_txt[service_txt.index(service_signature):].split(b"\r\n}\r\n", 1)[0]
    (out / "can_service_entry.c").write_bytes(service_func + b"\r\n}\r\n")
    (out / "FreeRTOS.h").write_text('#include "task.h"\n', encoding="ascii")
    (out / "task.h").write_text(
        """#ifndef TEST_TASK_H
#define TEST_TASK_H
#include <stdint.h>
typedef uint32_t TickType_t;
extern uint32_t test_tick,test_depth;
#define portTICK_PERIOD_MS 1U
#define pdMS_TO_TICKS(x) (x)
#define taskENTER_CRITICAL() (++test_depth)
#define taskEXIT_CRITICAL() (--test_depth)
static __inline uint32_t xTaskGetTickCount(void) { return test_tick; }
static __inline void vTaskDelay(TickType_t ticks) { test_tick+=ticks; }
#endif
""", encoding="ascii")
    (out / "unit.sct").write_text(
        "LR_ROM 0x08000000 0x100000 { ER_ROM 0x08000000 0x100000 { unit.o (i.test_entry, +First) *(+RO) } "
        "RW_RAM 0x20000000 0x20000 { *(+RW,+ZI) } }", encoding="ascii")
    opts = ["--cpu", "Cortex-M4.fp.sp", "--c99", "--gnu", "--split_sections", "-O1", "-I.",
            "-DSTM32F40_41xxx", "-DUSE_STDPERIPH_DRIVER", "-DAPP_MOTOR_OFFLINE_ONLY=0",
            "-DZDT_EMM_SETTINGS_CONFIRMED=1", "-DAPP_CONTINUOUS_MAP_ENABLE=1","-DAPP_ZONE2_NAV_ENABLE=1",
            f"-DAPP_ZONE2_NAV_MOTION_ENABLE={int(not args.shadow)}",
            f"-DCHAIN_DROP_LINK={int(args.drop_radar)}", f"-DCHAIN_DROP_F6={int(args.drop_f6)}",
            f"-DCHAIN_OCCUPIED_DETOUR={int(args.occupied_detour)}",
            f"-DCHAIN_OCCUPIED_BAD_PATH={int(args.occupied_bad_path)}",
            f"-DCHAIN_MISS_COMMAND={args.miss_command}",
            f"-DCHAIN_STARTUP_TIMING={int(args.startup_timing is not None)}",
            f"-DCHAIN_REQUIRE_POST_BUDGET_MOTION={int(args.require_post_budget_motion)}",
            f"-DCHAIN_SCAN_DELAY_MS={args.startup_timing or 0}"]
    opts.extend("-I" + short(fw / p) for p in ("CORE", "USER", "StdPeriph_Driver/inc", "SYSTEM/CAN",
                "SYSTEM/SAFETY", "HARDWARE/POWER", "HARDWARE/RADAR_V6", "HARDWARE/RADAR_SERVO"))
    objects = []
    for source in [out / "unit.c", *units]:
        obj = source.stem + ".o"
        register_option = ["-I" + short(registers)] if source.parent == radar else []
        run([str(ARM / "armcc.exe"), *register_option, *opts, "-c", short(source), "-o", obj], out,
            source.stem + ".log")
        objects.append(obj)
    run([str(ARM / "armlink.exe"), "--entry", "test_entry", "--scatter", "unit.sct",
         *objects, "--library_type=microlib", "-o", "unit.axf"], out, "link.log")
    kernel = kernel_runner.read_text(encoding="utf-8")
    kernel = replace_once(kernel, "timeout=60_000_000, count=200_000_000",
                          "timeout=120_000_000, count=8_000_000_000")
    kernel = replace_once(kernel, "    return dict(failed_line=", """    chain_integers = ("stop_legs", "path_requests", "reuses", "fault", "cause", "reference",
                      "backend", "sends", "nonzero", "shadow", "map_state", "map_reasons", "map_updates",
                      "path_error", "stop_frames", "runtime_ms", "tracking_checks", "edge_checks",
                      "worst_leg", "worst_section", "worst_span", "worst_ms",
                      "recovery_ms", "recovery_stops", "recovery_quiet", "truth_samples", "truth_contacts",
                      "first_contact_ms", "robust_contacts", "one_miss", "one_miss_ms", "owner_miss",
                      "partial", "f6_possible", "f6_tx", "f6_ack", "start_ms", "scoring_budget_end_ms",
                      "post_budget_nonzero",
                      "occupied_intervals", "occupied_contacts", "detour_witness",
                      "occupied_reject_nonzero_after_first_stop",
                      "outcome_ms", "scan_tx_ms", "scan_result_ms", "scan_ready_ms", "lift_leg_ms")
    chain_floats = ("x", "y", "heading", "max_error_mm", "max_heading_deg", "edge_error_mm",
                    "worst_ref_x", "worst_ref_y", "truth_x", "truth_y", "truth_yaw", "truth_clearance",
                    "robust_clearance", "occupied_clearance", "detour_witness_x", "detour_witness_y")
    return dict(**{name: read("chain_" + name, "<I") for name in chain_integers},
                **{name: read("chain_" + name, "<f") for name in chain_floats},
                **{name: read("chain_" + name, "<8I")
                   for name in ("arrival_ms", "plan_ms", "plan_sections", "track_ms", "scan_stage_ms")},
                failed_line=""")
    read_chain = r"""match = re.search(r"\b" + name + r"\s+(0x[0-9a-fA-F]+)", symbols)
        if not match and name in SHADOW_OMITTED:
            return (0,) * 8 if fmt == "<8I" else 0
        if not match:
            raise ValueError(f"Missing replay symbol: {name}")
        values = struct.unpack(fmt, machine.mem_read(int(match[1], 16), struct.calcsize(fmt)))
        return values if fmt == "<8I" else values[0]"""
    kernel = replace_once(kernel, r"""address = int(re.search(r"\b" + name + r"\s+(0x[0-9a-fA-F]+)", symbols)[1], 16)
        return struct.unpack(fmt, machine.mem_read(address, 4))[0]""", read_chain)
    omitted = ("track_ms", "tracking_checks", "edge_checks", "worst_leg", "worst_section",
               "worst_span", "worst_ms", "worst_ref_x", "worst_ref_y", "max_error_mm",
               "max_heading_deg", "edge_error_mm", "plan_ms", "plan_sections")
    if not args.drop_radar and not args.drop_f6:
        omitted_nonfault = {"chain_recovery_ms", "chain_recovery_stops", "chain_recovery_quiet"}
    else:
        omitted_nonfault = set()
    if not args.miss_command:
        omitted_nonfault.add("chain_one_miss_ms")
    if not args.drop_f6:
        omitted_nonfault.add("chain_partial")
    if not (args.occupied_detour or args.occupied_bad_path):
        omitted_nonfault.update(("chain_occupied_intervals", "chain_occupied_contacts", "chain_occupied_clearance"))
    if not args.occupied_detour:
        omitted_nonfault.update(("chain_detour_witness", "chain_detour_witness_x", "chain_detour_witness_y"))
    if not args.occupied_bad_path:
        omitted_nonfault.add("chain_occupied_reject_nonzero_after_first_stop")
    if args.startup_timing is None:
        omitted_nonfault.update("chain_" + name for name in ("scan_stage_ms", "scan_tx_ms",
                               "scan_result_ms", "scan_ready_ms", "lift_leg_ms"))
    runtime = {"__file__": str(kernel_runner), "__name__": "__chain_emulator__",
               "SHADOW_OMITTED": omitted_nonfault | ({"chain_" + name for name in omitted} if args.shadow else set())}
    exec(compile(kernel, str(kernel_runner), "exec"), runtime)
    result = runtime["execute"](out)
    result.update(hardware_accessed=False, motion_enabled=not args.shadow, radar_drop_test=args.drop_radar,
                  f6_ack_drop_test=args.drop_f6,
                  occupied_grid_mode=("detour" if args.occupied_detour else
                                      "bad_path" if args.occupied_bad_path else None),
                  startup_scan_result_delay_ms=args.startup_timing,
                  require_post_budget_motion=args.require_post_budget_motion,
                  scoring_margin_ms=result["scoring_budget_end_ms"] - result["outcome_ms"],
                  course_completed=result["stop_legs"] == 8 and result["fault"] == 0,
                  injected_missing_reply=f"0x{args.miss_command:02X}" if args.miss_command else None,
                  input_sha256=hashes, axf_sha256=sha(out / "unit.axf"))
    if result["failed_line"]:
        result["failed_expression"] = fixture.read_text().splitlines()[result["failed_line"]-1].strip()
    if hashes != {p.name: sha(p) for p in inputs}:
        raise RuntimeError("Replay inputs changed")
    raw = json.dumps(result, indent=2) + "\n"
    (out / "receipt.json").write_text(raw, encoding="utf-8")
    if args.receipt:
        with (HERE / args.receipt).open("x", encoding="utf-8") as report:
            report.write(raw)
    print(raw)
    return int(bool(result["failed_line"]))


if __name__ == "__main__":
    raise SystemExit(main())
