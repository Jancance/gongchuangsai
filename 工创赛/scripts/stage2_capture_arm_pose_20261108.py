"""Capture a stationary 4310 pose via IT7; never command a motion or enable."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import struct
import time


ROOT = Path(__file__).resolve().parents[1]
IMAGE = ROOT / "handoff/stage2_pick_20261108/project/Objects/Vehicle_LiftRuler/Vehicle_LiftRuler.axf"
IMAGE_SHA = "7c6554b6e09f1bea8439033b127a08a0aaee0c00e06fc3ef3fbe9ac9b92f1c71"
EVIDENCE = ROOT / "evidence/integration/2026-11-07/stage2-mainline"
FAULTS = ("g_safety_fault", "g_rtos_fault", "g_can_ctrl_fault_latched",
          "g_lift_ruler_cause", "g_startup_axes_cause", "g_dm_jog_first_fault")


def capture(pose):
    import gripper210_attach_read_20261105 as probe

    if hashlib.sha256(IMAGE.read_bytes()).hexdigest() != IMAGE_SHA:
        raise RuntimeError("Reference AXF changed")
    probe.IMAGE = IMAGE
    names, segments = probe.load_image()
    with probe.attach() as session:
        target = session.target
        if target.get_state().name != "RUNNING":
            raise RuntimeError("Target is not running")
        matched = 0
        for address, data in segments:
            for offset in range(0, len(data), 4096):
                expected = data[offset:offset + 4096]
                if bytes(target.read_memory_block8(address + offset, len(expected))) != expected:
                    raise RuntimeError("Board flash differs from the reference AXF")
                matched += len(expected)

        def read(name):
            return target.read32(names[name])

        def degrees(name):
            return struct.unpack("<f", struct.pack("<I", read(name)))[0]

        if (any(read(name) for name in FAULTS) or read("g_safety_ready") != 1
                or read("g_it_state") not in (0, 2) or read("g_it_request") != 0
                or read("g_lift_compare_top_valid") != 1 or read("g_lift_ruler_state") != 4
                or read("g_startup_axes_state") != 4 or read("g_extension_hold_confirmed") != 1
                or read("g_chassis_auto_active") or read("g_vehicle_arm_state")
                or read("g_dm_posvel_holding") or read("g_dm_jog_request")):
            raise RuntimeError("Board is not idle with live lift/extension references and DM not holding")
        previous = read("g_dm_can2_read_cycle")
        target.write32(names["g_it_request"], 7)
        end = time.monotonic() + 16.0
        while time.monotonic() < end:
            if any(read(name) for name in FAULTS) or read("g_it_state") == 9:
                raise RuntimeError("IT7 failed; no angle accepted")
            if read("g_it_action") == 7 and read("g_it_request") == 0:
                if read("g_it_rejected"):
                    raise RuntimeError("IT7 rejected; no angle accepted")
                if read("g_it_state") == 2 and read("g_dm_can2_read_cycle") != previous:
                    break
            time.sleep(0.05)
        else:
            raise RuntimeError("IT7 did not complete")
        if (read("g_dm_can2_result") != 1 or read("g_dm_can2_feedback_valid") != 1
                or read("g_dm_can2_feedback_tick") == 0):
            raise RuntimeError("No fresh valid 4310 response")

        now = datetime.now()
        record = {
            "pose": pose, "when": now.isoformat(timespec="seconds"), "flash_bytes": matched,
            "request": 7, "read_cycle": read("g_dm_can2_read_cycle"),
            "feedback_tick": read("g_dm_can2_feedback_tick"),
            "raw_deg": degrees("g_dm_can2_position_deg"),
            "old_logical_deg": degrees("g_dm_can2_logical_position_deg"),
            "dm_velocity_rad_s": degrees("g_dm_can2_velocity_rad_s"),
            "dm_motor_state": read("g_dm_can2_motor_state"),
            "vm_pe2": (target.read32(0x40021014) >> 2) & 1,
            "lift_feedback_raw": read("g_lift_ruler_after_raw"),
            "extension_deg": degrees("g_startup_extension_current_deg"),
        }
    path = EVIDENCE / f"arm-teach-{pose}-{now:%Y%m%d-%H%M%S}.json"
    with path.open("x", encoding="utf-8") as stream:
        json.dump(record, stream, indent=2)
    print(json.dumps(record, indent=2))
    print(path)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pose", choices=("left-zero", "left-pick", "slot1"))
    capture(parser.parse_args().pose)
