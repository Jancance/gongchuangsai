"""仅只读attach：同步USB收发候选，无复位/暂停/下载/执行器写入。"""
from pathlib import Path
import hashlib
import json
import logging
import struct
import sys
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import gripper210_attach_read_20261105 as live
from pyocd.core.helpers import ConnectHelper
from pyocd.core.session import Session
from pyocd.core.target import Target
from libusb_package import find
import usb.util

OUT = ROOT / "evidence/integration/2026-11-05/source-markers/paper100-transfer/continuation/body270x250/top-motion-review"


def synchronous_interface(interface):
    """替换当前实例USB收发，不改安装包；单包顺序传输，不触发USB设备复位。"""
    def open_sync():
        if not interface.closed:
            return
        device = find(idVendor=interface.vid, idProduct=interface.pid,
                      custom_match=lambda d: d.serial_number == interface.serial_number)
        if device is None:
            raise RuntimeError("Expected probe USB device disappeared")
        config = device.get_active_configuration()
        selected = [i for i in config if i.bInterfaceClass == 255 and i.bInterfaceNumber == 0]
        if len(selected) != 1:
            raise RuntimeError("Expected the already-inspected CMSIS-DAPv2 interface")
        intf = selected[0]
        endpoints = list(intf)
        if [ep.bEndpointAddress for ep in endpoints[:2]] != [1, 0x81]:
            raise RuntimeError("Probe endpoints changed")
        usb.util.claim_interface(device, intf.bInterfaceNumber)
        interface.dev = device
        interface.intf_number = intf.bInterfaceNumber
        interface.ep_out, interface.ep_in = endpoints[:2]
        interface.packet_size = 64
        interface.closed = False

    def close_sync():
        if interface.closed:
            return
        try:
            usb.util.release_interface(interface.dev, interface.intf_number)
        finally:
            usb.util.dispose_resources(interface.dev)
            interface.closed = True
            interface.dev = None

    def write_sync(data):
        return interface.ep_out.write(data, timeout=1000)

    def read_sync():
        return bytes(interface.ep_in.read(interface.packet_size, timeout=1000))

    interface.open = open_sync
    interface.close = close_sync
    interface.write = write_sync
    interface.read = read_sync


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    logging.basicConfig(filename=str(OUT / "readonly-sync-debug.log"), level=logging.DEBUG, encoding="utf-8")
    options = {"target_override": "cortex_m", "connect_mode": "attach", "frequency": 1000000,
               "auto_unlock": False, "resume_on_disconnect": False, "dap_protocol": "swd", "no_config": True,
               "cmsis_dap.prefer_v1": False, "cmsis_dap.limit_packets": True}
    report = {"adapter": "instance-local synchronous CMSIS-DAPv2 USB transfers",
              "actuator_writes": 0, "reset_requested": False, "halt_requested": False,
              "flash_requested": False, "usb_device_reset_called": False, "stage": "enumeration"}
    session = None
    try:
        settings = Session(None, options=options)
        probes = ConnectHelper.get_all_connected_probes(blocking=False)
        if len(probes) != 1:
            raise RuntimeError(f"Expected exactly one probe, found {len(probes)}")
        probe = probes[0]
        interface = probe._link._interface
        if not getattr(interface, "is_bulk", False):
            raise RuntimeError("Only already-inspected bulk USB interface is permitted")
        report["interface_attributes"] = {k: getattr(interface, k, None) for k in ("vid", "pid", "serial_number")}
        synchronous_interface(interface)
        report["stage"] = "create_session"
        session = Session(probe, options=options)
        report["stage"] = "open_session"
        session.open()
        target = session.target
        report["stage"] = "core_state"
        report["core_state"] = target.get_state().name
        if target.get_state() != Target.State.RUNNING:
            raise RuntimeError("Core not running; do not resume or reset")
        symbols, segments = live.load_image()
        report["image_sha256"] = hashlib.sha256(live.IMAGE.read_bytes()).hexdigest()
        report["stage"] = "compare_flash"
        compared = 0
        for address, data in segments:
            for offset in range(0, len(data), 4096):
                expected = data[offset:offset+4096]
                if bytes(target.read_memory_block8(address+offset, len(expected))) != expected:
                    raise RuntimeError("Flash image mismatch; no symbol snapshot trusted")
                compared += len(expected)
        report["flash_matches"] = True
        report["flash_bytes_compared"] = compared
        report["snapshot"] = live.snapshot(target, symbols)
        report["lift_reference_snapshots"] = {}
        for name in ["g_vehicle_lift_reference_valid", "g_vehicle_lift_feedback_valid", "g_vehicle_lift_current_deg",
                     "g_lift_home_state", "g_lift_home_result", "g_lift_motion_state", "g_lift_motion_result",
                     "g_lift_motion_request", "g_lift_x_position_deg"]:
            if name in symbols:
                raw = target.read32(symbols[name])
                report["lift_reference_snapshots"][name] = struct.unpack("<f", struct.pack("<I", raw))[0] if name.endswith("_deg") else raw
        report["core_state_after"] = target.get_state().name
        report["stage"] = "complete"
    except Exception as exc:
        report["error_type"] = type(exc).__name__
        report["error_repr"] = repr(exc)
        report["traceback"] = traceback.format_exc()
    finally:
        if session is not None:
            try:
                session.close()
            except Exception as exc:
                report["close_error"] = repr(exc)
        (OUT / "readonly-sync-report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["stage"] == "complete" else 1


if __name__ == "__main__":
    raise SystemExit(main())
