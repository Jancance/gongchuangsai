from __future__ import annotations

import argparse
import json
import logging
from pathlib import Path
import signal
import time

import numpy as np

from .camera import LatestFrameCamera
from .config import load_config_dir, validate_deployment_config
from .models import Frame, RobotMode, RobotState
from .planner import LocalAvoidancePlanner
from .state_machine import SafetyStateMachine
from .vision import DarkObstacleDetector, TemporalObstacleFilter

LOG = logging.getLogger("smart_carrier")


def run(config_dir: str, simulate: bool, cycles: int) -> int:
    config = load_config_dir(config_dir)
    deployment_errors = validate_deployment_config(config)
    if deployment_errors and not simulate:
        for error in deployment_errors:
            LOG.error(error)
        return 2
    machine = SafetyStateMachine()
    machine.transition(RobotMode.SELF_TEST, "configuration loaded")
    camera_cfg = config["cameras"]["cameras"]["front"]
    vision_cfg = config["vision"]["front_camera"]
    detector = DarkObstacleDetector.from_config(vision_cfg)
    temporal_filter = TemporalObstacleFilter(
        int(vision_cfg["stable_frames"]),
        float(vision_cfg["stability_tolerance_mm"]),
    )
    hardware = config["hardware"]
    planner = LocalAvoidancePlanner(
        float(hardware["robot"]["chassis_width_mm"]),
        float(hardware["robot"]["boundary_margin_mm"]),
        float(camera_cfg["max_frame_age_ms"]),
        float(vision_cfg["min_confidence"]),
    )
    camera = None if simulate else LatestFrameCamera("front", camera_cfg)
    stop = False

    def request_stop(*_: object) -> None:
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        if camera:
            camera.start()
        machine.transition(RobotMode.READY, "self-test passed")
        machine.transition(RobotMode.RUNNING, "one-key start accepted")
        processed = 0
        while not stop and (cycles <= 0 or processed < cycles):
            if simulate:
                now = time.monotonic_ns()
                image = np.full((720, 1280, 3), 150, dtype=np.uint8)
                frame = Frame(image, now, processed + 1, "front", detector.calibration_id)
            else:
                frame = camera.read_latest() if camera else None
                if frame is None:
                    machine.safe_stop("front camera has no current frame")
                    time.sleep(0.1)
                    continue
            observations, _ = detector.process(frame)
            observations = temporal_filter.update(observations)
            state = RobotState(time.monotonic_ns(), machine.mode, stm32_heartbeat_age_ms=0.0, tof_min_mm=1000.0)
            plan = planner.plan(observations, state, now_ns=frame.timestamp_ns if simulate else None)
            print(json.dumps({"mode": machine.mode.value, "plan": plan.action.value, "reason": plan.reason, "target_lateral_mm": plan.target_lateral_mm}))
            processed += 1
            time.sleep(0.02)
        if machine.mode == RobotMode.RUNNING:
            machine.transition(RobotMode.DONE, "requested cycles completed")
        return 0
    except Exception:
        machine.safe_stop("unhandled runtime exception")
        LOG.exception("runtime failed; SAFE_STOP requested")
        return 1
    finally:
        if camera:
            camera.stop()
        LOG.info("final state=%s reason=%s", machine.mode.value, machine.reason)


def main() -> None:
    parser = argparse.ArgumentParser(description="Smart carrier fail-safe runtime")
    parser.add_argument("--config", default="/opt/smart-carrier/config")
    parser.add_argument("--simulate", action="store_true")
    parser.add_argument("--cycles", type=int, default=0, help="0 means run until stopped")
    parser.add_argument("--log-level", default="INFO")
    args = parser.parse_args()
    logging.basicConfig(level=getattr(logging, args.log_level.upper()), format="%(asctime)s %(levelname)s %(name)s %(message)s")
    raise SystemExit(run(args.config, args.simulate, args.cycles))


if __name__ == "__main__":
    main()
