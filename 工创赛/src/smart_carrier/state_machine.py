from __future__ import annotations

from dataclasses import dataclass

from .models import RobotMode


class InvalidTransition(RuntimeError):
    pass


_ALLOWED = {
    RobotMode.BOOT: {RobotMode.SELF_TEST, RobotMode.FAULT},
    RobotMode.SELF_TEST: {RobotMode.READY, RobotMode.SAFE_STOP, RobotMode.FAULT},
    RobotMode.READY: {RobotMode.RUNNING, RobotMode.SAFE_STOP, RobotMode.FAULT},
    RobotMode.RUNNING: {RobotMode.SAFE_STOP, RobotMode.FAULT, RobotMode.DONE},
    RobotMode.SAFE_STOP: {RobotMode.SELF_TEST, RobotMode.FAULT, RobotMode.DONE},
    RobotMode.FAULT: {RobotMode.SELF_TEST},
    RobotMode.DONE: {RobotMode.SELF_TEST},
}


@dataclass
class SafetyStateMachine:
    mode: RobotMode = RobotMode.BOOT
    reason: str = "power-on default"

    def transition(self, target: RobotMode, reason: str) -> None:
        if target not in _ALLOWED[self.mode]:
            raise InvalidTransition(f"invalid transition {self.mode.value} -> {target.value}")
        self.mode = target
        self.reason = reason

    def safe_stop(self, reason: str) -> None:
        if self.mode in (RobotMode.SAFE_STOP, RobotMode.FAULT, RobotMode.DONE):
            self.reason = reason
            return
        self.transition(RobotMode.SAFE_STOP, reason)

