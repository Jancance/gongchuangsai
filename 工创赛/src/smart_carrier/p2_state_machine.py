from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class P2Phase(str, Enum):
    WAIT_TASK = "WAIT_TASK"
    NAVIGATE = "NAVIGATE"
    LOCALIZE_TARGET = "LOCALIZE_TARGET"
    WAIT_P1 = "WAIT_P1"
    VERIFY = "VERIFY"
    RECOVER = "RECOVER"
    SAFE_STOP = "SAFE_STOP"
    COMPLETE = "COMPLETE"


@dataclass(frozen=True)
class P2Transition:
    previous: P2Phase
    current: P2Phase
    reason: str
    retries: int


class P2StateMachine:
    """Finite recovery state machine; never waits indefinitely for perception/P1."""

    def __init__(self, max_retries: int = 2, decision_deadline_ms: int = 3000) -> None:
        self.phase = P2Phase.WAIT_TASK
        self.max_retries = max_retries
        self.decision_deadline_ms = decision_deadline_ms
        self.retries = 0

    def transition(self, target: P2Phase, reason: str) -> P2Transition:
        previous = self.phase
        self.phase = target
        if target not in (P2Phase.RECOVER, P2Phase.SAFE_STOP):
            self.retries = 0
        return P2Transition(previous, target, reason, self.retries)

    def perception_failure(self, reason: str, elapsed_ms: int) -> P2Transition:
        previous = self.phase
        self.retries += 1
        if self.retries > self.max_retries or elapsed_ms >= self.decision_deadline_ms:
            self.phase = P2Phase.SAFE_STOP
        else:
            self.phase = P2Phase.RECOVER
        return P2Transition(previous, self.phase, reason, self.retries)
