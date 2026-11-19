from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum
from typing import Any


class Zone(str, Enum):
    SOURCE = 'source'
    ROUGH = 'rough'
    STORAGE = 'storage'


class Operation(str, Enum):
    LOAD_SOURCE = 'load_source'
    MANUAL_LOAD = 'manual_load'
    PLACE = 'place'
    RECOVER = 'recover'
    STACK = 'stack'


class Phase(str, Enum):
    IDLE = 'IDLE'
    TRAVEL = 'TRAVEL'
    WORK = 'WORK'
    WAIT_PARAMETER = 'WAIT_PARAMETER'
    WAIT_VISION = 'WAIT_VISION'
    WAIT_EVIDENCE = 'WAIT_EVIDENCE'
    DONE = 'DONE'
    STOPPING = 'STOPPING'
    STOPPED = 'STOPPED'


@dataclass(frozen=True)
class Job:
    context_id: str
    zone: Zone
    operation: Operation
    colors: tuple[int, ...]
    positions: tuple[int, ...]
    carrier_slots: tuple[int, ...] = (1, 2, 3)
    task_source: str = 'scanner'
    supervised: bool = False


@dataclass(frozen=True)
class Action:
    context_id: str
    sequence: int
    kind: str
    goal: dict[str, Any]
    evidence_required: tuple[str, ...] = ()


@dataclass(frozen=True)
class Feedback:
    context_id: str
    sequence: int
    status: str  # ACCEPTED/DONE/FAILED/TRANSIENT/PARAMETER_REJECTED
    evidence: dict[str, Any] = field(default_factory=dict)
    reason: str = ''


@dataclass(frozen=True)
class Arrival:
    context_id: str
    zone: Zone
    arrived: bool
    stopped: bool
    status: str = 'DONE'
    source: str = 'chassis'
    sequence: int = 0


@dataclass(frozen=True)
class Target:
    context_id: str
    instance_id: str
    color_code: int
    pose: dict[str, float]
    observed_ms: int
    present: bool
    position_confirmed: bool
    stationary: bool
    identity_source: str = 'VISION_CONFIRMED'
    usable_until_ms: int | None = None
