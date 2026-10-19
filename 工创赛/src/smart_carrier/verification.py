from __future__ import annotations

from dataclasses import dataclass

from .models import ActionResult, ActionStatus, PlacementVerification


@dataclass(frozen=True)
class EvidenceDecision:
    accepted: bool
    reason: str
    counter_increment: int


class EvidenceGate:
    """Count a physical result once, only with independent P1 and P2 evidence."""

    def __init__(self) -> None:
        self._committed_sequences: set[int] = set()

    def verify_tray_load(
        self,
        result: ActionResult,
        *,
        visual_present: bool,
        tray_sensor: bool,
    ) -> EvidenceDecision:
        if result.seq in self._committed_sequences:
            return EvidenceDecision(False, "DUPLICATE_SEQUENCE", 0)
        if result.status is not ActionStatus.DONE:
            return EvidenceDecision(False, "P1_ACTION_NOT_DONE", 0)
        if not visual_present:
            return EvidenceDecision(False, "NO_CURRENT_VISUAL_EVIDENCE", 0)
        if not tray_sensor:
            return EvidenceDecision(False, "TRAY_SENSOR_NOT_OCCUPIED", 0)
        self._committed_sequences.add(result.seq)
        return EvidenceDecision(True, "P1_P2_SENSOR_CONSENSUS", 1)

    def verify_placement(
        self,
        result: ActionResult,
        observation: PlacementVerification,
    ) -> EvidenceDecision:
        if result.seq != observation.action_seq:
            return EvidenceDecision(False, "SEQUENCE_MISMATCH", 0)
        if result.seq in self._committed_sequences:
            return EvidenceDecision(False, "DUPLICATE_SEQUENCE", 0)
        if result.status is not ActionStatus.DONE:
            return EvidenceDecision(False, "P1_ACTION_NOT_DONE", 0)
        if not observation.accepted or not observation.visual_present:
            return EvidenceDecision(False, "NO_CURRENT_VISUAL_EVIDENCE", 0)
        if not observation.centered or not observation.stable:
            return EvidenceDecision(False, "PLACEMENT_GEOMETRY_FAILED", 0)
        self._committed_sequences.add(result.seq)
        return EvidenceDecision(True, "P1_P2_PLACEMENT_CONFIRMED", 1)
