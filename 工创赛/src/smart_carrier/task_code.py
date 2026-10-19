from __future__ import annotations

from collections import Counter, deque
from dataclasses import dataclass
import re


TASK_RE = re.compile(r"^(?P<b1>[1-6]{3})\+(?P<p1>[1-3]{3})\+(?P<b2>[1-6]{3})\+(?P<p2>[1-3]{3})$")


@dataclass(frozen=True)
class BatchTask:
    pickup_order: tuple[int, int, int]
    rough_positions: tuple[int, int, int]
    temporary_positions: tuple[int, int, int] | None = None
    stacking_positions: tuple[int, int, int] | None = None


@dataclass(frozen=True)
class TaskCode:
    raw_text: str
    parser_version: str
    start_id: int
    batch_1: BatchTask
    batch_2: BatchTask
    executable: bool
    rejection_reason: str | None = None


def _digits(value: str) -> tuple[int, int, int]:
    return tuple(int(ch) for ch in value)  # type: ignore[return-value]


def parse_task_code(
    text: str,
    *,
    start_id: int,
    parser_version: str = "p2-four-segment-v1",
    official_mapping_confirmed: bool = False,
    require_unique_pickup_symbols: bool = False,
) -> TaskCode:
    """Parse syntax now; keep execution locked until QR-04 mapping is confirmed."""

    cleaned = text.strip()
    match = TASK_RE.fullmatch(cleaned)
    if match is None:
        raise ValueError("task code must be DDD+DDD+DDD+DDD with documented ranges")
    if start_id not in (1, 2):
        raise ValueError("start_id must be 1 or 2")

    b1_order, b1_positions = _digits(match["b1"]), _digits(match["p1"])
    b2_order, b2_positions = _digits(match["b2"]), _digits(match["p2"])
    if require_unique_pickup_symbols and (len(set(b1_order)) != 3 or len(set(b2_order)) != 3):
        raise ValueError("pickup order contains duplicate color symbols")
    if len(set(b1_positions)) != 3 or len(set(b2_positions)) != 3:
        raise ValueError("position segment must be a permutation of 1,2,3")

    reason = None if official_mapping_confirmed else "QR-04_MAPPING_UNCONFIRMED"
    return TaskCode(
        raw_text=cleaned,
        parser_version=parser_version,
        start_id=start_id,
        # QR-04尚未确认每个位置段在不同功能区的最终语义。
        # 当前数据结构保留两个显式目的地字段；同一段暂作两处映射，
        # execution_lock会阻止它在官方映射确认前驱动物理动作。
        batch_1=BatchTask(
            pickup_order=b1_order,
            rough_positions=b1_positions,
            temporary_positions=b1_positions,
        ),
        batch_2=BatchTask(
            pickup_order=b2_order,
            rough_positions=b2_positions,
            stacking_positions=b2_positions,
        ),
        executable=official_mapping_confirmed,
        rejection_reason=reason,
    )


@dataclass(frozen=True)
class ScanDecision:
    value: str | None
    accepted: bool
    reason: str


class DualChannelConsensus:
    """Require temporal stability and reject scanner/OpenCV disagreement."""

    def __init__(self, window: int = 5, minimum_votes: int = 3) -> None:
        if not 1 <= minimum_votes <= window:
            raise ValueError("minimum_votes must be within window")
        self.window = window
        self.minimum_votes = minimum_votes
        self._samples: dict[str, deque[str]] = {
            "scanner": deque(maxlen=window),
            "opencv": deque(maxlen=window),
        }

    def add(self, source: str, value: str) -> ScanDecision:
        if source not in self._samples:
            raise ValueError(f"unknown scan source: {source}")
        self._samples[source].append(value.strip())
        winners: dict[str, str] = {}
        for name, samples in self._samples.items():
            if not samples:
                continue
            candidate, count = Counter(samples).most_common(1)[0]
            if count >= self.minimum_votes:
                winners[name] = candidate
        if len(winners) == 2 and len(set(winners.values())) != 1:
            return ScanDecision(None, False, "CHANNEL_CONFLICT")
        if winners:
            value_out = next(iter(winners.values()))
            return ScanDecision(value_out, True, "CONSENSUS")
        return ScanDecision(None, False, "INSUFFICIENT_STABLE_FRAMES")
