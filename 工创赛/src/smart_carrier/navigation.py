from __future__ import annotations

from dataclasses import dataclass
import heapq
import math

import cv2
import numpy as np

from .models import IntersectionObservation, LaneObservation, Obstacle


@dataclass(frozen=True)
class LaneEstimate:
    mask: np.ndarray
    lateral_error_mm: float
    heading_error_rad: float
    visible_distance_mm: float
    confidence: float
    rejection_reason: str | None


class GrayLaneDetector:
    """Bird's-eye gray-lane detector; thresholds are configuration, not rules."""

    def __init__(
        self,
        *,
        mm_per_pixel: float,
        min_l: int = 35,
        max_l: int = 220,
        max_chroma: int = 18,
        min_area_ratio: float = 0.08,
    ) -> None:
        self.mm_per_pixel = mm_per_pixel
        self.min_l = min_l
        self.max_l = max_l
        self.max_chroma = max_chroma
        self.min_area_ratio = min_area_ratio

    def detect(self, birdseye_bgr: np.ndarray) -> LaneEstimate:
        lab = cv2.cvtColor(birdseye_bgr, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        chroma = np.maximum(np.abs(a.astype(np.int16) - 128), np.abs(b.astype(np.int16) - 128))
        mask = ((l >= self.min_l) & (l <= self.max_l) & (chroma <= self.max_chroma)).astype(np.uint8) * 255
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
        if count <= 1:
            return LaneEstimate(mask * 0, 0.0, 0.0, 0.0, 0.0, "NO_LANE_COMPONENT")
        best = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        lane = np.where(labels == best, 255, 0).astype(np.uint8)
        area_ratio = float(np.mean(lane > 0))
        if area_ratio < self.min_area_ratio:
            return LaneEstimate(lane, 0.0, 0.0, 0.0, area_ratio, "LANE_AREA_TOO_SMALL")
        height, width = lane.shape
        samples: list[tuple[float, float]] = []
        for y in np.linspace(height * 0.35, height - 1, 8).astype(int):
            xs = np.flatnonzero(lane[y])
            if xs.size >= 2:
                samples.append((float(y), float((xs[0] + xs[-1]) / 2)))
        if len(samples) < 4:
            return LaneEstimate(lane, 0.0, 0.0, 0.0, area_ratio, "INSUFFICIENT_VISIBLE_ROWS")
        ys = np.array([s[0] for s in samples])
        xs = np.array([s[1] for s in samples])
        slope, intercept = np.polyfit(ys, xs, 1)
        center_bottom = slope * (height - 1) + intercept
        lateral = (center_bottom - width / 2.0) * self.mm_per_pixel
        heading = math.atan(float(slope))
        visible = (max(ys) - min(ys)) * self.mm_per_pixel
        confidence = min(1.0, area_ratio / max(self.min_area_ratio * 3.0, 1e-6))
        return LaneEstimate(lane, lateral, heading, visible, confidence, None)


class IntersectionHysteresis:
    def __init__(self, enter_frames: int = 3, exit_frames: int = 3) -> None:
        self.enter_frames = enter_frames
        self.exit_frames = exit_frames
        self._candidate: tuple[str, ...] = ()
        self._candidate_count = 0
        self._missing_count = 0
        self._active: tuple[str, ...] = ()
        self._event_id = 0

    def update(self, branches: tuple[str, ...]) -> tuple[tuple[str, ...], int]:
        normalized = tuple(sorted(set(branches)))
        if normalized:
            self._missing_count = 0
            if normalized == self._candidate:
                self._candidate_count += 1
            else:
                self._candidate, self._candidate_count = normalized, 1
            if self._candidate_count >= self.enter_frames and normalized != self._active:
                self._active = normalized
                self._event_id += 1
        else:
            self._candidate, self._candidate_count = (), 0
            self._missing_count += 1
            if self._missing_count >= self.exit_frames:
                self._active = ()
        return self._active, self._event_id


def all_corners_inside_zone(corners_xy_mm: tuple[tuple[float, float], ...], zone: tuple[float, float, float, float]) -> bool:
    x0, y0, x1, y1 = zone
    return bool(corners_xy_mm) and all(x0 <= x <= x1 and y0 <= y <= y1 for x, y in corners_xy_mm)


class TopologyPlanner:
    """Dijkstra route planner with runtime-blockable edges."""

    def __init__(self, edges: dict[str, dict[str, float]]) -> None:
        self.edges = edges

    def route(self, start: str, goal: str, blocked: set[tuple[str, str]] | None = None) -> tuple[str, ...]:
        blocked = blocked or set()
        queue: list[tuple[float, str, tuple[str, ...]]] = [(0.0, start, (start,))]
        seen: dict[str, float] = {}
        while queue:
            cost, node, path = heapq.heappop(queue)
            if node == goal:
                return path
            if cost >= seen.get(node, float("inf")):
                continue
            seen[node] = cost
            for neighbor, edge_cost in self.edges.get(node, {}).items():
                if (node, neighbor) in blocked or (neighbor, node) in blocked:
                    continue
                heapq.heappush(queue, (cost + edge_cost, neighbor, path + (neighbor,)))
        return ()


def conservative_obstacle_fusion(
    visual: tuple[Obstacle, ...],
    tof_mm: tuple[float | None, float | None, float | None],
    *,
    stop_threshold_mm: float,
) -> tuple[bool, str]:
    if any(distance is None for distance in tof_mm):
        return True, "TOF_UNKNOWN"
    if min(distance for distance in tof_mm if distance is not None) < stop_threshold_mm:
        return True, "TOF_STOP"
    if visual:
        return True, "VISUAL_OBSTACLE"
    return False, "CLEAR"
