from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np

from .models import Frame, Obstacle, Observations


TASK_CODE_RE = re.compile(r"^[1-6]{3}\+[1-3]{3}\+[1-6]{3}\+[1-3]{3}$")


def parse_task_code(value: str) -> tuple[tuple[int, int, int], ...]:
    value = value.strip()
    if not TASK_CODE_RE.fullmatch(value):
        raise ValueError(f"invalid contest task code: {value!r}")
    return tuple(tuple(int(char) for char in part) for part in value.split("+"))


def project_point(homography: np.ndarray, u: float, v: float) -> tuple[float, float]:
    point = homography @ np.array([u, v, 1.0], dtype=np.float64)
    if abs(point[2]) < 1e-9:
        raise ValueError("homography maps point to infinity")
    return float(point[0] / point[2]), float(point[1] / point[2])


@dataclass
class DarkObstacleDetector:
    homography: np.ndarray
    calibration_id: str
    roi_fraction: tuple[float, float, float, float]
    dark_l_max: int = 65
    min_ground_l_median: int = 70
    max_dark_fraction: float = 0.35
    min_area_px: int = 120
    max_area_fraction: float = 0.20
    kernel_px: int = 5
    obstacle_radius_mm: float = 25.0

    @classmethod
    def from_config(cls, data: dict[str, Any]) -> "DarkObstacleDetector":
        return cls(
            np.asarray(data["homography_image_to_ground"], dtype=np.float64),
            str(data["calibration_id"]),
            tuple(float(v) for v in data["roi_fraction"]),
            int(data["dark_l_max"]),
            int(data["min_ground_l_median"]),
            float(data["max_dark_fraction"]),
            int(data["min_component_area_px"]),
            float(data["max_component_area_fraction"]),
            int(data["morphology_kernel_px"]),
            float(data["obstacle_radius_mm"]),
        )

    def process(self, frame: Frame) -> tuple[Observations, np.ndarray]:
        if frame.calibration_id != self.calibration_id:
            observation = Observations(
                frame.sequence,
                frame.timestamp_ns,
                frame.calibration_id,
                0.0,
                rejection_reason="frame and vision calibration IDs differ",
            )
            return observation, np.zeros(frame.image.shape[:2], dtype=np.uint8)
        image = np.asarray(frame.image)
        if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
            raise ValueError("vision input contract is uint8 HxWx3 BGR")
        height, width = image.shape[:2]
        x0, y0, x1, y1 = self.roi_fraction
        left, top = int(width * x0), int(height * y0)
        right, bottom = int(width * x1), int(height * y1)
        if not (0 <= left < right <= width and 0 <= top < bottom <= height):
            raise ValueError("invalid ROI")
        roi = image[top:bottom, left:right]
        lab = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)
        mask = cv2.inRange(lab[:, :, 0], 0, self.dark_l_max)
        kernel_size = max(1, self.kernel_px | 1)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        dark_fraction = cv2.countNonZero(mask) / float(mask.shape[0] * mask.shape[1])
        ground_l_median = float(np.median(lab[:, :, 0]))
        if dark_fraction > self.max_dark_fraction or ground_l_median < self.min_ground_l_median:
            full_mask = np.zeros((height, width), dtype=np.uint8)
            full_mask[top:bottom, left:right] = mask
            return (
                Observations(
                    frame.sequence,
                    frame.timestamp_ns,
                    self.calibration_id,
                    0.0,
                    rejection_reason="ground scene is obscured or too dark",
                ),
                full_mask,
            )

        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
        obstacles: list[Obstacle] = []
        roi_area = roi.shape[0] * roi.shape[1]
        for label in range(1, count):
            x, y, w, h, area = (int(value) for value in stats[label])
            if area < self.min_area_px or area > roi_area * self.max_area_fraction:
                continue
            if x <= 0 or y <= 0 or x + w >= roi.shape[1] or y + h >= roi.shape[0]:
                continue
            u = left + x + w / 2.0
            v = top + y + h
            ground_x, ground_y = project_point(self.homography, u, v)
            fill = area / max(1.0, float(w * h))
            confidence = float(np.clip(0.45 + 0.55 * fill, 0.0, 1.0))
            obstacles.append(Obstacle(ground_x, ground_y, self.obstacle_radius_mm, confidence))

        confidence = min((item.confidence for item in obstacles), default=1.0)
        full_mask = np.zeros((height, width), dtype=np.uint8)
        full_mask[top:bottom, left:right] = mask
        return (
            Observations(frame.sequence, frame.timestamp_ns, self.calibration_id, confidence, tuple(obstacles)),
            full_mask,
        )


def decode_qr(image_bgr: np.ndarray) -> str | None:
    value, _, _ = cv2.QRCodeDetector().detectAndDecode(image_bgr)
    value = value.strip()
    if not value:
        return None
    parse_task_code(value)
    return value


def classify_color_hsv(image_bgr: np.ndarray, ranges: dict[str, list[list[int]]]) -> tuple[str, float]:
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    total = float(hsv.shape[0] * hsv.shape[1])
    scores: dict[str, float] = {}
    for name, bounds in ranges.items():
        lower = np.asarray(bounds[0], dtype=np.uint8)
        upper = np.asarray(bounds[1], dtype=np.uint8)
        scores[name] = cv2.countNonZero(cv2.inRange(hsv, lower, upper)) / total
    name = max(scores, key=scores.get)
    return name, scores[name]


class TemporalObstacleFilter:
    """Requires consecutive geometrically consistent observations before motion."""

    def __init__(self, required_frames: int = 3, tolerance_mm: float = 60.0) -> None:
        if required_frames < 1:
            raise ValueError("required_frames must be positive")
        self.required_frames = required_frames
        self.tolerance_mm = tolerance_mm
        self._last: Observations | None = None
        self._stable_count = 0

    def update(self, current: Observations) -> Observations:
        if current.rejection_reason:
            self._last = None
            self._stable_count = 0
            return current
        if self._last is not None and self._consistent(self._last, current):
            self._stable_count += 1
        else:
            self._stable_count = 1
        self._last = current
        if self._stable_count >= self.required_frames:
            return current
        return Observations(
            current.frame_id,
            current.timestamp_ns,
            current.calibration_id,
            0.0,
            current.obstacles,
            current.lane_left_mm,
            current.lane_right_mm,
            f"waiting for temporal stability ({self._stable_count}/{self.required_frames})",
        )

    def _consistent(self, previous: Observations, current: Observations) -> bool:
        if previous.calibration_id != current.calibration_id or len(previous.obstacles) != len(current.obstacles):
            return False
        left = sorted(previous.obstacles, key=lambda item: (item.x_mm, item.y_mm))
        right = sorted(current.obstacles, key=lambda item: (item.x_mm, item.y_mm))
        return all(
            (a.x_mm - b.x_mm) ** 2 + (a.y_mm - b.y_mm) ** 2 <= self.tolerance_mm**2
            for a, b in zip(left, right)
        )
