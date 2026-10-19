from __future__ import annotations

from dataclasses import dataclass
import time

import cv2
import numpy as np


@dataclass(frozen=True)
class FrameQuality:
    accepted: bool
    sharpness: float
    clipped_dark_ratio: float
    clipped_bright_ratio: float
    age_ms: float
    reasons: tuple[str, ...]


class FrameQualityGate:
    def __init__(
        self,
        *,
        min_sharpness: float = 35.0,
        max_clipped_ratio: float = 0.25,
        max_age_ms: float = 200.0,
    ) -> None:
        self.min_sharpness = min_sharpness
        self.max_clipped_ratio = max_clipped_ratio
        self.max_age_ms = max_age_ms

    def evaluate(self, image: np.ndarray, timestamp_ns: int, now_ns: int | None = None) -> FrameQuality:
        if image is None or image.size == 0:
            return FrameQuality(False, 0.0, 1.0, 1.0, float("inf"), ("EMPTY_FRAME",))
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
        sharpness = float(cv2.Laplacian(gray, cv2.CV_64F).var())
        dark = float(np.mean(gray <= 3))
        bright = float(np.mean(gray >= 252))
        age_ms = ((now_ns or time.monotonic_ns()) - timestamp_ns) / 1_000_000.0
        reasons: list[str] = []
        if sharpness < self.min_sharpness:
            reasons.append("BLUR")
        if dark > self.max_clipped_ratio:
            reasons.append("DARK_CLIPPING")
        if bright > self.max_clipped_ratio:
            reasons.append("BRIGHT_CLIPPING")
        if age_ms < 0 or age_ms > self.max_age_ms:
            reasons.append("STALE_FRAME")
        return FrameQuality(not reasons, sharpness, dark, bright, age_ms, tuple(reasons))
