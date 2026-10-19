from __future__ import annotations

from collections import deque
import logging
import threading
import time
from typing import Any

import cv2

from .models import Frame

LOG = logging.getLogger(__name__)


class LatestFrameCamera:
    """V4L2 capture that keeps only the newest frame and reopens after failure."""

    def __init__(self, source_id: str, config: dict[str, Any]) -> None:
        self.source_id = source_id
        self.config = config
        self._frames: deque[Frame] = deque(maxlen=1)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._sequence = 0
        self.failures = 0

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name=f"camera-{self.source_id}", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=3.0)

    def read_latest(self) -> Frame | None:
        with self._lock:
            return self._frames[-1] if self._frames else None

    def _open(self) -> cv2.VideoCapture:
        cap = cv2.VideoCapture(str(self.config["device_by_id"]), cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.config["width"]))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.config["height"]))
        cap.set(cv2.CAP_PROP_FPS, int(self.config["fps"]))
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*str(self.config["fourcc"])))
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap

    def _run(self) -> None:
        cap: cv2.VideoCapture | None = None
        while not self._stop.is_set():
            if cap is None or not cap.isOpened():
                if cap is not None:
                    cap.release()
                cap = self._open()
                if not cap.isOpened():
                    self.failures += 1
                    self._stop.wait(1.0)
                    continue
            ok, image = cap.read()
            if not ok or image is None:
                self.failures += 1
                cap.release()
                cap = None
                self._stop.wait(0.2)
                continue
            self._sequence += 1
            frame = Frame(image, time.monotonic_ns(), self._sequence, self.source_id, str(self.config["calibration_id"]))
            with self._lock:
                self._frames.append(frame)
        if cap is not None:
            cap.release()

