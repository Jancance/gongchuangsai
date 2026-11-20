"""Validate calibrated ring occupancy on independent host-fresh frames."""

import json
import math
import time

import cv2
import numpy as np

from 颜色标定 import make_all_color_masks


class StorageObservation:
    def __init__(self, path, colors):
        self.config = json.loads(path.read_text(encoding="utf-8"))
        self.colors = colors
        for profile in self.config["modes"].values():
            if profile.get("verified") is not True:
                continue
            if len(profile["rings"]) != 3:
                raise ValueError("Verified storage mode requires three calibrated ring records")
            for ring in profile["rings"]:
                self.validate_ring(ring)
                if profile is self.config["modes"].get("4") and ring.get("top_height_separates_base") is not True:
                    raise ValueError("Stack confirmation requires a top-only sample validated against the base layer")
        self.sequence = 0
        self.reset()

    @staticmethod
    def validate_ring(ring):
        if (len(ring["offset_px"]) != 2 or int(ring["radius_px"]) < 2
                or not 0 <= ring["empty_color_max"] < ring["occupied_color_min"] <= 1
                or not 0 < ring["empty_background_min"] <= 1):
            raise ValueError("Invalid storage ring sample geometry or fractions")
        low, high = (ring["empty_hsv_low"], ring["empty_hsv_high"])
        if (len(low) != 3 or len(high) != 3
                or any(not 0 <= a <= b <= maximum
                       for a, b, maximum in zip(low, high, (179, 255, 255)))):
            raise ValueError("Invalid storage empty background HSV")

    def reset(self):
        self.reset_history()
        self.last_sent = None

    def reset_history(self):
        self.history = None
        self.count = 0

    def observe(self, station, image, result, state, acquired_after, geometry_verified):
        modes = self.config.get("modes", {})
        profile = modes.get(str(station.mode), {})
        if (not geometry_verified or profile.get("verified") is not True
                or not station.ring or not station.color
                or state != 5 or result is None
                or not result.detected or not result.stable):
            self.reset_history()
            return None
        occupied = self.classify(station, image, result, profile)
        if occupied is None:
            self.reset_history()
            return None
        key = station, state, occupied
        self.count = self.count + 1 if key == self.history else 1
        self.history = key
        now = time.monotonic()
        age = math.ceil((now - acquired_after) * 1000)
        if (self.count < 3 or not 0 <= age <= 200
                or self.last_sent is not None and now - self.last_sent < .1):
            return None
        self.sequence = (self.sequence + 1) & 0xFFFFFFFF
        self.last_sent = now
        return station.observation(self.sequence, int(occupied), age)

    def classify(self, station, image, result, profile):
        centers = getattr(result, "centers_px", ())
        if len(centers) != 3:
            return None
        cfg = profile["rings"][station.ring - 1]
        cx, cy = centers[station.ring - 1]
        dx, dy = cfg["offset_px"]
        cx, cy = round(cx + dx), round(cy + dy)
        radius = int(cfg["radius_px"])
        empty_max, occupied_min = cfg["empty_color_max"], cfg["occupied_color_min"]
        if (radius < 2 or not 0 <= empty_max < occupied_min <= 1
                or cx - radius < 0 or cx + radius >= image.shape[1]
                or cy - radius < 0 or cy + radius >= image.shape[0]):
            return None
        sample = image[cy-radius:cy+radius+1, cx-radius:cx+radius+1]
        disk = np.zeros(sample.shape[:2], dtype=np.uint8)
        cv2.circle(disk, (radius, radius), radius, 1, -1)
        masks = make_all_color_masks(sample, self.colors)
        support = np.count_nonzero(disk)
        fractions = {color: np.count_nonzero(mask & disk) / support
                     for color, mask in masks.items()}
        if (fractions[station.color] >= occupied_min
                and all(v <= empty_max for c, v in fractions.items() if c != station.color)):
            return True
        if all(v <= empty_max for v in fractions.values()):
            if station.mode == 4:
                return None
            hsv = cv2.cvtColor(sample, cv2.COLOR_BGR2HSV)
            background = cv2.inRange(hsv, np.array(cfg["empty_hsv_low"], dtype=np.uint8),
                                     np.array(cfg["empty_hsv_high"], dtype=np.uint8))
            if np.count_nonzero(background & disk) / support >= cfg["empty_background_min"]:
                return False
        return None
