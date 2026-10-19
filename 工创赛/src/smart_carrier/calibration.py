from __future__ import annotations

import cv2
import numpy as np


def estimate_ground_homography(
    image_points: np.ndarray,
    ground_points_mm: np.ndarray,
) -> tuple[np.ndarray, float, np.ndarray]:
    image = np.asarray(image_points, dtype=np.float64)
    ground = np.asarray(ground_points_mm, dtype=np.float64)
    if image.shape != ground.shape or image.ndim != 2 or image.shape[1] != 2 or image.shape[0] < 4:
        raise ValueError("image and ground points must both have shape Nx2 with N>=4")
    matrix, inliers = cv2.findHomography(image, ground, cv2.RANSAC, 3.0)
    if matrix is None or inliers is None or int(inliers.sum()) < 4:
        raise ValueError("homography estimation failed or has fewer than four inliers")
    projected = cv2.perspectiveTransform(image.reshape(-1, 1, 2), matrix).reshape(-1, 2)
    residuals = np.linalg.norm(projected - ground, axis=1)
    rms_mm = float(np.sqrt(np.mean(np.square(residuals[inliers.ravel().astype(bool)]))))
    condition = float(np.linalg.cond(matrix))
    if not np.isfinite(condition) or condition > 1e12:
        raise ValueError("homography is ill-conditioned")
    return matrix / matrix[2, 2], rms_mm, inliers.ravel().astype(bool)

