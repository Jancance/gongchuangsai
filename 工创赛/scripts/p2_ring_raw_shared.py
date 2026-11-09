"""原算法的机械拆分：提取可复用中间图，原几何计算不改。
来源 ring_station_detector.py SHA256 d614953a9357bc5c1f10b43e6198630cf3a278893e6dcf6d533d01a0ce4cfb08。
由原函数复制拆分；每个回归输入对照原raw结果，不修改原文件。
"""
import math
import cv2
import numpy as np
from ring_station_detector import _EllipseObservation

def extract_with_edges(image, config):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 1.2)
    edges = cv2.Canny(blurred, config.canny_low, config.canny_high)
    contours, _ = cv2.findContours(
        edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE
    )
    observations = []
    for contour in contours:
        if len(contour) < config.min_contour_points:
            continue
        center, axes, angle = cv2.fitEllipse(contour)
        major, minor = max(axes), min(axes)
        if (
            minor < config.min_minor_axis_px
            or major > config.max_major_axis_px
            or minor / major < config.min_ellipse_aspect
        ):
            continue
        observations.append(_EllipseObservation(
            center=np.asarray(center, dtype=np.float64),
            axes=(float(axes[0]), float(axes[1])),
            angle_deg=float(angle),
            major_axis=float(major),
            minor_axis=float(minor),
        ))
    return observations, blurred, edges

def raw_from_candidates(candidates, image_width, config, module):
    selection = module._select_triple(candidates, image_width, config)
    if selection is None:
        return None
    _, ordered, collinearity, _ = selection
    spacing_12 = float(np.linalg.norm(ordered[1].center - ordered[0].center))
    spacing_23 = float(np.linalg.norm(ordered[2].center - ordered[1].center))
    perspective = spacing_12 / spacing_23
    geometry = math.exp(
        -10.0 * collinearity - 0.5 * abs(math.log(perspective))
    )
    support = min(item.support for item in ordered) / 10.0
    bands = min(item.diameter_bands for item in ordered) / 6.0
    confidence = min(1.0, 0.55 * geometry + 0.25 * min(support, 1.0) + 0.20 * min(bands, 1.0))

    # 使用中间环椭圆和三环中心距建立与旧实现一致的局部仿射质量指标。
    image_x_per_mm = (
        ordered[2].center - ordered[0].center
    ) / (2.0 * config.center_spacing_mm)
    outer = ordered[1].ellipse
    theta = math.radians(outer.angle_deg)
    rotation = np.asarray([
        [math.cos(theta), -math.sin(theta)],
        [math.sin(theta), math.cos(theta)],
    ])
    ellipse_shape = rotation @ np.diag([
        (outer.axes[0] * 0.5) ** 2,
        (outer.axes[1] * 0.5) ** 2,
    ]) @ rotation.T
    radius_mm = config.outer_diameter_mm * 0.5
    remaining = ellipse_shape / (radius_mm * radius_mm) - np.outer(
        image_x_per_mm, image_x_per_mm
    )
    eigenvalues = np.linalg.eigvalsh((remaining + remaining.T) * 0.5)
    largest = max(float(eigenvalues[-1]), 0.0)
    residual = (
        abs(float(eigenvalues[0])) / largest if largest > 1e-9 else math.inf
    )
    return {
        "centers": tuple(tuple(float(v) for v in item.center) for item in ordered),
        "ellipses": tuple((
            tuple(float(v) for v in item.ellipse.center),
            tuple(float(v) for v in item.ellipse.axes),
            float(item.ellipse.angle_deg),
        ) for item in ordered),
        "confidence": float(confidence),
        "yaw": math.degrees(math.atan2(
            float(ordered[2].center[1] - ordered[0].center[1]),
            float(ordered[2].center[0] - ordered[0].center[0]),
        )),
        "perspective": float(perspective),
        "residual": float(residual),
    }
