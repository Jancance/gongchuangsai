#!/usr/bin/env python3
"""无 ROS 的中圆环加上下长直线码垛对位检测。

该模式移植自旧工程 ``stacked_ring_alignment.py``：只检测可能仍可见的
中间圆环来计算工位平移，并在圆心上下约 75 mm 的窄带内检测两条长边，
双线时以共同方向、单线时以该线方向计算工位 yaw。最后按照标定基准重建三个圆心，使结果
可以继续复用现有三圆环串口上报和底盘纠偏协议。
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, replace
import hashlib
import os
import tempfile
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np

from ring_station_detector import RingStationConfig
from ring_station_detector import _extract_observations, _cluster_observations, _select_triple


class StackedRingDetectionError(ValueError):
    """中环、长边或配置无法形成可靠码垛观测。"""


@dataclass(frozen=True)
class StackedRingConfig:
    """中环 Hough 和上下长边 Hough 的全部可调参数。"""

    search_half_width_px: int
    search_half_height_px: int
    maximum_reference_distance_px: float
    circle_hough_dp: float
    circle_hough_min_distance_px: float
    circle_canny_high: float
    circle_accumulator_threshold: float
    edge_canny_low: float
    edge_canny_high: float
    minimum_radius_px: int
    maximum_radius_px: int
    preferred_radius_px: float
    minimum_edge_support: float
    circle_refinement_distance_px: float
    circle_refinement_minimum_points: int
    circle_refinement_minimum_arc_deg: float
    line_offset_mm: float
    line_band_half_height_px: int
    line_minimum_half_width_px: float
    line_station_span_width_factor: float
    line_hough_theta_step_deg: float
    line_hough_vote_threshold: int
    line_maximum_hough_candidates: int
    line_maximum_angle_deg: float
    line_maximum_target_distance_px: float
    line_refinement_distance_px: float
    line_minimum_edge_points: int
    line_minimum_x_span_px: float
    maximum_line_disagreement_deg: float
    search_calibration: dict | None = None


@dataclass(frozen=True)
class StackedRingDetection:
    """中环与长边检测、稳定状态和底盘纠偏计算结果。"""

    detected: bool = False
    stable: bool = False
    confidence: float = 0.0
    middle_center_px: tuple[float, float] | None = None
    middle_radius_px: float = 0.0
    centers_px: tuple[tuple[float, float], ...] = ()
    station_yaw_deg: float = 0.0
    top_line_angle_deg: float | None = None
    bottom_line_angle_deg: float | None = None
    line_segments_px: tuple[tuple[tuple[int, int], tuple[int, int]], ...] = ()
    circle_support: float = 0.0
    line_quality: float = 0.0
    angle_error_deg: float = 0.0
    error_x_mm: float = 0.0
    error_y_mm: float = 0.0
    position_error_mm: float = 0.0
    correction_body_x_mm: float | None = None
    correction_body_y_mm: float | None = None
    correction_yaw_deg: float | None = None
    within_tolerance: bool = False
    processing_ms: float = 0.0
    detail: str = "尚未识别"
    outer_ellipse: tuple | None = None
    search_rois: tuple = ()


def _require(condition: bool, detail: str) -> None:
    if not condition:
        raise StackedRingDetectionError(detail)


def load_stacked_ring_config(path: Path | str) -> StackedRingConfig:
    """读取并严格校验中环加长边参数。"""

    resolved = Path(path).expanduser().resolve()
    try:
        raw = json.loads(resolved.read_text(encoding="utf-8"))
        _require(int(raw["format_version"]) == 1, "不支持的码垛检测配置版本")
        circle = raw["middle_circle"]
        lines = raw["reference_lines"]
        config = StackedRingConfig(
            search_half_width_px=int(circle["search_half_width_px"]),
            search_half_height_px=int(circle["search_half_height_px"]),
            maximum_reference_distance_px=float(
                circle["maximum_reference_distance_px"]
            ),
            circle_hough_dp=float(circle["hough_dp"]),
            circle_hough_min_distance_px=float(
                circle["hough_min_distance_px"]
            ),
            circle_canny_high=float(circle["canny_high"]),
            circle_accumulator_threshold=float(
                circle["accumulator_threshold"]
            ),
            edge_canny_low=float(circle["edge_canny_low"]),
            edge_canny_high=float(circle["edge_canny_high"]),
            minimum_radius_px=int(circle["minimum_radius_px"]),
            maximum_radius_px=int(circle["maximum_radius_px"]),
            preferred_radius_px=float(circle["preferred_radius_px"]),
            minimum_edge_support=float(circle["minimum_edge_support"]),
            circle_refinement_distance_px=float(
                circle["refinement_distance_px"]
            ),
            circle_refinement_minimum_points=int(
                circle["refinement_minimum_points"]
            ),
            circle_refinement_minimum_arc_deg=float(
                circle["refinement_minimum_arc_deg"]
            ),
            line_offset_mm=float(lines["offset_mm"]),
            line_band_half_height_px=int(lines["band_half_height_px"]),
            line_minimum_half_width_px=float(lines["minimum_half_width_px"]),
            line_station_span_width_factor=float(
                lines["station_span_width_factor"]
            ),
            line_hough_theta_step_deg=float(lines["hough_theta_step_deg"]),
            line_hough_vote_threshold=int(lines["hough_vote_threshold"]),
            line_maximum_hough_candidates=int(
                lines["maximum_hough_candidates"]
            ),
            line_maximum_angle_deg=float(lines["maximum_angle_deg"]),
            line_maximum_target_distance_px=float(
                lines["maximum_target_distance_px"]
            ),
            line_refinement_distance_px=float(
                lines["refinement_distance_px"]
            ),
            line_minimum_edge_points=int(lines["minimum_edge_points"]),
            line_minimum_x_span_px=float(lines["minimum_x_span_px"]),
            search_calibration=raw.get("search_calibration"),
            maximum_line_disagreement_deg=float(
                lines["maximum_line_disagreement_deg"]
            ),
        )
    except StackedRingDetectionError:
        raise
    except (OSError, json.JSONDecodeError, KeyError, TypeError, ValueError) as error:
        raise StackedRingDetectionError(
            f"无法读取码垛检测配置 {resolved}：{error}"
        ) from error

    _require(config.search_half_width_px >= 90, "中环搜索宽度过小")
    _require(config.search_half_height_px >= 90, "中环搜索高度过小")
    _require(
        0 < config.minimum_radius_px < config.maximum_radius_px,
        "中环半径范围无效",
    )
    _require(config.circle_hough_dp >= 1.0, "中环 Hough dp 必须不小于1")
    _require(
        config.circle_hough_min_distance_px > 0
        and config.circle_canny_high > 0
        and config.circle_accumulator_threshold > 0,
        "中环 Hough 参数必须大于0",
    )
    _require(
        0 < config.edge_canny_low < config.edge_canny_high,
        "中环与长边的 Canny 阈值无效",
    )
    _require(
        0.0 < config.minimum_edge_support <= 1.0,
        "中环边缘支持率必须位于0到1之间",
    )
    _require(config.line_offset_mm > 0, "长边相对中环距离必须大于0")
    _require(
        0.0 < config.line_hough_theta_step_deg <= 5.0,
        "直线 Hough 角度步长无效",
    )
    _require(config.line_hough_vote_threshold > 0, "直线票数门槛必须大于0")
    _require(
        1 <= config.line_maximum_hough_candidates <= 100,
        "直线候选数量必须位于1~100",
    )
    _require(
        0.0 < config.maximum_line_disagreement_deg <= 10.0,
        "上下直线角度差门槛无效",
    )
    return config


def _normalize_angle(angle_deg: float) -> float:
    return (float(angle_deg) + 180.0) % 360.0 - 180.0


def _circle_edge_support(edge_image, center, radius) -> float:
    """向量化统计圆周 ±4 px 内存在 Canny 边缘的角度比例。"""

    angles = np.linspace(0.0, 2.0 * math.pi, 360, endpoint=False)
    offsets = np.arange(-4.0, 5.0)
    radii = float(radius) + offsets[None, :]
    sample_x = np.rint(
        float(center[0]) + np.cos(angles)[:, None] * radii
    ).astype(np.int32)
    sample_y = np.rint(
        float(center[1]) + np.sin(angles)[:, None] * radii
    ).astype(np.int32)
    valid = (
        (sample_x >= 0) & (sample_x < edge_image.shape[1])
        & (sample_y >= 0) & (sample_y < edge_image.shape[0])
    )
    hits = np.zeros_like(valid)
    hits[valid] = edge_image[sample_y[valid], sample_x[valid]] != 0
    return float(np.mean(np.any(hits, axis=1)))


def _refine_middle_circle(edge_image, center, radius, config):
    """用粗 Hough 圆附近的边缘拟合亚像素中环。"""

    edge_y, edge_x = np.nonzero(edge_image)
    point_x = edge_x.astype(np.float64)
    point_y = edge_y.astype(np.float64)
    center_x, center_y = map(float, center)
    fitted_radius = float(radius)
    residual = np.abs(
        np.hypot(point_x - center_x, point_y - center_y) - fitted_radius
    )
    keep = residual <= config.circle_refinement_distance_px
    if np.count_nonzero(keep) < config.circle_refinement_minimum_points:
        return center_x, center_y, fitted_radius

    last_valid = (center_x, center_y, fitted_radius, keep)
    for cutoff in (2.5, 1.8, 1.5):
        selected_x = point_x[keep]
        selected_y = point_y[keep]
        design = np.column_stack((
            2.0 * selected_x,
            2.0 * selected_y,
            np.ones(len(selected_x), dtype=np.float64),
        ))
        squared_distance = selected_x ** 2 + selected_y ** 2
        solution, *_ = np.linalg.lstsq(design, squared_distance, rcond=None)
        candidate_x, candidate_y, constant = map(float, solution)
        radius_squared = constant + candidate_x ** 2 + candidate_y ** 2
        if radius_squared <= 0.0:
            break
        candidate_radius = math.sqrt(radius_squared)
        if (
            math.hypot(candidate_x - float(center[0]),
                       candidate_y - float(center[1])) > 4.0
            or abs(candidate_radius - float(radius)) > 4.0
        ):
            break
        candidate_residual = np.abs(
            np.hypot(point_x - candidate_x, point_y - candidate_y)
            - candidate_radius
        )
        candidate_keep = candidate_residual <= cutoff
        if np.count_nonzero(candidate_keep) < config.circle_refinement_minimum_points:
            break
        last_valid = (
            candidate_x, candidate_y, candidate_radius, candidate_keep
        )
        keep = candidate_keep

    center_x, center_y, fitted_radius, keep = last_valid
    angles = np.arctan2(
        point_y[keep] - center_y, point_x[keep] - center_x
    )
    bins = np.floor(
        (angles + math.pi) / (2.0 * math.pi) * 72.0
    ).astype(np.int32)
    visible_arc_deg = len(np.unique(bins)) * 5.0
    if visible_arc_deg < config.circle_refinement_minimum_arc_deg:
        return float(center[0]), float(center[1]), float(radius)
    return center_x, center_y, fitted_radius


def _detect_middle_ring(
    gray,
    reference_center,
    config,
    *,
    search_half_width_px=None,
    search_half_height_px=None,
    maximum_reference_distance_px=None,
):
    """在给定种子附近搜索中环。

    可选参数供锁定后的局部搜索使用；配置中的范围仍是首次定位和
    局部跟踪失败后的全局恢复范围。
    """

    height, width = gray.shape
    reference_x, reference_y = map(float, reference_center)
    half_width = int(
        config.search_half_width_px
        if search_half_width_px is None else search_half_width_px
    )
    half_height = int(
        config.search_half_height_px
        if search_half_height_px is None else search_half_height_px
    )
    maximum_distance = float(
        config.maximum_reference_distance_px
        if maximum_reference_distance_px is None
        else maximum_reference_distance_px
    )
    x0 = max(0, int(round(reference_x)) - half_width)
    x1 = min(width, int(round(reference_x)) + half_width)
    y0 = max(0, int(round(reference_y)) - half_height)
    y1 = min(height, int(round(reference_y)) + half_height)
    if x1 - x0 < 180 or y1 - y0 < 180:
        raise StackedRingDetectionError("中环搜索区域超出画面")

    blurred = cv2.GaussianBlur(gray[y0:y1, x0:x1], (5, 5), 1.2)
    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=config.circle_hough_dp,
        minDist=config.circle_hough_min_distance_px,
        param1=config.circle_canny_high,
        param2=config.circle_accumulator_threshold,
        minRadius=config.minimum_radius_px,
        maxRadius=config.maximum_radius_px,
    )
    if circles is None or len(circles[0]) == 0:
        raise StackedRingDetectionError("未检出被物料占用的中间圆环外圈")

    # 当前实机的下基准边是白底上的浅灰细线。30/90 能保留该边，
    # 同时中环搜索区域和几何约束会排除桌面纹理造成的伪边缘。
    edges = cv2.Canny(
        gray, config.edge_canny_low, config.edge_canny_high
    )
    candidates = []
    rejected_by_distance = []
    rejected_by_support = []
    for local_x, local_y, radius in circles[0]:
        center = (float(local_x + x0), float(local_y + y0))
        distance = math.hypot(center[0] - reference_x, center[1] - reference_y)
        if distance > maximum_distance:
            rejected_by_distance.append(distance)
            continue
        support = _circle_edge_support(edges, center, float(radius))
        if support < config.minimum_edge_support:
            rejected_by_support.append(support)
            continue
        score = (
            abs(float(radius) - config.preferred_radius_px)
            / max(1.0, config.maximum_radius_px - config.minimum_radius_px)
            + distance / max(1.0, maximum_distance)
            - support * 0.35
        )
        candidates.append((score, center, float(radius)))
    if not candidates:
        if rejected_by_distance:
            nearest = min(rejected_by_distance)
            raise StackedRingDetectionError(
                f"中环距搜索基准{nearest:.1f}px，超过{maximum_distance:.1f}px"
            )
        if rejected_by_support:
            best_support = max(rejected_by_support)
            raise StackedRingDetectionError(
                f"中环圆弧支持率{best_support:.2f}，低于"
                f"{config.minimum_edge_support:.2f}"
            )
        raise StackedRingDetectionError("中环候选的圆弧连续性不足")
    _, center, radius = min(candidates, key=lambda item: item[0])
    # 圆拟合只需要圆周附近的边缘。旧实现对整张 1280x720 边缘图
    # 执行 nonzero 和径向距离计算，实机约多耗 6ms；裁到圆外接方框
    # 不改变拟合点集，却显著减少逐帧内存扫描量。
    refinement_margin = int(math.ceil(
        radius + config.circle_refinement_distance_px + 7.0
    ))
    crop_x0 = max(0, int(round(center[0])) - refinement_margin)
    crop_x1 = min(width, int(round(center[0])) + refinement_margin + 1)
    crop_y0 = max(0, int(round(center[1])) - refinement_margin)
    crop_y1 = min(height, int(round(center[1])) + refinement_margin + 1)
    local_center = (center[0] - crop_x0, center[1] - crop_y0)
    center_x, center_y, radius = _refine_middle_circle(
        edges[crop_y0:crop_y1, crop_x0:crop_x1],
        local_center,
        radius,
        config,
    )
    center = (center_x + crop_x0, center_y + crop_y0)
    support = _circle_edge_support(edges, center, radius)
    return center, radius, support, edges


def _refine_line_candidate(edge_points, raw_line, target_x, target_y, config):
    rho, theta = map(float, raw_line)
    normal_x, normal_y = math.cos(theta), math.sin(theta)
    if abs(normal_y) <= 1e-6:
        return None
    raw_angle = _normalize_angle(math.degrees(theta - math.pi / 2.0))
    if abs(raw_angle) > config.line_maximum_angle_deg:
        return None
    raw_y_at_target = (rho - target_x * normal_x) / normal_y
    if abs(raw_y_at_target - target_y) > config.line_maximum_target_distance_px:
        return None

    distances = np.abs(
        edge_points[:, 0] * normal_x
        + edge_points[:, 1] * normal_y - rho
    )
    nearby_mask = distances <= config.line_refinement_distance_px
    nearby = edge_points[nearby_mask]
    nearby_distances = distances[nearby_mask]
    if len(nearby):
        order = np.lexsort((nearby_distances, nearby[:, 0]))
        nearby = nearby[order]
        keep = np.ones(len(nearby), dtype=bool)
        keep[1:] = nearby[1:, 0] != nearby[:-1, 0]
        nearby = nearby[keep]
    if len(nearby) < config.line_minimum_edge_points:
        return None

    direction_x, direction_y, point_x, point_y = (
        float(value) for value in cv2.fitLine(
            nearby.reshape(-1, 1, 2), cv2.DIST_HUBER,
            0.0, 0.01, 0.01,
        ).reshape(-1)
    )
    if direction_x < 0.0:
        direction_x, direction_y = -direction_x, -direction_y
    if abs(direction_x) <= 1e-6:
        return None
    angle_deg = math.degrees(math.atan2(direction_y, direction_x))
    if abs(angle_deg) > config.line_maximum_angle_deg:
        return None
    y_at_target = point_y + direction_y / direction_x * (target_x - point_x)
    target_distance = abs(y_at_target - target_y)
    x_span = float(np.ptp(nearby[:, 0]))
    if (
        target_distance > config.line_maximum_target_distance_px
        or x_span < config.line_minimum_x_span_px
    ):
        return None
    return angle_deg, x_span, target_distance, len(nearby), y_at_target


def _line_candidates(
    edge_image, target_point, x_bounds, config, previous_line=None,
    *, minimum_roi_size=(20, 260),
):
    target_x, target_y = map(float, target_point)
    y0 = max(0, int(round(target_y)) - config.line_band_half_height_px)
    y1 = min(
        edge_image.shape[0],
        int(round(target_y)) + config.line_band_half_height_px + 1,
    )
    x0, x1 = x_bounds
    if y1 - y0 < minimum_roi_size[0] or x1 - x0 < minimum_roi_size[1]:
        return []
    edge_band = edge_image[y0:y1, x0:x1]
    local_target_x = target_x - x0
    local_target_y = target_y - y0
    edge_y, edge_x = np.nonzero(edge_band)
    edge_points = np.column_stack((edge_x, edge_y)).astype(np.float32)

    if previous_line is not None and len(edge_points):
        # 首帧用标准 Hough 获得可靠直线；后续帧把上一帧直线转换成
        # 一个 Hough 候选，只在其 ±refinement_distance 内做 fitLine。
        # 这条路径不需要重新遍历整个 theta/rho 累加器，且不会像按列
        # 平均全部边缘那样被圆环纹理拉偏。
        previous_angle, previous_y_at_target = map(float, previous_line)
        theta = math.radians(previous_angle + 90.0)
        normal_x, normal_y = math.cos(theta), math.sin(theta)
        previous_local_y = previous_y_at_target - y0
        rho = local_target_x * normal_x + previous_local_y * normal_y
        refined = _refine_line_candidate(
            edge_points,
            (rho, theta),
            local_target_x,
            local_target_y,
            config,
        )
        if refined is not None:
            angle, span, distance, support_columns, local_y_at_target = refined
            return [(
                support_columns - 20.0 * distance,
                angle,
                span,
                distance,
                float(local_y_at_target + y0),
            )]

    # 尚未锁定或上一帧直线已经失效时，支付一次标准 Hough 的开销。
    lines = cv2.HoughLines(
        edge_band,
        1.0,
        math.radians(config.line_hough_theta_step_deg),
        threshold=config.line_hough_vote_threshold,
        min_theta=math.radians(90.0 - config.line_maximum_angle_deg),
        max_theta=math.radians(90.0 + config.line_maximum_angle_deg),
    )
    if lines is None:
        return []
    result = []
    # 标准 Hough 会为同一条粗边返回大量相邻 rho/theta 候选。OpenCV
    # 已按票数降序排列，只精修前若干个即可覆盖真实长边，避免一帧对
    # 数百个近似重复候选反复调用 fitLine。
    raw_candidates = lines[:config.line_maximum_hough_candidates, 0]
    for rank, raw in enumerate(raw_candidates):
        refined = _refine_line_candidate(
            edge_points, raw, local_target_x, local_target_y, config
        )
        if refined is None:
            continue
        angle, span, distance, support_columns, local_y_at_target = refined
        score = support_columns - 20.0 * distance - 20.0 * rank
        result.append((
            score, angle, span, distance,
            float(local_y_at_target + y0),
        ))
    return result


def _line_segment(angle_deg, y_at_target, target_x, x0, x1):
    slope = math.tan(math.radians(angle_deg))
    y_left = y_at_target + slope * (x0 - target_x)
    y_right = y_at_target + slope * (x1 - target_x)
    return ((int(round(x0)), int(round(y_left))),
            (int(round(x1)), int(round(y_right))))


def _select_reference_lines(groups, config, require_both):
    """双线按原评分配对；只有一侧有效时选该侧最佳候选，不掩盖双线冲突。"""
    if not any(groups):
        raise StackedRingDetectionError("未找到有效基准线")
    if not all(groups):
        _require(not require_both, "标定需要上下两条有效基准线")
        return tuple(max(group, key=lambda item: item[0]) if group else None
                     for group in groups)
    pairs = []
    for top in groups[0]:
        for bottom in groups[1]:
            disagreement = abs(_normalize_angle(top[1] - bottom[1]))
            if disagreement <= config.maximum_line_disagreement_deg:
                score = top[0] + bottom[0] - 80.0 * disagreement
                pairs.append((score, top, bottom))
    _require(bool(pairs), "上下基准线方向不一致")
    _, top, bottom = max(pairs, key=lambda item: item[0])
    return top, bottom


def _detect_reference_lines(
    edges, middle_center, ring_config, config, previous_lines=None, *, require_both=True
):
    affine = np.asarray(ring_config.image_to_station_affine, dtype=np.float64)
    try:
        station_to_image = np.linalg.inv(affine)
    except np.linalg.LinAlgError as error:
        raise StackedRingDetectionError("图像毫米换算矩阵不可逆") from error

    center = np.asarray(middle_center, dtype=np.float64)
    top_offset = station_to_image @ np.asarray((0.0, -config.line_offset_mm))
    bottom_offset = station_to_image @ np.asarray((0.0, config.line_offset_mm))
    reference = np.asarray(ring_config.reference_centers_px, dtype=np.float64)
    station_span = float(reference[-1, 0] - reference[0, 0])
    half_width = int(round(max(
        config.line_minimum_half_width_px,
        station_span * config.line_station_span_width_factor,
    )))
    x0 = max(0, int(round(center[0])) - half_width)
    x1 = min(edges.shape[1], int(round(center[0])) + half_width)

    groups = []
    targets = (center + top_offset, center + bottom_offset)
    line_seeds = previous_lines or (None, None)
    for point, previous in zip(targets, line_seeds):
        candidates = []
        if 0 <= point[1] < edges.shape[0]:
            candidates = _line_candidates(edges, point, (x0, x1), config, previous)
        groups.append(candidates)

    top, bottom = _select_reference_lines(groups, config, require_both)
    if top is not None and bottom is not None:
        disagreement = abs(_normalize_angle(top[1] - bottom[1]))
        total_span = top[2] + bottom[2]
        station_yaw = _normalize_angle(
            (top[1] * top[2] + bottom[1] * bottom[2]) / total_span
        )
        line_quality = min(
            1.0,
            min(top[2], bottom[2]) / 300.0,
            max(0.0, 1.0 - disagreement / config.maximum_line_disagreement_deg),
        )
    else:
        single = top if top is not None else bottom
        station_yaw = single[1]
        line_quality = min(1.0, single[2] / (2 * config.line_minimum_x_span_px))
    segments = tuple(
        _line_segment(item[1], item[4], float(target[0]), x0, x1)
        for item, target in zip((top, bottom), targets) if item is not None
    )
    # 丢失侧不保留旧种子，恢复时重新搜索真实边缘。
    updated_seeds = tuple((item[1], item[4]) if item is not None else None
                          for item in (top, bottom))
    return (
        station_yaw,
        top[1] if top is not None else None,
        bottom[1] if bottom is not None else None,
        line_quality,
        segments,
        updated_seeds,
    )


def _synthesise_centers(ring_config, middle_center, station_yaw):
    reference = np.asarray(ring_config.reference_centers_px, dtype=np.float64)
    reference_middle = reference[1]
    angle_error = math.radians(_normalize_angle(
        station_yaw - ring_config.reference_yaw_deg
    ))
    cosine, sine = math.cos(angle_error), math.sin(angle_error)
    rotation = np.asarray(((cosine, -sine), (sine, cosine)))
    middle = np.asarray(middle_center, dtype=np.float64)
    return tuple(
        tuple((middle + rotation @ (point - reference_middle)).tolist())
        for point in reference
    )


# ---------- 外圈尺度标定：只决定搜索，不修改工位零点或纠偏矩阵 ----------
def camera_geometry_signature(width, height, undistort_enabled, calibration_path):
    """关闭矫正时不读取内参；开启时对实际参与remap的参数作指纹。"""
    fingerprint = None
    if undistort_enabled:
        storage = cv2.FileStorage(str(Path(calibration_path).expanduser().resolve()), cv2.FILE_STORAGE_READ)
        try:
            _require(storage.isOpened(), "无法读取相机参数")
            values = [storage.getNode(name).real() for name in ("image_width", "image_height")]
            for name in ("camera_matrix", "distortion_coefficients", "new_camera_matrix"):
                matrix = storage.getNode(name).mat()
                _require(matrix is not None, f"相机参数缺少{name}")
                values.append(matrix.tolist())
            fingerprint = hashlib.sha256(json.dumps(values).encode()).hexdigest()
        finally:
            storage.release()
    return {"image_size": [int(width), int(height)],
            "undistort_enabled": bool(undistort_enabled),
            "camera_fingerprint": fingerprint}


def validate_search_calibration(data):
    """可选标定段独立于旧参数；损坏时由码垛模式提示，不影响其他模式。"""
    try:
        _require(isinstance(data, dict) and data["version"] == 1, "搜索标定版本无效")
        size = np.asarray(data["image_size"], dtype=float)
        center = np.asarray(data["center_px"], dtype=float)
        axes = np.asarray(data["outer_axes_px"], dtype=float)
        _require(size.shape == center.shape == axes.shape == (2,), "搜索标定尺寸无效")
        values = np.r_[size, center, axes, data["outer_angle_deg"],
                       data["outer_diameter_mm"], data["line_offset_mm"],
                       data["line_yaw_deg"], data["search_half_width_diameters"],
                       data["band_half_height_diameters"]]
        _require(np.isfinite(values).all(), "搜索标定含非有限数值")
        _require(np.all(size > 0) and np.all(size == np.floor(size)), "图像尺寸无效")
        _require(np.all(center >= 0) and np.all(center < size), "标定圆心越界")
        _require(np.all(axes > 0) and min(axes) / max(axes) >= 0.75,
                 "外圈变形过大，请调整视角")
        _require(data["outer_diameter_mm"] > 0 and
                 data["line_offset_mm"] > data["outer_diameter_mm"] / 2,
                 "物理尺寸无效")
        _require(1 <= data["search_half_width_diameters"] <= 5 and
                 0.02 <= data["band_half_height_diameters"] <= 0.5, "搜索尺度无效")
        _require(type(data["undistort_enabled"]) is bool, "矫正设置无效")
        fingerprint = data["camera_fingerprint"]
        _require((not data["undistort_enabled"] and fingerprint is None) or
                 (data["undistort_enabled"] and isinstance(fingerprint, str)
                  and len(fingerprint) == 64), "相机指纹无效")
    except (KeyError, TypeError, ValueError) as error:
        raise StackedRingDetectionError(f"Recalibrate stacking: invalid search calibration ({error})") from error
    return data


def ellipse_shape_matrix(ellipse):
    """椭圆半轴平方矩阵Q；沿单位法向n的投影半径为sqrt(n.T Q n)。"""
    _, axes, angle = ellipse
    theta = math.radians(angle)
    rotation = np.array(((math.cos(theta), -math.sin(theta)),
                         (math.sin(theta), math.cos(theta))))
    return rotation @ np.diag((np.asarray(axes) / 2.0) ** 2) @ rotation.T


def _outer_ring_from_edges(edges, candidate):
    """从同心结构向外找圆周；分散的虚线段也必须覆盖多数角度分区。"""
    ellipse = (tuple(candidate.center), candidate.ellipse.axes, candidate.ellipse.angle_deg)
    center, axes, angle = ellipse
    theta = math.radians(angle)
    rotation = np.array(((math.cos(theta), -math.sin(theta)),
                         (math.sin(theta), math.cos(theta))))
    angles = np.linspace(0, 2 * math.pi, 360, endpoint=False)
    vectors = np.column_stack((np.cos(angles), np.sin(angles)))
    vectors = (vectors * (np.asarray(axes) / 2)) @ rotation.T
    # 起始轮廓可能是内圈；向外扫描可找回被findContours拆开的虚线最外圈。
    scales = np.arange(0.96, 1.56, 1.0 / max(axes))
    points = np.asarray(center) + scales[:, None, None] * vectors[None, :, :]
    pixels = np.rint(points).astype(int)
    valid = ((pixels[:, :, 0] >= 0) & (pixels[:, :, 0] < edges.shape[1]) &
             (pixels[:, :, 1] >= 0) & (pixels[:, :, 1] < edges.shape[0]))
    hits = np.zeros(valid.shape, dtype=bool)
    hits[valid] = edges[pixels[:, :, 1][valid], pixels[:, :, 0][valid]] != 0
    coverage = hits.mean(axis=1)
    sectors = hits.reshape(len(scales), 12, 30).any(axis=2).mean(axis=1)
    reliable = np.flatnonzero((coverage >= 0.35) & (sectors >= 0.9) & valid.all(axis=1))
    _require(len(reliable) > 0, "最外圈支持不足，请露出完整外圈")
    # 从最外侧连续峰中取支持最高的位置，避免把线宽的外缘噪点当新圆。
    groups = np.split(reliable, np.where(np.diff(reliable) > 3)[0] + 1)
    outer_group = groups[-1]
    index = outer_group[np.argmax(coverage[outer_group])]
    nearby = np.abs(scales - scales[index]) * max(axes) / 2 <= 1.5
    selected = points[nearby][hits[nearby]].astype(np.float32)
    _require(len(selected) >= 30, "最外圈点数不足")
    fitted = cv2.fitEllipse(selected.reshape(-1, 1, 2))
    fitted_center, fitted_axes, _ = fitted
    _require(np.linalg.norm(np.asarray(fitted_center) - center) <= max(2.0, min(axes) * 0.02),
             "外圈与同心轮廓不一致")
    _require(min(fitted_axes) / max(fitted_axes) >= 0.75, "透视变形过大，请调整视角")
    # 使用椭圆方程检查亚像素拟合残差，不能仅按轮廓大小决定外圈。
    inverse = np.linalg.inv(ellipse_shape_matrix(fitted))
    delta = selected - fitted_center
    radial = np.sqrt(np.einsum('ni,ij,nj->n', delta, inverse, delta))
    _require(float(np.median(np.abs(radial - 1))) <= 0.035, "外圈拟合残差过大")
    return fitted, float(coverage[index])


def stacked_edge_image(frame, config):
    """外圈复核和直线搜索共用一次全图Canny，轮廓粗搜仍沿用原模块。"""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return cv2.Canny(gray, config.edge_canny_low, config.edge_canny_high)


def _partial_side_ring_support(edges, center, middle_ellipse):
    """侧环只辅助确认排列：按中环尺度检查残留外圆弧，不把物料边缘当圆心。"""
    _, axes, angle = middle_ellipse
    theta = math.radians(angle)
    rotation = np.array(((math.cos(theta), -math.sin(theta)),
                         (math.sin(theta), math.cos(theta))))
    angles = np.linspace(0, 2 * math.pi, 360, endpoint=False)
    vectors = np.column_stack((np.cos(angles), np.sin(angles)))
    vectors = (vectors * (np.asarray(axes) / 2)) @ rotation.T
    points = np.rint(np.asarray(center) +
                     np.linspace(0.94, 1.06, 13)[:, None, None] * vectors).astype(int)
    valid = ((points[:, :, 0] >= 0) & (points[:, :, 0] < edges.shape[1]) &
             (points[:, :, 1] >= 0) & (points[:, :, 1] < edges.shape[0]))
    hits = np.zeros(valid.shape, dtype=bool)
    hits[valid] = edges[points[:, :, 1][valid], points[:, :, 0][valid]] != 0
    angular_hits = hits.any(axis=0)
    return (float(angular_hits.mean()) >= 0.35 and
            float(angular_hits.reshape(12, 30).any(axis=1).mean()) >= 0.5)


def _select_middle_with_partial_sides(observations, reliable, config, width, edges):
    """只有侧环可降低同心轮廓数量；中环必须来自原来的可靠候选集合。"""
    auxiliary_config = replace(config, min_cluster_support=2, min_diameter_bands=2)
    auxiliary = _cluster_observations(observations, auxiliary_config)
    triple = _select_triple(auxiliary, width, auxiliary_config)
    _require(triple is not None, "遮挡重定位缺少可靠三环排列")
    ordered = triple[1]
    middle = next((candidate for candidate in reliable
                   if np.linalg.norm(candidate.center - ordered[1].center)
                   <= config.center_cluster_radius_px), None)
    _require(middle is not None, "中环同心轮廓不足，不能仅用侧环推算中环")
    spacing = [float(np.linalg.norm(ordered[i + 1].center - ordered[i].center))
               for i in (0, 1)]
    _require(min(spacing) / max(spacing) >= 0.85, "遮挡重定位的三环间距不一致")
    # 中环仍使用完整的外圈质量检查；侧环仅帮助确认这是R2。
    ellipse, _ = _outer_ring_from_edges(edges, middle)
    direction = ordered[2].center - ordered[0].center
    direction = direction / np.linalg.norm(direction)
    projected_diameter = 2 * math.sqrt(float(direction @ ellipse_shape_matrix(ellipse) @ direction))
    expected_spacing = projected_diameter * config.center_spacing_mm / config.outer_diameter_mm
    _require(all(abs(value / expected_spacing - 1) <= 0.2 for value in spacing),
             "遮挡重定位的环间距与外圈尺度不符")
    _require(all(_partial_side_ring_support(edges, item.center, ellipse)
                 for item in (ordered[0], ordered[2])), "侧环残留外圆弧不足，无法确认中环位置")
    return middle


def find_calibration_outer_ring(frame, ring_config, selection=None, expected_axes=None,
                                edge_image=None, *, allow_partial_sides=False):
    """复用同心轮廓筛选；无唯一中环时要求点击，不能按画面中心猜。"""
    height, width = frame.shape[:2]
    x0 = y0 = 0
    view = frame
    if selection is not None and expected_axes is not None:
        half = max(expected_axes) * 1.25
        x0, y0 = max(0, int(selection[0] - half)), max(0, int(selection[1] - half))
        x1, y1 = min(width, int(selection[0] + half)), min(height, int(selection[1] + half))
        view = frame[y0:y1, x0:x1]
    _require(view.size > 0, "圆环搜索区域越界")
    # 首次全图搜不同尺寸，跟踪时按标定外圈尺度限制；不受旧60~82px限制。
    minimum = 20.0 if expected_axes is None else min(expected_axes) * 0.25
    maximum = min(width, height) * 0.8 if expected_axes is None else max(expected_axes) * 1.35
    local_config = replace(ring_config, min_minor_axis_px=minimum,
                           max_major_axis_px=maximum, min_contour_points=20,
                           center_cluster_radius_px=max(4.0, minimum * 0.15),
                           diameter_band_gap_px=max(2.0, minimum * 0.04))
    observations = _extract_observations(view, local_config)
    candidates = _cluster_observations(observations, local_config)
    _require(bool(candidates), "未找到可靠同心靶环，请露出外圈")
    for candidate in candidates:
        candidate.center[:] += (x0, y0)
    if edge_image is None:
        edge_image = cv2.Canny(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), 20, 60)
    if selection is None:
        if allow_partial_sides and len(candidates) != 3:
            candidate = _select_middle_with_partial_sides(
                observations, candidates, local_config, width, edge_image)
        else:
            _require(len(candidates) == 3, "请点击选择中环 / Click middle ring")
            triple = _select_triple(candidates, width, local_config)
            _require(triple is not None, "三环几何关系不可靠，请点击中环")
            ordered = triple[1]
            spacing = [np.linalg.norm(ordered[i + 1].center - ordered[i].center) for i in (0, 1)]
            _require(min(spacing) / max(spacing) >= 0.85, "明显透视，请调整视角")
            candidate = ordered[1]
    else:
        candidate = min(candidates, key=lambda item: np.linalg.norm(item.center - selection))
        distance = np.linalg.norm(candidate.center - selection)
        _require(distance <= max(candidate.ellipse.axes) * 0.6, "选点附近未找到靶环")
    ellipse, support = _outer_ring_from_edges(edge_image, candidate)
    if expected_axes is not None:
        ratios = np.sort(ellipse[1]) / np.sort(expected_axes)
        _require(np.all((ratios >= 0.8) & (ratios <= 1.2)), "外圈尺度变化过大，请重新标定")
    return ellipse, support


def detect_scaled_reference_lines(frame, ellipse, config, diameter_mm=95.0,
                                  offset_mm=75.0, yaw_hint=0.0,
                                  half_width_diameters=2.7, band_diameters=0.14, debug=None, edge_image=None,
                                  *, require_both=True):
    """局部仿射下：线到圆心距离/椭圆法向半径 = 75/47.5。"""
    center = np.asarray(ellipse[0])
    shape = ellipse_shape_matrix(ellipse)
    diameter = max(ellipse[1])
    angle = math.radians(yaw_hint)
    normal = np.array((-math.sin(angle), math.cos(angle)))
    normal_radius = math.sqrt(float(normal @ shape @ normal))
    offset = 2 * normal_radius * offset_mm / diameter_mm
    band = max(5, int(round(diameter * band_diameters)))
    half_width = int(round(diameter * half_width_diameters))
    x0 = max(0, int(center[0]) - half_width)
    x1 = min(frame.shape[1], int(center[0]) + half_width)
    scaled = replace(config, line_band_half_height_px=band,
                     line_maximum_target_distance_px=band,
                     line_refinement_distance_px=max(1.0, diameter * 0.015),
                     line_minimum_edge_points=max(12, int(diameter * 0.3)),
                     line_minimum_x_span_px=diameter * 1.4,
                     line_hough_vote_threshold=max(12, int(diameter * 0.4)))
    edges = stacked_edge_image(frame, config) if edge_image is None else edge_image
    groups, rois = [], []
    if debug is not None:
        debug.update(ellipse=ellipse, rois=rois, segments=(), candidate_segments=[])
    for sign in (-1, 1):
        # 用同一x处的纵向截距预测水平搜索带，保留原有小角度限制。
        target = (center[0], center[1] + sign * offset / normal[1])
        y0 = max(0, int(round(target[1])) - band)
        y1 = min(frame.shape[0], int(round(target[1])) + band + 1)
        # 一侧出界只表示该侧缺失，另一侧仍继续搜索。
        if not 0 <= target[1] < frame.shape[0]:
            groups.append([])
            continue
        rois.append((x0, y0, x1, y1))
        candidates = _line_candidates(edges, target, (x0, x1), scaled,
                                      minimum_roi_size=(5, max(20, int(diameter * 1.4))))
        if debug is not None:
            debug["candidate_segments"].extend(
                _line_segment(item[1], item[4], center[0], x0, x1) for item in candidates)
        valid = []
        for item in candidates:
            theta = math.radians(item[1])
            n = np.array((-math.sin(theta), math.cos(theta)))
            actual = (item[4] - center[1]) * n[1]
            expected = 2 * math.sqrt(float(n @ shape @ n)) * offset_mm / diameter_mm
            # 物理距离复核比搜索带窄，避免内圈、桌面边缘被稳定误锁。
            tolerance = max(2.0, expected * 0.05)
            if actual * sign > 0 and abs(abs(actual) - expected) <= tolerance:
                valid.append(item)
        groups.append(valid)
    top, bottom = _select_reference_lines(groups, config, require_both)
    if top is not None and bottom is not None:
        yaw = (top[1] * top[2] + bottom[1] * bottom[2]) / (top[2] + bottom[2])
        quality = max(0.0, 1 - abs(top[1] - bottom[1]) / config.maximum_line_disagreement_deg)
    else:
        single = top if top is not None else bottom
        yaw = single[1]
        quality = min(1.0, single[2] / (2 * scaled.line_minimum_x_span_px))
    segments = tuple(_line_segment(item[1], item[4], center[0], x0, x1)
                     for item in (top, bottom) if item is not None)
    return {"ellipse": ellipse, "middle": tuple(center), "radius": min(ellipse[1]) / 2,
            "yaw": yaw, "top_angle": top[1] if top is not None else None,
            "bottom_angle": bottom[1] if bottom is not None else None,
            "segments": segments, "rois": tuple(rois), "line_quality": quality}


def has_both_reference_lines(observation):
    return (observation is not None and observation.get("top_angle") is not None
            and observation.get("bottom_angle") is not None
            and len(observation.get("segments", ())) == 2)


def make_search_calibration(observation, signature, diameter_mm=95.0, offset_mm=75.0):
    _require(has_both_reference_lines(observation), "标定保存需要上下两条有效基准线")
    ellipse = observation["ellipse"]
    return validate_search_calibration({
        "version": 1, **signature, "center_px": list(map(float, ellipse[0])),
        "outer_axes_px": list(map(float, ellipse[1])), "outer_angle_deg": float(ellipse[2]),
        "outer_diameter_mm": float(diameter_mm), "line_offset_mm": float(offset_mm),
        "line_yaw_deg": float(observation["yaw"]),
        "search_half_width_diameters": 2.7, "band_half_height_diameters": 0.14,
    })


def save_search_calibration(path, data):
    """只替换可选标定段，临时文件校验成功后再原子替换原文件。"""
    validate_search_calibration(data)
    path = Path(path).expanduser().resolve()
    document = json.loads(path.read_text(encoding="utf-8"))
    document["search_calibration"] = data
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                         prefix=".stacked-", suffix=".json", delete=False) as stream:
            temporary = Path(stream.name)
            json.dump(document, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        load_stacked_ring_config(temporary)
        temporary.chmod(path.stat().st_mode & 0o777)
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def calibration_samples_stable(samples, ring_config):
    if len(samples) < ring_config.stable_frame_count:
        return False
    centers = np.asarray([item["middle"] for item in samples])
    axes = np.asarray([item["ellipse"][1] for item in samples])
    yaw = np.asarray([item["yaw"] for item in samples])
    return bool(np.max(np.linalg.norm(centers - np.median(centers, axis=0), axis=1))
                <= ring_config.maximum_center_jitter_px
                and np.max(np.abs(axes - np.median(axes, axis=0))) <= 2.0
                and np.ptp(yaw) <= 0.5)


def draw_search_calibration(frame, observation):
    for x0, y0, x1, y1 in observation["rois"]:
        cv2.rectangle(frame, (x0, y0), (x1 - 1, y1 - 1), (255, 180, 0), 1)
    cv2.ellipse(frame, observation["ellipse"], (0, 255, 0), 2, cv2.LINE_AA)
    for start, end in observation.get("candidate_segments", ()):
        cv2.line(frame, start, end, (0, 180, 255), 1, cv2.LINE_AA)
    for start, end in observation.get("segments", ()):
        cv2.line(frame, start, end, (255, 0, 255), 2, cv2.LINE_AA)
    return frame


class StackedRingDetector:
    """中环与至少一条长边逐帧检测，并输出时间中值稳定结果。"""

    def __init__(self, config: StackedRingConfig, ring_config: RingStationConfig):
        self.config = config
        self.ring_config = ring_config
        self._samples = deque(maxlen=ring_config.stable_frame_count)
        self._middle_seed = None
        self._line_seeds = None
        self._camera_signature = None
        self.search_observation = None
        self._next_ring_relocation = 0.0
        self._ring_relocation_error = None

    def reset(self) -> None:
        self._samples.clear()
        self._middle_seed = None
        self._line_seeds = None
        self.search_observation = None
        self._next_ring_relocation = 0.0
        self._ring_relocation_error = None

    def set_camera_geometry(self, signature):
        self._camera_signature = signature
        self.reset()

    def _find_calibrated_ring(self, frame, data, edges):
        """先局部跟踪，再查安装参考位置；失锁后用三环关系重新确定R2。"""
        centers = [data["center_px"]]
        if self._middle_seed is not None:
            centers.insert(0, self._middle_seed)
        for center in centers:
            try:
                return find_calibration_outer_ring(
                    frame, self.ring_config, center, data["outer_axes_px"], edges)
            except StackedRingDetectionError as error:
                local_error = error

        # 全图仅以可靠三环排列确定中环，不能把最近的侧环当作R2。
        # 失败时限频，避免没有目标时每一帧都支付全图轮廓搜索开销。
        now = time.monotonic()
        if now < self._next_ring_relocation:
            raise StackedRingDetectionError(self._ring_relocation_error or str(local_error))
        self._next_ring_relocation = now + 0.5
        try:
            return find_calibration_outer_ring(
                frame, self.ring_config, None, data["outer_axes_px"], edges,
                allow_partial_sides=True)
        except StackedRingDetectionError as error:
            self._ring_relocation_error = (
                f"中环全图重定位失败：{error}；局部搜索：{local_error}"
            )
            raise StackedRingDetectionError(self._ring_relocation_error) from error

    def _calibrated_detection(self, frame):
        data = validate_search_calibration(self.config.search_calibration)
        _require(self._camera_signature is not None, "Recalibrate stacking: camera geometry not configured")
        for key in ("image_size", "undistort_enabled", "camera_fingerprint"):
            _require(data[key] == self._camera_signature[key], "Recalibrate stacking: camera settings changed")
        _require(list(frame.shape[1::-1]) == data["image_size"], "Recalibrate stacking: image size changed")
        edges = stacked_edge_image(frame, self.config)
        ellipse, support = self._find_calibrated_ring(frame, data, edges)
        raw = detect_scaled_reference_lines(
            frame, ellipse, self.config, data["outer_diameter_mm"], data["line_offset_mm"],
            data["line_yaw_deg"], data["search_half_width_diameters"],
            data["band_half_height_diameters"], edge_image=edges, require_both=False)
        raw["circle_support"] = support
        raw["confidence"] = 0.92 + 0.08 * min(support, raw["line_quality"])
        raw["centers"] = _synthesise_centers(self.ring_config, raw["middle"], raw["yaw"])
        self._middle_seed = raw["middle"]
        self._ring_relocation_error = None
        self.search_observation = raw
        return raw

    def _raw_detection(self, frame):
        if self.config.search_calibration is not None:
            return self._calibrated_detection(frame)
        # 1. 找中环：有种子先局部搜索，失败时回到标定参考位置搜索。
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self._middle_seed is not None:
            try:
                # 圆半径不到 85px，锁定后使用 105px 半宽足以覆盖圆周。
                # 每帧只在小 ROI 内运行 Hough，可显著降低大范围重定位
                # 为适应底盘偏移而放宽后带来的计算开销。
                middle, radius, circle_support, edges = _detect_middle_ring(
                    gray,
                    self._middle_seed,
                    self.config,
                    search_half_width_px=105,
                    search_half_height_px=105,
                    maximum_reference_distance_px=35.0,
                )
            except StackedRingDetectionError:
                self._middle_seed = None
                middle, radius, circle_support, edges = _detect_middle_ring(
                    gray,
                    self.ring_config.reference_centers_px[1],
                    self.config,
                )
        else:
            middle, radius, circle_support, edges = _detect_middle_ring(
                gray, self.ring_config.reference_centers_px[1], self.config
            )
        self._middle_seed = middle
        # 2. 双线加权、单线直接给出yaw；按标定间距重建两侧圆心。
        (
            yaw,
            top_angle,
            bottom_angle,
            line_quality,
            segments,
            self._line_seeds,
        ) = (
            _detect_reference_lines(
                edges,
                middle,
                self.ring_config,
                self.config,
                self._line_seeds,
                require_both=False,
            )
        )
        centers = _synthesise_centers(self.ring_config, middle, yaw)
        return {
            "middle": middle,
            "radius": radius,
            "circle_support": circle_support,
            "yaw": yaw,
            "top_angle": top_angle,
            "bottom_angle": bottom_angle,
            "line_quality": line_quality,
            "segments": segments,
            "centers": centers,
            "confidence": 0.92 + 0.08 * min(circle_support, line_quality),
        }

    def detect(self, frame) -> StackedRingDetection:
        started = time.perf_counter()
        if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
            raise StackedRingDetectionError("码垛视觉输入必须是 BGR 图像")
        try:
            raw = self._raw_detection(frame)
        except (StackedRingDetectionError, cv2.error, np.linalg.LinAlgError) as error:
            self._samples.clear()
            self._line_seeds = None
            self.search_observation = None
            if self.config.search_calibration is not None:
                self._middle_seed = None
            return StackedRingDetection(
                processing_ms=(time.perf_counter() - started) * 1000.0,
                detail=str(error),
            )

        self._samples.append(raw)
        sample_count = len(self._samples)
        # 连续帧中，每个圆心的X/Y分别取中值；坐标单位为原图像素。
        centers = []
        for ring_index in range(3):
            xs = [sample["centers"][ring_index][0] for sample in self._samples]
            ys = [sample["centers"][ring_index][1] for sample in self._samples]
            centers.append((float(np.median(xs)), float(np.median(ys))))
        centers = tuple(centers)
        middle = centers[1]

        maximum_jitter = 0.0
        for sample in self._samples:
            for ring_index in range(3):
                dx = sample["centers"][ring_index][0] - centers[ring_index][0]
                dy = sample["centers"][ring_index][1] - centers[ring_index][1]
                maximum_jitter = max(maximum_jitter, math.hypot(dx, dy))
        stable = bool(
            sample_count == self._samples.maxlen
            and maximum_jitter <= self.ring_config.maximum_center_jitter_px
        )
        if self.config.search_calibration is not None:
            stable = stable and calibration_samples_stable(self._samples, self.ring_config)
        yaw_errors = [
            _normalize_angle(sample["yaw"] - self.ring_config.reference_yaw_deg)
            for sample in self._samples
        ]
        angle_error = float(np.median(yaw_errors))
        station_yaw = _normalize_angle(
            self.ring_config.reference_yaw_deg + angle_error
        )
        affine = np.asarray(self.ring_config.image_to_station_affine)
        # 图像像素偏差乘2x2标定矩阵，得到工位平面上的毫米偏差。
        metric_errors = []
        for center, reference in zip(centers, self.ring_config.reference_centers_px):
            pixel_error = np.asarray(center) - np.asarray(reference)
            metric_errors.append(affine @ pixel_error)
        metric_errors = tuple(metric_errors)
        error_x, error_y = (float(value) for value in metric_errors[1])
        position_error = max(
            float(np.linalg.norm(error)) for error in metric_errors
        )
        within = bool(
            stable
            and position_error <= self.ring_config.maximum_position_error_mm
            and abs(angle_error) <= self.ring_config.maximum_angle_error_deg
        )
        # 3x3底盘标定矩阵把(x毫米, y毫米, 角度)转换为底盘纠偏量。
        correction = None
        if self.ring_config.chassis_image_to_motion is not None:
            correction = np.asarray(
                self.ring_config.chassis_image_to_motion
            ) @ np.asarray((error_x, error_y, angle_error))
        if not stable:
            detail = f"等待稳定观测（{sample_count}/{self._samples.maxlen}）"
        elif within:
            detail = "中环与基准线已达到验收精度"
        else:
            detail = "中环与基准线偏差超出验收精度"
        return StackedRingDetection(
            detected=True,
            stable=stable,
            outer_ellipse=raw.get("ellipse"),
            search_rois=raw.get("rois", ()),
            confidence=float(np.median([
                sample["confidence"] for sample in self._samples
            ])),
            middle_center_px=middle,
            middle_radius_px=float(np.median([
                sample["radius"] for sample in self._samples
            ])),
            centers_px=centers,
            station_yaw_deg=station_yaw,
            top_line_angle_deg=_present_angle_median(self._samples, raw, "top_angle"),
            bottom_line_angle_deg=_present_angle_median(self._samples, raw, "bottom_angle"),
            line_segments_px=raw["segments"],
            circle_support=float(np.median([
                sample["circle_support"] for sample in self._samples
            ])),
            line_quality=float(np.median([
                sample["line_quality"] for sample in self._samples
            ])),
            angle_error_deg=angle_error,
            error_x_mm=error_x,
            error_y_mm=error_y,
            position_error_mm=position_error,
            correction_body_x_mm=(
                None if correction is None else float(correction[0])
            ),
            correction_body_y_mm=(
                None if correction is None else float(correction[1])
            ),
            correction_yaw_deg=(
                None if correction is None else float(correction[2])
            ),
            within_tolerance=within,
            processing_ms=(time.perf_counter() - started) * 1000.0,
            detail=detail,
        )


def _present_angle_median(samples, current, key):
    # 当前缺失就输出None，不能把上一帧的线显示为仍然存在。
    if current[key] is None:
        return None
    return float(np.median([sample[key] for sample in samples if sample[key] is not None]))


def format_line_angle(angle):
    return "--" if angle is None else f"{angle:+.2f}"


def format_line_status(result):
    if result.top_line_angle_deg is not None and result.bottom_line_angle_deg is not None:
        return "Lines=2"
    if result.top_line_angle_deg is not None:
        return "Lines=1 (TOP)"
    if result.bottom_line_angle_deg is not None:
        return "Lines=1 (BOTTOM)"
    return "Lines=0"


def stacked_failure_text(detail):
    """Hershey字体不支持中文；终端保留原始detail，窗口用英文概括。"""
    if detail.isascii():
        return detail[:80]
    if "Recalibrate" in detail or "重新标定" in detail:
        return "Recalibrate stacking; see terminal for details"
    if "方向不一致" in detail:
        return "Reference line angles disagree"
    if "需要上下两条" in detail:
        return "Calibration requires both reference lines"
    if "全图重定位失败" in detail:
        return "Ring relocation failed; check middle ring and side-ring evidence"
    if "选点附近" in detail:
        return "Ring outside local search; waiting for ring relocation"
    if "基准线" in detail or "边线" in detail:
        return "No valid reference line; check ROI and geometry"
    if "环" in detail or "外圈" in detail or "透视" in detail:
        return "Ring detection failed; check position, scale and visibility"
    return "Detection failed; see terminal for details"


def draw_stacked_ring_detection(frame, result: StackedRingDetection):
    """绘制中环、上下长边、重建三圆心和当前误差。"""

    if not result.detected:
        cv2.putText(
            frame, "Stacked ring: SEARCHING", (20, 95),
            cv2.FONT_HERSHEY_SIMPLEX, 0.72, (0, 0, 255), 2,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame, stacked_failure_text(result.detail), (20, 125),
            cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 0, 255), 1,
            cv2.LINE_AA,
        )
        return frame

    for x0, y0, x1, y1 in result.search_rois:
        cv2.rectangle(frame, (x0, y0), (x1 - 1, y1 - 1), (255, 180, 0), 1)
    for segment in result.line_segments_px:
        cv2.line(frame, segment[0], segment[1], (255, 0, 255), 3, cv2.LINE_AA)
    middle = tuple(int(round(value)) for value in result.middle_center_px)
    if result.outer_ellipse is not None:
        cv2.ellipse(frame, result.outer_ellipse, (0, 255, 0), 3, cv2.LINE_AA)
    else:
        cv2.circle(
            frame, middle, int(round(result.middle_radius_px)),
            (0, 255, 0), 3, cv2.LINE_AA,
        )
    colors = ((0, 180, 255), (0, 255, 0), (255, 160, 0))
    points = []
    for index, (center, color) in enumerate(zip(result.centers_px, colors), 1):
        point = tuple(int(round(value)) for value in center)
        points.append(point)
        cv2.drawMarker(frame, point, color, cv2.MARKER_CROSS, 24, 2)
        cv2.putText(
            frame, f"R{index}", (point[0] + 10, point[1] - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2, cv2.LINE_AA,
        )
    cv2.polylines(
        frame, [np.asarray(points, np.int32)], False, (255, 0, 255), 2
    )
    cv2.rectangle(frame, (10, 65), (920, 145), (0, 0, 0), -1)
    cv2.putText(
        frame,
        f"Stacked: {'STABLE' if result.stable else 'WAITING'}  "
        f"dX={result.error_x_mm:+.2f}mm dY={result.error_y_mm:+.2f}mm "
        f"dYaw={result.angle_error_deg:+.2f}deg",
        (20, 98), cv2.FONT_HERSHEY_SIMPLEX, 0.66,
        (80, 255, 80) if result.within_tolerance else (255, 255, 255),
        2, cv2.LINE_AA,
    )
    cv2.putText(
        frame,
        f"Circle={result.circle_support:.2f} Line={result.line_quality:.2f} "
        f"{format_line_status(result)} "
        f"Top={format_line_angle(result.top_line_angle_deg)} "
        f"Bottom={format_line_angle(result.bottom_line_angle_deg)} "
        f"{result.processing_ms:.1f}ms",
        (20, 132), cv2.FONT_HERSHEY_SIMPLEX, 0.56,
        (255, 255, 255), 2, cv2.LINE_AA,
    )
    return frame
