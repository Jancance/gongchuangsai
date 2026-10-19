#!/usr/bin/env python3
"""利用三件圆柱物料快速估计 300 mm 转盘的粗圆心。

这是旧 ROS 工程 ``material_turntable_fast.py`` 的无 ROS 移植版。
三件物料只负责为局部外圆拟合提供搜索种子。三个底端的轨迹圆心不会
直接作为最终坐标；最终结果必须来自通过质量检查的300 mm真实外圆。
"""

from __future__ import annotations

from dataclasses import dataclass
import itertools
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np

from 颜色标定 import make_all_color_masks


class ThreeMaterialSeedError(ValueError):
    """三物料粗定位配置或输入无效。"""


@dataclass(frozen=True)
class ThreeMaterialSeedConfig:
    enabled: bool
    reference_center_px: tuple[float, float]
    reference_radius_px: float
    roi_margin_px: float
    processing_scale: float
    minimum_end_radius_px: int
    maximum_end_radius_px: int
    hough_dp: float
    hough_edge_threshold: float
    hough_accumulator_threshold: float
    hough_minimum_distance_px: float
    maximum_candidates: int
    minimum_color_support_ratio: float
    endpoint_cluster_distance_px: float
    maximum_clusters: int
    minimum_material_spacing_px: float
    minimum_orbit_radius_px: float
    maximum_orbit_radius_px: float
    maximum_center_reference_distance_px: float
    outer_radial_search_half_width_px: float
    retry_interval_frames: int


@dataclass(frozen=True)
class ThreeMaterialSeedResult:
    detected: bool
    seed: tuple[float, float, float] | None = None
    material_centers_px: tuple[tuple[float, float], ...] = ()
    orbit_radius_px: float = 0.0
    candidate_count: int = 0
    cluster_count: int = 0
    processing_ms: float = 0.0
    detail: str = "尚未执行三物料粗定位"


def _require(condition: bool, detail: str) -> None:
    if not condition:
        raise ThreeMaterialSeedError(detail)


def load_three_material_seed_config(
    path: Path | str,
) -> ThreeMaterialSeedConfig:
    """读取并严格校验三物料粗定位参数。"""

    resolved = Path(path).expanduser().resolve()
    try:
        document = json.loads(resolved.read_text(encoding="utf-8"))
        _require(int(document["format_version"]) == 1, "不支持的配置版本")
        reference = document["reference"]
        detection = document["detection"]
        center = tuple(float(value) for value in reference["center_px"])
        _require(len(center) == 2, "参考圆心必须包含X和Y")
        config = ThreeMaterialSeedConfig(
            enabled=bool(document["enabled"]),
            reference_center_px=(center[0], center[1]),
            reference_radius_px=float(reference["radius_px"]),
            roi_margin_px=float(detection["roi_margin_px"]),
            processing_scale=float(detection["processing_scale"]),
            minimum_end_radius_px=int(detection["minimum_end_radius_px"]),
            maximum_end_radius_px=int(detection["maximum_end_radius_px"]),
            hough_dp=float(detection["hough_dp"]),
            hough_edge_threshold=float(detection["hough_edge_threshold"]),
            hough_accumulator_threshold=float(
                detection["hough_accumulator_threshold"]
            ),
            hough_minimum_distance_px=float(
                detection["hough_minimum_distance_px"]
            ),
            maximum_candidates=int(detection["maximum_candidates"]),
            minimum_color_support_ratio=float(
                detection["minimum_color_support_ratio"]
            ),
            endpoint_cluster_distance_px=float(
                detection["endpoint_cluster_distance_px"]
            ),
            maximum_clusters=int(detection["maximum_clusters"]),
            minimum_material_spacing_px=float(
                detection["minimum_material_spacing_px"]
            ),
            minimum_orbit_radius_px=float(
                detection["minimum_orbit_radius_px"]
            ),
            maximum_orbit_radius_px=float(
                detection["maximum_orbit_radius_px"]
            ),
            maximum_center_reference_distance_px=float(
                detection["maximum_center_reference_distance_px"]
            ),
            outer_radial_search_half_width_px=float(
                detection["outer_radial_search_half_width_px"]
            ),
            retry_interval_frames=int(detection["retry_interval_frames"]),
        )
    except ThreeMaterialSeedError:
        raise
    except (OSError, json.JSONDecodeError, KeyError, TypeError, ValueError) as error:
        raise ThreeMaterialSeedError(
            f"无法读取三物料转盘配置 {resolved}：{error}"
        ) from error

    values = (*config.reference_center_px, config.reference_radius_px)
    _require(all(math.isfinite(value) for value in values), "参考圆参数无效")
    _require(config.reference_radius_px > 0.0, "参考转盘半径必须大于0")
    _require(config.roi_margin_px >= 0.0, "搜索区域余量不能为负数")
    _require(0.25 <= config.processing_scale <= 1.0, "处理缩放必须位于0.25~1")
    _require(
        3 <= config.minimum_end_radius_px < config.maximum_end_radius_px,
        "物料端面半径范围无效",
    )
    _require(config.hough_dp >= 1.0, "Hough dp必须不小于1")
    _require(
        config.hough_edge_threshold > 0.0
        and config.hough_accumulator_threshold > 0.0
        and config.hough_minimum_distance_px > 0.0,
        "物料端面Hough参数必须大于0",
    )
    _require(3 <= config.maximum_candidates <= 100, "Hough候选数量无效")
    _require(
        0.0 <= config.minimum_color_support_ratio <= 1.0,
        "物料端面颜色支持率必须位于0~1",
    )
    _require(3 <= config.maximum_clusters <= 12, "物料簇数量上限无效")
    _require(
        config.endpoint_cluster_distance_px > 0.0
        and config.minimum_material_spacing_px > 0.0,
        "物料聚类距离必须大于0",
    )
    _require(
        0.0 < config.minimum_orbit_radius_px
        < config.maximum_orbit_radius_px,
        "物料轨迹半径范围无效",
    )
    _require(
        config.maximum_center_reference_distance_px > 0.0,
        "粗圆心最大偏移必须大于0",
    )
    _require(
        config.outer_radial_search_half_width_px > 0.0,
        "外圆径向搜索宽度必须大于0",
    )
    _require(1 <= config.retry_interval_frames <= 60, "重试帧间隔无效")
    return config


def _cluster_circle_candidates(candidates, distance_px):
    """把同一圆柱物料上相邻的两个端面圆归入同一簇。"""

    clusters = []
    for candidate in candidates:
        nearest = None
        nearest_distance = math.inf
        for cluster in clusters:
            distance = min(
                math.hypot(
                    candidate[0] - member[0], candidate[1] - member[1]
                )
                for member in cluster
            )
            if distance < nearest_distance:
                nearest = cluster
                nearest_distance = distance
        if nearest is not None and nearest_distance <= float(distance_px):
            nearest.append(candidate)
        else:
            clusters.append([candidate])
    return clusters


def _fit_circle(points):
    points = np.asarray(points, dtype=np.float64)
    if points.shape != (3, 2):
        raise ThreeMaterialSeedError("三物料轨迹圆必须由三个点拟合")
    design = np.column_stack((
        2.0 * points[:, 0],
        2.0 * points[:, 1],
        np.ones(3, dtype=np.float64),
    ))
    target = np.sum(points * points, axis=1)
    center_x, center_y, constant = np.linalg.lstsq(
        design, target, rcond=None
    )[0]
    radius_squared = constant + center_x**2 + center_y**2
    if radius_squared <= 0.0 or not np.all(np.isfinite(
        (center_x, center_y, radius_squared)
    )):
        raise ThreeMaterialSeedError("三物料轨迹圆拟合结果无效")
    return float(center_x), float(center_y), math.sqrt(radius_squared)


def _select_three_materials(clusters, config):
    """从较强候选簇中选择几何关系最合理的三件物料。"""

    reference_x, reference_y = config.reference_center_px
    ranked_clusters = sorted(
        clusters,
        key=lambda cluster: min(candidate[3] for candidate in cluster),
    )[:config.maximum_clusters]
    choices = []
    for selected in itertools.combinations(ranked_clusters, 3):
        # 圆柱的底端/接触端比顶端更靠近转盘参考圆心。
        points = tuple(
            min(
                cluster,
                key=lambda item: math.hypot(
                    item[0] - reference_x, item[1] - reference_y
                ),
            )
            for cluster in selected
        )
        pairwise = tuple(
            math.hypot(first[0] - second[0], first[1] - second[1])
            for first, second in itertools.combinations(points, 2)
        )
        if min(pairwise) < config.minimum_material_spacing_px:
            continue
        try:
            center_x, center_y, orbit_radius = _fit_circle(
                tuple((point[0], point[1]) for point in points)
            )
        except (ThreeMaterialSeedError, np.linalg.LinAlgError):
            continue
        center_distance = math.hypot(
            center_x - reference_x, center_y - reference_y
        )
        if center_distance > config.maximum_center_reference_distance_px:
            continue
        if not (
            config.minimum_orbit_radius_px <= orbit_radius
            <= config.maximum_orbit_radius_px
        ):
            continue
        expected_orbit = config.reference_radius_px * 0.65
        order_penalty = sum(point[3] for point in points) / max(
            1.0, config.maximum_candidates * 3.0
        )
        score = (
            center_distance / config.maximum_center_reference_distance_px
            + abs(orbit_radius - expected_orbit)
            / max(1.0, config.reference_radius_px * 0.45)
            + 0.20 * order_penalty
        )
        choices.append((score, center_x, center_y, orbit_radius, points))
    if not choices:
        raise ThreeMaterialSeedError("小圆候选不能组成可靠的三物料轨迹圆")
    return min(choices, key=lambda item: item[0])


class ThreeMaterialTurntableSeeder:
    """在固定大范围 ROI 内检测物料端面并产生外圆粗种子。"""

    def __init__(self, config: ThreeMaterialSeedConfig, color_config=None):
        self.config = config
        self.color_config = color_config

    def reset(self) -> None:
        """接口与有状态视觉检测器保持一致；当前实现本身无历史。"""

    def _failed(self, started_s, detail, candidates=0, clusters=0):
        return ThreeMaterialSeedResult(
            detected=False,
            candidate_count=int(candidates),
            cluster_count=int(clusters),
            processing_ms=(time.perf_counter() - started_s) * 1000.0,
            detail=str(detail),
        )

    def detect(self, frame) -> ThreeMaterialSeedResult:
        started_s = time.perf_counter()
        config = self.config
        if not config.enabled:
            return self._failed(started_s, "三物料粗定位已禁用")
        if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
            return self._failed(started_s, "三物料粗定位要求BGR彩色图像")

        height, width = frame.shape[:2]
        reference_x, reference_y = config.reference_center_px
        search_radius = config.reference_radius_px + config.roi_margin_px
        x0 = max(0, int(math.floor(reference_x - search_radius)))
        x1 = min(width, int(math.ceil(reference_x + search_radius + 1.0)))
        y0 = max(0, int(math.floor(reference_y - search_radius)))
        y1 = min(height, int(math.ceil(reference_y + search_radius + 1.0)))
        roi = frame[y0:y1, x0:x1]
        if roi.size == 0:
            return self._failed(started_s, "三物料搜索区域为空")

        scale = config.processing_scale
        if scale < 1.0:
            working = cv2.resize(
                roi, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA
            )
        else:
            working = roi
        gray = cv2.cvtColor(working, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 1.1)
        color_masks = (
            None
            if self.color_config is None
            else make_all_color_masks(working, self.color_config)
        )
        try:
            circles = cv2.HoughCircles(
                gray,
                cv2.HOUGH_GRADIENT,
                dp=config.hough_dp,
                minDist=max(
                    5.0, config.hough_minimum_distance_px * scale
                ),
                param1=config.hough_edge_threshold,
                param2=config.hough_accumulator_threshold,
                minRadius=max(2, int(math.floor(
                    config.minimum_end_radius_px * scale
                ))),
                maxRadius=max(3, int(math.ceil(
                    config.maximum_end_radius_px * scale
                ))),
            )
        except cv2.error as error:
            return self._failed(started_s, f"物料端面Hough失败：{error}")
        if circles is None:
            return self._failed(started_s, "搜索区内未找到物料端面小圆")

        candidates = []
        for order, (local_x, local_y, radius) in enumerate(circles[0]):
            if color_masks is not None:
                center_x_work = int(round(float(local_x)))
                center_y_work = int(round(float(local_y)))
                sample_radius = max(3, int(round(float(radius) * 0.85)))
                mask_y, mask_x = np.ogrid[
                    -sample_radius:sample_radius + 1,
                    -sample_radius:sample_radius + 1,
                ]
                disk = mask_x * mask_x + mask_y * mask_y <= (
                    sample_radius * sample_radius
                )
                sx0 = max(0, center_x_work - sample_radius)
                sx1 = min(working.shape[1], center_x_work + sample_radius + 1)
                sy0 = max(0, center_y_work - sample_radius)
                sy1 = min(working.shape[0], center_y_work + sample_radius + 1)
                dx0 = sx0 - (center_x_work - sample_radius)
                dx1 = dx0 + (sx1 - sx0)
                dy0 = sy0 - (center_y_work - sample_radius)
                dy1 = dy0 + (sy1 - sy0)
                local_disk = disk[dy0:dy1, dx0:dx1]
                pixel_count = max(1, int(np.count_nonzero(local_disk)))
                ranked_colors = []
                for color_id, mask in color_masks.items():
                    colored_pixels = np.count_nonzero(mask[sy0:sy1, sx0:sx1][local_disk])
                    support = int(colored_pixels) / pixel_count
                    ranked_colors.append((support, int(color_id)))
                # 支持率相同仍按颜色编号排序，保持原来的候选选择规则。
                ranked_colors.sort()
                color_support, color_id = ranked_colors[-1]
                if color_support < config.minimum_color_support_ratio:
                    continue
            else:
                color_support, color_id = 1.0, 0
            # Hough使用缩小后的ROI坐标；先除缩放倍数，再加ROI左上角。
            center_x = float(local_x / scale + x0)
            center_y = float(local_y / scale + y0)
            if math.hypot(
                center_x - reference_x, center_y - reference_y
            ) > search_radius:
                continue
            candidates.append((
                center_x,
                center_y,
                float(radius / scale),
                int(order),
                int(color_id),
                float(color_support),
            ))
            if len(candidates) >= config.maximum_candidates:
                break
        if len(candidates) < 3:
            return self._failed(
                started_s,
                f"只找到{len(candidates)}个物料端面候选，需要至少3个",
                len(candidates),
            )

        clusters = _cluster_circle_candidates(
            candidates, config.endpoint_cluster_distance_px
        )
        if len(clusters) < 3:
            return self._failed(
                started_s,
                f"端面候选只形成{len(clusters)}件物料，需要3件",
                len(candidates),
                len(clusters),
            )
        try:
            _, center_x, center_y, orbit_radius, points = (
                _select_three_materials(clusters, config)
            )
        except ThreeMaterialSeedError as error:
            return self._failed(
                started_s, error, len(candidates), len(clusters)
            )
        material_centers = tuple((point[0], point[1]) for point in points)
        return ThreeMaterialSeedResult(
            detected=True,
            seed=(center_x, center_y, config.reference_radius_px),
            material_centers_px=material_centers,
            orbit_radius_px=float(orbit_radius),
            candidate_count=len(candidates),
            cluster_count=len(clusters),
            processing_ms=(time.perf_counter() - started_s) * 1000.0,
            detail=(
                f"三物料粗圆心=({center_x:.1f},{center_y:.1f})px，"
                f"轨迹半径={orbit_radius:.1f}px"
            ),
        )
