"""原料区标记点离线标定。只产生诊断坐标，绝不产生运动许可。"""
from __future__ import annotations

import copy
import cv2
import numpy as np

from .calibration import estimate_ground_homography


def points(value, minimum=1):
    data = np.asarray(value, dtype=float)
    if data.ndim != 2 or data.shape[1] != 2 or len(data) < minimum or not np.isfinite(data).all():
        raise ValueError("点必须是有限数值 Nx2，且数量足够")
    return data


def context_checked(context):
    axes = context.get("axes")
    if axes not in ("x_right_y_forward_mm", "x_along_ab_y_toward_ad_mm"):
        raise ValueError("必须声明车体坐标或方纸局部坐标，单位mm")
    origin_key = "chassis_origin" if axes == "x_right_y_forward_mm" else "plane_origin"
    for key in ("camera_id", "mount_id", "arm_pose_id", "plane_id", origin_key):
        if not isinstance(context.get(key), str) or not context[key].strip():
            raise ValueError(f"缺少实测上下文: {key}")
    size = context.get("image_size")
    if not isinstance(size, list) or len(size) != 2 or any(type(x) is not int or x <= 0 for x in size):
        raise ValueError("image_size 必须为实际宽高")
    z = context.get("plane_z_mm")
    if type(z) not in (int, float) or not np.isfinite(z):
        raise ValueError("必须填写实际特征平面高度")
    if context.get("pixel_space") not in ("raw", "undistorted"):
        raise ValueError("必须说明像素是否已去畸变")
    if context["pixel_space"] == "undistorted" and not context.get("lens_profile_id"):
        raise ValueError("去畸变图必须绑定镜头标定版本")


def project(matrix, image):
    image = points(image)
    matrix = np.asarray(matrix, dtype=float)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise ValueError("无效H矩阵")
    homogeneous = np.column_stack((image, np.ones(len(image)))) @ matrix.T
    scale = np.linalg.norm(matrix[2]) * np.linalg.norm(np.column_stack((image, np.ones(len(image)))), axis=1)
    if np.any(np.abs(homogeneous[:, 2]) <= 1e-10 * np.maximum(scale, 1e-15)):
        raise ValueError("投影接近无穷远")
    mapped = homogeneous[:, :2] / homogeneous[:, 2:]
    if not np.isfinite(mapped).all():
        raise ValueError("非有限投影")
    return mapped


def inside_hull(image, fit_image):
    hull = cv2.convexHull(np.asarray(fit_image, dtype=np.float32))
    return all(cv2.pointPolygonTest(hull, tuple(map(float, p)), False) >= 0 for p in image)


def fit_markers(dataset):
    """至少四个拟合点、三个独立验证点；验证点不参与求解。"""
    context = dataset["context"]
    context_checked(context)
    if not isinstance(dataset.get("calibration_id"), str) or not dataset["calibration_id"].strip():
        raise ValueError("缺少标定版本")
    rows = dataset["markers"]
    if any(r.get("role") not in ("fit", "validation") for r in rows):
        raise ValueError("标记点role必须为fit或validation")
    ids = [r.get("id") for r in rows]
    if any(not isinstance(x, str) or not x.strip() for x in ids) or len(set(ids)) != len(ids):
        raise ValueError("标记点编号为空或重复")
    pixels = points([r["image_px"] for r in rows], 7)
    chassis_frame = context["axes"] == "x_right_y_forward_mm"
    physical_key = "chassis_mm" if chassis_frame else "plane_mm"
    physical = points([r[physical_key] for r in rows], 7)
    width, height = context["image_size"]
    if np.any(pixels < 0) or np.any(pixels >= [width, height]):
        raise ValueError("标记点超出原图范围")
    for values in (pixels, physical):
        # 同一个点不能换编号后同时进入拟合与验证。
        distances = np.linalg.norm(values[:, None] - values[None, :], axis=2)
        np.fill_diagonal(distances, np.inf)
        if np.min(distances) <= 1e-6:
            raise ValueError("存在重复像素或实测点，验证不独立")
    mask = np.array([r["role"] == "fit" for r in rows])
    if mask.sum() < 4 or (~mask).sum() < 3:
        raise ValueError("至少4个拟合点和3个独立验证点")
    for values in (pixels[mask], physical[mask], pixels[~mask], physical[~mask]):
        if np.linalg.matrix_rank(values - values.mean(axis=0)) < 2:
            raise ValueError("拟合/验证点不能共线")
    if not inside_hull(pixels[~mask], pixels[mask]):
        raise ValueError("验证点须位于拟合点包围区域内")
    limits = dataset["limits"]
    for key in ("fit_max_mm", "validation_max_mm"):
        if type(limits.get(key)) not in (float, int) or not np.isfinite(limits[key]) or limits[key] <= 0:
            raise ValueError("请填写正数工程误差门限，不使用官方精度假设")
    matrix, _, inliers = estimate_ground_homography(pixels[mask], physical[mask])
    errors = np.linalg.norm(project(matrix, pixels) - physical, axis=1)
    passed = bool(inliers.all() and errors[mask].max() <= limits["fit_max_mm"]
                  and errors[~mask].max() <= limits["validation_max_mm"])
    def stats(e):
        return {"rms_mm": float(np.sqrt(np.mean(e**2))), "p95_mm": float(np.percentile(e, 95)),
                "max_mm": float(e.max()), "count": len(e)}
    matrix_key = "homography_image_to_chassis" if chassis_frame else "homography_image_to_plane"
    return {"schema_version": 1, "calibration_id": dataset["calibration_id"],
            "context": copy.deepcopy(context), matrix_key: matrix.tolist(),
            "coordinate_frame": "chassis" if chassis_frame else "paper_local",
            "fit_hull_image_px": pixels[mask].tolist(), "fit": stats(errors[mask]),
            "validation": stats(errors[~mask]), "errors_mm": dict(zip(ids, errors.tolist())),
            "limits": copy.deepcopy(limits), "independent_check_passed": passed,
            "motion_allowed": False, "physical_pick_verified": False,
            "note": "仅当前姿态/特征平面的离线映射；原始图尚未完成CAL-03，不是动作许可"}


def preview_points(calibration, observation):
    """映射三位置或真实标记盘心。禁止圆拟合中心和不同平面混用。"""
    if calibration.get("independent_check_passed") is not True:
        raise ValueError("独立验证未通过")
    context_checked(observation["context"])
    if calibration["context"] != observation["context"]:
        raise ValueError("相机/机构姿态/分辨率/工作平面/车体基准不匹配")
    if observation.get("calibration_id") != calibration["calibration_id"]:
        raise ValueError("标定版本不匹配")
    chassis_frame = observation["context"]["axes"] == "x_right_y_forward_mm"
    if not chassis_frame and "alignment" in observation:
        raise ValueError("方纸局部坐标不能生成车体纠偏，须先标定纸到车体/TCP关系")
    targets = observation["targets"]
    if not targets or any(t.get("kind") not in ("physical_marker", "feature_on_calibrated_plane") for t in targets):
        raise ValueError("仅接受实物标记或同标定平面的特征；图像拟合圆心不能直接使用")
    ids = [t.get("id") for t in targets]
    if any(not isinstance(i, str) or not i.strip() for i in ids) or len(ids) != len(set(ids)):
        raise ValueError("目标ID必须非空且唯一")
    pixels = points([t["image_px"] for t in targets])
    if not inside_hull(pixels, calibration["fit_hull_image_px"]):
        raise ValueError("目标超出实测标定覆盖区域，拒绝外推")
    matrix_key = "homography_image_to_chassis" if chassis_frame else "homography_image_to_plane"
    mapped = project(calibration[matrix_key], pixels)
    output_key = "chassis_mm" if chassis_frame else "plane_mm"
    result = {"calibration_id": calibration["calibration_id"], "motion_allowed": False,
              "coordinate_frame": "chassis" if chassis_frame else "paper_local",
              "targets": [{"id": t["id"], output_key: p.tolist()} for t, p in zip(targets, mapped)],
              "requires_fresh_observation_after_chassis_or_arm_motion": True}
    if "alignment" in observation:
        align = observation["alignment"]
        selected = [i for i, t in enumerate(targets) if t["id"] == align["center_marker_id"]]
        if len(selected) != 1 or targets[selected[0]]["kind"] != "physical_marker":
            raise ValueError("纠偏须用真实盘心标记，不能直接使用A拟合圆心")
        desired = points([align["desired_center_chassis_mm"]])[0]
        delta = mapped[selected[0]] - desired
        limit = align["max_translation_mm"]
        if type(limit) not in (float, int) or not np.isfinite(limit) or limit <= 0 or np.linalg.norm(delta) > limit:
            raise ValueError("纠偏超过本次工程平移上限")
        result["translation_preview_mm"] = delta.tolist()
        result["translation_assumption"] = "静止场地目标；底盘纯平移、航向保持；移动后重新取像"
        result["yaw_observable"] = False
    return result


def fit_paper(dataset):
    """矩形四角拟合，三个位于对角线上的已知标记仅作留出检查。

    A/B/C/D沿纸边依次排列；M为对角线交点，E/F为AM/BM中点。
    使用实测长宽，不假设裁纸准确，也不假设纸与车轴对齐。
    """
    width, height = dataset.get("width_mm"), dataset.get("height_mm")
    if any(type(v) not in (int, float) or not np.isfinite(v) or v <= 0 for v in (width, height)):
        raise ValueError("必须提供方纸实际边长width_mm/height_mm")
    if dataset["context"].get("axes") != "x_along_ab_y_toward_ad_mm":
        raise ValueError("方纸法必须使用局部坐标，不能直接声明车体坐标")
    rows = dataset["markers"]
    ids = [r["id"] for r in rows]
    if len(ids) != 7 or set(ids) != set("ABCDMEF"):
        raise ValueError("必须提供且只提供A/B/C/D/M/E/F七个像素点")
    image_by_id = {r["id"]: r["image_px"] for r in rows}
    corners = points([image_by_id[i] for i in "ABCD"], 4)
    if not cv2.isContourConvex(corners.astype(np.float32)):
        raise ValueError("四角须沿纸边依次排列，不能交叉或共线")
    physical = {"A": [0, 0], "B": [width, 0], "C": [width, height], "D": [0, height],
                "M": [width/2, height/2], "E": [width/4, height/4], "F": [3*width/4, height/4]}
    prepared = copy.deepcopy(dataset)
    prepared["markers"] = [{"id": i, "role": "fit" if i in "ABCD" else "validation",
                            "image_px": image_by_id[i], "plane_mm": physical[i]} for i in "ABCDMEF"]
    result = fit_markers(prepared)
    result["method"] = "measured_rectangle_four_corners_with_held_out_marks"
    result["measured_paper_mm"] = [width, height]
    result["validation_scope"] = "held-out paper marks only; ruler scale, lens, TCP and physical placement remain unverified"
    return result


def check_paper_transfer(calibration, dataset):
    """用移位方纸的已知长度检查原H；不重新拟合、不扩大可执行范围。"""
    context_checked(dataset["context"])
    if calibration.get("coordinate_frame") != "paper_local":
        raise ValueError("跨区域方纸检查必须使用方纸局部坐标")
    if calibration.get("independent_check_passed") is not True:
        raise ValueError("原方纸的独立检查未通过")
    if dataset["context"] != calibration["context"]:
        raise ValueError("相机/姿态/平面上下文与原标定不匹配")
    if dataset.get("calibration_id") != calibration["calibration_id"]:
        raise ValueError("标定版本不匹配")
    limit = dataset.get("length_error_limit_mm")
    if type(limit) not in (int, float) or not np.isfinite(limit) or limit <= 0:
        raise ValueError("须显式填写正数长度诊断门限")
    views = dataset["views"]
    if not views:
        raise ValueError("缺少跨区域方纸图")
    labels = [view.get("label") for view in views]
    if any(not isinstance(label, str) or not label.strip() for label in labels) or len(set(labels)) != len(labels):
        raise ValueError("检查区域名称须非空且唯一")
    matrix = calibration["homography_image_to_plane"]
    width, height = dataset["context"]["image_size"]
    results = []
    for view in views:
        w, h = view["measured_paper_mm"]
        if any(type(v) not in (int, float) or not np.isfinite(v) or v <= 0 for v in (w, h)):
            raise ValueError("须提供检查方纸的实测长宽")
        corners = points(view["corners_ABCD_px"], 4)
        if len(corners) != 4 or np.any(corners < 0) or np.any(corners >= [width, height]):
            raise ValueError("四角须完整位于原图内，不补出缺失角")
        if not cv2.isContourConvex(corners.astype(np.float32)):
            raise ValueError("检查四角须依次排列且不能交叉/共线")
        # 此处允许原拟合区外的诊断投影；preview_points的执行边界不变。
        mm = project(matrix, corners)
        sides = np.linalg.norm(np.roll(mm, -1, axis=0) - mm, axis=1)
        diagonals = np.linalg.norm(mm[[2, 3]] - mm[[0, 1]], axis=1)
        expected = np.array([w, h, w, h, np.hypot(w, h), np.hypot(w, h)])
        errors = np.concatenate((sides, diagonals)) - expected
        angles = []
        for i in range(4):
            a, b = mm[i-1] - mm[i], mm[(i+1) % 4] - mm[i]
            cosine = a @ b / (np.linalg.norm(a) * np.linalg.norm(b))
            angles.append(float(np.degrees(np.arccos(np.clip(cosine, -1, 1)))))
        results.append({
            "label": view["label"], "diagnostic_corners_paper_mm": mm.tolist(),
            "AB_BC_CD_DA_mm": sides.tolist(), "AC_BD_mm": diagonals.tolist(),
            "corner_angles_deg": angles, "length_errors_mm": errors.tolist(),
            "max_abs_length_error_mm": float(np.max(np.abs(errors))),
            "length_consistency_passed": bool(np.max(np.abs(errors)) <= limit),
            "outside_original_fit_hull": not inside_hull(corners, calibration["fit_hull_image_px"]),
        })
    return {
        "schema_version": 1, "calibration_id": calibration["calibration_id"],
        "coordinate_frame": "paper_local", "context": copy.deepcopy(dataset["context"]),
        "length_error_limit_mm": limit, "views": results,
        "length_consistency_passed": all(r["length_consistency_passed"] for r in results),
        "original_homography_refitted": False, "operational_extrapolation_enabled": False,
        "absolute_position_verified": False, "field_calibration_accepted": False,
        "physical_pick_verified": False, "motion_allowed": False,
        "note": "只检查已知长度/形状；移动纸的绝对位置、镜头、车体/TCP及物料顶面未验证",
    }


def fit_circle_in_plane(matrix, boundary_image_px):
    """先映射边界再求物理平面圆心；输出只是数学拟合，不授权动作。"""
    mapped = project(matrix, points(boundary_image_px, 12))
    origin = mapped.mean(axis=0)
    local = mapped - origin
    design = np.column_stack((2 * local, np.ones(len(local))))
    coefficients, _, rank, _ = np.linalg.lstsq(design, np.sum(local**2, axis=1), rcond=None)
    if rank != 3:
        raise ValueError("圆边界退化，不能由共线点恢复圆心")
    radius2 = coefficients[2] + coefficients[:2] @ coefficients[:2]
    if not np.isfinite(radius2) or radius2 <= 0:
        raise ValueError("无有效平面圆半径")
    center = coefficients[:2] + origin
    radius = float(np.sqrt(radius2))
    distances = np.linalg.norm(mapped - center, axis=1)
    angles = np.sort(np.arctan2(mapped[:, 1] - center[1], mapped[:, 0] - center[0]))
    gaps = np.diff(np.append(angles, angles[0] + 2 * np.pi))
    return {"center_plane_mm": center.tolist(), "radius_mm": radius,
            "boundary_rms_mm": float(np.sqrt(np.mean((distances - radius)**2))),
            "boundary_max_mm": float(np.max(np.abs(distances - radius))),
            "angular_coverage_deg": float(np.degrees(2 * np.pi - gaps.max())),
            "boundary_point_count": len(mapped), "motion_allowed": False}


def tcp_relative_offsets(matrix, reference_image_px, extension_image_direction, targets_plane_mm):
    """以用户给出的夹爪参考像素和伸出方向建立局部右/前坐标，不是车体坐标。"""
    reference = points([reference_image_px])[0]
    direction = points([extension_image_direction])[0]
    if np.linalg.norm(direction) <= 1e-9:
        raise ValueError("伸出方向不能为零")
    direction /= np.linalg.norm(direction)
    origin, ahead = project(matrix, [reference, reference + direction])
    forward = ahead - origin
    if np.linalg.norm(forward) <= 1e-9:
        raise ValueError("伸出方向在平面中退化")
    forward /= np.linalg.norm(forward)
    right = np.array([-forward[1], forward[0]])
    _, right_pixel = project(matrix, [reference, reference + [1, 0]])
    if right @ (right_pixel - origin) < 0:
        right = -right
    offsets = points(targets_plane_mm) - origin
    return {"reference_plane_mm": origin.tolist(), "right_unit_in_paper": right.tolist(),
            "forward_unit_in_paper": forward.tolist(),
            "right_forward_mm": np.column_stack((offsets @ right, offsets @ forward)).tolist(),
            "coordinate_frame": "tcp_plane_local_right_forward",
            "motion_allowed": False, "physical_tcp_verified": False,
            "note": "偏移不是关节命令；仍需旋转中心/臂长/行程/旋转符号和同平面验证"}
