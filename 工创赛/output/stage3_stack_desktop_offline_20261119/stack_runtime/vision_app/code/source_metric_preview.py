"""Perspective-correct outer-rim observation. No serial, camera or motor I/O."""
from __future__ import annotations
import math
import numpy as np
import cv2


def _points(value, minimum=12):
    p = np.asarray(value, dtype=float)
    if p.ndim != 2 or p.shape[1] != 2 or len(p) < minimum or not np.isfinite(p).all():
        raise ValueError('finite Nx2 boundary points required')
    if len(np.unique(p, axis=0)) < minimum:
        raise ValueError('insufficient distinct boundary points')
    return p


def _project(h, points):
    h = np.asarray(h, dtype=float)
    if h.shape != (3, 3) or not np.isfinite(h).all() or np.linalg.matrix_rank(h) != 3:
        raise ValueError('invalid plane homography')
    q = np.column_stack((points, np.ones(len(points)))) @ h.T
    if np.any(np.abs(q[:, 2]) < 1e-10):
        raise ValueError('projection at infinity')
    q = q[:, :2] / q[:, 2:]
    if not np.isfinite(q).all():
        raise ValueError('nonfinite projection')
    return q


def _circle(p):
    origin = p.mean(axis=0)
    q = p - origin
    a = np.column_stack((2*q, np.ones(len(q))))
    v, _, rank, _ = np.linalg.lstsq(a, np.sum(q*q, axis=1), rcond=None)
    radius2 = v[2] + v[:2] @ v[:2]
    if rank != 3 or not math.isfinite(radius2) or radius2 <= 0:
        raise ValueError('degenerate circle boundary')
    return origin + v[:2], float(math.sqrt(radius2))


def preview_rim(boundary_px, calibration, *, frame_context, stable, confidence,
                frame_age_ms=None, timing_verified=False):
    """Rectify actual edge points, fit the physical circle, preview body delta.

    A successful preview never grants motion. An observation is eligible only
    with independently accepted full-plane, pose, body transform and timing.
    Paper-local diagnostics remain useful while those fields are incomplete.
    """
    result = {'status': 'UNCONFIGURED', 'motion_allowed': False,
              'metric_observation_ready': False, 'diagnostic_only': True}
    if not calibration or calibration.get('homography_image_to_plane') is None:
        return result
    try:
        if not isinstance(frame_context, dict) or frame_context != calibration.get('context'):
            result.update(status='CONTEXT_MISMATCH')
            return result
        if stable is not True:
            result.update(status='WAIT_STABLE')
            return result
        for key in ('camera_id', 'mount_id', 'arm_pose_id', 'plane_id'):
            if not isinstance(frame_context.get(key), str) or not frame_context[key].strip():
                raise ValueError('missing current context: ' + key)
        if frame_context.get('pixel_space') not in ('raw', 'undistorted'):
            raise ValueError('pixel space declaration required')
        if frame_context.get('pixel_space') == 'undistorted' and not frame_context.get('lens_profile_id'):
            raise ValueError('undistorted coordinates require lens profile')
        if type(confidence) not in (int, float) or not math.isfinite(confidence) or not 0 <= confidence <= 1:
            raise ValueError('invalid confidence')
        limits = calibration.get('rim_limits', {})
        minimum = int(limits.get('minimum_points', 12))
        if minimum < 12:
            raise ValueError('minimum_points below12')
        points = _points(boundary_px, minimum)
        size = frame_context.get('image_size')
        if not isinstance(size, list) or len(size) != 2 or any(type(v) is not int or v <= 0 for v in size):
            raise ValueError('actual image size required')
        if np.any(points < 0) or np.any(points >= size):
            raise ValueError('boundary outside frame')
        q = _project(calibration['homography_image_to_plane'], points)
        keep = np.ones(len(q), dtype=bool)
        for _ in range(5):
            if np.count_nonzero(keep) < minimum:
                raise ValueError('too few retained rim points')
            center, radius = _circle(q[keep])
            residual = np.linalg.norm(q-center, axis=1)-radius
            median = np.median(residual[keep])
            mad = np.median(np.abs(residual[keep]-median))
            keep = np.abs(residual-median) <= max(1.5, 3*1.4826*mad)
        if np.count_nonzero(keep) < minimum:
            raise ValueError('too few final rim points')
        center, radius = _circle(q[keep])
        residual = np.linalg.norm(q[keep]-center, axis=1)-radius
        rms = float(np.sqrt(np.mean(residual**2)))
        angles = np.sort(np.arctan2(q[keep, 1]-center[1], q[keep, 0]-center[0]))
        coverage = float(np.degrees(2*np.pi-np.max(np.diff(np.r_[angles, angles[0]+2*np.pi]))))
        minimum_arc = float(limits.get('minimum_arc_deg', 90))
        maximum_rms = float(limits.get('maximum_rms_mm', 3))
        lo, hi = limits.get('radius_range_mm', [130, 170])
        if not all(math.isfinite(float(v)) for v in [minimum_arc, maximum_rms, lo, hi]) or not (0 < minimum_arc <= 360 and maximum_rms > 0 and 0 < lo < hi):
            raise ValueError('invalid rim limits')
        if coverage < minimum_arc or rms > maximum_rms or not lo <= radius <= hi:
            result.update(status='RIM_GEOMETRY_REJECTED', radius_mm=radius, rms_mm=rms, coverage_deg=coverage)
            return result
        hull = calibration.get('fit_hull_image_px')
        outside = True
        if hull is not None:
            poly = cv2.convexHull(_points(hull, 4).astype(np.float32))
            outside = any(cv2.pointPolygonTest(poly, tuple(map(float, p)), False) < 0 for p in points)
        result.update(status='PLANE_PREVIEW', center_plane_mm=center.tolist(), radius_mm=radius,
                      planning_obstacle_radius_mm=max(150., radius), boundary_rms_mm=rms,
                      angular_coverage_deg=coverage, retained_points=int(keep.sum()),
                      boundary_points_px=points[keep].tolist(), outside_calibrated_hull=outside,
                      coordinate_frame='paper_or_feature_plane_mm')
        transform = calibration.get('plane_to_chassis')
        target = calibration.get('desired_disk_center_chassis_mm')
        if transform is None or target is None:
            return result
        transform = np.asarray(transform, dtype=float)
        target = np.asarray(target, dtype=float)
        if (transform.shape != (3, 3) or not np.isfinite(transform).all() or
                not np.allclose(transform[2], [0, 0, 1], atol=1e-8) or
                not np.allclose(transform[:2, :2].T @ transform[:2, :2], np.eye(2), atol=1e-6) or
                target.shape != (2,) or not np.isfinite(target).all()):
            raise ValueError('measured rigid plane-to-body transform and target required')
        body = _project(transform, center.reshape(1, 2))[0]
        delta = body-target
        maximum = float(calibration.get('maximum_preview_translation_mm', 100))
        if not math.isfinite(maximum) or not 0 < maximum <= 100 or np.any(np.abs(delta) > maximum):
            result.update(status='DELTA_OUT_OF_RANGE')
            return result
        result.update(status='CHASSIS_DELTA_PREVIEW', center_chassis_mm=body.tolist(),
                      delta_chassis_mm=delta.tolist(), yaw_deg=None, circle_provides_heading=False)
        maximum_age = calibration.get('max_frame_age_ms', 200)
        if type(maximum_age) is not int or not 1 <= maximum_age <= 200:
            raise ValueError('invalid maximum frame age')
        # Current host-read timing is expressly insufficient to assert exposure age.
        age_ok = (timing_verified is True and type(frame_age_ms) in (int, float) and
                  math.isfinite(frame_age_ms) and 0 <= frame_age_ms < maximum_age)
        accepted = (calibration.get('independent_check_passed') is True and
                    calibration.get('field_calibration_accepted') is True and
                    calibration.get('pose_binding_verified') is True and
                    calibration.get('plane_to_chassis_verified') is True and
                    (not outside or calibration.get('operational_extrapolation_enabled') is True) and
                    confidence >= calibration.get('min_confidence', .9) and age_ok)
        result['metric_observation_ready'] = bool(accepted)
        result['diagnostic_only'] = not bool(accepted)
        return result
    except (ValueError, TypeError, KeyError, np.linalg.LinAlgError) as exc:
        result.update(status='INVALID_GEOMETRY', detail=str(exc), metric_observation_ready=False)
        return result
