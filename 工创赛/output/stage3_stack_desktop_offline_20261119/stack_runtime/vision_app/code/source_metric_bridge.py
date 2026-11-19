"""Build an observation packet from verified preview data. Never opens serial."""
import math
from metric_alignment_protocol import MetricObservation, MetricRequest, calibrated_delta_to_wire, encode_observation


def observation_packet(preview, request, *, profile_ids, frame_sequence,
                       capture_after_request_ms, frame_age_ms, confidence,
                       timing_verified, translation_only_verified):
    if not isinstance(request, MetricRequest):
        raise ValueError('decoded live request required')
    request.validate()
    if (preview.get('metric_observation_ready') is not True or
            preview.get('status') != 'CHASSIS_DELTA_PREVIEW' or
            timing_verified is not True or translation_only_verified is not True):
        raise ValueError('verified geometry, exposure timing and translation-only hold required')
    keys = ['camera_id', 'calibration_id', 'mount_id', 'height_id', 'target_id']
    if any(type(profile_ids.get(k)) is not int or profile_ids[k] != getattr(request, k) for k in keys):
        raise ValueError('request and actual calibration/profile do not match')
    if (type(frame_age_ms) is not int or not 0 <= frame_age_ms < request.max_frame_age_ms or
            type(capture_after_request_ms) is not int or not 0 <= capture_after_request_ms < request.timeout_ms or
            type(confidence) not in (int, float) or not math.isfinite(confidence) or not 0 <= confidence <= 1):
        raise ValueError('invalid current frame age or confidence')
    confidence_permille = int(confidence*1000)
    if confidence_permille < request.min_confidence_permille:
        raise ValueError('insufficient confidence')
    dx, dy = preview['delta_chassis_mm']
    # yaw=0 means a validated translation-only correction, not a heading
    # measured from a circle; that claim remains false in the preview result.
    x, y, yaw = calibrated_delta_to_wire(units='chassis_mm', calibrated=True, dx=dx, dy=dy, yaw_deg=0)
    if abs(x) > request.max_delta_milli_mm or abs(y) > request.max_delta_milli_mm:
        raise ValueError('request-specific delta envelope exceeded')
    return encode_observation(MetricObservation(
        request.request_id, request.run_id, request.stage, request.start_id, request.map_version,
        request.goal_cell, request.camera_id, request.calibration_id, request.mount_id,
        request.height_id, request.target_id, frame_sequence, capture_after_request_ms,
        frame_age_ms, confidence_permille, 1, x, y, yaw))
