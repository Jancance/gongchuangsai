"""Offline raw-image review of stage-3 ring 2 and three loaded stacking slots."""

import argparse
import hashlib
import inspect
import json
import math
from pathlib import Path, PureWindowsPath
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / "evidence/integration/2026-11-18/stage3-resume"
EMPTY_STAMP = "20261119-025120-769314"
EMPTY_PREFIX = "ring2-empty-after-forward12"
EMPTY_CAPTURE = EVIDENCE / f"{EMPTY_PREFIX}-capture-{EMPTY_STAMP}.json"
REPLAY_ONLY_EMPTY_HASHES = {
    "9aa75783f4be822dbdb2e7eb655724f453c6841b94c8880f4bced42beb5ccb7d",
    "a0189f08b904c8666123560ec702df65faca7012db32d50f96981a73544c4bc6",
}
EMPTY_RAW_SIZE = [1920, 1080]
RING_PROFILE = EVIDENCE / "image-analysis/before-search-config.json"
VISION = ROOT / "vision_app/code"
COLOR_PROFILE = VISION / "config/material_color_thresholds.json"
RELEASED = ROOT / "output/stage3_ring_pi_20261116"
sys.path.insert(0, str(VISION))
from turntable_serial import TurntableSerialLink
from 颜色标定 import load_color_config, make_all_color_masks, make_color_mask

sys.path.insert(0, str(RELEASED))
# stage3 imports the released detector bundle from its vendor directory.
from stage3 import RingStationDetector, camera_geometry_signature, ring_config, stacked_config
from stacked_ring_detector import StackedRingDetector, StackedRingDetectionError


def file_id(path):
    path = Path(path).resolve(strict=True)
    if not path.is_file():
        raise ValueError(f"Not a local file: {path}")
    data = path.read_bytes()
    return {"file": str(path), "sha256": hashlib.sha256(data).hexdigest()}, data


def read_frame(path):
    identity, data = file_id(path)
    frame = cv2.imdecode(np.frombuffer(data, np.uint8),
                         cv2.IMREAD_COLOR | cv2.IMREAD_IGNORE_ORIENTATION)
    if frame is None:
        raise ValueError(f"Image could not be decoded: {path}")
    identity.update(size=list(frame.shape[1::-1]),
                    decoded_sha256=hashlib.sha256(frame.tobytes()).hexdigest())
    return identity, frame


def load_raw_profile(path, frame):
    identity, data = file_id(path)
    profile = json.loads(data.decode("utf-8-sig"))
    camera = profile["camera"]
    if camera.get("undistort") is not False:
        raise ValueError("This tool accepts only the profile's unrectified raw image space")
    if [camera["width"], camera["height"]] != list(frame.shape[1::-1]):
        raise ValueError("Image dimensions do not match the raw profile")
    return identity, profile


def image_point(point, frame):
    h, w = frame.shape[:2]
    x, y = point
    if not all(math.isfinite(v) for v in point) or not (0 <= x < w and 0 <= y < h):
        raise ValueError("Point outside the raw image")
    return float(x), float(y)


def image_roi(roi, frame):
    h, w = frame.shape[:2]
    x, y, rw, rh = roi
    if min(x, y) < 0 or min(rw, rh) <= 0 or x + rw > w or y + rh > h:
        raise ValueError("ROI outside the raw image or empty")
    return x, y, rw, rh


def marker(frame, point, color, text):
    xy = tuple(round(v) for v in point)
    cv2.drawMarker(frame, xy, color, cv2.MARKER_CROSS, 26, 2, cv2.LINE_AA)
    label(frame, text, (xy[0] + 12, max(20, xy[1] - 12)), color)


def label(frame, text, point=(12, 30), color=(255, 255, 255)):
    cv2.putText(frame, text, point, cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(frame, text, point, cv2.FONT_HERSHEY_SIMPLEX, .65, color, 1, cv2.LINE_AA)


def capture_image_path(manifest, recorded_path):
    win = PureWindowsPath(recorded_path)
    native = Path(recorded_path)
    folder = manifest.parent
    if win.drive.startswith("\\\\") or recorded_path.startswith("//"):
        raise ValueError("Capture image must not use a network or device path")
    if native.is_absolute() or win.is_absolute():
        # Prefer a colocated copy when an evidence folder was relocated.
        local_copy = (folder / win.name).resolve()
        if not local_copy.is_relative_to(folder):
            raise ValueError("Colocated capture image resolves outside its evidence directory")
        if local_copy.is_file():
            return local_copy
        if native.is_absolute():
            return native
        raise ValueError("Absolute capture path is unavailable; place its image beside the manifest")
    if win.anchor or ".." in win.parts:
        raise ValueError("Relative capture image must stay inside its manifest directory")
    resolved = (folder / Path(*win.parts)).resolve()
    if not resolved.is_relative_to(folder):
        raise ValueError("Relative capture image resolves outside its manifest directory")
    return resolved


def measure_empty(capture_path=None, *, for_physical=False):
    if for_physical and capture_path is None:
        raise ValueError("Physical loaded observations require --empty-capture from the same post-repower view")
    capture_id, data = file_id(capture_path if capture_path is not None else EMPTY_CAPTURE)
    manifest = Path(capture_id["file"])
    capture = json.loads(data.decode("utf-8-sig"))
    if capture["same_stopped_pose_pair"] is not True:
        raise ValueError("Empty-pair manifest lacks same-stopped-pose evidence")
    if for_physical and any(image["sha256"] in REPLAY_ONLY_EMPTY_HASHES
                            for image in (capture["images"]["before"], capture["images"]["after"])):
        raise ValueError("Known pre-repower empty evidence is replay-only; use a new same-view manifest")
    results, overlays = [], {}
    for name in ("before", "after"):
        image = capture["images"][name]
        identity, frame = read_frame(capture_image_path(manifest, image["local"]))
        if (identity["sha256"] != image["sha256"]
                or identity["size"] != image["size"] or identity["size"] != EMPTY_RAW_SIZE):
            raise ValueError(f"Empty {name} frame disagrees with its capture manifest")
        profile_id, profile = load_raw_profile(RING_PROFILE, frame)
        detector = RingStationDetector(ring_config(profile, tuple(identity["size"])))
        result = detector.detect(frame)
        if not result.detected or len(result.centers_px) != 3:
            raise ValueError(f"No three actual rings found in empty {name} frame: {result.detail}")
        for slot, center in enumerate(result.centers_px, 1):
            marker(frame, center, (0, 235, 0), f"R{slot}")
        label(frame, f"ARCHIVED EMPTY {name.upper()} | raw pixels")
        results.append({**identity, "actual_centers_px": result.centers_px})
        overlays[f"empty-{name}.png"] = frame
    if results[0]["decoded_sha256"] == results[1]["decoded_sha256"]:
        raise ValueError("Empty pair duplicated")
    r2 = np.asarray([row["actual_centers_px"][1] for row in results])
    return {
        "provenance": "archived_same_stopped_pose_empty_pair",
        "manifest_selection": "explicit" if capture_path is not None else "implicit_replay_only",
        "capture_manifest": capture_id,
        "profile": profile_id,
        "detector": file_id(inspect.getfile(RingStationDetector))[0],
        "frames": results,
        "ring2_floor_center_px": tuple(r2.mean(axis=0)),
        "ring2_pair_after_minus_before_px": tuple(r2[1] - r2[0]),
    }, overlays


def red_top(frame, roi, colors, overlay):
    x, y, w, h = image_roi(roi, frame)
    mask = make_color_mask(frame[y:y + h, x:x + w], 1, colors)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted((c for c in contours if cv2.contourArea(c) > 0),
                      key=cv2.contourArea, reverse=True)
    cv2.rectangle(overlay, (x, y), (x + w - 1, y + h - 1), (20, 230, 255), 1)
    evidence = {"roi_px": roi, "meaning": "red_mask_contour_centroid_not_verified_top_center",
                "component_areas_px2": [cv2.contourArea(c) for c in contours]}
    if len(contours) != 1:
        return None, {**evidence, "detail": "Need one red contour; review mask or mark center manually"}, mask
    contour = contours[0]
    bx, by, bw, bh = cv2.boundingRect(contour)
    cv2.drawContours(overlay, [contour + (x, y)], -1, (20, 230, 255), 2)
    if bx == 0 or by == 0 or bx + bw == w or by + bh == h:
        return None, {**evidence, "detail": "Contour touches ROI boundary; use wider ROI or mark manually"}, mask
    m = cv2.moments(contour)
    return (x + m["m10"] / m["m00"], y + m["m01"] / m["m00"]), evidence, mask


def load_independent(paths, excluded=()):
    seen = set(excluded)
    for path in paths:
        identity, frame = read_frame(path)
        if identity["decoded_sha256"] in seen:
            raise ValueError("Duplicate decoded image cannot count as independent evidence")
        seen.add(identity["decoded_sha256"])
        yield identity, frame


def ring2_review(args):
    reference, overlays = measure_empty(args.empty_capture, for_physical=args.provenance == "physical")
    results = []
    report = {
        "mode": "ring2", "reference": reference, "frames": results,
        "same_view_operator_confirmed": args.same_view_confirmed,
        "camera_pose_independently_verified": False,
        "physical_test_slot_operator_declared": 2,
        "provenance": args.provenance or "empty_archive_only",
        "status": "AWAITING_LOADED_FRAME",
        "coordinate_contract": "raw image px: +x right, +y down; floor vs elevated top",
    }
    if not args.loaded:
        if args.top_roi or args.top_point or args.same_view_confirmed or args.provenance:
            raise ValueError("Loaded-frame annotations require --loaded")
        return report, overlays

    marks = args.top_roi if args.top_roi else args.top_point
    if marks and len(marks) != len(args.loaded):
        raise ValueError("Supply one ROI or manual point per loaded frame")
    if not args.provenance:
        raise ValueError("Declare loaded-image --provenance")
    if args.top_roi:
        report["color_config"] = file_id(args.color_config)[0]
        colors = load_color_config(args.color_config)
        report["color_config_declares_calibrated_camera"] = bool(colors.get("calibrated_for_this_camera"))
    floor_ref = reference["ring2_floor_center_px"]
    for i, (identity, frame) in enumerate(load_independent(
            args.loaded, [r["decoded_sha256"] for r in reference["frames"]])):
        if identity["size"] != reference["frames"][0]["size"]:
            raise ValueError("Loaded image has different dimensions; do not crop, resize, or substitute a phone image")
        overlay = frame.copy()
        center, evidence = None, {"detail": "Supply a top-face ROI or manually mark its center"}
        if args.top_roi:
            center, evidence, mask = red_top(frame, args.top_roi[i], colors, overlay)
            overlays[f"loaded-{i + 1:02}-red-mask.png"] = mask
        elif args.top_point:
            center = image_point(args.top_point[i], frame)
            evidence = {"meaning": "operator_marked_projected_top_center_not_detector_output"}
        delta = None
        if args.same_view_confirmed:
            marker(overlay, floor_ref, (0, 235, 0), "R2 EMPTY REF")
            if center is not None:
                delta = tuple(a - b for a, b in zip(center, floor_ref))
                cv2.line(overlay, tuple(map(round, floor_ref)), tuple(map(round, center)),
                         (220, 220, 40), 2, cv2.LINE_AA)
        if center is not None:
            marker(overlay, center, (0, 155, 255), "RED TOP")
        label(overlay, f"{args.provenance.upper()} | raw pixels only | no floor offset")
        results.append({**identity, "red_top_observation_px": center, "evidence": evidence,
                        "top_minus_archived_floor_reference_px": delta})
        overlays[f"loaded-{i + 1:02}.png"] = overlay
    report["status"] = (
        "REVIEW_PIXEL_OBSERVATIONS_ONLY"
        if all(row["top_minus_archived_floor_reference_px"] is not None for row in results)
        else "INCOMPLETE_PIXEL_OBSERVATIONS"
    )
    return report, overlays


def expected_colors(task_code):
    protocol = TurntableSerialLink
    task = protocol.decode_task_code_frame(
        bytes((protocol.RX_HEADER, protocol.RX_TASK_FUNCTION))
        + task_code.encode("ascii") + bytes((protocol.RX_TAIL,)))
    return dict(zip(task.first_rings, task.first_colors))


def check_slots(rois, frame):
    rois = [image_roi(roi, frame) for roi in rois]
    if len(rois) != 3 or any(a[0] + a[2] > b[0] for a, b in zip(rois, rois[1:])):
        raise ValueError("Need three distinct, nonoverlapping top-face ROIs in physical slot order, left to right")
    return rois


def inspect_slot(roi, frame, colors, expected):
    x, y, w, h = roi
    counts = {key: int(np.count_nonzero(mask)) for key, mask in
              make_all_color_masks(frame[y:y + h, x:x + w], colors).items()}
    winners = [color for color, count in counts.items() if 2 * count > w * h]
    # Multiple HSV classes can overlap; an unresolved overlap stays unknown.
    observed = winners[0] if len(winners) == 1 else None
    return {"roi_px": roi, "matching_pixels_by_color": counts, "roi_pixels": w * h,
            "expected_color_id": expected, "majority_color_id": observed,
            "roi_color_matches": observed == expected}


def stack_review(args):
    expected = expected_colors(args.task_code)
    color_id, _ = file_id(args.color_config)
    colors = load_color_config(args.color_config)
    results, overlays = [], {}
    for i, (identity, frame) in enumerate(load_independent(args.loaded)):
        profile_id, profile = load_raw_profile(args.profile, frame)
        if not profile.get("search_calibration"):
            raise ValueError("S needs a matching ring/long-edge search record; no template fallback")
        detector = StackedRingDetector(stacked_config(profile), ring_config(profile, tuple(identity["size"])))
        detector.set_camera_geometry(camera_geometry_signature(*identity["size"], False, None))
        overlay = frame.copy()
        try:
            # Pixel-detection stage of S: never enter its legacy metric/correction path.
            raw = detector._raw_detection(frame)
        except (StackedRingDetectionError, cv2.error, np.linalg.LinAlgError) as error:
            observation = {"detected": False, "detail": str(error)}
        else:
            observation = {
                "detected": True, "middle_ring_center_px": raw["middle"],
                "middle_ring_edge_support": raw["circle_support"],
                "long_line_segments_px": raw["segments"],
                "long_line_quality": raw["line_quality"],
                "modeled_side_centers_not_measured": True,
            }
            marker(overlay, raw["middle"], (0, 235, 0), "S MIDDLE")
            for start, end in raw["segments"]:
                cv2.line(overlay, tuple(map(round, start)), tuple(map(round, end)), (255, 100, 255), 2)
        slots = {}
        for slot, roi in enumerate(check_slots(args.top_roi, frame), 1):
            slots[slot] = inspect_slot(roi, frame, colors, expected[slot])
            x, y, w, h = roi
            color = (0, 235, 0) if slots[slot]["roi_color_matches"] else (0, 105, 255)
            cv2.rectangle(overlay, (x, y), (x + w - 1, y + h - 1), color, 2)
            label(overlay, f"R{slot}: {slots[slot]['majority_color_id']}/{expected[slot]}",
                  (x, max(20, y - 8)), color)
        label(overlay, f"{args.provenance.upper()} | loaded S {'detected' if observation['detected'] else 'UNKNOWN'}")
        overlays[f"stack-{i + 1:02}.png"] = overlay
        results.append({**identity, "s_mode": observation, "operator_selected_slot_colors": slots})
    found = all(r["s_mode"]["detected"] and
                all(s["roi_color_matches"] for s in r["operator_selected_slot_colors"].values())
                for r in results)
    return {
        "mode": "stack", "provenance": args.provenance, "task_code": args.task_code,
        "expected_first_batch_color_by_slot": expected,
        "profile": profile_id, "color_config": color_id,
        "color_config_declares_calibrated_camera": bool(colors.get("calibrated_for_this_camera")),
        "detector": file_id(inspect.getfile(StackedRingDetector))[0],
        "frames": results, "coordinate_contract": "raw image px; operator selected top-face ROIs",
        "status": "REVIEW_S_AND_COLOR_EVIDENCE_ONLY" if found else "S_OR_SLOT_COLOR_UNCONFIRMED",
        "geometric_stack_acceptance": False,
    }, overlays


def csv_values(text, parser, count):
    try:
        values = tuple(parser(value) for value in text.split(","))
        if len(values) == count:
            return values
    except ValueError:
        pass
    raise argparse.ArgumentTypeError(f"Expected {count} comma-separated coordinates")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    subs = parser.add_subparsers(dest="mode", required=True)
    ring = subs.add_parser("ring2", help="Compare a red top with the archived slot-2 empty stance")
    stack = subs.add_parser("stack", help="Inspect S detection and first-batch base colors")
    for sub in (ring, stack):
        sub.add_argument("--output-dir", type=Path, help="New folder for report and overlays")
        sub.add_argument("--loaded", type=Path, action="append", default=[], required=sub is stack)
        sub.add_argument("--provenance", choices=("physical", "archived", "synthetic"),
                         required=sub is stack)
        sub.add_argument("--color-config", type=Path, default=COLOR_PROFILE)
    marks = ring.add_mutually_exclusive_group()
    marks.add_argument("--top-point", action="append", type=lambda s: csv_values(s, float, 2),
                       help="Manually marked raw top-center x,y (one per loaded frame)")
    marks.add_argument("--top-roi", action="append", type=lambda s: csv_values(s, int, 4),
                       help="Locate a red contour in raw top-face x,y,w,h (one per loaded frame)")
    ring.add_argument("--same-view-confirmed", action="store_true",
                      help="Operator checked physical ring 2, camera/vehicle pose, lens and uncropped raster")
    ring.add_argument("--empty-capture", type=Path,
                      help="Same-view empty capture JSON; required with physical loaded images")
    stack.add_argument("--top-roi", action="append", required=True,
                       type=lambda s: csv_values(s, int, 4),
                       help="Three separate top-face interior x,y,w,h ROIs, slots 1/2/3, valid in all frames")
    stack.add_argument("--profile", required=True, type=Path,
                       help="Matching stage3-format raw S profile with search calibration")
    stack.add_argument("--task-code", required=True)
    args = parser.parse_args(argv)
    try:
        report, overlays = (ring2_review(args) if args.mode == "ring2" else stack_review(args))
        report.update(motion_allowed=False, metric_valid=False, acceptance_claim=False)
        text = json.dumps(report, indent=2, ensure_ascii=True, allow_nan=False)
        if args.output_dir:
            args.output_dir.mkdir(exist_ok=False)
            for name, image in overlays.items():
                ok, encoded = cv2.imencode(".png", image)
                if not ok:
                    raise OSError(f"Cannot encode {name}")
                (args.output_dir / name).write_bytes(encoded.tobytes())
            (args.output_dir / "report.json").write_text(text + "\n", encoding="utf-8")
        print(text)
    except (ValueError, TypeError, KeyError, OSError, cv2.error) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
