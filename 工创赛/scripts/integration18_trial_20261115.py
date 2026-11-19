"""Use the verified b014231a owner with this session's MATLAB field geometry.

Default checks local files only. Motion requires an explicit action flag.
"""
import argparse
import json
from pathlib import Path

import source_commit_trial_20261114 as trial
from smart_carrier.vision_manipulation.source_body_registration import load_profile

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / "evidence/integration/2026-11-15/integration18"
CANDIDATE_EVIDENCE = ROOT / "evidence/integration/2026-11-14/overlap200-simplified"
FIELD_EVIDENCE = EVIDENCE


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    actions = parser.add_mutually_exclusive_group()
    actions.add_argument("--align-only", action="store_true")
    actions.add_argument("--once-batch", action="store_true")
    parser.add_argument("--debug-budget", type=float, default=0)
    parser.add_argument("--task-code", default="215+213+512+321")
    parser.add_argument("--port", type=int, default=18887)
    args = parser.parse_args()

    # Reuse the complete original candidate/source/Flash/MATLAB manifest checks.
    trial.E = CANDIDATE_EVIDENCE
    image, _, _ = trial.manifest()
    geometry = json.loads((FIELD_EVIDENCE / "parking-current-matlab.json").read_text(encoding="utf-8"))
    assert geometry["passed"] and geometry["local_matlab"]
    assert geometry["near_within_existing_tolerance"]
    poses = list(zip(geometry["current_arm_angles_deg"], geometry["current_extensions_mm"]))
    poses[2] = (180.0, 0.0)  # Existing near-region contract, checked by current image.
    profile = load_profile(FIELD_EVIDENCE / "source_body_registration_field.json")
    trial.E = EVIDENCE
    trial.manifest = lambda: (image, poses, profile)
    if args.align_only or args.once_batch:
        trial.run(args)
    else:
        print(json.dumps(dict(local_check=True, hardware_access=False,
            image_sha256=image["sha256"], field_profile=profile["profile_id"],
            poses=poses, task_code=args.task_code), ensure_ascii=False))


if __name__ == "__main__":
    main()
