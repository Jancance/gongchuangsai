"""Prepare an isolated HSV live trial from the benchmarked modules; no device I/O."""
from pathlib import Path
import argparse
import hashlib
import json
import shutil


def prepare(source, candidate, destination):
    source, candidate, destination = map(Path, (source, candidate, destination))
    expected = {
        'project_detector.py': 'cfbe8b1135059f8d3194fddf59c4a2fa090163e204152041d7c206ca9970e270',
        'project_hsv_masks.py': 'ddb301a6bbae5148580e93d1687e5493d721183cdf3add104664a99bdaccb87b',
    }
    sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    if destination.exists():
        raise RuntimeError('Trial already exists; refuse overwrite: ' + str(destination))
    if sha(source/'code/project_detector.py') != 'e5014e86b7869d22cacaf01b9cfb30d57e8e6b4ed8670bbf043962f6df448c95':
        raise RuntimeError('Unexpected source detector')
    for name, digest in expected.items():
        if sha(candidate/name) != digest:
            raise RuntimeError('Unexpected candidate: ' + name)
    profile = json.loads((source/'code/config/project_profile.json').read_text(encoding='utf-8'))
    if not (profile['serial_enabled'] is False and profile['follow_grasp_enabled'] is False
            and profile['geometry_verified'] is False and profile['tracking_units'] == 'pixels'):
        raise RuntimeError('Unexpected source permission or geometry profile')
    originals = {p.relative_to(source).as_posix(): sha(p) for p in source.rglob('*')
                 if p.is_file() and '__pycache__' not in p.parts}
    shutil.copytree(source, destination, ignore=shutil.ignore_patterns('__pycache__', 'diagnostics', 'color-follow-audits'))
    for name in expected:
        shutil.copy2(candidate/name, destination/'code'/name)
    changed = []
    for name, digest in originals.items():
        if sha(source/name) != digest:
            raise RuntimeError('Source changed during preparation: ' + name)
        target = destination/name
        if target.exists() and sha(target) != digest:
            changed.append(name)
    if changed != ['code/project_detector.py']:
        raise RuntimeError('Unexpected copied-file changes: ' + repr(changed))
    additions = sorted(p.relative_to(destination).as_posix() for p in destination.rglob('*')
                       if p.is_file() and p.relative_to(destination).as_posix() not in originals)
    if additions != ['code/project_hsv_masks.py']:
        raise RuntimeError('Unexpected additions: ' + repr(additions))
    result = dict(source=str(source), destination=str(destination), changed_files=changed,
                  added_files=additions, candidate_sha256=expected, source_unchanged=True,
                  camera_opened=False, serial_opened=False, live_acceptance='pending')
    (destination.parent/'hsv-live-preparation.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--home', type=Path, default=Path('/home/orangepi'))
    args = parser.parse_args()
    print(json.dumps(prepare(args.home/'p2-blue-perimeter-20261105/vision_app',
                             args.home/'p2-hsv-benchmark-20261105',
                             args.home/'p2-hsv-live-20261105/vision_app'), indent=2))
