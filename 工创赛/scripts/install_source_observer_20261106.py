"""Install into a new directory and inherit current saved configs. No app launch."""
from pathlib import Path
import argparse
import hashlib
import json
import shutil


def install(payload, base, destination):
    payload, base, destination = Path(payload).resolve(), Path(base).resolve(), Path(destination).resolve()
    if not (base / 'run.py').is_file() or not (base / 'code/config').is_dir():
        raise ValueError('--base must be the existing vision_app directory')
    if destination.exists() or destination == base or base in destination.parents:
        raise ValueError('destination must be a new directory outside the old app')
    manifest = json.loads((payload.parent / 'payload-manifest.json').read_text(encoding='utf-8'))
    for relative, digest in manifest['sha256'].items():
        path = (payload / relative).resolve()
        if payload not in path.parents or not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            raise ValueError('payload checksum/path mismatch: ' + relative)
    saved = {}
    for path in sorted((base / 'code/config').rglob('*')):
        if path.is_file() and not path.is_symlink() and path.suffix.lower() in ('.json', '.yaml', '.yml'):
            data = path.read_bytes()
            relative = path.relative_to(base)
            saved[relative.as_posix()] = (data, hashlib.sha256(data).hexdigest())
    if not saved:
        raise ValueError('no existing camera/HSV/config files found; do not substitute placeholders')
    shutil.copytree(payload, destination, ignore=shutil.ignore_patterns('__pycache__', 'diagnostics'))
    for relative, (data, _) in saved.items():
        p = destination / relative
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_bytes(data)
    for relative, (_, digest) in saved.items():
        if hashlib.sha256((base / relative).read_bytes()).hexdigest() != digest:
            raise RuntimeError('source config changed during installation: ' + relative)
    report = {'base': str(base), 'destination': str(destination), 'source_app_modified': False,
              'app_launched': False, 'camera_opened': False, 'serial_opened': False,
              'inherited_config_sha256': {name: digest for name, (_, digest) in saved.items()},
              'motion_allowed': False}
    (destination / 'install-report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base', type=Path, required=True)
    parser.add_argument('--destination', type=Path, required=True)
    parser.add_argument('--payload', type=Path, default=Path(__file__).parent / 'vision_app')
    args = parser.parse_args()
    print(json.dumps(install(args.payload, args.base, args.destination), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
