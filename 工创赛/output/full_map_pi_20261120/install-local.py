#!/usr/bin/env python3
"""Offline install from an extracted package; never launch, stop or contact hardware."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--destination',type=Path,required=True)
    args=parser.parse_args()
    source=Path(__file__).resolve().parent
    destination=args.destination.expanduser().resolve()
    if destination==source or source in destination.parents:
        parser.error('Destination must be outside the extracted package')
    manifest=json.loads((source/'manifest.json').read_text())
    old_path=destination/'manifest.json'
    old=json.loads(old_path.read_text()) if old_path.is_file() else {}
    # Validate every file before the first write; preserve untracked/local edits.
    for name,digest in manifest.items():
        relative=Path(name)
        if relative.is_absolute() or '..' in relative.parts:
            raise ValueError('Unsafe manifest path: '+name)
        original=source/relative; target=destination/relative
        if original.is_symlink() or hashlib.sha256(original.read_bytes()).hexdigest()!=digest:
            raise ValueError('Source hash mismatch: '+name)
        if destination.is_symlink() or any(p.is_symlink() for p in [target,*target.parents]):
            raise ValueError('Symlink destination rejected: '+name)
        if target.exists() and hashlib.sha256(target.read_bytes()).hexdigest() not in (digest,old.get(name)):
            raise ValueError('Existing local edit preserved: '+name)
    for name in manifest:
        target=destination/name
        target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(source/name,target)
        if hashlib.sha256(target.read_bytes()).hexdigest()!=manifest[name]:
            raise ValueError('Destination hash mismatch: '+name)
    shutil.copyfile(source/'manifest.json',old_path)
    print(json.dumps(dict(installed=str(destination),files=len(manifest),started=False,
                         stopped=False,autostart_changed=False,external_configs_changed=False)))


if __name__=='__main__':main()
