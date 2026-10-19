"""Install the opt-in black recovery candidate with exact-version guards."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
from zipfile import ZipFile

FILES=('vision_app/code/project_detector.py','vision_app/code/config/material_color_thresholds.json')

def main():
    root=Path(__file__).resolve().parent
    with ZipFile(root/'black_edge_patch_v12_2026-10-07.zip') as archive:
        if archive.testzip() is not None:raise RuntimeError('Patch ZIP integrity failure')
        manifest=json.loads(archive.read('PATCH_MANIFEST.json'))
        if set(manifest['files'])!=set(FILES):raise RuntimeError('Unexpected patch members')
        for name,digest in manifest['preserve_sha256'].items():
            path=(root/name).resolve()
            if not path.is_relative_to(root):raise RuntimeError('Invalid preserved path')
            if hashlib.sha256(path.read_bytes()).hexdigest()!=digest:
                raise RuntimeError('Follower/tracker differs from returned V11 evidence; left untouched: '+name)
        before={};after={}
        for name in FILES:
            path=(root/name).resolve()
            if not path.is_relative_to(root):raise RuntimeError('Invalid destination')
            before[name]=path.read_bytes();entry=manifest['files'][name]
            if hashlib.sha256(before[name]).hexdigest() not in (entry['before_sha256'],entry['after_sha256']):
                raise RuntimeError('Code/config differs from returned version; left untouched: '+name)
            after[name]=archive.read(name)
            if hashlib.sha256(after[name]).hexdigest()!=entry['after_sha256']:raise RuntimeError('Patch hash mismatch: '+name)
            if name.endswith('.py'):compile(after[name],name,'exec')
            else:json.loads(after[name])
    if all(before[n]==after[n] for n in FILES):
        print('Black edge v12 already applied.');return 0
    backup=root/'patch-backups/black-edge-v12-20261007'
    for name in FILES:
        target=backup/name
        if not target.exists():
            target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(root/name,target)
    try:
        for name in FILES:(root/name).write_bytes(after[name])
        checked=subprocess.run([sys.executable,str(root/'vision_app/run.py'),'--check'],cwd=root)
        if checked.returncode:raise RuntimeError('Startup check failed')
    except BaseException:
        for name in FILES:(root/name).write_bytes(before[name])
        print('Restored previous code/config; patch not accepted.');raise
    print('Black edge v12 applied; previous code/config backed up:',backup)
    print('Current-edge validation only. Original colour/camera ranges and freshness/serial gates preserved.')
    return 0

if __name__=='__main__':raise SystemExit(main())
