"""Install equivalent detector/follower optimization; do not write settings."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
from zipfile import ZipFile

FILES=('vision_app/code/project_detector.py','vision_app/code/project_follow.py')

def main():
    root=Path(__file__).resolve().parent
    with ZipFile(root/'follow_speed_patch_v9_2026-10-06.zip') as archive:
        if archive.testzip() is not None:raise RuntimeError('Patch ZIP integrity failure')
        manifest=json.loads(archive.read('PATCH_MANIFEST.json'))
        if set(manifest['files'])!=set(FILES):raise RuntimeError('Unexpected patch members')
        for name,digest in manifest['preserve_sha256'].items():
            path=(root/name).resolve()
            if not path.is_relative_to(root):raise RuntimeError('Invalid preserved path')
            if hashlib.sha256(path.read_bytes()).hexdigest()!=digest:
                raise RuntimeError('Settings/tracker differ from returned V8 evidence; left untouched: '+name)
        before={};after={}
        for name in FILES:
            path=(root/name).resolve()
            if not path.is_relative_to(root):raise RuntimeError('Invalid destination')
            before[name]=path.read_bytes();entry=manifest['files'][name]
            if hashlib.sha256(before[name]).hexdigest() not in (entry['before_sha256'],entry['after_sha256']):
                raise RuntimeError('Code differs from returned version; left untouched: '+name)
            after[name]=archive.read(name)
            if hashlib.sha256(after[name]).hexdigest()!=entry['after_sha256']:
                raise RuntimeError('Patch member hash mismatch: '+name)
            compile(after[name],name,'exec')
    if all(before[n]==after[n] for n in FILES):
        print('Follow speed v9 already applied. Settings unchanged.');return 0
    backup=root/'patch-backups/follow-speed-v9-20261006'
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
        print('Restored previous code; patch not accepted.');raise
    print('Follow speed v9 applied; previous code backed up:',backup)
    print('Color/camera settings unchanged. Freshness limit remains 150ms. No camera/serial opened by check.')
    return 0

if __name__=='__main__':raise SystemExit(main())
