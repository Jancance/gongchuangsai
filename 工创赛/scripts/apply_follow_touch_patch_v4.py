"""Install V4 with four-file version checks, backups and startup rollback."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
from zipfile import ZipFile

FILES=('vision_app/code/project_detector.py','vision_app/code/project_follow.py',
       'vision_app/code/project_tracker.py','p2_integrated_follow_audit.py')


def main():
    root=Path(__file__).resolve().parent
    with ZipFile(root/'follow_touch_patch_v4_2026-10-06.zip') as archive:
        if archive.testzip() is not None:raise RuntimeError('Patch ZIP integrity failure')
        manifest=json.loads(archive.read('PATCH_MANIFEST.json'))
        if set(manifest['files'])!=set(FILES):raise RuntimeError('Unexpected patch members')
        before={};after={}
        for name in FILES:
            path=(root/name).resolve()
            if not path.is_relative_to(root):raise RuntimeError('Invalid destination')
            before[name]=path.read_bytes();entry=manifest['files'][name]
            current=hashlib.sha256(before[name]).hexdigest()
            if current not in entry['allowed_before_sha256']+[entry['after_sha256']]:
                raise RuntimeError('Code differs from expected version; left untouched: '+name)
            after[name]=archive.read(name)
            if hashlib.sha256(after[name]).hexdigest()!=entry['after_sha256']:
                raise RuntimeError('Patch member hash mismatch: '+name)
            compile(after[name],name,'exec')
    if all(before[n]==after[n] for n in FILES):
        print('Patch v4 already applied. Config unchanged.')
        return 0
    backup=root/'patch-backups/follow-touch-v4-20261006'
    for name in FILES:
        target=backup/name
        if not target.exists():
            target.parent.mkdir(parents=True,exist_ok=True)
            shutil.copy2(root/name,target)
    try:
        for name in FILES:(root/name).write_bytes(after[name])
        check=subprocess.run([sys.executable,str(root/'vision_app/run.py'),'--check'],cwd=root)
        if check.returncode:raise RuntimeError('Startup check failed')
    except BaseException:
        for name in FILES:(root/name).write_bytes(before[name])
        print('Restored previous code; patch not accepted.')
        raise
    print('Patch v4 applied; previous code backed up:',backup)
    print('Config unchanged. No camera/serial opened by this check.')
    return 0


if __name__=='__main__':raise SystemExit(main())
