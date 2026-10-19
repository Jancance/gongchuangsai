"""Apply the two-file diagnostic candidate; preserve config and back up code."""
import hashlib
from pathlib import Path
import shutil
import subprocess
import sys
from zipfile import ZipFile
import json

FILES=('vision_app/code/project_detector.py','vision_app/code/project_follow.py')


def main():
    root=Path(__file__).resolve().parent
    archive=root/'follow_touch_patch_2026-10-06.zip'
    with ZipFile(archive) as z:
        if z.testzip() is not None:raise RuntimeError('Patch ZIP integrity failure')
        manifest=json.loads(z.read('PATCH_MANIFEST.json'))
        if set(manifest['files'])!=set(FILES):raise RuntimeError('Unexpected patch members')
        updates={}
        originals={}
        for name in FILES:
            path=(root/name).resolve()
            if not path.is_relative_to(root):raise RuntimeError('Invalid destination')
            originals[name]=path.read_bytes()
            info=manifest['files'][name]
            current=hashlib.sha256(originals[name]).hexdigest()
            if current not in (info['before_sha256'],info['after_sha256']):
                raise RuntimeError('Code differs from expected version; left untouched: '+name)
            payload=z.read(name)
            if hashlib.sha256(payload).hexdigest()!=info['after_sha256']:
                raise RuntimeError('Patch member hash mismatch: '+name)
            compile(payload,name,'exec')
            updates[name]=payload
    if all(updates[n]==originals[n] for n in FILES):
        print('Patch already applied. Config unchanged.')
        return 0
    backup=root/'patch-backups/follow-touch-20261006'
    for name in FILES:
        saved=backup/name
        if not saved.exists():
            saved.parent.mkdir(parents=True,exist_ok=True)
            shutil.copy2(root/name,saved)
    try:
        for name in FILES:(root/name).write_bytes(updates[name])
        checked=subprocess.run([sys.executable,str(root/'vision_app/run.py'),'--check'],cwd=root)
        if checked.returncode:raise RuntimeError('Startup check failed')
    except BaseException:
        for name in FILES:(root/name).write_bytes(originals[name])
        print('Restored previous code; patch not accepted.')
        raise
    print('Patch applied; previous code backed up:',backup)
    print('Config unchanged. No camera/serial opened by this check.')
    return 0


if __name__=='__main__':raise SystemExit(main())
