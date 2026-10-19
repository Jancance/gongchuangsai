"""Package only the two changed modules, with the installed baseline hashes."""
import hashlib
import json
from pathlib import Path
import shutil
from zipfile import ZipFile,ZIP_DEFLATED

ROOT=Path(__file__).resolve().parents[1]
NAMES=('vision_app/code/project_detector.py','vision_app/code/project_follow.py')
out=ROOT/'output/follow_touch_patch_2026-10-06.zip'
manifest={'version':'touch-candidate-2026-10-06','hardware_acceptance':False,'files':{}}
with ZipFile(ROOT/'output/vision_app_decxin_2026-10-03.zip') as baseline, ZipFile(out,'w',ZIP_DEFLATED) as package:
    for name in NAMES:
        data=(ROOT/name).read_bytes()
        manifest['files'][name]={'before_sha256':hashlib.sha256(baseline.read(name)).hexdigest(),
                                 'after_sha256':hashlib.sha256(data).hexdigest()}
        package.writestr(name,data)
    package.writestr('PATCH_MANIFEST.json',json.dumps(manifest,indent=2))
shutil.copy2(ROOT/'scripts/apply_follow_touch_patch.py',ROOT/'output/apply_follow_touch_patch.py')
with ZipFile(out) as package:assert package.testzip() is None
print('Package:',out,'SHA256:',hashlib.sha256(out.read_bytes()).hexdigest())
