"""Build V4; keep the previous delivered packages immutable."""
import hashlib
import json
from pathlib import Path
import shutil
from zipfile import ZipFile,ZIP_DEFLATED

ROOT=Path(__file__).resolve().parents[1]
out=ROOT/'output/follow_touch_patch_v4_2026-10-06.zip'
names=('vision_app/code/project_detector.py','vision_app/code/project_follow.py',
       'vision_app/code/project_tracker.py','p2_integrated_follow_audit.py')
manifest={'version':'revalidated-circle-candidate-v4-2026-10-06','hardware_acceptance':False,'files':{}}
with ZipFile(ROOT/'output/vision_app_decxin_2026-10-03.zip') as original, \
     ZipFile(ROOT/'output/follow_touch_patch_2026-10-06.zip') as v2, \
     ZipFile(ROOT/'output/follow_touch_patch_v3_2026-10-06.zip') as v3, \
     ZipFile(out,'w',ZIP_DEFLATED) as package:
    for name in names:
        data=(ROOT/name if name.startswith('vision_app/') else ROOT/'scripts'/name).read_bytes()
        if name.startswith('vision_app/'):
            allowed=[hashlib.sha256(original.read(name)).hexdigest()]
            for version in (v2,v3):
                if name in version.namelist():allowed.append(hashlib.sha256(version.read(name)).hexdigest())
        else:
            allowed=['9a1ee47418516ea37953d979b279f3a8ba9f1e9cbe45444e20c152f3c2ae71f4',
                     hashlib.sha256(v3.read(name)).hexdigest()]
        manifest['files'][name]={'allowed_before_sha256':sorted(set(allowed)),
                                  'after_sha256':hashlib.sha256(data).hexdigest()}
        package.writestr(name,data)
    package.writestr('PATCH_MANIFEST.json',json.dumps(manifest,indent=2))
shutil.copy2(ROOT/'scripts/apply_follow_touch_patch_v4.py',ROOT/'output/apply_follow_touch_patch_v4.py')
with ZipFile(out) as package:assert package.testzip() is None
print('Package:',out,'SHA256:',hashlib.sha256(out.read_bytes()).hexdigest())
