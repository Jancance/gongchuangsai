"""Build a candidate update accepting the original or v2 installed code."""
import hashlib
import json
from pathlib import Path
import shutil
from zipfile import ZipFile,ZIP_DEFLATED

ROOT=Path(__file__).resolve().parents[1]
out=ROOT/'output/follow_touch_patch_v3_2026-10-06.zip'
manifest={'version':'circle-touch-candidate-v3-2026-10-06','hardware_acceptance':False,'files':{}}
with ZipFile(ROOT/'output/vision_app_decxin_2026-10-03.zip') as original, \
     ZipFile(ROOT/'output/follow_touch_patch_2026-10-06.zip') as previous, \
     ZipFile(out,'w',ZIP_DEFLATED) as package:
    previous_manifest=json.loads(previous.read('PATCH_MANIFEST.json'))
    for name in ('vision_app/code/project_detector.py','vision_app/code/project_follow.py',
                 'p2_integrated_follow_audit.py'):
        if name.startswith('vision_app/'):
            data=(ROOT/name).read_bytes()
            allowed=[hashlib.sha256(original.read(name)).hexdigest(),previous_manifest['files'][name]['after_sha256']]
        else:
            data=(ROOT/'scripts'/name).read_bytes()
            allowed=['9a1ee47418516ea37953d979b279f3a8ba9f1e9cbe45444e20c152f3c2ae71f4']
        manifest['files'][name]={'allowed_before_sha256':allowed,'after_sha256':hashlib.sha256(data).hexdigest()}
        package.writestr(name,data)
    package.writestr('PATCH_MANIFEST.json',json.dumps(manifest,indent=2))
shutil.copy2(ROOT/'scripts/apply_follow_touch_patch_v3.py',ROOT/'output/apply_follow_touch_patch_v3.py')
with ZipFile(out) as package:assert package.testzip() is None
print('Package:',out,'SHA256:',hashlib.sha256(out.read_bytes()).hexdigest())
