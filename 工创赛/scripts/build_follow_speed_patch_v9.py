"""Build code-only optimization against V8 board hashes."""
import hashlib
import json
from pathlib import Path
import shutil
from zipfile import ZipFile,ZIP_DEFLATED

ROOT=Path(__file__).resolve().parents[1]
with ZipFile(ROOT/'evidence/20261006-195642-102168.zip') as evidence:
    hashes=json.loads(evidence.read('module_source_hashes.json'))
    preserved_config=hashlib.sha256(evidence.read('material_color_thresholds.json')).hexdigest()
out=ROOT/'output/follow_speed_patch_v9_2026-10-06.zip'
manifest={'version':'equivalent-roi-performance-candidate-v9-20261006','hardware_acceptance':False,
          'files':{},'preserve_sha256':{
              'vision_app/code/config/material_color_thresholds.json':preserved_config,
              'vision_app/code/project_tracker.py':hashes['project_tracker.py']}}
with ZipFile(out,'w',ZIP_DEFLATED) as archive:
    for name in ('vision_app/code/project_detector.py','vision_app/code/project_follow.py'):
        data=(ROOT/name).read_bytes();archive.writestr(name,data)
        manifest['files'][name]={'before_sha256':hashes[Path(name).name],'after_sha256':hashlib.sha256(data).hexdigest()}
    archive.writestr('PATCH_MANIFEST.json',json.dumps(manifest,indent=2))
shutil.copy2(ROOT/'scripts/apply_follow_speed_patch_v9.py',ROOT/'output/apply_follow_speed_patch_v9.py')
with ZipFile(out) as archive:assert archive.testzip() is None
print(out,'SHA256',hashlib.sha256(out.read_bytes()).hexdigest())
