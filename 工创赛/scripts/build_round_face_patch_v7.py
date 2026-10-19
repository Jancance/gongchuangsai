"""Build two code files against the directly captured V6 source hashes."""
import hashlib
import json
from pathlib import Path
import shutil
from zipfile import ZipFile,ZIP_DEFLATED

ROOT=Path(__file__).resolve().parents[1]
with ZipFile(ROOT/'evidence/20261006-183128-046231.zip') as evidence:
    hashes=json.loads(evidence.read('module_source_hashes.json'))
out=ROOT/'output/round_face_patch_v7_2026-10-06.zip'
manifest={'version':'round-face-candidate-v7-2026-10-06','hardware_acceptance':False,'files':{}}
with ZipFile(out,'w',ZIP_DEFLATED) as archive:
    for name in ('vision_app/code/project_detector.py','vision_app/code/project_tracker.py'):
        data=(ROOT/name).read_bytes()
        archive.writestr(name,data)
        manifest['files'][name]={'before_sha256':hashes[Path(name).name],'after_sha256':hashlib.sha256(data).hexdigest()}
    archive.writestr('PATCH_MANIFEST.json',json.dumps(manifest,indent=2))
shutil.copy2(ROOT/'scripts/apply_round_face_patch_v7.py',ROOT/'output/apply_round_face_patch_v7.py')
with ZipFile(out) as archive:assert archive.testzip() is None
print('Package:',out,'SHA256:',hashlib.sha256(out.read_bytes()).hexdigest())
