"""Build detector plus opt-in settings against the returned V7 board evidence."""
import hashlib
import json
from pathlib import Path
import shutil
from zipfile import ZipFile,ZIP_DEFLATED
from analyze_paired_color_v8 import candidate

ROOT=Path(__file__).resolve().parents[1]
with ZipFile(ROOT/'evidence/20261006-193600-046816.zip') as evidence:
    hashes=json.loads(evidence.read('module_source_hashes.json'))
    previous_config=evidence.read('material_color_thresholds.json')
config=(json.dumps(candidate(json.loads(previous_config)),ensure_ascii=False,indent=2)+'\n').encode('utf-8')
files={'vision_app/code/project_detector.py':((ROOT/'vision_app/code/project_detector.py').read_bytes(),hashes['project_detector.py']),
       'vision_app/code/config/material_color_thresholds.json':(config,hashlib.sha256(previous_config).hexdigest())}
out=ROOT/'output/paired_face_patch_v8_2026-10-06.zip'
manifest={'version':'paired-face-candidate-v8-20261006','hardware_acceptance':False,'files':{}}
with ZipFile(out,'w',ZIP_DEFLATED) as archive:
    for name,(data,before_hash) in files.items():
        archive.writestr(name,data)
        manifest['files'][name]={'before_sha256':before_hash,'after_sha256':hashlib.sha256(data).hexdigest()}
    archive.writestr('PATCH_MANIFEST.json',json.dumps(manifest,indent=2))
shutil.copy2(ROOT/'scripts/apply_paired_face_patch_v8.py',ROOT/'output/apply_paired_face_patch_v8.py')
with ZipFile(out) as archive:assert archive.testzip() is None
print(out,'SHA256',hashlib.sha256(out.read_bytes()).hexdigest())
