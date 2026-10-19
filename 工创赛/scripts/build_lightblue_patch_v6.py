"""Build the candidate from the exact measured board config, preserving other values."""
import hashlib
import json
from pathlib import Path
import shutil
from zipfile import ZipFile,ZIP_DEFLATED
from make_lightblue_color_candidate import make_candidate

ROOT=Path(__file__).resolve().parents[1]
with ZipFile(ROOT/'evidence/20261006-092130-937051.zip') as evidence:
    original=evidence.read('material_color_thresholds.json')
    hashes=json.loads(evidence.read('module_source_hashes.json'))
candidate=make_candidate(json.loads(original))
candidate_bytes=(json.dumps(candidate,ensure_ascii=False,indent=2)+'\n').encode('utf-8')
(ROOT/'output/lightblue_color_candidate_2026-10-06.json').write_bytes(candidate_bytes)
out=ROOT/'output/lightblue_patch_v6_2026-10-06.zip'
manifest={'version':'lightblue-candidate-v6-2026-10-06','hardware_acceptance':False,'files':{}}
entries=[('vision_app/code/project_detector.py',(ROOT/'vision_app/code/project_detector.py').read_bytes(),hashes['project_detector.py']),
         ('vision_app/code/config/material_color_thresholds.json',candidate_bytes,hashlib.sha256(original).hexdigest())]
with ZipFile(out,'w',ZIP_DEFLATED) as archive:
    for name,data,before in entries:
        archive.writestr(name,data)
        manifest['files'][name]={'before_sha256':before,'after_sha256':hashlib.sha256(data).hexdigest()}
    archive.writestr('PATCH_MANIFEST.json',json.dumps(manifest,indent=2))
shutil.copy2(ROOT/'scripts/apply_lightblue_patch_v6.py',ROOT/'output/apply_lightblue_patch_v6.py')
with ZipFile(out) as archive:assert archive.testzip() is None
print('Package:',out,'SHA256:',hashlib.sha256(out.read_bytes()).hexdigest())
