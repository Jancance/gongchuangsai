"""Package the two-file blue candidate against the recorded board hashes."""
import hashlib
import json
from pathlib import Path
import shutil
from zipfile import ZipFile,ZIP_DEFLATED

ROOT=Path(__file__).resolve().parents[1]
out=ROOT/'output/blue_highlight_patch_v5_2026-10-06.zip'
with ZipFile(ROOT/'evidence/20261006-085047-455357.zip') as evidence:
    hashes=json.loads(evidence.read('module_source_hashes.json'))
with ZipFile(ROOT/'output/follow_touch_patch_v4_2026-10-06.zip') as v4:
    assert hashes['project_detector.py']==hashlib.sha256(v4.read('vision_app/code/project_detector.py')).hexdigest()
manifest={'version':'blue-highlight-candidate-v5-2026-10-06','hardware_acceptance':False,'files':{}}
with ZipFile(out,'w',ZIP_DEFLATED) as archive:
    for name,source,key in [('vision_app/code/project_detector.py',ROOT/'vision_app/code/project_detector.py','project_detector.py'),
                            ('p2_color_follow_audit.py',ROOT/'scripts/p2_color_follow_audit.py','diagnostic_entry')]:
        data=source.read_bytes()
        manifest['files'][name]={'before_sha256':hashes[key],'after_sha256':hashlib.sha256(data).hexdigest()}
        archive.writestr(name,data)
    archive.writestr('PATCH_MANIFEST.json',json.dumps(manifest,indent=2))
shutil.copy2(ROOT/'scripts/apply_blue_highlight_patch_v5.py',ROOT/'output/apply_blue_highlight_patch_v5.py')
with ZipFile(out) as archive:assert archive.testzip() is None
print('Package:',out,'SHA256:',hashlib.sha256(out.read_bytes()).hexdigest())
