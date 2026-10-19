"""Package current dark-edge recovery against returned V11 board evidence."""
import hashlib
import json
from pathlib import Path
import shutil
from zipfile import ZipFile,ZIP_DEFLATED
from analyze_black_edge_v12 import candidate

ROOT=Path(__file__).resolve().parents[1]
with ZipFile(ROOT/'evidence/20261007-004517-208447.zip') as z:
    hashes=json.loads(z.read('module_source_hashes.json'))
    original=z.read('material_color_thresholds.json')
config=(json.dumps(candidate(json.loads(original)),ensure_ascii=False,indent=2)+'\n').encode('utf-8')
files={'vision_app/code/project_detector.py':((ROOT/'vision_app/code/project_detector.py').read_bytes(),hashes['project_detector.py']),
       'vision_app/code/config/material_color_thresholds.json':(config,hashlib.sha256(original).hexdigest())}
out=ROOT/'output/black_edge_patch_v12_2026-10-07.zip'
manifest={'version':'current-black-edge-ellipse-v12-20261007','hardware_acceptance':False,'files':{},
          'preserve_sha256':{'vision_app/code/project_follow.py':hashes['project_follow.py'],
                             'vision_app/code/project_tracker.py':hashes['project_tracker.py']}}
with ZipFile(out,'w',ZIP_DEFLATED) as z:
    for name,(data,digest) in files.items():
        z.writestr(name,data);manifest['files'][name]={'before_sha256':digest,'after_sha256':hashlib.sha256(data).hexdigest()}
    z.writestr('PATCH_MANIFEST.json',json.dumps(manifest,indent=2))
shutil.copy2(ROOT/'scripts/apply_black_edge_patch_v12.py',ROOT/'output/apply_black_edge_patch_v12.py')
with ZipFile(out) as z:assert z.testzip() is None
print(out,'SHA256',hashlib.sha256(out.read_bytes()).hexdigest())
