"""Exercise the actual delivered installer, including failure rollback."""
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from zipfile import ZipFile

ROOT=Path(__file__).resolve().parents[1]
FILES=('vision_app/code/project_detector.py','vision_app/code/config/material_color_thresholds.json')
with tempfile.TemporaryDirectory() as temporary:
    root=Path(temporary)
    shutil.copytree(ROOT/'vision_app',root/'vision_app',ignore=shutil.ignore_patterns('__pycache__'))
    for name in ('paired_face_patch_v8_2026-10-06.zip','apply_paired_face_patch_v8.py'):
        shutil.copy2(ROOT/'output'/name,root/name)
    with ZipFile(ROOT/'output/round_face_patch_v7_2026-10-06.zip') as archive:
        (root/FILES[0]).write_bytes(archive.read(FILES[0]))
    with ZipFile(ROOT/'evidence/20261006-193600-046816.zip') as archive:
        (root/FILES[1]).write_bytes(archive.read('material_color_thresholds.json'))
    before={name:(root/name).read_bytes() for name in FILES}
    untouched={p.relative_to(root):p.read_bytes() for p in (root/'vision_app/code/config').iterdir() if p.is_file() and p.name!='material_color_thresholds.json'}
    def run():
        return subprocess.run([sys.executable,str(root/'apply_paired_face_patch_v8.py')],capture_output=True,text=True,encoding='utf-8')
    assert run().returncode==0
    assert run().returncode==0
    for name,data in before.items():
        assert (root/'patch-backups/paired-face-v8-20261006'/name).read_bytes()==data
    for name,data in untouched.items():assert (root/name).read_bytes()==data
    old=json.loads(before[FILES[1]]);new=json.loads((root/FILES[1]).read_bytes())
    new.pop('round_face_color_candidate');assert old==new
    for name,data in before.items():(root/name).write_bytes(data)
    (root/'vision_app/run.py').write_text('raise SystemExit(42)\n',encoding='utf-8')
    assert run().returncode!=0
    for name,data in before.items():assert (root/name).read_bytes()==data
    (root/FILES[0]).write_bytes(before[FILES[0]]+b'\n# custom version\n')
    custom={name:(root/name).read_bytes() for name in FILES}
    assert run().returncode!=0
    for name,data in custom.items():assert (root/name).read_bytes()==data
print('PASS: V7 install, repeat install, exact backups, config preservation, failed-check rollback, custom source rejection')
