"""Exercise delivered V11 install and rollback against actual V10 settings."""
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
    for name in ('independent_blue_patch_v11_2026-10-06.zip','apply_independent_blue_patch_v11.py'):shutil.copy2(ROOT/'output'/name,root/name)
    with ZipFile(ROOT/'output/dark_pale_patch_v10_2026-10-06.zip') as z:
        (root/FILES[0]).write_bytes(z.read(FILES[0]))
    with ZipFile(ROOT/'evidence/20261006-204150-736167.zip') as z:
        (root/FILES[1]).write_bytes(z.read('material_color_thresholds.json'))
    before={n:(root/n).read_bytes() for n in FILES}
    settings={p.relative_to(root):p.read_bytes() for p in (root/'vision_app/code/config').iterdir() if p.is_file() and p.name!='material_color_thresholds.json'}
    def run():return subprocess.run([sys.executable,str(root/'apply_independent_blue_patch_v11.py')],capture_output=True,text=True,encoding='utf-8')
    result=run();assert result.returncode==0,result.stderr
    assert run().returncode==0
    for n,data in before.items():assert (root/'patch-backups/independent-blue-v11-20261006'/n).read_bytes()==data
    for n,data in settings.items():assert (root/n).read_bytes()==data
    new=json.loads((root/FILES[1]).read_bytes())
    new['round_face_color_candidate'].pop('reject_unverified_blue_family')
    new['round_face_color_candidate'].pop('conflict_candidate_version')
    assert new==json.loads(before[FILES[1]])
    for n,data in before.items():(root/n).write_bytes(data)
    (root/'vision_app/run.py').write_text('raise SystemExit(42)\n',encoding='utf-8')
    assert run().returncode!=0
    for n,data in before.items():assert (root/n).read_bytes()==data
    (root/FILES[0]).write_bytes(before[FILES[0]]+b'\n# custom source\n')
    custom={n:(root/n).read_bytes() for n in FILES};assert run().returncode!=0
    for n,data in custom.items():assert (root/n).read_bytes()==data
print('PASS: V10 install/check, repeat install, exact backups, settings/ranges preserved, rollback and code guards')
