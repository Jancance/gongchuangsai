"""Verify delivered installer, settings preservation, repeat install and rollback."""
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
    for name in ('black_edge_patch_v12_2026-10-07.zip','apply_black_edge_patch_v12.py'):shutil.copy2(ROOT/'output'/name,root/name)
    with ZipFile(ROOT/'output/independent_blue_patch_v11_2026-10-06.zip') as z:
        (root/FILES[0]).write_bytes(z.read(FILES[0]))
    with ZipFile(ROOT/'evidence/20261007-004517-208447.zip') as z:
        (root/FILES[1]).write_bytes(z.read('material_color_thresholds.json'))
    before={n:(root/n).read_bytes() for n in FILES}
    settings={p.relative_to(root):p.read_bytes() for p in (root/'vision_app/code/config').iterdir() if p.is_file() and p.name!='material_color_thresholds.json'}
    def run():return subprocess.run([sys.executable,str(root/'apply_black_edge_patch_v12.py')],capture_output=True,text=True,encoding='utf-8')
    result=run();assert result.returncode==0,result.stderr
    assert run().returncode==0
    for n,data in before.items():assert (root/'patch-backups/black-edge-v12-20261007'/n).read_bytes()==data
    for n,data in settings.items():assert (root/n).read_bytes()==data
    new=json.loads((root/FILES[1]).read_bytes());new.pop('black_edge_recovery_candidate')
    assert new==json.loads(before[FILES[1]])
    for n,data in before.items():(root/n).write_bytes(data)
    (root/'vision_app/run.py').write_text('raise SystemExit(42)\n',encoding='utf-8')
    assert run().returncode!=0
    for n,data in before.items():assert (root/n).read_bytes()==data
    (root/FILES[0]).write_bytes(before[FILES[0]]+b'\n# custom source\n')
    custom={n:(root/n).read_bytes() for n in FILES};assert run().returncode!=0
    for n,data in custom.items():assert (root/n).read_bytes()==data
print('PASS: V11 install/check, repeat install, backups, settings/ranges unchanged, rollback and code guards')
