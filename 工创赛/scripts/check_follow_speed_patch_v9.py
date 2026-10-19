"""Check the delivered code-only patch against actual returned V8 settings."""
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from zipfile import ZipFile

ROOT=Path(__file__).resolve().parents[1]
FILES=('vision_app/code/project_detector.py','vision_app/code/project_follow.py')
with tempfile.TemporaryDirectory() as temporary:
    root=Path(temporary)
    shutil.copytree(ROOT/'vision_app',root/'vision_app',ignore=shutil.ignore_patterns('__pycache__'))
    for name in ('follow_speed_patch_v9_2026-10-06.zip','apply_follow_speed_patch_v9.py'):
        shutil.copy2(ROOT/'output'/name,root/name)
    with ZipFile(ROOT/'output/paired_face_patch_v8_2026-10-06.zip') as archive:
        (root/FILES[0]).write_bytes(archive.read(FILES[0]))
    # Following was unchanged after V4; use its immutable package, not current V9 source.
    with ZipFile(ROOT/'output/follow_touch_patch_v4_2026-10-06.zip') as archive:
        (root/FILES[1]).write_bytes(archive.read(FILES[1]))
    with ZipFile(ROOT/'evidence/20261006-195642-102168.zip') as archive:
        (root/'vision_app/code/config/material_color_thresholds.json').write_bytes(archive.read('material_color_thresholds.json'))
    before={name:(root/name).read_bytes() for name in FILES}
    settings={p.relative_to(root):p.read_bytes() for p in (root/'vision_app/code/config').iterdir() if p.is_file()}
    def run():return subprocess.run([sys.executable,str(root/'apply_follow_speed_patch_v9.py')],capture_output=True,text=True,encoding='utf-8')
    installed=run()
    assert installed.returncode==0,installed.stderr
    assert run().returncode==0
    for name,data in before.items():assert (root/'patch-backups/follow-speed-v9-20261006'/name).read_bytes()==data
    for name,data in settings.items():assert (root/name).read_bytes()==data
    for name,data in before.items():(root/name).write_bytes(data)
    (root/'vision_app/run.py').write_text('raise SystemExit(42)\n',encoding='utf-8')
    assert run().returncode!=0
    for name,data in before.items():assert (root/name).read_bytes()==data
    (root/FILES[0]).write_bytes(before[FILES[0]]+b'\n# custom code\n')
    custom={name:(root/name).read_bytes() for name in FILES}
    assert run().returncode!=0
    for name,data in custom.items():assert (root/name).read_bytes()==data
print('PASS: V8 install/check, repeat install, exact backups, byte-identical settings, rollback, custom code rejection')
