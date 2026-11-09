"""Archive and verify explicitly scoped old projects; does not delete anything."""
from pathlib import Path
import hashlib
import json
import xml.etree.ElementTree as ET
import zipfile

ROOT = Path('D:/新建文件夹4/工创赛/代码/stm32').resolve(strict=True)
KEEP = ROOT / '3.stm32f407工程'
OUT = Path(__file__).resolve().parents[1] / 'output/stm32-cleanup-20261105'
OUT.mkdir(parents=True, exist_ok=True)
ARCHIVE = OUT / 'old-projects-before-cleanup.zip'
assert not ARCHIVE.exists(), 'Refusing to replace an existing archive'
project = KEEP / 'gongxun-f407.uvprojx'
targets = []
for target in ET.parse(project).findall('.//Target'):
    paths = [(project.parent / p.text.replace('\\', '/')).resolve(strict=True)
             for p in target.findall('.//FilePath')]
    assert all(p.is_file() and p.is_relative_to(KEEP) for p in paths)
    targets.append(dict(name=target.findtext('TargetName'), source_files=len(paths)))

remove_names = ['1.270°舵机旋转', '2.串口屏显示二维码',
                '3.stm32f407工程_bak_20261104_1845', 'stm32f407工程', '_ed',
                'b1.log', 'b2.log', 'b3.log', 'b4.log', 'b5.log', 'b6.log', 'b7.log',
                'build_baseline.log', 'flash.log', 'rebuild.log']
assert {p.name for p in ROOT.iterdir()} == set(remove_names) | {KEEP.name}
entries = []
for name in remove_names:
    item = ROOT / name
    assert not item.is_symlink() and item.resolve(strict=True).parent == ROOT
    for p in sorted(item.rglob('*')) if item.is_dir() else [item]:
        assert not p.is_symlink(), f'Refusing link: {p}'
        if p.is_file():
            assert p.resolve().is_relative_to(ROOT)
            entries.append(dict(path=p.relative_to(ROOT).as_posix(), size=p.stat().st_size,
                                sha256=hashlib.sha256(p.read_bytes()).hexdigest()))
current = {p.relative_to(KEEP).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
           for p in KEEP.rglob('*') if p.is_file()}
with zipfile.ZipFile(ARCHIVE, 'x', compression=zipfile.ZIP_DEFLATED, compresslevel=3) as z:
    for item in entries:
        z.write(ROOT / item['path'], item['path'])
with zipfile.ZipFile(ARCHIVE) as z:
    assert z.testzip() is None
    assert len(z.infolist()) == len(entries)
    for item in entries:
        data = z.read(item['path'])
        assert len(data) == item['size']
        assert hashlib.sha256(data).hexdigest() == item['sha256']
report = dict(root=str(ROOT), keep=str(KEEP), project=str(project), targets=targets,
              archive=str(ARCHIVE), archive_sha256=hashlib.sha256(ARCHIVE.read_bytes()).hexdigest(),
              archive_verified=True, remove_names=remove_names, archived_files=entries,
              kept_files_sha256=current, deleted=False, firmware_flashed=False)
(OUT / 'cleanup-manifest.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n',
                                          encoding='utf-8')
print(json.dumps(dict(archive=str(ARCHIVE), archive_bytes=ARCHIVE.stat().st_size,
                     archived_files=len(entries), kept_files=len(current), targets=targets,
                     deleted=False), ensure_ascii=False))
