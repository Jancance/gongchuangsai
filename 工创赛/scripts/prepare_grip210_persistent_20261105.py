"""准备原料区210度候选，不操作实际工程或硬件。"""
from pathlib import Path
import hashlib
import json
import shutil

ROOT = Path(__file__).resolve().parents[1]
ACTUAL = Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
STAGE = ROOT / 'tmp/source_grip210_persistent_20261105/project'
OUT = ROOT / 'evidence/integration/2026-11-05/source-grip210-persistent'
FILES = ['USER/vehicle_parameters.h', 'CHASSIS/USER/source_prepare.inc']


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def sources(root):
    return {p.relative_to(root).as_posix(): sha(p) for p in root.rglob('*')
            if p.is_file() and p.suffix.lower() in {'.c', '.h', '.inc', '.s', '.uvprojx'}
            and not {'Objects', 'Listings', '.git'}.intersection(p.relative_to(root).parts)}


if __name__ == '__main__':
    OUT.mkdir(parents=True, exist_ok=True)
    assert not (OUT / 'manifest.json').exists(), '候选已存在，避免覆盖基线'
    baseline = sources(ACTUAL)
    shutil.copytree(ACTUAL, STAGE, ignore=shutil.ignore_patterns('Objects', 'Listings', '.git', '.vscode'))
    for name in FILES:
        backup = OUT / 'before' / name
        backup.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ACTUAL / name, backup)
    changes = {
        FILES[0]: (b'#define VEHICLE_SOURCE_PREP_GRIP_DEG 180.0f', b'#define VEHICLE_SOURCE_PREP_GRIP_DEG 210.0f'),
        FILES[1]: (b'VEHICLE_SOURCE_PREP_GRIP_DEG<=180 &&', b'VEHICLE_SOURCE_PREP_GRIP_DEG<=210 &&')}
    for name, (old, new) in changes.items():
        p = STAGE / name
        data = p.read_bytes()
        assert data.count(old) == 1, name
        p.write_bytes(data.replace(old, new))
    after = sources(STAGE)
    assert {n for n in baseline if baseline[n] != after[n]} == set(FILES)
    manifest = {'actual': str(ACTUAL), 'stage': str(STAGE), 'baseline': baseline,
                'changed': {n: after[n] for n in FILES}, 'requested_source_grip_deg': 210,
                'boot_fold_grip_deg_unchanged': 145, 'installed': False, 'flashed': False}
    (OUT / 'manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    print(f'Staged 2 changes; {len(baseline)-2} other source/project files preserved')
