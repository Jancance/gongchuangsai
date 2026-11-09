"""安装已构建/桩检查的4310单项5文件；不下载、不复位、不访问串口。"""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess

R = Path(__file__).resolve().parents[1]
E = R / 'evidence/integration/2026-11-03/dm-startup-only'
S = R / 'tmp/dm_startup_only_20261103'
A = Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
M = E / 'manifest.json'
data = json.loads(M.read_text(encoding='utf-8'))
changed = list(data['original'])


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def sources(root):
    return {p.relative_to(root).as_posix(): sha(p) for p in root.rglob('*')
            if p.is_file() and p.suffix.lower() in {'.c', '.h', '.s', '.inc', '.uvprojx'}
            and not {'Objects', 'Listings', '.git'}.intersection(p.relative_to(root).parts)}


for name in ['dm-only-check.json', 'plan-check.json', 'mission-check.json']:
    result = json.loads((E / name).read_text(encoding='utf-8'))
    assert result.get('passed', all(x['passed'] for x in result.get('cases', []))), name
for target in ['Vehicle_Integrated', 'Vehicle_Offline']:
    assert '0 Error(s), 0 Warning(s)' in (E / f'stage-{target}.log').read_text(encoding='utf-8', errors='replace')
before = sources(A)
for rel, expected in data['original'].items():
    assert before[rel] == expected, ('Actual file changed since intake', rel)
for rel in changed:
    shutil.copyfile(S / rel, A / rel)
after = sources(A)
protected = {rel: h for rel, h in before.items() if rel not in changed}
assert all(after[rel] == h for rel, h in protected.items())
assert all(sha(A / rel) == sha(S / rel) for rel in changed)
data.update(installed=True, changed={rel: sha(A / rel) for rel in changed},
            protected_source_files_verified=len(protected), hardware_accessed=False, flashed=False)
M.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')
print('Installed', len(changed), 'files; preserved', len(protected), 'other source/project files', flush=True)
builds = []
for target in ['Vehicle_Integrated', 'Vehicle_Offline']:
    log = E / f'actual-{target}.log'
    p = subprocess.run(['C:/Keil_v5/UV4/UV4.exe', '-r', str(A / 'gongxun-f407.uvprojx'),
                        '-t', target, '-j0', '-o', str(log)], capture_output=True, timeout=180)
    text = log.read_text(encoding='utf-8', errors='replace')
    passed = p.returncode == 0 and '0 Error(s), 0 Warning(s)' in text
    builds.append(dict(target=target, passed=passed, returncode=p.returncode,
                       log=log.name, program_size=next((s for s in text.splitlines() if 'Program Size:' in s), '')))
    data['actual_builds'] = builds
    M.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')
    print(target, p.returncode, text[-450:], flush=True)
    assert passed, text[-2500:]
assert all(sha(A / rel) == data['changed'][rel] for rel in changed)
print('Actual build and source hashes verified; no download or hardware commands', flush=True)
