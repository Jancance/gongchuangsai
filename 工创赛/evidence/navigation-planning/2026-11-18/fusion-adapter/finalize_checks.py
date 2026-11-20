from pathlib import Path
import datetime, hashlib, json, subprocess, xml.etree.ElementTree as ET

R = Path.cwd()
E = Path(__file__).resolve().parent
U = R / '3.stm32f407工程/CHASSIS/USER'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
prep = json.loads((E / 'preparation.json').read_text('utf-8-sig'))
result = {'time': datetime.datetime.now().astimezone().isoformat(),
          'base_commit': prep['base_commit'].strip(), 'hardware_accessed': False,
          'runtime_caller_added': False, 'source_sha256': {}}
old = {}
for name, expected in prep['source_sha256'].items():
    if name.endswith('.uvprojx'):
        continue
    old[name] = sha(R / name) == expected
assert all(old.values()), old
result['protected_sources_unchanged'] = old
dirty = json.loads((R / '工创赛/evidence/navigation-planning/2026-11-17/turn90-90-prep/original_dirty_sha256.json').read_text('utf-8-sig'))
result['original_dirty_unchanged'] = {n: sha(R / n) == s for n, s in dirty.items()}
assert all(result['original_dirty_unchanged'].values())
for name in ['fusion_adapter.c', 'fusion_adapter.h']:
    p = U / name
    b = p.read_bytes()
    assert b.isascii() and b.count(b'\r\n') == b.count(b'\n')
    result['source_sha256'][name] = sha(p)
project = R / '3.stm32f407工程/gongxun-f407.uvprojx'
before = E / 'before/gongxun-f407.uvprojx'
b, a = before.read_bytes(), project.read_bytes()
assert sha(before) == prep['source_sha256']['3.stm32f407工程/gongxun-f407.uvprojx']
assert bytes(x for x in b if x >= 128) == bytes(x for x in a if x >= 128)
assert (b.count(b'\r\n'), b.count(b'\n') - b.count(b'\r\n'))[0] == 0
assert a.count(b'\r\n') == 0
for target in ET.fromstring(a).findall('./Targets/Target'):
    files = [f.text for f in target.findall('.//File/FileName')]
    for name in ['fusion_adapter.c', 'fusion_adapter.h', 'fusion_reference.c', 'fusion_reference.h']:
        assert files.count(name) == 1
result['project_lf_and_non_ascii_preserved'] = True
builds = json.loads((E / 'build_receipts.json').read_text('utf-8'))
for name, receipt in builds.items():
    assert receipt['success'] and receipt['compiled_reference']
    assert receipt['source_sha256'] == result['source_sha256']
    for filename, expected in receipt['frozen_sha256'].items():
        assert sha(E / 'frozen' / name / filename) == expected
result['builds'] = {name: {'success': v['success'], 'size_line': v['size_line']}
                    for name, v in builds.items()}
result['git_diff_check'] = subprocess.run(['git', '-c', 'core.whitespace=cr-at-eol', 'diff', '--check', '--', str(project)], capture_output=True, text=True).returncode == 0
assert result['git_diff_check']
(E / 'final_checks.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), 'utf-8')
print(json.dumps(result, ensure_ascii=False, indent=2))
