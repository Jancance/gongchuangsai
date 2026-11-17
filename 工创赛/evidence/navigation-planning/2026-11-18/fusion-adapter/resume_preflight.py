from pathlib import Path
import datetime
import hashlib
import json
import subprocess

R = Path.cwd()
E = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
prep = json.loads((E / 'preparation.json').read_text('utf-8-sig'))
dirty = json.loads((E.parent.parent / '2026-11-17/turn90-90-prep/original_dirty_sha256.json').read_text('utf-8-sig'))
sources = {n: sha(R / n) == s for n, s in prep['source_sha256'].items()}
unchanged = {n: sha(R / n) == s for n, s in dirty.items()}
assert all(sources.values()) and all(unchanged.values())
head = subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip()
assert head.startswith('70b230c4')
untracked = subprocess.check_output(['git', 'ls-files', '--others', '--exclude-standard', '-z'])
result = {
    'time': datetime.datetime.now().astimezone().isoformat(),
    'head': head,
    'branch': subprocess.check_output(['git', 'branch', '--show-current'], text=True).strip(),
    'protected_sources_unchanged': sources,
    'original_dirty_unchanged': unchanged,
    'untracked_count': len([x for x in untracked.split(b'\0') if x]),
    'untracked_listing_sha256': hashlib.sha256(untracked).hexdigest(),
    'original_dirty_diff_sha256': hashlib.sha256(subprocess.check_output(['git', 'diff', '--', *dirty])).hexdigest(),
    'adapter_product_exists': {n: (R / '3.stm32f407\u5de5\u7a0b/CHASSIS/USER' / n).exists() for n in ['fusion_adapter.c', 'fusion_adapter.h']},
    'hardware_accessed': False,
    'fetch_push_permitted': False,
    'hardware_paused': True,
}
assert not any(result['adapter_product_exists'].values())
p = E / 'resume_preflight.json'
assert not p.exists()
p.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
print('resume_preflight saved; protected sources and original dirty match; no adapter products')
