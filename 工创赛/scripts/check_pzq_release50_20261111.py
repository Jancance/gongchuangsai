"""Test exact pzq owner49/50ms boundaries and source-only rollback scope."""
from pathlib import Path
import hashlib
import json
import subprocess

R=Path(__file__).resolve().parents[1]
P=R/'tmp/pzq_release50_20261111/project'
E=R/'evidence/integration/2026-11-11/pzq-release50'
B=R/'handoff/pzq/project'
source=(R/'scripts/check_integration13_immediate_20261109.py').read_text(encoding='utf-8')
source=source[:source.index("src=OUT/")]
ns={'__file__':str(R/'scripts/check_integration13_immediate_20261109.py')}
exec(source,ns)
unit=ns['unit'].replace('now+2199','now+2049').replace('now+2200','now+2050').replace('waits199/200ms','waits49/50ms')
src=E/'owner-release50.c';exe=E/'owner-release50.exe';src.write_text(unit,encoding='utf-8')
proc=subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-I',str(P/'CHASSIS/USER'),str(src),'-o',str(exe)],capture_output=True,text=True)
log=proc.stdout+proc.stderr
if proc.returncode==0:
    proc=subprocess.run([str(exe)],capture_output=True,text=True,timeout=15);log+=proc.stdout+proc.stderr
(E/'owner-release50.log').write_text(log,encoding='utf-8');print(log,flush=True);assert proc.returncode==0
changed=[]
for p in P.rglob('*'):
    if not p.is_file() or any(x in ('Objects','Listings') for x in p.relative_to(P).parts):continue
    if p.read_bytes()!=(B/p.relative_to(P)).read_bytes():changed.append(p.relative_to(P).as_posix())
assert changed==['CHASSIS/USER/integration13_immediate.inc'],changed
old=(B/changed[0]).read_bytes();new=(P/changed[0]).read_bytes()
assert new==old.replace(b'if(g_it_auto_phase_ms<200U)return;',b'if(g_it_auto_phase_ms<50U)return;',1).replace(b'before200ms.',b'before50ms.',1)
manifest=json.loads((R/'handoff/pzq/source-manifest.json').read_text(encoding='utf-8'))
for rel,h in manifest['files'].items():assert hashlib.sha256((R/'handoff/pzq'/rel).read_bytes()).hexdigest()==h
(E/'checks.json').write_text(json.dumps(dict(passed=True,actual_owner_three_cycles=True,boundaries_ms=[49,50],original_pzq_files_verified=len(manifest['files']),only_release_interval_changed=True,hardware_access=False),indent=2)+'\n',encoding='utf-8')
print('Only the release interval changed. Frozen pzq source and original motion backend verified.')
