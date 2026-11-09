"""Install only validated stage2 files, preserve other actual source, rebuild."""
from pathlib import Path
import hashlib,json,subprocess
R=Path(__file__).resolve().parents[1];E=R/'evidence/integration/2026-11-03/stage2-grid-route';V=R/'evidence/integration/2026-11-04/stage2-speed'
M=E/'manifest.json';m=json.loads(M.read_text(encoding='utf-8'));A=Path(m['actual']);S=Path(m['stage'])
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def sources(root):
 return {p.relative_to(root).as_posix():sha(p) for p in root.rglob('*') if p.is_file() and p.suffix.lower() in {'.c','.h','.s','.inc','.uvprojx'} and not {'Objects','Listings','.git','ObjectsOffline'}.intersection(p.relative_to(root).parts)}
assert not m['installed']
for name in ['c-model.json','backend-dynamic.json','matlab-result.json']:
 assert json.loads((V/name).read_text(encoding='utf-8'))['passed'],name
assert json.loads((E/'contract-check.json').read_text(encoding='utf-8'))['passed']
for target in ['Vehicle_Integrated','Vehicle_Offline']:
 assert '0 Error(s), 0 Warning(s)' in (E/f'stage-{target}.log').read_text(encoding='utf-8',errors='replace')
before=sources(A)
for rel,original in m['original'].items():
 if original is None:assert not (A/rel).exists(),rel
 else:assert before[rel]==original,('actual source changed',rel)
 assert sha(S/rel)==m['changed'][rel],('stage hash changed',rel)
for rel in m['changed']:(A/rel).write_bytes((S/rel).read_bytes())
after=sources(A);protected={k:v for k,v in before.items() if k not in m['changed']}
assert all(after[k]==v for k,v in protected.items())
m.update(installed=True,protected_source_files_verified=len(protected))
M.write_text(json.dumps(m,ensure_ascii=False,indent=2),encoding='utf-8')
print('Installed',len(m['changed']),'files; preserved',len(protected),flush=True)
for target in ['Vehicle_Integrated','Vehicle_Offline']:
 log=E/f'actual-{target}.log'
 p=subprocess.run(['C:/Keil_v5/UV4/UV4.exe','-r',str(A/'gongxun-f407.uvprojx'),'-t',target,'-j0','-o',str(log)],timeout=180)
 assert p.returncode==0 and '0 Error(s), 0 Warning(s)' in log.read_text(encoding='utf-8',errors='replace')
 print(target,'0 errors, 0 warnings',flush=True)
m['actual_builds_passed']=True;M.write_text(json.dumps(m,ensure_ascii=False,indent=2),encoding='utf-8')
