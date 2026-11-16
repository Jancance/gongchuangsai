from pathlib import Path
import hashlib,json,subprocess,os,sys,shutil
R=Path.cwd();DAY=next(R.glob('*/evidence/navigation-planning/2026-11-16'));P=next(R.glob('3.stm32f407*'));E=Path(__file__).resolve().parent
D=DAY/'lateral-forward-drift-correction';H=DAY/'wheel-enable-hold'
def sha(b):return hashlib.sha256(b).hexdigest()
def git(*args,input=None,env=None):
 p=subprocess.run(['git',*args],cwd=R,input=input,capture_output=True,env=env)
 if p.returncode:raise RuntimeError(p.stderr.decode('utf8','replace'))
 return p.stdout
assert not git('diff','--cached','--name-only').strip(),'Pre-existing staged changes; do not touch index'
base=json.loads((DAY/'lateral-field-run2/candidate_binding.json').read_text('utf8'))['input_sha256']
final=json.loads((D/'frozen_verification_manifest.json').read_text('utf8'))
hold=json.loads((H/'source_changes.json').read_text('utf8'))
selected={};overrides={};bad=[]
for n,h in base.items():
 n=n.replace('\\','/');p=R/n
 if not n.startswith(P.name+'/') or p.suffix.lower() not in ['.c','.h','.s','.uvprojx']:continue
 selected[n]=h
for n,h in final['source_sha256'].items():selected[(P/'CHASSIS/USER'/n).relative_to(R).as_posix()]=h
for n,h in hold['before'].items():
 k=(P/'CHASSIS/USER'/n).relative_to(R).as_posix();b=(H/'source_before'/n).read_bytes()
 assert sha(b)==h and selected[k]==h,(n,'hold before is not verified baseline')
 overrides[k]=b
for n,h in selected.items():
 b=overrides[n] if n in overrides else (R/n).read_bytes()
 if sha(b)!=h:bad.append(n)
assert not bad,('Validated source drift',bad)
image=Path(json.loads((D/'build_receipt.json').read_text('utf8'))['axf'])
assert sha(image.read_bytes())=='73b4fad295ac044af9002e93e65858d7bc87165543557cd034a7a758406224ce'
(E/'left210_trim4').mkdir(exist_ok=True)
shutil.copyfile(image,E/'left210_trim4/Vehicle_Integrated.axf')
build=json.loads((D/'build_receipt.json').read_text('utf8'));shutil.copyfile(Path(build['command'][2]),E/'left210_trim4.uvprojx')
report={'source_count':len(selected),'worktree_preserved':True,'temporary_index_overrides':list(overrides),'candidate_sha256':sha(image.read_bytes()),'image_size':image.stat().st_size,'head_before':git('rev-parse','HEAD').decode().strip(),'scope':'Previously validated chassis/fusion source dependencies, selected verification and field evidence, one exact tested candidate image; excludes unfinished enable-hold mode and unrelated worktree docs/options.'}
(E/'checkpoint_manifest.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
# Persist exact manifest. This preparatory script does not stage or commit.
(E/'validated_sources.json').write_text(json.dumps(selected,ensure_ascii=False,indent=2),encoding='utf8')
print(json.dumps(report,ensure_ascii=True,indent=2))