from pathlib import Path
import hashlib,json,subprocess,os,zipfile
R=Path.cwd();E=Path(__file__).resolve().parent;DAY=next(R.glob('*/evidence/navigation-planning/2026-11-16'));P=next(R.glob('3.stm32f407*'));H=DAY/'wheel-enable-hold'
def run(args,*,input=None,env=None):
 p=subprocess.run(['git',*args],cwd=R,input=input,capture_output=True,env=env)
 if p.returncode:raise RuntimeError(p.stderr.decode('utf8','replace'))
 return p.stdout
sources=json.loads((E/'validated_sources.json').read_text('utf8'));old=json.loads((H/'source_changes.json').read_text('utf8'))['before']
overrides={(P/'CHASSIS/USER'/n).relative_to(R).as_posix():(H/'source_before'/n).read_bytes() for n in old}
def data(n):return overrides[n] if n in overrides else (R/n).read_bytes()
for n,h in sources.items():assert hashlib.sha256(data(n)).hexdigest()==h,n
with zipfile.ZipFile(E/'validated_sources.zip','w',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as z:
 for n in sorted(sources):
  info=zipfile.ZipInfo(n,date_time=(2026,11,16,0,0,0));info.compress_type=zipfile.ZIP_DEFLATED;z.writestr(info,data(n))
# Seed a separate index and stage only existing meaningful source changes/new inputs.
index=E/'checkpoint.index';assert not index.exists(),'Do not blindly rerun a partially staged checkpoint'
env=os.environ.copy();env['GIT_INDEX_FILE']=str(index)
head=run(['rev-parse','HEAD']).decode().strip();assert head==json.loads((E/'checkpoint_manifest.json').read_text('utf8'))['head_before']
assert not run(['diff','--cached','--name-only']).strip()
run(['read-tree','HEAD'],env=env)
head_paths=set(x.decode('utf8') for x in run(['ls-tree','-r','--name-only','-z','HEAD']).split(b'\0') if x)
modified=set(x.decode('utf8') for x in run(['diff','--name-only','-z','HEAD']).split(b'\0') if x)
stage={n:data(n) for n in sources if n not in head_paths or n in modified}
# Key complete observations and the exact candidate verification dossier.
names=['inertial-field-run1','inertial-field-run2','inertial-field-run3-retry1','inertial-three-run-analysis','lateral-field-run1','lateral-field-run2','lateral-drift-trim-run1','lateral-drift-trim-run2','lateral-forward-drift-correction','lateral-distance-diagnosis']
exts={'.md','.json','.csv','.m','.py','.log','.txt','.bin','.raw','.jsonl','.png','.c','.h'}
for name in names:
 for p in (DAY/name).rglob('*'):
  if p.is_file() and p.suffix.lower() in exts and '__pycache__' not in p.parts:stage[p.relative_to(R).as_posix()]=p.read_bytes()
for p in E.rglob('*'):
 if p.is_file() and p.suffix.lower() in {'.md','.json','.zip','.uvprojx','.axf','.py'}:stage[p.relative_to(R).as_posix()]=p.read_bytes()
updates=[]
for n,b in sorted(stage.items()):
 oid=run(['hash-object','-w','--stdin','--path='+n],input=b).decode().strip()
 updates.append(b'100644 '+oid.encode()+b'\t'+n.encode('utf8')+b'\0')
run(['update-index','-z','--index-info'],input=b''.join(updates),env=env)
# No unfinished enable-hold token can enter the saved product baseline.
for n,b in overrides.items():
 saved=run(['show',':'+n],env=env)
 assert saved.replace(b'\r\n',b'\n')==b.replace(b'\r\n',b'\n'),n
 assert b'APP_WHEEL_ENABLE_HOLD_ENABLE' not in saved,n
check=subprocess.run(['git','-c','core.whitespace=cr-at-eol','diff','--cached','--check'],cwd=R,capture_output=True,env=env)
(E/'staged_diff_check.txt').write_bytes(check.stdout+check.stderr)
# Legacy/test-generated text may have whitespace; report it for review before commit.
stat=run(['diff','--cached','--stat'],env=env).decode('utf8','replace');(E/'staged_stat.txt').write_text(stat,encoding='utf8')
report={'head_before':head,'staged_count':len(run(['diff','--cached','--name-only','-z'],env=env).split(b'\0'))-1,'source_manifest_count':len(sources),'staged_source_candidates':sum(n in sources for n in stage),'working_tree_untouched':True,'unfinished_enable_excluded':True,'diff_check_returncode':check.returncode,'source_archive_sha256':hashlib.sha256((E/'validated_sources.zip').read_bytes()).hexdigest(),'alternate_index':str(index)}
(E/'stage_receipt.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
print(json.dumps(report,ensure_ascii=True,indent=2));print(stat[-5000:])