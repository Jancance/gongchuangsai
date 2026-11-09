from pathlib import Path
import sys,json,subprocess,os,re,hashlib
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'scripts'))
from sync_team_handoff_20261102 import git,tree,D,G
info=json.loads((D/'download-result.json').read_text(encoding='utf-8'));base=info['remote'];T=tree(base)
conflicts=set(info['conflicts'])
assert not conflicts,'Resolve upstream/local differences first: '+repr(sorted(conflicts))
candidates={};excluded=[]
for folder in ['docs','config','src','vision_app','scripts','tests','calibration','systemd','evidence']:
    for p in (R/folder).rglob('*'):
        if not p.is_file() or any(x in p.parts for x in ['__pycache__','.pytest_cache','.git']):continue
        rel=p.relative_to(R).as_posix();dest='工创赛/'+rel
        if '/p2-camera/' in rel and '/offline/' in rel and p.suffix.lower() in ['.png','.jpg','.jpeg']:
            excluded.append(dict(path=rel,bytes=p.stat().st_size,already_remote=dest in T,reason='Repeated offline diagnostic rendering retained locally; code, review and representative board images included'));continue
        if folder=='evidence' and ('/run-' in rel or p.name.startswith('sample-')) and p.suffix.lower() in ['.png','.jpg','.jpeg'] and p.name!='first-frame-raw.png':
            excluded.append(dict(path=rel,bytes=p.stat().st_size,already_remote=dest in T,reason='Repeated diagnostic sample image retained locally; review JSON and representative first frame included'));continue
        if p.suffix.lower() in ['.exe','.dll','.bin','.axf','.o','.pyc','.zip','.gz','.mp4','.avi','.mkv'] or p.stat().st_size>5_000_000:
            excluded.append(dict(path=rel,bytes=p.stat().st_size,already_remote=dest in T));continue
        candidates[dest]=p
for folder in ['handoff/p2_fixed_cycle_20261102']:
    for p in (R/folder).rglob('*'):
        if p.is_file():candidates['工创赛/'+p.relative_to(R).as_posix()]=p
for n in ['README.md','pyproject.toml']:
    candidates['工创赛/'+n]=R/n
candidates['P1电控组交接_2026-10-15.md']=R/'P1电控组交接_2026-10-15.md'
changed={};sus=[]
pattern=re.compile(rb'(?:gh[pousr]_[A-Za-z0-9]{30,}|github_pat_[A-Za-z0-9_]{30,}|sk-proj-[A-Za-z0-9_-]{20,}|-----BEGIN (?:RSA |OPENSSH |EC )?PRIVATE KEY-----)')
for dest,p in candidates.items():
    b=p.read_bytes();oid=hashlib.sha1(b'blob '+str(len(b)).encode()+b'\0'+b).hexdigest()
    if dest in T and T[dest][1]==oid:continue
    if pattern.search(b):sus.append(dest)
    changed[dest]=dict(local=str(p),oid=oid,bytes=len(b),mode=T.get(dest,('100644',''))[0])
assert not sus,'Credential-like content requires review: '+repr(sus)
report=dict(base=base,changes=changed,excluded=excluded,deleted_paths=[],remote_firmware_preserved=True,
            local_snapshot=info['snapshot'],downloaded=info['local_changes'])
(D/'upload-plan.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
print('Prepared',len(changed),'changed paths,',sum(v['bytes'] for v in changed.values()),'bytes;',len(excluded),'archives/artifacts omitted, remote originals retained')
if '--stage' not in sys.argv:sys.exit()
env=os.environ.copy();env['GIT_INDEX_FILE']=str((D/'upload.index').resolve())
git('read-tree',base,env=env)
records=[]
paths=[Path(item['local']).resolve().as_posix() for item in changed.values()]
oids=git('hash-object','--no-filters','-w','--stdin-paths',data=('\n'.join(paths)+'\n').encode('utf-8')).decode().split()
assert len(oids)==len(changed)
for (dest,item),oid in zip(changed.items(),oids):
    assert oid==item['oid'],dest
    records.append((item['mode']+' '+oid+'\t'+dest).encode('utf-8')+b'\0')
git('update-index','-z','--index-info',data=b''.join(records),env=env)
newtree=git('write-tree',env=env).decode().strip()
stat=git('-c','core.quotePath=false','diff','--stat','--no-renames',base,newtree).decode('utf-8')
(D/'diff-stat.txt').write_text(stat,encoding='utf-8')
q=subprocess.run(['git','-c','core.whitespace=cr-at-eol','-C',str(G),'diff','--check',base,newtree],capture_output=True)
(D/'diff-whitespace-review.txt').write_bytes(q.stdout+q.stderr)
assert q.returncode in (0,2),q.stderr
report['tree']=newtree;(D/'upload-plan.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
assert tree(newtree).get('3.stm32f407工程/USER/main.c')==T.get('3.stm32f407工程/USER/main.c')
print('Staged tree:',newtree,'remote root firmware unchanged')
