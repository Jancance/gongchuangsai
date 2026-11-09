from pathlib import Path
import subprocess,json,hashlib,sys,os,concurrent.futures
R=Path(__file__).resolve().parents[1];G=R/'tmp/gongchuangsai_remote_cache';D=R/'tmp/team_sync_emm_20261103'
D.mkdir(parents=True,exist_ok=True)
BASE='6dc294d6e69cbebd301ff92f27684b02d9f65604'
def git(*args,data=None):
    for i in range(3):
        q=subprocess.run(['git','-c','http.version=HTTP/1.1','-C',str(G),*args],input=data,capture_output=True,timeout=240)
        if q.returncode==0:return q.stdout
        if i==2:raise RuntimeError(q.stderr.decode(errors='replace')[-1400:])
def tree(ref):
    result={}
    for row in git('ls-tree','-rz',ref).split(b'\0'):
        if row:
            h,p=row.split(b'\t',1);mode,kind,oid=h.decode().split()
            if kind=='blob':result[p.decode('utf-8')]=(mode,oid)
    return result
def save(name,data):(D/name).write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')
def blobid(b):return hashlib.sha1(b'blob '+str(len(b)).encode()+b'\0'+b).hexdigest()
def localpath(p):return R/(p[len('工创赛/'):] if p.startswith('工创赛/') else p)
def write(p,b):p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(b)
if '--download' in sys.argv or '--remaining' in sys.argv:
    ref=git('rev-parse','origin/main').decode().strip();T=tree(ref)
    changed=[p.decode('utf-8') for p in git('diff','--no-renames','--name-only','-z',BASE,ref).split(b'\0') if p]
    if '--remaining' in sys.argv:selected=[p for p in changed if p in T and not (D/'remote-latest'/p).exists()]
    else:selected=[p for p in changed if p in T and '/evidence/' not in p]
    present=set(git('cat-file','--batch-all-objects','--batch-check=%(objectname)').decode().split())
    reuse=[];need={T[p][1] for p in selected}-present
    for p in selected:
        oid=T[p][1];local=localpath(p)
        if oid in need and local.is_file() and blobid(local.read_bytes())==oid:reuse.append(local.resolve().as_posix());need.remove(oid)
    if reuse:git('hash-object','--no-filters','-w','--stdin-paths',data=('\n'.join(reuse)+'\n').encode('utf-8'))
    print('Upstream',ref,'selected',len(selected),'reuse',len(reuse),'fetch blobs',len(need),flush=True)
    if need:
        git('fetch','--refetch','--filter=blob:none','--no-write-fetch-head','--no-auto-maintenance','--recurse-submodules=no','--no-tags','origin','--stdin',data=('\n'.join(sorted(need))+'\n').encode())
    proc=subprocess.Popen(['git','-C',str(G),'cat-file','--batch'],stdin=subprocess.PIPE,stdout=subprocess.PIPE)
    for p in selected:
        oid=T[p][1];proc.stdin.write((oid+'\n').encode());proc.stdin.flush();h=proc.stdout.readline().split()
        assert len(h)==3 and h[1]==b'blob',(p,h)
        b=proc.stdout.read(int(h[2]));assert proc.stdout.read(1)==b'\n' and blobid(b)==oid
        write(D/'remote-latest'/p,b)
    proc.stdin.close();assert proc.wait()==0
    save('remote-index.json',dict(base=BASE,remote=ref,changed=changed,tree=T))
    print('Downloaded and byte-verified',len(selected),'paths',flush=True)
if '--merge' in sys.argv:
    data=json.loads((D/'remote-index.json').read_text(encoding='utf-8'));T=data['tree'];B=tree(BASE)
    merged=[];conflicts=[];preserved=[]
    for p in data['changed']:
        upstream=D/'remote-latest'/p
        if p not in T:
            dst=localpath(p)
            if not dst.exists():merged.append(['already_removed',p]);continue
            local=dst.read_bytes();backup=D/'local-backup'/p
            if not backup.exists():write(backup,local)
            if p in B and blobid(local)==B[p][1]:
                assert dst.resolve().is_relative_to(R.resolve()) and p.startswith('3.stm32f407工程/USER/')
                dst.unlink();merged.append(['removed_obsolete_backed_up',p]);continue
            conflicts.append(p);continue
        if not upstream.is_file():continue
        if p.endswith('AGENTS.md'):preserved.append([p,'Current user-provided AGENTS takes precedence']);continue
        dst=localpath(p);remote=upstream.read_bytes()
        if not dst.exists():write(dst,remote);merged.append(['added',p]);continue
        local=dst.read_bytes()
        if local==remote:merged.append(['identical',p]);continue
        backup=D/'local-backup'/p
        if not backup.exists():write(backup,local)
        old=B.get(p)
        if old and blobid(local)==old[1]:write(dst,remote);merged.append(['updated',p]);continue
        if old and b'\0' not in local and b'\0' not in remote:
            oldbytes=git('cat-file','blob',old[1]);write(D/'merge-local',local);write(D/'merge-base',oldbytes);write(D/'merge-remote',remote)
            q=subprocess.run(['git','merge-file','-p',str(D/'merge-local'),str(D/'merge-base'),str(D/'merge-remote')],capture_output=True)
            if q.returncode==0:write(dst,q.stdout);merged.append(['merged',p]);continue
            write(D/'conflicts'/p,q.stdout)
        conflicts.append(p)
    save('integration-result.json',dict(base=BASE,remote=data['remote'],merged=merged,conflicts=conflicts,preserved=preserved,downloaded_changed_paths=sum((D/'remote-latest'/p).is_file() for p in data['changed']),total_changed_paths=len(data['changed'])))
    print('Integrated',len(merged),'paths; conflicts',len(conflicts),flush=True)
