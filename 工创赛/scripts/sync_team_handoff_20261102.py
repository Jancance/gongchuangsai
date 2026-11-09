from pathlib import Path
import subprocess,os,json,hashlib,time,sys
R=Path(__file__).resolve().parents[1];G=R/'tmp/gongchuangsai_remote_cache';D=R/'tmp/team_sync_20261102'
D.mkdir(parents=True,exist_ok=True)
def git(*a,data=None,env=None):
    for attempt in range(3):
        q=subprocess.run(['git','-c','http.version=HTTP/1.1','-C',str(G),*a],input=data,capture_output=True,env=env)
        if q.returncode==0:return q.stdout
        if attempt==2:raise RuntimeError(q.stderr.decode(errors='replace')[-1600:])
def tree(ref):
    out={}
    for row in git('ls-tree','-rz',ref).split(b'\0'):
        if not row:continue
        head,p=row.split(b'\t',1);mode,kind,oid=head.decode().split()
        if kind=='blob':out[p.decode('utf-8')]=(mode,oid)
    return out
def write(p,b):p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(b)
if '--download' in sys.argv:
    ref=git('rev-parse','origin/main').decode().strip();T=tree(ref)
    present=set(git('cat-file','--batch-all-objects','--batch-check=%(objectname)').decode().split())
    missing=sorted({v[1] for v in T.values()}-present)
    print('Remote',ref,'paths',len(T),'missing blobs',len(missing),flush=True)
    reuse=[];needed=set(missing)
    for p,(_,oid) in T.items():
        if oid not in needed:continue
        rel=p[len('工创赛/'):] if p.startswith('工创赛/') else p
        candidates=[R/rel,R/'tmp/p1_team_upload_20261022'/p]
        for candidate in candidates:
            if not candidate.is_file():continue
            b=candidate.read_bytes()
            if hashlib.sha1(b'blob '+str(len(b)).encode()+b'\0'+b).hexdigest()==oid:
                reuse.append(candidate.resolve().as_posix());needed.remove(oid);break
    if reuse:
        git('hash-object','--no-filters','-w','--stdin-paths',data=('\n'.join(reuse)+'\n').encode('utf-8'))
    missing=sorted(needed)
    print('Reused local blobs',len(reuse),'download only',len(missing),flush=True)
    if missing:
        # Request missing latest-tree objects together instead of one TLS session per file.
        git('fetch','--no-filter','--no-write-fetch-head','--no-auto-maintenance','--no-tags','origin','--stdin',data=('\n'.join(missing)+'\n').encode())
    proc=subprocess.Popen(['git','-C',str(G),'cat-file','--batch'],stdin=subprocess.PIPE,stdout=subprocess.PIPE)
    blobs={}
    for oid in {v[1] for v in T.values()}:
        proc.stdin.write((oid+'\n').encode());proc.stdin.flush()
        header=proc.stdout.readline().split();assert len(header)==3 and header[1]==b'blob',header
        b=proc.stdout.read(int(header[2]));assert proc.stdout.read(1)==b'\n';blobs[oid]=b
    proc.stdin.close();assert proc.wait()==0
    for p,(_,oid) in T.items():write(D/'remote-latest'/p,blobs[oid])
    B=tree('dc5207f');old=tree('158e3df')
    changes=[];conflicts=[]
    for p,(mode,oid) in T.items():
        if p.startswith('工创赛/'):
            local=R/p[len('工创赛/'):]
        elif p.startswith('3.stm32f407工程/') or p.startswith('工具安装包/') or p=='P1电控组交接_2026-10-15.md':local=R/p
        else:continue
        b=blobs[oid]
        if not local.exists():write(local,b);changes.append(['added',p]);continue
        before=local.read_bytes()
        if before==b:continue
        # Only update an existing file when upstream changed it since the last known sync.
        base=B.get(p,old.get(p));baseoid=base[1] if base else None
        if oid==baseoid:continue
        write(D/'local-backup'/p,before)
        localoid=hashlib.sha1(b'blob '+str(len(before)).encode()+b'\0'+before).hexdigest()
        if localoid in {v[1] for v in (B.get(p),old.get(p)) if v}:
            write(local,b);changes.append(['updated',p]);continue
        if baseoid and b'\0' not in before and b'\0' not in b:
            basebytes=git('cat-file','blob',baseoid)
            x=D/'merge-local';y=D/'merge-base';z=D/'merge-remote'
            x.write_bytes(before);y.write_bytes(basebytes);z.write_bytes(b)
            q=subprocess.run(['git','merge-file','-p',str(x),str(y),str(z)],capture_output=True)
            if q.returncode==0:write(local,q.stdout);changes.append(['merged',p]);continue
        conflicts.append(p)
    result=dict(remote=ref,paths=len(T),missing_blobs_downloaded=len(missing),local_changes=changes,conflicts=conflicts,snapshot=str(D/'remote-latest'))
    (D/'download-result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print('Snapshot complete. Local updates',len(changes),'divergences retained',len(conflicts),flush=True)
