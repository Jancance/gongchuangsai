from pathlib import Path
import sys,os,json,hashlib,subprocess,xml.etree.ElementTree as ET,shutil,re
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'scripts'))
from sync_team_handoff_20261102 import git,tree,D,G
base=json.loads((D/'download-result.json').read_text(encoding='utf-8'))['remote'];T=tree(base)
F=R/'handoff/p2_fixed_cycle_20261102/firmware';project=F/'gongxun-f407.uvprojx';xml=ET.parse(project)
selected={project}
for e in xml.findall('.//FilePath'):
    p=(F/e.text.replace('\\','/')).resolve();assert p.is_file();selected.add(p)
for e in xml.findall('.//IncludePath'):
    for part in (e.text or '').split(';'):
        if part:
            folder=F/part.replace('\\','/')
            selected.update(p.resolve() for p in folder.glob('*.h'))
S=R/'tmp/team_sync_20261102/scoped-firmware'
assert not S.exists()
for p in selected:
    dst=S/p.relative_to(F.resolve());dst.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(p,dst)
log=D/'scoped-firmware-build.log'
q=subprocess.run(['C:/Keil_v5/UV4/UV4.exe','-b',str(S/'gongxun-f407.uvprojx'),'-j0','-o',str(log)],timeout=120)
text=log.read_text(encoding='utf-8',errors='replace');print(text[-700:],flush=True)
assert q.returncode==0 and '0 Error(s), 0 Warning(s)' in text
out=R/'evidence/team-sync/2026-11-02';shutil.copyfile(log,out/'scoped-firmware-build.log')
manifest=R/'handoff/p2_fixed_cycle_20261102/manifest.json'
shutil.copyfile(manifest,D/'full-local-firmware-manifest.json')
manifest.write_text(json.dumps(dict(date='2026-11-02',entry='firmware/gongxun-f407.uvprojx',source='D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程',files={p.relative_to(F.resolve()).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(selected)},scope='仅Keil实际引用的源文件、包含目录头文件及工程文件；本地未引用第三方历史文件不上传',build_pass=True,close_deg=135,source_down=105000,source_up=104000,carrier_down=28000,carrier_up=27000,vehicle_integration_passed=False),ensure_ascii=False,indent=2),encoding='utf-8')
candidates={'工创赛/'+p.relative_to(R.resolve()).as_posix():p for p in selected}
for p in (R/'handoff/p2_fixed_cycle_20261102').glob('*'):
    if p.is_file():candidates['工创赛/'+p.relative_to(R).as_posix()]=p
docs=['P2_HANDOFF_CURRENT_2026-11-02.md','P2_HANDOFF_CURRENT_2026-10-31.md','P2_HANDOFF_AND_EIGHT_SEGMENTS_2026-10-31.md','P2_ARM_PARAMETERS_2026-11-02.md','P2_ONE_START_MANUAL_2026-11-02.md','competition_requirements_2027.md','TEAM_SYNC_2026-11-02.md','P1_CURRENT_NEXT_PLAN_2026-10-07.md','archive/P1_LOCAL_HANDOFF_BEFORE_SYNC_2026-11-02.md','archive/P1_LOCAL_PLAN_BEFORE_SYNC_2026-11-02.md']
for n in docs:candidates['工创赛/docs/'+n]=R/'docs'/n
candidates['P1电控组交接_2026-10-15.md']=R/'P1电控组交接_2026-10-15.md'
for folder in ['evidence/p2-arm/2026-11-02/close-135','evidence/p2-arm/2026-11-02/parameters-145-104000-27000','evidence/p2-arm/2026-11-02/fixed-cycle-accepted','evidence/team-sync/2026-11-02']:
    for p in (R/folder).glob('*'):
        if p.is_file():candidates['工创赛/'+p.relative_to(R).as_posix()]=p
for name in ['adjust_arm_parameters_20261102.py','prepare_arm_smooth_home_20261102.py','check_arm_parameters_20261102.py','check_arm_smooth_home_20261102.py','set_arm_close_135_20261102.py']:
    candidates['工创赛/scripts/'+name]=R/'scripts'/name
changes={}
for dest,p in candidates.items():
    b=p.read_bytes();oid=hashlib.sha1(b'blob '+str(len(b)).encode()+b'\0'+b).hexdigest()
    if dest in T and T[dest][1]==oid:continue
    assert not re.search(rb'(?m)^(?:<<<<<<< |>>>>>>> |=======$)',b),dest
    assert not re.search(rb'(?:gh[pousr]_[A-Za-z0-9]{30,}|github_pat_[A-Za-z0-9_]{30,}|sk-proj-[A-Za-z0-9_-]{20,}|-----BEGIN (?:RSA |OPENSSH |EC )?PRIVATE KEY-----)',b),dest
    changes[dest]=dict(local=str(p),oid=oid,bytes=len(b),mode=T.get(dest,('100644',''))[0])
env=os.environ.copy();env['GIT_INDEX_FILE']=str((D/'scoped-upload.index').resolve())
git('read-tree',base,env=env)
oids=git('hash-object','--no-filters','-w','--stdin-paths',data=('\n'.join(Path(v['local']).resolve().as_posix() for v in changes.values())+'\n').encode('utf-8')).decode().split()
records=[]
for (dest,v),oid in zip(changes.items(),oids):assert oid==v['oid'];records.append((v['mode']+' '+oid+'\t'+dest).encode('utf-8')+b'\0')
assert len(oids)==len(changes)
git('update-index','-z','--index-info',data=b''.join(records),env=env)
new=git('write-tree',env=env).decode().strip()
(D/'scoped-upload-plan.json').write_text(json.dumps(dict(base=base,tree=new,changes=changes,firmware_files=len(selected),bytes=sum(v['bytes'] for v in changes.values()),root_firmware_preserved=True),ensure_ascii=False,indent=2),encoding='utf-8')
print('Scoped upload:',len(changes),'paths,',sum(v['bytes'] for v in changes.values()),'bytes; required firmware files:',len(selected),flush=True)
