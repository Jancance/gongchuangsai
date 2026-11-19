"""Back up and synchronize only verified candidate differences; never flash."""
from pathlib import Path
import argparse,datetime,hashlib,json,os,shutil,subprocess,sys
R=Path(__file__).resolve().parents[1]
E=R/'evidence/integration/2026-11-18/home-write-gap'
DEST=Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
BACKUP=R/'handoff/external_before_verified_mirror_20261118'


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--apply',action='store_true');a=ap.parse_args()
    c=json.loads((E/'candidate.json').read_text());source=Path(c['project'])
    manifest=json.loads((E/'source-manifest.json').read_text());planpath=E/'external-sync-plan.json'
    assert DEST.exists()
    assert all(sha(source/n)==h for n,h in manifest.items()),'Candidate source drift'
    if not a.apply:
        assert not planpath.exists(),'Inspect existing plan instead of replacing it'
        entries=[]
        for n,newsha in manifest.items():
            old=DEST/n
            if not old.exists() or sha(old)!=newsha:entries.append(dict(path=n,before_sha256=sha(old) if old.exists() else None,after_sha256=newsha))
        plan=dict(source=str(source),destination=str(DEST),backup=str(BACKUP),entries=entries,created=datetime.datetime.now().isoformat(),source_files=len(manifest),authorization='User requested replacing the old working program after successful vehicle verification; latest retry completed3/fault0 and user reported no problems')
        planpath.write_text(json.dumps(plan,indent=2)+'\n');print(json.dumps(dict(planned_files=len(entries),back_up_existing=sum(x['before_sha256'] is not None for x in entries),new_files=sum(x['before_sha256'] is None for x in entries),apply=False)));return
    plan=json.loads(planpath.read_text());assert plan['source']==str(source) and plan['destination']==str(DEST)
    assert not BACKUP.exists(),'Never overwrite the pre-sync backup'
    # Validate every preimage before the first external write.
    for x in plan['entries']:
        p=DEST/x['path'];assert (sha(p) if p.exists() else None)==x['before_sha256'],('External source drift',str(p))
    BACKUP.mkdir(parents=True)
    for x in plan['entries']:
        if x['before_sha256'] is not None:
            p=BACKUP/x['path'];p.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(DEST/x['path'],p);assert sha(p)==x['before_sha256']
    (BACKUP/'sync-plan.json').write_text(json.dumps(plan,indent=2)+'\n')
    report=dict(started=datetime.datetime.now().isoformat(),backup=str(BACKUP),written=[],flashed=False,reset=False,builds={})
    out=E/'external-sync-result.json'
    try:
        for x in plan['entries']:
            p=DEST/x['path'];tmp=p.with_name(p.name+'.verified-mirror-new')
            assert not tmp.exists();tmp.write_bytes((source/x['path']).read_bytes());assert sha(tmp)==x['after_sha256'];os.replace(tmp,p)
            report['written'].append(x['path']);out.write_text(json.dumps(report,indent=2)+'\n')
        assert all(sha(DEST/n)==h for n,h in manifest.items());report['source_verified_files']=len(manifest)
        sys.path[:0]=[str(R/'tmp/gripper_debug_deps'),str(R/'src')]
        from elftools.elf.elffile import ELFFile
        from smart_carrier.vision_manipulation.mirror_committed_pick_board import flash_segments_digest
        for target in ('Vehicle_LiftRuler','Vehicle_LiftRuler_Offline'):
            log=E/('external-'+target+'-build.log')
            run=subprocess.run(['C:/Keil_v5/UV4/UV4.exe','-r',str(DEST/'gongxun-f407.uvprojx'),'-t',target,'-o',str(log)])
            assert b'0 Error(s), 0 Warning(s)' in log.read_bytes(),('Build failed; inspect log',str(log),run.returncode)
            image=DEST/'Objects'/target/(target+'.axf')
            with image.open('rb') as f:
                elf=ELFFile(f);segments=[(int(s['p_paddr']),s.data()) for s in elf.iter_segments() if s['p_type']=='PT_LOAD' and s['p_filesz'] and 0x08000000<=s['p_paddr']<0x08100000]
            digest=flash_segments_digest(segments)
            assert digest==c['images'][target]['flash_segments_sha256'],('Flash differs from vehicle-tested candidate',target)
            report['builds'][target]=dict(clean=True,axf_sha256=sha(image),flash_segments_sha256=digest,flash_matches_tested_candidate=True)
        report['completed']=datetime.datetime.now().isoformat();report['passed']=True
    except Exception as error:
        report['passed']=False;report['error']=repr(error);raise
    finally:out.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(passed=report['passed'],updated_files=len(report['written']),verified_source_files=report['source_verified_files'],backup=str(BACKUP),builds=report['builds'],flashed=False)))


if __name__=='__main__':main()
