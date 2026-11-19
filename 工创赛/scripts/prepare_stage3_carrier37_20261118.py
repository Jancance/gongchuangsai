"""User correction: stage3 carrier pick default37deg. Isolated build, no hardware."""
from pathlib import Path
import hashlib,json,shutil,subprocess
ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'tmp/three_frame_grip_lead20_20261118/project'
P=ROOT/'tmp/stage3_carrier37_20261118/project'
E=ROOT/'evidence/integration/2026-11-18/stage3-carrier37'

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    assert not P.exists(),'Preserve an existing candidate; no overwrite'
    source=BASE/'CHASSIS/USER/stage3_cycle.inc';raw=source.read_bytes()
    old=b'g_stage3_carrier_yaw=36.5f';new=b'g_stage3_carrier_yaw=37.0f'
    assert raw.count(old)==1
    E.mkdir(parents=True,exist_ok=True)
    shutil.copytree(BASE,P,ignore=shutil.ignore_patterns('Objects','Listings','__pycache__'))
    changed=P/'CHASSIS/USER/stage3_cycle.inc';changed.write_bytes(raw.replace(old,new,1))
    assert changed.read_bytes().replace(new,old,1)==raw
    unchanged={}
    for path in P.rglob('*'):
        if not path.is_file():continue
        rel=path.relative_to(P).as_posix()
        if rel!='CHASSIS/USER/stage3_cycle.inc':
            assert path.read_bytes()==(BASE/rel).read_bytes(),rel
            unchanged[rel]=sha(path)
    report=dict(base=str(BASE),project=str(P),user_confirmed_carrier_yaw=37.0,
        scope='Stage3 carrier pick yaw only; source stage2/other DM poses unchanged',
        changes={'CHASSIS/USER/stage3_cycle.inc':sha(changed)},source_files_preserved=len(unchanged),
        encoding_and_line_endings_preserved=True,flashed=False,hardware_access=False,images={})
    (E/'source-manifest.json').write_text(json.dumps({**unchanged,**report['changes']},indent=2),encoding='utf-8')
    for target in ('Vehicle_LiftRuler','Vehicle_LiftRuler_Offline'):
        for folder in ('Objects','Listings'):(P/folder/target).mkdir(parents=True,exist_ok=True)
        log=E/(target+'-build.log')
        result=subprocess.run(['C:/Keil_v5/UV4/UV4.exe','-r',str(P/'gongxun-f407.uvprojx'),'-t',target,'-j0','-o',str(log)],timeout=180)
        data=log.read_bytes();assert result.returncode==0 and b'0 Error(s), 0 Warning(s)' in data
        axf=P/'Objects'/target/(target+'.axf');report['images'][target]=dict(axf=str(axf),sha256=sha(axf))
        print(target+' 0 errors, 0 warnings',flush=True)
    report['build_passed']=True
    (E/'candidate.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report),flush=True)

if __name__=='__main__':main()
