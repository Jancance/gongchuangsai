"""Build the isolated lead8 top-view candidate; no hardware access."""
from pathlib import Path
import hashlib,json,subprocess

R=Path(__file__).resolve().parents[1]
B=R/'tmp/source_low_readonly_diag_20261114/project'
P=R/'tmp/source_top_pick_lead8_20261114/project'
E=R/'evidence/integration/2026-11-14/top-pick-lead8'

def main():
    assert json.loads((E/'matlab-result.json').read_text())['passed']
    changes={}
    for path in P.rglob('*'):
        rel=path.relative_to(P)
        if path.is_file() and not any(x in ('Objects','Listings') for x in rel.parts):
            if not (B/rel).exists() or path.read_bytes()!=(B/rel).read_bytes():
                changes[rel.as_posix()]=hashlib.sha256(path.read_bytes()).hexdigest()
    candidate=dict(base=B.relative_to(R).as_posix(),project=P.relative_to(R).as_posix(),
      changes=changes,images={},build_passed=False,flashed=False,hardware_access=False,
      matlab_prior_to_implementation=True,read_only_diag_supported=True)
    for target in ['Vehicle_LiftRuler','Vehicle_LiftRuler_Offline']:
        for folder in ['Objects','Listings']:(P/folder/target).mkdir(parents=True,exist_ok=True)
        log=E/(target+'-build.log')
        result=subprocess.run(['C:/Keil_v5/UV4/UV4.exe','-r',str(P/'gongxun-f407.uvprojx'),
          '-t',target,'-j0','-o',str(log)],timeout=180)
        raw=log.read_bytes();print(raw.decode('gb18030',errors='replace')[-1800:],flush=True)
        assert result.returncode==0 and b'0 Error(s), 0 Warning(s)' in raw
        image=P/'Objects'/target/(target+'.axf')
        candidate['images'][target]=dict(axf=image.relative_to(R).as_posix(),sha256=hashlib.sha256(image.read_bytes()).hexdigest())
    candidate['build_passed']=True
    (E/'candidate.json').write_text(json.dumps(candidate,indent=2),encoding='utf-8')
    (E/'checks.json').write_text(json.dumps(dict(passed=True,dual_build=True,hardware_access=False)),encoding='utf-8')
    print(json.dumps(candidate['images']),flush=True)

if __name__=='__main__':main()
