"""Build the original pzq with only its release interval changed; never flash."""
from pathlib import Path
import hashlib
import json
import subprocess

R=Path(__file__).resolve().parents[1]
P=R/'tmp/pzq_release50_20261111/project'
E=R/'evidence/integration/2026-11-11/pzq-release50'


def main():
    m=json.loads((E/'candidate.json').read_text(encoding='utf-8'))
    assert not m['deployed']
    m['images']={}
    for target in ('Vehicle_LiftRuler','Vehicle_LiftRuler_Offline'):
        for folder in ('Objects','Listings'):(P/folder/target).mkdir(parents=True,exist_ok=True)
        log=E/(target+'-build.log')
        proc=subprocess.run(['C:/Keil_v5/UV4/UV4.exe','-r',str(P/'gongxun-f407.uvprojx'),'-t',target,'-j0','-o',str(log)],timeout=180)
        content=log.read_bytes();print(content.decode('gb18030',errors='replace')[-1100:],flush=True)
        assert proc.returncode==0 and b'0 Error(s), 0 Warning(s)' in content,str(log)
        f=P/'Objects'/target/(target+'.axf')
        m['images'][target]=dict(axf=f.relative_to(R).as_posix(),sha256=hashlib.sha256(f.read_bytes()).hexdigest(),build_log=log.relative_to(R).as_posix())
    (E/'candidate.json').write_text(json.dumps(m,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(m['images'],indent=2))


if __name__=='__main__':main()
