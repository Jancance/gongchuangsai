"""Build the round-trip test image with persistent Stage3 boot selection."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

ROOT=Path(__file__).resolve().parents[1]
EVIDENCE=ROOT/'evidence/integration/2026-11-18/stage3-resume'


def main():
    manifest=json.loads((EVIDENCE/'full-cycle-compatible.json').read_text(encoding='utf-8'))
    base=Path(manifest['project'])
    project=ROOT/'tmp/stage3_full_cycle_stage3_boot_20261119/project'
    shutil.copytree(base,project,ignore=shutil.ignore_patterns('Objects','Listings'))
    name='CHASSIS/USER/region_dispatch.inc'
    path=project/name
    raw=path.read_bytes()
    old=b'g_region_dispatch_area=1U,g_region_dispatch_rejected;'
    assert raw.count(old)==1
    path.write_bytes(raw.replace(old,b'g_region_dispatch_area=3U,g_region_dispatch_rejected;'))
    manifest['unchanged_sha256'].pop(name)
    manifest['changes'][name]=hashlib.sha256(path.read_bytes()).hexdigest()
    manifest.update(project=str(project),images={},flashed=False,stage3_boot_area=3)
    for target in ('Vehicle_LiftRuler','Vehicle_LiftRuler_Offline'):
        for folder in ('Objects','Listings'):
            (project/folder/target).mkdir(parents=True,exist_ok=True)
        log=EVIDENCE/f'full-cycle-stage3-boot-{target}.log'
        result=subprocess.run(['C:/Keil_v5/UV4/UV4.exe','-r',str(project/'gongxun-f407.uvprojx'),
                               '-t',target,'-j0','-o',str(log)],timeout=180)
        assert result.returncode==0 and b'0 Error(s), 0 Warning(s)' in log.read_bytes(),log
        axf=project/'Objects'/target/f'{target}.axf'
        manifest['images'][target]={'axf':str(axf),'sha256':hashlib.sha256(axf.read_bytes()).hexdigest()}
        print(target,'0 errors, 0 warnings',flush=True)
    (EVIDENCE/'full-cycle-stage3-boot-compatible.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')


if __name__=='__main__':main()
