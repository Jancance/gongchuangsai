"""Actual C owners and dual Keil builds. No probe or hardware access."""
from pathlib import Path
import hashlib,json,subprocess
from prepare_stage3_dispatch_20261116 import R,BASE,P,E

def run(name,source,extra=()):
    exe=E/(name+'.exe')
    command=['gcc','-std=c99','-Wall','-Wextra','-Werror','-Wno-misleading-indentation','-Wno-unused-function',
             '-I'+str(P/'CHASSIS/USER'),str(R/source),*map(str,extra),'-lm','-o',str(exe)]
    subprocess.run(command,check=True)
    result=subprocess.run([str(exe)],capture_output=True,text=True)
    (E/(name+'.log')).write_text(result.stdout+result.stderr)
    print(result.stdout+result.stderr,flush=True);assert result.returncode==0
    return result.stdout.strip()

def main():
    c=run('actual-stage3','tests/stage3_dispatch_20261116.c')
    h=run('actual-heading','tests/stage3_heading_20261116.c',[P/'CHASSIS/USER/heading_session.c',P/'CHASSIS/USER/heading_controller.c'])
    changes={};unchanged={}
    for path in P.rglob('*'):
        if not path.is_file() or any(x in ('Objects','Listings') for x in path.relative_to(P).parts):continue
        rel=path.relative_to(P).as_posix();data=path.read_bytes()
        before=BASE/rel
        if not before.exists() or before.read_bytes()!=data:changes[rel]=hashlib.sha256(data).hexdigest()
    for n in ['CHASSIS/USER/source_mcu_cycle.inc','CHASSIS/USER/vehicle_source_parallel.inc','CHASSIS/USER/motion_control.h']:
        assert (P/n).read_bytes()==(BASE/n).read_bytes(),n
        unchanged[n]=hashlib.sha256((P/n).read_bytes()).hexdigest()
    for n in ['CHASSIS/USER/main.c','CHASSIS/USER/source_body_step.inc','CHASSIS/USER/pzq_commit_overlap.inc']:
        before=(BASE/n).read_bytes();after=(P/n).read_bytes()
        assert before.count(b'\n')==before.count(b'\r\n') and after.count(b'\n')==after.count(b'\r\n')
    images={}
    for target in ['Vehicle_LiftRuler','Vehicle_LiftRuler_Offline']:
        for folder in ['Objects','Listings']:(P/folder/target).mkdir(parents=True,exist_ok=True)
        log=E/(target+'-build.log')
        r=subprocess.run(['C:/Keil_v5/UV4/UV4.exe','-r',str(P/'gongxun-f407.uvprojx'),'-t',target,'-j0','-o',str(log)],timeout=180)
        data=log.read_bytes();print(data.decode('gb18030',errors='replace')[-500:],flush=True)
        assert r.returncode==0 and b'0 Error(s), 0 Warning(s)' in data
        axf=P/'Objects'/target/(target+'.axf')
        images[target]={'axf':axf.relative_to(R).as_posix(),'sha256':hashlib.sha256(axf.read_bytes()).hexdigest()}
    candidate=dict(base=BASE.relative_to(R).as_posix(),project=P.relative_to(R).as_posix(),changes=changes,
        unchanged_sha256=unchanged,images=images,build_passed=True,flashed=False,hardware_access=False,
        matlab_prior_to_implementation=True,physical_calibration_open=True)
    (E/'candidate.json').write_text(json.dumps(candidate,indent=2)+'\n')
    (E/'checks.json').write_text(json.dumps(dict(passed=True,actual_c=c,actual_heading=h,dual_build=True,
        hardware_access=False,flashed=False,physical_calibration_open=True),indent=2)+'\n')
if __name__=='__main__':main()
